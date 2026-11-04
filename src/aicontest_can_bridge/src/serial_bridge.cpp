// Pi ⇔ Nucleo の変換ノード(UART 版)。Nucleo の USB(ST-LINK の仮想シリアル、/dev/ttyACM0 など)でつなぐ。
// トピックは CAN 版(can_bridge)と同じ:
//   入力  command_joint_states(駆動輪の目標角速度)→ 10ms ごとに COMMAND パケット
//   出力  joint_states(駆動輪 + 計測輪)、imu(旋回の速さ)、~/status(状態フラグ)。TELEMETRY パケットが届くたびに出す
//   Nucleo の printf の文字(TEXT パケット)は "[nucleo] ..." としてログに出す
// パケットの形は include/aicontest_can_bridge/serial_framing.hpp(ファームウェアの pi_link.c と同じ)。
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>

#include "aicontest_can_bridge/serial_framing.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace serial_bridge {

namespace proto = aicontest_can_bridge::protocol;
namespace ser   = aicontest_can_bridge::serial;
using namespace std::chrono_literals;

static speed_t to_speed (int baud) {
    switch (baud) {
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 2000000: return B2000000;
        default: return 0;
    }
}

class serial_bridge : public rclcpp::Node {
public:
    explicit serial_bridge (const rclcpp::NodeOptions &options) : Node ("serial_bridge", options) {
        port_                   = this->declare_parameter<std::string> ("port", "/dev/ttyACM0");
        baud_                   = static_cast<int> (this->declare_parameter<int64_t> ("baud", 460800));
        drive_wheel_names_      = this->declare_parameter<std::vector<std::string>> ("drive_wheel_names", {"mecanum_fl", "mecanum_fr", "mecanum_rl", "mecanum_rr"});
        odom_wheel_names_       = this->declare_parameter<std::vector<std::string>> ("odom_wheel_names", {"odom_wheel_l", "odom_wheel_s"});
        drive_wheel_directions_ = this->declare_parameter<std::vector<double>> ("drive_wheel_directions", {1.0, 1.0, 1.0, 1.0});
        odom_wheel_directions_  = this->declare_parameter<std::vector<double>> ("odom_wheel_directions", {1.0, 1.0});
        odom_counts_per_rev_    = this->declare_parameter<std::vector<double>> ("odom_counts_per_rev", {4096.0, 4096.0});
        imu_frame_id_           = this->declare_parameter<std::string> ("imu_frame_id", "base_link");
        imu_sign_               = this->declare_parameter<double> ("imu_sign", 1.0);
        command_timeout_sec_    = this->declare_parameter<double> ("command_timeout_sec", 0.5);
        const double tx_rate_hz = this->declare_parameter<double> ("tx_rate_hz", 100.0);
        const double stats_sec  = this->declare_parameter<double> ("stats_period_sec", 10.0);

        if (drive_wheel_names_.size () != proto::kNumDriveWheels || drive_wheel_directions_.size () != proto::kNumDriveWheels) {
            throw std::runtime_error ("drive_wheel_names / drive_wheel_directions must have 4 elements");
        }
        if (odom_wheel_names_.size () != proto::kNumOdomWheels || odom_wheel_directions_.size () != proto::kNumOdomWheels || odom_counts_per_rev_.size () != proto::kNumOdomWheels) {
            throw std::runtime_error ("odom_wheel_names / odom_wheel_directions / odom_counts_per_rev must have 2 elements");
        }
        if (to_speed (baud_) == 0) throw std::runtime_error ("unsupported baud (use 115200 / 230400 / 460800 / 921600 / 1000000 / 2000000)");
        if (tx_rate_hz <= 0.0) throw std::runtime_error ("tx_rate_hz must be positive");

        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState> ("joint_states", rclcpp::SensorDataQoS ());
        status_publisher_      = this->create_publisher<std_msgs::msg::UInt8> ("~/status", 10);
        imu_publisher_         = this->create_publisher<sensor_msgs::msg::Imu> ("imu", rclcpp::SensorDataQoS ());
        command_subscriber_    = this->create_subscription<sensor_msgs::msg::JointState> ("command_joint_states", rclcpp::SensorDataQoS (), [this] (const sensor_msgs::msg::JointState::SharedPtr msg) { command_callback (msg); });

        last_command_time_ = this->now ();
        tx_timer_          = this->create_wall_timer (std::chrono::duration<double> (1.0 / tx_rate_hz), [this] () { tx_timer_callback (); });
        if (stats_sec > 0.0) stats_timer_ = this->create_wall_timer (std::chrono::duration<double> (stats_sec), [this] () { stats_timer_callback (); });

        RCLCPP_INFO (this->get_logger (), "port %s, %d bps, tx %.0f Hz, command_timeout %.2f s", port_.c_str (), baud_, tx_rate_hz, command_timeout_sec_);
        rx_thread_ = std::thread ([this] () { rx_loop (); });
    }

    ~serial_bridge () override {
        running_ = false;
        if (rx_thread_.joinable ()) rx_thread_.join ();
        close_port ();
    }

private:
    // ---- シリアルポート ----
    bool open_port () {
        // 書き込みで止まらないように O_NONBLOCK(Nucleo が止まっていて受け取らないと、送信が詰まって ROS のタイマーごと止まるため)
        const int fd = ::open (port_.c_str (), O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK);
        if (fd < 0) {
            RCLCPP_ERROR_THROTTLE (this->get_logger (), *this->get_clock (), 5000, "open(%s) failed: %s. Retrying... (Nucleo の USB はつながっている? dialout グループに入っている?)", port_.c_str (), std::strerror (errno));
            return false;
        }
        struct termios tio{};
        if (::tcgetattr (fd, &tio) < 0) {
            ::close (fd);
            return false;
        }
        ::cfmakeraw (&tio);
        tio.c_cflag |= CLOCAL | CREAD;
        tio.c_cflag &= ~CRTSCTS;
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 0;
        ::cfsetispeed (&tio, to_speed (baud_));
        ::cfsetospeed (&tio, to_speed (baud_));
        if (::tcsetattr (fd, TCSANOW, &tio) < 0) {
            RCLCPP_ERROR (this->get_logger (), "tcsetattr(%s) failed: %s", port_.c_str (), std::strerror (errno));
            ::close (fd);
            return false;
        }
        ::tcflush (fd, TCIOFLUSH);
        {
            std::lock_guard<std::mutex> lock (port_mutex_);
            fd_ = fd;
        }
        RCLCPP_INFO (this->get_logger (), "Opened %s", port_.c_str ());
        return true;
    }

    void close_port () {
        std::lock_guard<std::mutex> lock (port_mutex_);
        if (fd_ >= 0) {
            ::close (fd_);
            fd_ = -1;
        }
    }

    void rx_loop () {
        ser::PacketReader reader;
        uint8_t           buf[256];
        while (running_) {
            int fd;
            {
                std::lock_guard<std::mutex> lock (port_mutex_);
                fd = fd_;
            }
            if (fd < 0) {
                if (!open_port ()) std::this_thread::sleep_for (1s);
                continue;
            }
            // 0.1 秒待って、届いた分を読む(終了要求を確認できるように)
            struct pollfd pfd{fd, POLLIN, 0};
            const int     pr = ::poll (&pfd, 1, 100);
            if (pr == 0) continue;
            if (pr < 0) {
                if (errno == EINTR) continue;
                RCLCPP_ERROR (this->get_logger (), "poll(%s) failed: %s. Reopening...", port_.c_str (), std::strerror (errno));
                close_port ();
                std::this_thread::sleep_for (1s);
                continue;
            }
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                RCLCPP_ERROR (this->get_logger (), "%s closed (USB unplugged?). Reopening...", port_.c_str ());
                close_port ();
                std::this_thread::sleep_for (1s);
                continue;
            }
            const ssize_t n = ::read (fd, buf, sizeof (buf));
            if (n < 0) {
                if (errno == EAGAIN || errno == EINTR) continue;
                RCLCPP_ERROR (this->get_logger (), "read(%s) failed: %s. Reopening...", port_.c_str (), std::strerror (errno));
                close_port ();
                std::this_thread::sleep_for (1s);
                continue;
            }
            if (n == 0) continue;
            for (ssize_t i = 0; i < n; ++i) {
                std::optional<ser::Packet> packet;
                if (!reader.push (buf[i], packet)) continue;
                if (!packet) {
                    rx_bad_++;
                    continue;
                }
                handle_packet (*packet);
            }
        }
    }

    void handle_packet (const ser::Packet &p) {
        if (p.type == ser::kTypeTelemetry) {
            const auto t = ser::decode_telemetry (p.data);
            if (!t) {
                rx_bad_++;
                return;
            }
            rx_telemetry_++;
            publish_joint_states (*t);
            publish_imu (t->imu);
            publish_status (t->status, t->calib);
        } else if (p.type == ser::kTypeText) {
            RCLCPP_INFO (this->get_logger (), "[nucleo] %s", std::string (p.data.begin (), p.data.end ()).c_str ());
        } else {
            rx_bad_++;
        }
    }

    // ---- ROS へ ----
    void publish_joint_states (const ser::Telemetry &t) {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now ();
        for (size_t i = 0; i < proto::kNumDriveWheels; ++i) {
            msg.name.push_back (drive_wheel_names_[i]);
            msg.position.push_back (0.0);
            msg.velocity.push_back (t.drive_rad_s[i] * drive_wheel_directions_[i]);
        }
        for (size_t i = 0; i < proto::kNumOdomWheels; ++i) {
            msg.name.push_back (odom_wheel_names_[i]);
            msg.position.push_back (odom_wheel_directions_[i] * proto::count_to_rad (t.odom[i].count, odom_counts_per_rev_[i]));
            msg.velocity.push_back (odom_wheel_directions_[i] * proto::rpm_to_rad_s (t.odom[i].rpm));
        }
        joint_state_publisher_->publish (msg);
    }

    void publish_imu (const proto::ImuFrame &imu) {
        if (!std::isfinite (imu.yaw_rate)) {  // Nucleo は IMU が応答しないとき NaN を送る
            RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 2000, "IMU data invalid (NaN): /imu not published");
            return;
        }
        sensor_msgs::msg::Imu msg;
        msg.header.stamp                      = this->now ();
        msg.header.frame_id                   = imu_frame_id_;
        msg.angular_velocity.z                = imu_sign_ * static_cast<double> (imu.yaw_rate);
        const double h                        = std::isfinite (imu.heading) ? imu_sign_ * static_cast<double> (imu.heading) : 0.0;
        msg.orientation.z                     = std::sin (h / 2.0);
        msg.orientation.w                     = std::cos (h / 2.0);
        msg.orientation_covariance[8]         = 0.01;
        msg.angular_velocity_covariance[8]    = 1e-4;
        msg.linear_acceleration_covariance[0] = -1.0;
        imu_publisher_->publish (msg);
    }

    void publish_status (uint8_t flags, uint8_t calib) {
        std_msgs::msg::UInt8 msg;
        msg.data = flags;
        status_publisher_->publish (msg);
        if (flags != last_status_) {
            if (flags == 0) {
                RCLCPP_INFO (this->get_logger (), "Nucleo status: OK");
            } else {
                std::string text;
                if (flags & proto::kStatusWatchdogStop) text += " watchdog-stop";
                if (flags & proto::kStatusImuLost) text += " IMU-lost";
                for (int m = 0; m < proto::kNumDriveWheels; ++m) {
                    if (proto::motor_feedback_lost (flags, m)) text += " C610-ID" + std::to_string (m + 1) + "-lost";
                }
                RCLCPP_WARN (this->get_logger (), "Nucleo status 0x%02X:%s", flags, text.c_str ());
            }
            last_status_ = flags;
        }
        if (calib != last_calib_) {
            RCLCPP_INFO (this->get_logger (), "IMU calibration 0x%02X (gyro %d/3; BMX055: 3 = bias measured at power-on)", calib, proto::gyro_calibration (calib));
            last_calib_ = calib;
        }
    }

    // ---- Nucleo へ ----
    void command_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock (command_mutex_);
        for (size_t j = 0; j < msg->name.size () && j < msg->velocity.size (); ++j) {
            for (size_t i = 0; i < proto::kNumDriveWheels; ++i) {
                if (msg->name[j] == drive_wheel_names_[i]) {
                    command_rad_s_[i] = msg->velocity[j];
                    break;
                }
            }
        }
        last_command_time_ = this->now ();
        command_received_  = true;
    }

    void tx_timer_callback () {
        std::array<double, proto::kNumDriveWheels> rad_s{};
        {
            std::lock_guard<std::mutex> lock (command_mutex_);
            const bool                  stale = !command_received_ || (this->now () - last_command_time_).seconds () > command_timeout_sec_;
            if (stale != command_stale_) {
                if (stale) {
                    RCLCPP_WARN (this->get_logger (), "No /command_joint_states for %.2f s: sending zero velocity", command_timeout_sec_);
                } else {
                    RCLCPP_INFO (this->get_logger (), "Receiving /command_joint_states");
                }
                command_stale_ = stale;
            }
            // 指令が途絶えたら 0 を送り続ける(ノードごと落ちた場合は Nucleo のウォッチドッグが 0.1 秒で止める)
            if (!stale) {
                for (size_t i = 0; i < proto::kNumDriveWheels; ++i) rad_s[i] = command_rad_s_[i] * drive_wheel_directions_[i];
            }
        }
        const auto data   = proto::encode_drive_command (rad_s);
        const auto packet = ser::make_packet (ser::kTypeCommand, data.data (), data.size ());
        std::lock_guard<std::mutex> lock (port_mutex_);
        if (fd_ < 0) return;
        const ssize_t n = ::write (fd_, packet.data (), packet.size ());  // 詰まっていたら(EAGAIN)送らずに次へ
        if (n == static_cast<ssize_t> (packet.size ())) {
            tx_packets_++;
        } else {
            tx_errors_++;
        }
    }

    void stats_timer_callback () {
        const auto tel = rx_telemetry_.exchange (0);
        const auto bad = rx_bad_.exchange (0);
        const auto tx  = tx_packets_.exchange (0);
        const auto err = tx_errors_.exchange (0);
        if (bad > 0 || err > 0 || tel == 0) {
            RCLCPP_WARN (this->get_logger (), "rx %lu telemetry, %lu bad packets | tx %lu commands, %lu errors", tel, bad, tx, err);
        } else {
            RCLCPP_INFO (this->get_logger (), "rx %lu telemetry | tx %lu commands", tel, tx);
        }
    }

    // ---- 設定 ----
    std::string              port_;
    int                      baud_ = 460800;
    std::vector<std::string> drive_wheel_names_, odom_wheel_names_;
    std::vector<double>      drive_wheel_directions_, odom_wheel_directions_, odom_counts_per_rev_;
    std::string              imu_frame_id_;
    double                   imu_sign_            = 1.0;
    double                   command_timeout_sec_ = 0.5;

    // ---- ROS ----
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    joint_state_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr            status_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr           imu_publisher_;
    rclcpp::TimerBase::SharedPtr                                  tx_timer_, stats_timer_;

    // ---- 指令 ----
    std::mutex                                 command_mutex_;
    std::array<double, proto::kNumDriveWheels> command_rad_s_{};
    rclcpp::Time                               last_command_time_;
    bool                                       command_received_ = false;
    bool                                       command_stale_    = true;

    // ---- 状態 ----
    uint8_t last_status_ = 0xFF;
    uint8_t last_calib_  = 0xFF;

    // ---- ポート ----
    std::mutex       port_mutex_;
    int              fd_ = -1;
    std::thread      rx_thread_;
    std::atomic_bool running_{true};

    // ---- 統計 ----
    std::atomic_uint64_t rx_telemetry_{0}, rx_bad_{0}, tx_packets_{0}, tx_errors_{0};
};

}  // namespace serial_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (serial_bridge::serial_bridge)
