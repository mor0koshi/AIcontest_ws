#include "aicontest_can_bridge/can_bridge.hpp"

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <chrono>
#include <cstring>

namespace can_bridge {

namespace proto = aicontest_can_bridge::protocol;
using namespace std::chrono_literals;

can_bridge::can_bridge (const rclcpp::NodeOptions &options) : Node ("can_bridge", options) {
    can_interface_          = this->declare_parameter<std::string> ("can_interface", "can0");
    drive_wheel_names_      = this->declare_parameter<std::vector<std::string>> ("drive_wheel_names", {"mecanum_fl", "mecanum_fr", "mecanum_rl", "mecanum_rr"});
    odom_wheel_names_       = this->declare_parameter<std::vector<std::string>> ("odom_wheel_names", {"odom_wheel_l", "odom_wheel_s"});
    drive_wheel_directions_ = this->declare_parameter<std::vector<double>> ("drive_wheel_directions", {1.0, 1.0, 1.0, 1.0});
    odom_wheel_directions_  = this->declare_parameter<std::vector<double>> ("odom_wheel_directions", {1.0, 1.0});
    odom_counts_per_rev_    = this->declare_parameter<std::vector<double>> ("odom_counts_per_rev", {4096.0, 4096.0});
    imu_frame_id_           = this->declare_parameter<std::string> ("imu_frame_id", "base_link");
    imu_sign_               = this->declare_parameter<double> ("imu_sign", 1.0);  // 旋回の向きが逆なら -1.0
    command_timeout_sec_    = this->declare_parameter<double> ("command_timeout_sec", 0.5);
    const double tx_rate_hz = this->declare_parameter<double> ("tx_rate_hz", 100.0);
    const double stats_sec  = this->declare_parameter<double> ("stats_period_sec", 10.0);

    if (drive_wheel_names_.size () != proto::kNumDriveWheels || drive_wheel_directions_.size () != proto::kNumDriveWheels) {
        throw std::runtime_error ("drive_wheel_names / drive_wheel_directions must have 4 elements");
    }
    if (odom_wheel_names_.size () != proto::kNumOdomWheels || odom_wheel_directions_.size () != proto::kNumOdomWheels || odom_counts_per_rev_.size () != proto::kNumOdomWheels) {
        throw std::runtime_error ("odom_wheel_names / odom_wheel_directions / odom_counts_per_rev must have 2 elements");
    }
    if (tx_rate_hz <= 0.0) throw std::runtime_error ("tx_rate_hz must be positive");

    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState> ("joint_states", rclcpp::SensorDataQoS ());
    status_publisher_      = this->create_publisher<std_msgs::msg::UInt8> ("~/status", 10);
    imu_publisher_         = this->create_publisher<sensor_msgs::msg::Imu> ("imu", rclcpp::SensorDataQoS ());
    command_subscriber_    = this->create_subscription<sensor_msgs::msg::JointState> ("command_joint_states", rclcpp::SensorDataQoS (), std::bind (&can_bridge::command_callback, this, std::placeholders::_1));

    last_command_time_ = this->now ();
    tx_timer_          = this->create_wall_timer (std::chrono::duration<double> (1.0 / tx_rate_hz), std::bind (&can_bridge::tx_timer_callback, this));
    if (stats_sec > 0.0) {
        stats_timer_ = this->create_wall_timer (std::chrono::duration<double> (stats_sec), std::bind (&can_bridge::stats_timer_callback, this));
    }

    RCLCPP_INFO (this->get_logger (), "can_interface: %s, tx %.0f Hz, command_timeout %.2f s", can_interface_.c_str (), tx_rate_hz, command_timeout_sec_);
    RCLCPP_INFO (this->get_logger (), "drive wheels (0x300 order): %s, %s, %s, %s", drive_wheel_names_[0].c_str (), drive_wheel_names_[1].c_str (), drive_wheel_names_[2].c_str (), drive_wheel_names_[3].c_str ());
    RCLCPP_INFO (this->get_logger (), "odom wheels (0x311, 0x312): %s, %s / IMU (0x313) frame %s", odom_wheel_names_[0].c_str (), odom_wheel_names_[1].c_str (), imu_frame_id_.c_str ());

    rx_thread_ = std::thread (&can_bridge::rx_loop, this);
}

can_bridge::~can_bridge () {
    running_ = false;
    if (rx_thread_.joinable ()) rx_thread_.join ();
    close_socket ();
}

bool can_bridge::open_socket () {
    const int fd = ::socket (PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        RCLCPP_ERROR_THROTTLE (this->get_logger (), *this->get_clock (), 5000, "socket(PF_CAN) failed: %s", std::strerror (errno));
        return false;
    }

    struct ifreq ifr{};
    std::strncpy (ifr.ifr_name, can_interface_.c_str (), IFNAMSIZ - 1);
    if (::ioctl (fd, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR_THROTTLE (this->get_logger (), *this->get_clock (), 5000, "CAN interface '%s' not found (%s). Retrying...", can_interface_.c_str (), std::strerror (errno));
        ::close (fd);
        return false;
    }

    // 標準ID・データフレームの 0x310〜0x317 だけをカーネル内で通す(C610 のフィードバックは捨てる)
    struct can_filter filter{};
    filter.can_id   = proto::kRxFilterId;
    filter.can_mask = (CAN_SFF_MASK & proto::kRxFilterMask) | CAN_EFF_FLAG | CAN_RTR_FLAG;
    ::setsockopt (fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof (filter));

    // バスの異常(バスオフ、ACKなし、送信タイムアウトなど)はエラーフレームで受け取ってログに出す
    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_CRTL | CAN_ERR_BUSOFF | CAN_ERR_RESTARTED | CAN_ERR_ACK | CAN_ERR_BUSERROR;
    ::setsockopt (fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof (err_mask));

    // read() を 100 ms でタイムアウトさせて、終了要求を確認できるようにする
    struct timeval tv{};
    tv.tv_usec = 100 * 1000;
    ::setsockopt (fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof (tv));

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind (fd, reinterpret_cast<struct sockaddr *> (&addr), sizeof (addr)) < 0) {
        RCLCPP_ERROR_THROTTLE (this->get_logger (), *this->get_clock (), 5000, "bind(%s) failed: %s", can_interface_.c_str (), std::strerror (errno));
        ::close (fd);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock (socket_mutex_);
        socket_fd_ = fd;
    }
    RCLCPP_INFO (this->get_logger (), "Opened CAN interface %s", can_interface_.c_str ());
    return true;
}

void can_bridge::close_socket () {
    std::lock_guard<std::mutex> lock (socket_mutex_);
    if (socket_fd_ >= 0) {
        ::close (socket_fd_);
        socket_fd_ = -1;
    }
}

void can_bridge::rx_loop () {
    while (running_) {
        int fd;
        {
            std::lock_guard<std::mutex> lock (socket_mutex_);
            fd = socket_fd_;
        }
        if (fd < 0) {
            if (!open_socket ()) std::this_thread::sleep_for (1s);
            continue;
        }

        struct can_frame frame{};
        const ssize_t    n = ::read (fd, &frame, sizeof (frame));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            RCLCPP_ERROR (this->get_logger (), "CAN read failed: %s. Reopening...", std::strerror (errno));
            close_socket ();
            std::this_thread::sleep_for (1s);
            continue;
        }
        if (n != static_cast<ssize_t> (sizeof (frame))) continue;

        if (frame.can_id & CAN_ERR_FLAG) {
            bus_errors_++;
            RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 2000, "CAN error frame: class 0x%X (bus-off / no ACK / controller error)", frame.can_id & CAN_ERR_MASK);
            continue;
        }
        rx_frames_++;
        handle_frame (frame);
    }
}

void can_bridge::handle_frame (const can_frame &frame) {
    if (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) return;
    const uint32_t id = frame.can_id & CAN_SFF_MASK;

    switch (id) {
        case proto::kIdDriveFeedback: {
            if (frame.len < 8) return;
            const auto rad_s = proto::decode_drive_feedback (frame.data);
            for (size_t i = 0; i < proto::kNumDriveWheels; ++i) drive_rad_s_[i] = rad_s[i] * drive_wheel_directions_[i];
            drive_fresh_ = true;
            break;
        }
        case proto::kIdOdomWheel0:
        case proto::kIdOdomWheel1: {
            if (frame.len < 8) return;
            const size_t index = id - proto::kIdOdomWheel0;
            odom_[index]       = proto::decode_odom_wheel (frame.data);
            odom_fresh_[index] = true;
            break;
        }
        case proto::kIdImu: {
            // 0x313 が1周期の最後のフレーム。ここで駆動輪・計測輪をまとめて出し、IMU も出す
            if (frame.len < 8) return;
            rx_cycles_++;
            if (!(drive_fresh_ && odom_fresh_[0] && odom_fresh_[1])) partial_cycles_++;
            publish_joint_states ();
            publish_imu (proto::decode_imu (frame.data));
            drive_fresh_ = false;
            odom_fresh_.fill (false);
            break;
        }
        case proto::kIdStatus: {
            if (frame.len < 1) return;
            const uint8_t        flags = frame.data[0];
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
            if (frame.len >= 2 && frame.data[1] != last_calib_) {
                RCLCPP_INFO (this->get_logger (), "IMU calibration 0x%02X (gyro %d/3; BMX055: 3 = bias measured at power-on)", frame.data[1], proto::gyro_calibration (frame.data[1]));
                last_calib_ = frame.data[1];
            }
            break;
        }
        default:
            break;
    }
}

void can_bridge::publish_joint_states () {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now ();
    msg.name.reserve (proto::kNumDriveWheels + proto::kNumOdomWheels);
    msg.position.reserve (proto::kNumDriveWheels + proto::kNumOdomWheels);
    msg.velocity.reserve (proto::kNumDriveWheels + proto::kNumOdomWheels);

    for (size_t i = 0; i < proto::kNumDriveWheels; ++i) {
        msg.name.push_back (drive_wheel_names_[i]);
        msg.position.push_back (0.0);  // 駆動輪の角度は送られてこない
        msg.velocity.push_back (drive_rad_s_[i]);
    }
    // natto_wheel_odometry(omni)が使うのは velocity だけ。position はデバッグ用
    for (size_t i = 0; i < proto::kNumOdomWheels; ++i) {
        msg.name.push_back (odom_wheel_names_[i]);
        msg.position.push_back (odom_wheel_directions_[i] * proto::count_to_rad (odom_[i].count, odom_counts_per_rev_[i]));
        msg.velocity.push_back (odom_wheel_directions_[i] * proto::rpm_to_rad_s (odom_[i].rpm));
    }
    joint_state_publisher_->publish (msg);
}

void can_bridge::publish_imu (const proto::ImuFrame &imu) {
    // Nucleo は IMU が応答しないとき NaN を送ってくる。そのときは出さない(オドメトリは計測輪だけで続ける)
    if (!std::isfinite (imu.yaw_rate)) {
        RCLCPP_WARN_THROTTLE (this->get_logger (), *this->get_clock (), 2000, "IMU data invalid (NaN): /imu not published");
        return;
    }
    sensor_msgs::msg::Imu msg;
    msg.header.stamp          = this->now ();
    msg.header.frame_id       = imu_frame_id_;
    msg.angular_velocity.z    = imu_sign_ * static_cast<double> (imu.yaw_rate);
    const double h            = std::isfinite (imu.heading) ? imu_sign_ * static_cast<double> (imu.heading) : 0.0;
    msg.orientation.z         = std::sin (h / 2.0);
    msg.orientation.w         = std::cos (h / 2.0);
    msg.orientation_covariance[8]      = 0.01;  // 向き(yaw)だけ。roll / pitch は使わない
    msg.angular_velocity_covariance[8] = 1e-4;
    msg.linear_acceleration_covariance[0] = -1.0;  // 加速度は出さない
    imu_publisher_->publish (msg);
}

void can_bridge::command_callback (const sensor_msgs::msg::JointState::SharedPtr msg) {
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

void can_bridge::tx_timer_callback () {
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
        // 指令が途絶えたら 0 を送り続ける(ノードごと落ちた場合は Nucleo のウォッチドッグが止める)
        if (!stale) {
            for (size_t i = 0; i < proto::kNumDriveWheels; ++i) rad_s[i] = command_rad_s_[i] * drive_wheel_directions_[i];
        }
    }

    struct can_frame frame{};
    frame.can_id   = proto::kIdDriveCommand;
    frame.len      = 8;
    const auto data = proto::encode_drive_command (rad_s);
    std::memcpy (frame.data, data.data (), data.size ());

    std::lock_guard<std::mutex> lock (socket_mutex_);
    if (socket_fd_ < 0) return;
    if (::write (socket_fd_, &frame, sizeof (frame)) == static_cast<ssize_t> (sizeof (frame))) {
        tx_frames_++;
    } else {
        // ENOBUFS: 送信キューが満杯(相手がいなくて ACK が返らない、など)
        tx_errors_++;
    }
}

void can_bridge::stats_timer_callback () {
    const auto rx      = rx_frames_.exchange (0);
    const auto cycles  = rx_cycles_.exchange (0);
    const auto partial = partial_cycles_.exchange (0);
    const auto tx      = tx_frames_.exchange (0);
    const auto tx_err  = tx_errors_.exchange (0);
    const auto bus_err = bus_errors_.exchange (0);
    if (partial > 0 || tx_err > 0 || bus_err > 0 || cycles == 0) {
        RCLCPP_WARN (this->get_logger (), "rx %lu frames, %lu cycles (%lu incomplete) | tx %lu frames, %lu errors | bus errors %lu", rx, cycles, partial, tx, tx_err, bus_err);
    } else {
        RCLCPP_INFO (this->get_logger (), "rx %lu frames, %lu cycles | tx %lu frames", rx, cycles, tx);
    }
}

}  // namespace can_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE (can_bridge::can_bridge)
