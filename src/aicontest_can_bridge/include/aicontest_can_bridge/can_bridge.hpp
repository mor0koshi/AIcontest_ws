#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "aicontest_can_bridge/protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8.hpp"

struct can_frame;

namespace can_bridge {

// ROS 2 ⇔ SocketCAN の変換ノード
//   /command_joint_states (rad/s) → 0x300 を一定周期で送信
//   0x310〜0x312 → /joint_states(駆動輪4 + 計測輪2 をまとめて1つ、0x313 を受け取ったときに出す)
//   0x313 → /imu(BNO055 の旋回の速さと向き)
//   0x314 → /can_bridge/status
class can_bridge : public rclcpp::Node {
   public:
    explicit can_bridge (const rclcpp::NodeOptions &options);
    ~can_bridge () override;

   private:
    void command_callback (const sensor_msgs::msg::JointState::SharedPtr msg);
    void tx_timer_callback ();
    void stats_timer_callback ();
    void rx_loop ();
    bool open_socket ();
    void close_socket ();
    void handle_frame (const can_frame &frame);
    void publish_joint_states ();
    void publish_imu (const aicontest_can_bridge::protocol::ImuFrame &imu);

    // パラメータ
    std::string                                     can_interface_;
    std::vector<std::string>                        drive_wheel_names_;
    std::vector<std::string>                        odom_wheel_names_;
    std::vector<double>                             drive_wheel_directions_;
    std::vector<double>                             odom_wheel_directions_;
    std::vector<double>                             odom_counts_per_rev_;
    double                                          command_timeout_sec_;
    std::string                                     imu_frame_id_;
    double                                          imu_sign_;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    joint_state_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr            status_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr           imu_publisher_;
    rclcpp::TimerBase::SharedPtr                                  tx_timer_;
    rclcpp::TimerBase::SharedPtr                                  stats_timer_;

    // 指令(command_callback と tx_timer で共有)
    std::mutex                                                     command_mutex_;
    std::array<double, aicontest_can_bridge::protocol::kNumDriveWheels> command_rad_s_{};
    rclcpp::Time                                                   last_command_time_;
    bool                                                           command_received_ = false;
    bool                                                           command_stale_    = true;

    // 受信状態(rx スレッドだけが触る)
    std::array<double, aicontest_can_bridge::protocol::kNumDriveWheels> drive_rad_s_{};
    std::array<aicontest_can_bridge::protocol::OdomWheel, aicontest_can_bridge::protocol::kNumOdomWheels> odom_{};
    std::array<bool, aicontest_can_bridge::protocol::kNumOdomWheels> odom_fresh_{};
    bool                                                           drive_fresh_ = false;
    int                                                            last_status_ = -1;
    int                                                            last_calib_  = -1;

    // ソケット
    std::mutex       socket_mutex_;
    int              socket_fd_ = -1;
    std::thread      rx_thread_;
    std::atomic_bool running_{true};

    // 統計(stats_timer で表示してリセット)
    std::atomic_uint64_t rx_frames_{0};
    std::atomic_uint64_t rx_cycles_{0};
    std::atomic_uint64_t partial_cycles_{0};
    std::atomic_uint64_t tx_frames_{0};
    std::atomic_uint64_t tx_errors_{0};
    std::atomic_uint64_t bus_errors_{0};
};

}  // namespace can_bridge
