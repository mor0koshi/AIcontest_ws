// Pi ⇔ Nucleo の CAN フレームの組み立て・分解(docs/can_protocol.md v0.3)
// ROS に依存しないので単体テストできる。マルチバイト値はすべてリトルエンディアン。
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace aicontest_can_bridge::protocol {

// ---- CAN ID ----
constexpr uint32_t kIdDriveCommand  = 0x300;  // Pi → Nucleo 駆動輪目標速度
constexpr uint32_t kIdDriveFeedback = 0x310;  // Nucleo → Pi 駆動輪実速度
constexpr uint32_t kIdOdomWheel0    = 0x311;  // Nucleo → Pi 計測輪0(前後向き)
constexpr uint32_t kIdOdomWheel1    = 0x312;  // Nucleo → Pi 計測輪1(横向き)
constexpr uint32_t kIdImu           = 0x313;  // Nucleo → Pi IMU(BNO055)。1周期の最後のフレーム
constexpr uint32_t kIdStatus        = 0x314;  // Nucleo → Pi 状態フラグ + IMU の校正状態

// 受信フィルタ: 0x310〜0x317 を通す
constexpr uint32_t kRxFilterId   = 0x310;
constexpr uint32_t kRxFilterMask = 0x7F8;

constexpr int    kNumDriveWheels = 4;
constexpr int    kNumOdomWheels  = 2;
constexpr double kVelocityScale  = 100.0;  // int16 1 = 0.01 rad/s

// ---- 状態フラグ(0x314 byte0)----
constexpr uint8_t kStatusWatchdogStop = 1u << 0;  // Piからの指令が途絶えて停止中
constexpr uint8_t kStatusImuLost      = 1u << 5;  // IMU(BNO055)が応答しない
inline bool motor_feedback_lost (uint8_t flags, int motor_index) {  // motor_index 0〜3 = C610 ID1〜4
    return (flags >> (1 + motor_index)) & 1u;
}

// ---- リトルエンディアンの読み書き ----
inline void put_i16 (uint8_t *p, int16_t v) {
    const auto u = static_cast<uint16_t> (v);
    p[0]         = static_cast<uint8_t> (u & 0xFF);
    p[1]         = static_cast<uint8_t> (u >> 8);
}
inline int16_t get_i16 (const uint8_t *p) {
    return static_cast<int16_t> (static_cast<uint16_t> (p[0] | (p[1] << 8)));
}
inline void put_i32 (uint8_t *p, int32_t v) {
    const auto u = static_cast<uint32_t> (v);
    for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t> ((u >> (8 * i)) & 0xFF);
}
inline int32_t get_i32 (const uint8_t *p) {
    uint32_t u = 0;
    for (int i = 0; i < 4; ++i) u |= static_cast<uint32_t> (p[i]) << (8 * i);
    return static_cast<int32_t> (u);
}
inline void put_f32 (uint8_t *p, float v) {
    uint32_t u;
    std::memcpy (&u, &v, sizeof (u));
    put_i32 (p, static_cast<int32_t> (u));
}
inline float get_f32 (const uint8_t *p) {
    const auto u = static_cast<uint32_t> (get_i32 (p));
    float      v;
    std::memcpy (&v, &u, sizeof (v));
    return v;
}

// rad/s → int16(0.01 rad/s 単位)。範囲外は飽和させる。NaN は 0
inline int16_t velocity_to_raw (double rad_s) {
    if (!std::isfinite (rad_s)) return 0;
    const double scaled = std::round (rad_s * kVelocityScale);
    return static_cast<int16_t> (std::clamp (scaled, -32768.0, 32767.0));
}
inline double raw_to_velocity (int16_t raw) {
    return static_cast<double> (raw) / kVelocityScale;
}

// ---- 0x300 駆動輪目標速度 ----
inline std::array<uint8_t, 8> encode_drive_command (const std::array<double, kNumDriveWheels> &rad_s) {
    std::array<uint8_t, 8> data{};
    for (int i = 0; i < kNumDriveWheels; ++i) put_i16 (&data[static_cast<size_t> (2 * i)], velocity_to_raw (rad_s[static_cast<size_t> (i)]));
    return data;
}

// ---- 0x310 駆動輪実速度 ----
inline std::array<double, kNumDriveWheels> decode_drive_feedback (const uint8_t *data) {
    std::array<double, kNumDriveWheels> rad_s{};
    for (int i = 0; i < kNumDriveWheels; ++i) rad_s[static_cast<size_t> (i)] = raw_to_velocity (get_i16 (&data[2 * i]));
    return rad_s;
}

// ---- 0x311・0x312 計測輪(dinosaur_library encoder_can 形式)----
struct OdomWheel {
    int32_t count = 0;    // 累積カウント(4逓倍込み)
    float   rpm   = 0.f;  // 計測輪の回転数
};
inline std::array<uint8_t, 8> encode_odom_wheel (const OdomWheel &w) {
    std::array<uint8_t, 8> data{};
    put_i32 (&data[0], w.count);
    put_f32 (&data[4], w.rpm);
    return data;
}
inline OdomWheel decode_odom_wheel (const uint8_t *data) {
    return OdomWheel{get_i32 (&data[0]), get_f32 (&data[4])};
}
// ---- 0x313 IMU(BNO055)----
struct ImuFrame {
    float yaw_rate = 0.f;  // 旋回の速さ [rad/s](反時計回りが正)
    float heading  = 0.f;  // 向き [rad](BNO055 の姿勢推定、反時計回りが正、電源投入時が 0)
};
inline std::array<uint8_t, 8> encode_imu (const ImuFrame &f) {
    std::array<uint8_t, 8> data{};
    put_f32 (&data[0], f.yaw_rate);
    put_f32 (&data[4], f.heading);
    return data;
}
inline ImuFrame decode_imu (const uint8_t *data) {
    return ImuFrame{get_f32 (&data[0]), get_f32 (&data[4])};
}
// BNO055 の CALIB_STAT から、ジャイロの校正状態(0〜3)を取り出す
inline int gyro_calibration (uint8_t calib_stat) {
    return (calib_stat >> 4) & 0x3;
}

inline double rpm_to_rad_s (double rpm) {
    return rpm * 2.0 * M_PI / 60.0;
}
inline double count_to_rad (int32_t count, double counts_per_rev) {
    return counts_per_rev > 0.0 ? static_cast<double> (count) * 2.0 * M_PI / counts_per_rev : 0.0;
}

}  // namespace aicontest_can_bridge::protocol
