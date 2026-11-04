#include <gtest/gtest.h>

#include "aicontest_can_bridge/protocol.hpp"

namespace proto = aicontest_can_bridge::protocol;

TEST (Protocol, DriveCommandByteLayoutIsLittleEndian) {
    // 1.00 rad/s → 100 = 0x0064, -1.00 → -100 = 0xFF9C
    const auto d = proto::encode_drive_command ({1.0, -1.0, 0.0, 3.27});
    EXPECT_EQ (d[0], 0x64);
    EXPECT_EQ (d[1], 0x00);
    EXPECT_EQ (d[2], 0x9C);
    EXPECT_EQ (d[3], 0xFF);
    EXPECT_EQ (d[4], 0x00);
    EXPECT_EQ (d[5], 0x00);
    EXPECT_EQ (d[6], 0x47);  // 327 = 0x0147
    EXPECT_EQ (d[7], 0x01);
}

TEST (Protocol, VelocityRoundsAndSaturates) {
    EXPECT_EQ (proto::velocity_to_raw (0.004), 0);
    EXPECT_EQ (proto::velocity_to_raw (0.005), 1);
    EXPECT_EQ (proto::velocity_to_raw (-0.006), -1);
    EXPECT_EQ (proto::velocity_to_raw (1000.0), 32767);
    EXPECT_EQ (proto::velocity_to_raw (-1000.0), -32768);
    EXPECT_EQ (proto::velocity_to_raw (std::nan ("")), 0);
}

TEST (Protocol, DriveFeedbackRoundTrip) {
    const std::array<double, 4> in = {7.07, -7.07, 52.0, -0.01};
    const auto                  d  = proto::encode_drive_command (in);  // 0x310 も同じ並び
    const auto                  out = proto::decode_drive_feedback (d.data ());
    for (size_t i = 0; i < 4; ++i) EXPECT_NEAR (out[i], in[i], 0.005);
}

TEST (Protocol, OdomWheelMatchesEncoderCanLayout) {
    // [int32 count][float rpm](リトルエンディアン)
    const auto d = proto::encode_odom_wheel ({-2, 1.0f});
    EXPECT_EQ (d[0], 0xFE);
    EXPECT_EQ (d[1], 0xFF);
    EXPECT_EQ (d[2], 0xFF);
    EXPECT_EQ (d[3], 0xFF);
    // 1.0f = 0x3F800000
    EXPECT_EQ (d[4], 0x00);
    EXPECT_EQ (d[5], 0x00);
    EXPECT_EQ (d[6], 0x80);
    EXPECT_EQ (d[7], 0x3F);

    const auto w = proto::decode_odom_wheel (proto::encode_odom_wheel ({123456789, -98.5f}).data ());
    EXPECT_EQ (w.count, 123456789);
    EXPECT_FLOAT_EQ (w.rpm, -98.5f);
}

TEST (Protocol, UnitConversions) {
    EXPECT_NEAR (proto::rpm_to_rad_s (60.0), 2.0 * M_PI, 1e-9);
    EXPECT_NEAR (proto::count_to_rad (4096, 4096.0), 2.0 * M_PI, 1e-9);
    EXPECT_EQ (proto::count_to_rad (100, 0.0), 0.0);
}

TEST (Protocol, StatusFlags) {
    const uint8_t flags = proto::kStatusWatchdogStop | (1u << 3);  // watchdog + C610 ID3
    EXPECT_TRUE (flags & proto::kStatusWatchdogStop);
    EXPECT_FALSE (proto::motor_feedback_lost (flags, 0));
    EXPECT_TRUE (proto::motor_feedback_lost (flags, 2));
}

TEST (Protocol, ImuFrameRoundTrip) {
    // [float yaw_rate][float heading](リトルエンディアン)
    const auto d = proto::encode_imu ({1.0f, -0.5f});
    EXPECT_EQ (d[3], 0x3F);  // 1.0f = 0x3F800000
    EXPECT_EQ (d[2], 0x80);
    const auto f = proto::decode_imu (d.data ());
    EXPECT_FLOAT_EQ (f.yaw_rate, 1.0f);
    EXPECT_FLOAT_EQ (f.heading, -0.5f);
    EXPECT_EQ (proto::kIdImu, 0x313u);
}

TEST (Protocol, ImuStatusAndCalibration) {
    EXPECT_TRUE (proto::kStatusImuLost & (1u << 5));
    EXPECT_EQ (proto::gyro_calibration (0b11'11'00'00), 3);  // sys=3, gyro=3
    EXPECT_EQ (proto::gyro_calibration (0b00'01'11'11), 1);
}
