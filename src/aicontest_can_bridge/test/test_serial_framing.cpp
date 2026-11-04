// UART のパケット(serial_framing.hpp)の単体テスト。ファームウェアの pi_link.c と同じ形かを確かめる
#include <gtest/gtest.h>

#include "aicontest_can_bridge/serial_framing.hpp"

namespace ser   = aicontest_can_bridge::serial;
namespace proto = aicontest_can_bridge::protocol;

TEST (SerialFraming, Crc16CcittFalseKnownValue) {
    const std::string s = "123456789";
    EXPECT_EQ (ser::crc16 (reinterpret_cast<const uint8_t *> (s.data ()), s.size ()), 0x29B1);  // CRC-16/CCITT-FALSE の検査値
}

TEST (SerialFraming, CobsRoundTripWithZerosAndLongRuns) {
    std::vector<std::vector<uint8_t>> cases = {{}, {0}, {0, 0}, {1, 2, 3}, {1, 0, 2, 0, 0, 3}};
    std::vector<uint8_t>              longrun (600);
    for (size_t i = 0; i < longrun.size (); ++i) longrun[i] = static_cast<uint8_t> (i % 255 + 1);  // 0 を含まない 254 超の連続
    cases.push_back (longrun);
    longrun[300] = 0;
    cases.push_back (longrun);
    for (const auto &c : cases) {
        const auto enc = ser::cobs_encode (c);
        for (uint8_t b : enc) EXPECT_NE (b, 0);  // 符号化後に 0 は出ない(0 は区切り専用)
        const auto dec = ser::cobs_decode (enc);
        ASSERT_TRUE (dec.has_value ());
        EXPECT_EQ (*dec, c);
    }
}

TEST (SerialFraming, PacketRoundTripAndCrcRejectsCorruption) {
    const uint8_t data[8] = {0x10, 0x00, 0xF0, 0xFF, 0x00, 0x00, 0x7F, 0x80};
    auto          bytes   = ser::make_packet (ser::kTypeCommand, data, sizeof (data));
    ASSERT_EQ (bytes.back (), 0x00);

    ser::PacketReader          reader;
    std::optional<ser::Packet> pkt;
    int                        done = 0;
    for (uint8_t b : bytes) done += reader.push (b, pkt) ? 1 : 0;
    ASSERT_EQ (done, 1);
    ASSERT_TRUE (pkt.has_value ());
    EXPECT_EQ (pkt->type, ser::kTypeCommand);
    EXPECT_EQ (pkt->data, std::vector<uint8_t> (data, data + 8));

    bytes[3] ^= 0x01;  // 1 ビット壊す
    std::optional<ser::Packet> bad;
    for (uint8_t b : bytes) reader.push (b, bad);
    EXPECT_FALSE (bad.has_value ());
}

TEST (SerialFraming, ReaderResyncsAfterGarbage) {
    ser::PacketReader          reader;
    std::optional<ser::Packet> pkt;
    for (uint8_t b : std::vector<uint8_t>{0x55, 0x12, 0x99}) reader.push (b, pkt);  // 途中から読み始めた残り
    reader.push (0x00, pkt);                                                          // 壊れたパケットとして捨てる
    EXPECT_FALSE (pkt.has_value ());
    const uint8_t text[] = {'h', 'i'};
    std::optional<ser::Packet> ok;
    for (uint8_t b : ser::make_packet (ser::kTypeText, text, 2)) reader.push (b, ok);
    ASSERT_TRUE (ok.has_value ());
    EXPECT_EQ (ok->type, ser::kTypeText);
}

TEST (SerialFraming, TelemetryLayoutMatchesCanFrames) {
    ser::Telemetry t;
    t.drive_rad_s = {1.23, -4.56, 0.0, 7.89};
    t.odom[0]     = {123456, 12.5f};
    t.odom[1]     = {-42, -3.25f};
    t.imu         = {0.5f, -1.0f};
    t.status      = proto::kStatusImuLost;
    t.calib       = 0x30;
    const auto d  = ser::encode_telemetry (t);
    ASSERT_EQ (d.size (), ser::kTelemetrySize);
    // 各 8 バイトが CAN 版の 0x310 / 0x311 / 0x312 / 0x313 の中身と同じ
    const auto o0 = proto::encode_odom_wheel (t.odom[0]);
    EXPECT_TRUE (std::equal (o0.begin (), o0.end (), d.begin () + 8));
    const auto u = ser::decode_telemetry (d);
    ASSERT_TRUE (u.has_value ());
    EXPECT_NEAR (u->drive_rad_s[1], -4.56, 1e-9);
    EXPECT_EQ (u->odom[0].count, 123456);
    EXPECT_FLOAT_EQ (u->odom[1].rpm, -3.25f);
    EXPECT_FLOAT_EQ (u->imu.yaw_rate, 0.5f);
    EXPECT_EQ (u->status, proto::kStatusImuLost);
    EXPECT_EQ (u->calib, 0x30);
    EXPECT_FALSE (ser::decode_telemetry (std::vector<uint8_t> (10, 0)).has_value ());
}
