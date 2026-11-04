// Pi ⇔ Nucleo の UART のパケット(docs/can_protocol.md v0.4 の「UART」)。ファームウェアの pi_link.c と同じ。
// パケット = COBS([種類 1][データ][CRC16 2]) + 0x00。CRC16 は CCITT-FALSE(初期値 0xFFFF、多項式 0x1021)をリトルエンディアンで。
// ROS に依存しないので単体テストできる。データの中身は CAN 版(protocol.hpp)と同じ。
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "aicontest_can_bridge/protocol.hpp"

namespace aicontest_can_bridge::serial {

constexpr uint8_t kTypeCommand   = 0x01;  // Pi → Nucleo: 駆動輪の目標角速度 int16 ×4(= CAN 0x300)
constexpr uint8_t kTypeTelemetry = 0x10;  // Nucleo → Pi: 0x310(8)+ 0x311(8)+ 0x312(8)+ 0x313(8)+ 0x314(2)= 34 バイト
constexpr uint8_t kTypeText      = 0x20;  // Nucleo → Pi: printf の文字(1 行)
constexpr size_t  kTelemetrySize = 34;
constexpr size_t  kMaxPacket     = 256;   // 符号化後の最大(これより長い受信は捨てる)

inline uint16_t crc16 (const uint8_t *p, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= static_cast<uint16_t> (p[i]) << 8;
        for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? static_cast<uint16_t> ((crc << 1) ^ 0x1021) : static_cast<uint16_t> (crc << 1);
    }
    return crc;
}

inline std::vector<uint8_t> cobs_encode (const std::vector<uint8_t> &in) {
    std::vector<uint8_t> out (1, 0);
    size_t               code_pos = 0;
    uint8_t              code     = 1;
    for (uint8_t b : in) {
        if (b == 0) {
            out[code_pos] = code;
            code_pos      = out.size ();
            out.push_back (0);
            code = 1;
        } else {
            out.push_back (b);
            if (++code == 0xFF) {
                out[code_pos] = code;
                code_pos      = out.size ();
                out.push_back (0);
                code = 1;
            }
        }
    }
    out[code_pos] = code;
    return out;
}

// 壊れていれば nullopt
inline std::optional<std::vector<uint8_t>> cobs_decode (const std::vector<uint8_t> &in) {
    std::vector<uint8_t> out;
    size_t               i = 0;
    while (i < in.size ()) {
        const uint8_t code = in[i++];
        if (code == 0) return std::nullopt;
        for (uint8_t k = 1; k < code; ++k) {
            if (i >= in.size ()) return std::nullopt;
            out.push_back (in[i++]);
        }
        if (code != 0xFF && i < in.size ()) out.push_back (0);
    }
    return out;
}

// [種類][データ] → 送るバイト列(区切りの 0x00 まで含む)
inline std::vector<uint8_t> make_packet (uint8_t type, const uint8_t *data, size_t len) {
    std::vector<uint8_t> raw;
    raw.reserve (len + 3);
    raw.push_back (type);
    raw.insert (raw.end (), data, data + len);
    const uint16_t crc = crc16 (raw.data (), raw.size ());
    raw.push_back (static_cast<uint8_t> (crc & 0xFF));
    raw.push_back (static_cast<uint8_t> (crc >> 8));
    auto out = cobs_encode (raw);
    out.push_back (0x00);
    return out;
}

struct Packet {
    uint8_t              type = 0;
    std::vector<uint8_t> data;
};

// 区切り(0x00)の手前までの符号化されたバイト列 → パケット。COBS か CRC が合わなければ nullopt
inline std::optional<Packet> parse_packet (const std::vector<uint8_t> &encoded) {
    auto dec = cobs_decode (encoded);
    if (!dec || dec->size () < 3) return std::nullopt;
    const size_t   n   = dec->size ();
    const uint16_t crc = static_cast<uint16_t> ((*dec)[n - 2] | ((*dec)[n - 1] << 8));
    if (crc16 (dec->data (), n - 2) != crc) return std::nullopt;
    return Packet{(*dec)[0], std::vector<uint8_t> (dec->begin () + 1, dec->end () - 2)};
}

// 受信したバイトを少しずつ入れると、区切りごとにパケットを返す
class PacketReader {
public:
    // 区切りが来たら true を返し、packet にパケット(壊れていれば nullopt)を入れる
    bool push (uint8_t b, std::optional<Packet> &packet) {
        if (b == 0x00) {
            if (buf_.empty () && !overflow_) return false;  // 続けて来た区切りは無視
            packet = overflow_ ? std::nullopt : parse_packet (buf_);
            buf_.clear ();
            overflow_ = false;
            return true;
        }
        if (buf_.size () < kMaxPacket) {
            buf_.push_back (b);
        } else {
            overflow_ = true;
        }
        return false;
    }

private:
    std::vector<uint8_t> buf_;
    bool                 overflow_ = false;
};

struct Telemetry {
    std::array<double, protocol::kNumDriveWheels>             drive_rad_s{};
    std::array<protocol::OdomWheel, protocol::kNumOdomWheels> odom{};
    protocol::ImuFrame                                        imu{};
    uint8_t                                                   status = 0;
    uint8_t                                                   calib  = 0;
};

inline std::optional<Telemetry> decode_telemetry (const std::vector<uint8_t> &d) {
    if (d.size () < kTelemetrySize) return std::nullopt;
    Telemetry t;
    t.drive_rad_s = protocol::decode_drive_feedback (&d[0]);
    t.odom[0]     = protocol::decode_odom_wheel (&d[8]);
    t.odom[1]     = protocol::decode_odom_wheel (&d[16]);
    t.imu         = protocol::decode_imu (&d[24]);
    t.status      = d[32];
    t.calib       = d[33];
    return t;
}

inline std::vector<uint8_t> encode_telemetry (const Telemetry &t) {  // 偽 Nucleo・テスト用
    std::vector<uint8_t> d (kTelemetrySize, 0);
    for (int i = 0; i < protocol::kNumDriveWheels; ++i) protocol::put_i16 (&d[static_cast<size_t> (2 * i)], protocol::velocity_to_raw (t.drive_rad_s[static_cast<size_t> (i)]));
    const auto o0  = protocol::encode_odom_wheel (t.odom[0]);
    const auto o1  = protocol::encode_odom_wheel (t.odom[1]);
    const auto imu = protocol::encode_imu (t.imu);
    std::copy (o0.begin (), o0.end (), d.begin () + 8);
    std::copy (o1.begin (), o1.end (), d.begin () + 16);
    std::copy (imu.begin (), imu.end (), d.begin () + 24);
    d[32] = t.status;
    d[33] = t.calib;
    return d;
}

}  // namespace aicontest_can_bridge::serial
