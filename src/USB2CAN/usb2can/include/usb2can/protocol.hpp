#pragma once
//
// USB <-> STM32 DCU protocol.
//
// Both the down-link (host -> DCU) and up-link (DCU -> host) frames have the
// same total length of 245 bytes. Layout is documented inline below.
//
//  ----- DOWNLINK (host -> DCU) -----
//  D0..D1   header (0xAA 0x55 by default, configurable)
//  D2       length (payload length = 245 - 2 - 1 = 242 = 0xF2)
//  D3..D67  CANFD CTRL1 : D3 = Cmd, D4..D67 = 64-byte payload
//  D68..D132  CANFD CTRL2 : same layout
//  D133..D197 CANFD CTRL3 : same layout
//  D198     IMU CMD
//  D199..D242 Reserved (zeroed)
//  D243..D244 CRC-16 (little-endian) over D0..D242
//
//  ----- UPLINK (DCU -> host) -----
//  D0..D1   header
//  D2       length
//  D3..D66  CANFD CTRL1 (64 bytes, ID-indexed: ID i -> bytes[i*8 .. i*8+7])
//  D67..D130  CANFD CTRL2 (64 bytes)
//  D131..D194 CANFD CTRL3 (64 bytes)
//  D195..D234 IMU data:
//     D195..D206 Acc  X/Y/Z   (3 x float, little-endian)
//     D207..D218 Gyro X/Y/Z   (3 x float, little-endian)
//     D219..D234 Quat W/X/Y/Z (4 x float, little-endian)
//  D235..D242 Reserved
//  D243..D244 CRC-16 (little-endian) over D0..D242
//
#include <array>
#include <cstdint>
#include <cstring>

namespace usb2can {

constexpr size_t FRAME_SIZE        = 245;
constexpr size_t HEADER_SIZE       = 2;
constexpr size_t LEN_FIELD_SIZE    = 1;
constexpr size_t CRC_SIZE          = 2;
constexpr size_t CRC_OFFSET        = FRAME_SIZE - CRC_SIZE;   // 243
constexpr size_t LEN_OFFSET        = 2;
// D2 = payload length = frame_total - header(2) - length(1) = 242
constexpr uint8_t PAYLOAD_LEN      = FRAME_SIZE - HEADER_SIZE - LEN_FIELD_SIZE; // 242
constexpr uint8_t DEFAULT_HEADER0  = 0xAA;
constexpr uint8_t DEFAULT_HEADER1  = 0x55;

// ---- Downlink (TX) field offsets ----
constexpr size_t TX_CTRL1_OFFSET   = 3;     // D3
constexpr size_t TX_CTRL2_OFFSET   = 68;    // D68
constexpr size_t TX_CTRL3_OFFSET   = 133;   // D133
constexpr size_t TX_CTRL_BLOCK_LEN = 65;    // 1 (Cmd) + 64 (Payload)
constexpr size_t TX_IMU_CMD_OFFSET = 198;   // D198
constexpr size_t TX_RESERVED_OFFSET = 199;  // D199..D242
constexpr size_t TX_RESERVED_LEN   = 244 - 199 + 1; // 44

// ---- Uplink (RX) field offsets ----
constexpr size_t RX_CTRL1_OFFSET   = 3;     // D3
constexpr size_t RX_CTRL2_OFFSET   = 67;    // D67
constexpr size_t RX_CTRL3_OFFSET   = 131;   // D131
constexpr size_t RX_CTRL_LEN       = 64;
constexpr size_t RX_IMU_OFFSET     = 195;   // D195
constexpr size_t RX_IMU_LEN        = 40;    // 3+3+4 floats
constexpr size_t RX_RESERVED_OFFSET = 235;  // D235..D242
constexpr size_t RX_RESERVED_LEN   = 8;

struct ChannelCmd {
  uint8_t cmd = 0;
  std::array<uint8_t, 64> payload{};
};

struct DownlinkFrame {
  ChannelCmd ctrl1;
  ChannelCmd ctrl2;
  ChannelCmd ctrl3;
  uint8_t imu_cmd = 0;
};

struct ImuSample {
  float acc_x = 0, acc_y = 0, acc_z = 0;
  float gyr_x = 0, gyr_y = 0, gyr_z = 0;
  float qw = 1, qx = 0, qy = 0, qz = 0;
};

struct UplinkFrame {
  std::array<uint8_t, 64> ctrl1{};
  std::array<uint8_t, 64> ctrl2{};
  std::array<uint8_t, 64> ctrl3{};
  ImuSample imu;
};

// Pack a DownlinkFrame into a 245-byte buffer (with CRC).
void pack_downlink(const DownlinkFrame& in,
                   uint8_t header0, uint8_t header1,
                   uint8_t* out_buf /* size FRAME_SIZE */);

// Try to parse a 245-byte uplink buffer. Returns true on success (header + CRC ok).
bool parse_uplink(const uint8_t* buf,
                  uint8_t header0, uint8_t header1,
                  UplinkFrame& out);

}  // namespace usb2can
