#include "usb2can/protocol.hpp"
#include "usb2can/crc16.hpp"

#include <cstring>

namespace usb2can {

static inline void write_le_u16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

static inline uint16_t read_le_u16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

static inline float read_le_float(const uint8_t* p) {
  float f;
  std::memcpy(&f, p, sizeof(float));  // host is LE on x86/ARM Linux
  return f;
}

void pack_downlink(const DownlinkFrame& in,
                   uint8_t header0, uint8_t header1,
                   uint8_t* buf) {
  std::memset(buf, 0, FRAME_SIZE);
  buf[0] = header0;
  buf[1] = header1;
  // D2 = 数据长度 = 总帧长 - 帧头(2) - 长度字段(1) = 242
  buf[LEN_OFFSET] = PAYLOAD_LEN;

  // CTRL1
  buf[TX_CTRL1_OFFSET] = in.ctrl1.cmd;
  std::memcpy(&buf[TX_CTRL1_OFFSET + 1], in.ctrl1.payload.data(), 64);
  // CTRL2
  buf[TX_CTRL2_OFFSET] = in.ctrl2.cmd;
  std::memcpy(&buf[TX_CTRL2_OFFSET + 1], in.ctrl2.payload.data(), 64);
  // CTRL3
  buf[TX_CTRL3_OFFSET] = in.ctrl3.cmd;
  std::memcpy(&buf[TX_CTRL3_OFFSET + 1], in.ctrl3.payload.data(), 64);

  buf[TX_IMU_CMD_OFFSET] = in.imu_cmd;
  // Reserved already zero.

  uint16_t crc = crc16_modbus(buf, CRC_OFFSET);
  write_le_u16(&buf[CRC_OFFSET], crc);
}

bool parse_uplink(const uint8_t* buf,
                  uint8_t header0, uint8_t header1,
                  UplinkFrame& out) {
  if (buf[0] != header0 || buf[1] != header1) return false;
  uint16_t crc_calc = crc16_modbus(buf, CRC_OFFSET);
  uint16_t crc_recv = read_le_u16(&buf[CRC_OFFSET]);
  if (crc_calc != crc_recv) return false;

  std::memcpy(out.ctrl1.data(), &buf[RX_CTRL1_OFFSET], RX_CTRL_LEN);
  std::memcpy(out.ctrl2.data(), &buf[RX_CTRL2_OFFSET], RX_CTRL_LEN);
  std::memcpy(out.ctrl3.data(), &buf[RX_CTRL3_OFFSET], RX_CTRL_LEN);

  const uint8_t* p = &buf[RX_IMU_OFFSET];
  out.imu.acc_x = read_le_float(p +  0);
  out.imu.acc_y = read_le_float(p +  4);
  out.imu.acc_z = read_le_float(p +  8);
  out.imu.gyr_x = read_le_float(p + 12);
  out.imu.gyr_y = read_le_float(p + 16);
  out.imu.gyr_z = read_le_float(p + 20);
  out.imu.qw    = read_le_float(p + 24);
  out.imu.qx    = read_le_float(p + 28);
  out.imu.qy    = read_le_float(p + 32);
  out.imu.qz    = read_le_float(p + 36);
  return true;
}

}  // namespace usb2can
