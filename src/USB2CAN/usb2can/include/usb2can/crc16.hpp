#pragma once
#include <cstdint>
#include <cstddef>

namespace usb2can {

// CRC-16/MODBUS: poly 0xA001 (reflected 0x8005), init 0xFFFF, no final xor, refin/refout = true.
uint16_t crc16_modbus(const uint8_t* data, size_t len);

}  // namespace usb2can
