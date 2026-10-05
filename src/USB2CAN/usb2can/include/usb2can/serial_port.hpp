#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <mutex>

namespace usb2can {

// Low-latency Linux serial wrapper using termios + non-blocking I/O.
// A dedicated RX thread reads bytes as soon as they arrive and feeds them to
// a frame-synchronizer callback.
class SerialPort {
public:
  using FrameCallback = std::function<void(const uint8_t* buf, size_t len)>;

  SerialPort();
  ~SerialPort();

  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  // Open the device and configure 8N1 raw mode at the requested baud.
  // header0/header1 + frame_size define how RX bytes are framed.
  bool open(const std::string& device,
            int baudrate,
            uint8_t header0,
            uint8_t header1,
            size_t frame_size,
            std::string& err);

  void close();
  bool is_open() const { return fd_ >= 0; }
  // True only while RX thread is alive AND fd is valid AND no I/O error has been observed.
  bool is_connected() const { return connected_.load(); }

  // Cached parameters used by reconnect().
  bool reopen(std::string& err);

  // Write a complete frame. Thread-safe.
  bool write_frame(const uint8_t* buf, size_t len);

  // Set the RX callback (called from the RX thread for every well-framed
  // packet of frame_size_ bytes whose first two bytes match the headers).
  void set_frame_callback(FrameCallback cb) { cb_ = std::move(cb); }

private:
  void rx_loop();
  bool configure_termios(int baudrate);
  void enable_low_latency();

  int fd_ = -1;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::thread rx_thread_;
  std::mutex write_mtx_;

  // cached open() parameters for reopen()
  std::string device_;
  int baudrate_ = 0;
  uint8_t header0_ = 0xAA;
  uint8_t header1_ = 0x55;
  size_t  frame_size_ = 0;

  FrameCallback cb_;
};

}  // namespace usb2can
