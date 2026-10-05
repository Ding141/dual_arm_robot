#include "usb2can/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>

#include <cerrno>
#include <cstring>
#include <vector>

namespace usb2can {

namespace {
int baud_to_termios(int baud) {
  switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 500000:  return B500000;
    case 921600:  return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 3000000: return B3000000;
    default:      return -1;
  }
}
}  // namespace

SerialPort::SerialPort() = default;

SerialPort::~SerialPort() { close(); }

bool SerialPort::configure_termios(int baudrate) {
  struct termios tty{};
  if (tcgetattr(fd_, &tty) != 0) return false;

  cfmakeraw(&tty);

  int b = baud_to_termios(baudrate);
  if (b < 0) {
    // Fallback: many USB CDC devices ignore baud completely.
    b = B115200;
  }
  cfsetispeed(&tty, b);
  cfsetospeed(&tty, b);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  // Non-blocking style read: return immediately with whatever is available.
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  tcflush(fd_, TCIOFLUSH);
  return tcsetattr(fd_, TCSANOW, &tty) == 0;
}

void SerialPort::enable_low_latency() {
  // Set the kernel ASYNC_LOW_LATENCY flag if the driver supports it
  // (FTDI / CDC-ACM / CH34x all do). This drops the default 16 ms
  // poll interval down to ~1 ms or better.
  struct serial_struct ss{};
  if (ioctl(fd_, TIOCGSERIAL, &ss) == 0) {
    ss.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd_, TIOCSSERIAL, &ss);
  }
}

bool SerialPort::open(const std::string& device,
                      int baudrate,
                      uint8_t header0,
                      uint8_t header1,
                      size_t frame_size,
                      std::string& err) {
  close();

  device_     = device;
  baudrate_   = baudrate;
  header0_    = header0;
  header1_    = header1;
  frame_size_ = frame_size;

  return reopen(err);
}

bool SerialPort::reopen(std::string& err) {
  // Tear down any prior fd / rx thread but keep cached parameters.
  running_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  connected_ = false;

  if (device_.empty()) { err = "device not configured"; return false; }

  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    err = std::string("open ") + device_ + ": " + std::strerror(errno);
    return false;
  }

  if (!configure_termios(baudrate_)) {
    err = std::string("termios config failed: ") + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  enable_low_latency();

  running_ = true;
  connected_ = true;
  rx_thread_ = std::thread(&SerialPort::rx_loop, this);

  sched_param sp{};
  sp.sched_priority = 20;
  pthread_setschedparam(rx_thread_.native_handle(), SCHED_FIFO, &sp);

  return true;
}

void SerialPort::close() {
  running_ = false;
  connected_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::write_frame(const uint8_t* buf, size_t len) {
  if (fd_ < 0 || !connected_) return false;
  std::lock_guard<std::mutex> lk(write_mtx_);
  size_t total = 0;
  while (total < len) {
    ssize_t n = ::write(fd_, buf + total, len - total);
    if (n > 0) {
      total += static_cast<size_t>(n);
    } else if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) {
        struct pollfd pfd{fd_, POLLOUT, 0};
        ::poll(&pfd, 1, 10);
        continue;
      }
      // Hard write error -> mark disconnected so node can reconnect.
      connected_ = false;
      return false;
    } else {
      connected_ = false;
      return false;
    }
  }
  return true;
}

void SerialPort::rx_loop() {
  // Sliding-window framer. Looks for header0,header1 and then collects
  // exactly frame_size_ bytes (including the header). On success, hand the
  // buffer to the callback.
  std::vector<uint8_t> rolling;
  rolling.reserve(frame_size_ * 4);
  uint8_t chunk[1024];

  while (running_) {
    struct pollfd pfd{fd_, POLLIN, 0};
    int pr = ::poll(&pfd, 1, 50);
    if (pr < 0) {
      if (errno == EINTR) continue;
      connected_ = false;
      running_   = false;
      break;
    }
    if (pr == 0) continue;
    // USB device unplugged typically surfaces as POLLHUP / POLLERR / POLLNVAL.
    if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
      connected_ = false;
      running_   = false;
      break;
    }
    if (!(pfd.revents & POLLIN)) continue;

    ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n == 0) {
      // EOF on a tty usually means the USB device went away.
      connected_ = false;
      running_   = false;
      break;
    }
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) continue;
      // ENXIO / ENODEV / EIO -> device disconnected.
      connected_ = false;
      running_   = false;
      break;
    }
    rolling.insert(rolling.end(), chunk, chunk + n);

    // Try to extract as many frames as possible.
    while (rolling.size() >= frame_size_) {
      // Re-sync to header.
      size_t i = 0;
      bool found = false;
      for (; i + 1 < rolling.size(); ++i) {
        if (rolling[i] == header0_ && rolling[i + 1] == header1_) {
          found = true;
          break;
        }
      }
      if (!found) {
        // Keep the last byte (it might start a header).
        if (rolling.size() > 1) rolling.erase(rolling.begin(), rolling.end() - 1);
        break;
      }
      if (i > 0) rolling.erase(rolling.begin(), rolling.begin() + i);
      if (rolling.size() < frame_size_) break;

      // We have a candidate frame at the front.
      if (cb_) cb_(rolling.data(), frame_size_);
      rolling.erase(rolling.begin(), rolling.begin() + frame_size_);
    }

    // Bound memory in pathological cases.
    if (rolling.size() > frame_size_ * 8) {
      rolling.erase(rolling.begin(), rolling.end() - frame_size_);
    }
  }
}

}  // namespace usb2can
