// Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#include "cobra_driver/serial_port.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <stdexcept>

namespace cobra_driver
{

SerialPort::SerialPort() = default;

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & port, int baud_rate)
{
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  // Save original settings so we can restore on close
  if (tcgetattr(fd_, &original_tty_) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));

  speed_t speed = baudRateToSpeed(baud_rate);

  // Input/output baud rate
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // 8N1 — 8 data bits, no parity, 1 stop bit
  tty.c_cflag |= (CS8 | CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

  // Raw input — no canonical processing, no echo, no signals
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  // No software flow control, no special input processing
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT |
                    PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  // Raw output
  tty.c_oflag &= ~OPOST;

  // Non-blocking read: return immediately with whatever is available
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Flush any stale data
  tcflush(fd_, TCIOFLUSH);

  // Clear non-blocking flag now that termios is configured —
  // we use select() for timed reads instead.
  int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

  read_buffer_.clear();
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    tcsetattr(fd_, TCSANOW, &original_tty_);
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::isOpen() const
{
  return fd_ >= 0;
}

ssize_t SerialPort::write(const std::string & data)
{
  if (fd_ < 0) {
    return -1;
  }
  return ::write(fd_, data.c_str(), data.size());
}

std::string SerialPort::readLine(int timeout_ms)
{
  // Check if we already have a complete line buffered
  auto newline_pos = read_buffer_.find('\n');
  if (newline_pos != std::string::npos) {
    std::string line = read_buffer_.substr(0, newline_pos);
    read_buffer_.erase(0, newline_pos + 1);
    // Strip trailing \r if present
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    return line;
  }

  if (fd_ < 0) {
    return "";
  }

  // Read until we get a newline or timeout
  char buf[256];
  while (true) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret <= 0) {
      // Timeout or error
      return "";
    }

    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) {
      return "";
    }

    read_buffer_.append(buf, static_cast<size_t>(n));

    newline_pos = read_buffer_.find('\n');
    if (newline_pos != std::string::npos) {
      std::string line = read_buffer_.substr(0, newline_pos);
      read_buffer_.erase(0, newline_pos + 1);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      return line;
    }
  }
}

speed_t SerialPort::baudRateToSpeed(int baud_rate)
{
  switch (baud_rate) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:     return B115200;
  }
}

}  // namespace cobra_driver
