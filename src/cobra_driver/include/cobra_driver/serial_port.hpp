// Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#ifndef COBRA_DRIVER__SERIAL_PORT_HPP_
#define COBRA_DRIVER__SERIAL_PORT_HPP_

#include <string>
#include <termios.h>

namespace cobra_driver
{

/// Minimal POSIX serial port wrapper (no external library dependencies).
class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  /// Open the serial port at the given path and baud rate.
  /// @return true on success.
  bool open(const std::string & port, int baud_rate);

  /// Close the port if open.
  void close();

  /// @return true if the port file descriptor is valid.
  bool isOpen() const;

  /// Write a string to the serial port.
  /// @return number of bytes written, or -1 on error.
  ssize_t write(const std::string & data);

  /// Read a single line (terminated by '\n') from the serial port.
  /// Blocks until a full line is available or timeout expires.
  /// @param timeout_ms  read timeout in milliseconds (0 = blocking).
  /// @return the line (without trailing newline), or empty string on timeout/error.
  std::string readLine(int timeout_ms = 100);

private:
  int fd_{-1};
  struct termios original_tty_;
  std::string read_buffer_;

  static speed_t baudRateToSpeed(int baud_rate);
};

}  // namespace cobra_driver

#endif  // COBRA_DRIVER__SERIAL_PORT_HPP_
