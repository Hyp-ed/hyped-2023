#pragma once

#include <termios.h>

#include <cstdio>

#include <core/logger.hpp>
#include <core/types.hpp>

// Uart3 not exposed in BBB headers
enum class UartBus { kuUart0 = 0, kUart1 = 1, kUart2 = 2, kUart4 = 4, kUart5 = 5 };

namespace hyped::io {

class Uart {
 public:
  Uart(core::ILogger &logger, const UartBus bus, const std::uint32_t baudrate);
  ~Uart();

  core::Result send(char *tx, std::uint8_t length);
  core::Result read(unsigned char *rx, std::uint8_t length);

 private:
  const UartBus bus_;
  core::ILogger &logger_;
  int baudrate_;
  int file_descriptor_;
  struct termios tty_;
};

}  // namespace hyped::io