#pragma once

#include <strings.h>
#include <termios.h>

#include <cstdio>

#include <core/logger.hpp>
#include <core/types.hpp>

// Uart3 not exposed in BBB headers
enum class UartBus { kUart0 = 0, kUart1 = 1, kUart2 = 2, kUart4 = 4, kUart5 = 5 };

namespace hyped::io {

class Uart {
 public:
  static std::optional<Uart> create(
    core::ILogger &logger,
    const UartBus bus,
    const std::uint32_t baud_rate,  // TODOLater: Figure out a default for this by testing
    const std::uint8_t bits_per_bye = 8);
  ~Uart();

  core::Result sendBytes(char *tx, std::uint8_t length);
  core::Result readBytes(unsigned char *rx, std::uint8_t length);

 private:
  Uart(core::ILogger &logger, const int file_descriptor);
  // BBB can work with baudrates from 300 to 3686400 (AM335x and AMIC110 Sitara™ manual)
  static std::optional<std::uint8_t> getBaudRateMask(const std::uint32_t baud_rate);
  // Acceptable values are 5,6,7 or 8
  static std::optional<std::uint8_t> getBitsPerByteMask(const std::uint8_t bits_per_byte);
  static core::Result configureFileForOperation(core::ILogger &logger,
                                                const int file_descriptor,
                                                const std::uint8_t baud_mask,
                                                const std::uint8_t bits_per_byte_mask);
  core::ILogger &logger_;
  const int file_descriptor_;
};

}  // namespace hyped::io