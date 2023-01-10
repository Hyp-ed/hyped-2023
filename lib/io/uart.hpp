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
  /**
   * @brief  Creates a UART object.
   * @details  Defaults to 8 bits per byte
   */
  static std::optional<Uart> create(
    core::ILogger &logger,
    const UartBus bus,
    const std::uint32_t baud_rate,  // TODOLater: Figure out a default for this by testing
    const std::uint8_t bits_per_byte = 8);
  ~Uart();

  /**
   * @brief  Sends a byte array over the UART bus.
   * @param  tx  Pointer to the byte array to be sent.
   * @param  length  Length of the byte array.
   */
  core::Result sendBytes(char *tx, std::uint8_t length);

  /**
   * @brief Receives a byte array over the UART bus.
   * @param rx Pointer to the byte array to be received.
   * @param length Length of the byte array.
   */
  core::Result readBytes(unsigned char *rx, std::uint8_t length);

 private:
  Uart(core::ILogger &logger, const int file_descriptor);
  /**
   * @brief  Returns the baud rate mask for the given baud rate.
   * @details BBB can work with baudrates from 300 to 3686400 (AM335x and AMIC110 Sitara™ manual)
   * @param baud_rate  Baud rate to be used.
   */
  static std::optional<std::uint8_t> getBaudRateMask(const std::uint32_t baud_rate);

  /**
   * @brief  Returns the bits per byte mask for the given bits per byte.
   * @details BBB can work with 5, 6, 7, 8 bits per byte
   * @param bits_per_byte  Bits per byte to be used.
   */
  static std::optional<std::uint8_t> getBitsPerByteMask(const std::uint8_t bits_per_byte);

  /**
   * @brief  Configures the UART file descriptor with the provided masks and pre-set settings
   */
  static core::Result configureFileForOperation(core::ILogger &logger,
                                                const int file_descriptor,
                                                const std::uint8_t baud_mask,
                                                const std::uint8_t bits_per_byte_mask);

 private:
  core::ILogger &logger_;
  const int file_descriptor_;
};

}  // namespace hyped::io