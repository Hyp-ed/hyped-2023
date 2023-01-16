#pragma once

#include <cstdint>
#include <cstdio>
#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>

namespace hyped::io {

class I2c {
 public:
  I2c(const std::uint8_t bus_address, core::ILogger &log);
  ~I2c();

  /**
   * @brief      Reads a byte from some device on the I2C bus.
   */
  std::optional<std::uint8_t> readByte(const std::uint8_t device_address,
                                       const std::uint8_t register_address);
  /**
   * @brief      General function to write a byte to a register to some device on the I2C bus
   */
  core::Result writeByteToRegister(const std::uint8_t device_address,
                                   const std::uint8_t register_address,
                                   const std::uint8_t data);
  /**
   * @brief      Writes a byte to single register devices such as the mux
   */
  core::Result writeByte(const std::uint8_t device_address, std::uint8_t data);

 private:
  void setSensorAddress(const std::uint8_t device_address);

 private:
  int file_descriptor_;
  std::uint8_t sensor_address_;
  core::ILogger &log_;
};

}  // namespace hyped::io
