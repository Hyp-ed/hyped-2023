#pragma once

#include <cstdint>
#include <cstdio>
#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>

namespace hyped::io {

class I2c {
 public:
  I2c(const std::uint8_t bus_address, hyped::core::ILogger &log);
  ~I2c();

  std::optional<std::uint8_t> readByte(const std::uint8_t device_address,
                                       const std::uint8_t register_address);
  hyped::core::Result writeByte(const std::uint8_t device_address,
                                const std::uint8_t register_address,
                                const std::uint8_t data);

 private:
  void setSensorAddress(const std::uint8_t device_address);

 private:
  int file_descriptor_;
  std::uint8_t sensor_address_;
  hyped::core::ILogger &log_;
};

}  // namespace hyped::io
