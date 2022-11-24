#pragma once

#include "i2c.hpp"

#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>

namespace hyped::io {

class HardwareI2c : public II2c {
 public:
  HardwareI2c(const std::uint8_t bus_address, core::ILogger &log);
  ~HardwareI2c();

  virtual std::optional<std::uint8_t> readByte(const std::uint8_t device_address,
                                               const std::uint8_t register_address);
  virtual core::Result writeByteToRegister(const std::uint8_t device_address,
                                           const std::uint8_t register_address,
                                           const std::uint8_t data);
  virtual core::Result writeByte(const std::uint8_t device_address, const std::uint8_t data);

 private:
  void setSensorAddress(const std::uint8_t device_address);

 private:
  int file_descriptor_;
  std::uint8_t sensor_address_;
  core::ILogger &log_;
};

}  // namespace hyped::io
