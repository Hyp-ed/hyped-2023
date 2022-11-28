#pragma once

#include "i2c.hpp"

#include <core/logger.hpp>
#include <core/types.hpp>

namespace hyped::io {

class HardwareI2c : public II2c {
 public:
  HardwareI2c(core::ILogger &logger, const std::uint8_t bus_address);
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
  core::ILogger &logger_;
  std::uint8_t sensor_address_;
  int file_descriptor_;
};

}  // namespace hyped::io
