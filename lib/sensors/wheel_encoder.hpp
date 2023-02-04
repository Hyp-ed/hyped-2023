#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <io/gpio.hpp>

namespace hyped::sensors {
class WheelEncoder {
 public:
  static std::optional<WheelEncoder> create(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel);
  ~WheelEncoder();

  std::uint24_t getWheelTurnCount();
  void resetWheelTurnCount();

 private:
  WheelEncoder(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel);

 private:
  core::ILogger &logger_;
  io::II2c &i2c_;
  const std::uint8_t channel_;
};

}  // namespace hyped::sensors