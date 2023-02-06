#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <core/logger.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

// ! these values still have to be found
static constexpr std::uint8_t kDefaultWheelEncoderAddress = 0x0;

static constexpr std::uint8_t low_byte_address_ = 0x0;
static constexpr std::uint8_t middle_byte_address_ = 0x0;
static constexpr std::uint8_t high_byte_address_ = 0x0;

static constexpr std::uint8_t kFreeRegisterAddress = 0x0;
static constexpr std::uint8_t kFreeRegisterValue = 0x2;



class WheelEncoder {
 public:
  static std::optional<WheelEncoder> create(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel);
  ~WheelEncoder();

  std::optional<std::uint32_t> getWheelTurnCount();
  core::Result resetWheelTurnCount();

 private:
  WheelEncoder(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel);

 private:
  core::ILogger &logger_;
  io::II2c &i2c_;
  const std::uint8_t channel_;
};

}  // namespace hyped::sensors