#pragma once

#include "i2c_sensors.hpp"

#include <unistd.h>

#include <cstdint>
#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

constexpr std::string_view kAxisLabels[3] = {"x-axis", "y-axis", "z-axis"};

// ! these values come from the datasheet

constexpr std::uint8_t kDefaultAccelerometerAddress = 0x19;

constexpr std::uint8_t kCtrl1Address = 0x20;
// Sampling rate of 200 Hz
// Enable high performance mode
constexpr std::uint8_t kCtrl1Value = 0x64;

constexpr std::uint8_t kCtrl2Address = 0x21;
// Enable block data update
// Enable address auto increment
constexpr std::uint8_t kCtrl2Value = 0x0;

constexpr std::uint8_t kCtrl6Address = 0x25;
// Full scale +-16g
// Filter bandwidth = ODR/2
constexpr std::uint8_t kCtrl6Value = 0x30;

constexpr std::uint8_t kXOutLow  = 0x28;
constexpr std::uint8_t kXOutHigh = 0x29;

constexpr std::uint8_t kYOutLow  = 0x2A;
constexpr std::uint8_t kYOutHigh = 0x2B;

constexpr std::uint8_t kZOutLow  = 0x2C;
constexpr std::uint8_t kZOutHigh = 0x2D;

constexpr std::uint8_t kDataReady = 0x27;

constexpr std::uint8_t kDeviceIdAddress       = 0x0F;
constexpr std::uint8_t kExpectedDeviceIdValue = 0x44;

class Accelerometer : public II2cMuxSensor<core::RawAccelerationData> {
 public:
  Accelerometer(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel);
  ~Accelerometer();

  core::Result configure();
  std::optional<core::RawAccelerationData> read();
  std::uint8_t getChannel() const;

 private:
  std::optional<std::int16_t> getRawAcceleration(const core::Axis axis);
  std::int32_t getAccelerationFromRawValue(const std::int16_t rawAcceleration);
  void setRegisterAddressFromAxis(const core::Axis axis);

 private:
  core::ILogger &logger_;
  io::II2c &i2c_;
  const std::uint8_t channel_;
  std::uint8_t low_byte_address_;
  std::uint8_t high_byte_address_;
};

}  // namespace hyped::sensors