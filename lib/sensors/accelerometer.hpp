#pragma once

#include "i2c_sensors.hpp"

#include <unistd.h>

#include <cstdint>
#include <optional>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

enum class Axis { x = 0, y, z };
static constexpr char AxisStrings[3][10] = {"x-axis", "y-axis", "z-axis"};

// ! these values come from the datasheet

static constexpr std::uint8_t kDefaultAccelerometerAddress = 0x19;

static constexpr std::uint8_t kCtrl1Address = 0x20;
// Sampling rate of 200 Hz
// Enable high performance mode
static constexpr std::uint8_t kCtrl1Value = 0x64;

static constexpr std::uint8_t kCtrl2Address = 0x21;
// Enable block data update
// Enable address auto increment
static constexpr std::uint8_t kCtrl2Value = 0x0;

static constexpr std::uint8_t kCtrl6Address = 0x25;
// Full scale +-16g
// Filter bandwidth = ODR/2
static constexpr std::uint8_t kCtrl6Value = 0x30;

static constexpr std::uint8_t kXOutLow  = 0x28;
static constexpr std::uint8_t kXOutHigh = 0x29;

static constexpr std::uint8_t kYOutLow  = 0x2A;
static constexpr std::uint8_t kYOutHigh = 0x2B;

static constexpr std::uint8_t kZOutLow  = 0x2C;
static constexpr std::uint8_t kZOutHigh = 0x2D;

static constexpr std::uint8_t kDataReady = 0x27;

static constexpr std::uint8_t kDeviceId         = 0x0F;
static constexpr std::uint8_t kExpectedDeviceId = 0x44;

class Accelerometer : public II2cMuxSensor<core::RawAccelerationData> {
 public:
  Accelerometer(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel);
  ~Accelerometer();

  core::Result configure();
  std::optional<core::RawAccelerationData> read();
  std::uint8_t getChannel() const;

 private:
  std::optional<std::int16_t> getRawAcceleration(const Axis axis);
  std::int32_t getAccelerationFromRawValue(const std::int16_t rawAcceleration);
  void setRegisterAddressFromAxis(const Axis axis);

 private:
  core::ILogger &logger_;
  io::II2c &i2c_;
  const std::uint8_t channel_;
  std::uint8_t low_byte_address_;
  std::uint8_t high_byte_address_;
};

}  // namespace hyped::sensors