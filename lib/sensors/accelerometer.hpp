#pragma once

#include "i2c_sensors.hpp"

#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>

#include <core/logger.hpp>
#include <core/types.hpp>
#include <io/hardware_i2c.hpp>
#include <io/i2c.hpp>

namespace hyped::sensors {

enum Axis { x, y, z };
static const std::array<std::string, 3> AxisStrings{"x-axis", "y-axis", "z-axis"};

static constexpr std::uint8_t kDeviceAddress = 0x19;

static constexpr std::uint8_t kCtrl1Address = 0x20;
static constexpr std::uint8_t kCtrl1Value   = 0x64;

static constexpr std::uint8_t kCtrl2Address = 0x21;
static constexpr std::uint8_t kCtrl2Value   = 0x0;

static constexpr std::uint8_t kCtrl6Address = 0x25;
static constexpr std::uint8_t kCtrl6Value   = 0x30;

static constexpr std::uint8_t kXOutLow  = 0x28;
static constexpr std::uint8_t kXOutHigh = 0x29;

static constexpr std::uint8_t kYOutLow  = 0x2A;
static constexpr std::uint8_t kYOutHigh = 0x2B;

static constexpr std::uint8_t kZOutLow  = 0x2C;
static constexpr std::uint8_t kZOutHigh = 0x2D;

static constexpr std::uint8_t kDataReady = 0x27;

static constexpr std::uint8_t kDeviceId         = 0x0F;
static constexpr std::uint8_t kExpectedDeviceId = 0x44;

class Accelerometer : II2cMuxSensor<core::RawAccelerationData> {
 public:
  Accelerometer(core::ILogger &logger, io::HardwareI2c &i2c, const std::uint8_t channel);
  ~Accelerometer();

  virtual core::Result configure();
  virtual std::optional<core::RawAccelerationData> read();
  virtual std::uint8_t getChannel();

 private:
  core::ILogger &logger_;
  io::HardwareI2c &i2c_;
  const std::uint8_t channel_;

  std::optional<std::int16_t> getRawAcceleration(Axis axis);
  std::int16_t getAccelerationFromRaw(std::int16_t rawAcc);
};

}  // namespace hyped::sensors