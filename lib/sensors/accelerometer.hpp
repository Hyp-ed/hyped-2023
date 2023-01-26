#pragma once

#include "i2c_sensors.hpp"

#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <optional>

#include "core/types.hpp"
#include <core/logger.hpp>
#include <io/i2c.hpp>
#include <io/hardware_i2c.hpp>

namespace hyped::sensors {

static constexpr std::uint8_t kCTRL1Addr        = 0x20;
static constexpr std::uint8_t kCTRL1Value       = 0x64;

static constexpr std::uint8_t kCTRL2Addr        = 0x21;
static constexpr std::uint8_t kCTRL2Value        = 0x0;

static constexpr std::uint8_t kCTRL6Addr        = 0x25;
static constexpr std::uint8_t kCTRL6Value        = 0x30;

static constexpr std::uint8_t kXOutL        = 0x28;
static constexpr std::uint8_t kXOutH        = 0x29;

static constexpr std::uint8_t kYOutL        = 0x2A;
static constexpr std::uint8_t kYOutH        = 0x2B;

static constexpr std::uint8_t kZOutL        = 0x2C;
static constexpr std::uint8_t kZOutH        = 0x2D;

static constexpr std::uint8_t kDataReady    = 0x27;

static constexpr std::uint8_t kDevId        = 0x0F;
static constexpr std::uint8_t expectedDevId = 0x44;

class Accelerometer : II2cMuxSensor<core::RawAccelerationData> {
 public:
  Accelerometer(const std::uint8_t kDeviceAddr,
                const std::uint8_t channel,
                io::HardwareI2c &i2c,
                core::ILogger &log);
  ~Accelerometer();

  core::Result configure();
  std::optional<core::RawAccelerationData> read();
  std::uint8_t getChannel();

 private:
  core::ILogger &log_;
  io::HardwareI2c &i2c_;
  const std::uint8_t kDeviceAddr;
  const std::uint8_t channel_;

  std::optional<std::int16_t> getRawAccelerationX();
  std::optional<std::int16_t> getRawAccelerationY();
  std::optional<std::int16_t> getRawAccelerationZ();
};

}  // namespace hyped::sensors