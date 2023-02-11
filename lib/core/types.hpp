#pragma once

#include "time.hpp"

#include <array>
#include <cstdint>

namespace hyped::core {

static constexpr float kEpsilon = 0.0001;

enum class DigitalSignal { kLow = 0, kHigh };
enum class Result { kSuccess = 0, kFailure };

using Float = float;

struct CanFrame {
  std::uint32_t can_id; /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  std::uint8_t can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
  std::uint8_t __pad;   /* padding */
  std::uint8_t __res0;  /* reserved / padding */
  std::uint8_t __res1;  /* reserved / padding */
  std::array<std::uint8_t, 8> data;
};

struct Trajectory {
  Float displacement;
  Float velocity;
  Float acceleration;
};

// number of each type of sensors
static constexpr std::uint8_t kNumAccelerometers = 4;
static constexpr std::uint8_t kNumAxis           = 3;
static constexpr std::uint8_t kNumEncoders       = 4;
static constexpr std::uint8_t kNumKeyence        = 2;

template<typename T>
struct Measurement {
  Measurement(const TimePoint &measured_at, const T &value) : measured_at(measured_at), value(value)
  {
  }

  TimePoint measured_at;
  T value;
};

struct RawAcceleration {
  std::int32_t x;  // mg
  std::int32_t y;  // mg
  std::int32_t z;  // mg
};
using RawAccelerometerData         = Measurement<RawAcceleration>;
using CombinedRawAccelerometerData = Measurement<std::array<RawAcceleration, kNumAccelerometers>>;
using RawEncoderData               = Measurement<std::array<std::uint32_t, kNumEncoders>>;
using RawKeyenceData               = Measurement<std::array<std::uint32_t, kNumKeyence>>;

enum class Axis { kX = 0, kY, kZ };

}  // namespace hyped::core
