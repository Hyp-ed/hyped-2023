#pragma once

#include <array>
#include <cstdint>

namespace hyped::core {

static constexpr float kEpsilon = 0.0001;

enum class DigitalSignal { kLow = 0, kHigh };
enum class Result { kSuccess = 0, kFailure };

using Float = float;

// current trajectory struct
struct Trajectory {
  Float displacement;
  Float velocity;
  Float acceleration;
};

// number of each type of sensors
static constexpr std::uint8_t kNumImus     = 4;
static constexpr std::uint8_t kNumAxis     = 3;
static constexpr std::uint8_t kNumEncoders = 4;
static constexpr std::uint8_t kNumKeyence  = 2;

// data format for raw sensor data
using RawImuData  = std::array<std::array<Float, kNumAxis>, kNumImus>;
using ImuData     = std::array<Float, kNumImus>;
using EncoderData = std::array<std::uint32_t, kNumEncoders>;
using KeyenceData = std::array<std::uint32_t, kNumKeyence>;

}  // namespace hyped::core
