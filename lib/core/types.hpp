#pragma once

#include <array>
#include <cstdint>
#include <chrono>

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

// data produced by the accelerometer sensor
struct acceleration_struct {

  acceleration_struct(core::Float x, core::Float y, core::Float z, std::chrono::high_resolution_clock::time_point time) : x(x), y(y), z(z), time(time) {}

  core::Float x;
  core::Float y;
  core::Float z;
  std::chrono::high_resolution_clock::time_point time;
};

}  // namespace hyped::core
