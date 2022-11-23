#pragma once

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

static constexpr std::uint8_t kNumImus     = 4;
static constexpr std::uint8_t kNumAxis     = 3;
static constexpr std::uint8_t kNumEncoders = 4;
static constexpr std::uint8_t kNumKeyence  = 2;

using RawAccelerationData     = std::array<std::array<Float, kNumAxis>, kNumImus>;
using RawEncoderData = std::array<std::uint32_t, kNumEncoders>;
using RawKeyenceData = std::array<std::uint32_t, kNumKeyence>;

}  // namespace hyped::core
