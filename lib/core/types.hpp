#pragma once
#include <array>
#include <cstdint>
namespace hyped::core {

static constexpr float kEpsilon = 0.0001;

enum class DigitalSignal { kLow = 0, kHigh };

using Float = float;

struct CanFrame {
  uint32_t can_id; /* 32 bit CAN_ID + EFF/RTR/ERR flags */
  uint8_t can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
  uint8_t __pad;   /* padding */
  uint8_t __res0;  /* reserved / padding */
  uint8_t __res1;  /* reserved / padding */
  std::array<uint8_t, 8> data;
};

// current trajectory struct
struct Trajectory {
  Float displacement;
  Float velocity;
  Float acceleration;
};

// number of each type of sensors
static constexpr uint8_t kNumImus     = 4;
static constexpr uint8_t kNumAxis  = 3;
static constexpr uint8_t kNumEncoders = 4;
static constexpr uint8_t kNumKeyence  = 2;

// data format for raw sensor data
using RawImuData  = std::array<std::array<Float, kNumAxis>, kNumImus>;
using ImuData     = std::array<Float, kNumImus>;
using EncoderData = std::array<uint32_t, kNumEncoders>;
using KeyenceData = std::array<uint32_t, kNumKeyence>;

}  // namespace hyped::core
