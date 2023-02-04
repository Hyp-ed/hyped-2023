#pragma once

#include <optional>

#include <core/types.hpp>
namespace hyped::navigation {

enum class SensorDisagreement { kUnacceptable = 0, kAcceptable };

using EncoderData       = std::array<std::uint64_t, core::kNumEncoders>;
using AccelerometerData = std::array<core::Float, core::kNumAccelerometers>;
using KeyenceData       = std::array<std::uint32_t, core::kNumKeyence>;

struct Quartiles {
  core::Float q1;
  core::Float median;
  core::Float q3;
};

class INavigator {
 public:
  virtual std::optional<core::Trajectory> currentTrajectory()                           = 0;
  virtual void keyenceUpdate(const core::RawKeyenceData &keyence_data)                  = 0;
  virtual void encoderUpdate(const core::RawEncoderData &encoder_data)                  = 0;
  virtual void accelerometerUpdate(const core::RawAccelerometerData &acceleration_data) = 0;
};

}  // namespace hyped::navigation
