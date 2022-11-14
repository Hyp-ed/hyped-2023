#pragma once
#include <optional>

#include <core/types.hpp>
namespace hyped::navigation {

enum class SensorDisagreement { kUnacceptable = 0, kAcceptable };

using EncoderData = std::array<std::uint64_t, core::kNumEncoders>;
using ImuData     = std::array<core::Float, core::kNumImus>;
using KeyenceData = std::uint64_t;

class INavigator {
 public:
  virtual std::optional<core::Trajectory> currentTrajectory()          = 0;
  virtual void keyenceUpdate(const core::RawKeyenceData &keyence_data) = 0;
  virtual void encoderUpdate(const core::RawEncoderData &encoder_data) = 0;
  virtual void imuUpdate(const core::RawImuData &imu_data)             = 0;
};

}  // namespace hyped::navigation
