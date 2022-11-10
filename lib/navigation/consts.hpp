#include <array>
#include <cstdint>
#include <optional>

#include "core/types.hpp"

namespace hyped::navigation {

static constexpr core::Float kTrackLength       = 100.0;  // m
static constexpr core::Float kBrakingDistance   = 20.0;   // m TODOLater:check!
static constexpr core::Float kPi                = 3.14159265359;
static constexpr core::Float kWheelCicumference = kPi * 0.1;  // m TODOLater: check!
static constexpr core::Float kStripeDistance    = 10.0;       // m TODOLater:check!

// define sensor checks return type
enum class SensorChecks { kUnacceptable = 0, kAcceptable };

struct Quartiles {
  core::Float q1;
  core::Float median;
  core::Float q3;
};

inline core::Trajectory zero_trajectory = {0, 0, 0};

class INavigator {
 public:
  virtual std::optional<core::Trajectory> currentTrajectory()       = 0;
  virtual void keyenceUpdate(const core::KeyenceData &keyence_data) = 0;
  virtual void encoderUpdate(const core::EncoderData &encoder_data) = 0;
  virtual void imuUpdate(const core::RawImuData &imu_data)          = 0;
};
}  // namespace hyped::navigation