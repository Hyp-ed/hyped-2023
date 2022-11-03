#include <array>
#include <cstdint>

#include "core/types.hpp"

namespace hyped::navigation {

static constexpr core::Float kTrackLength       = 100.0;  // m
static constexpr core::Float kBrakingDistance   = 20.0;   // m TODOLater:check!
static constexpr core::Float kPi                = 3.14159265359;
static constexpr core::Float kWheelCicumference = kPi * 0.1;  // m TODOLater: check!
static constexpr core::Float kStripeDistance    = 10.0;       // m TODOLater:check!

// define sensor checks return type
enum class SensorChecks { kUnacceptable = 0, kAcceptable };

// explicitly define zero trajectory
inline core::Trajectory zero_trajectory = {0, 0, 0};

//TODOLater fix this
template<typename SensorData>
class INavigator {
 public:
  virtual core::Trajectory currentTrajectory()       = 0;
  virtual void update(const SensorData &sensor_data) = 0;
};
}  // namespace hyped::navigation