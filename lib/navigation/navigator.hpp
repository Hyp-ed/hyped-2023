#pragma once

#include <core/types.hpp>

namespace hyped::navigation {

struct Trajectory {
  core::Displacement displacement;
  core::Velocity velocity;
  core::Acceleration acceleration;
};

inline bool operator==(const Trajectory &lhs, const Trajectory &rhs)
{
  return lhs.displacement == rhs.displacement && lhs.velocity == rhs.velocity
         && lhs.acceleration == rhs.acceleration;
}

inline Trajectory zero_trajectory = {0, 0, 0};

template<typename SensorData>
class INavigator {
 public:
  virtual Trajectory currentTrajectory()             = 0;
  virtual void update(const SensorData &sensor_data) = 0;
};

}  // namespace hyped::navigation
