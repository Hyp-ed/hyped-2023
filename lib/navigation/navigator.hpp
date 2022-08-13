#pragma once

#include <core/types.hpp>

namespace hyped::navigator {

struct Trajectory {
  core::Displacement displacement;
  core::Velocity velocity;
  core::Acceleration acceleration;
};

inline Trajectory zero_trajectory = {0, 0, 0};

template<typename SensorData>
class INavigator {
 public:
  virtual Trajectory currentTrajectory()             = 0;
  virtual void update(const SensorData &sensor_data) = 0;
};

}  // namespace hyped::navigator
