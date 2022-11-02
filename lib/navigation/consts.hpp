#include <cstdint>
#include <array>
#include "core/types.hpp"

namespace hyped::navigation{

  static constexpr core::Float kTrackLength       = 100.0; //m
  static constexpr core::Float kBrakingDistance   = 20.0;  //m
  static constexpr core::Float kPi                = 3.14159265359;
  static constexpr core::Float kWheelCicumference = kPi * 0.1; //m //TODO: check!
  static constexpr core::Float kStripeDistance    = 10.0; //m

    //inline trajectory equal to oprator for use in testing
inline bool operator==(const core::Trajectory &lhs, const core::Trajectory &rhs)
{
  return lhs.displacement == rhs.displacement && lhs.velocity == rhs.velocity
         && lhs.acceleration == rhs.acceleration;
}

//inline trajectory not equal to oprator for use in testing
inline bool operator!=(const core::Trajectory &lhs, const core::Trajectory &rhs)
{
 return !(lhs == rhs);
}

//explicitly define zero trajectory
inline core::Trajectory zero_trajectory = {0, 0, 0};

  template<typename SensorData>
  class INavigator {
  public:
    virtual core::Trajectory currentTrajectory()       = 0;
    virtual void update(const SensorData &sensor_data) = 0;
};
}