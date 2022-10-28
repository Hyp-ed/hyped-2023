#include <cstdint>


namespace hyped::navigation{

  using nav_t = float;
  static constexpr nav_t kTrackLength       = 100.0; //m
  static constexpr nav_t kBrakingDistance   = 20.0;  //m
  static constexpr uint8_t kNumImus         = 4;
  static constexpr uint8_t kNumEncoders     = 4;
  static constexpr uint8_t kNumKeyence      = 2;
  static constexpr nav_t kPi                = 3.14159265359;
  static constexpr nav_t kWheelCicumference = kPi * 0.1; //m //TODO: check!
  static constexpr nav_t kStripeDistance    = 10.0; //m
  
  //current trajectory struct
  struct Trajectory {
  nav_t displacement;
  nav_t velocity;
  nav_t acceleration;
};

//inline trajectory equal to oprator for use in testing
inline bool operator==(const Trajectory &lhs, const Trajectory &rhs)
{
  return lhs.displacement == rhs.displacement && lhs.velocity == rhs.velocity
         && lhs.acceleration == rhs.acceleration;
}


//inline trajectory not equal to oprator for use in testing
inline bool operator!=(const Trajectory &lhs, const Trajectory &rhs)
{
 return !(lhs == rhs);
}

//explicitly define zero trajectory
inline Trajectory zero_trajectory = {0, 0, 0};

template<typename SensorData>
class INavigator {
 public:
  virtual Trajectory currentTrajectory()             = 0;
  virtual void update(const SensorData &sensor_data) = 0;
};
}