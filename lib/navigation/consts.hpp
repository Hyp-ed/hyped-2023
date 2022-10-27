#include <cstdint>

namespace hyped::navigation{

  using nav_t = float;
  static constexpr nav_t kTrackLength       = 100.0; //m
  static constexpr nav_t kBrakingDistance   = 20.0;  //m
  static constexpr uint8_t kNumberImus      = 4;
  static constexpr uint8_t kNumberEncoders  = 4;
  static constexpr uint8_t kNumberKeyence   = 2;
  static constexpr nav_t kPi                = 3.14159265359;
  static constexpr nav_t kWheelCicumference = kPi * 0.1; //m //TODO: check!
  static constexpr nav_t kStripeDistance    = 10.0; //m
  
}