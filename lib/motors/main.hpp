#pragma once

namespace hyped::motors {
// Temporary states, actual states TBC.
enum class MotorStates {
  kInitialisation,
  kCalibration, // Probably nothing to be done in this state
  kReady, // Probably nothing to be done in this state
  kAcceleration, // Send motor to drive
  kNominalBreaking, // Generic breaking state
  kStopped, // Generic breaking state
  kFailureBreaking, // Generic breaking state
  kMotorBreaking, // non generic LIM breaking state
  kFrictionBreaking, // Generic breaking state
  kMitigate, // Probably nothing to be done in this state
  kPEAccelerating,
  kPECruising,
  kPEBreaking // Generic breaking state
};

class Main {
 public:
  Main();

  void run();
};

}  // namespace hyped::motors
