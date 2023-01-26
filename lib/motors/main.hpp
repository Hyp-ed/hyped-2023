#pragma once

namespace hyped::motors {
  // Temporary states, actual states TBC.
enum class MotorStates { kInitialisation, kCalibration, kReady, kAcceleration, kNominalBreaking, kStopped, kFailureBreaking, kMotorBreaking, kFrictionBreaking, kMitigate, kPEAccelerating, kPECruising, kPEBreaking };

class Main {
 public:
  Main();

  void run();
};

}  // namespace hyped::motors
