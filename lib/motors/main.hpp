#pragma once

namespace hyped::motors {
  // Temporary states, actual states TBC.
enum class MotorStates { kInitialisation, kCalibration, kMotorReady, kMotorAccelerating, kMotorBraking, kRunComplete, kEmergencyBreak };

class Main {
 public:
  Main();

  void run();
};

}  // namespace hyped::motors
