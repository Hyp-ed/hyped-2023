#pragma once

namespace hyped::state_machine {

// State class containing all the states
enum class State {
  kIdle,
  kCalibrating,
  kReady,
  kAccelerating,
  kCruising,
  kNominalBraking,
  kMotorBraking,
  kFrictionBraking,
  kFailureBraking,
  kStopped,
  kFailureStopped,
  kOff
};

}  // namespace hyped::state_machine