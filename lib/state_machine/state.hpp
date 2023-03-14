#pragma once

namespace hyped::state_machine {

enum class State {
  kIdle,
  kCalibrating,
  kReady,
  kAccelerating,
  kCruising,
  kMotorBraking,
  kPreFrictionBraking,
  kFrictionBraking,
  kFailureBraking,
  kStopped,
  kFailureStopped,
  kOff
};

}  // namespace hyped::state_machine