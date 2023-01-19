#pragma once

namespace hyped::state_machine {

// State class containing all the states
enum class State {
  // TODO Add the actual states
  kIdle,
  kCalibrating,
  kReady,
  kMitigate,
  kAccelerating,
  kPEAccelerating,
  kCruising,
  kPECruising,
  kNominalBraking,
  kMotorBraking,
  kFrictionBraking,
  kPEBraking,
  kFailureBraking,
  kStopped,
  kFailureStopped,
  kOff
};

}  // namespace hyped::state_machine