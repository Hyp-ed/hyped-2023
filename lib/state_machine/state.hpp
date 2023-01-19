#pragma once

namespace hyped::state_machine {

// State class containing all the states
enum class State {
  // TODO Add the actual states
  kIdle,
  kCalibrating,
  kReady,
  kAccelerating,
  kCruising,
  kBraking,
  kFailureBraking,
  kStopped,
  kFailureStopped,
  kOff
};

}  // namespace hyped::state_machine