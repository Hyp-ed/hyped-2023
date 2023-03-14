#pragma once

namespace hyped::state_machine {

// Message class containing all the states
enum class Message {
  kNextNominalState,
  kFailure,
  kFailureBrake,
  kMotorBrake,
  kPreFrictionBrake,
  kFrictionBrake,
};

}  // namespace hyped::state_machine