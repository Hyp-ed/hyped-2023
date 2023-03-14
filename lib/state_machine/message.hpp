#pragma once

namespace hyped::state_machine {

// Message class containing messages that prompt transitions
enum class Message {
  kNextNominalState,
  kFailure,
  kFailureBrake,
  kMotorBrake,
  kPreFrictionBrake,
  kFrictionBrake,
};

}  // namespace hyped::state_machine