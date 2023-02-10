#pragma once

namespace hyped::state_machine {

// State class containing all the states
enum class Message {
  kNextNominalState,
  kFailure,
  kFailureBrake,
  kMotorBrake,
  kFrictionBrake,
  kNominalBrake
};

}  // namespace hyped::state_machine