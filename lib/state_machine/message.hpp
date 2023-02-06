#pragma once

namespace hyped::state_machine {

// State class containing all the states
enum class Message {
  mForward,
  mFailure,
  mFailureBrake,
  mMotorBrake,
  mFrictionBrake,
  mNominalBrake
};

}  // namespace hyped::state_machine