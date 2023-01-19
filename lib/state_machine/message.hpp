#pragma once

namespace hyped::state_machine {

// State class containing all the states
enum class Message {
  // TODOLater Add the actual messages
  mForward,
  mFailure,
  mMitigate,
  mPostEmergency,
  mResume
};

}  // namespace hyped::state_machine