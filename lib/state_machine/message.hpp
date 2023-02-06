#pragma once

namespace hyped::state_machine {

// State class containing all the states
enum class Message {
  mForward,
  mFailure
};

}  // namespace hyped::state_machine