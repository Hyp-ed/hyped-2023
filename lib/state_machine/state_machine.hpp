#pragma once

#include "consts.hpp"
#include "state.hpp"
#include "types.hpp"

#include <array>
#include <optional>

namespace hyped::state_machine {

class StateMachine {
 public:
  StateMachine() { current_state = State::kIdle; };

  void transition(const Message message);

  std::optional<Message> checkTransition();

  void reset();

  // TODO change transitions to actual transitions
  std::array<Transition, kNumTransitions> transitions{
    {{State::kIdle, State::kCalibrating, Message::mForward},
     {State::kCalibrating, State::kReady, Message::mForward},
     {State::kReady, State::kAccelerating, Message::mForward},
     {State::kAccelerating, State::kCruising, Message::mForward},
     {State::kCruising, State::kBraking, Message::mForward},
     {State::kBraking, State::kStopped, Message::mForward},
     {State::kStopped, State::kOff, Message::mForward},
     {State::kIdle, State::kFailureStopped, Message::mFailure},
     {State::kCalibrating, State::kFailureStopped, Message::mFailure},
     {State::kReady, State::kFailureStopped, Message::mFailure},
     {State::kAccelerating, State::kFailureBraking, Message::mFailure},
     {State::kCruising, State::kFailureBraking, Message::mFailure},
     {State::kBraking, State::kFailureBraking, Message::mFailure},
     {State::kStopped, State::kFailureStopped, Message::mFailure},
     {State::kFailureBraking, State::kFailureStopped, Message::mFailure}}};

  State current_state;
};
}  // namespace hyped::state_machine