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
  std::array<Transition, kNumTransitions> transitions{{
    {State::kIdle, State::kCalibrating, Message::mCalibrate},
    {State::kCalibrating, State::kReady, Message::mReady},
    {State::kReady, State::kAccelerating, Message::mAccelerating},
    {State::kAccelerating, State::kCruising, Message::mCruising},
    {State::kCruising, State::kBraking, Message::mBraking},
    {State::kBraking, State::kStopped, Message::mStoppped},
    {State::kStopped, State::kOff, Message::mOff},
  }};

  State current_state;
};
}  // namespace hyped::state_machine