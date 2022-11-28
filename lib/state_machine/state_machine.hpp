#pragma once

#include "consts.hpp"
#include "state.hpp"
#include "types.hpp"

#include <array>
#include <optional>

namespace hyped::state_machine {

class StateMachine {
 public:
  StateMachine() { current_state = State::kInitialState; };

  void transition(const Message message);

  Message checkTransition();

  void reset();

  // TODOLater change transitions to actual transitions
  std::array<Transition, kNumTransitions> transitions{{
    {State::kInitialState, State::state1, Message::mStart},
    {State::state1, State::state2, Message::message1},
    {State::state1, State::state3, Message::message2},
    {State::state2, State::state4, Message::message2},
    {State::state3, State::state4, Message::message1},
  }};

  State current_state;
};
}  // namespace hyped::state_machine