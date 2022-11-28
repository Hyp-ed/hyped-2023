#pragma once

#include "state.cpp"
#include "transition.cpp"
#include "consts.hpp"

#include <optional>

namespace hyped::state_machine {

class StateMachine {
 public:
  StateMachine() { current_state = State::kInitialState; };

  void transition(const Message message);

  std::optional<Message> checkTransition();

  void reset();

  // TODOLater change states to actual states
  /*
  State states[5]
    = {State::kInitialState, State::state1, State::state2, State::state3, State::state4};*/

  // TODOLater change transitions to actual transitions
  Transition transitions[kNumTransitions] = {
    Transition(State::kInitialState, State::state1, Message::mStart),
    Transition(State::state1, State::state2, Message::message1),
    Transition(State::state1, State::state3, Message::message2),
    Transition(State::state2, State::state4, Message::message2),
    Transition(State::state3, State::state4, Message::message1),
  };

  State current_state;
};
}  // namespace hyped::state_machine