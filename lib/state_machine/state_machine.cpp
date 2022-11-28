#include "state_machine.hpp"
#include "types.hpp"

#include <optional>

namespace hyped::state_machine {

StateMachine::StateMachine()
{
}

// TODOLater implement
Message StateMachine::checkTransition()
{
  /*
  update data
  should check if a transition is needed with current data
  then run transition to move to next state
  return transition message
  */
  return Message::noneTransition;
}

// Transition to next state
void StateMachine::transition(const Message message)
{
  for (Transition transition : transitions) {
    if (transition.from == current_state && transition.message == message) {
      current_state = transition.to;
    }
  }
}

void StateMachine::reset()
{
  current_state = State::kInitialState;
}

}  // namespace hyped::state_machine