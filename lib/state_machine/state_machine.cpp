#include "state_machine.hpp"
#include "types.hpp"

#include <optional>

namespace hyped::state_machine {

StateMachine::StateMachine()
{
}

// TODOLater implement
std::optional<Message> StateMachine::handleData()
{
  /*
  check if a transition is needed with current data
  then run transition to move to next state
  return transition message or no transition
  */
  return std::nullopt;
}

// Transition to next state
void StateMachine::handleMessage(const Message message)
{
  for (Transition transition : transitions) {
    if (transition.source == current_state && transition.message == message) {
      current_state = transition.target;
    }
  }
}

void StateMachine::reset()
{
  current_state = State::kIdle;
}

}  // namespace hyped::state_machine