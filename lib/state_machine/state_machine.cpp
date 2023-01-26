#include "state_machine.hpp"
#include "types.hpp"

namespace hyped::state_machine {

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
  current_state = transitions.at({current_state, message});
}

void StateMachine::reset()
{
  current_state = State::kIdle;
}

}  // namespace hyped::state_machine