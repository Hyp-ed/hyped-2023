#include "state_machine.hpp"

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
  current_state = transitionMap.at({current_state, message});
}

}  // namespace hyped::state_machine