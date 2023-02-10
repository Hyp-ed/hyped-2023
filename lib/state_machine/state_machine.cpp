#include "state_machine.hpp"

namespace hyped::state_machine {

// TODOLater implement
std::optional<Message> StateMachine::checkTransition()
{
  /*
  check if a transition is needed with current data from ros node
  then run transition to move to next state
  return transition message or no transition
  */
  return std::nullopt;
}

// Transition to next state
void StateMachine::handleMessage(const Message message)
{
  current_state = transition_to_state_.at({current_state, message});
}

Message StateMachine::stringToMessage(const std::string &message_name)
{
  return string_to_message_.at(message_name);
}

std::string StateMachine::messageToString(const Message &message)
{
  return message_to_string_.at(message);
}

}  // namespace hyped::state_machine