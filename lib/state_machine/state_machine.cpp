#include "state_machine.hpp"

#include <iostream>

namespace hyped::state_machine {

StateMachine::StateMachine() : current_state_{State::kIdle}
{
}

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
void StateMachine::handleMessage(const Message &message)
{
  const auto transition = transition_to_state_.find({current_state_, message});
  if (transition != transition_to_state_.end()) { current_state_ = transition->second; }
}

Message StateMachine::stringToMessage(const std::string &message_name)
{
  return string_to_message_.at(message_name);
}

std::string StateMachine::messageToString(const Message &message)
{
  return message_to_string_.at(message);
}

State StateMachine::getCurrentState()
{
  return current_state_;
}

}  // namespace hyped::state_machine