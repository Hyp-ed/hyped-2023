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
  current_state = TransitionMap.at({current_state, message});
}

Message StateMachine::stringToMessage(const std::string &message_name)
{
  Message message = StringMap.at(message_name);
  return message;
}

std::string StateMachine::messageToString(const Message &message)
{
  std::string message_name = MessageMap.at(message);
  return message_name;
}

}  // namespace hyped::state_machine