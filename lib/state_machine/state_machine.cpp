#include "state_machine.hpp"

namespace hyped::state_machine {

StateMachine::StateMachine() : current_state_{State::kIdle}, previous_message_{Message::kNone}
{
}

// TODOLater implement
void StateMachine::checkTransition(const Message &message)
{
  /*
  check if a transition is needed with current data from ros node
  then run transition to move to next state
  return transition message or no transition
  */
  previous_message_ = message;
  if (message != Message::kNone) {
    /*
    return the boolean returned by handleMessage so that we can check if a transition was made
    this allows us to only publish successful transitions and avoid echo chamber effect
    */
    handleMessage(message);
  }
}

// Transition to next state
bool StateMachine::handleMessage(const Message &message)
{
  previous_message_     = message;
  const auto transition = transition_to_state_.find({current_state_, message});
  // if there is a transition, make it and return true
  if (transition != transition_to_state_.end()) {
    current_state_ = transition->second;
    return true;
  }
  return false;
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

Message StateMachine::getPreviousMessage()
{
  return previous_message_;
}

}  // namespace hyped::state_machine