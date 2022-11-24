#include "state_machine.hpp"

namespace hyped::state_machine {

StateMachine::StateMachine()
{

}

// TODOLater implement
void StateMachine::checkTransition()
{
  /*
  update data
  should check if a transition is needed with current data
  then run transition to move to next state
  return (None || transition_message)
  */
}

//Transition to next state
void StateMachine::transition(Message message)
{
  for (Transition transition : transitions) {
    if (transition.from == current_state && transition.message == message) {
      current_state = transition.to;
    }
  }
}

void StateMachine::reset()
{
  current_state = states[0];
}

} //namespace hyped::state_machine