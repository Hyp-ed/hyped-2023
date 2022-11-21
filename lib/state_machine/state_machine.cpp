#include "state_machine.hpp"


namespace hyped::state_machine {

    StateMachine::StateMachine()
    {

    }

    //Check if the current states transition function returns true
    // TODOLater implement:
    void StateMachine::checkTransition()
    {
        //update data
        //should check if a transition is needed with current data
        //then run transition to move to next state
    }

    //Transition to next state
    void StateMachine::transition(Message message)
    {
        for (Transition transition : transitions) {
            if (transition.from == current_state && transition.message == message) {
                current_state = transition.to;
                return;
            }
        }
    }

} //namespace hyped::state_machine