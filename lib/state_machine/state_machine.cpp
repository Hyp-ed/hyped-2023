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
    // TODO implement:
    void StateMachine::transition()
    {
        //takes in a state and changes the current state to that state
    }

} //namespace hyped::state_machine