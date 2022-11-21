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
        //return (None || transition_message)
    }

    //Transition to next state
    // TODO implement:
    void StateMachine::transition()
    {
        //make the corresponding transition based on the transition message and the current state
        //for (int i = 0; i < length(self.transitions; i++)) {
        //  if (transitions[i].message == message && transitions[i].start == self.state) {
        //      self.state = transitions[i].end;
        //  }
        //}
    }

} //namespace hyped::state_machine