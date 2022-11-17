#pragma once

#include "state.cpp"
#include "transition.cpp"

namespace hyped::state_machine {

    class StateMachine {
        public:
        StateMachine();

        void transition();

        void checkTransition();

        // TODO add state field:
        //requires State be implemented
        //State current_state = State::kInitialState
        
    };
} //namespace hyped::state_machine