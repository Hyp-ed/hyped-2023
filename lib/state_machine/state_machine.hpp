#include "state.cpp"
#include "transition.cpp"

namespace hyped::state_machine {

    class StateMachine {
        public:
        StateMachine();

        void transition();

        void checkTransition();

        //TODO: add fields to hold the current state, states and transitions
        struct states
        {
            //requires State be implemented
            //State::a
        };

        struct transitions
        {
          //requires Transition be implemented
          //Transition::b 
        };

        //requires State be implemented
        //State current_state
        
    };
} //namespace hyped::state_machine