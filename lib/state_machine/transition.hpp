#pragma once

#include "state.hpp"
#include "message.cpp"

namespace hyped::state_machine {

    // Transition class.
    // Each transition has a current state and next state, as well as a message indicating the kind of transition.
    class Transition
    {
        public:
        Transition(State from_state, State to_state, Message transition_message)
        {
            from = from_state;
            to = to_state;
            message = transition_message;
        };
        State from;
        State to;
        Message message;
    };

};