#pragma once

#include "state.hpp"

namespace hyped::state_machine {

    // Transition class.
    // Each transition has a current state and next state, as well as a message indicating the kind of transition.
    class Transition
    {
        State from;
        State to;
        Message message;
    };
};