#pragma once

#include "state.hpp"

namespace hyped::state_machine {

    // Transition class.
    // Each transitions has a current state and next state
    class Transition
    {
        State curr_state;
        State next_state;
        
    };
};