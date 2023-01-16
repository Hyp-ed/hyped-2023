#pragma once

#include "message.hpp"
#include "state.hpp"

namespace hyped::state_machine {

struct Transition {
  State from;
  State to;
  Message message;
};

}  // namespace hyped::state_machine