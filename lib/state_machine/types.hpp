#pragma once

#include "message.hpp"
#include "state.hpp"

namespace hyped::state_machine {

struct Transition {
  State source;
  State target;
  Message message;
};

}  // namespace hyped::state_machine