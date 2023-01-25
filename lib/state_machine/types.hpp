#pragma once

#include "message.hpp"
#include "state.hpp"

namespace hyped::state_machine {

struct TransitionKey {
  State source;
  Message message;

  bool operator==(const TransitionKey &key) const
  {
    return key.source == source && key.message == message;
  }
};

struct TransitionHasher {
  std::size_t operator()(const TransitionKey &key) const
  {
    using std::size_t;

    return ((int)key.source);
  }
};

}  // namespace hyped::state_machine