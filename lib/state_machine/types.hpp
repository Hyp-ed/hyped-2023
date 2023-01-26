#pragma once

#include "message.hpp"
#include "state.hpp"

#include <cstdint>

#include "utils/hash.hpp"

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
    std::size_t seed = 0;
    utils::hash_combine(seed, key.source);
    utils::hash_combine(seed, key.message);
    return seed;
  }
};

}  // namespace hyped::state_machine