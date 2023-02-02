#pragma once

#include "message.hpp"
#include "state.hpp"

#include <cstdint>

#include <boost/functional/hash.hpp>

namespace hyped::state_machine {
struct SourceAndMessage {
  State source;
  Message message;

  bool operator==(const SourceAndMessage &key) const
  {
    return key.source == source && key.message == message;
  }
};

std::size_t hash_value(SourceAndMessage const &key)
{
  std::size_t seed = 0;
  boost::hash_combine(seed, key.message);
  boost::hash_combine(seed, key.source);
  return seed;
}
}  // namespace hyped::state_machine