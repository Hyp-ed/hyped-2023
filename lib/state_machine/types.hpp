#pragma once

#include "message.cpp"
#include "state.cpp"

#include <cstdint>

#include "utils/hash.hpp"

namespace hyped::state_machine {
struct SourceAndMessage {
  State source;
  Message message;

  bool operator==(const SourceAndMessage &key) const
  {
    return key.source == source && key.message == message;
  }
};
}  // namespace hyped::state_machine

namespace std {
template<>
struct hash<hyped::state_machine::SourceAndMessage> {
  std::size_t operator()(const hyped::state_machine::SourceAndMessage &c) const
  {
    std::size_t result = 0;
    hyped::utils::hash_combine(result, c.source);
    hyped::utils::hash_combine(result, c.message);
    return result;
  }
};
}  // namespace std