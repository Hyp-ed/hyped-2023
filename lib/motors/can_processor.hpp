#pragma once

#include <cstdint>

#include <core/types.hpp>

namespace hyped::motors {

class CanProcessor {
 public:
  CanProcessor();

  bool sendMessage(const core::CanFrame frame);
  void processMessage(const core::CanFrame frame);
};

}  // namespace hyped::motors
