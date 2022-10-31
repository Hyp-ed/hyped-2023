#pragma once

#include <cstdint>

#include <core/types.hpp>
#include <io/can.hpp>

namespace hyped::motors {

class CanProcessor {
 public:
  CanProcessor();

  bool sendMessage(const CanFrame frame);
  void processMessage(const CanFrame frame);
};

}  // namespace hyped::motors
