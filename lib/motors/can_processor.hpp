#pragma once

#include <cstdint>

#include <core/types.hpp>

namespace hyped::motors {
// every frame sent to the controller contains 8 bytes
static constexpr std::uint8_t kControllerCanFrameLength = 8;

class CanProcessor {
 public:
  CanProcessor();

  bool sendMessage(const core::CanFrame frame);
  void processMessage(const core::CanFrame frame);
};

}  // namespace hyped::motors
