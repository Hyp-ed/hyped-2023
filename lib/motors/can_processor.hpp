#pragma once

#include <cstdint>

#include <core/types.hpp>
#include <io/can.hpp>

namespace hyped::motors {

class CanProcessor {
 public:
  CanProcessor();

  void processMessage(const io::CanFrame frame);
};

}  // namespace hyped::motors
