#pragma once

#include <cstdint>

#include <core/types.hpp>
#include <io/hardware_can.hpp>

namespace hyped::motors {

class CanProcessor : public io::ICanProcessor {
 public:
  CanProcessor();

  void processMessage(const io::CanFrame &frame);
};

}  // namespace hyped::motors
