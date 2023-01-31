#pragma once

#include <core/types.hpp>

namespace hyped::io {
// every frame sent to the controller contains 8 bytes
static constexpr std::uint8_t kControllerCanFrameLength = 8;

class Can {
 public:
  Can();
};

}  // namespace hyped::io
