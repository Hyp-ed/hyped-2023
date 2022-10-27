#pragma once

#include <array>
#include <cstdint>

namespace hyped::motorcontrollers {

using ControllerMessage = std::array<uint8_t, 8>;

// TODO add needed messages in below format
static constexpr ControllerMessage kExampleMessage
  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

}  // namespace hyped::motorcontrollers