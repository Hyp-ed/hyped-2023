#include "can_processor.hpp"
#include "controller.hpp"

#include <climits>
#include <cstdint>
#include <iostream>

namespace hyped::motors {

CanProcessor::CanProcessor(std::shared_ptr<Controller> controller)
{
  controller_ = controller;
}

bool CanProcessor::sendMessage(const core::CanFrame frame)
{
  // TODOLater implement
  return false;
}

// Process little endian to big endian

void CanProcessor::processMessage(const core::CanFrame frame)
{
  if (frame.can_id == 0x80) {
    // process Emergency message
    std::uint32_t data = (static_cast<std::uint64_t>(frame.data[7]) << 24)
                         | (static_cast<std::uint64_t>(frame.data[6]) << 16)
                         | (static_cast<std::uint64_t>(frame.data[5]) << 8)
                         | static_cast<std::uint64_t>(frame.data[4]);
    controller_->processErrorMessage(data);
  } else if (frame.can_id == 0x580) {
    // process SDO frame
    // TODO: convert frame.data into index, subindex and data

    // Retrieve data and index from can frame
    std::uint16_t SDORef = (static_cast<std::uint64_t>(frame.data[1]) << 8)
                           | static_cast<std::uint64_t>(frame.data[0]);
    std::uint32_t data = (static_cast<std::uint64_t>(frame.data[7]) << 24)
                         | (static_cast<std::uint64_t>(frame.data[6]) << 16)
                         | (static_cast<std::uint64_t>(frame.data[5]) << 8)
                         | static_cast<std::uint64_t>(frame.data[4]);

    // Process data from can frame
    if ((SDORef == 0x603f) && frame.data[2] == 0x00) {
      // Error message received
      controller_->processErrorMessage(data);
    } else if ((SDORef == 0x2027) && frame.data[2] == 0x00) {
      // Warning message received
      controller_->processWarningMessage(data);
    }
  } else if (frame.can_id == 0x700) {
    // TODO: handle NMT frame
  } else {
    // TODO: Implement
  }
}

}  // namespace hyped::motors
