#include "can_processor.hpp"
#include "controller.hpp"

#include <climits>
#include <cstdint>
#include <iostream>

namespace hyped::motors {

uint64_t CanProcessor::little_endian_array_to_big_endian_int(const uint8_t little_endian_array[8])
{
  return (static_cast<uint64_t>(little_endian_array[7]) << 56 | static_cast<uint64_t>(little_endian_array[6]) << 48
          | static_cast<uint64_t>(little_endian_array[5]) << 40 | static_cast<uint64_t>(little_endian_array[4]) << 32
          | (static_cast<uint64_t>(little_endian_array[3]) << 24)
         |(static_cast<uint64_t>(little_endian_array[2]) << 16) | (static_cast<uint64_t>(little_endian_array[1]) << 8) | static_cast<uint64_t>(little_endian_array[0]));
}

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
    // TODO: convert frame.data into index, subindex and data 
    if (frame.data) {
      controller_->processErrorMessage(little_endian_array_to_big_endian_int(frame.data[0]));
    } else {
      controller_->processWarningMessage(little_endian_array_to_big_endian_int(frame.data[1]));
    }

  } else {
    // TODO: Implement
  }
}

}  // namespace hyped::motors
