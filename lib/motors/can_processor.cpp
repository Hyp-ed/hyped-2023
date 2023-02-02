#include "can_processor.hpp"
#include "controller.hpp"
#include <cstdint>
#include <iostream>
#include <climits>



namespace hyped::motors {

uint64_t little_endian_array_to_big_endian_int(const uint8_t little_endian_array[8]) {
    return (little_endian_array[7]<<56|little_endian_array[6]<<48|little_endian_array[5]<<40|little_endian_array[4]<<32|little_endian_array[3] << 24) | (little_endian_array[2] << 16) | (little_endian_array[1] << 8) | little_endian_array[0];
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

//Process little endian to big endian


void CanProcessor::processMessage(const core::CanFrame frame)
{


  if (frame.can_id == 0x80) {
    if (frame.data) {
      controller_->processErrorMessage(little_endian_array_to_big_endian_int(frame.data[0]));
    }else {
      controller_->processWarningMessage(little_endian_array_to_big_endian_int(frame.data[1]));
    }

  }
  else {

  }
  
}

}  // namespace hyped::motors
