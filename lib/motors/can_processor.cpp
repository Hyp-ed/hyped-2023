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
  if (frame.can_id == 0x580) {
    // TODO: convert frame.data into index, subindex and data 
   
   //Retrieve data and index from can frame
    uint16_t SDORef =  (static_cast<uint64_t>(frame.data[1]) << 8) | static_cast<uint64_t>(frame.data[0]);
    uint32_t data = (static_cast<uint64_t>(frame.data[7]) << 24) | (static_cast<uint64_t>(frame.data[6]) << 16) | (static_cast<uint64_t>(frame.data[5]) << 8) | static_cast<uint64_t>(frame.data[4]);
    
    //Process data from can frame
    if ((SDORef  == 0x603f) && frame.data[2] == 0x00) {
      //Error message received
      controller_->processErrorMessage(data);
    } else if ((SDORef  == 0x2027) && frame.data[2] == 0x00){
            //Warning message received
      controller_->processWarningMessage(data);
    } 
  

  } else {
    // TODO: Implement
  }
}

}  // namespace hyped::motors
