#include "can_processor.hpp"
#include "controller.hpp"
#include "intrin.h"

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

void CanProcessor::processMessage(const core::CanFrame frame)
{
  uint16_t = frame.data;

  if (frame.can_id == 0x580) {
    if (frame.data) {
      controller_->processErrorMessage(frame.data);
    }else {
      controller_->processWarningMessage(frame.data);
    }

  }
  else {

  }
  
}

}  // namespace hyped::motors
