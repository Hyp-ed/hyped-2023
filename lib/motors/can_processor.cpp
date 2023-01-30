#include "can_processor.hpp"
#include "controller.hpp"

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
  
}

}  // namespace hyped::motors
