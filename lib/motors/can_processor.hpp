#pragma once

#include <cstdint>

#include <core/types.hpp>
#include "controller.hpp"

namespace hyped::motors {

class CanProcessor {
 public:
  CanProcessor(std::shared_ptr<Controller> controller);
  bool sendMessage(const core::CanFrame frame);
  void processMessage(const core::CanFrame frame);
 private: 
  std::shared_ptr<Controller> controller_;

};

}  // namespace hyped::motors
