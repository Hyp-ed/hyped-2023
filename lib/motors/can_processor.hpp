#pragma once

#include "controller.hpp"

#include <cstdint>
#include <memory>

#include <core/types.hpp>

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
