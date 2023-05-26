#pragma once

#include "controller.hpp"

#include <cstdint>
#include <memory>

#include <core/types.hpp>
#include <io/hardware_can.hpp>

namespace hyped::motors {

class CanProcessor : public io::ICanProcessor {
 public:
  CanProcessor(std::shared_ptr<Controller> controller);
  core::Result processMessage(const io::CanFrame &frame);

 private:
  std::shared_ptr<Controller> controller_;
};

}  // namespace hyped::motors
