#pragma once

#include "controller.hpp"

#include <cstdint>
#include <memory>

#include <core/types.hpp>

namespace hyped::motors {

class CanProcessor {
 public:
  CanProcessor(std::shared_ptr<Controller> controller);
  uint64_t little_endian_array_to_big_endian_int(const std::uint8_t little_endian_array[8]);
  bool sendMessage(const core::CanFrame frame);
  void processMessage(const core::CanFrame frame);

 private:
  std::shared_ptr<Controller> controller_;
};

}  // namespace hyped::motors
