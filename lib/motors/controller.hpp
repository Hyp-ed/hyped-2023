#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <linux/can.h>

#include "core/logger.hpp"
#include "core/types.hpp"

namespace hyped::motors {
enum class ControllerStatus { kControllerTemperatureExceeded, kUnrecoverableWarning, kNominal };

class Controller {
 public:
  Controller(core::ILogger &logger);
  core::Result parseMessageFile(const std::string &path);
  void processErrorMessage(const std::uint16_t error_code);
  ControllerStatus processWarningMessage(const std::uint8_t warning_code);

 private:
  core::ILogger &logger_;
  std::unordered_map<std::string, can_frame> messages_;
  std::vector<can_frame> configuration_messages_;
};

}  // namespace hyped::motors