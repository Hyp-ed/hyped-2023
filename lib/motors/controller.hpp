#pragma once
#include <cstdint>

#include "core/logger.hpp"

namespace hyped::motors {
enum class controllerStatus { kControllerTemperatureExceeded, kUnrecoverableWarning, kNominal };

class Controller {
 public:
  Controller(core::ILogger &logger);
  void processErrorMessage(const std::uint16_t error_code);
  controllerStatus processWarningMessage(const std::uint8_t warning_code);

 private:
  core::ILogger &logger_;
};

}  // namespace hyped::motors