#pragma once
#include <cstdint>

#include "core/logger.hpp"

namespace hyped::motors {
enum class ControllerStatus { kControllerTemperatureExceeded, kUnrecoverableWarning, kNominal };
enum class ControllerStates { kInitialisation, kCalibration, kMotorReady, kMotorAccelerating, kMotorBraking, kRunComplete, kEmergencyBreak };

class Controller {
 public:
  Controller(core::ILogger &logger);
  void processErrorMessage(const std::uint16_t error_code);
  ControllerStatus processWarningMessage(const std::uint8_t warning_code);

 private:
  core::ILogger &logger_;
};

}  // namespace hyped::motors