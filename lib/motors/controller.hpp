#pragma once
#include <cstdint>

#include "core/logger.hpp"

namespace hyped::motors {
enum class ControllerStatus { kControllerTemperatureExceeded, kUnrecoverableWarning, kNominal };
enum class ControllerState {
  koperationalState,
  kpreOperationalState,
  kstopState,
  kresetNodeState,
  kresetCommunicationState
};

class Controller {
 public:
  Controller(core::ILogger &logger);
  void processErrorMessage(const std::uint16_t error_code);
  ControllerStatus processWarningMessage(const std::uint8_t warning_code);
  ControllerState processNMTMessage(const std::uint8_t nmt_code);
  ControllerState getControllerState();

 private:
  float temperature;
  float current;
  ControllerState state;
  core::ILogger &logger_;
};

}  // namespace hyped::motors