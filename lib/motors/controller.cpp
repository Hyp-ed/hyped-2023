#include "controller.hpp"


namespace hyped::motors {

Controller::Controller(core::ILogger &logger) : logger_(logger)
{
}

void Controller::processErrorMessage(const std::uint16_t error_code)
{
  switch (error_code) {
    case 0xFF01:
      logger_.log(core::LogLevel::kFatal,
                  "ERROR_CURRENT_A: Current phase A hall sensor missing or damaged");
      break;
    case 0xFF02:
      logger_.log(core::LogLevel::kFatal,
                  "ERROR_CURRENT_B: Current phase A hall sensor missing or damaged");
      break;
    case 0xFF03:
      logger_.log(core::LogLevel::kFatal, "ERROR_HS_FET: High side Fet short circuit");
      break;
    case 0xFF04:
      logger_.log(core::LogLevel::kFatal, "ERROR_LS_FET: Low side Fet short circuit");
      break;
    case 0xFF05:
      logger_.log(core::LogLevel::kFatal, "ERROR_DRV_LS_L1: Low side Fet phase 1 short circuit");
      break;
    case 0xFF06:
      logger_.log(core::LogLevel::kFatal, "ERROR_DRV_LS_L2: Low side Fet phase 2 short circuit");
      break;
    case 0xFF07:
      logger_.log(core::LogLevel::kFatal, "ERROR_DRV_LS_L3: Low side Fet phase 3 short circuit");
      break;
    case 0xFF08:
      logger_.log(core::LogLevel::kFatal, "ERROR_DRV_HS_L1: High side Fet phase 1 short circuit");
      break;
    case 0xFF09:
      logger_.log(core::LogLevel::kFatal, "ERROR_DRV_HS_L2: High side Fet phase 2 short circuit");
      break;
    case 0xFF0A:
      logger_.log(core::LogLevel::kFatal, "ERROR_DRV_HS_L3: High side Fet phase 3 short circuit ");
      break;
    case 0xFF0B:
      logger_.log(core::LogLevel::kFatal,
                  "ERROR_MOTOR_FEEDBACK: Wrong feedback selected (check feedback type)");
      break;
    case 0xFF0C:
      logger_.log(core::LogLevel::kFatal,
                  "ERROR_DC_LINK_UNDERVOLTAGE: DC voltage not applied to bridge or too low");
      break;
    case 0xFF0D:
      logger_.log(core::LogLevel::kFatal, "ERROR_PULS_MODE_FINISHED: Pulse mode finished");
      break;
    case 0xFF0E:
      logger_.log(core::LogLevel::kFatal, "ERROR_APP_ERROR");
      break;
    case 0xFF0F:
      logger_.log(core::LogLevel::kFatal, "ERROR_EMERGENCY_BUTTON: Emergency button pressed");
      break;
    case 0xFF10:
      logger_.log(core::LogLevel::kFatal,
                  "ERROR_CONTROLLER_OVERTEMPERATURE: Controller overtemperature");
      break;
    case 0x3210:
      logger_.log(core::LogLevel::kFatal,
                  "ERROR_DC_LINK_OVERVOLTAGE: Power supply voltage too high");
      break;
    default:  
      logger_.log(core::LogLevel::kFatal, "GENERIC_ERROR: Unspecific error occurred");
      break;
  }
}

controllerStatus Controller::processWarningMessage(const std::uint8_t warning_code)
{
  controllerStatus priority_error = controllerStatus::kNominal;

  // Flag specific warnings in binary
  if ((warning_code & 0b0000'0001) == 0b000'0000'0001) {
    logger_.log(core::LogLevel::kInfo, "Controller Temperature Exceeded");
    priority_error = controllerStatus::kControllerTemperatureExceeded;
  }
  if ((warning_code & 0b0000'0000'0010) == 0b0000'0000'0010) {
    logger_.log(core::LogLevel::kFatal, "Motor Temperature Exceeded");
    priority_error = controllerStatus::kUnrecoverableWarning;
  }
  if ((warning_code & 0b0000'0000'0100) == 0b0000'0000'0100) {
    logger_.log(core::LogLevel::kFatal, "DC link under voltage");
    priority_error = controllerStatus::kUnrecoverableWarning;
  }
  if ((warning_code & 0b0000'0000'1000) == 0b0000'0000'1000) {
    logger_.log(core::LogLevel::kFatal, "DC link over voltage");
    priority_error = controllerStatus::kUnrecoverableWarning;
  }
  if ((warning_code & 0b0000'0001'0000) == 0b0000'0001'0000) {
    logger_.log(core::LogLevel::kFatal, "DC over priority_errorent");
    priority_error = controllerStatus::kUnrecoverableWarning;
  }
  if ((warning_code & 0b0000'0010'0000) == 0b0000'0010'0000) {
    logger_.log(core::LogLevel::kFatal, "Stall protection active");
    priority_error = controllerStatus::kUnrecoverableWarning;
  }
  if ((warning_code & 0b0000'0100'0000) == 0b0000'0100'0000) {
    logger_.log(core::LogLevel::kFatal, "Max velocity exceeded");
    priority_error = controllerStatus::kUnrecoverableWarning;
  }
  if ((warning_code & 0b0000'1000'0000) == 0b0000'1000'0000) {
    logger_.log(core::LogLevel::kFatal, "BMS Proposed Power");
    priority_error = controllerStatus::kUnrecoverableWarning;
  }
  if ((warning_code & 0b0001'0000'0000) == 0b0001'0000'0000) {
    logger_.log(core::LogLevel::kFatal, "Capacitor temporature exceeded");
    priority_error = controllerStatus::kUnrecoverableWarning;
  }
  if ((warning_code & 0b0010'0000'0000) == 0b0010'0000'0000) {
    logger_.log(core::LogLevel::kFatal, "I2T protection");
    priority_error = controllerStatus::kUnrecoverableWarning;
  }
  if ((warning_code & 0b0100'0000'0000) == 0b0100'0000'0000) {
    logger_.log(core::LogLevel::kFatal, "Field weakening active");
    priority_error = controllerStatus::kUnrecoverableWarning;
  }

  return priority_error;
}
}  // namespace hyped::motors
