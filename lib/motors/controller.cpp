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
      logger_.log(core::LogLevel::kFatal,
                  "GENERIC_ERROR: Unspecific error occurred with code %i",
                  error_code);
      break;
  }
}

ControllerStatus Controller::processWarningMessage(const std::uint8_t warning_code)
{
  ControllerStatus priority_error = ControllerStatus::kNominal;

  // In the event any warning are found, print entire error code.
  if (warning_code != 0) {
    logger_.log(core::LogLevel::kInfo, "Controller Error found, (code: %x)", warning_code);
  } else {
    return priority_error;
  }

  // In the event some warning have occured, print and return highest priority.
  if (warning_code & 0x1) {
    logger_.log(core::LogLevel::kInfo, "Controller Warning: Controller Temperature Exceeded");
    priority_error = ControllerStatus::kControllerTemperatureExceeded;
  }
  if (warning_code & 0x2) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: Motor Temperature Exceeded");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x4) {
    logger_.log(
      core::LogLevel::kFatal, "Controller Warning: DC link under voltage");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x8) {
    logger_.log(
      core::LogLevel::kFatal, "Controller Warning: DC link over voltage");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x10) {
    logger_.log(
      core::LogLevel::kFatal, "Controller Warning: DC link over current");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x20) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: Stall protection active");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x40) {
    logger_.log(
      core::LogLevel::kFatal, "Controller Warning: Max velocity exceeded");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x80) {
    logger_.log(
      core::LogLevel::kFatal, "Controller Warning: BMS Proposed Power");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x100) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: Capacitor temperature exceeded");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x200) {
    logger_.log(
      core::LogLevel::kFatal, "Controller Warning: I2T protection");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }
  if (warning_code & 0x400) {
    logger_.log(core::LogLevel::kFatal, "Controller Warning: Field weakening active");
    priority_error = ControllerStatus::kUnrecoverableWarning;
  }

  return priority_error;
}
}  // namespace hyped::motors