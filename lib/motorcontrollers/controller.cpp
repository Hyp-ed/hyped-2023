#include "controller.hpp"

namespace hyped::motorcontrollers {
Controller::Controller(core::ILogger &logger) : logger_(logger))
{
  //TODO: Implement Enum for warning codes
  // Implement function to iterate through warning codes given a warning bit selection
  // and output a vector of enums

  void processErrorMessage(const uint16_t error_code) {
    switch (error_code) 
    {
      case 0xFF01:
        logger_.log("ERROR_CURRENT_A: Current phase A hall sensor missing or damaged");
        break;
      case 0xFF02:
        logger_.log("ERROR_CURRENT_B: Current phase A hall sensor missing or damaged");
        break;
      case 0xFF03:
        logger_.log("ERROR_HS_FET: High side Fet short circuit");
        break;
      case 0xFF04:
        logger_.log("ERROR_LS_FET: Low side Fet short circuit");
        break;
      case 0xFF05:
        logger_.log("ERROR_DRV_LS_L1: Low side Fet phase 1 short circuit");
        break;
      case 0xFF06:
        logger_.log("ERROR_DRV_LS_L2: Low side Fet phase 2 short circuit");
        break;
      case 0xFF07:
        logger_.log("ERROR_DRV_LS_L3: Low side Fet phase 3 short circuit");
        break;
      case 0xFF08:
        logger_.log("ERROR_DRV_HS_L1: High side Fet phase 1 short circuit");
        break;
      case 0xFF09:
        logger_.log("ERROR_DRV_HS_L2: High side Fet phase 2 short circuit");
        break;
      case 0xFF0A:
        logger_.log("ERROR_DRV_HS_L3: High side Fet phase 3 short circuit ");
        break;
      case 0xFF0B:
        logger_.log("ERROR_MOTOR_FEEDBACK: Wrong feedback selected (check feedback type)");
        break;
      case 0xFF0C:
        logger_.log("ERROR_DC_LINK_UNDERVOLTAGE: DC voltage not applied to bridge or to low");
        break;
      case 0xFF0D:
        logger_.log("ERROR_PULS_MODE_FINISHED: Puls mode finished");
        break;
      case 0xFF0E:
        logger_.log("ERROR_APP_ERROR");
        break;
      case 0xFF0F:
        logger_.log("ERROR_EMERGENCY_BUTTON: Emergency button pressed");
        break;
      case 0xFF10:
        logger_.log("ERROR_CONTROLLER_OVERTEMPERATURE: Controller overtemperature");
        break;
      case 0x3210:
        logger_.log("ERROR_DC_LINK_OVERVOLTAGE: Power supply voltage tophigh");
        break;
      default:
        logger_.log("GENERIC_ERROR: Unspecific error occurred");
        break;
    }
  }
}  // namespace hyped::motorcontrollers
