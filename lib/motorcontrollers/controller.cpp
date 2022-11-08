#include "controller.hpp"
#include <vector>

namespace hyped::motorcontrollers {

  

Controller::Controller(core::ILogger &logger) : logger_(logger))
{
  //TODO: Implement Enum for warning codes
  // Implement function to iterate through warning codes given a warning bit selection
  // and output a vector of enums
  enum warnings {ControllerTemperatureExceeded, MotorTemperatureExceeded, DCLinkUnderVoltage, 
                  DCLinkOverVoltage, DCLinkOverCurrent, StallProtectionActive, 
                  MaxVelocityExceeded, BMSProposedPower, CapacitorTemperatureExceeded, 
                  I2TProtection, FieldWeakeningActive};
                  
  void processErrorMessage(const uint16_t error_code){

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

  std::vector<warnings> processWarningMessage(const uint8_t warning_code){

    std::vector<warnings> flagged_warnings;

    //Flag specific warnings in binary
    if ((warning_code & 0b0000'0001) == 0b0000'0001){
      flagged_warnings.push_back(warnings(0));
    }
    if ((warning_code & 0b0000'0000'0010) == 0b0000'0000'0010)
    {
      flagged_warnings.push_back(warnings(1));
    }
    if ((warning_code & 0b0000'0000'0100) == 0b0000'0000'0100)
    {
      flagged_warnings.push_back(warnings(2));
    }
    if ((warning_code & 0b0000'0000'1000) == 0b0000'0000'1000)
    {
      flagged_warnings.push_back(warnings(3));
    }
    if ((warning_code & 0b0000'0001'0000) == 0b0000'0001'0000)
    {
      flagged_warnings.push_back(warnings(4));
    }
    if ((warning_code & 0b0000'0010'0000) == 0b0000'0010'0000)
    {
      flagged_warnings.push_back(warnings(5));
    }
    if ((warning_code & 0b0000'0100'0000) == 0b0000'0100'0000)
    {
      flagged_warnings.push_back(warnings(6));
    }
    if ((warning_code & 0b0000'1000'0000) == 0b0000'1000'0000)
    {
      flagged_warnings.push_back(warnings(7));
    }
    if ((warning_code & 0b0001'0000'0000) == 0b0001'0000'0000)
    {
      flagged_warnings.push_back(warnings(8));
    }
    if ((warning_code & 0b0010'0000'0000) == 0b0010'0000'0000)
    {
      flagged_warnings.push_back(warnings(9));
    }
    if ((warning_code & 0b0100'0000'0000) == 0b0100'0000'0000)
    {
      flagged_warnings.push_back(warnings(10));
    }

    return flagged_warnings;
  }
}  // namespace hyped::motorcontrollers

