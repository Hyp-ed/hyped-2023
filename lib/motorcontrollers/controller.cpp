#include "controller.hpp"

namespace hyped::motorcontrollers {
Controller::Controller()
{
  void processEmergencyMessage(core::types::Canframe data) {
    //TODO: Implement
  }

  void processErrorMessage(const uint16_t error_code) {
    switch (error_code) 
    {
      case 0xFF01:
        log_.error("ERROR_CURRENT_A");
        break;
      case 0xFF02:
        log_.error("ERROR_CURRENT_B");
        break;
      case 0xFF03:
        log_.error("ERROR_HS_FET");
        break;
      case 0xFF04:
        log_.error("ERROR_LS_FET");
        break;
      case 0xFF05:
        log_.error("ERROR_DRV_LS_L1");
        break;
      case 0xFF06:
        log_.error("ERROR_DRV_LS_L2");
        break;
      case 0xFF07:
        log_.error("ERROR_DRV_LS_L3");
        break;
      case 0xFF08:
        log_.error("ERROR_DRV_HS_L1");
        break;
      case 0xFF09:
        log_.error("ERROR_DRV_HS_L2");
        break;
      case 0xFF0A:
        log_.error("ERROR_DRV_HS_L3");
        break;
      case 0xFF0B:
        log_.error("ERROR_MOTOR_FEEDBACK");
        break;
      case 0xFF0C:
        log_.error("ERROR_DC_LINK_UNDERVOLTAGE");
        break;
      case 0xFF0D:
        log_.error("ERROR_PULS_MODE_FINISHED");
        break;
      case 0xFF0E:
        log_.error("ERROR_APP_ERROR");
        break;
      case 0xFF0F:
        log_.error("ERROR_EMERGENCY_BUTTON");
        break;
      case 0xFF10:
        log_.error("ERROR_CONTROLLER_OVERTEMPERATURE");
        break;
      case 0x3210:
        log_.error("ERROR_DC_LINK_OVERVOLTAGE");
        break;
    }
  }

  void processSdoMessage(core::types::Canframe data) {
    //TODO: Implement
  }

  void processNmtMessage(core::types::Canframe data) {
    //TODO: Implement
  }
}  // namespace hyped::motorcontrollers