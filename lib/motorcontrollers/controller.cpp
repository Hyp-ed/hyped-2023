#include "controller.hpp"

namespace hyped::motorcontrollers {
Controller::Controller()
{
void processEmergencyMessage(core::types::Canframe data) {
    //TODO: Implement
}

void processErrorMessage(core::types::Canframe data) {
    //TODO: Implement
}

void processSdoMessage(core::types::Canframe data) {
    switch(data) {
        case 1:
            log_.error("Controller temperature exceeded");
            break;
        case 2:
            log_.error("Motor temperature exceeded");
            break;
        case 4:
            log_.error("DC link under voltage");
            break;
        case 8:
            log_.error("DC link over voltage");
            break;
        case 16:
            log_.error("DC link over current");
            break;
        case 32:
            log_.error("Stall protection active");
            break;
        case 64:
            log_.error("Max velocity exceeded");
            break;
        case 128:
            log_.error("BMS proposed power");
            break;
        case 256:
            log_.error("Capacitor temperature exceeded");
            break;
        case 512:
            log_.error("I2T protection");
            break;
        case 1024:
            log_.error("Field weakening active");
            break;
        default:
            log_.error("Unknown Error");

    }

void processNmtMessage(core::types::Canframe data) {
    //TODO: Implement
}
}  // namespace hyped::motorcontrollers