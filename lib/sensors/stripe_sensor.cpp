#include "stripe_sensor.hpp"

namespace hyped::sensors {

    Sensor::Sensor(const std::uint8_t newPin){
        pin = newPin;
        HardwareGpio sensor(&log_);
        sensor.getReader(newPin);
    };
    
    int Sensor::getStripeCount(){
        return stripeCount;
    }

    void Sensor::updateStripes(){
        if(sensor.read() == core::DigitalSignal::kHigh){
            stripeCount++;
        };
    }

}
