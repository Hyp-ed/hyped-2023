#include "temperature.hpp"

namespace hyped::sensors {

Temperature::Temperature(hyped::io::I2c &i2c, hyped::core::ILogger &log) : log_(log), i2c_(i2c)
{
}

    std::optional<uint16_t> Temperature::read() 
    {
        // TODOlater: read from i2c
        if (I2c::readByte(TEMPERATURE_1, STATUS) == 0 ){

        auto Temp_High = I2c::readByte(TEMPERATURE_1, TEMP_H);
        auto Temp_Low = I2c::readByte(TEMPERATURE_1, TEMP_L);
        // getting the high and low values from the register

        auto Temp = (TEMP_H << 8) | TEMP_L;
        Temp = Temp*0.01

        return std::optional<uint16_t>{Temp};
        //Reading from the Status register to see if the tempurature is ready to be read
        //TODO have it do the correct effect and output a log of what occurred
        };
        else{
            return std::nullopt;
        };
        //log_.log(hyped::core::LogLevel::kFatal, "Temperature read not implemented");
    }

    bool Temperature::configure()
    {

        I2c::writeByte(TEMPERATURE_1, CTRL, 1);
        // TODO We need to know if a begalebone has two registers set to it
        //Writing to the Control register 0x04 and setting that to 1 for TEMPERATURE_1


        //log_.log(hyped::core::LogLevel::kFatal, "Temperature configure not implemented");
        return true; 
    }

bool Temperature::configure()
{
  // TODOLater: configure i2c
  log_.log(hyped::core::LogLevel::kFatal, "Temperature configure not implemented");
  return false;
}

}  // namespace hyped::sensors