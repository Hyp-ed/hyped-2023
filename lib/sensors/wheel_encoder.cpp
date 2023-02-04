#include "WheelEncoder.hpp"

namespace hyped::sensors {

// TODOLater: here finish the implementation once other i2c instance has been made
std::optional<WheelEncoder> WheelEncoder::create(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel)
{
}

WheelEncoder::WheelEncoder(core::ILogger &logger, io::II2c &i2c, const std::uint8_t channel)
    : logger_(logger),
      i2c_(i2c),
      channel_(channel)
{
}

WheelEncoder::~WheelEncoder()
{
}

std::uint8_t WheelEncoder::getWheelTurnCount()
{
}

void WheelEncoder::resetWheelTurnCount()
{
}

}  // namespace hyped::sensors