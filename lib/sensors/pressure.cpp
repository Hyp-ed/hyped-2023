#include "pressure.hpp"

namespace hyped::sensors {

std::optional<Pressure> Pressure::create(core::ILogger &logger,
                                        io::IAdc &adc)
{
  logger.log(core::LogLevel::kDebug, "Successfully created Pressure instance");
  return Pressure(logger, adc);
}

Pressure::Pressure(core::ILogger &logger, std::shared_ptr<io::IAdc> adc)
    : adc_(adc),
      logger_(logger)
{
}

Pressure::~Pressure()
{
}

std::uint8_t Pressure::getPressure()
{
  const auto optionalResult = adc.readValue();
  if (!optionalResult) {
    logger_.log(core::LogLevel::kFatal, "Failue for Pressure sensor to read adc value");
  }

  return *optionalResult;
}

}  // namespace hyped::sensors