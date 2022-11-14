#include <iostream>

#include <core/logger.hpp>
#include <core/timer.hpp>
#include <core/types.hpp>
#include <core/wall_clock.hpp>
#include <io/hardware_gpio.hpp>

int main(int argc, char **argv)
{
  hyped::core::WallClock time;
  hyped::core::Timer timer(time);
  hyped::core::Logger logger("GPIO", hyped::core::LogLevel::kDebug, time);
  hyped::io::HardwareGpio gpio(logger);
  auto gpio_reader_opt = gpio.getReader(0);
  if (!gpio_reader_opt) {
    logger.log(hyped::core::LogLevel::kFatal, "Error");
  } else {
    auto gpio_reader = *gpio_reader_opt;
    if (gpio_reader->read() == hyped::core::DigitalSignal::kHigh) {
      logger.log(hyped::core::LogLevel::kInfo, "High");
    } else {
      logger.log(hyped::core::LogLevel::kInfo, "Low");
    };
  }
  std::cout << "Ran for " << timer.elapsed().count() << " ns" << std::endl;
}
