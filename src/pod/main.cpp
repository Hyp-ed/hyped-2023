#include <iostream>

#include <core/timer.hpp>
#include <core/types.hpp>
#include <core/wall_clock.hpp>
#include <io/gpio.hpp>
#include <core/logger.hpp>

int main(int argc, char **argv)
{
  hyped::core::WallClock time;
  hyped::core::Timer timer(time);
  const auto execution_time = timer.measure_execution_time([time]() {
    hyped::core::Logger logger("GPIO", hyped::core::LogLevel::kDebug, time);
    hyped::io::Gpio gpio(logger);
    auto gpio_reader_opt = gpio.getReader(0);
    if (!gpio_reader_opt) {
      logger.log(hyped::core::LogLevel::kFatal, "Error");
    } else if (gpio_reader_opt->read() == hyped::core::LowOrHigh::kHigh) {
      logger.log(hyped::core::LogLevel::kInfo, "High");
    } else {
      logger.log(hyped::core::LogLevel::kInfo, "Low");
    };
  });
  std::cout << "Ran for " << execution_time.count() << " ns" << std::endl;
}