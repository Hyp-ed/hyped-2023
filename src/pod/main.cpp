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
  const auto execution_time = timer.measure_execution_time([]() {
    hyped::utils::Logger log("GPIO", hyped::utils::Level::kDebug);
    hyped::io::Gpio gpio(log);
    auto gpio_reader_opt = gpio.getReader(0);
    if (!gpio_reader_opt) {
      std::cout << "Error" << std::endl;
    } else if (gpio_reader_opt->read() == hyped::core::LowOrHigh::kHigh) {
      std::cout << "Pin is high" << std::endl;
    } else {
      std::cout << "Pin is low" << std::endl;
    }
  });
  std::cout << "Ran for " << execution_time.count() << " ns" << std::endl;
}