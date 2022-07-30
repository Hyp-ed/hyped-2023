#include <iostream>

#include <core/types.hpp>
#include <io/gpio.hpp>

int main(int argc, char **argv)
{
  hyped::io::Gpio gpio;
  auto gpio_reader_opt = gpio.getReader(0);
  if (!gpio_reader_opt) {
    std::cout << "Error" << std::endl;
    return -1;
  }
  if (gpio_reader_opt->read() == hyped::core::LowOrHigh::kHigh) {
    std::cout << "Pin is high" << std::endl;
  } else {
    std::cout << "Pin is low" << std::endl;
  }
}