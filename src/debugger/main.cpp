#include <iostream>

#include <core/logger.hpp>
#include <core/timer.hpp>
#include <core/types.hpp>
#include <core/wall_clock.hpp>
#include <io/hardware_gpio.hpp>

int main(int argc, char **argv)
{
  std::string input;
  while (1) {
    std::cout << ">> ";
    std::cin >> input;
    if (input == "exit") {
      std::cout << "Exiting..." << std::endl;
      break;
    }
    std::cout << "You entered: " << input << std::endl;
  }
}
