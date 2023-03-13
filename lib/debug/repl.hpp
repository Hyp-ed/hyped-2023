#pragma once

#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include <core/logger.hpp>
#include <io/hardware_adc.hpp>
#include <io/hardware_gpio.hpp>
#include <io/hardware_i2c.hpp>
#include <io/hardware_spi.hpp>
#include <io/hardware_uart.hpp>
#include <io/pwm.hpp>
#include <sensors/accelerometer.hpp>
#include <sensors/temperature.hpp>

namespace hyped::debug {

struct Command {
  std::string name;
  std::string description;
  std::function<void(void)> handler;
};

class Repl {
 public:
  Repl(core::ILogger &logger);
  void run();
  std::optional<std::unique_ptr<Repl>> fromFile(const std::string &filename);

 private:
  void printCommands();
  void handleCommand();
  void addCommand(const Command &cmd);

  void addQuitCommand();
  void addHelpCommand();
  void addAdcCommands(const std::uint8_t pin);
  void addI2cCommands(const std::uint8_t bus);
  void addPwmCommands(const std::uint8_t module, const std::uint32_t period);
  void addSpiCommands(const std::uint8_t bus);
  void addAccelerometerCommands(const std::uint8_t bus, const std::uint8_t device_address);
  void addTemperatureCommands(const std::uint8_t bus, const std::uint8_t device_address);
  void addUartCommands(const std::uint8_t bus);

  core::ILogger &logger_;
  std::map<std::string, Command> command_map_;
};

}  // namespace hyped::debug