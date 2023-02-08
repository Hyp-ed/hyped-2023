#pragma once

#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <core/logger.hpp>
#include <io/hardware_adc.hpp>
#include <io/hardware_gpio.hpp>
#include <io/hardware_i2c.hpp>
#include <io/hardware_spi.hpp>
#include <io/hardware_uart.hpp>
#include <io/pwm.hpp>
#include <sensors/accelerometer.hpp>

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
  void addPwmCommands(const std::uint8_t module);
  void addSpiCommands(const std::uint8_t bus);
  void addAccelerometerCommands(const std::uint8_t bus, const std::uint8_t device_address);
  void addUartCommands(const std::uint8_t bus);

  std::optional<std::shared_ptr<io::II2c>> getI2C(const std::uint8_t bus);

  core::ILogger &logger_;
  std::map<std::string, Command> command_map_;
  std::unordered_map<std::uint8_t, std::shared_ptr<io::II2c>> i2c_;
};

}  // namespace hyped::debug