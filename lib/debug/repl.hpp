#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <core/logger.hpp>
#include <io/adc.hpp>
#include <io/hardware_gpio.hpp>
#include <io/i2c.hpp>

namespace hyped::debug {

struct Command {
  std::string name;
  std::string description;
  std::function<void(void)> handler;
};

class Repl {
 public:
  Repl(hyped::core::ILogger &log);
  void run();
  std::optional<std::unique_ptr<Repl>> fromFile(const std::string &filename);

 private:
  void printCommands();
  void handleCommand();
  void addCommand(const Command &cmd);

  void addQuitCommand();
  void addHelpCommand();
  void addAdcCommands(const uint8_t pin);
  void addI2cCommands(const uint8_t channel);

  hyped::core::ILogger &log_;
  std::map<std::string, Command> command_map_;
};
}  // namespace hyped::debug
