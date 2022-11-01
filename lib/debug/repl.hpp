#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>

#include <core/logger.hpp>
#include <io/adc.hpp>
#include <io/i2c.hpp>

namespace hyped::debug {

struct Command {
  std::string name;
  std::string description;
  std::function<void(void)> callback;
};

class Repl {
 public:
  Repl(hyped::core::ILogger &log);
  void run();

 private:
  void printCommands();
  void addCommand(const Command &cmd);
  void addQuitCommand();
  void addHelpCommand();
  void handleCommand();

  void addAdcCommands(const uint8_t pin);

  std::vector<Command> commands_;
  hyped::core::ILogger &log_;
  std::unordered_map<std::string, Command> command_map_;
};
}  // namespace hyped::debug
