#include "repl.hpp"

#include <sstream>

namespace hyped::debug {
Repl::Repl(hyped::core::ILogger &log) : log_(log)
{
  addQuitCommand();
  addHelpCommand();
}

void Repl::run()
{
  while (true) {
    handleCommand();
  }
}

void Repl::printCommands()
{
  log_.log(hyped::core::LogLevel::kInfo, "Available commands:");
  for (const auto &[name, command] : command_map_) {
    log_.log(hyped::core::LogLevel::kInfo, "  %s: %s", name.c_str(), command.description.c_str());
  }
}

void Repl::handleCommand()
{
  std::cout << "> ";
  std::string command;
  std::getline(std::cin, command);
  auto nameAndCommand = command_map_.find(command);
  if (nameAndCommand == command_map_.end()) {
    log_.log(hyped::core::LogLevel::kFatal, "Unknown command: %s", command.c_str());
    return;
  }
  nameAndCommand->second.handler();
}

void Repl::addCommand(const Command &cmd)
{
  command_map_.emplace(cmd.name, cmd);
  log_.log(hyped::core::LogLevel::kDebug, "Added command: %s", cmd.name.c_str());
}

void Repl::addQuitCommand()
{
  addCommand(Command{"quit", "Quit the REPL", [this]() { exit(0); }});
}

void Repl::addHelpCommand()
{
  addCommand(Command{"help", "Print this help message", [this]() { printCommands(); }});
}

void Repl::addAdcCommands(const uint8_t pin)
{
  const auto adc = std::make_shared<hyped::io::Adc>(pin, log_);
  {
    Command adc_read_command;
    std::stringstream identifier;
    identifier << "adc " << static_cast<int>(pin) << " read";
    adc_read_command.name = identifier.str();
    std::stringstream description;
    description << "Read from ADC pin " << static_cast<int>(pin);
    adc_read_command.description = description.str();
    adc_read_command.handler     = [this, adc]() {
      const auto value = adc->readValue();
      if (value) { log_.log(hyped::core::LogLevel::kInfo, "ADC value: %d", *value); }
    };
    addCommand(adc_read_command);
  }
}

void Repl::addI2cCommands(const uint8_t channel)
{
  const auto i2c = std::make_shared<hyped::io::I2c>(channel, log_);
  {
    Command i2c_read_command;
    std::stringstream identifier;
    identifier << "i2c " << static_cast<int>(channel) << " read";
    i2c_read_command.name = identifier.str();
    std::stringstream description;
    description << "Read from I2C channel " << static_cast<int>(channel);
    i2c_read_command.description = description.str();
    i2c_read_command.handler     = [this, i2c]() {
      uint8_t device_address, register_address;
      std::cout << "Device address: ";
      std::cin >> device_address;
      std::cout << "Register address: ";
      std::cin >> register_address;
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      const auto value = i2c->readByte(device_address, register_address);
      if (value) { log_.log(hyped::core::LogLevel::kInfo, "I2C value: %d", *value); }
    };
    addCommand(i2c_read_command);
  }
  {
    Command i2c_write_command;
    std::stringstream identifier;
    identifier << "i2c " << static_cast<int>(channel) << " write";
    i2c_write_command.name = identifier.str();
    std::stringstream description;
    description << "Write to I2C channel " << static_cast<int>(channel);
    i2c_write_command.description = description.str();
    i2c_write_command.handler     = [this, i2c]() {
      uint32_t device_address, register_address, data;
      std::cout << "Device address: ";
      std::cin >> std::hex >> device_address;
      std::cout << "Register address: ";
      std::cin >> std::hex >> register_address;
      std::cout << "Data: ";
      std::cin >> std::hex >> data;
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      const hyped::io::I2cWriteResult result
        = i2c->writeByte(device_address, register_address, data);
      if (result == hyped::io::I2cWriteResult::kSuccess) {
        log_.log(hyped::core::LogLevel::kInfo, "I2C value %d written succesfully", data);
      }
    };
    addCommand(i2c_write_command);
  }
}
}  // namespace hyped::debug