#include "repl.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/istreamwrapper.h>

namespace hyped::debug {

Repl::Repl(core::ILogger &logger) : logger_(logger)
{
}

void Repl::run()
{
  while (true) {
    handleCommand();
  }
}

std::optional<std::unique_ptr<Repl>> Repl::fromFile(const std::string &path)
{
  std::ifstream input_stream(path);
  if (!input_stream.is_open()) {
    logger_.log(core::LogLevel::kFatal, "Failed to open file %s", path.c_str());
    return std::nullopt;
  }

  rapidjson::IStreamWrapper input_stream_wrapper(input_stream);
  rapidjson::Document document;
  rapidjson::ParseResult result = document.ParseStream(input_stream_wrapper);
  if (!result) {
    logger_.log(core::LogLevel::kFatal,
                "Error parsing JSON: %s",
                rapidjson::GetParseError_En(document.GetParseError()));
    return std::nullopt;
  }

  if (!document.HasMember("debugger")) {
    logger_.log(core::LogLevel::kFatal,
                "Missing required field 'debugger' in configuration file at %s",
                path.c_str());
    return std::nullopt;
  }
  const auto debugger = document["debugger"].GetObject();
  auto repl           = std::make_unique<Repl>(logger_);
  repl->addHelpCommand();
  repl->addQuitCommand();

  if (!debugger.HasMember("io")) {
    logger_.log(core::LogLevel::kFatal,
                "Missing required field 'debugger.io' in configuration file at %s",
                path.c_str());
    return std::nullopt;
  }
  const auto io = debugger["io"].GetObject();

  if (!io.HasMember("adc")) {
    logger_.log(core::LogLevel::kFatal, "Missing required field 'io.adc' in configuration file");
    return std::nullopt;
  }
  const auto adc = io["adc"].GetObject();
  if (!adc.HasMember("enabled")) {
    logger_.log(core::LogLevel::kFatal,
                "Missing required field 'io.adc.enabled' in configuration file");
    return std::nullopt;
  }
  if (adc["enabled"].GetBool()) {
    if (!adc.HasMember("pins")) {
      logger_.log(core::LogLevel::kFatal,
                  "Missing required field 'io.adc.pins' in configuration file");
      return std::nullopt;
    }
    const auto pins = adc["pins"].GetArray();
    for (auto &pin : pins) {
      repl->addAdcCommands(pin.GetUint());
    }
  }

  if (!io.HasMember("i2c")) {
    logger_.log(core::LogLevel::kFatal, "Missing required field 'io.i2c' in configuration file");
    return std::nullopt;
  }
  const auto i2c = io["i2c"].GetObject();
  if (!i2c.HasMember("enabled")) {
    logger_.log(core::LogLevel::kFatal,
                "Missing required field 'io.i2c.enabled' in configuration file");
    return std::nullopt;
  }
  if (i2c["enabled"].GetBool()) {
    if (!i2c.HasMember("buses")) {
      logger_.log(core::LogLevel::kFatal,
                  "Missing required field 'io.i2c.buses' in configuration file");
      return std::nullopt;
    }
  }
  const auto buses = i2c["buses"].GetArray();
  for (auto &bus : buses) {
    repl->addI2cCommands(bus.GetUint());
  }

  if (!io.HasMember("spi")) {
    logger_.log(core::LogLevel::kFatal, "Missing required field 'io.spi' in configuration file");
    return std::nullopt;
  }
  const auto spi = io["spi"].GetObject();
  if (!spi.HasMember("enabled")) {
    logger_.log(core::LogLevel::kFatal,
                "Missing required field 'io.spi.enabled' in configuration file");
    return std::nullopt;
  }
  if (spi["enabled"].GetBool()) {
    if (!spi.HasMember("buses")) {
      logger_.log(core::LogLevel::kFatal,
                  "Missing required field 'io.spi.buses' in configuration file");
      return std::nullopt;
    }
    const auto buses = spi["buses"].GetArray();
    for (auto &bus : buses) {
      repl->addI2cCommands(bus.GetUint());
    }
  }
  return repl;
}

void Repl::printCommands()
{
  logger_.log(core::LogLevel::kInfo, "Available commands:");
  for (const auto &[name, command] : command_map_) {
    logger_.log(core::LogLevel::kInfo, "  %s: %s", name.c_str(), command.description.c_str());
  }
}

void Repl::handleCommand()
{
  std::cout << "> ";
  std::string command;
  std::getline(std::cin, command);
  auto nameAndCommand = command_map_.find(command);
  if (nameAndCommand == command_map_.end()) {
    logger_.log(core::LogLevel::kFatal, "Unknown command: %s", command.c_str());
    return;
  }
  nameAndCommand->second.handler();
}

void Repl::addCommand(const Command &cmd)
{
  command_map_.emplace(cmd.name, cmd);
  logger_.log(core::LogLevel::kDebug, "Added command: %s", cmd.name.c_str());
}

void Repl::addQuitCommand()
{
  addCommand(Command{"quit", "Quit the REPL", [this]() { exit(0); }});
}

void Repl::addHelpCommand()
{
  addCommand(Command{"help", "Print this help message", [this]() { printCommands(); }});
}

void Repl::addAdcCommands(const std::uint8_t pin)
{
  const auto optional_adc = io::Adc::create(logger_, pin);
  if (!optional_adc) {
    logger_.log(core::LogLevel::kFatal, "Failed to create ADC instance on pin %d", pin);
    return;
  }
  const auto adc = std::make_shared<io::Adc>(*optional_adc);
  Command adc_read_command;
  std::stringstream identifier;
  identifier << "adc " << static_cast<int>(pin) << " read";
  adc_read_command.name = identifier.str();
  std::stringstream description;
  description << "Read from ADC pin " << static_cast<int>(pin);
  adc_read_command.description = description.str();
  adc_read_command.handler     = [this, adc, pin]() {
    const auto value = adc->readValue();
    if (value) {
      logger_.log(core::LogLevel::kInfo, "ADC value from pin %d: %d", pin, *value);
    } else {
      logger_.log(core::LogLevel::kFatal, "Failed to read from ADC pin %d", pin);
    }
  };
  addCommand(adc_read_command);
}

void Repl::addI2cCommands(const std::uint8_t bus)
{
  const auto optional_i2c = io::HardwareI2c::create(logger_, bus);
  if (!optional_i2c) {
    logger_.log(core::LogLevel::kFatal, "Failed to create I2C instance on bus %d", bus);
    return;
  }
  const auto i2c = std::make_shared<io::HardwareI2c>(*optional_i2c);
  {
    Command i2c_read_command;
    std::stringstream identifier;
    identifier << "i2c " << static_cast<int>(bus) << " read";
    i2c_read_command.name = identifier.str();
    std::stringstream description;
    description << "Read from I2C bus " << static_cast<int>(bus);
    i2c_read_command.description = description.str();
    i2c_read_command.handler     = [this, i2c, bus]() {
      std::uint8_t device_address, register_address;
      std::cout << "Device address: ";
      std::cin >> device_address;
      std::cout << "Register address: ";
      std::cin >> register_address;
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      const auto value = i2c->readByte(device_address, register_address);
      if (value) {
        logger_.log(core::LogLevel::kInfo, "I2C value from bus %d: %d", bus, *value);
      } else {
        logger_.log(core::LogLevel::kFatal, "Failed to read from I2C bus %d", bus);
      }
    };
    addCommand(i2c_read_command);
  }
  {
    Command i2c_write_command;
    std::stringstream identifier;
    identifier << "i2c " << static_cast<int>(bus) << " write";
    i2c_write_command.name = identifier.str();
    std::stringstream description;
    description << "Write to I2C bus " << static_cast<int>(bus);
    i2c_write_command.description = description.str();
    i2c_write_command.handler     = [this, i2c, bus]() {
      std::uint32_t device_address, register_address, data;
      std::cout << "Device address: ";
      std::cin >> std::hex >> device_address;
      std::cout << "Register address: ";
      std::cin >> std::hex >> register_address;
      std::cout << "Data: ";
      std::cin >> std::hex >> data;
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      const core::Result result = i2c->writeByteToRegister(device_address, register_address, data);
      if (result == core::Result::kSuccess) {
        logger_.log(
          core::LogLevel::kInfo, "I2C write successful to device %d on %d", device_address, bus);
      } else {
        logger_.log(core::LogLevel::kFatal, "Failed to write to I2C bus: %d", bus);
      }
    };
    addCommand(i2c_write_command);
  }
}

void Repl::addSpiCommands(const std::uint8_t bus)
{
  const auto optional_spi = io::HardwareSpi::create(logger_);
  if (!optional_spi) {
    logger_.log(core::LogLevel::kFatal, "Failed to create I2C instance on bus %d", bus);
    return;
  }
  const auto spi = std::make_shared<io::HardwareSpi>(*optional_spi);
  {
    Command spi_read_byte_command;
    std::stringstream identifier;
    identifier << "spi " << static_cast<int>(bus) << " read byte";
    spi_read_byte_command.name = identifier.str();
    std::stringstream description;
    description << "Read from SPI bus " << static_cast<int>(bus);
    spi_read_byte_command.description = description.str();
    spi_read_byte_command.handler     = [this, spi, bus]() {
      std::uint8_t register_address;
      std::cout << "Register address: ";
      std::cin >> register_address;
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::uint8_t read_buffer;
      const core::Result result = spi->read(register_address, &read_buffer, 1);
      if (result == core::Result::kSuccess) {
        logger_.log(core::LogLevel::kInfo, "SPI value from bus %d: %d", bus, read_buffer);
      } else {
        logger_.log(core::LogLevel::kFatal, "Failed to read from SPI bus %d", bus);
      }
    };
    addCommand(spi_read_byte_command);
  }
  {
    Command spi_write_byte_command;
    std::stringstream identifier;
    identifier << "spi " << static_cast<int>(bus) << " write byte";
    spi_write_byte_command.name = identifier.str();
    std::stringstream description;
    description << "Write to SPI bus " << static_cast<int>(bus);
    spi_write_byte_command.description = description.str();
    spi_write_byte_command.handler     = [this, spi, bus]() {
      std::uint32_t register_address, data;
      std::cout << "Register address: ";
      std::cin >> std::hex >> register_address;
      std::cout << "Data: ";
      std::cin >> std::hex >> data;
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      const std::uint8_t *data_ptr = reinterpret_cast<const std::uint8_t *>(&data);
      const core::Result result    = spi->write(register_address, data_ptr, 1);
      if (result == core::Result::kSuccess) {
        logger_.log(
          core::LogLevel::kInfo, "Successful SPI write to device %d on %d", register_address, bus);
      } else {
        logger_.log(core::LogLevel::kFatal, "Failed to write to SPI bus: %d", bus);
      }
    };
    addCommand(spi_write_byte_command);
  }
}
}  // namespace hyped::debug
