#include "can_test.hpp"

int main()
{
  hyped::core::WallClock time;
  hyped::core::Logger logger("CanTest", hyped::core::LogLevel::kDebug, time);

  hyped::debug::CanTest can_test = hyped::debug::CanTest(logger);
}

namespace hyped::debug {

CanTest::CanTest(core::Logger &logger) : logger_(logger)
{
}

void CanTest::run()
{
  std::cout << "Enter s for send or r for receive" << std::endl;
  char input;
  std::cin >> input;
  if (input == 's') {
    send();
  } else if (input == 'r') {
    receive();
  } else {
    std::cout << "Invalid input" << std::endl;
  }
}

void CanTest::send()
{
  std::string bus         = "can0";
  const auto optional_can = io::HardwareCan::create(logger_, bus);
  if (!optional_can) {
    logger_.log(core::LogLevel::kFatal, "Failed to create CAN instance");
    return;
  }
  const auto can = std::move(*optional_can);
  while (1) {
    std::cout << "Enter CAN ID: ";
    std::uint32_t id;
    std::cin >> std::hex >> id;
    std::cout << "Enter CAN data: ";
    std::string data;
    std::cin >> std::hex >> data;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if (data.length() > 16) {
      logger_.log(core::LogLevel::kFatal, "Cannot send can data longer than 8 bytes");
      return;
    }
    const auto can_data = std::vector<std::uint8_t>(data.begin(), data.end());
    io::CanFrame can_frame;
    can_frame.can_id  = id;
    can_frame.can_dlc = can_data.size();
    for (int i = 0; i < can_data.size(); i++) {
      can_frame.data[i] = can_data[i];
    }
    core::Result result = can->send(can_frame);
    if (result == core::Result::kFailure) {
      logger_.log(core::LogLevel::kFatal, "Failed to write to CAN bus %s", bus.c_str());
      return;
    }
    logger_.log(core::LogLevel::kDebug, "Wrote to CAN bus %s", bus.c_str());
  }
}

void CanTest::receive()
{
  std::string bus         = "can0";
  const auto optional_can = io::HardwareCan::create(logger_, bus);
  if (!optional_can) {
    logger_.log(core::LogLevel::kFatal, "Failed to create CAN instance");
    return;
  }
  const auto can = std::move(*optional_can);
  while (1) {
    can->receive();
  }
}
}  // namespace hyped::debug