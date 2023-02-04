#include <iostream>

#include <core/logger.hpp>
#include <core/timer.hpp>
#include <core/types.hpp>
#include <core/wall_clock.hpp>
#include <io/hardware_can.hpp>
#include <io/hardware_gpio.hpp>

int main(int argc, char **argv)
{
  hyped::core::WallClock time;
  hyped::core::Timer timer(time);
  const auto execution_time = timer.measure_execution_time([time]() {
    hyped::core::Logger logger("CAN", hyped::core::LogLevel::kDebug, time);
    logger.log(hyped::core::LogLevel::kDebug, "CAN test");
    // auto maybe_can = hyped::io::HardwareCan::create(logger, "can1");
    // if (!maybe_can) {
    //   logger.log(hyped::core::LogLevel::kFatal, "no can pain");
    //   return;
    // }
    // logger.log(hyped::core::LogLevel::kDebug, "CAN socket enabled");
    // auto can = *maybe_can;
    hyped::io::HardwareCan can = hyped::io::HardwareCan(logger);
    can.initialise("can1");
    hyped::io::CanFrame frame;
    frame.can_id  = 1;
    frame.can_dlc = 8;
    for (int i = 0; i < 8; i++) {
      frame.data[i] = 5;
    }
    auto result = can.send(frame);
    if (result == hyped::core::Result::kFailure) {
      logger.log(hyped::core::LogLevel::kFatal, "no can send");
    } else {
      logger.log(hyped::core::LogLevel::kFatal, "yes can send");
    }
  });
  std::cout << "Ran for " << execution_time.count() << " ns" << std::endl;
}
