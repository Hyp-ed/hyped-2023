#include <unistd.h>

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
    auto maybe_can = hyped::io::HardwareCan::create(logger, "can1");
    if (!maybe_can) {
      logger.log(hyped::core::LogLevel::kFatal, "no can pain");
      return;
    }
    logger.log(hyped::core::LogLevel::kDebug, "CAN socket enabled");
    hyped::io::CanFrame frame;
    frame.can_id  = 1;
    frame.can_dlc = 8;
    for (int i = 0; i < 8; i++) {
      frame.data[i] = 5;
    }
    auto can = *maybe_can;
    logger.log(hyped::core::LogLevel::kDebug, "socket: %i", can->getSocket());
    // auto result = can->send(frame);
    // if (result == hyped::core::Result::kFailure) {
    //   logger.log(hyped::core::LogLevel::kFatal, "no can send");
    // } else {
    //   logger.log(hyped::core::LogLevel::kDebug, "yes can send");
    // }
    while (1) {
      sleep(10);
      hyped::io::CanFrame data = *can->receive();
      logger.log(hyped::core::LogLevel::kDebug,
                 "CAN data received, ID:%i, DATA: %i %i %i %i %i %i %i %i",
                 static_cast<int>(data.can_id),
                 static_cast<int>(data.data[0]),
                 static_cast<int>(data.data[1]),
                 static_cast<int>(data.data[2]),
                 static_cast<int>(data.data[3]),
                 static_cast<int>(data.data[4]),
                 static_cast<int>(data.data[5]),
                 static_cast<int>(data.data[6]),
                 static_cast<int>(data.data[7]));
    }
    
  });
  std::cout << "Ran for " << execution_time.count() << " ns" << std::endl;
}
