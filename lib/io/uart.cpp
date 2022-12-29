#include "uart.hpp"

#include <fcntl.h>
#include <unistd.h>

namespace hyped::io {

Uart::Uart(core::ILogger &logger, const UartBus bus, const std::uint32_t baudrate)
    : logger_(logger),
      bus_(bus),
      baudrate_(baudrate)
{
  char path[13];  // up to "/dev/ttyO5"
  sprintf(path, "/dev/ttyO%d", static_cast<std::uint8_t>(bus_));
  file_descriptor_ = open(path, O_RDWR);
  if (file_descriptor_ < 0) { logger_.log(core::LogLevel::kFatal, "Unable to open UART file"); }
}

Uart::~Uart()
{
  close(file_descriptor_);
}

core::Result Uart::send(char *tx, std::uint8_t length)
{
  // TODO
  return core::Result::kFailure;
}

core::Result Uart::read(unsigned char *rx, std::uint8_t length)
{
  // TODO
  return core::Result::kFailure;
}

}  // namespace hyped::io