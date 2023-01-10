#include "uart.hpp"

#include <fcntl.h>
#include <unistd.h>

namespace hyped::io {

std::optional<Uart> Uart::create(core::ILogger &logger,
                                 const UartBus bus,
                                 const std::uint32_t baudrate)
{
  char path[13];  // up to "/dev/ttyO5"
  sprintf(path, "/dev/ttyO%d", static_cast<std::uint8_t>(bus));
  const int file_descriptor = open(path, O_RDWR);
  if (file_descriptor < 0) {
    logger.log(core::LogLevel::kFatal,
               "Failed to open UART file descriptor, cannot create UART instance");
    return std::nullopt;
  }
  logger.log(core::LogLevel::kDebug, "Successfully created UART instance");
  return Uart(logger, file_descriptor, baudrate);
}

Uart::Uart(core::ILogger &logger, const int file_descriptor, const std::uint32_t baudrate)
    : logger_(logger),
      file_descriptor_(file_descriptor),
      baudrate_(baudrate)
{
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