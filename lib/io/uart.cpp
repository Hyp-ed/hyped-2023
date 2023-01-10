#include "uart.hpp"

#include <fcntl.h>
#include <unistd.h>

namespace hyped::io {

std::optional<Uart> Uart::create(core::ILogger &logger,
                                 const UartBus bus,
                                 const std::uint32_t baudrate)
{
  char path[15];  // up to "/dev/ttyO5"
  sprintf(path, "/dev/ttyO%d", static_cast<std::uint8_t>(bus));
  const int file_descriptor = open(path, O_RDWR | O_NOCTTY);
  if (file_descriptor < 0) {
    logger.log(core::LogLevel::kFatal,
               "Failed to open UART file descriptor, cannot create UART instance");
    return std::nullopt;
  }
  const auto baud_mask = getBaudRateMask(baudrate);
  if (!baud_mask) {
    logger.log(core::LogLevel::kFatal, "Failed to set baudrate, invalid baudrate provided");
    return std::nullopt;
  }
  logger.log(core::LogLevel::kDebug, "Successfully created UART instance");
  return Uart(logger, file_descriptor, baud_mask.value());
}

std::optional<std::uint32_t> Uart::getBaudRateMask(const std::uint32_t baudrate)
{
  std::uint32_t baud;
  switch (baudrate) {
    case 300:
      baud = B300;
      break;
    case 600:
      baud = B600;
      break;
    case 1200:
      baud = B1200;
      break;
    case 1800:
      baud = B1800;
      break;
    case 2400:
      baud = B2400;
      break;
    case 4800:
      baud = B4800;
      break;
    case 9600:
      baud = B9600;
      break;
    case 19200:
      baud = B19200;
      break;
    case 38400:
      baud = B38400;
      break;
    case 57600:
      baud = B57600;
      break;
    case 115200:
      baud = B115200;
      break;
    case 230400:
      baud = B230400;
      break;
    case 460800:
      baud = B460800;
      break;
    case 500000:
      baud = B500000;
      break;
    case 576000:
      baud = B576000;
      break;
    case 921600:
      baud = B921600;
      break;
    case 1000000:
      baud = B1000000;
      break;
    case 1152000:
      baud = B1152000;
      break;
    case 1500000:
      baud = B1500000;
      break;
    case 2000000:
      baud = B2000000;
      break;
    case 2500000:
      baud = B2500000;
      break;
    case 3000000:
      baud = B3000000;
      break;
    case 3500000:
      baud = B3500000;
      break;
    default:
      return std::nullopt;
  }
  return baud;
}

Uart::Uart(core::ILogger &logger, const int file_descriptor, const std::uint32_t baud_mask)
    : logger_(logger),
      file_descriptor_(file_descriptor),
      baud_mask_(baud_mask)
{
  int baud = B115200;
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