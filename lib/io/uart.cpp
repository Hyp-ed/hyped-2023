#include "uart.hpp"

#include <fcntl.h>
#include <unistd.h>

namespace hyped::io {

std::optional<Uart> Uart::create(core::ILogger &logger,
                                 const UartBus bus,
                                 const std::uint32_t baud_rate,
                                 const std::uint8_t bits_per_byte)
{
  char path[15];  // up to "/dev/ttyO5"
  sprintf(path, "/dev/ttyO%d", static_cast<std::uint8_t>(bus));
  const int file_descriptor = open(path, O_RDWR | O_NOCTTY);
  if (file_descriptor < 0) {
    logger.log(core::LogLevel::kFatal,
               "Failed to open UART file descriptor, could not create UART instance");
    return std::nullopt;
  }
  const auto baud_mask = getBaudRateMask(baud_rate);
  if (!baud_mask) {
    logger.log(core::LogLevel::kFatal,
               "Failed to set invalid baudrate, could not create UART instance");
    return std::nullopt;
  }
  const auto bits_per_byte_mask = getBitsPerByteMask(bits_per_byte);
  if (!bits_per_byte_mask) {
    logger.log(core::LogLevel::kFatal,
               "Failed to set invalid number of bits per byte, could not create UART instance");
    return std::nullopt;
  }
  const auto configuration_result = configureFileForOperation(
    logger, file_descriptor, baud_mask.value(), bits_per_byte_mask.value());
  if (configuration_result == core::Result::kFailure) {
    logger.log(core::LogLevel::kFatal,
               "Failed to configure UART file, could not create UART instance");
    return std::nullopt;
  }
  logger.log(core::LogLevel::kDebug, "Successfully created UART instance");
  return Uart(logger, file_descriptor);
}

core::Result Uart::configureFileForOperation(core::ILogger &logger,
                                             const int file_descriptor,
                                             const std::uint8_t baud_mask,
                                             const std::uint8_t bits_per_byte_mask)
{
  struct termios tty;
  // ensuring all bits are initially 0, else any set bit could lead to undefined behavior
  bzero(&tty, sizeof(tty));
  tty.c_cflag                    = baud_mask | bits_per_byte_mask | CLOCAL | CREAD;
  tty.c_iflag                    = IGNPAR | ICRNL | IGNCR;
  tty.c_oflag                    = 0;
  tty.c_lflag                    = 0;
  tty.c_cc[VTIME]                = 0;
  tty.c_cc[VMIN]                 = 0;
  const int termios_flush_result = tcflush(file_descriptor, TCIFLUSH);
  if (termios_flush_result < 0) {
    logger.log(core::LogLevel::kFatal, "Failed to flush UART file for non-transmitted data");
    return core::Result::kFailure;
  }
  const int termios_config_result = tcsetattr(file_descriptor, TCSANOW, &tty);
  if (termios_config_result < 0) {
    logger.log(core::LogLevel::kFatal, "Failed to configure UART file for operation");
    return core::Result::kFailure;
  }
  logger.log(core::LogLevel::kDebug, "Successfully configured UART file for operation");
  return core::Result::kSuccess;
}

std::optional<std::uint8_t> Uart::getBitsPerByteMask(const std::uint8_t bits_per_byte)
{
  switch (bits_per_byte) {
    case 5:
      return CS5;
    case 6:
      return CS6;
    case 7:
      return CS7;
    case 8:
      return CS8;
    default:
      return std::nullopt;
  }
}

std::optional<std::uint8_t> Uart::getBaudRateMask(const std::uint32_t baudrate)
{
  switch (baudrate) {
    case 300:
      return B300;
    case 600:
      return B600;
    case 1200:
      return B1200;
    case 1800:
      return B1800;
    case 2400:
      return B2400;
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    default:
      return std::nullopt;
  }
}

Uart::Uart(core::ILogger &logger, const int file_descriptor)
    : logger_(logger),
      file_descriptor_(file_descriptor)
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