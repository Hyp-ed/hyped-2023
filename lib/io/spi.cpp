#include "spi.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <sys/ioctl.h>

namespace hyped::io {

std::optional<Spi> Spi::create(core::ILogger &logger,
                               const SpiBus bus,
                               const SpiMode mode,
                               const SpiWordSize word_size,
                               const SpiBitOrder bit_order)
{
  Spi spi(logger);
  const auto initialisation_result = spi.initialise(bus, mode, word_size, bit_order);
  if (initialisation_result == core::Result::kFailure) {
    logger.log(core::LogLevel::kFatal, "Failed to initialise SPI");
    return std::nullopt;
  }
  logger.log(core::LogLevel::kDebug, "Successfully initialised SPI");
  return spi;
}

Spi::Spi(core::ILogger &logger) : logger_(logger), file_descriptor_(-1)
{
}

Spi::~Spi()
{
  close(file_descriptor_);
}

core::Result Spi::read(std::uint8_t addr, std::uint8_t *rx, std::uint16_t len)
{
  if (file_descriptor_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to open SPI device wile reading");
    return core::Result::kFailure;
  }
  spi_ioc_transfer message[2] = {};
  // send address
  message[0].tx_buf = reinterpret_cast<std::uint64_t>(&addr);
  message[0].rx_buf = 0;
  message[0].len    = 1;
  // receive data
  message[1].tx_buf      = 0;
  message[1].rx_buf      = reinterpret_cast<std::uint64_t>(rx);
  message[1].len         = len;
  const auto read_result = ioctl(file_descriptor_, SPI_IOC_MESSAGE(2), message);
  if (read_result < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to read from SPI device");
    return core::Result::kFailure;
  }
  logger_.log(core::LogLevel::kDebug, "Successfully read from SPI device");
  return core::Result::kSuccess;
}

core::Result Spi::write(std::uint8_t addr, std::uint8_t *tx, std::uint16_t len)
{
  if (file_descriptor_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to open SPI device wile writing");
    return core::Result::kFailure;
  }
  spi_ioc_transfer message[2] = {};
  // send address
  message[0].tx_buf = reinterpret_cast<std::uint64_t>(&addr);
  message[0].rx_buf = 0;
  message[0].len    = 1;
  // write data
  message[1].tx_buf       = reinterpret_cast<std::uint64_t>(tx);
  message[1].rx_buf       = 0;
  message[1].len          = len;
  const auto write_result = ioctl(file_descriptor_, SPI_IOC_MESSAGE(2), message);
  if (write_result < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to write to SPI device");
    return core::Result::kFailure;
  }
  logger_.log(core::LogLevel::kDebug, "Successfully wrote to SPI device");
  return core::Result::kSuccess;
}

const char *Spi::getSpiBusAdress(const SpiBus bus)
{
  if (bus == SpiBus::kSpi0) {
    return "/dev/spidev0.0";
  } else {
    return "/dev/spidev1.0";
  }
}

core::Result Spi::initialise(const SpiBus bus,
                             const SpiMode mode,
                             const SpiWordSize word_size,
                             const SpiBitOrder bit_order)
{
  // SPI bus only works in kernel mode on Linux, so we neeed to call the provided driver
  const char *spi_bus_address = getSpiBusAdress(bus);
  file_descriptor_            = open(spi_bus_address, O_RDWR, 0);
  if (file_descriptor_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to open SPI device");
    return core::Result::kFailure;
  }
  // Set clock frequency
  const auto clock_set_result = setClock(Clock::k500KHz);
  if (clock_set_result == core::Result::kFailure) {
    logger_.log(core::LogLevel::kFatal,
                "Failed to set clock frequency, so SPI device not initialised");
    return core::Result::kFailure;
  }
  // Set word size
  const std::uint8_t bits_per_word = static_cast<std::uint8_t>(word_size);
  const auto word_size_write_result
    = ioctl(file_descriptor_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
  if (word_size_write_result < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to set bits per word");
    return core::Result::kFailure;
  }
  // Set SPI mode
  const std::uint8_t selected_mode = static_cast<std::uint8_t>(mode);
  const auto mode_write_result     = ioctl(file_descriptor_, SPI_IOC_WR_MODE, &selected_mode);
  if (mode_write_result < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to set SPI mode");
    return core::Result::kFailure;
  }
  // Set bit order
  const std::uint8_t order      = static_cast<std::uint8_t>(bit_order);
  const auto order_write_result = ioctl(file_descriptor_, SPI_IOC_WR_LSB_FIRST, &order);
  if (order_write_result < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to set bit order");
    return core::Result::kFailure;
  }
  logger_.log(core::LogLevel::kDebug, "Successfully initialised SPI");
  return core::Result::kSuccess;
}

core::Result Spi::setClock(Clock clock)
{
  std::uint32_t data;
  switch (clock) {
    case Clock::k500KHz:
      data = 500000;
      break;
    case Clock::k1MHz:
      data = 1000000;
      break;
    case Clock::k4MHz:
      data = 4000000;
      break;
    case Clock::k16MHz:
      data = 16000000;
      break;
    case Clock::k20MHz:
      data = 20000000;
      break;
  }
  const auto clock_write_result = ioctl(file_descriptor_, SPI_IOC_WR_MAX_SPEED_HZ, &data);
  if (clock_write_result < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to set clock frequency to %d", data);
    return core::Result::kFailure;
  }
  logger_.log(core::LogLevel::kDebug, "Successfully set clock frequency to %d", data);
  return core::Result::kSuccess;
}

}  // namespace hyped::io