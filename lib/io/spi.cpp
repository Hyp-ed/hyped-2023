#include "spi.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/mman.h>

#if LINUX
#include <linux/spi/spidev.h>
#else
// Relevant definitions from linux/spi/spidev.h for non-Linux systems
#define _IOC_SIZEBITS 14
#define SPI_IOC_MAGIC 'k'
#define SPI_IOC_WR_MODE _IOW(SPI_IOC_MAGIC, 1, std::uint8_t)
#define SPI_IOC_WR_MAX_SPEED_HZ _IOW(SPI_IOC_MAGIC, 4, std::uint32_t)
#define SPI_IOC_WR_LSB_FIRST _IOW(SPI_IOC_MAGIC, 2, std::uint8_t)
#define SPI_IOC_WR_BITS_PER_WORD _IOW(SPI_IOC_MAGIC, 3, std::uint8_t)

struct spi_ioc_transfer {
  std::uint64_t tx_buf;
  std::uint64_t rx_buf;
  std::uint32_t len;
  std::uint32_t speed_hz;
  std::uint16_t delay_usecs;
  std::uint8_t bits_per_word;
  std::uint8_t cs_change;
  std::uint8_t tx_nbits;
  std::uint8_t rx_nbits;
  std::uint16_t pad;
};

#define SPI_MSGSIZE(N)                                                                             \
  ((((N) * (sizeof(struct spi_ioc_transfer))) < (1 << _IOC_SIZEBITS))                              \
     ? ((N) * (sizeof(struct spi_ioc_transfer)))                                                   \
     : 0)
#define SPI_IOC_MESSAGE(N) _IOW(SPI_IOC_MAGIC, 0, char[SPI_MSGSIZE(N)])
#define SPI_CS_HIGH 0x04
#endif  // if LINUX

namespace hyped::io {

// define what the address space of SPI looks like, see
// https://github.com/Hyp-ed/hyped-2023/wiki/SPI-Interfacing-on-BBB#spi-register-summary
#pragma pack(1)
struct Spi_Channel_Registers {
  std::uint32_t conf;  // 0x00
  std::uint32_t stat;  // 0x04
  std::uint32_t ctrl;  // 0x08
  std::uint32_t tx;    // 0x0c
  std::uint32_t rx;    // 0x10
};

#pragma pack(1)  // ensuring compiler does not add padding
struct Spi_Registers {
  std::uint32_t revision;          // 0x000
  std::uint32_t reserved0[0x43];   // 0x004 - 0x110
  std::uint32_t sysconfig;         // 0x110
  std::uint32_t sysstatus;         // 0x114
  std::uint32_t irqstatus;         // 0x118
  std::uint32_t irqenable;         // 0x11c
  std::uint32_t reserved1[2];      // 0x120 - 0x124
  std::uint32_t syst;              // 0x124
  std::uint32_t modulctr;          // 0x128
  Spi_Channel_Registers channel0;  // 0x12c - 0x140
  Spi_Channel_Registers channel1;  // 0x140 - 0x154
  Spi_Channel_Registers channel2;  // 0x154 - 0x168
  Spi_Channel_Registers channel3;  // 0x168 - 0x17c
  std::uint32_t xferlevel;         // 0x17c
};

Spi::Spi(core::ILogger &logger)
    : logger_(logger),
      spi_registers_(0),
      spi_channel0_registers_(0),
      has_initialised_(Initialised::kFalse)
{
}

Spi::~Spi()
{
  close(file_descriptor_);
}

core::Result Spi::read(std::uint8_t addr, std::uint8_t *rx, std::uint16_t len)
{
  if (has_initialised_ == Initialised::kFalse) {
    logger_.log(core::LogLevel::kFatal, "SPI device has not been initialised");
    return core::Result::kFailure;
  }
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
  if (has_initialised_ == Initialised::kFalse) {
    logger_.log(core::LogLevel::kFatal, "SPI device has not been initialised");
    return core::Result::kFailure;
  }
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

core::Result Spi::initialiseSpi(const SpiBus bus,
                                const SpiMode mode,
                                const SpiWordSize word_size,
                                const SpiBitOrder bit_order)
{
  // SPI bus only works in kernel mode on Linux, so we neeed to call the provided driver
  char spi_bus_address[15];
  if (bus == SpiBus::kSpi0) {
    strncat(spi_bus_address, "/dev/spidev0.0", 15);
  } else {
    strncat(spi_bus_address, "/dev/spidev1.0", 15);
  }
  file_descriptor_ = open(spi_bus_address, O_RDWR, 0);
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
  // Create SPI virtal memory mappings
  const auto virtual_mapping_result = createVirtualMapping(bus);
  if (virtual_mapping_result == core::Result::kFailure) {
    logger_.log(core::LogLevel::kFatal,
                "Failed to initialise SPI, could not create virtual mapping");
    return core::Result::kFailure;
  }
  logger_.log(core::LogLevel::kDebug, "Successfully initialised SPI");
  has_initialised_ = Initialised::kTrue;
  return core::Result::kSuccess;
}

core::Result Spi::createVirtualMapping(const SpiBus bus)
{
  // First open the /dev/mem (device file that represents the whole physical memory)
  const int mapping_file_descriptor = open("/dev/mem", O_RDWR);
  if (mapping_file_descriptor < 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to open /dev/mem");
    return core::Result::kFailure;
  }
  // Map the relevant SPI registers into virtual memory
  void *spi_registers_base;
  if (bus == SpiBus::kSpi0) {
    spi_registers_base = mmap(0,
                              kSpiMemoryMapSize,
                              PROT_READ | PROT_WRITE,
                              MAP_SHARED,
                              mapping_file_descriptor,
                              kSpi0AddrBase);
  } else {
    spi_registers_base = mmap(0,
                              kSpiMemoryMapSize,
                              PROT_READ | PROT_WRITE,
                              MAP_SHARED,
                              mapping_file_descriptor,
                              kSpi1AddrBase);
  }
  if (spi_registers_base == MAP_FAILED) {
    logger_.log(core::LogLevel::kFatal, "Failed to map SPI registers");
    return core::Result::kFailure;
  }
  // Get values of relevant registers
  spi_registers_          = reinterpret_cast<Spi_Registers *>(spi_registers_base);
  spi_channel0_registers_ = &spi_registers_->channel0;
  logger_.log(core::LogLevel::kDebug, "Successfully created mapping for SPI registers");
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