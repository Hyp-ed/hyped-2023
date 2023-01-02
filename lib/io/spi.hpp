#pragma once

#include <cstdint>
#include <cstring>

#include <core/logger.hpp>
#include <core/types.hpp>

#if __linux__
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

// All values and configuration options used are sourced from the AM335x and AMIC110 Sitara™
// Processors Technical Reference Manual, please refer to the manual for more information.

// Two SPI buses are available on the BeagleBone Black
enum class SpiBus { kSpi0 = 0, kSpi1 };
// Four SPI modes are available on the BeagleBone Black - for more information, see
// https://github.com/Hyp-ed/hyped-2023/wiki/SPI-Interfacing-on-BBB#spi-modes-summary
enum class SpiMode { kMode0 = 0, kMode1, kMode2, kMode3 };
// Common word sizes (in bits) for SPI communcation
enum class SpiWordSize { kWordSize4 = 4, kWordSize8 = 8, kWordSize16 = 16, kWordSize32 = 32 };
enum class SpiBitOrder { kMsbFirst = 0, kLsbFirst };
// Maximum clock frequency for SPI is 100MHz
enum class Clock { k500KHz, k1MHz, k4MHz, k16MHz, k20MHz };

namespace hyped::io {

class Spi {
 public:
  /**
   * @brief Construct a new Spi object. By default we initialise to SPI1, Mode 3 (SPICLK
   * active low and sampling occurs on the rising edge) with 8-bit words and MSB first.
   * @param logger - logger to be used
   * @param bus    - SPI bus to be used
   * @param mode   - SPI mode to be used
   * @param clock  - clock frequency to be used
   */
  static std::optional<Spi> create(core::ILogger &logger,
                                   const SpiBus bus            = SpiBus::kSpi1,
                                   const SpiMode mode          = SpiMode::kMode3,
                                   const SpiWordSize word_size = SpiWordSize::kWordSize8,
                                   const SpiBitOrder bit_order = SpiBitOrder::kMsbFirst);
  ~Spi();

  /**
   * @brief Get data from sensor, starting at some address.
   * @param addr  - register from which the reading should start
   * @param rx    - pointer to head of read buffer
   * @param len   - number of BYTES to be read, i.e. size of the read buffer
   */
  core::Result read(std::uint8_t addr, std::uint8_t *rx, std::uint16_t len);

  /**
   * @brief Write data to sensor, starting at some address.
   * @param addr  - register from which writing to starts
   * @param tx    - pointer to head of write buffer
   * @param len   - number of BYTES to be written, i.e. size of the write buffer
   */
  core::Result write(std::uint8_t addr, std::uint8_t *tx, std::uint16_t len);

 private:
  Spi(core::ILogger &logger, const int file_descriptor);
  /**
   * @brief Get the address of the SPI bus in use
   * @param bus - SPI bus to be used
   */
  static const char *getSpiBusAddress(const SpiBus bus);

  /**
   * @brief Set the clock frequency of the SPI bus
   * @param clock - clock frequency to be set
   * @return uint32_t - actual clock frequency
   */
  static uint32_t getClockValue(Clock clock);

  core::ILogger &logger_;
  int file_descriptor_;
};

}  // namespace hyped::io
