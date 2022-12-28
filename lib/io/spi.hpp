#pragma once

#include <cstdint>
#include <cstring>

#include <core/logger.hpp>
#include <core/types.hpp>

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
enum class Initialised { kFalse = 0, kTrue };
// Only one chip select is available by default on the BeagleBone Black
static constexpr std::uint32_t kSpi0AddrBase = 0x48030000;  // For SPI0 Chip Select 0
static constexpr std::uint32_t kSpi1AddrBase = 0x481A0000;  // For SPI1 Chip Select 0
// The size of the virtual memory mapping for SPI - 4KB
static constexpr std::uint32_t kSpiMemoryMapSize = 0x1000;

namespace hyped::io {

// The following structs are used to access the registers of the SPI bus directly
struct Spi_Registers;
struct Spi_Channel_Registers;

class Spi {
 public:
  Spi(core::ILogger &logger);
  ~Spi();

  /**
   * @brief Initialise the SPI bus. We default to SPI1, Mode 3 (SPICLK active low and sampling
   * occurs on the rising edge) with 8-bit words and MSB first
   * @param bus       - SPI bus to be used
   * @param mode      - SPI mode to be used
   * @param word_size - word size to be used
   * @param bit_order - bit order to be used
   */
  core::Result initialiseSpi(const SpiBus bus            = SpiBus::kSpi1,
                             const SpiMode mode          = SpiMode::kMode3,
                             const SpiWordSize word_size = SpiWordSize::kWordSize8,
                             const SpiBitOrder bit_order = SpiBitOrder::kMsbFirst);

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
  /**
   * @brief Create a virtual memory mapping for the SPI bus in use,
   * this allows us to access the registers of the SPI bus directly
   * @param bus - SPI bus to be used
   */
  core::Result createVirtualMapping(const SpiBus bus);
  /**
   * @brief Set the clock frequency of the SPI bus
   * @param clk - clock frequency to be set
   */
  core::Result setClock(Clock clk);

  int file_descriptor_;
  Initialised has_initialised_;
  Spi_Registers *spi_registers_;
  Spi_Channel_Registers *spi_channel0_registers_;
  core::ILogger &logger_;
};

}  // namespace hyped::io
