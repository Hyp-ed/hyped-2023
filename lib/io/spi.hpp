#pragma once

#include <core/logger.hpp>

namespace hyped::io {
// forward declaration
struct SPI_HW;
struct SPI_CH;

class Spi {
 public:
  Spi(core::ILogger &logger);
  ~Spi();

  enum class Clock { k500KHz, k1MHz, k4MHz, k16MHz, k20MHz };

  void setClock(Clock clk);

  /**
   * @brief simultaneous write and read. Write and read buffer should have the same length
   * @param tx  - pointer to head of write buffer
   * @param rx  - pointer to head of read  buffer
   * @param len - number of BYTES in each buffer
   */
  void transfer(std::uint8_t *tx, std::uint8_t *rx, std::uint16_t len);

  /**
   * @brief Get data from sensor, starting at some address.
   * @param addr  - register from which the reading should start
   * @param rx    - pointer to head of read buffer
   * @param len   - number of BYTES to be read, i.e. size of the read buffer
   */
  void read(std::uint8_t addr, std::uint8_t *rx, std::uint16_t len);

  /**
   * @brief Write data to sensor, starting at some address.
   * @param addr  - register from which writing to starts
   * @param tx    - pointer to head of write buffer
   * @param len   - number of BYTES to be written, i.e. size of the write buffer
   */
  void write(std::uint8_t addr, std::uint8_t *tx, std::uint16_t len);

 private:
  /**
   * @brief Fill in base_mapping_ with pointers to mmap-ed /dev/spidev1.0
   * to 2 SPI banks/ports.
   */
  bool initialise();

 private:
  int spi_fd_;
  SPI_HW *hw_;
  SPI_CH *ch_;
  core::ILogger &logger_;
};

}  // namespace hyped::io
