#include "gpio.hpp"

#include <fcntl.h>
#include <utils/system.hpp>

namespace hyped::io {

Gpio::Gpio()
{
  // TODO: implement
  if (!initialised_) initialise();
}

void Gpio::initialise()
{
  int fd;  // file descriptor
  off_t offset;
  void *base;

  fd = open("/dev/mem", O_RDWR);
  if (fd < 0) {
    kLog.error("could not open /dev/mem");
    return;
  }

  for (size_t i = 0; i < Gpio::kBankNum; ++i) {
    offset = Gpio::kBases[i];
    /**
     * @brief mmap() creates a new mapping in the virtual address space of the calling process
        The starting address is zero
        The length argument is kMmapSize
     * void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);
     */
    base = mmap(0, Gpio::kMmapSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
    if (base == MAP_FAILED) {
      kLog.error("could not map bank %d", offset);
      return;
    }

    base_mapping_[i] = base;
  }
  atexit(uninitialise);
  initialised_ = true;
}

std::optional<GpioReader> Gpio::getReader(const uint8_t pin)
{
  // TODO: implement
  return std::nullopt;
}

std::optional<GpioWriter> Gpio::getWriter(const uint8_t pin)
{
  // TODO: implement
  return std::nullopt;
}

}  // namespace hyped::io