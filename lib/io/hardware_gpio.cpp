#include "hardware_gpio.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <sys/mman.h>

namespace hyped::io {

std::optional<core::DigitalSignal> HardwareGpioReader::read()
{
  // pinMap just 0000.... with 1 flipped in pin number.
  // So AND with readAddr to extract the specific pin
  const std::uint8_t out = 0;
  if (*gpio_readAddr & pin_map){
    return core::DigitalSignal::kHigh;
  } else {
    return core::DigitalSignal::kLow;
  }

}

core::Result HardwareGpioWriter::write(const core::DigitalSignal state)
{
  // Set and Clear are seperate registers.
  if (state == core::DigitalSignal::kHigh) {
    *gpio_setAddr = pin_map;
  } else {
    *gpio_clearAddr = pin_map;
  }
  return core::Result::kSuccess;
}

HardwareGpio::HardwareGpio(core::ILogger &log) : log_(log)
{
}

std::optional<std::shared_ptr<IGpioReader>> HardwareGpio::getReader(const std::uint8_t pin)
{
  // What happens if all shared pointers are removed and this is called?
  // Map solves this, GPIO will always hold one pointer so it dosen't get destroyed.
  if (initialized_readers_.count(pin) != 0) {
    std::shared_ptr<IGpioReader> reader = initialized_readers_[pin];
    return reader;
  }

  // There are 4 Bank Numbers, 0 1 2 3
  // Integer divison to get bank number
  const std::uint32_t bank = pin / 32;
  // Modulo by 32 gets us the ID of the pin relative to the bank
  const std::uint32_t pin_id = pin % 32;
  // Gpio addresses contain 32 pins, so we use pinmap to specify specific pin.
  const std::uint32_t pin_map = (1 << pin_id);
  if (bank > 3) {
    log_.log(core::LogLevel::kFatal, "invalid pin number");
    return std::nullopt;
  }

  // Get memory address from bank address
  const off_t pin_address = bank_addresses[bank];
  volatile void *gpio_addr;
  volatile std::uint32_t *gpio_read;

  // /dev/mem and mmap is related to making the registry available as a virtual memory address.
  int fd = open("/dev/mem", O_RDWR);
  if (fd < 0) {
    log_.log(core::LogLevel::kFatal, "opening /dev/mem failed");
    return std::nullopt;
  }

  // Get the memory mapping of the GPIO pin.
  gpio_addr = mmap(0, pin_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, pin_address);
  if (gpio_addr == MAP_FAILED) {
    log_.log(core::LogLevel::kFatal, "mmap failed");
    return std::nullopt;
  }
  // Type conversion for address adding from void
  const std::uint64_t base = reinterpret_cast<std::uint64_t>(gpio_addr);
  // pinRead is the hardware specified address for reading.
  gpio_read = reinterpret_cast<volatile std::uint32_t *>(base + pin_read);

  // Keep track of intialized pins
  initialized_readers_[pin] = std::shared_ptr<HardwareGpioReader>(new HardwareGpioReader(pin_map, gpio_read));
  std::shared_ptr<IGpioReader> reader = initialized_readers_[pin];
  return reader;
}

std::optional<std::shared_ptr<IGpioWriter>> HardwareGpio::getWriter(const std::uint8_t pin)
{
  // Check if pin is already initialized
  if (initialized_writers_.count(pin) != 0) {
    std::shared_ptr<IGpioWriter> writer = initialized_writers_[pin];
    return writer;
  }

  const std::uint32_t bank   = pin / 32;
  const std::uint32_t pin_id  = pin % 32;
  const std::uint32_t pin_map = (1 << pin_id);

  const off_t pin_address = bank_addresses[bank];
  volatile void *gpio_addr;
  volatile std::uint32_t *gpio_set;
  volatile std::uint32_t *gpio_clear;

  int fd = open("/dev/mem", O_RDWR);
  if (fd < 0) { return std::nullopt; }

  gpio_addr = mmap(0, pin_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, pin_address);
  if (gpio_addr == MAP_FAILED) { return std::nullopt; }

  const std::uint64_t base = reinterpret_cast<std::uint64_t>(gpio_addr);

  // pinset is the hardware specified address for reading.
  gpio_set = reinterpret_cast<volatile std::uint32_t *>(base + pin_set);

  // pinclear is the hardware specified address for reading.
  gpio_clear = reinterpret_cast<volatile std::uint32_t *>(base + pin_clear);

  initialized_writers_[pin]
    = std::shared_ptr<HardwareGpioWriter>(new HardwareGpioWriter(pin_map, gpio_set, gpio_clear));
  std::shared_ptr<IGpioWriter> writer = initialized_writers_[pin];
  return writer;
}

}  // namespace hyped::io