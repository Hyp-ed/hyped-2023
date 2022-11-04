#include "hardware_gpio.hpp"
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>





namespace hyped::io {

std::optional<core::DigitalSignal> HardwareGpioReader::read()
{
  
  int out = *gpio_readAddr & pinMAP;
  if (out > 0) {
    return core::DigitalSignal::kHigh;
  } else{
    return core::DigitalSignal::kLow;
  }
}

GpioWriteResult HardwareGpioWriter::write(const core::DigitalSignal state)
{
  //Not sure if this correct
  //May just erase everything
  //May need to 
  if (state == core::DigitalSignal::kHigh) {
    *gpio_setAddr |= pinMAP;
  } else {
    *gpio_clearAddr &= ~pinMAP;
  }

  throw -1;
}

HardwareGpio::HardwareGpio(hyped::core::ILogger &log) : log_(log)
{
  // TODO: implement
}



std::optional<std::shared_ptr<IGpioReader>> HardwareGpio::getReader(const uint8_t pin)
{
  //What happens if all shared pointers are removed and this is called?
  //Map solves this, GPIO will always hold one pointer so it dosen't get destroyed.
  if (InitializedReaders.count(pin) != 0) {
    std::shared_ptr<HardwareGpioReader> reader;
    return reader;
  }
  const uint8_t bank = pin / 32;
  const uint8_t pinID = pin % 32;
  const uint8_t pinMAP = (1 << pinID);

  const off_t pinAddress = bankAddresses[bank];
  volatile void *gpio_addr;
  volatile unsigned int *gpio_read;


  int fd = open("/dev/mem", O_RDWR);
  gpio_addr = mmap(0, pinSize , PROT_READ | PROT_WRITE, MAP_SHARED, fd, pinAddress);
  gpio_read = static_cast<volatile unsigned int*>(gpio_addr) + pinRead;
  
  InitializedReaders[pin] = std::shared_ptr<HardwareGpioReader>(new HardwareGpioReader(pinMAP, gpio_read));
  std::shared_ptr<HardwareGpioReader> reader;
  return reader;
}

std::optional<std::shared_ptr<IGpioWriter>> HardwareGpio::getWriter(const uint8_t pin)
{
  if (InitializedWriters.count(pin) != 0) {
    std::shared_ptr<HardwareGpioWriter> writer;
    return writer;
  }


  const uint8_t bank = pin / 32;
  const uint8_t pinID = pin % 32;
  const uint8_t pinMAP = (1 << pinID);

  const off_t pinAddress = bankAddresses[bank];
  volatile void *gpio_addr;
  volatile unsigned int *gpio_set;
  volatile unsigned int *gpio_clear;

  int fd = open("/dev/mem", O_RDWR);
  gpio_addr = mmap(0, pinSize , PROT_READ | PROT_WRITE, MAP_SHARED, fd, pinAddress);
  gpio_set =   static_cast<volatile unsigned int*>(gpio_addr) + pinSet;
  gpio_clear = static_cast<volatile unsigned int*>(gpio_addr) + pinClear;
  InitializedWriters[pin] = std::shared_ptr<HardwareGpioWriter>(new HardwareGpioWriter(pinMAP, gpio_set, gpio_clear));
  std::shared_ptr<HardwareGpioWriter> writer;
  return writer;
}

}  // namespace hyped::io
