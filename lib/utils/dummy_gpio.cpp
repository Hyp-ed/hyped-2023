
#include "dummy_gpio.hpp"

namespace hyped::utils {

DummyGpioReader::DummyGpioReader(const std::uint8_t pin, DummyGpioReader::ReadHandler read_handler)
    : read_handler_(read_handler),
      pin_(pin)
{
}

std::optional<core::DigitalSignal> DummyGpioReader::read()
{
  return read_handler_(pin_);
}

DummyGpioWriter::DummyGpioWriter(const std::uint8_t pin,
                                 DummyGpioWriter::WriteHandler write_handler)
    : write_handler_(write_handler),
      pin_(pin)
{
}

io::GpioWriteResult DummyGpioWriter::write(const core::DigitalSignal state)
{
  return write_handler_(pin_, state);
}

DummyGpio::DummyGpio()
{
}

std::optional<std::shared_ptr<io::IGpioReader>> DummyGpio::getReader(const uint8_t pin)
{
  return std::make_shared<DummyGpioReader>(DummyGpioReader(pin, read_handler_));
}

std::optional<std::shared_ptr<io::IGpioWriter>> DummyGpio::getWriter(const uint8_t pin)
{
  return std::make_shared<DummyGpioWriter>(DummyGpioWriter(pin, write_handler_));
}

}  // namespace hyped::utils
