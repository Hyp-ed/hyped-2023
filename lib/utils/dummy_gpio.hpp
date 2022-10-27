#pragma once

#include <functional>

#include <io/gpio.hpp>

namespace hyped::utils {

class DummyGpioReader : public io::IGpioReader {
 public:
  using ReadHandler = std::function<std::optional<core::DigitalSignal>(const std::uint8_t pin)>;
  virtual std::optional<core::DigitalSignal> read();

 private:
  DummyGpioReader(const std::uint8_t pin, ReadHandler read_handler);
  friend class DummyGpio;

  const std::uint8_t pin_;
  ReadHandler read_handler_;
};

class DummyGpioWriter : public io::IGpioWriter {
 public:
  using WriteHandler
    = std::function<io::GpioWriteResult(const std::uint8_t pin, const core::DigitalSignal state)>;
  virtual io::GpioWriteResult write(const core::DigitalSignal state);

 private:
  DummyGpioWriter(const std::uint8_t pin, WriteHandler write_handler);
  friend class DummyGpio;

  const std::uint8_t pin_;
  WriteHandler write_handler_;
};

class DummyGpio : public io::IGpio {
 public:
  DummyGpio();

  std::optional<std::shared_ptr<io::IGpioReader>> getReader(const std::uint8_t pin);
  std::optional<std::shared_ptr<io::IGpioWriter>> getWriter(const std::uint8_t pin);
  DummyGpioReader::ReadHandler read_handler_;
  DummyGpioWriter::WriteHandler write_handler_;
};

}  // namespace hyped::utils
