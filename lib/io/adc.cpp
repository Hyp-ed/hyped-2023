#include "adc.hpp"

#include <fcntl.h>   // define O_WONLY and O_RDONLY
#include <unistd.h>  // close()

namespace hyped::io {

Adc::Adc(const std::uint8_t pin, core::ILogger &logger) : pin_(pin), log_(logger)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "/sys/bus/iio/devices/iio:device0/in_voltage%i_raw", pin_);
  file_ = open(buf, O_RDONLY);
  if (file_ < 0) { log_.log(core::LogLevel::kFatal, "Unable to open ADC file"); }
}

Adc::~Adc()
{
  close(file_);
}

std::optional<std::uint16_t> Adc::readValue()
{
  const std::optional<std::uint16_t> raw_voltage = resetAndRead4(file_);
  if (raw_voltage) {
    log_.log(core::LogLevel::kDebug, "Raw voltage: %i", raw_voltage.value());
    return *raw_voltage;
  }
  return std::nullopt;
}

std::optional<std::uint16_t> Adc::resetAndRead4(const int file_descriptor)
{
  const auto offset = lseek(file_descriptor, 0, SEEK_SET);  // reset file pointer
  if (offset != 0) {
    log_.log(core::LogLevel::kFatal, "Failed to reset file offset");
    return std::nullopt;
  }
  char read_buffer[4];  // buffer size 4 for fs value
  const int num_bytes_read
    = read(file_descriptor,
           &read_buffer,
           sizeof(read_buffer));  // actually consume new data, changes value in buffer
  if (num_bytes_read != sizeof(read_buffer)) {
    log_.log(core::LogLevel::kFatal, "Failed to read 4 bytes");

    return std::nullopt;  // returning NULL since we did not get any value
  }
  const int raw_voltage = std::atoi(read_buffer);
  return static_cast<std::uint16_t>(raw_voltage);  // max value is 2^12-1 = 4095
}

}  // namespace hyped::io
