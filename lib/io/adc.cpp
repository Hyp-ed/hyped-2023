#include "adc.hpp"

#include <fcntl.h>
#include <unistd.h>

namespace hyped::io {

Adc::Adc(core::ILogger &logger, const std::uint8_t pin) : logger_(logger), pin_(pin)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "/sys/bus/iio/devices/iio:device0/in_voltage%i_raw", pin_);
  file_ = open(buf, O_RDONLY);
  if (file_ < 0) { logger_.log(core::LogLevel::kFatal, "Unable to open ADC file"); }
}

Adc::~Adc()
{
  close(file_);
}

std::optional<std::uint16_t> Adc::readValue()
{
  if (file_ < 0) {
    logger_.log(core::LogLevel::kFatal, "Unable to find ADC file during read");
    return std::nullopt;
  }
  const std::optional<std::uint16_t> raw_voltage = resetAndRead4(file_);
  if (raw_voltage) {
    logger_.log(
      core::LogLevel::kDebug, "Raw voltage from ADC pin %d: %i", pin_, raw_voltage.value());
    return *raw_voltage;
  }
  return std::nullopt;
}

std::optional<std::uint16_t> Adc::resetAndRead4(const int file_descriptor)
{
  const auto offset = lseek(file_descriptor, 0, SEEK_SET);  // reset file pointer
  if (offset != 0) {
    logger_.log(core::LogLevel::kFatal, "Failed to reset file offset");
    return std::nullopt;
  }
  char read_buffer[4];  // buffer size 4 for fs value
  const int num_bytes_read = read(file_descriptor, &read_buffer, sizeof(read_buffer));
  if (num_bytes_read < 2) {  // 2 bytes minimum as value ranges from [0, 4095]
    logger_.log(core::LogLevel::kFatal, "Failed to read sufficient bytes from ADC");
    return std::nullopt;
  }
  const int raw_voltage = std::atoi(read_buffer);
  return static_cast<std::uint16_t>(raw_voltage);  // max value is 2^12-1 = 4095
}

}  // namespace hyped::io
