#include "hardware_adc.hpp"

#include <fcntl.h>
#include <unistd.h>

namespace hyped::io {

std::optional<std::shared_ptr<HardwareAdc>> HardwareAdc::create(core::ILogger &logger,
                                                                const std::uint8_t pin)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "/sys/bus/iio/devices/iio:device0/in_voltage%i_raw", pin);
  const int file_descriptor = open(buf, O_RDONLY);
  if (file_descriptor < 0) {
    logger.log(core::LogLevel::kFatal, "Failed to open ADC file");
    return std::nullopt;
  }
  logger.log(core::LogLevel::kDebug, "Successfully created Adc instance");
  return std::make_shared<HardwareAdc>(logger, file_descriptor);
}

HardwareAdc::HardwareAdc(core::ILogger &logger, const int file_descriptor)
    : logger_(logger),
      file_descriptor_(file_descriptor)
{
}

HardwareAdc::~HardwareAdc()
{
  close(file_descriptor_);
}

std::optional<std::uint16_t> HardwareAdc::readValue()
{
  const std::optional<std::uint16_t> raw_voltage = resetAndRead4(file_descriptor_);
  if (raw_voltage) {
    logger_.log(
      core::LogLevel::kDebug, "Raw voltage from ADC pin %d: %i", pin_, raw_voltage.value());
    return *raw_voltage;
  }
  return std::nullopt;
}

std::optional<std::uint16_t> HardwareAdc::resetAndRead4(const int file_descriptor)
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
  // convert raw voltage to voltage between [0, 1.8] in V
  return static_cast<std::uint16_t>((raw_voltage / kMaxAdcRawValue) * kMaxAdcVolatage);
}

}  // namespace hyped::io
