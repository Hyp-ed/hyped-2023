#include "adc.hpp"

#include <fcntl.h>   // define O_WONLY and O_RDONLY
#include <unistd.h>  // close()

namespace hyped::io {

Adc::Adc(const uint32_t pin) : pin_(pin)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "/sys/bus/iio/devices/iio:device0/in_voltage%i_raw", pin_);
  file_ = open(buf, O_RDONLY);
  if (file_ < 0) { /* TODO: log that there is a problem reading raw voltage for pin_*/; }
}

Adc::~Adc()
{
  close(file_);
}

std::optional<uint16_t> Adc::read()
{
  const std::optional<uint16_t> raw_voltage = resetAndRead4(file_);
  if (raw_voltage) {
    /* TODO: log the value */
    return *raw_voltage;
  }
  return std::nullopt;
}

std::optional<uint16_t> resetAndRead4(const int file_descriptor)
{
  const auto offset = lseek(file_descriptor, 0, SEEK_SET);    // reset file pointer
  if (offset != 0) {
    /*TODO: log that we failed in resetting file_descriptor's file offset*/
    return std::nullopt;
  }
  char read_buffer[4];                                        // buffer size 4 for fs value
  const int num_bytes_read = read(file_descriptor, &read_buffer,
                              sizeof(read_buffer));           // actually consume new data, changes value in buffer
  if (num_bytes_read != sizeof(read_buffer)) {
    /*TODO: log that we failed in reading 4 bytes or reached EOF*/
    return std::nullopt;  // returning NULL since we did not get any value
  }
  const int raw_voltage = std::atoi(read_buffer);
  return static_cast<uint16_t>(raw_voltage);  // max value is 2^12-1 = 4095
}

}  // namespace hyped::io