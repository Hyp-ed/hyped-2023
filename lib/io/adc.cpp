#include "adc.hpp"

#include <fcntl.h>   // define O_WONLY and O_RDONLY
#include <unistd.h>  // close()

namespace hyped::io {
    
Adc::Adc(const uint32_t pin) : pin_(pin)
{
  char buf[100];
  snprintf(buf, sizeof(buf), "/sys/bus/iio/devices/iio:device0/in_voltage%i_raw", pin_);
  file_ = open(buf, O_RDONLY);
  if (file_ < 0) { /* TODO: log that there is a problem reading raw voltage for pin_*/ ;}
  /* TODO: log the file descriptor */
}

Adc::~Adc()
{
  close(file_); 
}

std::optional<uint16_t> Adc::read()
{
  const std::optional<uint16_t> val = resetAndRead4(file_);
  if(val) 
  {
      /* TODO: log the value */ 
      return *val;
  }
  return std::nullopt; 
}

std::optional<uint16_t> resetAndRead4(int file_descriptor)
{
  char buf[4];                                                 // buffer size 4 for fs value
  lseek(file_descriptor, 0, SEEK_SET);                         // reset file pointer
  int bytes_read = read(file_descriptor, &buf, sizeof(buf));   // actually consume new data, changes value in buffer
  if(bytes_read != sizeof(buf)) 
  {
    /*TODO: log that we failed in reading 4 bytes or reached EOF*/
    return std::nullopt;                                       // returning NULL since we did not get any value
  }
  return std::atoi(buf);
}

}  // namespace hyped::io