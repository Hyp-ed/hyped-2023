#include "i2c.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <sys/ioctl.h>

#if LINUX
#include <linux/i2c-dev.h>
#else
#define I2C_SLAVE 0x0703  // To specify that we are making I2C transactions
#endif

namespace hyped::io {

I2c::I2c(const uint8_t bus_address) : sensor_address_(0)
{
  char path[13];  // up to "/dev/i2c-2"
  sprintf(path, "/dev/i2c-%d", bus_address);
  file_descriptor_ = open(path, O_RDWR, 0);
  if (file_descriptor_ < 0) { /* log "Could not open i2c device" */ };
}

I2c::~I2c()
{
  close(file_descriptor_);
}

void I2c::setSensorAddress(const uint8_t device_address)
{
  if (file_descriptor_ < 0) {
    /* log "Could not find i2c device while setting sensor address"*/;
    return;
  }
  sensor_address_        = device_address;
  const int return_value = ioctl(file_descriptor_, I2C_SLAVE, device_address);
  if (return_value < 0) {
    /*log "Could not set sensor address" */;
    return;
  }
}

std::optional<uint8_t> I2c::readByte(const uint8_t device_address, const uint8_t register_address)
{
  if (file_descriptor_ < 0) {
    /* log "Could not find i2c device while reading"*/;
    return std::nullopt;
  }
  if (sensor_address_ != device_address) { setSensorAddress(device_address); }
  // Contains data which we will read
  uint8_t read_buffer[1];
  // Contains data which we need to write
  const uint8_t write_buffer[1] = {register_address};
  // Writing the register address so we switch to it
  const int num_bytes_written = write(file_descriptor_, write_buffer, 1);
  if (num_bytes_written != 1) {
    /* log "could not write to i2c device"*/
    return std::nullopt;
  }
  const int num_bytes_read = read(file_descriptor_, read_buffer, 1);
  if (num_bytes_read != 1) {
    /*log "could not read from i2c device"*/
    return std::nullopt;
  }
  return read_buffer[0];
}

I2cWriteResult I2c::writeByte(const uint8_t device_address, const uint8_t register_address,
                              const uint8_t data)
{
  if (file_descriptor_ < 0) {
    /*log "Could not find i2c device while writing"*/;
    return I2cWriteResult::kFailure;
  }
  if (sensor_address_ != device_address) { setSensorAddress(device_address); }
  const uint8_t write_buffer[2] = {register_address, data};
  const auto num_bytes_written  = write(file_descriptor_, write_buffer, 2);
  if (num_bytes_written != 2) {
    /* log "Could not write to i2c device" */;
    return I2cWriteResult::kFailure;
  }
  return I2cWriteResult::kSuccess;
}

}  // namespace hyped::io
