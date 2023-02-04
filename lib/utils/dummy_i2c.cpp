#include "dummy_i2c.hpp"

namespace hyped::utils {

DummyI2c::DummyI2c(ReadHandler read_handler,
                   WriteByteHandler write_byte_handler,
                   WriteByteToRegisterHandler write_byte_to_register_handler)
    : read_handler_(read_handler),
      write_byte_handler_(write_byte_handler),
      write_byte_to_register_handler_(write_byte_to_register_handler)
{
}

std::optional<std::uint8_t> DummyI2c::readByte(const std::uint8_t device_address,
                                               const std::uint8_t register_address)
{
  return read_handler_(device_address, register_address);
}

core::Result DummyI2c::writeByteToRegister(const std::uint8_t device_address,
                                           const std::uint8_t register_address,
                                           const std::uint8_t data)
{
  return write_byte_to_register_handler_(device_address, register_address, data);
}

core::Result DummyI2c::writeByte(const std::uint8_t device_address, const std::uint8_t data)
{
  return write_byte_handler_(device_address, data);
}

}  // namespace hyped::utils
