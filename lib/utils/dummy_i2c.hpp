#pragma once

#include <functional>

#include <io/i2c.hpp>

namespace hyped::utils {

/**
 * A basic implementation of II2c that does not require any hardware. The purpose of this
 * class is to
 *
 *  1. demonstrate how to implement the II2c interface, and
 *  2. allow for quick test implementations.
 */
class DummyI2c : public io::II2c {
 public:
  using ReadHandler = std::function<std::optional<std::uint8_t>(
    const std::uint8_t device_address, const std::uint8_t register_address)>;
  using WriteByteHandler
    = std::function<core::Result(const std::uint8_t device_address, const std::uint8_t data)>;
  using WriteByteToRegisterHandler = std::function<core::Result(const std::uint8_t device_address,
                                                                const std::uint8_t register_address,
                                                                const std::uint8_t data)>;

  DummyI2c(ReadHandler read_handler,
           WriteByteHandler write_byte_handler,
           WriteByteToRegisterHandler write_byte_to_register_handler);

  std::optional<std::uint8_t> readByte(const std::uint8_t device_address,
                                       const std::uint8_t register_address);
  core::Result writeByteToRegister(const std::uint8_t device_address,
                                   const std::uint8_t register_address,
                                   const std::uint8_t data);
  core::Result writeByte(const std::uint8_t device_address, const std::uint8_t data);

 private:
  ReadHandler read_handler_;
  WriteByteHandler write_byte_handler_;
  WriteByteToRegisterHandler write_byte_to_register_handler_;
};

}  // namespace hyped::utils
