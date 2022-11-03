#include <cstdint>
#include <cstdio>
#include <optional>

#include <core/logger.hpp>

namespace hyped::io {

enum class I2cWriteResult { kSuccess, kFailure };

class I2c {
 public:
  I2c(const uint8_t bus_address, hyped::core::ILogger &log);
  ~I2c();

  std::optional<uint8_t> readByte(const uint8_t device_address, const uint8_t register_address);
  I2cWriteResult writeByte(const uint8_t device_address,
                           const uint8_t register_address,
                           uint8_t data);

 private:
  void setSensorAddress(uint8_t device_address);

 private:
  int file_descriptor_;
  uint8_t sensor_address_;
  hyped::core::ILogger &log_;
};

}  // namespace hyped::io
