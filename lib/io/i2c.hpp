#include <cstdint>
#include <cstdio>

namespace hyped::io {

class I2c {
 public:
  I2c(const uint8_t bus_address);
  ~I2c();

  int readData(const uint32_t address, uint8_t *data, const size_t len);
  int writeData(const uint32_t address, uint8_t *data, const size_t len);

 private:
  void setSensorAddress(uint32_t address);

 private:
  int fd_;
  uint32_t sensor_address_;
};

}  // namespace hyped::io