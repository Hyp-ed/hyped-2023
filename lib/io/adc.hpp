#pragma once

#include <cstdint>
#include <vector>
#include <cstdio>

namespace hyped::io {
class Adc {
 public:
  /**
   * @param pin
   */
  explicit Adc(const uint32_t pin);

  /**
   * @brief reads AIN value from file system
   *
   * @return uint16_t return two bytes for [0,4095] range 
   *         because the BBB has 12-bit ADCs (2^12 = 4096)
   */
  uint16_t read();


  /**
   * @param fd is a file path to where we read the raw values from
   * @return   uint16_t returns two bytes of current voltage data
   */ 
  uint16_t readHelper(int fd); 

 private:
  uint32_t pin_;
  int file_;
};
}  // namespace hyped::io
