#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>  // for atoi

namespace hyped::io {

class Adc {
 public:
  /**
   * @param pin
   */
  Adc(const uint32_t pin);

  /**
   * @brief reads AIN value from file system
   *
   * @return uint16_t return two bytes for [0,4095] range 
   *         because the BBB has 12-bit ADCs (2^12 = 4096)
   */
  uint16_t read();


  /**
   * @param    file_descriptor specifying the file voltage values are read from
   * @return   uint16_t returns two bytes of current voltage data
   */ 
  uint16_t resetAndRead4(const int file_descriptor); 

 private:
  uint32_t pin_;
  int file_;
};
}  // namespace hyped::io
