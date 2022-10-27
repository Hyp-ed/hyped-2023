#include <cstdint>

#include <core/logger.hpp>

namespace hyped::io {

enum class Polarity { kActiveHigh = 0, kActiveLow };

enum class PwmWriteResult { kSuccess = 0, kError };

class Pwm {
 public:
  Pwm(const uint8_t channel, hyped::core::ILogger &log);
  ~Pwm();

  PwmWriteResult enable();
  PwmWriteResult disable();

  PwmWriteResult setFrequency(uint16_t frequency);
  PwmWriteResult setDutyCycle(float duty_cycle);
  PwmWriteResult setPolarity(Polarity polarity);
  
 private:
  hyped::core::ILogger &log_;
  uint16_t frequency_;  // Hz
  float dutyCycle_; // Percentage
  float analogMax_;  // V
  std::string path_;

  int enableFd_;
  int dutyFd_;
  int periodFd_;
  int polarityFd_;
};
}  // namespace hyped::io