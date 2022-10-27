#include <cstdint>

#include <core/logger.hpp>

namespace hyped::io {

enum class Polarity { kActiveHigh = 0, kActiveLow };

class Pwm {
 public:
  Pwm(const std::string device, hyped::core::ILogger &log);
  ~Pwm();

  void enable();
  void disable();

  void setFrequency(uint16_t frequency);
  void setDutyCycle(uint8_t duty_cycle);
  void setPolarity(Polarity polarity);

 private:
  hyped::core::ILogger &log_;
  float frequency_;  // Hz
  float analogMax_;  // V
};
}  // namespace hyped::io