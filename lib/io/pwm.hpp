#include <cstdint>

#include <core/logger.hpp>
#include <core/types.hpp>

namespace hyped::io {

enum class Polarity { kActiveHigh = 0, kActiveLow };

class Pwm {
 public:
  Pwm(const uint8_t channel, hyped::core::ILogger &log);
  ~Pwm();

  hyped::core::Result enable();
  hyped::core::Result disable();

  hyped::core::Result setFrequency(uint32_t frequency);
  std::optional<uint32_t> getFrequency();

  hyped::core::Result setDutyCycle(float duty_cycle);
  std::optional<float> getDutyCycle();

  hyped::core::Result setPolarity(Polarity polarity);
  std::optional<Polarity> getPolarity();

  hyped::core::Result setPeriod(uint32_t period);
  std::optional<uint32_t> getPeriod();

  hyped::core::Result run();

 private:
  hyped::core::ILogger &log_;
  float dutyCycle_; // Percentage
  float analogMax_;  // V
  std::string path_;

  int enableFd_;
  int dutyFd_;
  int periodFd_;
  int polarityFd_;
};
}  // namespace hyped::io