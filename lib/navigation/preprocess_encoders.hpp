#include "consts.hpp"

#include <array>
#include <cstdint>

#include "core/types.hpp"
#include "core/logger.hpp"

namespace hyped::navigation {
class EncodersPreprocessor {
 public:
  //EncodersPreprocessor();
  EncodersPreprocessor(core::ILogger &logger);

  std::optional<core::EncoderData> processData(const core::EncoderData encoder_data);

 private:
  core::EncoderData detectOutliers(const core::EncoderData encoder_data);
  std::uint8_t max_consecutives = 10;

  SensorChecks checkReliable();

  template<std::size_t N>
  Quartile getQuartiles(std::array<std::uint32_t, N> &reliable_data);

  template<std::size_t N>
  core::Float getQuartile(const std::array<std::uint32_t, N> &reliable_data, const core::Float quartile_percent);

  // number of reliable encoders, initialised as core::kNumEncoders
  std::uint8_t num_reliable_encoders_;

  // initialised as {0, 0, 0, 0}, count of consecutive outliers
  std::array<uint16_t, core::kNumEncoders> encoder_outliers;

  // initialised as all true, bool mask of reliable sensors
  std::array<bool, core::kNumEncoders> encoders_reliable;

  core::ILogger &logger_;

};

}  // namespace hyped::navigation