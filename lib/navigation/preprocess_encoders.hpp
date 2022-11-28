#include "consts.hpp"

#include <array>
#include <cstdint>

#include "core/types.hpp"

namespace hyped::navigation {
class EncodersPreprocessor {
 public:
  EncodersPreprocessor();

  core::EncoderData processData(const core::EncoderData encoder_data);

 private:
  core::EncoderData detectOutliers(const core::EncoderData encoder_data);
  std::uint8_t max_consecutives = 10;

  void checkReliable();

  template<std::size_t N>
  Quartile getQuartiles(const std::array<std::uint32_t, N> &encoder_data);

  // number of reliable encoders, initialised as core::kNumEncoders
  std::uint8_t num_reliable_encoders_;

  // initialised as {0, 0, 0, 0}, count of consecutive outliers
  std::array<uint16_t, core::kNumEncoders> num_outliers_per_encoder_;

  // initialised as all true, bool mask of reliable sensors
  std::array<bool, core::kNumEncoders> are_encoders_reliable_;
};

}  // namespace hyped::navigation