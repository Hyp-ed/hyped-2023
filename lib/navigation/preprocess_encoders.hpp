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

  void checkReliable(const core::EncoderData &encoder_data);

  // initialised as {0, 0, 0, 0}, count of consecutive outliers
  std::array<uint16_t, core::kNumEncoders> num_outliers_per_encoder_;

  // initialised as all true, bool mask of reliable sensors
  std::array<bool, core::kNumEncoders> is_encoder_reliable_;
};

}  // namespace hyped::navigation