#include "types.hpp"

#include <array>
#include <cstdint>

#include "core/types.hpp"

namespace hyped::navigation {

class EncodersPreprocessor {
 public:
  EncodersPreprocessor();

  std::optional<EncoderData> processData(const core::RawEncoderData &encoder_data);

 private:
  EncoderData detectOutliers(const core::RawEncoderData &encoder_data);

  void checkReliable(const core::RawEncoderData &raw_encoder_data);

  // initialised as {0, 0, 0, 0}, count of consecutive outliers
  std::array<std::uint16_t, core::kNumEncoders> num_outliers_per_encoder_;

  // initialised as all true, bool mask of reliable sensors
  std::array<bool, core::kNumEncoders> are_encoders_reliable_;
};

}  // namespace hyped::navigation
