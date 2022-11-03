#include <array>
#include <cstdint>

#include "core/types.hpp"

namespace hyped::navigation {
class PreprocessEncoders {
 public:
  PreprocessEncoders();

  core::EncoderData processData(const core::EncoderData encoder_data);

 private:
  core::EncoderData encodersOutlierDetection(const core::EncoderData encoder_data);

  void checkEncodersReliable(const core::EncoderData encoder_data);

  // initialised as {0, 0, 0, 0}, count of consecutive outliers
  std::array<uint16_t, core::kNumEncoders> outlier_encoders_;

  // initialised as all true, bool mask of reliable sensors
  std::array<bool, core::kNumEncoders> reliable_encoders_;
};

}  // namespace hyped::navigation