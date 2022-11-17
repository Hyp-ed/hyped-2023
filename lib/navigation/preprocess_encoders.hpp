#include <array>
#include <cstdint>

#include "core/types.hpp"

namespace hyped::navigation {
class EncodersPreprocessor {
 public:
  EncodersPreprocessor();

  core::EncoderData processData(const core::EncoderData encoder_data);

 private:
 //extern core::EncoderData reliability_of_encoders;      // some addition
 //extern core::EncoderData num_outliers_per_encoder;
  core::EncoderData detectOutliers(const core::EncoderData encoder_data);
  
  //void checkReliable(const core::EncoderData &encoder_data);
  void checkReliable(const core::EncoderData num_outliers_per_encoder);

  //template<typename T>
  //std::array<core::Float,3> quartiles(T encoder_data);

  template<std::size_t N>
  Quartile getQuartiles(std::array<std::uint32_t , N> & encoder_data);



  // initialised as {0, 0, 0, 0}, count of consecutive outliers
  std::array<uint16_t, core::kNumEncoders> num_outliers_per_encoder_;

  // initialised as all true, bool mask of reliable sensors
  std::array<bool, core::kNumEncoders> are_encoders_reliable_;
};

}  // namespace hyped::navigationcore::Float calculating_median(const core::EncoderData encoder_data)