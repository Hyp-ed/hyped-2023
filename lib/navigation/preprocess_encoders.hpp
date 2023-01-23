#include <array>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <optional>

#include "consts.hpp"
#include "core/types.hpp"
#include "core/logger.hpp"


namespace hyped::navigation {
class EncodersPreprocessor {
 public:
  EncodersPreprocessor(core::ILogger &logger);

  /** TODO
   * @brief
   * 
   * @param encoder_data
   * @return
  */
  std::optional<core::EncoderData> processData(const core::EncoderData encoder_data);

 private:
 
  /** TODO
   * @brief tidies up the received data by detecting the outliers or data received from faulty 
     sensors and replaces them with median value of the dataset.
   *
   * @param encoder_data array containing the values received from various encoder sensors
   * @return consistent data with no outliers
   */
  std::optional<core::EncoderData> detectOutliers(const core::EncoderData encoder_data);
  
  /** TODO
   * @brief
   * 
   * @return
  */
  SensorChecks checkReliable();

  /** TODO
   * @brief
   * 
   * @param reliable_data
   * @param quartile_percent
   * @return 
  */
  template<std::size_t N>
  core::Float getSpecificQuartile(const std::array<std::uint32_t, N> &reliable_data, const core::Float quartile_percent){

    const core::Float quartile_index = (num_reliable_encoders_ - 1)*quartile_percent;
    const std::uint8_t quartile_high = static_cast<std::uint8_t>(std::ceil(quartile_index));
    const std::uint8_t quartile_low = static_cast<std::uint8_t>(std::floor(quartile_index));
    const core::Float quartile = (reliable_data.at(quartile_high) + reliable_data.at(quartile_low))/2.0;
    return quartile;
}

/** TODO
 * @brief
 * 
 * @param reliable_data
 * @return
*/
 template<std::size_t N>
 Quartile getQuartiles(std::array<std::uint32_t, N> &reliable_data){
    std::sort(reliable_data.begin(), reliable_data.end());
    return {.q1     = getSpecificQuartile(reliable_data, 0.25),
            .median = getSpecificQuartile(reliable_data, 0.5),
            .q3     = getSpecificQuartile(reliable_data, 0.75)};
}

  // number of reliable encoders, initialised as core::kNumEncoders
  std::uint8_t num_reliable_encoders_;

  // initialised as {0, 0, 0, 0}, count of consecutive outliers
  std::array<uint16_t, core::kNumEncoders> encoder_outliers_;

  // initialised as all true, bool mask of reliable sensors
  std::array<bool, core::kNumEncoders> encoders_reliable_;

  //TODO: comment use
  std::uint8_t max_consecutive_outliers_ = 10;

  //TODO: comment use
  core::ILogger &logger_;
};

}  // namespace hyped::navigation