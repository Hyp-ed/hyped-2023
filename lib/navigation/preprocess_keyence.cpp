#include "preprocess_keyence.hpp"

#include <cstdint>

namespace hyped::navigation {

KeyencePreprocessor::KeyencePreprocessor(core::ILogger &logger)
    : log_(logger),
      previous_data_status_(KeyenceDataStatus::kAgreed)
{
}

SensorDisagreement KeyencePreprocessor::checkKeyenceAgrees(const KeyenceData &keyence_data)
{
  KeyenceDataStatus current_data_status = KeyenceDataStatus::kAgreed;
  for (std::size_t i = 1; i < keyence_data.value.size(); ++i) {
    if (keyence_data.value.at(0) != keyence_data.value.at(i)) {
      current_data_status = KeyenceDataStatus::kDisagreed;
      break;
    }
  }
  if (current_data_status == KeyenceDataStatus::kDisagreed
      && previous_data_status_ == KeyenceDataStatus::kDisagreed) {
    log_.log(core::LogLevel::kFatal, "Keyence disagreement for two consecutive readings.");
    return SensorDisagreement::kUnacceptable;
  }
  previous_data_status_ = current_data_status;
  return SensorDisagreement::kAcceptable;
}

}  // namespace hyped::navigation
