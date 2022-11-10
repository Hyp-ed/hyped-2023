#include "preprocess_keyence.hpp"

#include <cstdint>

namespace hyped::navigation {

KeyencePreprocessor::KeyencePreprocessor(hyped::core::ILogger &logger) : log_(logger)
{
  // Currently no extra parameters so constructor is empty
  has_keyence_disagreed_ = false;
}

SensorChecks KeyencePreprocessor::checkKeyenceAgrees(const core::KeyenceData &keyence_data)
{
  /***/
  bool keyence_data_disagrees = false;
  for (std::size_t i = 0; i < keyence_data.size() - 1; ++i) {
    if (keyence_data.at(i) != keyence_data.at(i + 1)) { keyence_data_disagrees = true; }
  }

  if (keyence_data_disagrees && has_keyence_disagreed_) {
    log_.log(hyped::core::LogLevel::kFatal, "Keyence disagreed more than once");
    return SensorChecks::kUnacceptable;

  } else if (keyence_data_disagrees && !(has_keyence_disagreed_)) {
    has_keyence_disagreed_ = true;

  } else if (!(keyence_data_disagrees) && has_keyence_disagreed_) {
    has_keyence_disagreed_ = false;

  } else {
    return SensorChecks::kAcceptable;
  }
}

}  // namespace hyped::navigation