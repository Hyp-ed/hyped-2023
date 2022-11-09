#include "preprocess_keyence.hpp"

namespace hyped::navigation {

KeyencePreprocessor::KeyencePreprocessor(hyped::core::ILogger &logger) : logger_(logger)
{
  // Currently no extra parameters so constructor is empty
  has_keyence_disagreed_ = false;
}

SensorChecks KeyencePreprocessor::checkKeyenceAgrees(const core::KeyenceData &keyence_data)
{
  /*
   TODOLater: implement:
   roughly:
    if 
   - if keyence data disagrees and does_keyence_agree_ is false, fail state, return kUnacceptable

   - if keyence data disagrees and does_keyence_agree_ is true, set does_keyence_agree_ false return
   kAcceptable
   - if keyence data agrees and does_keyence_agree_ is false, et does_keyence_agree_ true return
   kAcceptable
   - if keyence data agrees and does_keyence_agree_ is true, all is good in the world, return
   kAcceptable.
  */
  bool keyence_data_disagrees = false;
  for (size_t i = 0; i < keyence_data.size() - 1; ++i){
    if (keyence_data.at(i) != keyence_data.at(i+1)){
      keyence_data_disagrees = true;
    }
  }
  if (keyence_data_disagrees && has_keyence_disagreed_){
    logger_.log(hyped::core::LogLevel::kFatal, "Keyence disagreed more than once");
    return SensorChecks::kUnacceptable;
  }
  else if (keyence_data_disagrees && !(has_keyence_disagreed_)){
    has_keyence_disagreed_ = true;
  }
  else if (!(keyence_data_disagrees) && has_keyence_disagreed_){
    has_keyence_disagreed_ = false;

  }
  return SensorChecks::kAcceptable;
}

}  // namespace hyped::navigation