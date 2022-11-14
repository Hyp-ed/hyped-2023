#include "preprocess_keyence.hpp"

namespace hyped::navigation {

KeyencePreprocessor::KeyencePreprocessor()
{
  // TODOLater: implement
}

std::optional<KeyenceData> processData(const core::RawKeyenceData &raw_keyence_data)
{
  /*
   TODOLater: implement:
   roughly:
   - if keyence data disagrees and does_keyence_agree_ is false, fail state, return kUnacceptable
   - if keyence data disagrees and does_keyence_agree_ is true, set does_keyence_agree_ false return
   kAcceptable
   - if keyence data agrees and does_keyence_agree_ is false, et does_keyence_agree_ true return
   kAcceptable
   - if keyence data agrees and does_keyence_agree_ is true, all is good in the world, return
   kAcceptable.
  */
  return std::nullopt;
}

}  // namespace hyped::navigation
