#include "preprocess_keyence.hpp"

namespace hyped::navigation {

PreprocessKeyence::PreprocessKeyence()
{
  // TODO: implement
}

SensorChecks checkKeyenceAgrees(const core::KeyenceData keyence_data)
{
  /*
   TODO: implement:
   roughly:
   - if keyence data disagrees and does_keyence_agree_ is false, fail state, return kUnacceptable
   - if keyence data disagrees and does_keyence_agree_ is true, set does_keyence_agree_ false return
   kAcceptable
   - if keyence data agrees and does_keyence_agree_ is false, et does_keyence_agree_ true return
   kAcceptable
   - if keyence data agrees and does_keyence_agree_ is true, all is good in the world, return
   kAcceptable.
  */
  return SensorChecks::kAcceptable;
}

}  // namespace hyped::navigation