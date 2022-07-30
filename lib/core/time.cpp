#include "time.hpp"

namespace hyped::core {

Time::Time()
{
}

TimePoint Time::now() const
{
  return std::chrono::high_resolution_clock::now();
}

}  // namespace hyped::core
