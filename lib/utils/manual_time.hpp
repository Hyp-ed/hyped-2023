#include <core/time.hpp>

namespace hyped::utils {

class ManualTime : public core::ITimeSource {
 public:
  ManualTime();
  virtual core::TimePoint now() const;

  void setTime(const core::TimePoint time_point);
  void setSecondsSinceEpoch(const std::uint64_t seconds_since_epoch);
  void addTime(const core::Duration duration);
  void addSeconds(const std::uint64_t num_seconds);

 private:
  core::TimePoint current_time_;
};

}  // namespace hyped::utils
