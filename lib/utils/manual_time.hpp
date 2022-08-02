#include <core/time.hpp>

namespace hyped::utils {

class ManualTime : public core::ITime {
 public:
  ManualTime();
  virtual core::TimePoint now() const;
  void set_time(const core::TimePoint time_point);

 private:
  core::TimePoint current_time_;
};

}  // namespace hyped::utils
