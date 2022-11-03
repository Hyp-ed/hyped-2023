#pragma once

#include "core/types.hpp"
#include <navigation/navigator.hpp>

namespace hyped::utils {

class NaiveNavigator : public navigation::INavigator<core::Trajectory> {
 public:
  NaiveNavigator();
  virtual core::Trajectory currentTrajectory();
  virtual void update(const core::Trajectory &current_trajectory);

 private:
  core::Trajectory current_trajectory_;
};

}  // namespace hyped::utils
