#pragma once

#include <navigation/navigator.hpp>

namespace hyped::helper {

class NaiveNavigator : public navigator::INavigator<navigator::Trajectory> {
 public:
  NaiveNavigator();
  virtual navigator::Trajectory currentTrajectory();
  virtual void update(const navigator::Trajectory &current_trajectory);

 private:
  navigator::Trajectory current_trajectory_;
};

}  // namespace hyped::helper
