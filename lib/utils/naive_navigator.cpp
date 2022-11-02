#include "naive_navigator.hpp"

namespace hyped::utils {

NaiveNavigator::NaiveNavigator() : current_trajectory_{}
{
}

core::Trajectory NaiveNavigator::currentTrajectory()
{
  return current_trajectory_;
}

void NaiveNavigator::update(const core::Trajectory &current_trajectory)
{
  current_trajectory_ = current_trajectory;
}

}  // namespace hyped::utils
