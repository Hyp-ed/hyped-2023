#include "naive_navigator.hpp"

namespace hyped::helper {

NaiveNavigator::NaiveNavigator() : current_trajectory_{}
{
}

navigation::Trajectory NaiveNavigator::currentTrajectory()
{
  return current_trajectory_;
}

void NaiveNavigator::update(const navigation::Trajectory &current_trajectory)
{
  current_trajectory_ = current_trajectory;
}

}  // namespace hyped::helper
