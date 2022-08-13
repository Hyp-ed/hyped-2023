#pragma once

#include <navigation/navigator.hpp>

namespace hyped::helper {

class NaiveNavigator : public navigation::INavigator<navigation::Trajectory> {
 public:
  NaiveNavigator();
  virtual navigation::Trajectory currentTrajectory();
  virtual void update(const navigation::Trajectory &current_trajectory);

 private:
  navigation::Trajectory current_trajectory_;
};

}  // namespace hyped::helper
