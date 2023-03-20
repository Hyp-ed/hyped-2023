#include "accelerometer_kalman.hpp"

namespace hyped::navigation {


// TODO: implement this to return the matrix as a fuction of time_delta.
Eigen::Matrix<core::Float, 3, 3> AccelerometerKalman::getStateTransitionMatrix(
  core::Duration time_delta)
{
  // so it compiles
  Eigen::Matrix<core::Float, 3, 3> x;
  x(0, 0) = 1;
  x(0, 1) = 0;
  x(0, 2) = 0;
  x(1, 0) = time_delta.count();
  x(1, 1) = 1;
  x(1, 2) = 0;
  x(2, 0) = 0.5 * time_delta.count() * time_delta.count();
  x(2, 1) = time_delta.count();
  x(2, 2) = 1;
  return x;
}

}  // namespace hyped::navigation