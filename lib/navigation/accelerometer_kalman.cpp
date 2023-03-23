#include "accelerometer_kalman.hpp"

namespace hyped::navigation {

AccelerometerKalman::AccelerometerKalman(core::ILogger &logger, const core::ITimeSource &time)
    : logger_(logger),
      time_(time)
{
  // TODO: instantiate kalman
}

// TODO: implement this to return the matrix as a fuction of time_delta.
void AccelerometerKalman::getStateTransitionMatrix(core::Duration time_delta)
{
  // TODO: implement this once figuring out dimensions and jerk
  return;
}

void AccelerometerKalman::getMeasurementVector(core::Float acceleration)
{
  // TODO: implement
  return;
}

void AccelerometerKalman::getStateTransitionCovarianceMatrix()
{
  // TODO: implement
  return;
}

void AccelerometerKalman::getMeasurementNoiseCovarianceMatrix()
{
  // TODO: implement
  return;
}

core::Float AccelerometerKalman::filter()
{
  // TODO: run getter functions to get matrices, run kalman filter and
  // return kalman_filter.getStateEstimate()
  return 0.0F;
}

}  // namespace hyped::navigation