#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "core/types.hpp"
#include <eigen3/Eigen/Dense>

namespace hyped::navigation {

constexpr std::size_t state_dimension       = 3;
constexpr std::size_t measurement_dimension = 1;
constexpr std::size_t extended_dimension    = 2;
// in order acc, velocity, displacement
using StateVector           = Eigen::Matrix<core::Float, state_dimension, 1>;
using StateTransitionMatrix = Eigen::Matrix<core::Float, state_dimension, state_dimension>;
using StateTransitionCovarianceMatrix
  = Eigen::Matrix<core::Float, state_dimension, state_dimension>;
using ErrorCovarianceMatrix = Eigen::Matrix<core::Float, state_dimension, state_dimension>;
using MeasurementMatrix
  = Eigen::Matrix<core::Float, measurement_dimension, state_dimension>;  //[[1, 0, 0], [0, 0, 0],
                                                                         //[0, 0, 0]]
using MeasurementVector = Eigen::Matrix<core::Float, measurement_dimension, 1>;  //[acc_val, 0, 0]
using MeasurementNoiseCovarianceMatrix
  = Eigen::Matrix<core::Float, measurement_dimension, measurement_dimension>;
using ExtendedStateVector
  = Eigen::Matrix<core::Float, extended_dimension, 1>;  //[Jerk, Snap, Crackle, Pop, ...]
using JacobianMatrix
  = Eigen::Matrix<core::Float, extended_dimension, state_dimension>;  // higher order derivatives
                                                                      // for propogation

static constexpr core::Float kTrackLength       = 100.0;  // m
static constexpr core::Float kBrakingDistance   = 20.0;   // m TODOLater:check!
static constexpr core::Float kPi                = 3.14159265359;
static constexpr core::Float kWheelCicumference = kPi * 0.1;  // m TODOLater: check!
static constexpr core::Float kStripeDistance    = 10.0;       // m TODOLater:check!

// define sensor checks return type
enum class SensorChecks { kUnacceptable = 0, kAcceptable };

struct Quartiles {
  core::Float q1;
  core::Float median;
  core::Float q3;
};

struct HigherDerivatives {
  core::Float jerk;
  core::Float snap;
};

inline core::Trajectory zero_trajectory = {0, 0, 0};

class INavigator {
 public:
  virtual std::optional<core::Trajectory> currentTrajectory()                            = 0;
  virtual void keyenceUpdate(const core::KeyenceData &keyence_data)                      = 0;
  virtual void encoderUpdate(const core::EncoderData &encoder_data)                      = 0;
  virtual void accelerometerUpdate(const core::RawAccelerometerData &accelerometer_data) = 0;
};
}  // namespace hyped::navigation