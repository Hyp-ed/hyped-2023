#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "core/types.hpp"

namespace hyped::navigation {

static constexpr core::Float kTrackLength       = 100.0;  // m
static constexpr core::Float kBrakingDistance   = 20.0;   // m TODOLater:check!
static constexpr core::Float kPi                = 3.14159265359;
static constexpr core::Float kWheelCicumference = kPi * 0.1;  // m TODOLater: check!
static constexpr core::Float kStripeDistance    = 10.0;       // m TODOLater:check!

static constexpr core::Trajectory kZeroTrajectory = {0, 0, 0};

}  // namespace hyped::navigation
