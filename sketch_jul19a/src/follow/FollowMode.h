#pragma once

namespace follow_demo {

// Follow mode selection is kept separate from tuning parameters.
// Change only this block when switching behavior, not when tuning thresholds.
#define BU04_FOLLOW_MODE_STANDARD_EMA 0  // Standard follow with clamp + EMA.
#define BU04_FOLLOW_MODE_STANDARD_KALMAN 1  // Standard follow with Kalman.
#define BU04_FOLLOW_MODE_LATEST_ANGLE_DISPATCH 2  // Independent 5s latest-angle dispatch.

#define BU04_FOLLOW_MODE BU04_FOLLOW_MODE_STANDARD_KALMAN  // Current follow mode.

#define BU04_FOLLOW_ENABLE_LATEST_ANGLE_DISPATCH \
  (BU04_FOLLOW_MODE == BU04_FOLLOW_MODE_LATEST_ANGLE_DISPATCH)
#define BU04_FOLLOW_ENABLE_KALMAN_FILTER \
  (BU04_FOLLOW_MODE == BU04_FOLLOW_MODE_STANDARD_KALMAN)

}  // namespace follow_demo
