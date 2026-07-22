#pragma once

#include <Arduino.h>
#include "FollowMode.h"

namespace follow_demo {

constexpr const char* kWifiSsid = "YQDL-Office-5G";
constexpr const char* kWifiPassword = "Yqdl@123";

inline IPAddress robotIp() {
  return IPAddress(192, 168, 0, 56);
}
constexpr uint16_t kRobotPort = 1448;

#define BU04_FOLLOW_SAMPLE_INTERVAL_MS 0  // Reserved; current follow path processes every valid frame.
#define BU04_FOLLOW_ACTION_CHECK_MS 50  // Action status poll interval, ms.
#define BU04_FOLLOW_WIFI_RECONNECT_MS 15000  // WiFi reconnect interval, ms.
#define BU04_FOLLOW_REST_RETRY_MS 1200  // REST retry delay after failure, ms.
#define BU04_FOLLOW_STATUS_LOG_MS 15000  // Status log interval, ms.

#define BU04_FOLLOW_QUEUE_SIZE 10  // Target queue capacity.
#define BU04_FOLLOW_BOOTSTRAP_POINTS 1  // Points required before first dispatch.
#define BU04_FOLLOW_LOOKAHEAD_POINTS 1  // Lookahead points taken from queue.
#define BU04_FOLLOW_LIVE_REPLAN_MIN_POINTS 1  // Min queued points before replanning.
#define BU04_FOLLOW_LIVE_REPLAN_MIN_MS 60  // Min gap between replans, ms.
#define BU04_FOLLOW_ENABLE_POINT_SPACING_GUARD 1  // 1 = suppress points that are too close to the last queued point.
#define BU04_FOLLOW_INVERT_X_AXIS 1  // 1 = invert BU04 X axis before filtering, 0 = keep original direction.
#define BU04_FOLLOW_MIN_SEND_DISTANCE_M 0.05f  // Minimum distance between two dispatched targets, m.
#define BU04_FOLLOW_MIN_SEND_INTERVAL_MS 40  // Minimum time between two dispatches, ms.
#define BU04_FOLLOW_FORCE_SEND_INTERVAL_MS 2000  // Force one dispatch at least every N ms when data is available.
#define BU04_FOLLOW_LATEST_ANGLE_SEND_MS 5000  // Dispatch interval for the latest-angle motion mode, ms.
#define BU04_FOLLOW_MEAN_WINDOW_SIZE 4  // Sliding sample window size for robust averaging.
#define BU04_FOLLOW_MEAN_MIN_KEEP 2  // Minimum samples kept after outlier rejection.
#define BU04_FOLLOW_MEAN_OUTLIER_DISTANCE_CM 30.0f  // Jump threshold for window averaging, cm.
#define BU04_FOLLOW_KALMAN_MEAS_NOISE_CM2 16.0f  // Measurement noise variance for X/Y, cm^2.
#define BU04_FOLLOW_KALMAN_ACCEL_NOISE_CM2_PER_S4 2500.0f  // Process acceleration noise variance, cm^2/s^4.
#define BU04_FOLLOW_KALMAN_INIT_POS_VAR 100.0f  // Initial position variance, cm^2.
#define BU04_FOLLOW_KALMAN_INIT_VEL_VAR 400.0f  // Initial velocity variance, (cm/s)^2.

#define BU04_FOLLOW_EMA_ALPHA 0.28f  // EMA factor for filtered X/Y.
#define BU04_FOLLOW_VELOCITY_ALPHA 0.35f  // EMA factor for velocity estimate.
#define BU04_FOLLOW_CLAMP_MAX_JUMP_CM 50.0f  // Max single-frame jump clamp, cm.
#define BU04_FOLLOW_LOOKAHEAD_SEC_NEAR 0.30f  // Lookahead time for near range, s.
#define BU04_FOLLOW_LOOKAHEAD_SEC_MID 0.55f  // Lookahead time for mid range, s.
#define BU04_FOLLOW_LOOKAHEAD_SEC_FAR 0.80f  // Lookahead time for far range, s.
#define BU04_FOLLOW_MIN_POINT_SPACING_CM 2.0f  // Minimum distance between queued points, cm.
#define BU04_FOLLOW_REPLAN_TARGET_DELTA_M 0.25f  // Target delta that triggers replan, m.
#define BU04_FOLLOW_SLOWDOWN_DISTANCE_M 0.60f  // Start soft slowdown when distance is below this, m.
#define BU04_FOLLOW_SAFETY_STOP_HOLD_MS 200  // Hold safety stop briefly before canceling action, ms.

#define BU04_FOLLOW_VALID_ANGLE_MIN_DEG -60.0f  // BU04 angle lower bound for valid tracking, deg.
#define BU04_FOLLOW_VALID_ANGLE_MAX_DEG 60.0f  // BU04 angle upper bound for valid tracking, deg.
#define BU04_FOLLOW_INVALID_ANGLE_HOLD_MS 250  // Ignore brief angle spikes before pausing tracking, ms.

#define BU04_FOLLOW_TOO_CLOSE_DISTANCE_M 0.30f  // Stop getting closer if target is too near, m.
#define BU04_FOLLOW_SLOW_DISTANCE_M 1.0f  // Boundary for slow/medium logic, m.
#define BU04_FOLLOW_FAST_DISTANCE_M 2.5f  // Boundary for medium/fast logic, m.
#define BU04_FOLLOW_LOST_SIGNAL_TIMEOUT_MS 2000  // Lost-signal timeout, ms.
#define BU04_FOLLOW_SAFETY_STOP_DISTANCE_M 0.30f  // Emergency stop distance, m.

}  // namespace follow_demo
