#pragma once

#include <Arduino.h>

#include "FollowV2Mode.h"

namespace follow_v2 {

constexpr const char* kWifiSsid = "YQDL-Office-5G";
constexpr const char* kWifiPassword = "Yqdl@123";

inline IPAddress robotIp() {
  return IPAddress(192, 168, 0, 56);
}
constexpr uint16_t kRobotPort = 1448;

#define BU04_FOLLOW_V2_SAMPLE_INTERVAL_MS 500  // Fixed 500 ms sampling for the single-point mode.
#define BU04_FOLLOW_V2_ACTION_CHECK_MS 10  // Action state polling interval.
#define BU04_FOLLOW_V2_WIFI_RECONNECT_MS 15000  // WiFi reconnect interval.
#define BU04_FOLLOW_V2_REST_RETRY_MS 1200  // REST retry delay after a failed request.
#define BU04_FOLLOW_V2_STATUS_LOG_MS 15000  // Status log interval.

#define BU04_FOLLOW_V2_QUEUE_SIZE 30  // Rolling queue capacity.
#define BU04_FOLLOW_V2_BOOTSTRAP_POINTS 1  // Minimum points required before the first window dispatch.
#define BU04_FOLLOW_V2_STARTUP_BATCH_SIZE 1  // Smaller launch window so the robot starts sooner after power-up.
#define BU04_FOLLOW_V2_WINDOW_BUCKET_MS 50  // Aggregate window-mode samples over a shorter control window.
#define BU04_FOLLOW_V2_WINDOW_ACCEPT_INTERVAL_MS 10  // Keep most BU04 frames while still limiting over-frequent commits.
#define BU04_FOLLOW_V2_WINDOW_USE_LATEST_SAMPLE 1  // Prefer the newest valid sample for stable sensor data.
#define BU04_FOLLOW_V2_BATCH_SIZE 2  // Path points sent per rolling window dispatch.
#define BU04_FOLLOW_V2_BATCH_ADVANCE_POINTS 1  // Advance only one point per rolling window to keep overlap.
#define BU04_FOLLOW_V2_BATCH_MIN_RESERVE 1  // Keep at least this many queued points before refreshing the window.
#define BU04_FOLLOW_V2_BATCH_REISSUE_MS 120  // Reissue a rolling window more aggressively for smoother tracking.

#define BU04_FOLLOW_V2_INVERT_X_AXIS 1  // Invert BU04 X axis before filtering.
#define BU04_FOLLOW_V2_ENABLE_POINT_SPACING_GUARD 1  // Drop points that are too close to the last queued point.
#define BU04_FOLLOW_V2_MIN_POINT_SPACING_CM 2.0f  // Minimum distance between queued points.
#define BU04_FOLLOW_V2_STATIONARY_SPEED_CMPS 6.0f  // Treat the target as stationary below this filtered speed.
#define BU04_FOLLOW_V2_STATIONARY_RESUME_SPEED_CMPS 12.0f  // Treat the target as moving again above this filtered speed.
#define BU04_FOLLOW_V2_STATIONARY_ROBOT_DISTANCE_M 0.22f  // If robot is this close to a stationary target, stop feeding it.

#define BU04_FOLLOW_V2_MIN_SEND_DISTANCE_M 0.05f  // Legacy guard used by the 1 Hz mode.
#define BU04_FOLLOW_V2_MIN_SEND_INTERVAL_MS 500  // Legacy guard used by the 1 Hz mode.

#define BU04_FOLLOW_V2_PATH_SPEED_RATIO 0.35f  // Lower speed ratio for single-point follow.
#define BU04_FOLLOW_V2_WINDOW_PATH_SPEED_RATIO 0.80f  // Faster rolling-window motion to reduce perceived lag.
#define BU04_FOLLOW_V2_PATH_ACCEPTABLE_PRECISION_M 0.18f  // Accept point within this range.
#define BU04_FOLLOW_V2_WINDOW_PATH_ACCEPTABLE_PRECISION_M 0.14f  // Tighter precision for rolling-window path following.
#define BU04_FOLLOW_V2_PATH_FAIL_RETRY_COUNT 1  // Retry count for FollowPathPointsAction.

#define BU04_FOLLOW_V2_MEAN_WINDOW_SIZE 4  // Sliding window for robust averaging.
#define BU04_FOLLOW_V2_MEAN_MIN_KEEP 2  // Minimum kept samples after outlier rejection.
#define BU04_FOLLOW_V2_MEAN_OUTLIER_DISTANCE_CM 30.0f  // Outlier threshold for the averaging window.

#define BU04_FOLLOW_V2_KALMAN_MEAS_NOISE_CM2 64.0f  // Measurement noise variance for x/y in cm^2.
#define BU04_FOLLOW_V2_KALMAN_PROCESS_NOISE_POS 2.0f  // Process noise for position.
#define BU04_FOLLOW_V2_KALMAN_PROCESS_NOISE_VEL 12.0f  // Process noise for velocity.

#define BU04_FOLLOW_V2_EMA_ALPHA 0.28f  // EMA factor for filtered X/Y.
#define BU04_FOLLOW_V2_VELOCITY_ALPHA 0.35f  // EMA factor for velocity estimation.
#define BU04_FOLLOW_V2_CLAMP_MAX_JUMP_CM 50.0f  // Max single-sample jump clamp.

#define BU04_FOLLOW_V2_VALID_ANGLE_MIN_DEG -60.0f
#define BU04_FOLLOW_V2_VALID_ANGLE_MAX_DEG 60.0f
#define BU04_FOLLOW_V2_INVALID_ANGLE_HOLD_MS 250

#define BU04_FOLLOW_V2_TOO_CLOSE_DISTANCE_M 0.30f
#define BU04_FOLLOW_V2_SLOW_DISTANCE_M 1.0f
#define BU04_FOLLOW_V2_FAST_DISTANCE_M 2.5f
#define BU04_FOLLOW_V2_SAFETY_STOP_DISTANCE_M 0.30f
#define BU04_FOLLOW_V2_SAFETY_STOP_HOLD_MS 200

#define BU04_FOLLOW_V2_LOOKAHEAD_SEC_NEAR 0.30f
#define BU04_FOLLOW_V2_LOOKAHEAD_SEC_MID 0.55f
#define BU04_FOLLOW_V2_LOOKAHEAD_SEC_FAR 0.80f

}  // namespace follow_v2
