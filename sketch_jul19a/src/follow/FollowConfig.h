#pragma once

#include <Arduino.h>

namespace follow_demo {

constexpr const char* kWifiSsid = "yuechuang24";
constexpr const char* kWifiPassword = "Yuechuang1";

inline IPAddress robotIp() {
  return IPAddress(192, 168, 12, 1);
}
constexpr uint16_t kRobotPort = 1448;

#define BU04_FOLLOW_SAMPLE_INTERVAL_MS 200  // UWB sample throttle, ms.
#define BU04_FOLLOW_POSE_REFRESH_MS 200  // Minimum pose refresh interval, ms.
#define BU04_FOLLOW_ACTION_CHECK_MS 300  // Action status poll interval, ms.
#define BU04_FOLLOW_WIFI_RECONNECT_MS 15000  // WiFi reconnect interval, ms.
#define BU04_FOLLOW_REST_RETRY_MS 1200  // REST retry delay after failure, ms.
#define BU04_FOLLOW_STATUS_LOG_MS 15000  // Status log interval, ms.

#define BU04_FOLLOW_QUEUE_SIZE 30  // Target queue capacity.
#define BU04_FOLLOW_BOOTSTRAP_POINTS 8  // Points required before first dispatch.
#define BU04_FOLLOW_LOOKAHEAD_POINTS 5  // Lookahead points taken from queue.
#define BU04_FOLLOW_LIVE_REPLAN_MIN_POINTS 4  // Min queued points before replanning.
#define BU04_FOLLOW_LIVE_REPLAN_MIN_MS 900  // Min gap between replans, ms.
#define BU04_FOLLOW_ENABLE_POINT_SPACING_GUARD 1  // 1 = suppress points that are too close to the last queued point.
#define BU04_FOLLOW_MIN_SEND_DISTANCE_M 0.20f  // Minimum distance between two dispatched targets, m.
#define BU04_FOLLOW_MIN_SEND_INTERVAL_MS 600  // Minimum time between two dispatches, ms.

#define BU04_FOLLOW_EMA_ALPHA 0.28f  // EMA factor for filtered X/Y.
#define BU04_FOLLOW_VELOCITY_ALPHA 0.35f  // EMA factor for velocity estimate.
#define BU04_FOLLOW_CLAMP_MAX_JUMP_CM 50.0f  // Max single-frame jump clamp, cm.
#define BU04_FOLLOW_LOOKAHEAD_SEC_NEAR 0.45f  // Lookahead time for near range, s.
#define BU04_FOLLOW_LOOKAHEAD_SEC_MID 0.75f  // Lookahead time for mid range, s.
#define BU04_FOLLOW_LOOKAHEAD_SEC_FAR 1.0f  // Lookahead time for far range, s.
#define BU04_FOLLOW_MIN_POINT_SPACING_CM 8.0f  // Minimum distance between queued points, cm.
#define BU04_FOLLOW_REPLAN_TARGET_DELTA_M 0.25f  // Target delta that triggers replan, m.

#define BU04_FOLLOW_TOO_CLOSE_DISTANCE_M 0.5f  // Stop getting closer if target is too near, m.
#define BU04_FOLLOW_SLOW_DISTANCE_M 1.0f  // Boundary for slow/medium logic, m.
#define BU04_FOLLOW_FAST_DISTANCE_M 2.5f  // Boundary for medium/fast logic, m.
#define BU04_FOLLOW_MAX_TRACK_ANGLE_DEG 65.0f  // Max allowed tracking angle, deg.
#define BU04_FOLLOW_ANGLE_RECOVER_GAP_DEG 55.0f  // Angle hysteresis recovery gap, deg.
#define BU04_FOLLOW_LOST_SIGNAL_TIMEOUT_MS 2000  // Lost-signal timeout, ms.
#define BU04_FOLLOW_SAFETY_STOP_DISTANCE_M 0.35f  // Emergency stop distance, m.

}  // namespace follow_demo
