#pragma once

#include <Arduino.h>

namespace follow_v3 {

constexpr const char* kWifiSsid = "YQDL-Office-5G";
constexpr const char* kWifiPassword = "Yqdl@123";

inline IPAddress robotIp() {
  return IPAddress(192, 168, 0, 56);
}

constexpr uint16_t kRobotPort = 1448;

// Legacy single-sample timing kept for compatibility when batch mode is disabled.
#define BU04_FOLLOW_V3_SAMPLE_INTERVAL_MS 500
#define BU04_FOLLOW_V3_ACTION_CHECK_MS 800
#define BU04_FOLLOW_V3_WIFI_RECONNECT_MS 10000
#define BU04_FOLLOW_V3_REST_RETRY_MS 1200
#define BU04_FOLLOW_V3_STATUS_LOG_MS 15000
#define BU04_FOLLOW_V3_LOST_SIGNAL_TIMEOUT_MS 1500

// Batch analysis: collect up to 10 BU04 frames within a 1-second window, then analyze once.
#define BU04_FOLLOW_V3_ENABLE_BATCH_ANALYSIS 1
#define BU04_FOLLOW_V3_BATCH_FRAME_COUNT 10
#define BU04_FOLLOW_V3_BATCH_WINDOW_MS 1000
#define BU04_FOLLOW_V3_BATCH_STATIC_ANGLE_GAP_DEG 5.0f
#define BU04_FOLLOW_V3_BATCH_MOVING_ANGLE_GAP_DEG 20.0f
#define BU04_FOLLOW_V3_BATCH_MIN_KEEP 6

// Keep serial output quiet unless you are actively debugging follow_v3.
#define BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG 0
// Minimal USB serial output for leg status.
#define BU04_FOLLOW_V3_ENABLE_USB_LEG_LOG 1

// Mirror the final follow target point over TCP in a background task.
#define BU04_FOLLOW_V3_ENABLE_TCP_TARGET_MIRROR 0
#define BU04_FOLLOW_V3_TCP_TARGET_QUEUE_SIZE 1
#define BU04_FOLLOW_V3_TCP_TARGET_RETRY_MS 5000

// Leg follow integration.
// Set to 0 to fully disable leg fetching and related follow logic.
#define BU04_FOLLOW_V3_ENABLE_LEG_ASSIST 1
#define BU04_FOLLOW_V3_LEG_SAMPLE_INTERVAL_MS 500

// Set to 1 to run follow_v3 as a leg-only controller and ignore BU04 UWB frames for decisions.
#define BU04_FOLLOW_V3_LEG_ONLY 1

// Optional 1-second leg report over TCP.
#define BU04_FOLLOW_V3_ENABLE_TCP_REPORT 1
#define BU04_FOLLOW_V3_TCP_REPORT_INTERVAL_MS 1000

inline IPAddress tcpReportServerIp() {
  return IPAddress(192, 168, 0, 85);
}

constexpr uint16_t kTcpReportServerPort = 5000;

#define BU04_FOLLOW_V3_EMA_ALPHA 0.30f
#define BU04_FOLLOW_V3_CLAMP_MAX_JUMP_CM 50.0f
#define BU04_FOLLOW_V3_MAX_TRACK_ANGLE_DEG 65.0f
#define BU04_FOLLOW_V3_ANGLE_RECOVER_GAP_DEG 10.0f
#define BU04_FOLLOW_V3_TOO_CLOSE_DISTANCE_M 0.50f
#define BU04_FOLLOW_V3_SLOW_DISTANCE_M 1.0f
#define BU04_FOLLOW_V3_FAST_DISTANCE_M 2.0f
#define BU04_FOLLOW_V3_SLOW_SPEED_RATIO 0.80f
#define BU04_FOLLOW_V3_NORMAL_SPEED_RATIO 1.00f
#define BU04_FOLLOW_V3_FAST_SPEED_RATIO 1.20f
#define BU04_FOLLOW_V3_PATH_SPEED_RATIO 1.00f
#define BU04_FOLLOW_V3_PATH_ACCEPTABLE_PRECISION_M 0.18f
#define BU04_FOLLOW_V3_PATH_FAIL_RETRY_COUNT 1

}  // namespace follow_v3
