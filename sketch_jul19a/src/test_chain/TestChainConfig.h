#pragma once

#include <Arduino.h>

namespace test_chain {

constexpr const char* kWifiSsid = "YQDL-Office-5G";
constexpr const char* kWifiPassword = "Yqdl@123";

inline IPAddress robotIp() {
  return IPAddress(192, 168, 0, 56);
}
constexpr uint16_t kRobotPort = 1448;

#define BU04_TEST_CHAIN_MODE_PERIODIC_MOVE 0
#define BU04_TEST_CHAIN_MODE_STRAIGHT_ONLY 1
#define BU04_TEST_CHAIN_MODE_ROTATE_ONLY 2

// Change this macro to pick one test path.
// Current test focus: sample one BU04 frame every 5 seconds and use it only for angle test.
#define BU04_TEST_CHAIN_MODE BU04_TEST_CHAIN_MODE_ROTATE_ONLY

#define BU04_TEST_CHAIN_PERIODIC_SEND_MS 4000  // Periodic move-to dispatch interval, ms.
#define BU04_TEST_CHAIN_PERIODIC_STEP_M 1.0f  // Fixed forward step for periodic move mode, m.

#define BU04_TEST_CHAIN_STRAIGHT_SEND_INTERVAL_MS 250  // Straight-mode dispatch interval, ms.
#define BU04_TEST_CHAIN_STRAIGHT_USE_ANGLE_FILTER 0  // 1 = keep near-straight angle filter, 0 = accept all frames for straight motion.
#define BU04_TEST_CHAIN_STRAIGHT_MIN_ANGLE_DEG -10.0f  // Angle filter lower bound when enabled, deg.
#define BU04_TEST_CHAIN_STRAIGHT_MAX_ANGLE_DEG 10.0f  // Angle filter upper bound when enabled, deg.
#define BU04_TEST_CHAIN_STRAIGHT_FORWARD_MIN_M 0.20f  // Minimum straight-step distance, m.
#define BU04_TEST_CHAIN_STRAIGHT_FORWARD_MAX_M 1.20f  // Maximum straight-step distance, m.

#define BU04_TEST_CHAIN_ANGLE_SAMPLE_MS 10000  // Take one BU04 frame for angle test every 10 seconds, ms.
#define BU04_TEST_CHAIN_ROTATE_MIN_ANGLE_DEG -60.0f  // Acceptable angle-test lower bound, deg.
#define BU04_TEST_CHAIN_ROTATE_MAX_ANGLE_DEG 60.0f  // Acceptable angle-test upper bound, deg.
#define BU04_TEST_CHAIN_ROTATE_TARGET_DEG 90.0f  // Reserved fixed yaw target if you want to override frame-angle rotation, deg.

#define BU04_TEST_CHAIN_WIFI_RECONNECT_MS 15000  // WiFi reconnect interval, ms.
#define BU04_TEST_CHAIN_REST_RETRY_MS 1200  // REST retry delay after failure, ms.
#define BU04_TEST_CHAIN_STATUS_LOG_MS 5000  // Status log interval, ms.

#define BU04_TEST_CHAIN_FRAME_IDLE_LOG_MS 3000  // Frame idle warning interval, ms.

}  // namespace test_chain
