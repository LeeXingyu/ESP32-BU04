#pragma once

#include <Arduino.h>

namespace net_demo {

// WiFi credentials.
constexpr const char* kWifiSsid = "YQDL-Office-5G";
constexpr const char* kWifiPassword = "Yqdl@123";

// TCP server that receives BU04 data.
inline IPAddress serverIp() {
  return IPAddress(192, 168, 0, 85);
}
constexpr uint16_t kServerPort = 5000;

// Reconnect behavior.
constexpr uint32_t kWifiReconnectMs = 15000;
constexpr uint32_t kServerReconnectMs = 8000;
constexpr uint32_t kStatusLogMs = 15000;

// TCP averaging / outlier rejection.
#define BU04_TCP_ENABLE_OUTLIER_GUARD 1  // 1 = drop jumpy samples during averaging, not during acquisition.
#define BU04_TCP_OUTLIER_DISTANCE_CM 45.0f  // Samples farther than this from the group median are excluded, cm.
#define BU04_TCP_OUTLIER_MIN_KEEP 3  // Keep at least this many samples even if the guard rejects many points.

}  // namespace net_demo
