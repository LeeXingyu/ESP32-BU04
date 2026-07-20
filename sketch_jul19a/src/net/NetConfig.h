#pragma once

#include <Arduino.h>

namespace net_demo {

// WiFi credentials.
constexpr const char* kWifiSsid = "yuechuang24";
constexpr const char* kWifiPassword = "Yuechuang1";

// TCP server that receives BU04 data.
inline IPAddress serverIp() {
  return IPAddress(192, 168, 0, 243);
}
constexpr uint16_t kServerPort = 5000;

// Reconnect behavior.
constexpr uint32_t kWifiReconnectMs = 15000;
constexpr uint32_t kServerReconnectMs = 8000;
constexpr uint32_t kStatusLogMs = 15000;

}  // namespace net_demo
