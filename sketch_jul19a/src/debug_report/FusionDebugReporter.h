#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "../bu04/Bu04Uart.h"
#include "../leg/LegFollowDetector.h"

namespace fusion_debug {

class FusionDebugReporter {
 public:
  explicit FusionDebugReporter(Stream& debug);

  void begin();
  void update(Bu04Uart& dataUart);

 private:
  enum class State {
    kWaitJ,
    kWaitS,
    kReadLen,
    kReadPayload,
  };

  struct UwbFrame {
    String a16;
    long t = 0;
    long d = 0;
    long xcm = 0;
    long ycm = 0;
  };

  struct Sample {
    float uwb_angle_deg = 0.0f;
    bool has_leg = false;
    float leg_angle_deg = 0.0f;
  };

  void ensureWifi();
  bool ensureServer();
  void handleIncomingByte(char c);
  void resetFrame();
  bool parseFrame(const String& frame, UwbFrame& out);
  static bool parseStringField(const String& text, const char* key, String& value);
  static bool parseLongField(const String& text, const char* key, long& value);
  static float computeAngleDeg(long xcm, long ycm);
  void refreshLeg();
  void captureSample();
  void sendWindowIfDue();
  void resetWindow();
  String buildPayload() const;

  Stream& debug_;
  WiFiClient client_;
  String lenBuffer_;
  String payloadBuffer_;
  UwbFrame latestFrame_;
  bool hasLatestFrame_;
  bool wifiReady_;
  bool serverReady_;
  bool hasLegSample_;
  bool hasLegTarget_;
  bool legReadyNotified_;
  leg_follow::FollowTargetReport lastLegReport_;
  uint16_t expectedLength_;
  State state_;
  unsigned long lastWifiAttempt_;
  unsigned long lastServerAttempt_;
  unsigned long lastLegSampleMs_;
  unsigned long lastSampleMs_;
  unsigned long lastWindowSendMs_;
  Sample samples_[15];
  uint8_t sampleCount_;
  unsigned long totalFrames_;
  unsigned long totalSamples_;
  unsigned long totalSent_;
  unsigned long parseFails_;
  unsigned long lastStatusLogMs_;
};

}  // namespace fusion_debug
