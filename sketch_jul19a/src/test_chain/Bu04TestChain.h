#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "../bu04/Bu04Uart.h"
#include "TestChainConfig.h"
#include "TestMotionClient.h"

namespace test_chain {

struct UwbFrame {
  String a16;
  long t = 0;
  long d = 0;
  long xcm = 0;
  long ycm = 0;
};

class Bu04TestChain {
 public:
  explicit Bu04TestChain(Stream& debug);

  void begin();
  void update(Bu04Uart& dataUart);

 private:
  enum class State {
    kWaitJ,
    kWaitS,
    kReadLen,
    kReadPayload,
  };

  void ensureWifi();
  bool refreshPose();
  void handleIncomingByte(char c);
  void resetFrame();
  bool parseFrame(const String& frame, UwbFrame& out);
  static bool parseLongField(const String& text, const char* key, long& value);
  static bool parseStringField(const String& text, const char* key, String& value);
  static float computeAngleDeg(long xcm, long ycm);
  static bool angleInRange(float angleDeg, float minDeg, float maxDeg);
  static float clampFloat(float value, float minValue, float maxValue);
  static float distanceM(const UwbFrame& frame);
  static float forwardDistanceM(const UwbFrame& frame);
  static float mapHeadingX(float distanceM, const RobotPose& pose);
  static float mapHeadingY(float distanceM, const RobotPose& pose);
  void tickPeriodicMove();
  void handleStraightFrame(const UwbFrame& frame);
  void handleRotateFrame(const UwbFrame& frame);
  void logStatusIfDue();
  void checkCurrentAction();

  Stream& debug_;
  TestMotionClient motion_;
  String lenBuffer_;
  String payloadBuffer_;
  uint16_t expectedLength_;
  State state_;
  unsigned long lastWifiAttempt_;
  unsigned long lastPoseAttemptMs_;
  unsigned long lastPeriodicSendMs_;
  unsigned long lastStraightSendMs_;
  unsigned long lastRotateSendMs_;
  unsigned long lastActionCheckMs_;
  unsigned long lastStatusLogMs_;
  unsigned long lastFrameSeenMs_;
  unsigned long lastRestAttemptMs_;
  bool wifiReady_;
  bool hasPose_;
  RobotPose lastPose_;
  bool actionActive_;
  String currentActionId_;
  bool hasCurrentTarget_;
  uint32_t totalFrames_;
  uint32_t totalSends_;
  uint32_t parseFails_;
  uint32_t rejectedFrames_;
  uint32_t straightAccepted_;
  uint32_t rotateAccepted_;
};

}  // namespace test_chain
