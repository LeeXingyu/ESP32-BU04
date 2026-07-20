#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "../bu04/Bu04Uart.h"
#include "FollowConfig.h"
#include "SlamtecRestClient.h"

namespace follow_demo {

struct UwbFrame {
  String a16;
  long t = 0;
  long d = 0;
  long xcm = 0;
  long ycm = 0;
};

struct MapPoint {
  float x = 0.0f;
  float y = 0.0f;
};

class UwbFollowRest {
 public:
  explicit UwbFollowRest(Stream& debug);

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
  void handleIncomingByte(char c);
  void resetFrame();
  bool parseFrame(const String& frame, UwbFrame& out);
  static bool parseLongField(const String& text, const char* key, long& value);
  static bool parseStringField(const String& text, const char* key, String& value);
  static float computeAngleDeg(long xcm, long ycm);
  static float clampJump(float prev, float next, float maxJump);
  static MapPoint tagToMap(float xcm, float ycm, const RobotPose& pose);
  static float distanceM(const UwbFrame& frame);
  static float chooseLookaheadSec(float distM);
  void processSample(const UwbFrame& frame);
  void pushPoint(const MapPoint& point);
  bool queueEmpty() const;
  const MapPoint& queueAt(uint8_t index) const;
  void popQueue(uint8_t count);
  void dispatchNextIfNeeded();
  void checkCurrentAction();
  void logStatusIfDue();

  Stream& debug_;
  SlamtecRestClient rest_;
  String lenBuffer_;
  String payloadBuffer_;
  uint16_t expectedLength_;
  State state_;
  unsigned long lastWifiAttempt_;
  unsigned long lastSampleMs_;
  unsigned long lastMotionSampleMs_;
  unsigned long lastPoseMs_;
  unsigned long lastActionCheckMs_;
  unsigned long lastStatusLogMs_;
  unsigned long lastDispatchMs_;
  unsigned long lastRestAttemptMs_;
  unsigned long lastGoodFrameMs_;
  bool wifiReady_;
  bool angleBlocked_;
  bool hasFiltered_;
  bool hasVelocity_;
  float filteredXcm_;
  float filteredYcm_;
  float velocityXcmPerSec_;
  float velocityYcmPerSec_;
  RobotPose lastPose_;
  bool hasPose_;
  UwbFrame lastFrame_;
  bool hasFrame_;
  MapPoint queue_[BU04_FOLLOW_QUEUE_SIZE];
  uint8_t queueHead_;
  uint8_t queueTail_;
  uint8_t queueCount_;
  bool actionActive_;
  String currentActionId_;
  MapPoint currentTarget_;
  bool hasCurrentTarget_;
  uint32_t totalFrames_;
  uint32_t totalPoints_;
  uint32_t totalActions_;
  uint32_t parseFails_;
};

}  // namespace follow_demo
