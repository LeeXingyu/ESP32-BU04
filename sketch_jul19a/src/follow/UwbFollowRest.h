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

struct FollowTarget {
  float xcm = 0.0f;
  float ycm = 0.0f;
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
  bool refreshPose();
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
  void pushMeanWindow(const UwbFrame& frame);
  bool computeRobustMean(UwbFrame& out);
  void pushPoint(const FollowTarget& point);
  bool queueEmpty() const;
  const FollowTarget& queueAt(uint8_t index) const;
  void popQueue(uint8_t count);
  void clearQueue();
  bool dispatchLatestAngleIfNeeded();
  void dispatchNextIfNeeded();
  void checkCurrentAction();
  void logStatusIfDue();
  static void sortLongs(long* values, uint8_t count);
  static long medianLong(const long* values, uint8_t count);

#if BU04_FOLLOW_ENABLE_KALMAN_FILTER
  struct Kalman1D {
    float pos = 0.0f;
    float vel = 0.0f;
    float p00 = BU04_FOLLOW_KALMAN_INIT_POS_VAR;
    float p01 = 0.0f;
    float p10 = 0.0f;
    float p11 = BU04_FOLLOW_KALMAN_INIT_VEL_VAR;
    bool initialized = false;
  };

  static void resetKalmanFilter(Kalman1D& filter, float measurement);
  static void predictKalmanFilter(Kalman1D& filter, float dtSec);
  static void updateKalmanFilter(Kalman1D& filter, float measurement);
  static void stepKalmanFilter(Kalman1D& filter, float measurement, float dtSec);
#endif

  Stream& debug_;
  SlamtecRestClient rest_;
  String lenBuffer_;
  String payloadBuffer_;
  uint16_t expectedLength_;
  State state_;
  unsigned long lastWifiAttempt_;
  unsigned long lastMotionSampleMs_;
  unsigned long angleInvalidSinceMs_;
  unsigned long safetyStopSinceMs_;
  unsigned long lastActionCheckMs_;
  unsigned long lastStatusLogMs_;
  unsigned long lastDispatchMs_;
  unsigned long lastForcedDispatchMs_;
  unsigned long lastRestAttemptMs_;
  unsigned long lastGoodFrameMs_;
  bool wifiReady_;
  bool hasFiltered_;
  bool hasVelocity_;
  float filteredXcm_;
  float filteredYcm_;
  float velocityXcmPerSec_;
  float velocityYcmPerSec_;
  UwbFrame meanWindow_[BU04_FOLLOW_MEAN_WINDOW_SIZE];
  uint8_t meanWindowCount_;
  uint8_t meanWindowHead_;
#if BU04_FOLLOW_ENABLE_KALMAN_FILTER
  Kalman1D kalmanX_;
  Kalman1D kalmanY_;
#endif
  RobotPose lastPose_;
  bool hasPose_;
  UwbFrame lastFrame_;
  bool hasFrame_;
  FollowTarget queue_[BU04_FOLLOW_QUEUE_SIZE];
  uint8_t queueHead_;
  uint8_t queueTail_;
  uint8_t queueCount_;
  bool actionActive_;
  String currentActionId_;
  MapPoint currentTarget_;
  bool hasCurrentTarget_;
  bool hasLatestAngle_;
  float latestAngleDeg_;
  float latestAngleRad_;
  unsigned long lastLatestAngleDispatchMs_;
  uint32_t totalFrames_;
  uint32_t totalPoints_;
  uint32_t totalActions_;
  uint32_t parseFails_;
  uint32_t rejectedSamples_;
};

}  // namespace follow_demo
