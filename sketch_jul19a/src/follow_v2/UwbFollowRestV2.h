#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "../bu04/Bu04Uart.h"
#include "FollowV2Config.h"

namespace follow_v2 {

struct UwbFrame {
  String a16;
  long t = 0;
  long d = 0;
  long xcm = 0;
  long ycm = 0;
};

struct RobotPose {
  float x = 0.0f;
  float y = 0.0f;
  float yaw = 0.0f;
};

struct MapPoint {
  float x = 0.0f;
  float y = 0.0f;
};

class UwbFollowRestV2 {
 public:
  explicit UwbFollowRestV2(Stream& debug);

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
  void pushMeanWindow(const UwbFrame& frame);
  bool buildMeanWindowFrame(UwbFrame& out) const;
  void resetKalman();
  void kalmanPredict(float dtSec);
  void kalmanUpdate(float measXcm, float measYcm, float dtSec);
  void processFilteredSample(const UwbFrame& frame, unsigned long now);
  bool prepareActionForDispatch(unsigned long now);
  bool shouldSuppressStationaryPoint(const MapPoint& point) const;
  bool shouldResumeFromStationaryHold() const;
  void processSample(const UwbFrame& frame);
  void pushPoint(const MapPoint& point);
  bool queueEmpty() const;
  const MapPoint& queueAt(uint8_t index) const;
  void popQueue(uint8_t count);
  void clearQueue();
  bool dispatchFixedPoint(const MapPoint& point, unsigned long now);
  bool dispatchSinglePointIfNeeded();
  bool dispatchRollingWindowIfNeeded();
  bool dispatchPathPoints(uint8_t count, uint8_t advanceCount);
  void checkCurrentAction();
  void logStatusIfDue();

  String baseUrl() const;
  static float degToRad(float value);
  static bool extractNumberField(const String& text, const char* key, float& value);
  static bool extractStringField(const String& text, const char* key, String& value);
  bool requestJson(const String& method,
                   const String& url,
                   const String& body,
                   int& httpCode,
                   String& response);
  bool getPose(RobotPose& pose);
  bool startFollowPathPoints(const MapPoint* points,
                             uint8_t count,
                             float yawRad,
                             float speedRatio,
                             float acceptablePrecision,
                             String& actionId);
  bool getActionRunning(const String& actionId, bool& running);
  bool cancelCurrentAction();

  Stream& debug_;
  String lenBuffer_;
  String payloadBuffer_;
  uint16_t expectedLength_;
  State state_;
  unsigned long lastWifiAttempt_;
  unsigned long lastPoseAttemptMs_;
  unsigned long lastSampleMs_;
  unsigned long angleInvalidSinceMs_;
  unsigned long safetyStopSinceMs_;
  unsigned long lastActionCheckMs_;
  unsigned long lastStatusLogMs_;
  unsigned long lastDispatchMs_;
  unsigned long lastRestAttemptMs_;
  unsigned long lastGoodFrameMs_;
  bool wifiReady_;
  bool hasFiltered_;
  bool hasVelocity_;
  float filteredXcm_;
  float filteredYcm_;
  float velocityXcmPerSec_;
  float velocityYcmPerSec_;
  bool kalmanInitialized_;
  unsigned long kalmanLastUpdateMs_;
  float kalmanStateXcm_;
  float kalmanStateYcm_;
  float kalmanStateVx_;
  float kalmanStateVy_;
  float kalmanCov_[4][4];
  UwbFrame meanWindow_[BU04_FOLLOW_V2_MEAN_WINDOW_SIZE];
  uint8_t meanWindowCount_;
  uint8_t meanWindowHead_;
  unsigned long windowBucketStartMs_;
  unsigned long windowBucketLastAcceptMs_;
  bool windowBucketActive_;
  RobotPose lastPose_;
  bool hasPose_;
  UwbFrame lastFrame_;
  bool hasFrame_;
  MapPoint queue_[BU04_FOLLOW_V2_QUEUE_SIZE];
  uint8_t queueHead_;
  uint8_t queueTail_;
  uint8_t queueCount_;
  bool actionActive_;
  bool stationaryHoldActive_;
  String currentActionId_;
  uint32_t totalFrames_;
  uint32_t totalPoints_;
  uint32_t totalActions_;
  uint32_t parseFails_;
  uint32_t rejectedSamples_;
};

}  // namespace follow_v2
