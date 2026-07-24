#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "../bu04/Bu04Uart.h"
#include "FollowV3Config.h"

#if BU04_FOLLOW_V3_ENABLE_TCP_TARGET_MIRROR
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#endif

#if BU04_FOLLOW_V3_ENABLE_LEG_ASSIST
#include "../leg/LegFollowDetector.h"
#endif

namespace follow_v3 {

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

class UwbFollowRestV3 {
 public:
  explicit UwbFollowRestV3(Stream& debug);

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
  void handleUwbFrame(const UwbFrame& frame);
  void pushBatchFrame(const UwbFrame& frame);
  void flushBatchWindowIfDue();
  bool buildBatchAverage(UwbFrame& out, uint8_t& keptCount, uint8_t& rejectedCount) const;
  void clearBatchWindow();
  static bool parseStringField(const String& text, const char* key, String& value);
  static bool parseLongField(const String& text, const char* key, long& value);
  static bool extractNumberField(const String& text, const char* key, float& value);
  static bool extractStringField(const String& text, const char* key, String& value);
  static float computeAngleDeg(long xcm, long ycm);
  static float clampJump(float prev, float next, float maxJump);
  static MapPoint tagToMap(float xcm, float ycm, const RobotPose& pose);
  static MapPoint polarToMap(float distanceM, float thetaDeg, const RobotPose& pose);
  static float distanceM(const UwbFrame& frame);
  static float speedRatioForDistance(float distM);
  static float angleGapDeg(float lhs, float rhs);
  void applyFilter(float rawXcm, float rawYcm, float& filtXcm, float& filtYcm, bool& clamped);
  bool processSample(const UwbFrame& frame);
  bool dispatchFixedPoint(const MapPoint& point,
                          float distM,
                          float speedRatio,
                          float acceptablePrecision,
                          unsigned long now);
  bool prepareActionForDispatch();
  void stopFollowMotion(const char* reason);
  bool getPose(RobotPose& pose);
  bool startFollowPathPoints(const MapPoint* points,
                             uint8_t count,
                             float yawRad,
                             float speedRatio,
                             float acceptablePrecision,
                             String& actionId);
  bool getActionRunning(const String& actionId, bool& running);
  bool cancelCurrentAction();
  void checkCurrentAction();
  void logStatusIfDue();
  String baseUrl() const;
  bool requestJson(const String& method,
                   const String& url,
                   const String& body,
                   int& httpCode,
                   String& response);
#if BU04_FOLLOW_V3_ENABLE_TCP_TARGET_MIRROR
  struct TcpTargetSample {
    bool valid = false;
    bool hasTarget = false;
    uint16_t personCount = 0;
    float x = 0.0f;
    float y = 0.0f;
    float distanceM = 0.0f;
    float thetaDeg = 0.0f;
  };
  static void tcpTargetTaskTrampoline(void* arg);
  void tcpTargetTaskLoop();
  bool enqueueTcpTarget(const leg_follow::FollowTargetReport& report, const MapPoint* point);
  bool connectTcpTargetServer();
  bool sendTcpTargetSample(const TcpTargetSample& sample);
  void shutdownTcpTargetSender();
#endif
#if BU04_FOLLOW_V3_ENABLE_TCP_REPORT
  bool ensureTcpReportServer();
  void resetTcpReportWindow();
  void accumulateTcpReport(bool uwbValid,
                           float uwbAngleDeg,
                           uint16_t personCount,
                           bool legValid,
                           float legAngleDeg,
                           const MapPoint& point,
                           unsigned long sampleMs);
  bool sendTcpReportIfDue();
  String buildTcpReportPayload() const;
#endif
#if BU04_FOLLOW_V3_ENABLE_LEG_ASSIST
  bool refreshLegAssist();
#endif

  Stream& debug_;
  String lenBuffer_;
  String payloadBuffer_;
  uint16_t expectedLength_;
  State state_;
  unsigned long lastWifiAttempt_;
  unsigned long lastActionCheckMs_;
  unsigned long lastStatusLogMs_;
  unsigned long lastDispatchMs_;
  unsigned long lastRestAttemptMs_;
  unsigned long lastGoodFrameMs_;
  unsigned long lastSampleMs_;
  bool wifiReady_;
  bool hasLatestUwb_;
  bool hasFiltered_;
  bool hasPose_;
  bool actionActive_;
  bool followPausedTooClose_;
  bool anglePaused_;
  float latestXcm_;
  float latestYcm_;
  float filteredXcm_;
  float filteredYcm_;
  unsigned long totalFrames_;
  unsigned long totalSamples_;
  unsigned long totalActions_;
  unsigned long totalClamped_;
  unsigned long parseFails_;
  String currentActionId_;
  RobotPose lastPose_;
  UwbFrame lastFrame_;
  UwbFrame batchFrames_[BU04_FOLLOW_V3_BATCH_FRAME_COUNT];
  uint8_t batchHead_;
  uint8_t batchCount_;
  unsigned long batchStartMs_;
#if BU04_FOLLOW_V3_ENABLE_LEG_ASSIST
  unsigned long lastLegSampleMs_;
  leg_follow::FollowTargetReport lastLegReport_;
  bool hasLegSample_;
  bool hasLegReport_;
  uint16_t lastLegPersonCount_;
  bool legReadyNotified_;
#endif
#if BU04_FOLLOW_V3_ENABLE_TCP_TARGET_MIRROR
  QueueHandle_t tcpTargetQueue_;
  TaskHandle_t tcpTargetTask_;
  WiFiClient tcpTargetClient_;
  unsigned long lastTcpTargetAttemptMs_;
  bool tcpTargetReady_;
  bool tcpTargetTaskStarted_;
#endif
#if BU04_FOLLOW_V3_ENABLE_TCP_REPORT
  WiFiClient tcpReportClient_;
  unsigned long lastTcpReportAttemptMs_;
  unsigned long lastTcpReportSentMs_;
  unsigned long tcpWindowStartMs_;
  static constexpr uint8_t kTcpReportMaxPairs = 4;
  unsigned long tcpSampleMs_[kTcpReportMaxPairs];
  float tcpUwbAngles_[kTcpReportMaxPairs];
  bool tcpUwbValid_[kTcpReportMaxPairs];
  uint16_t tcpPersonCounts_[kTcpReportMaxPairs];
  float tcpLegAngles_[kTcpReportMaxPairs];
  float tcpXs_[kTcpReportMaxPairs];
  float tcpYs_[kTcpReportMaxPairs];
  bool tcpLegValid_[kTcpReportMaxPairs];
  uint8_t tcpPairCount_;
#endif
};

}  // namespace follow_v3
