#include "UwbFollowRestV3.h"

#include <HTTPClient.h>
#include <algorithm>
#include <ctype.h>
#include <math.h>

namespace follow_v3 {

namespace {

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

float normalizeAngleDeg(float deg) {
  while (deg <= -180.0f) {
    deg += 360.0f;
  }
  while (deg > 180.0f) {
    deg -= 360.0f;
  }
  return deg;
}

float medianOfValues(const float* values, uint8_t count) {
  if (count == 0U) {
    return 0.0f;
  }

  float tmp[BU04_FOLLOW_V3_BATCH_FRAME_COUNT];
  for (uint8_t i = 0; i < count; ++i) {
    tmp[i] = values[i];
  }
  std::sort(tmp, tmp + count);

  if ((count & 1U) != 0U) {
    return tmp[count / 2U];
  }

  return 0.5f * (tmp[(count / 2U) - 1U] + tmp[count / 2U]);
}

}  // namespace

UwbFollowRestV3::UwbFollowRestV3(Stream& debug)
    : debug_(debug),
      expectedLength_(0),
      state_(State::kWaitJ),
      lastWifiAttempt_(0),
      lastActionCheckMs_(0),
      lastStatusLogMs_(0),
      lastDispatchMs_(0),
      lastRestAttemptMs_(0),
      lastGoodFrameMs_(0),
      lastSampleMs_(0),
      wifiReady_(false),
      hasLatestUwb_(false),
      hasFiltered_(false),
      hasPose_(false),
      actionActive_(false),
      followPausedTooClose_(false),
      anglePaused_(false),
      latestXcm_(0.0f),
      latestYcm_(0.0f),
      filteredXcm_(0.0f),
      filteredYcm_(0.0f),
      totalFrames_(0),
      totalSamples_(0),
      totalActions_(0),
      totalClamped_(0),
      parseFails_(0) {
  lenBuffer_.reserve(8);
  payloadBuffer_.reserve(256);
  currentActionId_.reserve(64);
  batchHead_ = 0;
  batchCount_ = 0;
  batchStartMs_ = 0;
#if BU04_FOLLOW_V3_ENABLE_LEG_ASSIST
  lastLegReport_ = leg_follow::FollowTargetReport{};
  lastLegSampleMs_ = 0;
  hasLegSample_ = false;
  hasLegReport_ = false;
  lastLegPersonCount_ = 0;
  legReadyNotified_ = false;
#endif
#if BU04_FOLLOW_V3_ENABLE_TCP_REPORT
  lastTcpReportAttemptMs_ = 0;
  lastTcpReportSentMs_ = 0;
  tcpWindowStartMs_ = millis();
  tcpPairCount_ = 0;
  resetTcpReportWindow();
#endif
#if BU04_FOLLOW_V3_ENABLE_TCP_TARGET_MIRROR
  tcpTargetQueue_ = nullptr;
  tcpTargetTask_ = nullptr;
  lastTcpTargetAttemptMs_ = 0;
  tcpTargetReady_ = false;
  tcpTargetTaskStarted_ = false;
#endif
}

void UwbFollowRestV3::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(follow_v3::kWifiSsid, follow_v3::kWifiPassword);
#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
  debug_.println("REST follow_v3 controller starting...");
#endif
#if BU04_FOLLOW_V3_ENABLE_TCP_TARGET_MIRROR
  if (tcpTargetQueue_ == nullptr) {
    tcpTargetQueue_ = xQueueCreate(BU04_FOLLOW_V3_TCP_TARGET_QUEUE_SIZE, sizeof(TcpTargetSample));
  }
  if (tcpTargetQueue_ != nullptr && tcpTargetTask_ == nullptr) {
    const BaseType_t ok = xTaskCreatePinnedToCore(tcpTargetTaskTrampoline,
                                                  "follow_v3_tcp",
                                                  6144,
                                                  this,
                                                  1,
                                                  &tcpTargetTask_,
                                                  0);
    tcpTargetTaskStarted_ = (ok == pdPASS);
#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
    debug_.println(tcpTargetTaskStarted_ ? "[follow_v3] tcp mirror task started"
                                         : "[follow_v3] tcp mirror task start failed");
#endif
  }
#endif
}

void UwbFollowRestV3::update(Bu04Uart& dataUart) {
  ensureWifi();

#if BU04_FOLLOW_V3_LEG_ONLY
  (void)dataUart;
  const unsigned long now = millis();
  if (now - lastSampleMs_ >= BU04_FOLLOW_V3_LEG_SAMPLE_INTERVAL_MS) {
    lastSampleMs_ = now;
    UwbFrame dummyFrame;
    processSample(dummyFrame);
  }
#else
  char c;
  while (dataUart.readByte(c)) {
    handleIncomingByte(c);
  }

  if (hasLatestUwb_ && millis() - lastGoodFrameMs_ > BU04_FOLLOW_V3_LOST_SIGNAL_TIMEOUT_MS) {
    if (actionActive_) {
      stopFollowMotion("uwb_lost");
    }
    hasLatestUwb_ = false;
    hasFiltered_ = false;
  }

#if BU04_FOLLOW_V3_ENABLE_BATCH_ANALYSIS
  flushBatchWindowIfDue();
#endif
#endif

  checkCurrentAction();
#if BU04_FOLLOW_V3_ENABLE_TCP_REPORT
  sendTcpReportIfDue();
#endif
  logStatusIfDue();
}

void UwbFollowRestV3::ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiReady_) {
#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
      debug_.print("WiFi connected, IP: ");
      debug_.println(WiFi.localIP());
#endif
      wifiReady_ = true;
    }
    return;
  }

  wifiReady_ = false;
  if (millis() - lastWifiAttempt_ < BU04_FOLLOW_V3_WIFI_RECONNECT_MS) {
    return;
  }

  lastWifiAttempt_ = millis();
#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
  debug_.println("Reconnecting WiFi for follow_v3...");
#endif
  WiFi.disconnect();
  WiFi.begin(follow_v3::kWifiSsid, follow_v3::kWifiPassword);
}

bool UwbFollowRestV3::refreshPose() {
  RobotPose pose;
  if (getPose(pose)) {
    lastPose_ = pose;
    hasPose_ = true;
    return true;
  }
  return hasPose_;
}

void UwbFollowRestV3::handleIncomingByte(char c) {
  if (c == '\r') {
    return;
  }

  switch (state_) {
    case State::kWaitJ:
      if (c == 'J') {
        state_ = State::kWaitS;
      }
      break;

    case State::kWaitS:
      if (c == 'S') {
        lenBuffer_ = "";
        state_ = State::kReadLen;
      } else if (c == 'J') {
        state_ = State::kWaitS;
      } else {
        state_ = State::kWaitJ;
      }
      break;

    case State::kReadLen:
      if (isxdigit(static_cast<unsigned char>(c))) {
        lenBuffer_ += c;
        if (lenBuffer_.length() >= 4) {
          char* endPtr = nullptr;
          const unsigned long parsed = strtoul(lenBuffer_.c_str(), &endPtr, 16);
          if (endPtr == lenBuffer_.c_str() || *endPtr != '\0' || parsed > 0xFFFFUL) {
            resetFrame();
            break;
          }
          expectedLength_ = static_cast<uint16_t>(parsed);
          payloadBuffer_ = "";
          payloadBuffer_.reserve(static_cast<size_t>(expectedLength_) + 1U);
          state_ = State::kReadPayload;
        }
      } else {
        resetFrame();
      }
      break;

    case State::kReadPayload:
      payloadBuffer_ += c;
      if (payloadBuffer_.length() >= expectedLength_) {
        UwbFrame frame;
        if (parseFrame(payloadBuffer_, frame)) {
          ++totalFrames_;
          handleUwbFrame(frame);
        } else {
          ++parseFails_;
        }
        resetFrame();
      }
      break;
  }
}

void UwbFollowRestV3::resetFrame() {
  state_ = State::kWaitJ;
  lenBuffer_ = "";
  payloadBuffer_ = "";
  expectedLength_ = 0;
}

void UwbFollowRestV3::handleUwbFrame(const UwbFrame& frame) {
  hasLatestUwb_ = true;
  lastFrame_ = frame;
  latestXcm_ = static_cast<float>(frame.xcm);
  latestYcm_ = static_cast<float>(frame.ycm);
  lastGoodFrameMs_ = millis();

#if BU04_FOLLOW_V3_ENABLE_BATCH_ANALYSIS
  flushBatchWindowIfDue();
  pushBatchFrame(frame);
#else
  if (WiFi.status() == WL_CONNECTED) {
    if (millis() - lastSampleMs_ < BU04_FOLLOW_V3_SAMPLE_INTERVAL_MS) {
      return;
    }
    lastSampleMs_ = millis();
    processSample(frame);
  }
#endif
}

void UwbFollowRestV3::clearBatchWindow() {
  batchHead_ = 0;
  batchCount_ = 0;
  batchStartMs_ = millis();
}

#if BU04_FOLLOW_V3_ENABLE_TCP_TARGET_MIRROR
void UwbFollowRestV3::tcpTargetTaskTrampoline(void* arg) {
  if (arg == nullptr) {
    vTaskDelete(nullptr);
    return;
  }

  static_cast<UwbFollowRestV3*>(arg)->tcpTargetTaskLoop();
}

void UwbFollowRestV3::shutdownTcpTargetSender() {
  if (tcpTargetTask_ != nullptr) {
    vTaskDelete(tcpTargetTask_);
    tcpTargetTask_ = nullptr;
  }
  if (tcpTargetQueue_ != nullptr) {
    vQueueDelete(tcpTargetQueue_);
    tcpTargetQueue_ = nullptr;
  }
  tcpTargetClient_.stop();
  tcpTargetReady_ = false;
}

bool UwbFollowRestV3::enqueueTcpTarget(const leg_follow::FollowTargetReport& report, const MapPoint* point) {
  if (tcpTargetQueue_ == nullptr || !tcpTargetTaskStarted_) {
    return false;
  }

  TcpTargetSample sample;
  sample.valid = report.valid;
  sample.hasTarget = report.has_target;
  sample.personCount = report.person_count;
  if (point != nullptr) {
    sample.x = point->x;
    sample.y = point->y;
  }
  if (report.has_target) {
    sample.distanceM = report.target.distance_m;
    sample.thetaDeg = report.target.theta_deg;
  }
  const BaseType_t ok = xQueueOverwrite(tcpTargetQueue_, &sample);
  return ok == pdTRUE;
}

bool UwbFollowRestV3::connectTcpTargetServer() {
  if (WiFi.status() != WL_CONNECTED) {
    tcpTargetReady_ = false;
    return false;
  }

  if (tcpTargetClient_.connected()) {
    tcpTargetReady_ = true;
    return true;
  }

  if (millis() - lastTcpTargetAttemptMs_ < BU04_FOLLOW_V3_TCP_TARGET_RETRY_MS) {
    return false;
  }

  lastTcpTargetAttemptMs_ = millis();
  tcpTargetClient_.stop();
  tcpTargetClient_.setNoDelay(true);
  tcpTargetClient_.setTimeout(0);
  tcpTargetReady_ = tcpTargetClient_.connect(follow_v3::tcpReportServerIp(), follow_v3::kTcpReportServerPort);
  return tcpTargetReady_;
}

bool UwbFollowRestV3::sendTcpTargetSample(const TcpTargetSample& sample) {
  if (!connectTcpTargetServer()) {
    return false;
  }

  String body;
  body.reserve(160);
  body += "{\"type\":\"follow_v3_target\",";
  body += "\"valid\":";
  body += sample.valid ? "true" : "false";
  body += ",\"has_target\":";
  body += sample.hasTarget ? "true" : "false";
  body += ",\"person_count\":";
  body += String(sample.personCount);
  body += ",\"x\":";
  body += String(sample.x, 3);
  body += ",\"y\":";
  body += String(sample.y, 3);
  body += ",\"distance_m\":";
  body += String(sample.distanceM, 3);
  body += ",\"theta_deg\":";
  body += String(sample.thetaDeg, 1);
  body += "}";

  const size_t sent = tcpTargetClient_.println(body);
  if (sent == 0U) {
    tcpTargetClient_.stop();
    tcpTargetReady_ = false;
    return false;
  }
  return true;
}

void UwbFollowRestV3::tcpTargetTaskLoop() {
  TcpTargetSample sample;
  for (;;) {
    if (tcpTargetQueue_ == nullptr) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (xQueueReceive(tcpTargetQueue_, &sample, pdMS_TO_TICKS(100)) == pdTRUE) {
      sendTcpTargetSample(sample);
    }
  }
}
#endif

void UwbFollowRestV3::pushBatchFrame(const UwbFrame& frame) {
  if (batchCount_ == 0U) {
    batchStartMs_ = millis();
  }

  const uint8_t insertIndex = static_cast<uint8_t>((batchHead_ + batchCount_) % BU04_FOLLOW_V3_BATCH_FRAME_COUNT);
  batchFrames_[insertIndex] = frame;
  if (batchCount_ < BU04_FOLLOW_V3_BATCH_FRAME_COUNT) {
    ++batchCount_;
  } else {
    batchHead_ = static_cast<uint8_t>((batchHead_ + 1U) % BU04_FOLLOW_V3_BATCH_FRAME_COUNT);
  }
}

void UwbFollowRestV3::flushBatchWindowIfDue() {
  if (batchCount_ == 0U) {
    return;
  }

  const unsigned long now = millis();
  if (now - batchStartMs_ < BU04_FOLLOW_V3_BATCH_WINDOW_MS) {
    return;
  }

  UwbFrame averagedFrame;
  uint8_t keptCount = 0U;
  uint8_t rejectedCount = 0U;
  if (!buildBatchAverage(averagedFrame, keptCount, rejectedCount)) {
#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
    debug_.print("[follow_v3] batch rejected kept=");
    debug_.print(keptCount);
    debug_.print(" rejected=");
    debug_.println(rejectedCount);
#endif
    clearBatchWindow();
    return;
  }

#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
  debug_.print("[follow_v3] batch ready kept=");
  debug_.print(keptCount);
  debug_.print(" rejected=");
  debug_.println(rejectedCount);
#endif

  lastFrame_ = averagedFrame;
  latestXcm_ = static_cast<float>(averagedFrame.xcm);
  latestYcm_ = static_cast<float>(averagedFrame.ycm);
  processSample(averagedFrame);
  clearBatchWindow();
}

bool UwbFollowRestV3::buildBatchAverage(UwbFrame& out, uint8_t& keptCount, uint8_t& rejectedCount) const {
  keptCount = 0U;
  rejectedCount = 0U;
  if (batchCount_ == 0U) {
    return false;
  }

  float angles[BU04_FOLLOW_V3_BATCH_FRAME_COUNT];
  for (uint8_t i = 0; i < batchCount_; ++i) {
    const UwbFrame& sample = batchFrames_[static_cast<uint8_t>((batchHead_ + i) % BU04_FOLLOW_V3_BATCH_FRAME_COUNT)];
    angles[i] = computeAngleDeg(sample.xcm, sample.ycm);
  }

  float peerGapAvg[BU04_FOLLOW_V3_BATCH_FRAME_COUNT];
  for (uint8_t i = 0; i < batchCount_; ++i) {
    float gapSum = 0.0f;
    for (uint8_t j = 0; j < batchCount_; ++j) {
      if (i == j) {
        continue;
      }
      gapSum += angleGapDeg(angles[i], angles[j]);
    }
    peerGapAvg[i] = batchCount_ > 1U ? gapSum / static_cast<float>(batchCount_ - 1U) : 0.0f;
  }

  const float medianGap = medianOfValues(peerGapAvg, batchCount_);
  const float thresholdDeg = (medianGap <= BU04_FOLLOW_V3_BATCH_STATIC_ANGLE_GAP_DEG)
                                 ? BU04_FOLLOW_V3_BATCH_STATIC_ANGLE_GAP_DEG
                                 : BU04_FOLLOW_V3_BATCH_MOVING_ANGLE_GAP_DEG;

  uint8_t kept = 0U;
  bool hasLastKept = false;
  UwbFrame lastKeptFrame;

  for (uint8_t i = 0; i < batchCount_; ++i) {
    const UwbFrame& sample = batchFrames_[static_cast<uint8_t>((batchHead_ + i) % BU04_FOLLOW_V3_BATCH_FRAME_COUNT)];
    if (peerGapAvg[i] > thresholdDeg) {
      ++rejectedCount;
      continue;
    }
    lastKeptFrame = sample;
    hasLastKept = true;
    ++kept;
  }

  keptCount = kept;
  if (kept < BU04_FOLLOW_V3_BATCH_MIN_KEEP) {
    return false;
  }

  if (!hasLastKept) {
    return false;
  }

  out = lastKeptFrame;
  return true;
}

bool UwbFollowRestV3::parseStringField(const String& text, const char* key, String& value) {
  const String needle = String("\"") + key + "\"";
  const int start = text.indexOf(needle);
  if (start < 0) {
    return false;
  }

  int valueStart = text.indexOf(':', start + static_cast<int>(needle.length()));
  if (valueStart < 0) {
    return false;
  }
  ++valueStart;
  while (valueStart < static_cast<int>(text.length()) &&
         isspace(static_cast<unsigned char>(text[valueStart]))) {
    ++valueStart;
  }
  if (valueStart >= static_cast<int>(text.length()) || text[valueStart] != '"') {
    return false;
  }
  ++valueStart;

  const int valueEnd = text.indexOf('"', valueStart);
  if (valueEnd < 0) {
    return false;
  }

  value = text.substring(valueStart, valueEnd);
  return true;
}

bool UwbFollowRestV3::parseLongField(const String& text, const char* key, long& value) {
  const String needle = String("\"") + key + "\"";
  const int start = text.indexOf(needle);
  if (start < 0) {
    return false;
  }

  int valueStart = text.indexOf(':', start + static_cast<int>(needle.length()));
  if (valueStart < 0) {
    return false;
  }
  ++valueStart;
  while (valueStart < static_cast<int>(text.length()) &&
         isspace(static_cast<unsigned char>(text[valueStart]))) {
    ++valueStart;
  }

  int valueEnd = valueStart;
  while (valueEnd < static_cast<int>(text.length())) {
    const char ch = text[valueEnd];
    if (!(isdigit(static_cast<unsigned char>(ch)) || ch == '-' || ch == '+')) {
      break;
    }
    ++valueEnd;
  }

  if (valueEnd <= valueStart) {
    return false;
  }

  value = text.substring(valueStart, valueEnd).toInt();
  return true;
}

bool UwbFollowRestV3::extractNumberField(const String& text, const char* key, float& value) {
  const String needle = String("\"") + key + "\"";
  const int start = text.indexOf(needle);
  if (start < 0) {
    return false;
  }

  int valueStart = text.indexOf(':', start + static_cast<int>(needle.length()));
  if (valueStart < 0) {
    return false;
  }
  ++valueStart;
  while (valueStart < static_cast<int>(text.length()) &&
         isspace(static_cast<unsigned char>(text[valueStart]))) {
    ++valueStart;
  }

  int valueEnd = valueStart;
  while (valueEnd < static_cast<int>(text.length())) {
    const char ch = text[valueEnd];
    if (!(isdigit(static_cast<unsigned char>(ch)) || ch == '-' || ch == '+' || ch == '.')) {
      break;
    }
    ++valueEnd;
  }

  if (valueEnd <= valueStart) {
    return false;
  }

  value = text.substring(valueStart, valueEnd).toFloat();
  return true;
}

bool UwbFollowRestV3::extractStringField(const String& text, const char* key, String& value) {
  const String needle = String("\"") + key + "\"";
  const int start = text.indexOf(needle);
  if (start < 0) {
    return false;
  }

  int valueStart = text.indexOf(':', start + static_cast<int>(needle.length()));
  if (valueStart < 0) {
    return false;
  }
  ++valueStart;
  while (valueStart < static_cast<int>(text.length()) &&
         isspace(static_cast<unsigned char>(text[valueStart]))) {
    ++valueStart;
  }
  if (valueStart >= static_cast<int>(text.length()) || text[valueStart] != '"') {
    return false;
  }
  ++valueStart;

  const int valueEnd = text.indexOf('"', valueStart);
  if (valueEnd < 0) {
    return false;
  }

  value = text.substring(valueStart, valueEnd);
  return true;
}

bool UwbFollowRestV3::parseFrame(const String& frame, UwbFrame& out) {
  if (frame.indexOf("\"TWR\"") < 0) {
    return false;
  }

  bool ok = true;
  ok &= parseStringField(frame, "a16", out.a16);
  ok &= parseLongField(frame, "T", out.t);
  ok &= parseLongField(frame, "D", out.d);
  ok &= parseLongField(frame, "Xcm", out.xcm);
  ok &= parseLongField(frame, "Ycm", out.ycm);
  return ok;
}

float UwbFollowRestV3::computeAngleDeg(long xcm, long ycm) {
  return atan2f(static_cast<float>(xcm), static_cast<float>(ycm)) * 180.0f / PI;
}

float UwbFollowRestV3::clampJump(float prev, float next, float maxJump) {
  const float delta = next - prev;
  if (delta > maxJump) {
    return prev + maxJump;
  }
  if (delta < -maxJump) {
    return prev - maxJump;
  }
  return next;
}

float UwbFollowRestV3::distanceM(const UwbFrame& frame) {
  return sqrtf(static_cast<float>(frame.xcm * frame.xcm + frame.ycm * frame.ycm)) * 0.01f;
}

float UwbFollowRestV3::speedRatioForDistance(float distM) {
  if (distM < 0.0f) {
    return BU04_FOLLOW_V3_NORMAL_SPEED_RATIO;
  }
  if (distM < BU04_FOLLOW_V3_TOO_CLOSE_DISTANCE_M) {
    return 0.0f;
  }
  if (distM < BU04_FOLLOW_V3_SLOW_DISTANCE_M) {
    return BU04_FOLLOW_V3_SLOW_SPEED_RATIO;
  }
  if (distM < BU04_FOLLOW_V3_FAST_DISTANCE_M) {
    return BU04_FOLLOW_V3_NORMAL_SPEED_RATIO;
  }
  return BU04_FOLLOW_V3_FAST_SPEED_RATIO;
}

float UwbFollowRestV3::angleGapDeg(float lhs, float rhs) {
  return fabsf(normalizeAngleDeg(lhs - rhs));
}

MapPoint UwbFollowRestV3::tagToMap(float xcm, float ycm, const RobotPose& pose) {
  const float dxBody = ycm * 0.01f;
  const float dyBody = xcm * 0.01f;
  const float cosYaw = cosf(pose.yaw);
  const float sinYaw = sinf(pose.yaw);

  MapPoint point;
  point.x = pose.x + dxBody * cosYaw - dyBody * sinYaw;
  point.y = pose.y + dxBody * sinYaw + dyBody * cosYaw;
  return point;
}

MapPoint UwbFollowRestV3::polarToMap(float distanceM, float thetaDeg, const RobotPose& pose) {
  const float thetaRad = thetaDeg * PI / 180.0f;
  const float leftCm = distanceM * sinf(thetaRad) * 100.0f;
  const float forwardCm = distanceM * cosf(thetaRad) * 100.0f;
  return tagToMap(leftCm, forwardCm, pose);
}

#if BU04_FOLLOW_V3_ENABLE_TCP_REPORT
void UwbFollowRestV3::resetTcpReportWindow() {
  tcpWindowStartMs_ = millis();
  tcpPairCount_ = 0;
  for (uint8_t i = 0; i < kTcpReportMaxPairs; ++i) {
    tcpSampleMs_[i] = 0;
    tcpUwbAngles_[i] = 0.0f;
    tcpUwbValid_[i] = false;
    tcpPersonCounts_[i] = 0;
    tcpLegAngles_[i] = 0.0f;
    tcpXs_[i] = 0.0f;
    tcpYs_[i] = 0.0f;
    tcpLegValid_[i] = false;
  }
}

void UwbFollowRestV3::accumulateTcpReport(bool uwbValid,
                                          float uwbAngleDeg,
                                          uint16_t personCount,
                                          bool legValid,
                                          float legAngleDeg,
                                          const MapPoint& point,
                                          unsigned long sampleMs) {
  if (tcpPairCount_ >= kTcpReportMaxPairs) {
    return;
  }

  const uint8_t idx = tcpPairCount_++;
  tcpSampleMs_[idx] = sampleMs;
  tcpUwbAngles_[idx] = uwbAngleDeg;
  tcpUwbValid_[idx] = uwbValid;
  tcpPersonCounts_[idx] = personCount;
  tcpXs_[idx] = point.x;
  tcpYs_[idx] = point.y;
  tcpLegAngles_[idx] = legAngleDeg;
  tcpLegValid_[idx] = legValid;
}

String UwbFollowRestV3::buildTcpReportPayload() const {
  String body;
  body.reserve(256);
  body += "{\"type\":\"follow_v3_leg_report\",";
  body += "\"window_ms\":";
  body += String(BU04_FOLLOW_V3_TCP_REPORT_INTERVAL_MS);
  body += ",\"samples\":[";
  for (uint8_t i = 0; i < tcpPairCount_; ++i) {
    if (i > 0U) {
      body += ",";
    }
    body += "[";
    body += String(tcpSampleMs_[i]);
    body += ",";
    body += String(tcpPersonCounts_[i]);
    body += ",";
    if (tcpUwbValid_[i]) {
      body += String(tcpUwbAngles_[i], 1);
    } else {
      body += "null";
    }
    body += ",";
    if (tcpLegValid_[i]) {
      body += String(tcpLegAngles_[i], 1);
    } else {
      body += "null";
    }
    body += ",";
    body += String(tcpXs_[i], 3);
    body += ",";
    body += String(tcpYs_[i], 3);
    body += "]";
  }
  body += "]";
  body += "}";
  return body;
}

bool UwbFollowRestV3::ensureTcpReportServer() {
  if (tcpReportClient_.connected()) {
    return true;
  }

  if (millis() - lastTcpReportAttemptMs_ < BU04_FOLLOW_V3_WIFI_RECONNECT_MS) {
    return false;
  }

  lastTcpReportAttemptMs_ = millis();
  tcpReportClient_.stop();
  return tcpReportClient_.connect(follow_v3::tcpReportServerIp(), follow_v3::kTcpReportServerPort);
}

bool UwbFollowRestV3::sendTcpReportIfDue() {
  const unsigned long now = millis();
  if (now - tcpWindowStartMs_ < BU04_FOLLOW_V3_TCP_REPORT_INTERVAL_MS) {
    return false;
  }

  if (tcpPairCount_ == 0U) {
    resetTcpReportWindow();
    return false;
  }

  if (!ensureTcpReportServer()) {
#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
    debug_.print("[follow_v3] tcp report connect fail ");
    debug_.print(follow_v3::tcpReportServerIp());
    debug_.print(":");
    debug_.println(follow_v3::kTcpReportServerPort);
#endif
    resetTcpReportWindow();
    return false;
  }

  const String payload = buildTcpReportPayload();
  const size_t sent = tcpReportClient_.println(payload);
  tcpReportClient_.flush();

  if (sent > 0U) {
    lastTcpReportSentMs_ = now;
#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
    debug_.print("[follow_v3] tcp report sent pairs=");
    debug_.print(tcpPairCount_);
    debug_.println();
#endif
  } else {
#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
    debug_.println("[follow_v3] tcp report send failed");
#endif
  }

  resetTcpReportWindow();
  return sent > 0U;
}
#endif

void UwbFollowRestV3::applyFilter(float rawXcm, float rawYcm, float& filtXcm, float& filtYcm, bool& clamped) {
  clamped = false;
  if (!hasFiltered_) {
    filteredXcm_ = rawXcm;
    filteredYcm_ = rawYcm;
    hasFiltered_ = true;
    filtXcm = rawXcm;
    filtYcm = rawYcm;
    return;
  }

  const float boundedX = clampJump(filteredXcm_, rawXcm, BU04_FOLLOW_V3_CLAMP_MAX_JUMP_CM);
  const float boundedY = clampJump(filteredYcm_, rawYcm, BU04_FOLLOW_V3_CLAMP_MAX_JUMP_CM);
  clamped = (boundedX != rawXcm) || (boundedY != rawYcm);
  if (clamped) {
    ++totalClamped_;
  }

  filtXcm = BU04_FOLLOW_V3_EMA_ALPHA * boundedX + (1.0f - BU04_FOLLOW_V3_EMA_ALPHA) * filteredXcm_;
  filtYcm = BU04_FOLLOW_V3_EMA_ALPHA * boundedY + (1.0f - BU04_FOLLOW_V3_EMA_ALPHA) * filteredYcm_;
  filteredXcm_ = filtXcm;
  filteredYcm_ = filtYcm;
}

#if BU04_FOLLOW_V3_ENABLE_LEG_ASSIST
bool UwbFollowRestV3::refreshLegAssist() {
  const unsigned long now = millis();

  if (hasLegSample_ && now - lastLegSampleMs_ < BU04_FOLLOW_V3_LEG_SAMPLE_INTERVAL_MS) {
    return true;
  }

  lastLegSampleMs_ = now;

  leg_follow::FollowTargetReport report;
  const String ipText = follow_v3::robotIp().toString();
  const leg_follow::PersonTarget* preferredTarget = (hasLegReport_ && lastLegReport_.valid && lastLegReport_.has_target)
                                                        ? &lastLegReport_.target
                                                        : nullptr;
  if (!leg_follow::fetchFollowTarget(ipText, report, follow_v3::kRobotPort, preferredTarget)) {
    hasLegSample_ = false;
    hasLegReport_ = false;
    lastLegReport_ = leg_follow::FollowTargetReport{};
    lastLegPersonCount_ = 0;
    legReadyNotified_ = false;
    return false;
  }

  hasLegSample_ = true;
  lastLegReport_ = report;
  lastLegPersonCount_ = report.person_count;
  hasLegReport_ = report.valid && report.has_target;

  if (hasLegReport_ && !legReadyNotified_) {
    legReadyNotified_ = true;
#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
    debug_.print("[follow_v3] leg ready");
    debug_.print(" count=");
    debug_.print(report.person_count);
    debug_.print(" target_d=");
    debug_.print(report.target.distance_m, 2);
    debug_.print("m target_angle=");
    debug_.println(report.target.theta_deg, 1);
#else
    debug_.println("[follow_v3] leg ready");
#endif
  }

  return true;
}
#endif

bool UwbFollowRestV3::processSample(const UwbFrame& frame) {
  if (!refreshPose()) {
    return false;
  }

  (void)frame;
  const unsigned long sampleMs = millis();

  if (!refreshLegAssist() || !lastLegReport_.valid) {
    if (actionActive_) {
      stopFollowMotion("leg_not_single");
    }
    return false;
  }

  const bool hasTarget = lastLegReport_.has_target;
  const MapPoint legPoint = hasTarget ? polarToMap(lastLegReport_.target.distance_m, lastLegReport_.target.theta_deg, lastPose_) : MapPoint{};
  const MapPoint* legPointPtr = hasTarget ? &legPoint : nullptr;
#if BU04_FOLLOW_V3_ENABLE_TCP_TARGET_MIRROR
  enqueueTcpTarget(lastLegReport_, legPointPtr);
#endif

#if BU04_FOLLOW_V3_ENABLE_USB_LEG_LOG
  debug_.print("[follow_v3] leg count=");
  debug_.print(lastLegPersonCount_);
  if (hasTarget) {
    debug_.print(" dist=");
    debug_.print(lastLegReport_.target.distance_m, 2);
    debug_.print("m theta=");
    debug_.print(lastLegReport_.target.theta_deg, 1);
    debug_.print("deg");
  }
  debug_.println();
#endif

  // Only a single leg target is allowed to drive follow.
  // No target or multiple targets means we stop following and wait for the next valid single-person sample.
  if (!hasTarget || lastLegPersonCount_ != 1U) {
#if BU04_FOLLOW_V3_ENABLE_TCP_REPORT
    accumulateTcpReport(false, 0.0f, lastLegPersonCount_, false, 0.0f, MapPoint{}, sampleMs);
#endif
    if (actionActive_) {
      stopFollowMotion("leg_not_single");
    }
    return false;
  }

#if BU04_FOLLOW_V3_ENABLE_TCP_REPORT
  accumulateTcpReport(false, 0.0f, lastLegPersonCount_, true, lastLegReport_.target.theta_deg, legPoint, sampleMs);
#endif

#if !BU04_FOLLOW_V3_LEG_ONLY
  if (!actionActive_) {
    return false;
  }
#endif

  const float legDistM = lastLegReport_.target.distance_m;
  const float speedRatio = speedRatioForDistance(legDistM);
  const bool dispatched = dispatchFixedPoint(legPoint, legDistM, speedRatio, BU04_FOLLOW_V3_PATH_ACCEPTABLE_PRECISION_M, millis());

#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
  if (totalSamples_ < 3U || (totalSamples_ % 5U == 1U)) {
    debug_.print("[follow_v3] sample #");
    debug_.print(totalSamples_ + 1U);
    debug_.print(" leg_count=");
    debug_.print(lastLegPersonCount_);
    debug_.print(" target=(");
    debug_.print(legPoint.x, 3);
    debug_.print(",");
    debug_.print(legPoint.y, 3);
    debug_.print(") dist=");
    debug_.print(legDistM, 2);
    debug_.print("m theta=");
    debug_.print(lastLegReport_.target.theta_deg, 1);
    debug_.print(" sent=");
    debug_.println(dispatched ? "1" : "0");
  }
#endif

  ++totalSamples_;
  return dispatched;
}

bool UwbFollowRestV3::dispatchFixedPoint(const MapPoint& point,
                                         float distM,
                                         float speedRatio,
                                         float acceptablePrecision,
                                         unsigned long now) {
  (void)distM;
  if (!prepareActionForDispatch()) {
    return false;
  }

  if (now - lastRestAttemptMs_ < BU04_FOLLOW_V3_REST_RETRY_MS) {
    return false;
  }

  const float yawRad = atan2f(point.y - lastPose_.y, point.x - lastPose_.x);
  String actionId;
  if (startFollowPathPoints(&point,
                            1,
                            yawRad,
                            speedRatio,
                            acceptablePrecision,
                            actionId)) {
    actionActive_ = true;
    currentActionId_ = actionId;
    lastDispatchMs_ = now;
    ++totalActions_;
#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
    debug_.print("[follow_v3] dispatched action_id=");
    debug_.println(actionId);
#endif
    return true;
  }

  lastRestAttemptMs_ = now;
  return false;
}

bool UwbFollowRestV3::prepareActionForDispatch() {
  if (!actionActive_) {
    return true;
  }

  stopFollowMotion("refresh_1hz");
  return true;
}

void UwbFollowRestV3::stopFollowMotion(const char* reason) {
  if (actionActive_) {
#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
    debug_.print("[follow_v3] cancel ");
    debug_.println(reason);
#endif
    cancelCurrentAction();
  }
  actionActive_ = false;
  currentActionId_ = "";
}

bool UwbFollowRestV3::getPose(RobotPose& pose) {
  int httpCode = 0;
  String response;
  if (!requestJson("GET", baseUrl() + "/api/core/slam/v1/localization/pose", "", httpCode, response)) {
    return false;
  }
  if (httpCode != HTTP_CODE_OK) {
    return false;
  }

  return extractNumberField(response, "x", pose.x) &&
         extractNumberField(response, "y", pose.y) &&
         extractNumberField(response, "yaw", pose.yaw);
}

bool UwbFollowRestV3::startFollowPathPoints(const MapPoint* points,
                                            uint8_t count,
                                            float yawRad,
                                            float speedRatio,
                                            float acceptablePrecision,
                                            String& actionId) {
  if (count == 0U) {
    return false;
  }

  String body;
  body.reserve(256 + static_cast<size_t>(count) * 48U);
  body += "{\"action_name\":\"slamtec.agent.actions.FollowPathPointsAction\",";
  body += "\"options\":{\"path_points\":[";
  for (uint8_t i = 0; i < count; ++i) {
    if (i > 0U) {
      body += ",";
    }
    body += "{\"x\":";
    body += String(points[i].x, 3);
    body += ",\"y\":";
    body += String(points[i].y, 3);
    body += ",\"z\":0}";
  }
  body += "],";
  body += "\"move_options\":{\"mode\":0,\"flags\":[\"precise\",\"with_yaw\"]},";
  body += "\"yaw\":";
  body += String(yawRad, 6);
  body += ",\"acceptable_precision\":";
  body += String(acceptablePrecision, 3);
  body += ",\"fail_retry_count\":";
  body += String(BU04_FOLLOW_V3_PATH_FAIL_RETRY_COUNT);
  body += ",\"speed_ratio\":";
  body += String(speedRatio, 3);
  body += "}}";

  int httpCode = 0;
  String response;
  if (!requestJson("POST", baseUrl() + "/api/core/motion/v1/actions", body, httpCode, response)) {
    return false;
  }
  if (httpCode < 200 || httpCode >= 300) {
    return false;
  }

  return extractStringField(response, "action_id", actionId);
}

bool UwbFollowRestV3::getActionRunning(const String& actionId, bool& running) {
  int httpCode = 0;
  String response;
  if (!requestJson("GET", baseUrl() + "/api/core/motion/v1/actions/" + actionId, "", httpCode, response)) {
    return false;
  }
  if (httpCode != HTTP_CODE_OK) {
    return false;
  }

  long status = -1;
  if (response.indexOf("\"state\"") >= 0) {
    const int stateStart = response.indexOf("\"state\"");
    const String stateText = response.substring(stateStart);
    if (parseLongField(stateText, "status", status)) {
      running = (status != 4L);
      return true;
    }
  }

  if (!parseLongField(response, "status", status)) {
    running = response.indexOf("\"action_name\"") >= 0;
    return true;
  }

  running = (status != 4L);
  return true;
}

bool UwbFollowRestV3::cancelCurrentAction() {
  int httpCode = 0;
  String response;
  if (!requestJson("DELETE", baseUrl() + "/api/core/motion/v1/actions/:current", "", httpCode, response)) {
    return false;
  }

  return httpCode == 200 || httpCode == 204 || httpCode == 404;
}

void UwbFollowRestV3::checkCurrentAction() {
  if (!actionActive_ || currentActionId_.isEmpty()) {
    return;
  }

  if (millis() - lastActionCheckMs_ < BU04_FOLLOW_V3_ACTION_CHECK_MS) {
    return;
  }
  lastActionCheckMs_ = millis();

  bool running = false;
  if (!getActionRunning(currentActionId_, running)) {
    return;
  }

  if (!running) {
    actionActive_ = false;
    currentActionId_ = "";
  }
}

void UwbFollowRestV3::logStatusIfDue() {
  if (millis() - lastStatusLogMs_ < BU04_FOLLOW_V3_STATUS_LOG_MS) {
    return;
  }
  lastStatusLogMs_ = millis();

#if BU04_FOLLOW_V3_ENABLE_VERBOSE_LOG
  debug_.print("[follow_v3] frames=");
  debug_.print(totalFrames_);
  debug_.print(" samples=");
  debug_.print(totalSamples_);
  debug_.print(" actions=");
  debug_.print(totalActions_);
  debug_.print(" clamped=");
  debug_.print(totalClamped_);
  debug_.print(" pose=");
  debug_.print(hasPose_ ? "1" : "0");
  debug_.print(" active=");
  debug_.println(actionActive_ ? "1" : "0");
#endif
}

String UwbFollowRestV3::baseUrl() const {
  String url = "http://";
  url += follow_v3::robotIp().toString();
  url += ":";
  url += String(follow_v3::kRobotPort);
  return url;
}

bool UwbFollowRestV3::requestJson(const String& method,
                                  const String& url,
                                  const String& body,
                                  int& httpCode,
                                  String& response) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  HTTPClient http;
  if (!http.begin(url)) {
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("accept", "application/json");

  if (method == "GET") {
    httpCode = http.GET();
  } else if (method == "POST") {
    httpCode = http.POST(body);
  } else if (method == "DELETE") {
    httpCode = http.sendRequest("DELETE");
  } else if (method == "PUT") {
    httpCode = http.PUT(body);
  } else {
    http.end();
    return false;
  }

  response = http.getString();
  http.end();
  return true;
}

}  // namespace follow_v3
