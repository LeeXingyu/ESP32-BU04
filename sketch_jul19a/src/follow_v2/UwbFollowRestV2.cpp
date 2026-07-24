#include "UwbFollowRestV2.h"

#include <HTTPClient.h>
#include <ctype.h>
#include <math.h>

namespace follow_v2 {

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

}  // namespace

UwbFollowRestV2::UwbFollowRestV2(Stream& debug)
    : debug_(debug),
      expectedLength_(0),
      state_(State::kWaitJ),
      lastWifiAttempt_(0),
      lastPoseAttemptMs_(0),
      lastSampleMs_(0),
      angleInvalidSinceMs_(0),
      safetyStopSinceMs_(0),
      lastActionCheckMs_(0),
      lastStatusLogMs_(0),
      lastDispatchMs_(0),
      lastRestAttemptMs_(0),
      lastGoodFrameMs_(0),
      wifiReady_(false),
      hasFiltered_(false),
      hasVelocity_(false),
      filteredXcm_(0.0f),
      filteredYcm_(0.0f),
      velocityXcmPerSec_(0.0f),
      velocityYcmPerSec_(0.0f),
      kalmanInitialized_(false),
      kalmanLastUpdateMs_(0),
      kalmanStateXcm_(0.0f),
      kalmanStateYcm_(0.0f),
      kalmanStateVx_(0.0f),
      kalmanStateVy_(0.0f),
      meanWindowCount_(0),
      meanWindowHead_(0),
      windowBucketStartMs_(0),
      windowBucketLastAcceptMs_(0),
      windowBucketActive_(false),
      hasPose_(false),
      hasFrame_(false),
      queueHead_(0),
      queueTail_(0),
      queueCount_(0),
      actionActive_(false),
      stationaryHoldActive_(false),
      totalFrames_(0),
      totalPoints_(0),
      totalActions_(0),
      parseFails_(0),
      rejectedSamples_(0) {
  lenBuffer_.reserve(8);
  payloadBuffer_.reserve(256);
  currentActionId_.reserve(64);
}

void UwbFollowRestV2::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(follow_v2::kWifiSsid, follow_v2::kWifiPassword);
  resetKalman();
  debug_.println("REST follow_v2 controller starting...");
}

void UwbFollowRestV2::update(Bu04Uart& dataUart) {
  ensureWifi();
  refreshPose();

  char c;
  while (dataUart.readByte(c)) {
    handleIncomingByte(c);
  }

  if (hasFrame_ && millis() - lastGoodFrameMs_ > BU04_FOLLOW_V2_SAFETY_STOP_HOLD_MS * 10UL) {
    if (actionActive_) {
      cancelCurrentAction();
      actionActive_ = false;
      currentActionId_ = "";
    }
    clearQueue();
  }

  // Disabled: dispatch does not poll chassis motion state.
  // checkCurrentAction();

#if BU04_FOLLOW_V2_ENABLE_FIXED_1HZ
  dispatchSinglePointIfNeeded();
#else
  dispatchRollingWindowIfNeeded();
#endif

  logStatusIfDue();
}

void UwbFollowRestV2::ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiReady_) {
      debug_.print("WiFi connected, IP: ");
      debug_.println(WiFi.localIP());
      wifiReady_ = true;
    }
    return;
  }

  wifiReady_ = false;
  if (millis() - lastWifiAttempt_ < BU04_FOLLOW_V2_WIFI_RECONNECT_MS) {
    return;
  }
  lastWifiAttempt_ = millis();

  debug_.println("Reconnecting WiFi for follow_v2...");
  WiFi.disconnect();
  WiFi.begin(follow_v2::kWifiSsid, follow_v2::kWifiPassword);
}

bool UwbFollowRestV2::refreshPose() {
  RobotPose pose;
  if (millis() - lastPoseAttemptMs_ < BU04_FOLLOW_V2_REST_RETRY_MS && hasPose_) {
    return true;
  }

  lastPoseAttemptMs_ = millis();
  if (getPose(pose)) {
    lastPose_ = pose;
    hasPose_ = true;
    return true;
  }
  return hasPose_;
}

void UwbFollowRestV2::handleIncomingByte(char c) {
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
          payloadBuffer_.reserve(static_cast<size_t>(expectedLength_) + 1);
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
          hasFrame_ = true;
          lastFrame_ = frame;
          lastGoodFrameMs_ = millis();
          processSample(frame);
        } else {
          ++parseFails_;
        }
        resetFrame();
      }
      break;
  }
}

void UwbFollowRestV2::resetFrame() {
  state_ = State::kWaitJ;
  lenBuffer_ = "";
  payloadBuffer_ = "";
  expectedLength_ = 0;
}

bool UwbFollowRestV2::parseStringField(const String& text, const char* key, String& value) {
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

bool UwbFollowRestV2::parseLongField(const String& text, const char* key, long& value) {
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
    if (!(isdigit(static_cast<unsigned char>(ch)) || ch == '-')) {
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

bool UwbFollowRestV2::parseFrame(const String& frame, UwbFrame& out) {
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

float UwbFollowRestV2::computeAngleDeg(long xcm, long ycm) {
  if (ycm == 0) {
    if (xcm > 0) {
      return 90.0f;
    }
    if (xcm < 0) {
      return -90.0f;
    }
    return 0.0f;
  }
  return atan2f(static_cast<float>(xcm), static_cast<float>(ycm)) * 180.0f / PI;
}

float UwbFollowRestV2::clampJump(float prev, float next, float maxJump) {
  const float delta = next - prev;
  if (delta > maxJump) {
    return prev + maxJump;
  }
  if (delta < -maxJump) {
    return prev - maxJump;
  }
  return next;
}

float UwbFollowRestV2::chooseLookaheadSec(float distM) {
  if (distM < BU04_FOLLOW_V2_TOO_CLOSE_DISTANCE_M) {
    return BU04_FOLLOW_V2_LOOKAHEAD_SEC_NEAR;
  }
  if (distM < BU04_FOLLOW_V2_SLOW_DISTANCE_M) {
    return BU04_FOLLOW_V2_LOOKAHEAD_SEC_NEAR;
  }
  if (distM < BU04_FOLLOW_V2_FAST_DISTANCE_M) {
    return BU04_FOLLOW_V2_LOOKAHEAD_SEC_MID;
  }
  return BU04_FOLLOW_V2_LOOKAHEAD_SEC_FAR;
}

MapPoint UwbFollowRestV2::tagToMap(float xcm, float ycm, const RobotPose& pose) {
  const float dxBody = ycm * 0.01f;
  const float dyBody = xcm * 0.01f;
  const float cosYaw = cosf(pose.yaw);
  const float sinYaw = sinf(pose.yaw);

  MapPoint point;
  point.x = pose.x + dxBody * cosYaw - dyBody * sinYaw;
  point.y = pose.y + dxBody * sinYaw + dyBody * cosYaw;
  return point;
}

float UwbFollowRestV2::distanceM(const UwbFrame& frame) {
  return static_cast<float>(frame.d) * 0.01f;
}

void UwbFollowRestV2::pushMeanWindow(const UwbFrame& frame) {
  meanWindow_[meanWindowHead_] = frame;
  meanWindowHead_ = static_cast<uint8_t>((meanWindowHead_ + 1U) % BU04_FOLLOW_V2_MEAN_WINDOW_SIZE);
  if (meanWindowCount_ < BU04_FOLLOW_V2_MEAN_WINDOW_SIZE) {
    ++meanWindowCount_;
  }
}

bool UwbFollowRestV2::buildMeanWindowFrame(UwbFrame& out) const {
  if (meanWindowCount_ == 0U) {
    return false;
  }

#if BU04_FOLLOW_V2_WINDOW_USE_LATEST_SAMPLE
  const uint8_t newestIndex = static_cast<uint8_t>((meanWindowHead_ +
      BU04_FOLLOW_V2_MEAN_WINDOW_SIZE - 1U) % BU04_FOLLOW_V2_MEAN_WINDOW_SIZE);
  out = meanWindow_[newestIndex];
  return true;
#else

  const uint8_t start = static_cast<uint8_t>((meanWindowHead_ +
      BU04_FOLLOW_V2_MEAN_WINDOW_SIZE - meanWindowCount_) % BU04_FOLLOW_V2_MEAN_WINDOW_SIZE);

  float meanX = 0.0f;
  float meanY = 0.0f;
  for (uint8_t i = 0; i < meanWindowCount_; ++i) {
    const UwbFrame& sample = meanWindow_[(start + i) % BU04_FOLLOW_V2_MEAN_WINDOW_SIZE];
    meanX += static_cast<float>(sample.xcm);
    meanY += static_cast<float>(sample.ycm);
  }

  meanX /= static_cast<float>(meanWindowCount_);
  meanY /= static_cast<float>(meanWindowCount_);

  float keepX = 0.0f;
  float keepY = 0.0f;
  float keepD = 0.0f;
  uint8_t keepCount = 0U;
  for (uint8_t i = 0; i < meanWindowCount_; ++i) {
    const UwbFrame& sample = meanWindow_[(start + i) % BU04_FOLLOW_V2_MEAN_WINDOW_SIZE];
    const float dx = static_cast<float>(sample.xcm) - meanX;
    const float dy = static_cast<float>(sample.ycm) - meanY;
    if (sqrtf(dx * dx + dy * dy) > BU04_FOLLOW_V2_MEAN_OUTLIER_DISTANCE_CM) {
      continue;
    }

    keepX += static_cast<float>(sample.xcm);
    keepY += static_cast<float>(sample.ycm);
    keepD += static_cast<float>(sample.d);
    ++keepCount;
  }

  if (keepCount < BU04_FOLLOW_V2_MEAN_MIN_KEEP) {
    keepX = 0.0f;
    keepY = 0.0f;
    keepD = 0.0f;
    keepCount = meanWindowCount_;
    for (uint8_t i = 0; i < meanWindowCount_; ++i) {
      const UwbFrame& sample = meanWindow_[(start + i) % BU04_FOLLOW_V2_MEAN_WINDOW_SIZE];
      keepX += static_cast<float>(sample.xcm);
      keepY += static_cast<float>(sample.ycm);
      keepD += static_cast<float>(sample.d);
    }
  }

  if (keepCount == 0U) {
    return false;
  }

  const uint8_t newestIndex = static_cast<uint8_t>((meanWindowHead_ +
      BU04_FOLLOW_V2_MEAN_WINDOW_SIZE - 1U) % BU04_FOLLOW_V2_MEAN_WINDOW_SIZE);
  out = meanWindow_[newestIndex];
  out.xcm = static_cast<long>(lroundf(keepX / static_cast<float>(keepCount)));
  out.ycm = static_cast<long>(lroundf(keepY / static_cast<float>(keepCount)));
  out.d = static_cast<long>(lroundf(keepD / static_cast<float>(keepCount)));
  return true;
#endif
}

void UwbFollowRestV2::resetKalman() {
  kalmanInitialized_ = false;
  kalmanLastUpdateMs_ = 0U;
  kalmanStateXcm_ = 0.0f;
  kalmanStateYcm_ = 0.0f;
  kalmanStateVx_ = 0.0f;
  kalmanStateVy_ = 0.0f;

  for (uint8_t i = 0; i < 4; ++i) {
    for (uint8_t j = 0; j < 4; ++j) {
      kalmanCov_[i][j] = 0.0f;
    }
  }
}

void UwbFollowRestV2::kalmanPredict(float dtSec) {
  if (!kalmanInitialized_) {
    return;
  }

  const float dt = dtSec < 0.001f ? 0.001f : dtSec;
  kalmanStateXcm_ += kalmanStateVx_ * dt;
  kalmanStateYcm_ += kalmanStateVy_ * dt;

  float next[4][4];
  const float dt2 = dt * dt;
  const float qPos = BU04_FOLLOW_V2_KALMAN_PROCESS_NOISE_POS;
  const float qVel = BU04_FOLLOW_V2_KALMAN_PROCESS_NOISE_VEL;

  for (uint8_t i = 0; i < 4; ++i) {
    for (uint8_t j = 0; j < 4; ++j) {
      next[i][j] = 0.0f;
    }
  }

  // F * P
  float fp[4][4];
  for (uint8_t j = 0; j < 4; ++j) {
    fp[0][j] = kalmanCov_[0][j] + dt * kalmanCov_[2][j];
    fp[1][j] = kalmanCov_[1][j] + dt * kalmanCov_[3][j];
    fp[2][j] = kalmanCov_[2][j];
    fp[3][j] = kalmanCov_[3][j];
  }

  // P' = F * P * F^T + Q
  for (uint8_t i = 0; i < 4; ++i) {
    next[i][0] = fp[i][0] + dt * fp[i][2];
    next[i][1] = fp[i][1] + dt * fp[i][3];
    next[i][2] = fp[i][2];
    next[i][3] = fp[i][3];
  }

  next[0][0] += qPos * (1.0f + dt2);
  next[1][1] += qPos * (1.0f + dt2);
  next[2][2] += qVel;
  next[3][3] += qVel;
  next[0][2] += qPos * dt;
  next[2][0] = next[0][2];
  next[1][3] += qPos * dt;
  next[3][1] = next[1][3];

  for (uint8_t i = 0; i < 4; ++i) {
    for (uint8_t j = 0; j < 4; ++j) {
      kalmanCov_[i][j] = next[i][j];
    }
  }
}

void UwbFollowRestV2::kalmanUpdate(float measXcm, float measYcm, float dtSec) {
  if (!kalmanInitialized_) {
    kalmanStateXcm_ = measXcm;
    kalmanStateYcm_ = measYcm;
    kalmanStateVx_ = 0.0f;
    kalmanStateVy_ = 0.0f;
    kalmanCov_[0][0] = 100.0f;
    kalmanCov_[1][1] = 100.0f;
    kalmanCov_[2][2] = 400.0f;
    kalmanCov_[3][3] = 400.0f;
    kalmanInitialized_ = true;
    return;
  }

  kalmanPredict(dtSec);

  const float z0 = measXcm;
  const float z1 = measYcm;
  const float y0 = z0 - kalmanStateXcm_;
  const float y1 = z1 - kalmanStateYcm_;
  const float r = BU04_FOLLOW_V2_KALMAN_MEAS_NOISE_CM2;

  const float s00 = kalmanCov_[0][0] + r;
  const float s01 = kalmanCov_[0][1];
  const float s10 = kalmanCov_[1][0];
  const float s11 = kalmanCov_[1][1] + r;
  const float det = s00 * s11 - s01 * s10;
  if (fabsf(det) < 1e-6f) {
    return;
  }
  const float inv00 = s11 / det;
  const float inv01 = -s01 / det;
  const float inv10 = -s10 / det;
  const float inv11 = s00 / det;

  float k[4][2];
  for (uint8_t i = 0; i < 4; ++i) {
    const float p0 = kalmanCov_[i][0];
    const float p1 = kalmanCov_[i][1];
    k[i][0] = p0 * inv00 + p1 * inv10;
    k[i][1] = p0 * inv01 + p1 * inv11;
  }

  kalmanStateXcm_ += k[0][0] * y0 + k[0][1] * y1;
  kalmanStateYcm_ += k[1][0] * y0 + k[1][1] * y1;
  kalmanStateVx_ += k[2][0] * y0 + k[2][1] * y1;
  kalmanStateVy_ += k[3][0] * y0 + k[3][1] * y1;

  float updated[4][4];
  for (uint8_t i = 0; i < 4; ++i) {
    for (uint8_t j = 0; j < 4; ++j) {
      updated[i][j] = kalmanCov_[i][j] - k[i][0] * kalmanCov_[0][j] - k[i][1] * kalmanCov_[1][j];
    }
  }

  for (uint8_t i = 0; i < 4; ++i) {
    for (uint8_t j = 0; j < 4; ++j) {
      kalmanCov_[i][j] = updated[i][j];
    }
  }

  kalmanCov_[0][1] = kalmanCov_[1][0] = 0.5f * (kalmanCov_[0][1] + kalmanCov_[1][0]);
  kalmanCov_[0][2] = kalmanCov_[2][0] = 0.5f * (kalmanCov_[0][2] + kalmanCov_[2][0]);
  kalmanCov_[0][3] = kalmanCov_[3][0] = 0.5f * (kalmanCov_[0][3] + kalmanCov_[3][0]);
  kalmanCov_[1][2] = kalmanCov_[2][1] = 0.5f * (kalmanCov_[1][2] + kalmanCov_[2][1]);
  kalmanCov_[1][3] = kalmanCov_[3][1] = 0.5f * (kalmanCov_[1][3] + kalmanCov_[3][1]);
  kalmanCov_[2][3] = kalmanCov_[3][2] = 0.5f * (kalmanCov_[2][3] + kalmanCov_[3][2]);
}

void UwbFollowRestV2::processFilteredSample(const UwbFrame& frame, unsigned long now) {
  const unsigned long prevSampleMs = lastSampleMs_;
  lastSampleMs_ = now;

  const float inputX = BU04_FOLLOW_V2_INVERT_X_AXIS ? -static_cast<float>(frame.xcm)
                                                     : static_cast<float>(frame.xcm);
  const float inputY = static_cast<float>(frame.ycm);
  const float inputAngleDeg = computeAngleDeg(static_cast<long>(inputX), static_cast<long>(inputY));
  if (inputAngleDeg < BU04_FOLLOW_V2_VALID_ANGLE_MIN_DEG ||
      inputAngleDeg > BU04_FOLLOW_V2_VALID_ANGLE_MAX_DEG) {
    if (angleInvalidSinceMs_ == 0U) {
      angleInvalidSinceMs_ = now;
    }
    if (now - angleInvalidSinceMs_ > BU04_FOLLOW_V2_INVALID_ANGLE_HOLD_MS) {
      ++rejectedSamples_;
    }
    return;
  }
  angleInvalidSinceMs_ = 0U;

  const float boundedRawX = clampJump(filteredXcm_, inputX, BU04_FOLLOW_V2_CLAMP_MAX_JUMP_CM);
  const float boundedRawY = clampJump(filteredYcm_, inputY, BU04_FOLLOW_V2_CLAMP_MAX_JUMP_CM);
  const float dtSec = prevSampleMs != 0U && now > prevSampleMs
      ? static_cast<float>(now - prevSampleMs) / 1000.0f
      : 1.0f;
  kalmanUpdate(boundedRawX, boundedRawY, dtSec);
  filteredXcm_ = kalmanStateXcm_;
  filteredYcm_ = kalmanStateYcm_;
  velocityXcmPerSec_ = kalmanStateVx_;
  velocityYcmPerSec_ = kalmanStateVy_;
  hasFiltered_ = true;
  hasVelocity_ = true;

  if (prevSampleMs == 0U) {
    kalmanLastUpdateMs_ = now;
  } else {
    kalmanLastUpdateMs_ = now;
  }

  const float distM = distanceM(frame);
  if (distM <= BU04_FOLLOW_V2_SAFETY_STOP_DISTANCE_M) {
    if (safetyStopSinceMs_ == 0U) {
      safetyStopSinceMs_ = now;
    }
    if (actionActive_ && now - safetyStopSinceMs_ >= BU04_FOLLOW_V2_SAFETY_STOP_HOLD_MS) {
      cancelCurrentAction();
      actionActive_ = false;
      currentActionId_ = "";
      clearQueue();
      stationaryHoldActive_ = true;
    }
    return;
  }
  safetyStopSinceMs_ = 0U;
  if (distM <= BU04_FOLLOW_V2_TOO_CLOSE_DISTANCE_M) {
    clearQueue();
    stationaryHoldActive_ = true;
    return;
  }

  if (!hasPose_) {
    return;
  }

  float lookaheadSec = chooseLookaheadSec(distM);
  if (distM < BU04_FOLLOW_V2_SLOW_DISTANCE_M) {
    const float denom = BU04_FOLLOW_V2_SLOW_DISTANCE_M - BU04_FOLLOW_V2_SAFETY_STOP_DISTANCE_M;
    if (denom > 0.001f) {
      float scale = (distM - BU04_FOLLOW_V2_SAFETY_STOP_DISTANCE_M) / denom;
      scale = clampFloat(scale, 0.15f, 1.0f);
      lookaheadSec *= scale;
    }
  }

  float predictedXcm = filteredXcm_;
  float predictedYcm = filteredYcm_;
  if (hasVelocity_) {
    predictedXcm += velocityXcmPerSec_ * lookaheadSec;
    predictedYcm += velocityYcmPerSec_ * lookaheadSec;
  }

  const MapPoint target = tagToMap(predictedXcm, predictedYcm, lastPose_);
  if (shouldResumeFromStationaryHold()) {
    if (actionActive_) {
      cancelCurrentAction();
      actionActive_ = false;
      currentActionId_ = "";
    }
    clearQueue();
    stationaryHoldActive_ = false;
  }
  if (shouldSuppressStationaryPoint(target)) {
    clearQueue();
    stationaryHoldActive_ = true;
    return;
  }

#if BU04_FOLLOW_V2_ENABLE_FIXED_1HZ
  if (dispatchFixedPoint(target, now)) {
    ++totalPoints_;
  }
  return;
#else
  pushPoint(target);
#endif
}

bool UwbFollowRestV2::dispatchFixedPoint(const MapPoint& point, unsigned long now) {
  if (!prepareActionForDispatch(now)) {
    return false;
  }

  if (now - lastDispatchMs_ < BU04_FOLLOW_V2_MIN_SEND_INTERVAL_MS) {
    return false;
  }
  if (now - lastRestAttemptMs_ < BU04_FOLLOW_V2_REST_RETRY_MS) {
    return false;
  }

  String actionId;
  if (startFollowPathPoints(&point,
                            1,
                            atan2f(point.y - lastPose_.y, point.x - lastPose_.x),
                            BU04_FOLLOW_V2_PATH_SPEED_RATIO,
                            BU04_FOLLOW_V2_PATH_ACCEPTABLE_PRECISION_M,
                            actionId)) {
    actionActive_ = true;
    currentActionId_ = actionId;
    lastDispatchMs_ = now;
    stationaryHoldActive_ = false;
    ++totalActions_;
    debug_.print("[REST FOLLOW V2] dispatched fixed point=(");
    debug_.print(point.x, 3);
    debug_.print(",");
    debug_.print(point.y, 3);
    debug_.println(")");
    return true;
  }

  lastRestAttemptMs_ = now;
  return false;
}

bool UwbFollowRestV2::prepareActionForDispatch(unsigned long now) {
  if (!actionActive_ || currentActionId_.isEmpty()) {
    return true;
  }

  if (now - lastActionCheckMs_ < BU04_FOLLOW_V2_ACTION_CHECK_MS) {
    return false;
  }
  lastActionCheckMs_ = now;

  bool running = false;
  if (!getActionRunning(currentActionId_, running)) {
    lastRestAttemptMs_ = now;
    return false;
  }

  if (running) {
    if (!cancelCurrentAction()) {
      lastRestAttemptMs_ = now;
      return false;
    }
  }

  actionActive_ = false;
  currentActionId_ = "";
  return true;
}

bool UwbFollowRestV2::shouldSuppressStationaryPoint(const MapPoint& point) const {
  if (!hasPose_ || !hasVelocity_) {
    return false;
  }

  const float speedCmPerSec = sqrtf(velocityXcmPerSec_ * velocityXcmPerSec_ +
                                    velocityYcmPerSec_ * velocityYcmPerSec_);
  if (speedCmPerSec > BU04_FOLLOW_V2_STATIONARY_SPEED_CMPS) {
    return false;
  }

  const float dx = point.x - lastPose_.x;
  const float dy = point.y - lastPose_.y;
  const float robotDistM = sqrtf(dx * dx + dy * dy);
  return robotDistM <= BU04_FOLLOW_V2_STATIONARY_ROBOT_DISTANCE_M;
}

bool UwbFollowRestV2::shouldResumeFromStationaryHold() const {
  if (!stationaryHoldActive_) {
    return false;
  }

  const float speedCmPerSec = sqrtf(velocityXcmPerSec_ * velocityXcmPerSec_ +
                                    velocityYcmPerSec_ * velocityYcmPerSec_);
  return speedCmPerSec >= BU04_FOLLOW_V2_STATIONARY_RESUME_SPEED_CMPS;
}

void UwbFollowRestV2::processSample(const UwbFrame& frame) {
  const unsigned long now = millis();
#if BU04_FOLLOW_V2_ENABLE_ROLLING_WINDOW
  if (!windowBucketActive_) {
    meanWindowCount_ = 0U;
    meanWindowHead_ = 0U;
    windowBucketStartMs_ = now;
    windowBucketLastAcceptMs_ = now;
    windowBucketActive_ = true;
    pushMeanWindow(frame);
    return;
  }

  if (now - windowBucketStartMs_ < BU04_FOLLOW_V2_WINDOW_BUCKET_MS) {
    if (windowBucketLastAcceptMs_ != 0U &&
        now - windowBucketLastAcceptMs_ < BU04_FOLLOW_V2_WINDOW_ACCEPT_INTERVAL_MS) {
      return;
    }
    windowBucketLastAcceptMs_ = now;
    pushMeanWindow(frame);
    return;
  }

  UwbFrame filteredFrame;
  if (!buildMeanWindowFrame(filteredFrame)) {
    filteredFrame = frame;
  }
  processFilteredSample(filteredFrame, now);
  meanWindowCount_ = 0U;
  meanWindowHead_ = 0U;
  windowBucketStartMs_ = now;
  windowBucketLastAcceptMs_ = now;
  pushMeanWindow(frame);
  return;
#elif BU04_FOLLOW_V2_ENABLE_FIXED_1HZ
  const unsigned long prevSampleMs = lastSampleMs_;
  if (prevSampleMs != 0U && now - prevSampleMs < BU04_FOLLOW_V2_SAMPLE_INTERVAL_MS) {
    return;
  }
  processFilteredSample(frame, now);
  return;
#endif
}

void UwbFollowRestV2::pushPoint(const MapPoint& point) {
  if (BU04_FOLLOW_V2_ENABLE_POINT_SPACING_GUARD && !queueEmpty()) {
    const MapPoint& newest = queueAt(static_cast<uint8_t>((queueTail_ + BU04_FOLLOW_V2_QUEUE_SIZE - 1U) %
                                                          BU04_FOLLOW_V2_QUEUE_SIZE));
    const float dx = point.x - newest.x;
    const float dy = point.y - newest.y;
    if (sqrtf(dx * dx + dy * dy) < (BU04_FOLLOW_V2_MIN_POINT_SPACING_CM * 0.01f)) {
      return;
    }
  }

  if (queueCount_ >= BU04_FOLLOW_V2_QUEUE_SIZE) {
    queueHead_ = static_cast<uint8_t>((queueHead_ + 1U) % BU04_FOLLOW_V2_QUEUE_SIZE);
    --queueCount_;
  }

  queue_[queueTail_] = point;
  queueTail_ = static_cast<uint8_t>((queueTail_ + 1U) % BU04_FOLLOW_V2_QUEUE_SIZE);
  ++queueCount_;
  ++totalPoints_;
}

bool UwbFollowRestV2::queueEmpty() const {
  return queueCount_ == 0;
}

const MapPoint& UwbFollowRestV2::queueAt(uint8_t index) const {
  return queue_[index % BU04_FOLLOW_V2_QUEUE_SIZE];
}

void UwbFollowRestV2::popQueue(uint8_t count) {
  while (count-- > 0 && queueCount_ > 0) {
    queueHead_ = static_cast<uint8_t>((queueHead_ + 1U) % BU04_FOLLOW_V2_QUEUE_SIZE);
    --queueCount_;
  }
}

void UwbFollowRestV2::clearQueue() {
  queueHead_ = 0;
  queueTail_ = 0;
  queueCount_ = 0;
}

bool UwbFollowRestV2::dispatchSinglePointIfNeeded() {
  if (queueEmpty()) {
    return false;
  }

  const unsigned long now = millis();
  if (!prepareActionForDispatch(now)) {
    return false;
  }
  if (now - lastDispatchMs_ < BU04_FOLLOW_V2_MIN_SEND_INTERVAL_MS) {
    return false;
  }
  if (now - lastRestAttemptMs_ < BU04_FOLLOW_V2_REST_RETRY_MS) {
    return false;
  }
  if (!refreshPose()) {
    return false;
  }

  const MapPoint target = queueAt(queueHead_);
  const float yawRad = atan2f(target.y - lastPose_.y, target.x - lastPose_.x);
  String actionId;
  if (startFollowPathPoints(&target,
                            1,
                            yawRad,
                            BU04_FOLLOW_V2_PATH_SPEED_RATIO,
                            BU04_FOLLOW_V2_PATH_ACCEPTABLE_PRECISION_M,
                            actionId)) {
    popQueue(1);
    actionActive_ = true;
    currentActionId_ = actionId;
    lastDispatchMs_ = now;
    stationaryHoldActive_ = false;
    ++totalActions_;
    return true;
  }

  lastRestAttemptMs_ = now;
  return false;
}

bool UwbFollowRestV2::dispatchRollingWindowIfNeeded() {
  if (queueEmpty()) {
    return false;
  }

  const unsigned long now = millis();
  if (!refreshPose()) {
    return false;
  }

  const bool startupDispatch = (totalActions_ == 0U);
  if (startupDispatch && queueCount_ < BU04_FOLLOW_V2_BOOTSTRAP_POINTS) {
    return false;
  }

  if (!prepareActionForDispatch(now)) {
    return false;
  }
  if (now - lastDispatchMs_ < BU04_FOLLOW_V2_BATCH_REISSUE_MS) {
    return false;
  }
  if (queueCount_ < BU04_FOLLOW_V2_BATCH_MIN_RESERVE) {
    return false;
  }

  const uint8_t minPoints = startupDispatch ? 1U : 2U;
  if (queueCount_ < minPoints) {
    return false;
  }

  const uint8_t windowLimit = startupDispatch ? BU04_FOLLOW_V2_STARTUP_BATCH_SIZE
                                              : BU04_FOLLOW_V2_BATCH_SIZE;
  const uint8_t windowCount = static_cast<uint8_t>(queueCount_ < windowLimit ? queueCount_ : windowLimit);
  return dispatchPathPoints(windowCount, BU04_FOLLOW_V2_BATCH_ADVANCE_POINTS);
}

bool UwbFollowRestV2::dispatchPathPoints(uint8_t count, uint8_t advanceCount) {
  if (count == 0U || queueCount_ == 0U) {
    return false;
  }
  if (!refreshPose()) {
    return false;
  }

  if (count > queueCount_) {
    count = queueCount_;
  }

  MapPoint points[BU04_FOLLOW_V2_BATCH_SIZE];
  for (uint8_t i = 0; i < count; ++i) {
    points[i] = queueAt(static_cast<uint8_t>((queueHead_ + i) % BU04_FOLLOW_V2_QUEUE_SIZE));
  }

  const MapPoint& first = points[0];
  const MapPoint& last = points[count - 1U];
  float yawRad = 0.0f;
  if (count >= 2U) {
    const MapPoint& prev = points[count - 2U];
    yawRad = atan2f(last.y - prev.y, last.x - prev.x);
  } else {
    yawRad = atan2f(last.y - lastPose_.y, last.x - lastPose_.x);
  }

  String actionId;
  if (startFollowPathPoints(points,
                            count,
                            yawRad,
                            BU04_FOLLOW_V2_WINDOW_PATH_SPEED_RATIO,
                            BU04_FOLLOW_V2_WINDOW_PATH_ACCEPTABLE_PRECISION_M,
                            actionId)) {
    const uint8_t consumed = static_cast<uint8_t>(advanceCount == 0U ? 1U : advanceCount);
    popQueue(consumed > count ? count : consumed);
    actionActive_ = true;
    currentActionId_ = actionId;
    lastDispatchMs_ = millis();
    stationaryHoldActive_ = false;
    ++totalActions_;
    debug_.print("[REST FOLLOW V2] dispatched points=");
    debug_.print(count);
    debug_.print(" first=(");
    debug_.print(first.x, 3);
    debug_.print(",");
    debug_.print(first.y, 3);
    debug_.print(") last=(");
    debug_.print(last.x, 3);
    debug_.print(",");
    debug_.print(last.y, 3);
    debug_.println(")");
    return true;
  }

  lastRestAttemptMs_ = millis();
  return false;
}

void UwbFollowRestV2::checkCurrentAction() {
  if (!actionActive_) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastActionCheckMs_ < BU04_FOLLOW_V2_ACTION_CHECK_MS) {
    return;
  }
  lastActionCheckMs_ = now;

  bool running = false;
  if (!getActionRunning(currentActionId_, running)) {
    lastRestAttemptMs_ = now;
    return;
  }

  if (!running) {
    actionActive_ = false;
    currentActionId_ = "";
  }
}

void UwbFollowRestV2::logStatusIfDue() {
  const unsigned long now = millis();
  if (now - lastStatusLogMs_ < BU04_FOLLOW_V2_STATUS_LOG_MS) {
    return;
  }
  lastStatusLogMs_ = now;
  debug_.print("[REST FOLLOW V2] frames=");
  debug_.print(totalFrames_);
  debug_.print(" points=");
  debug_.print(totalPoints_);
  debug_.print(" actions=");
  debug_.print(totalActions_);
  debug_.print(" queue=");
  debug_.print(queueCount_);
  debug_.print(" rejected=");
  debug_.print(rejectedSamples_);
  debug_.print(" pose=");
  debug_.print(hasPose_ ? "1" : "0");
  debug_.print(" active=");
  debug_.println(actionActive_ ? "1" : "0");
}

String UwbFollowRestV2::baseUrl() const {
  String url = "http://";
  url += follow_v2::robotIp().toString();
  url += ":";
  url += String(follow_v2::kRobotPort);
  return url;
}

float UwbFollowRestV2::degToRad(float value) {
  return value * PI / 180.0f;
}

bool UwbFollowRestV2::extractNumberField(const String& text, const char* key, float& value) {
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

bool UwbFollowRestV2::extractStringField(const String& text, const char* key, String& value) {
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

bool UwbFollowRestV2::requestJson(const String& method,
                                  const String& url,
                                  const String& body,
                                  int& httpCode,
                                  String& response) {
  HTTPClient http;
  if (!http.begin(url)) {
    return false;
  }

  http.addHeader("Content-Type", "application/json");

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

bool UwbFollowRestV2::getPose(RobotPose& pose) {
  int httpCode = 0;
  String body;
  if (!requestJson("GET", baseUrl() + "/api/core/slam/v1/localization/pose", "", httpCode, body)) {
    return false;
  }
  if (httpCode != HTTP_CODE_OK) {
    return false;
  }

  return extractNumberField(body, "x", pose.x) &&
         extractNumberField(body, "y", pose.y) &&
         extractNumberField(body, "yaw", pose.yaw);
}

bool UwbFollowRestV2::startFollowPathPoints(const MapPoint* points,
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
  body += String(BU04_FOLLOW_V2_PATH_FAIL_RETRY_COUNT);
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

bool UwbFollowRestV2::getActionRunning(const String& actionId, bool& running) {
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
      if (status != 4L) {
        running = true;
        return true;
      }
    }
  }

  if (!parseLongField(response, "status", status)) {
    if (response.indexOf("\"action_name\"") < 0) {
      running = false;
      return true;
    }
  } else if (status != 4L) {
    running = true;
    return true;
  }

  if (response.indexOf("success") >= 0 ||
      response.indexOf("succeeded") >= 0 ||
      response.indexOf("finished") >= 0 ||
      response.indexOf("completed") >= 0 ||
      response.indexOf("failed") >= 0 ||
      response.indexOf("canceled") >= 0 ||
      response.indexOf("cancelled") >= 0) {
    running = false;
    return true;
  }

  running = false;
  return true;
}

bool UwbFollowRestV2::cancelCurrentAction() {
  int httpCode = 0;
  String response;
  if (!requestJson("DELETE", baseUrl() + "/api/core/motion/v1/actions/:current", "", httpCode, response)) {
    return false;
  }
  return httpCode >= 200 && httpCode < 300;
}

}  // namespace follow_v2
