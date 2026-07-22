#include "UwbFollowRest.h"

#include <ctype.h>
#include <math.h>

namespace follow_demo {

namespace {

float degAbs(float value) {
  return value < 0.0f ? -value : value;
}

}  // namespace

void UwbFollowRest::sortLongs(long* values, uint8_t count) {
  for (uint8_t i = 1; i < count; ++i) {
    const long key = values[i];
    int8_t j = static_cast<int8_t>(i) - 1;
    while (j >= 0 && values[j] > key) {
      values[j + 1] = values[j];
      --j;
    }
    values[j + 1] = key;
  }
}

long UwbFollowRest::medianLong(const long* values, uint8_t count) {
  if (count == 0) {
    return 0;
  }

  long temp[BU04_FOLLOW_MEAN_WINDOW_SIZE];
  for (uint8_t i = 0; i < count; ++i) {
    temp[i] = values[i];
  }

  sortLongs(temp, count);
  const uint8_t mid = static_cast<uint8_t>(count / 2U);
  if ((count & 1U) != 0U) {
    return temp[mid];
  }
  return (temp[mid - 1U] + temp[mid]) / 2L;
}

void UwbFollowRest::pushMeanWindow(const UwbFrame& frame) {
  meanWindow_[meanWindowHead_] = frame;
  meanWindowHead_ = static_cast<uint8_t>((meanWindowHead_ + 1U) % BU04_FOLLOW_MEAN_WINDOW_SIZE);
  if (meanWindowCount_ < BU04_FOLLOW_MEAN_WINDOW_SIZE) {
    ++meanWindowCount_;
  }
}

bool UwbFollowRest::computeRobustMean(UwbFrame& out) {
  if (meanWindowCount_ == 0) {
    return false;
  }

  long xWindow[BU04_FOLLOW_MEAN_WINDOW_SIZE];
  long yWindow[BU04_FOLLOW_MEAN_WINDOW_SIZE];
  for (uint8_t i = 0; i < meanWindowCount_; ++i) {
    const uint8_t index = static_cast<uint8_t>((meanWindowHead_ + BU04_FOLLOW_MEAN_WINDOW_SIZE - meanWindowCount_ + i) %
                                               BU04_FOLLOW_MEAN_WINDOW_SIZE);
    const UwbFrame& sample = meanWindow_[index];
    xWindow[i] = sample.xcm;
    yWindow[i] = sample.ycm;
  }

  const long refXcm = medianLong(xWindow, meanWindowCount_);
  const long refYcm = medianLong(yWindow, meanWindowCount_);

  long sumXcm = 0;
  long sumYcm = 0;
  uint8_t keptCount = 0;
  const UwbFrame* lastKept = nullptr;
  const UwbFrame* lastSample = &meanWindow_[(meanWindowHead_ + BU04_FOLLOW_MEAN_WINDOW_SIZE - 1U) %
                                            BU04_FOLLOW_MEAN_WINDOW_SIZE];
  uint32_t droppedCount = 0;

  for (uint8_t i = 0; i < meanWindowCount_; ++i) {
    const uint8_t index = static_cast<uint8_t>((meanWindowHead_ + BU04_FOLLOW_MEAN_WINDOW_SIZE - meanWindowCount_ + i) %
                                               BU04_FOLLOW_MEAN_WINDOW_SIZE);
    const UwbFrame& sample = meanWindow_[index];
    const long dx = sample.xcm - refXcm;
    const long dy = sample.ycm - refYcm;
    const float jumpCm = sqrtf(static_cast<float>(dx * dx + dy * dy));

    const bool keepSample =
        (jumpCm <= BU04_FOLLOW_MEAN_OUTLIER_DISTANCE_CM) || (keptCount < BU04_FOLLOW_MEAN_MIN_KEEP);
    if (!keepSample) {
      ++droppedCount;
      continue;
    }

    lastKept = &sample;
    sumXcm += sample.xcm;
    sumYcm += sample.ycm;
    ++keptCount;
  }

  if (keptCount == 0) {
    lastKept = lastSample;
    sumXcm = lastSample->xcm;
    sumYcm = lastSample->ycm;
    keptCount = 1;
  }

  const UwbFrame& base = lastKept != nullptr ? *lastKept : *lastSample;
  out = base;
  out.t = lastSample->t;
  out.d = lastSample->d;
  out.xcm = sumXcm / keptCount;
  out.ycm = sumYcm / keptCount;
  rejectedSamples_ += droppedCount;
  return true;
}

#if BU04_FOLLOW_ENABLE_KALMAN_FILTER
void UwbFollowRest::resetKalmanFilter(Kalman1D& filter, float measurement) {
  filter.pos = measurement;
  filter.vel = 0.0f;
  filter.p00 = BU04_FOLLOW_KALMAN_INIT_POS_VAR;
  filter.p01 = 0.0f;
  filter.p10 = 0.0f;
  filter.p11 = BU04_FOLLOW_KALMAN_INIT_VEL_VAR;
  filter.initialized = true;
}

void UwbFollowRest::predictKalmanFilter(Kalman1D& filter, float dtSec) {
  if (!filter.initialized || dtSec <= 0.0f) {
    return;
  }

  filter.pos += filter.vel * dtSec;

  const float dt2 = dtSec * dtSec;
  const float dt3 = dt2 * dtSec;
  const float dt4 = dt2 * dt2;
  const float q = BU04_FOLLOW_KALMAN_ACCEL_NOISE_CM2_PER_S4;

  const float p00 = filter.p00;
  const float p01 = filter.p01;
  const float p10 = filter.p10;
  const float p11 = filter.p11;

  filter.p00 = p00 + dtSec * (p01 + p10) + dt2 * p11 + 0.25f * q * dt4;
  filter.p01 = p01 + dtSec * p11 + 0.5f * q * dt3;
  filter.p10 = p10 + dtSec * p11 + 0.5f * q * dt3;
  filter.p11 = p11 + q * dt2;
}

void UwbFollowRest::updateKalmanFilter(Kalman1D& filter, float measurement) {
  if (!filter.initialized) {
    resetKalmanFilter(filter, measurement);
    return;
  }

  const float oldP00 = filter.p00;
  const float oldP01 = filter.p01;
  const float oldP10 = filter.p10;
  const float oldP11 = filter.p11;

  const float innovation = measurement - filter.pos;
  const float s = oldP00 + BU04_FOLLOW_KALMAN_MEAS_NOISE_CM2;
  if (s <= 0.0f) {
    return;
  }

  const float k0 = oldP00 / s;
  const float k1 = oldP10 / s;
  filter.pos += k0 * innovation;
  filter.vel += k1 * innovation;

  filter.p00 = (1.0f - k0) * oldP00;
  filter.p01 = (1.0f - k0) * oldP01;
  filter.p10 = oldP10 - k1 * oldP00;
  filter.p11 = oldP11 - k1 * oldP01;

  const float sym = 0.5f * (filter.p01 + filter.p10);
  filter.p01 = sym;
  filter.p10 = sym;
}

void UwbFollowRest::stepKalmanFilter(Kalman1D& filter, float measurement, float dtSec) {
  if (!filter.initialized) {
    resetKalmanFilter(filter, measurement);
    return;
  }

  predictKalmanFilter(filter, dtSec);
  updateKalmanFilter(filter, measurement);
}
#endif

UwbFollowRest::UwbFollowRest(Stream& debug)
    : debug_(debug),
      rest_(debug),
      expectedLength_(0),
      state_(State::kWaitJ),
      lastWifiAttempt_(0),
      lastMotionSampleMs_(0),
      angleInvalidSinceMs_(0),
      safetyStopSinceMs_(0),
      lastActionCheckMs_(0),
      lastStatusLogMs_(0),
      lastDispatchMs_(0),
      lastForcedDispatchMs_(0),
      lastLatestAngleDispatchMs_(0),
      lastRestAttemptMs_(0),
      lastGoodFrameMs_(0),
      wifiReady_(false),
      hasFiltered_(false),
      hasVelocity_(false),
      filteredXcm_(0.0f),
      filteredYcm_(0.0f),
      velocityXcmPerSec_(0.0f),
      velocityYcmPerSec_(0.0f),
      meanWindowCount_(0),
      meanWindowHead_(0),
      hasPose_(false),
      hasFrame_(false),
      queueHead_(0),
      queueTail_(0),
      queueCount_(0),
      actionActive_(false),
      hasCurrentTarget_(false),
      hasLatestAngle_(false),
      latestAngleDeg_(0.0f),
      latestAngleRad_(0.0f),
      totalFrames_(0),
      totalPoints_(0),
      totalActions_(0),
      parseFails_(0),
      rejectedSamples_(0) {
  lenBuffer_.reserve(8);
  payloadBuffer_.reserve(256);
  currentActionId_.reserve(64);
}

void UwbFollowRest::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(follow_demo::kWifiSsid, follow_demo::kWifiPassword);
  debug_.println("REST follow controller starting...");
}

void UwbFollowRest::update(Bu04Uart& dataUart) {
  ensureWifi();

  char c;
  while (dataUart.readByte(c)) {
    handleIncomingByte(c);
  }

  if (hasFrame_ && millis() - lastGoodFrameMs_ > BU04_FOLLOW_LOST_SIGNAL_TIMEOUT_MS) {
    if (actionActive_) {
      rest_.cancelCurrentAction();
      actionActive_ = false;
      currentActionId_ = "";
    }
  }

  checkCurrentAction();
#if BU04_FOLLOW_ENABLE_LATEST_ANGLE_DISPATCH
  if (!dispatchLatestAngleIfNeeded()) {
    dispatchNextIfNeeded();
  }
#else
  dispatchNextIfNeeded();
#endif
  logStatusIfDue();
}

void UwbFollowRest::ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiReady_) {
      debug_.print("WiFi connected, IP: ");
      debug_.println(WiFi.localIP());
      wifiReady_ = true;
    }
    return;
  }

  wifiReady_ = false;
  if (millis() - lastWifiAttempt_ < BU04_FOLLOW_WIFI_RECONNECT_MS) {
    return;
  }
  lastWifiAttempt_ = millis();

  debug_.println("Reconnecting WiFi for REST follow...");
  WiFi.disconnect();
  WiFi.begin(follow_demo::kWifiSsid, follow_demo::kWifiPassword);
}

bool UwbFollowRest::refreshPose() {
  RobotPose pose;
  if (rest_.getPose(pose)) {
    lastPose_ = pose;
    hasPose_ = true;
    return true;
  }
  return false;
}

void UwbFollowRest::handleIncomingByte(char c) {
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

void UwbFollowRest::resetFrame() {
  state_ = State::kWaitJ;
  lenBuffer_ = "";
  payloadBuffer_ = "";
  expectedLength_ = 0;
}

bool UwbFollowRest::parseStringField(const String& text, const char* key, String& value) {
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

bool UwbFollowRest::parseLongField(const String& text, const char* key, long& value) {
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

bool UwbFollowRest::parseFrame(const String& frame, UwbFrame& out) {
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

float UwbFollowRest::computeAngleDeg(long xcm, long ycm) {
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

float UwbFollowRest::clampJump(float prev, float next, float maxJump) {
  const float delta = next - prev;
  if (delta > maxJump) {
    return prev + maxJump;
  }
  if (delta < -maxJump) {
    return prev - maxJump;
  }
  return next;
}

float UwbFollowRest::chooseLookaheadSec(float distM) {
  if (distM < BU04_FOLLOW_TOO_CLOSE_DISTANCE_M) {
    return BU04_FOLLOW_LOOKAHEAD_SEC_NEAR;
  }
  if (distM < BU04_FOLLOW_SLOW_DISTANCE_M) {
    return BU04_FOLLOW_LOOKAHEAD_SEC_NEAR;
  }
  if (distM < BU04_FOLLOW_FAST_DISTANCE_M) {
    return BU04_FOLLOW_LOOKAHEAD_SEC_MID;
  }
  return BU04_FOLLOW_LOOKAHEAD_SEC_FAR;
}

MapPoint UwbFollowRest::tagToMap(float xcm, float ycm, const RobotPose& pose) {
  const float dxBody = ycm * 0.01f;
  const float dyBody = xcm * 0.01f;
  const float cosYaw = cosf(pose.yaw);
  const float sinYaw = sinf(pose.yaw);

  MapPoint point;
  point.x = pose.x + dxBody * cosYaw - dyBody * sinYaw;
  point.y = pose.y + dxBody * sinYaw + dyBody * cosYaw;
  return point;
}

float UwbFollowRest::distanceM(const UwbFrame& frame) {
  return static_cast<float>(frame.d) * 0.01f;
}

void UwbFollowRest::processSample(const UwbFrame& frame) {
  const unsigned long now = millis();
  const float inputX = BU04_FOLLOW_INVERT_X_AXIS ? -static_cast<float>(frame.xcm)
                                                 : static_cast<float>(frame.xcm);
  const float inputY = static_cast<float>(frame.ycm);
  const float inputAngleDeg = computeAngleDeg(static_cast<long>(inputX), static_cast<long>(inputY));
  const bool inputAngleValid =
      inputAngleDeg >= BU04_FOLLOW_VALID_ANGLE_MIN_DEG && inputAngleDeg <= BU04_FOLLOW_VALID_ANGLE_MAX_DEG;
  if (!inputAngleValid) {
    if (angleInvalidSinceMs_ == 0U) {
      angleInvalidSinceMs_ = now;
    }
    if (now - angleInvalidSinceMs_ > BU04_FOLLOW_INVALID_ANGLE_HOLD_MS) {
      ++rejectedSamples_;
    }
    return;
  }
  angleInvalidSinceMs_ = 0;

  const float rawX = BU04_FOLLOW_INVERT_X_AXIS ? -static_cast<float>(frame.xcm)
                                               : static_cast<float>(frame.xcm);
  const float rawY = static_cast<float>(frame.ycm);

#if BU04_FOLLOW_ENABLE_LATEST_ANGLE_DISPATCH
  filteredXcm_ = rawX;
  filteredYcm_ = rawY;
  hasFiltered_ = true;
  hasVelocity_ = false;
  lastMotionSampleMs_ = now;
#else
  const unsigned long prevMotionSampleMs = lastMotionSampleMs_;
  pushMeanWindow(frame);

  UwbFrame averagedFrame;
  if (!computeRobustMean(averagedFrame)) {
    averagedFrame = frame;
  }

  const float filteredRawX = BU04_FOLLOW_INVERT_X_AXIS ? -static_cast<float>(averagedFrame.xcm)
                                                       : static_cast<float>(averagedFrame.xcm);
  const float filteredRawY = static_cast<float>(averagedFrame.ycm);
  const bool resetFilter = (meanWindowCount_ <= 1U);

#if BU04_FOLLOW_ENABLE_KALMAN_FILTER
  if (resetFilter) {
    kalmanX_.initialized = false;
    kalmanY_.initialized = false;
    hasFiltered_ = false;
    hasVelocity_ = false;
  }

  const float dtSec = (prevMotionSampleMs != 0 && now > prevMotionSampleMs)
                          ? static_cast<float>(now - prevMotionSampleMs) / 1000.0f
                          : 0.0f;

  stepKalmanFilter(kalmanX_, filteredRawX, dtSec);
  stepKalmanFilter(kalmanY_, filteredRawY, dtSec);
  filteredXcm_ = kalmanX_.pos;
  filteredYcm_ = kalmanY_.pos;
  velocityXcmPerSec_ = kalmanX_.vel;
  velocityYcmPerSec_ = kalmanY_.vel;
  hasFiltered_ = true;
  hasVelocity_ = true;
#else
  if (resetFilter || !hasFiltered_) {
    filteredXcm_ = filteredRawX;
    filteredYcm_ = filteredRawY;
    hasFiltered_ = true;
    hasVelocity_ = false;
  } else {
    const float prevX = filteredXcm_;
    const float prevY = filteredYcm_;
    const float boundedRawX = clampJump(prevX, filteredRawX, BU04_FOLLOW_CLAMP_MAX_JUMP_CM);
    const float boundedRawY = clampJump(prevY, filteredRawY, BU04_FOLLOW_CLAMP_MAX_JUMP_CM);
    filteredXcm_ = BU04_FOLLOW_EMA_ALPHA * boundedRawX + (1.0f - BU04_FOLLOW_EMA_ALPHA) * prevX;
    filteredYcm_ = BU04_FOLLOW_EMA_ALPHA * boundedRawY + (1.0f - BU04_FOLLOW_EMA_ALPHA) * prevY;

    if (prevMotionSampleMs != 0 && now > prevMotionSampleMs) {
      const float dtSec = static_cast<float>(now - prevMotionSampleMs) / 1000.0f;
      if (dtSec > 0.001f) {
        const float instVx = (filteredXcm_ - prevX) / dtSec;
        const float instVy = (filteredYcm_ - prevY) / dtSec;
        velocityXcmPerSec_ = BU04_FOLLOW_VELOCITY_ALPHA * instVx +
                              (1.0f - BU04_FOLLOW_VELOCITY_ALPHA) * velocityXcmPerSec_;
        velocityYcmPerSec_ = BU04_FOLLOW_VELOCITY_ALPHA * instVy +
                              (1.0f - BU04_FOLLOW_VELOCITY_ALPHA) * velocityYcmPerSec_;
        hasVelocity_ = true;
      }
    }
  }
#endif
#endif

  const float distM = distanceM(frame);
  if (distM <= BU04_FOLLOW_SAFETY_STOP_DISTANCE_M) {
    if (safetyStopSinceMs_ == 0U) {
      safetyStopSinceMs_ = now;
    }
    if (actionActive_ && now - safetyStopSinceMs_ >= BU04_FOLLOW_SAFETY_STOP_HOLD_MS) {
      rest_.cancelCurrentAction();
      actionActive_ = false;
      currentActionId_ = "";
    }
    return;
  }
  safetyStopSinceMs_ = 0U;
  if (distM <= BU04_FOLLOW_TOO_CLOSE_DISTANCE_M) {
    return;
  }

  float lookaheadSec = chooseLookaheadSec(distM);
  if (distM < BU04_FOLLOW_SLOWDOWN_DISTANCE_M) {
    const float denom = BU04_FOLLOW_SLOWDOWN_DISTANCE_M - BU04_FOLLOW_SAFETY_STOP_DISTANCE_M;
    if (denom > 0.001f) {
      float scale = (distM - BU04_FOLLOW_SAFETY_STOP_DISTANCE_M) / denom;
      if (scale < 0.15f) {
        scale = 0.15f;
      } else if (scale > 1.0f) {
        scale = 1.0f;
      }
      lookaheadSec *= scale;
    }
  }
  float predictedXcm = filteredXcm_;
  float predictedYcm = filteredYcm_;
  if (hasVelocity_) {
    predictedXcm += velocityXcmPerSec_ * lookaheadSec;
    predictedYcm += velocityYcmPerSec_ * lookaheadSec;
  }

  FollowTarget point;
  point.xcm = predictedXcm;
  point.ycm = predictedYcm;
  pushPoint(point);
  lastMotionSampleMs_ = now;
}

void UwbFollowRest::pushPoint(const FollowTarget& point) {
  if (BU04_FOLLOW_ENABLE_POINT_SPACING_GUARD && !queueEmpty()) {
    const FollowTarget& newest = queueAt(static_cast<uint8_t>((queueTail_ + BU04_FOLLOW_QUEUE_SIZE - 1U) % BU04_FOLLOW_QUEUE_SIZE));
    const float dx = point.xcm - newest.xcm;
    const float dy = point.ycm - newest.ycm;
    if (sqrtf(dx * dx + dy * dy) < (BU04_FOLLOW_MIN_POINT_SPACING_CM * 0.01f)) {
      return;
    }
  }

  if (queueCount_ >= BU04_FOLLOW_QUEUE_SIZE) {
    queueHead_ = static_cast<uint8_t>((queueHead_ + 1U) % BU04_FOLLOW_QUEUE_SIZE);
    --queueCount_;
  }

  queue_[queueTail_] = point;
  queueTail_ = static_cast<uint8_t>((queueTail_ + 1U) % BU04_FOLLOW_QUEUE_SIZE);
  ++queueCount_;
  ++totalPoints_;
}

bool UwbFollowRest::queueEmpty() const {
  return queueCount_ == 0;
}

const FollowTarget& UwbFollowRest::queueAt(uint8_t index) const {
  return queue_[index % BU04_FOLLOW_QUEUE_SIZE];
}

void UwbFollowRest::popQueue(uint8_t count) {
  while (count-- > 0 && queueCount_ > 0) {
    queueHead_ = static_cast<uint8_t>((queueHead_ + 1U) % BU04_FOLLOW_QUEUE_SIZE);
    --queueCount_;
  }
}

bool UwbFollowRest::dispatchLatestAngleIfNeeded() {
  if (queueEmpty()) {
    return false;
  }

  const unsigned long now = millis();
  if (now - lastLatestAngleDispatchMs_ < BU04_FOLLOW_LATEST_ANGLE_SEND_MS) {
    return false;
  }

  const FollowTarget& latestRaw =
      queueAt(static_cast<uint8_t>((queueTail_ + BU04_FOLLOW_QUEUE_SIZE - 1U) % BU04_FOLLOW_QUEUE_SIZE));
  const float inputX = BU04_FOLLOW_INVERT_X_AXIS ? -latestRaw.xcm : latestRaw.xcm;
  const float inputY = latestRaw.ycm;
  const float angleDeg = computeAngleDeg(static_cast<long>(inputX), static_cast<long>(inputY));
  if (angleDeg < BU04_FOLLOW_VALID_ANGLE_MIN_DEG || angleDeg > BU04_FOLLOW_VALID_ANGLE_MAX_DEG) {
    ++rejectedSamples_;
    lastLatestAngleDispatchMs_ = now;
    return false;
  }

  hasLatestAngle_ = true;
  latestAngleDeg_ = angleDeg;
  latestAngleRad_ = angleDeg * PI / 180.0f;
  lastLatestAngleDispatchMs_ = now;
  debug_.print("[REST FOLLOW] latest angle dispatch deg=");
  debug_.print(latestAngleDeg_, 3);
  debug_.print(" rad=");
  debug_.println(latestAngleRad_, 6);
  String actionId;
  if (rest_.startRotateTo(angleDeg, actionId)) {
    actionActive_ = true;
    currentActionId_ = actionId;
    hasCurrentTarget_ = false;
    lastDispatchMs_ = now;
    ++totalActions_;
    return true;
  }

  lastRestAttemptMs_ = now;
  return false;
}

void UwbFollowRest::dispatchNextIfNeeded() {
  if (queueEmpty()) {
    return;
  }

  if (!refreshPose()) {
    return;
  }

  if (actionActive_) {
    if (millis() - lastDispatchMs_ < BU04_FOLLOW_LIVE_REPLAN_MIN_MS) {
      return;
    }
    if (queueCount_ < BU04_FOLLOW_LIVE_REPLAN_MIN_POINTS) {
      return;
    }

    const FollowTarget& latestRaw =
        queueAt(static_cast<uint8_t>((queueTail_ + BU04_FOLLOW_QUEUE_SIZE - 1U) % BU04_FOLLOW_QUEUE_SIZE));
    const MapPoint latest = tagToMap(latestRaw.xcm, latestRaw.ycm, lastPose_);
    const float dx = latest.x - currentTarget_.x;
    const float dy = latest.y - currentTarget_.y;
    if (sqrtf(dx * dx + dy * dy) <= BU04_FOLLOW_REPLAN_TARGET_DELTA_M) {
      return;
    }

    rest_.cancelCurrentAction();
    actionActive_ = false;
    currentActionId_ = "";
  }

  if (queueCount_ < BU04_FOLLOW_BOOTSTRAP_POINTS) {
    return;
  }

  const unsigned long now = millis();
  const uint8_t dispatchIndex = static_cast<uint8_t>((queueCount_ > BU04_FOLLOW_LOOKAHEAD_POINTS)
      ? BU04_FOLLOW_LOOKAHEAD_POINTS
      : static_cast<uint8_t>(queueCount_ - 1U));

  const FollowTarget& rawTarget = queueAt(static_cast<uint8_t>((queueHead_ + dispatchIndex) % BU04_FOLLOW_QUEUE_SIZE));
  const MapPoint target = tagToMap(rawTarget.xcm, rawTarget.ycm, lastPose_);
  const bool forceSend = (lastForcedDispatchMs_ == 0U) ||
                         (now - lastForcedDispatchMs_ >= BU04_FOLLOW_FORCE_SEND_INTERVAL_MS);

  if (hasCurrentTarget_) {
    if (!forceSend && now - lastDispatchMs_ < BU04_FOLLOW_MIN_SEND_INTERVAL_MS) {
      return;
    }

    const float dx = target.x - currentTarget_.x;
    const float dy = target.y - currentTarget_.y;
    if (!forceSend && sqrtf(dx * dx + dy * dy) < BU04_FOLLOW_MIN_SEND_DISTANCE_M) {
      return;
    }
  }

  popQueue(static_cast<uint8_t>(dispatchIndex));

  String actionId;
  if (forceSend) {
    lastForcedDispatchMs_ = now;
  }
  if (rest_.startMoveTo(target.x, target.y, actionId)) {
    actionActive_ = true;
    currentActionId_ = actionId;
    currentTarget_ = target;
    hasCurrentTarget_ = true;
    lastDispatchMs_ = millis();
    ++totalActions_;
  } else {
    lastRestAttemptMs_ = millis();
  }
}

void UwbFollowRest::checkCurrentAction() {
  if (!actionActive_) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastActionCheckMs_ < BU04_FOLLOW_ACTION_CHECK_MS) {
    return;
  }
  lastActionCheckMs_ = now;

  bool running = false;
  if (!rest_.getActionRunning(currentActionId_, running)) {
    lastRestAttemptMs_ = now;
    return;
  }

  if (!running) {
    popQueue(1);
    actionActive_ = false;
    currentActionId_ = "";
  }
}

void UwbFollowRest::logStatusIfDue() {
  const unsigned long now = millis();
  if (now - lastStatusLogMs_ < BU04_FOLLOW_STATUS_LOG_MS) {
    return;
  }
  lastStatusLogMs_ = now;
  debug_.print("[REST FOLLOW] frames=");
  debug_.print(totalFrames_);
  debug_.print(" points=");
  debug_.print(totalPoints_);
  debug_.print(" actions=");
  debug_.print(totalActions_);
  debug_.print(" queue=");
  debug_.println(queueCount_);
  debug_.print(" rejected=");
  debug_.println(rejectedSamples_);
  debug_.print(" latest_angle_valid=");
  debug_.print(hasLatestAngle_ ? "1" : "0");
  debug_.print(" latest_angle_deg=");
  debug_.print(latestAngleDeg_, 3);
  debug_.print(" latest_angle_rad=");
  debug_.println(latestAngleRad_, 6);
}

}  // namespace follow_demo
