#include "UwbFollowRest.h"

#include <ctype.h>
#include <math.h>

namespace follow_demo {

namespace {

float degAbs(float value) {
  return value < 0.0f ? -value : value;
}

}  // namespace

UwbFollowRest::UwbFollowRest(Stream& debug)
    : debug_(debug),
      rest_(debug),
      expectedLength_(0),
      state_(State::kWaitJ),
      lastWifiAttempt_(0),
      lastSampleMs_(0),
      lastMotionSampleMs_(0),
      lastPoseMs_(0),
      lastActionCheckMs_(0),
      lastStatusLogMs_(0),
      lastDispatchMs_(0),
      lastRestAttemptMs_(0),
      lastGoodFrameMs_(0),
      wifiReady_(false),
      angleBlocked_(false),
      hasFiltered_(false),
      hasVelocity_(false),
      filteredXcm_(0.0f),
      filteredYcm_(0.0f),
      velocityXcmPerSec_(0.0f),
      velocityYcmPerSec_(0.0f),
      hasPose_(false),
      hasFrame_(false),
      queueHead_(0),
      queueTail_(0),
      queueCount_(0),
      actionActive_(false),
      hasCurrentTarget_(false),
      totalFrames_(0),
      totalPoints_(0),
      totalActions_(0),
      parseFails_(0) {
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
  dispatchNextIfNeeded();
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
          const unsigned long now = millis();
          if (now - lastSampleMs_ >= BU04_FOLLOW_SAMPLE_INTERVAL_MS) {
            lastSampleMs_ = now;
            processSample(frame);
          }
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
  const unsigned long prevMotionSampleMs = lastMotionSampleMs_;
  float rawX = static_cast<float>(frame.xcm);
  float rawY = static_cast<float>(frame.ycm);

  if (!hasFiltered_) {
    filteredXcm_ = rawX;
    filteredYcm_ = rawY;
    hasFiltered_ = true;
    hasVelocity_ = false;
  } else {
    const float prevX = filteredXcm_;
    const float prevY = filteredYcm_;
    const float boundedRawX = clampJump(prevX, rawX, BU04_FOLLOW_CLAMP_MAX_JUMP_CM);
    const float boundedRawY = clampJump(prevY, rawY, BU04_FOLLOW_CLAMP_MAX_JUMP_CM);
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

  const float angleDeg = computeAngleDeg(static_cast<long>(filteredXcm_), static_cast<long>(filteredYcm_));
  if (BU04_FOLLOW_MAX_TRACK_ANGLE_DEG > 0.0f) {
    if (!angleBlocked_ && degAbs(angleDeg) > BU04_FOLLOW_MAX_TRACK_ANGLE_DEG) {
      angleBlocked_ = true;
    } else if (angleBlocked_ && degAbs(angleDeg) < BU04_FOLLOW_ANGLE_RECOVER_GAP_DEG) {
      angleBlocked_ = false;
    }
  }
  if (angleBlocked_) {
    return;
  }

  const float distM = distanceM(frame);
  if (distM < BU04_FOLLOW_SAFETY_STOP_DISTANCE_M) {
    if (actionActive_) {
      rest_.cancelCurrentAction();
      actionActive_ = false;
      currentActionId_ = "";
    }
    return;
  }
  if (distM < BU04_FOLLOW_TOO_CLOSE_DISTANCE_M) {
    return;
  }

  if (!hasPose_ || now - lastPoseMs_ >= BU04_FOLLOW_POSE_REFRESH_MS) {
    if (lastRestAttemptMs_ != 0 && now - lastRestAttemptMs_ < BU04_FOLLOW_REST_RETRY_MS) {
      return;
    }
    hasPose_ = rest_.getPose(lastPose_);
    lastPoseMs_ = now;
    lastRestAttemptMs_ = now;
    if (!hasPose_) {
      return;
    }
  }

  const float lookaheadSec = chooseLookaheadSec(distM);
  float predictedXcm = filteredXcm_;
  float predictedYcm = filteredYcm_;
  if (hasVelocity_) {
    predictedXcm += velocityXcmPerSec_ * lookaheadSec;
    predictedYcm += velocityYcmPerSec_ * lookaheadSec;
  }

  const MapPoint point = tagToMap(predictedXcm, predictedYcm, lastPose_);
  pushPoint(point);
  lastMotionSampleMs_ = now;
}

void UwbFollowRest::pushPoint(const MapPoint& point) {
  if (BU04_FOLLOW_ENABLE_POINT_SPACING_GUARD && !queueEmpty()) {
    const MapPoint& newest = queueAt(static_cast<uint8_t>((queueTail_ + BU04_FOLLOW_QUEUE_SIZE - 1U) % BU04_FOLLOW_QUEUE_SIZE));
    const float dx = point.x - newest.x;
    const float dy = point.y - newest.y;
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

const MapPoint& UwbFollowRest::queueAt(uint8_t index) const {
  return queue_[index % BU04_FOLLOW_QUEUE_SIZE];
}

void UwbFollowRest::popQueue(uint8_t count) {
  while (count-- > 0 && queueCount_ > 0) {
    queueHead_ = static_cast<uint8_t>((queueHead_ + 1U) % BU04_FOLLOW_QUEUE_SIZE);
    --queueCount_;
  }
}

void UwbFollowRest::dispatchNextIfNeeded() {
  if (queueEmpty()) {
    return;
  }

  if (actionActive_) {
    if (millis() - lastDispatchMs_ < BU04_FOLLOW_LIVE_REPLAN_MIN_MS) {
      return;
    }
    if (queueCount_ < BU04_FOLLOW_LIVE_REPLAN_MIN_POINTS) {
      return;
    }

    const MapPoint& latest = queueAt(static_cast<uint8_t>((queueTail_ + BU04_FOLLOW_QUEUE_SIZE - 1U) % BU04_FOLLOW_QUEUE_SIZE));
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

  const uint8_t dispatchIndex = static_cast<uint8_t>((queueCount_ > BU04_FOLLOW_LOOKAHEAD_POINTS)
      ? BU04_FOLLOW_LOOKAHEAD_POINTS
      : static_cast<uint8_t>(queueCount_ - 1U));
  const MapPoint target = queueAt(static_cast<uint8_t>((queueHead_ + dispatchIndex) % BU04_FOLLOW_QUEUE_SIZE));

  if (hasCurrentTarget_) {
    const unsigned long now = millis();
    if (now - lastDispatchMs_ < BU04_FOLLOW_MIN_SEND_INTERVAL_MS) {
      return;
    }

    const float dx = target.x - currentTarget_.x;
    const float dy = target.y - currentTarget_.y;
    if (sqrtf(dx * dx + dy * dy) < BU04_FOLLOW_MIN_SEND_DISTANCE_M) {
      return;
    }
  }

  popQueue(static_cast<uint8_t>(dispatchIndex));

  String actionId;
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
}

}  // namespace follow_demo
