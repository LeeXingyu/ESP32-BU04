#include "Bu04TestChain.h"

#include <ctype.h>
#include <math.h>

namespace test_chain {

Bu04TestChain::Bu04TestChain(Stream& debug)
    : debug_(debug),
      motion_(debug),
      expectedLength_(0),
      state_(State::kWaitJ),
      lastWifiAttempt_(0),
      lastPoseAttemptMs_(0),
      lastPeriodicSendMs_(0),
      lastStraightSendMs_(0),
      lastRotateSendMs_(0),
      lastActionCheckMs_(0),
      lastStatusLogMs_(0),
      lastRxRateLogMs_(0),
      lastFrameSeenMs_(0),
      lastRestAttemptMs_(0),
      lastLegFetchMs_(0),
      wifiReady_(false),
      hasPose_(false),
      actionActive_(false),
      hasCurrentTarget_(false),
      totalFrames_(0),
      rxFramesSinceLog_(0),
      totalSends_(0),
      parseFails_(0),
      rejectedFrames_(0),
      straightAccepted_(0),
      rotateAccepted_(0) {
  lenBuffer_.reserve(8);
  payloadBuffer_.reserve(256);
  currentActionId_.reserve(64);
}

void Bu04TestChain::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(test_chain::kWifiSsid, test_chain::kWifiPassword);
  debug_.println("Test chain controller starting...");
#if BU04_TEST_CHAIN_MODE == BU04_TEST_CHAIN_MODE_LEG_ONLY
  debug_.println("Test chain leg-only mode active");
#endif
}

void Bu04TestChain::update(Bu04Uart& dataUart) {
#if BU04_TEST_CHAIN_MODE == BU04_TEST_CHAIN_MODE_LEG_ONLY
  (void)dataUart;
  runLegOnlyTest();
#else
#ifdef BU04_TEST_CHAIN_ENABLE_BU04_RX_RATE_LOG
  char c;
  while (dataUart.readByte(c)) {
    handleIncomingByte(c);
  }
  logRxRateIfDue();
#else
  ensureWifi();

  char c;
  while (dataUart.readByte(c)) {
    handleIncomingByte(c);
  }

  tickPeriodicMove();
  checkCurrentAction();
  logStatusIfDue();
  logRxRateIfDue();

  if (lastFrameSeenMs_ != 0U && millis() - lastFrameSeenMs_ > BU04_TEST_CHAIN_FRAME_IDLE_LOG_MS) {
    debug_.println("[TEST] frame idle");
    lastFrameSeenMs_ = millis();
  }
#endif
#endif
}

void Bu04TestChain::runLegOnlyTest() {
  ensureWifi();

  const unsigned long now = millis();
  if (now - lastLegFetchMs_ < BU04_TEST_CHAIN_LEG_FETCH_MS) {
    return;
  }
  lastLegFetchMs_ = now;

  const String ip = test_chain::robotIp().toString();
  debug_.print("[TEST] leg fetch start ms=");
  debug_.println(now);
  const bool ok = leg_follow::p15FetchAndPrintTimed(ip,
                                                    leg_follow::P15OutputFormat::kText,
                                                    debug_,
                                                    test_chain::kRobotPort);
  debug_.print("[TEST] leg fetch end ok=");
  debug_.println(ok ? 1 : 0);
}

void Bu04TestChain::ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiReady_) {
      debug_.print("WiFi connected, IP: ");
      debug_.println(WiFi.localIP());
      wifiReady_ = true;
    }
    return;
  }

  wifiReady_ = false;
  if (millis() - lastWifiAttempt_ < BU04_TEST_CHAIN_WIFI_RECONNECT_MS) {
    return;
  }
  lastWifiAttempt_ = millis();

  debug_.println("Reconnecting WiFi for test chain...");
  WiFi.disconnect();
  WiFi.begin(test_chain::kWifiSsid, test_chain::kWifiPassword);
}

bool Bu04TestChain::refreshPose() {
  if (millis() - lastPoseAttemptMs_ < BU04_TEST_CHAIN_REST_RETRY_MS) {
    return hasPose_;
  }

  lastPoseAttemptMs_ = millis();
  RobotPose pose;
  if (motion_.getPose(pose)) {
    lastPose_ = pose;
    hasPose_ = true;
    return true;
  }
  return hasPose_;
}

void Bu04TestChain::handleIncomingByte(char c) {
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
          ++rxFramesSinceLog_;
          lastFrameSeenMs_ = millis();
#ifdef BU04_TEST_CHAIN_ENABLE_BU04_RX_RATE_LOG
          (void)frame;
#else
#if BU04_TEST_CHAIN_MODE == BU04_TEST_CHAIN_MODE_STRAIGHT_ONLY
          handleStraightFrame(frame);
#elif BU04_TEST_CHAIN_MODE == BU04_TEST_CHAIN_MODE_ROTATE_ONLY
          handleRotateFrame(frame);
#else
          (void)frame;
#endif
#endif
        } else {
          ++parseFails_;
        }
        resetFrame();
      }
      break;
  }
}

void Bu04TestChain::resetFrame() {
  state_ = State::kWaitJ;
  lenBuffer_ = "";
  payloadBuffer_ = "";
  expectedLength_ = 0;
}

bool Bu04TestChain::parseStringField(const String& text, const char* key, String& value) {
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

bool Bu04TestChain::parseLongField(const String& text, const char* key, long& value) {
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

bool Bu04TestChain::parseFrame(const String& frame, UwbFrame& out) {
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

float Bu04TestChain::computeAngleDeg(long xcm, long ycm) {
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

bool Bu04TestChain::angleInRange(float angleDeg, float minDeg, float maxDeg) {
  return angleDeg >= minDeg && angleDeg <= maxDeg;
}

float Bu04TestChain::clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

float Bu04TestChain::distanceM(const UwbFrame& frame) {
  return static_cast<float>(frame.d) * 0.01f;
}

float Bu04TestChain::forwardDistanceM(const UwbFrame& frame) {
  return clampFloat(distanceM(frame),
                    BU04_TEST_CHAIN_STRAIGHT_FORWARD_MIN_M,
                    BU04_TEST_CHAIN_STRAIGHT_FORWARD_MAX_M);
}

float Bu04TestChain::mapHeadingX(float distanceM, const RobotPose& pose) {
  return pose.x + distanceM * cosf(pose.yaw);
}

float Bu04TestChain::mapHeadingY(float distanceM, const RobotPose& pose) {
  return pose.y + distanceM * sinf(pose.yaw);
}

void Bu04TestChain::tickPeriodicMove() {
#if BU04_TEST_CHAIN_MODE == BU04_TEST_CHAIN_MODE_PERIODIC_MOVE
  const unsigned long now = millis();
  if (now - lastPeriodicSendMs_ < BU04_TEST_CHAIN_PERIODIC_SEND_MS) {
    return;
  }
  if (!refreshPose()) {
    return;
  }

  const float targetX = mapHeadingX(BU04_TEST_CHAIN_PERIODIC_STEP_M, lastPose_);
  const float targetY = mapHeadingY(BU04_TEST_CHAIN_PERIODIC_STEP_M, lastPose_);
  String actionId;
  if (motion_.startMoveTo(targetX, targetY, actionId)) {
    currentActionId_ = actionId;
    actionActive_ = true;
    hasCurrentTarget_ = true;
    lastPeriodicSendMs_ = now;
    ++totalSends_;
  } else {
    lastRestAttemptMs_ = now;
  }
#endif
}

void Bu04TestChain::handleStraightFrame(const UwbFrame& frame) {
  const unsigned long now = millis();
  if (now - lastStraightSendMs_ < BU04_TEST_CHAIN_STRAIGHT_SEND_INTERVAL_MS) {
    return;
  }

#if BU04_TEST_CHAIN_STRAIGHT_USE_ANGLE_FILTER
  const float angleDeg = computeAngleDeg(frame.xcm, frame.ycm);
  if (!angleInRange(angleDeg,
                    BU04_TEST_CHAIN_STRAIGHT_MIN_ANGLE_DEG,
                    BU04_TEST_CHAIN_STRAIGHT_MAX_ANGLE_DEG)) {
    ++rejectedFrames_;
    return;
  }
#endif

  if (!refreshPose()) {
    return;
  }

  // Any frame can participate here. For the straight test, keep X unchanged and only
  // advance Y because Y is the chassis forward axis in the current coordinate system.
  const float forwardM = forwardDistanceM(frame);
  const float targetX = lastPose_.x;
  const float targetY = lastPose_.y + forwardM;
  String actionId;
  if (motion_.startMoveTo(targetX, targetY, actionId)) {
    currentActionId_ = actionId;
    actionActive_ = true;
    hasCurrentTarget_ = true;
    lastStraightSendMs_ = now;
    ++straightAccepted_;
    ++totalSends_;
  } else {
    lastRestAttemptMs_ = now;
  }
}

void Bu04TestChain::handleRotateFrame(const UwbFrame& frame) {
  const unsigned long now = millis();
  if (now - lastRotateSendMs_ < BU04_TEST_CHAIN_ANGLE_SAMPLE_MS) {
    return;
  }
  if (actionActive_) {
    return;
  }

  const float angleDeg = computeAngleDeg(frame.xcm, frame.ycm);
  if (!angleInRange(angleDeg,
                    BU04_TEST_CHAIN_ROTATE_MIN_ANGLE_DEG,
                    BU04_TEST_CHAIN_ROTATE_MAX_ANGLE_DEG)) {
    ++rejectedFrames_;
    return;
  }

  String actionId;
  if (motion_.startRotateTo(angleDeg, actionId)) {
    currentActionId_ = actionId;
    actionActive_ = true;
    hasCurrentTarget_ = false;
    lastRotateSendMs_ = now;
    ++rotateAccepted_;
    ++totalSends_;
    debug_.print("[ANGLE] sample angleDeg=");
    debug_.print(angleDeg, 3);
    debug_.print(" angleRad=");
    debug_.println(angleDeg * PI / 180.0f, 6);
  } else {
    lastRestAttemptMs_ = now;
  }
}

void Bu04TestChain::checkCurrentAction() {
  if (!actionActive_) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastActionCheckMs_ < BU04_TEST_CHAIN_STRAIGHT_SEND_INTERVAL_MS) {
    return;
  }
  lastActionCheckMs_ = now;

  bool running = false;
  if (!motion_.getActionRunning(currentActionId_, running)) {
    lastRestAttemptMs_ = now;
    return;
  }

  if (!running) {
    actionActive_ = false;
    currentActionId_ = "";
    hasCurrentTarget_ = false;
  }
}

void Bu04TestChain::logStatusIfDue() {
  const unsigned long now = millis();
  if (now - lastStatusLogMs_ < BU04_TEST_CHAIN_STATUS_LOG_MS) {
    return;
  }
  lastStatusLogMs_ = now;
  debug_.print("[TEST] frames=");
  debug_.print(totalFrames_);
  debug_.print(" sends=");
  debug_.print(totalSends_);
  debug_.print(" rejected=");
  debug_.print(rejectedFrames_);
  debug_.print(" straight=");
  debug_.print(straightAccepted_);
  debug_.print(" rotate=");
  debug_.println(rotateAccepted_);
}

void Bu04TestChain::logRxRateIfDue() {
#ifdef BU04_TEST_CHAIN_ENABLE_BU04_RX_RATE_LOG
  const unsigned long now = millis();
  if (now - lastRxRateLogMs_ < BU04_TEST_CHAIN_RX_RATE_LOG_MS) {
    return;
  }
  lastRxRateLogMs_ = now;
  debug_.print("[TEST] bu04_rx_frames_1s=");
  debug_.println(rxFramesSinceLog_);
  rxFramesSinceLog_ = 0U;
#else
  (void)rxFramesSinceLog_;
#endif
}

}  // namespace test_chain
