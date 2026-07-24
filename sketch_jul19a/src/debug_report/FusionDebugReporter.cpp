#include "FusionDebugReporter.h"

#include <HTTPClient.h>
#include <ctype.h>
#include <math.h>

#include "../leg/LegFollowDetector.h"
#include "../net/NetConfig.h"

namespace fusion_debug {
namespace {

constexpr char kRobotIp[] = "192.168.0.56";
constexpr uint16_t kRobotPort = 1448;
constexpr uint16_t kSampleIntervalMs = 66;
constexpr uint16_t kLegSampleIntervalMs = 500;
constexpr uint16_t kWindowMs = 1000;
constexpr uint8_t kMaxSamples = 15;

float normalizeAngleDeg(float deg) {
  while (deg <= -180.0f) {
    deg += 360.0f;
  }
  while (deg > 180.0f) {
    deg -= 360.0f;
  }
  return deg;
}

}  // namespace

FusionDebugReporter::FusionDebugReporter(Stream& debug)
    : debug_(debug),
      hasLatestFrame_(false),
      wifiReady_(false),
      serverReady_(false),
      hasLegSample_(false),
      hasLegTarget_(false),
      legReadyNotified_(false),
      lastLegReport_{},
      expectedLength_(0),
      state_(State::kWaitJ),
      lastWifiAttempt_(0),
      lastServerAttempt_(0),
      lastLegSampleMs_(0),
      lastSampleMs_(0),
      lastWindowSendMs_(0),
      sampleCount_(0),
      totalFrames_(0),
      totalSamples_(0),
      totalSent_(0),
      parseFails_(0),
      lastStatusLogMs_(0) {
  lenBuffer_.reserve(8);
  payloadBuffer_.reserve(256);
  resetWindow();
}

void FusionDebugReporter::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(net_demo::kWifiSsid, net_demo::kWifiPassword);
  debug_.println("fusion_debug reporter starting...");
}

void FusionDebugReporter::update(Bu04Uart& dataUart) {
  ensureWifi();
  ensureServer();

  char c;
  while (dataUart.readByte(c)) {
    handleIncomingByte(c);
  }

  const unsigned long now = millis();
  if (hasLatestFrame_ && now - lastSampleMs_ >= kSampleIntervalMs) {
    lastSampleMs_ = now;
    captureSample();
  }

  if (now - lastLegSampleMs_ >= kLegSampleIntervalMs) {
    lastLegSampleMs_ = now;
    refreshLeg();
  }

  sendWindowIfDue();

  if (now - lastStatusLogMs_ >= net_demo::kStatusLogMs) {
    lastStatusLogMs_ = now;
    debug_.print("[fusion_debug] frames=");
    debug_.print(totalFrames_);
    debug_.print(" samples=");
    debug_.print(totalSamples_);
    debug_.print(" sent=");
    debug_.print(totalSent_);
    debug_.print(" leg=");
    debug_.println(hasLegTarget_ ? "1" : "0");
  }
}

void FusionDebugReporter::ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiReady_) {
      debug_.print("WiFi connected, IP: ");
      debug_.println(WiFi.localIP());
      wifiReady_ = true;
    }
    return;
  }

  wifiReady_ = false;
  if (millis() - lastWifiAttempt_ < net_demo::kWifiReconnectMs) {
    return;
  }
  lastWifiAttempt_ = millis();

  debug_.println("Reconnecting WiFi for fusion_debug...");
  WiFi.disconnect();
  WiFi.begin(net_demo::kWifiSsid, net_demo::kWifiPassword);
}

bool FusionDebugReporter::ensureServer() {
  if (WiFi.status() != WL_CONNECTED) {
    if (client_.connected()) {
      client_.stop();
    }
    serverReady_ = false;
    return false;
  }

  if (client_.connected()) {
    serverReady_ = true;
    return true;
  }

  serverReady_ = false;
  if (millis() - lastServerAttempt_ < net_demo::kServerReconnectMs) {
    return false;
  }

  lastServerAttempt_ = millis();
  client_.stop();
  return client_.connect(net_demo::serverIp(), net_demo::kServerPort);
}

void FusionDebugReporter::handleIncomingByte(char c) {
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
          hasLatestFrame_ = true;
          latestFrame_ = frame;
        } else {
          ++parseFails_;
        }
        resetFrame();
      }
      break;
  }
}

void FusionDebugReporter::resetFrame() {
  state_ = State::kWaitJ;
  lenBuffer_ = "";
  payloadBuffer_ = "";
  expectedLength_ = 0;
}

bool FusionDebugReporter::parseStringField(const String& text, const char* key, String& value) {
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

bool FusionDebugReporter::parseLongField(const String& text, const char* key, long& value) {
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

bool FusionDebugReporter::parseFrame(const String& frame, UwbFrame& out) {
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

float FusionDebugReporter::computeAngleDeg(long xcm, long ycm) {
  return normalizeAngleDeg(atan2f(static_cast<float>(xcm), static_cast<float>(ycm)) * 180.0f / PI);
}

void FusionDebugReporter::refreshLeg() {
  hasLegSample_ = true;

  leg_follow::FollowTargetReport report;
  if (leg_follow::fetchFollowTarget(String(kRobotIp), report, kRobotPort)) {
    lastLegReport_ = report;
    hasLegTarget_ = report.valid && report.has_target;
    if (hasLegTarget_ && !legReadyNotified_) {
      legReadyNotified_ = true;
      debug_.print("[fusion_debug] leg ready angle=");
      debug_.println(report.target.theta_deg, 1);
    }
  } else {
    hasLegTarget_ = false;
    legReadyNotified_ = false;
  }
}

void FusionDebugReporter::captureSample() {
  if (sampleCount_ >= kMaxSamples) {
    return;
  }

  Sample sample;
  sample.uwb_angle_deg = computeAngleDeg(latestFrame_.xcm, latestFrame_.ycm);
  if (hasLegTarget_) {
    sample.has_leg = true;
    sample.leg_angle_deg = lastLegReport_.target.theta_deg;
  } else {
    sample.has_leg = false;
    sample.leg_angle_deg = 0.0f;
  }

  samples_[sampleCount_] = sample;
  ++sampleCount_;
  ++totalSamples_;
}

String FusionDebugReporter::buildPayload() const {
  String body;
  body.reserve(160);
  body += "{\"type\":\"fusion_debug\",";
  body += "\"samples\":[";
  for (uint8_t i = 0; i < sampleCount_; ++i) {
    if (i > 0U) {
      body += ",";
    }
    body += "[";
    body += String(samples_[i].uwb_angle_deg, 1);
    body += ",";
    if (samples_[i].has_leg) {
      body += String(samples_[i].leg_angle_deg, 1);
    } else {
      body += "null";
    }
    body += "]";
  }
  body += "]}";
  return body;
}

void FusionDebugReporter::sendWindowIfDue() {
  const unsigned long now = millis();
  if (now - lastWindowSendMs_ < kWindowMs) {
    return;
  }

  lastWindowSendMs_ = now;
  if (sampleCount_ == 0U) {
    resetWindow();
    return;
  }

  if (!ensureServer()) {
    debug_.print("[fusion_debug] tcp connect fail ");
    debug_.print(net_demo::serverIp());
    debug_.print(":");
    debug_.println(net_demo::kServerPort);
    resetWindow();
    return;
  }

  const String payload = buildPayload();
  const size_t sent = client_.println(payload);
  if (sent > 0U) {
    ++totalSent_;
    debug_.print("[fusion_debug] tcp sent samples=");
    debug_.println(sampleCount_);
  } else {
    debug_.println("[fusion_debug] tcp send fail");
  }

  resetWindow();
}

void FusionDebugReporter::resetWindow() {
  sampleCount_ = 0;
  for (uint8_t i = 0; i < kMaxSamples; ++i) {
    samples_[i] = Sample{};
  }
}

}  // namespace fusion_debug
