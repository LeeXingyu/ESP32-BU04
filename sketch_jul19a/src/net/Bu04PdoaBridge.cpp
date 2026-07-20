#include "Bu04PdoaBridge.h"

#include <ctype.h>

#include "NetConfig.h"

Bu04PdoaBridge::Bu04PdoaBridge(Stream& debug)
    : debug_(debug),
      expectedLength_(0),
      state_(State::kWaitJ),
      lastWifiAttempt_(0),
      lastServerAttempt_(0),
      lastStatusLogMs_(0),
      lastByteMs_(0),
      lastFrameMs_(0),
      lastWifiState_(false),
      lastServerState_(false),
      totalBytes_(0),
      totalFrames_(0),
      totalGroups_(0),
      totalSent_(0),
      parseFails_(0) {
  lenBuffer_.reserve(8);
  payloadBuffer_.reserve(256);
  sampleCount_ = 0;
  queueHead_ = 0;
  queueTail_ = 0;
  queueCount_ = 0;
  lastSendMs_ = 0;
}

void Bu04PdoaBridge::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(net_demo::kWifiSsid, net_demo::kWifiPassword);
  debug_.println("PDOA WiFi bridge starting...");
}

void Bu04PdoaBridge::update(Bu04Uart& dataUart) {
  ensureWifi();
  ensureServer();

  char c;
  while (dataUart.readByte(c)) {
    ++totalBytes_;
    lastByteMs_ = millis();
    handleIncomingByte(c);
  }

  sendQueuedIfDue();
  logStatusIfDue();
}

void Bu04PdoaBridge::ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!lastWifiState_) {
      debug_.print("WiFi connected, IP: ");
      debug_.println(WiFi.localIP());
      lastWifiState_ = true;
    }
    return;
  }

  lastWifiState_ = false;
  if (millis() - lastWifiAttempt_ < net_demo::kWifiReconnectMs) {
    return;
  }
  lastWifiAttempt_ = millis();

  debug_.println("Reconnecting WiFi...");
  WiFi.disconnect();
  WiFi.begin(net_demo::kWifiSsid, net_demo::kWifiPassword);
}

void Bu04PdoaBridge::ensureServer() {
  if (WiFi.status() != WL_CONNECTED) {
    if (client_.connected()) {
      client_.stop();
    }
    lastServerState_ = false;
    return;
  }

  if (client_.connected()) {
    if (!lastServerState_) {
      debug_.print("Server connected: ");
      debug_.print(net_demo::serverIp());
      debug_.print(":");
      debug_.println(net_demo::kServerPort);
      lastServerState_ = true;
    }
    return;
  }

  lastServerState_ = false;
  if (millis() - lastServerAttempt_ < net_demo::kServerReconnectMs) {
    return;
  }
  lastServerAttempt_ = millis();

  debug_.print("Connecting server ");
  debug_.print(net_demo::serverIp());
  debug_.print(":");
  debug_.println(net_demo::kServerPort);

  client_.stop();
  client_.connect(net_demo::serverIp(), net_demo::kServerPort);
}

void Bu04PdoaBridge::handleIncomingByte(char c) {
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
          if (!parseHexLength(lenBuffer_, expectedLength_)) {
            resetFrame();
            break;
          }
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
        parseFrame(payloadBuffer_);
        resetFrame();
      }
      break;
  }
}

bool Bu04PdoaBridge::parseHexLength(const String& text, uint16_t& value) {
  if (text.length() != 4) {
    return false;
  }

  char* endPtr = nullptr;
  const unsigned long parsed = strtoul(text.c_str(), &endPtr, 16);
  if (endPtr == text.c_str() || *endPtr != '\0' || parsed > 0xFFFFUL) {
    return false;
  }

  value = static_cast<uint16_t>(parsed);
  return true;
}

bool Bu04PdoaBridge::parseStringField(const String& text, const char* key, String& value) {
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
  while (valueStart < static_cast<int>(text.length()) && isspace(static_cast<unsigned char>(text[valueStart]))) {
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

bool Bu04PdoaBridge::parseLongField(const String& text, const char* key, long& value) {
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
  while (valueStart < static_cast<int>(text.length()) && isspace(static_cast<unsigned char>(text[valueStart]))) {
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

bool Bu04PdoaBridge::parseFrame(const String& frame) {
  if (frame.indexOf("\"TWR\"") < 0) {
    ++parseFails_;
    return false;
  }

  Bu04PdoaData data;
  bool ok = true;
  ok &= parseStringField(frame, "a16", data.a16);
  ok &= parseLongField(frame, "R", data.r);
  ok &= parseLongField(frame, "T", data.t);
  ok &= parseLongField(frame, "D", data.d);
  ok &= parseLongField(frame, "P", data.p);
  ok &= parseLongField(frame, "Xcm", data.xcm);
  ok &= parseLongField(frame, "Ycm", data.ycm);
  ok &= parseLongField(frame, "O", data.o);
  ok &= parseLongField(frame, "V", data.v);
  ok &= parseLongField(frame, "X", data.x);
  ok &= parseLongField(frame, "Y", data.y);
  ok &= parseLongField(frame, "Z", data.z);

  if (ok) {
    ++totalFrames_;
    lastFrameMs_ = millis();
    appendSample(data);
  } else {
    ++parseFails_;
  }

  return ok;
}

float Bu04PdoaBridge::computeAngleDeg(long xcm, long ycm) {
  if (ycm == 0) {
    if (xcm > 0) {
      return 90.0f;
    }
    if (xcm < 0) {
      return -90.0f;
    }
    return 0.0f;
  }

  const float ratio = static_cast<float>(xcm) / static_cast<float>(ycm);
  return atan(ratio) * 180.0f / PI;
}

void Bu04PdoaBridge::appendSample(const Bu04PdoaData& data) {
  if (sampleCount_ < 16) {
    samples_[sampleCount_] = data;
    ++sampleCount_;
  }

  if (sampleCount_ >= 16) {
    flushAveragedGroup();
    sampleCount_ = 0;
  }
}

void Bu04PdoaBridge::flushAveragedGroup() {
  if (sampleCount_ < 16) {
    return;
  }

  // Middle 10 frames are averaged. The last frame keeps T and non-averaged fields.
  long sumXcm = 0;
  long sumYcm = 0;
  long sumD = 0;
  long sumP = 0;
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  for (uint8_t i = 3; i < 13; ++i) {
    sumD += samples_[i].d;
    sumP += samples_[i].p;
    sumXcm += samples_[i].xcm;
    sumYcm += samples_[i].ycm;
    sumX += samples_[i].x;
    sumY += samples_[i].y;
    sumZ += samples_[i].z;
  }

  Bu04PdoaData avg = samples_[15];
  avg.d = sumD / 10;
  avg.p = sumP / 10;
  avg.xcm = sumXcm / 10;
  avg.ycm = sumYcm / 10;
  avg.x = sumX / 10;
  avg.y = sumY / 10;
  avg.z = sumZ / 10;

  ++totalGroups_;
  enqueueAveragedGroup(avg);
}

void Bu04PdoaBridge::enqueueAveragedGroup(const Bu04PdoaData& data) {
  Bu04PdoaSample sample;
  sample.data = data;
  sample.angleDeg = computeAngleDeg(data.xcm, data.ycm);

  if (queueCount_ >= 3) {
    queueHead_ = static_cast<uint8_t>((queueHead_ + 1U) % 3U);
    --queueCount_;
  }

  queue_[queueTail_] = sample;
  queueTail_ = static_cast<uint8_t>((queueTail_ + 1U) % 3U);
  ++queueCount_;
}

void Bu04PdoaBridge::sendQueuedIfDue() {
  if (queueCount_ == 0) {
    return;
  }

  if (!client_.connected()) {
    return;
  }

  const unsigned long now = millis();
  if (lastSendMs_ != 0 && (now - lastSendMs_ < 1000UL)) {
    return;
  }

  const Bu04PdoaSample sample = queue_[queueHead_];
  queueHead_ = static_cast<uint8_t>((queueHead_ + 1U) % 3U);
  --queueCount_;
  lastSendMs_ = now;

  sendJsonToServer(sample);
  ++totalSent_;
}

void Bu04PdoaBridge::sendJsonToServer(const Bu04PdoaSample& sample) {
  if (!client_.connected()) {
    return;
  }

  String out;
  out.reserve(128);
  out += "{\"type\":\"TWR\",";
  out += "\"a16\":\"";
  out += sample.data.a16;
  out += "\",\"T\":";
  out += String(sample.data.t);
  out += ",\"Xcm\":";
  out += String(sample.data.xcm);
  out += ",\"Ycm\":";
  out += String(sample.data.ycm);
  out += ",\"D\":";
  out += String(sample.data.d);
  out += ",\"angle\":";
  out += String(sample.angleDeg, 2);
  out += "}";

  client_.println(out);
}

void Bu04PdoaBridge::logStatusIfDue() {
  const unsigned long now = millis();
  if (now - lastStatusLogMs_ < net_demo::kStatusLogMs) {
    return;
  }
  lastStatusLogMs_ = now;

  debug_.print("[TCP] bytes=");
  debug_.print(totalBytes_);
  debug_.print(" frames=");
  debug_.print(totalFrames_);
  debug_.print(" groups=");
  debug_.print(totalGroups_);
  debug_.print(" sent=");
  debug_.print(totalSent_);
  debug_.print(" queue=");
  debug_.print(queueCount_);
  debug_.print(" parse_fail=");
  debug_.print(parseFails_);

  if (lastByteMs_ == 0) {
    debug_.println(" data=waiting");
    return;
  }

  const unsigned long idleMs = now - lastByteMs_;
  debug_.print(" data_idle_ms=");
  debug_.println(idleMs);
}

void Bu04PdoaBridge::resetFrame() {
  state_ = State::kWaitJ;
  lenBuffer_ = "";
  payloadBuffer_ = "";
  expectedLength_ = 0;
}
