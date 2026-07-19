#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "../bu04/Bu04Uart.h"

struct Bu04PdoaData {
  String a16;
  long r = 0;
  long t = 0;
  long d = 0;
  long p = 0;
  long xcm = 0;
  long ycm = 0;
  long o = 0;
  long v = 0;
  long x = 0;
  long y = 0;
  long z = 0;
};

struct Bu04PdoaSample {
  Bu04PdoaData data;
  float angleDeg = 0.0f;
};

class Bu04PdoaBridge {
 public:
  explicit Bu04PdoaBridge(Stream& debug);

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
  void ensureServer();
  void handleIncomingByte(char c);
  void resetFrame();
  static float computeAngleDeg(long xcm, long ycm);

  static bool parseHexLength(const String& text, uint16_t& value);
  static bool parseStringField(const String& text, const char* key, String& value);
  static bool parseLongField(const String& text, const char* key, long& value);
  bool parseFrame(const String& frame);
  void appendSample(const Bu04PdoaData& data);
  void flushAveragedGroup();
  void enqueueAveragedGroup(const Bu04PdoaData& data);
  void sendQueuedIfDue();
  void sendJsonToServer(const Bu04PdoaSample& sample);
  void logStatusIfDue();

  Stream& debug_;
  WiFiClient client_;
  String lenBuffer_;
  String payloadBuffer_;
  uint16_t expectedLength_;
  State state_;
  unsigned long lastWifiAttempt_;
  unsigned long lastServerAttempt_;
  unsigned long lastStatusLogMs_;
  unsigned long lastByteMs_;
  unsigned long lastFrameMs_;
  bool lastWifiState_;
  bool lastServerState_;
  uint32_t totalBytes_;
  uint32_t totalFrames_;
  uint32_t totalGroups_;
  uint32_t totalSent_;
  uint32_t parseFails_;
  Bu04PdoaData samples_[16];
  uint8_t sampleCount_;
  Bu04PdoaSample queue_[3];
  uint8_t queueHead_;
  uint8_t queueTail_;
  uint8_t queueCount_;
  unsigned long lastSendMs_;
};
