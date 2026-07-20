#pragma once

#include <Arduino.h>
#include <WiFi.h>

namespace follow_demo {

struct RobotPose {
  float x = 0.0f;
  float y = 0.0f;
  float yaw = 0.0f;
};

class SlamtecRestClient {
 public:
  explicit SlamtecRestClient(Stream& debug);

  bool getPose(RobotPose& pose);
  bool startMoveTo(float x, float y, String& actionId);
  bool getActionRunning(const String& actionId, bool& running);
  bool cancelCurrentAction();

 private:
  String baseUrl() const;
  static bool extractNumberField(const String& text, const char* key, float& value);
  static bool extractStringField(const String& text, const char* key, String& value);
  bool requestJson(const String& method,
                   const String& url,
                   const String& body,
                   int& httpCode,
                   String& response);

  Stream& debug_;
};

}  // namespace follow_demo
