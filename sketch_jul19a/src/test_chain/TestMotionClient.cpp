#include "TestMotionClient.h"

#include <HTTPClient.h>

namespace test_chain {

TestMotionClient::TestMotionClient(Stream& debug) : debug_(debug) {}

String TestMotionClient::baseUrl() const {
  String url = "http://";
  url += test_chain::robotIp().toString();
  url += ":";
  url += String(test_chain::kRobotPort);
  return url;
}

float TestMotionClient::degToRad(float value) {
  return value * PI / 180.0f;
}

bool TestMotionClient::extractNumberField(const String& text, const char* key, float& value) {
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

bool TestMotionClient::extractStringField(const String& text, const char* key, String& value) {
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

bool TestMotionClient::requestJson(const String& method,
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

bool TestMotionClient::getPose(RobotPose& pose) {
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

bool TestMotionClient::startMoveTo(float x, float y, String& actionId) {
  String body;
  body.reserve(256);
  body += "{\"action_name\":\"slamtec.agent.actions.MoveToAction\",";
  body += "\"options\":{\"target\":{\"x\":";
  body += String(x, 3);
  body += ",\"y\":";
  body += String(y, 3);
  body += ",\"z\":0},\"move_options\":{\"mode\":0,\"flags\":[]}}}";

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

bool TestMotionClient::startRotateTo(float angleDeg, String& actionId) {
  String body;
  body.reserve(192);
  const float angleRad = degToRad(angleDeg);
  body += "{\"action_name\":\"slamtec.agent.actions.RotateAction\",";
  body += "\"options\":{\"angle\":";
  body += String(angleRad, 6);
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

bool TestMotionClient::getActionRunning(const String& actionId, bool& running) {
  int httpCode = 0;
  String response;
  if (!requestJson("GET", baseUrl() + "/api/core/motion/v1/actions/" + actionId, "", httpCode, response)) {
    return false;
  }
  if (httpCode != HTTP_CODE_OK) {
    return false;
  }

  if (response.indexOf("\"action_name\"") < 0) {
    running = false;
    return true;
  }

  if (response.indexOf("finished") >= 0 ||
      response.indexOf("completed") >= 0 ||
      response.indexOf("canceled") >= 0 ||
      response.indexOf("cancelled") >= 0 ||
      response.indexOf("failed") >= 0) {
    running = false;
    return true;
  }

  running = true;
  return true;
}

bool TestMotionClient::cancelCurrentAction() {
  int httpCode = 0;
  String response;
  if (!requestJson("DELETE", baseUrl() + "/api/core/motion/v1/actions/:current", "", httpCode, response)) {
    return false;
  }
  return httpCode >= 200 && httpCode < 300;
}

}  // namespace test_chain
