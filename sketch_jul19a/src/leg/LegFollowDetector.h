#pragma once

#include <Arduino.h>

namespace leg_follow {

struct PersonTarget {
  float distance_m = 0.0f;
  float theta_deg = 0.0f;
};

struct FollowTargetReport {
  bool valid = false;
  uint16_t person_count = 0;
  bool has_target = false;
  PersonTarget target;
};

bool fetchLaserscan(const String& ip, String& response, uint16_t port = 1448);
bool parseFollowTargetFromLaserscan(const String& laserscanJson,
                                    FollowTargetReport& out,
                                    const PersonTarget* preferredTarget = nullptr);
bool fetchFollowTarget(const String& ip,
                       FollowTargetReport& out,
                       uint16_t port = 1448,
                       const PersonTarget* preferredTarget = nullptr);

enum class P15OutputFormat {
  kText,
  kJson,
  kFollow,
};

bool p15FetchLaserscan(const String& ip, String& response, uint16_t port = 1448);
bool p15BuildReportJson(const String& laserscanJson, String& jsonOut);
bool p15BuildReportJson(const String& laserscanJson, const String& robotIp, uint16_t robotPort, String& jsonOut);
bool p15BuildFollowTargetJson(const String& laserscanJson, String& jsonOut);
bool p15PrintResult(const String& laserscanJson, Stream& out);
bool p15PrintJsonOutput(const String& laserscanJson, Stream& out);
bool p15PrintFollowOutput(const String& laserscanJson, Stream& out);
bool p15FetchAndPrint(const String& ip, P15OutputFormat format, Stream& out, uint16_t port = 1448);
bool p15FetchAndPrintTimed(const String& ip, P15OutputFormat format, Stream& out, uint16_t port = 1448);

}  // namespace leg_follow
