#include "LegFollowDetector.h"

#include <HTTPClient.h>
#include <algorithm>
#include <ctype.h>
#include <math.h>
#include <time.h>
#include <WiFi.h>
#include <vector>

namespace leg_follow {
namespace {

#define BU04_LEG_FOLLOW_USB_LOG 0

void legDebugPrint(const char* msg) {
#if BU04_LEG_FOLLOW_USB_LOG
  Serial.println(msg);
#else
  (void)msg;
#endif
}

void legDebugPrintCounts(const char* tag, int a, int b, int c) {
#if BU04_LEG_FOLLOW_USB_LOG
  Serial.print("[leg] ");
  Serial.print(tag);
  Serial.print(" a=");
  Serial.print(a);
  Serial.print(" b=");
  Serial.print(b);
  Serial.print(" c=");
  Serial.println(c);
#else
  (void)tag;
  (void)a;
  (void)b;
  (void)c;
#endif
}

constexpr float kDistanceOffsetM = -0.36f;
constexpr float kLegWidthMin = 0.03f;
constexpr float kLegWidthMax = 0.22f;
constexpr float kPersonDistMin = 1.0f;
constexpr float kPersonDistMax = 3.0f;
constexpr float kPersonThetaMin = -60.0f;
constexpr float kPersonThetaMax = 60.0f;
constexpr float kLegPairDistMin = 0.10f;
constexpr float kLegPairDistMax = 0.60f;
constexpr float kClusterDistM = 0.10f;
constexpr float kAdaptiveFactor = 0.005f;
constexpr int kMinPoints = 3;
constexpr int kMaxPoints = 60;
constexpr float kDistMinM = 0.15f;
constexpr float kDistMaxM = 10.0f;
constexpr float kCircularityMax = 0.70f;
constexpr float kAspectMax = 4.5f;

struct CartPoint {
  float x = 0.0f;
  float y = 0.0f;
  float angle = 0.0f;
};

struct LegCandidate {
  float cx_m = 0.0f;
  float cy_m = 0.0f;
  float width_m = 0.0f;
  float distance_m = 0.0f;
  float angle_deg = 0.0f;
  int point_count = 0;
  float aspect = 0.0f;
  float circularity = 0.0f;
};

struct PersonCandidate {
  float d_m = 0.0f;
  float theta_deg = 0.0f;
  float mid_x_m = 0.0f;
  float mid_y_m = 0.0f;
  float leg_sep_m = 0.0f;
  LegCandidate left;
  LegCandidate right;
};

struct P15ReportData;

std::vector<LegCandidate> detectLegCandidates(const std::vector<CartPoint>& points);
bool parseBestTarget(const std::vector<CartPoint>& points, FollowTargetReport& out, const PersonTarget* preferredTarget);
bool buildP15ReportData(const std::vector<CartPoint>& points, P15ReportData& out);

float angleUwbDeg(float rad) {
  float deg = rad * 180.0f / PI;
  deg = fmodf(deg + 180.0f, 360.0f);
  if (deg < 0.0f) {
    deg += 360.0f;
  }
  deg -= 180.0f;
  return roundf(deg * 10.0f) / 10.0f;
}

bool extractNumberField(const String& text, const char* key, float& value) {
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

bool extractBoolField(const String& text, const char* key, bool& value) {
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

  if (valueStart + 4 <= static_cast<int>(text.length()) &&
      text.substring(valueStart, valueStart + 4) == "true") {
    value = true;
    return true;
  }
  if (valueStart + 5 <= static_cast<int>(text.length()) &&
      text.substring(valueStart, valueStart + 5) == "false") {
    value = false;
    return true;
  }
  return false;
}

float angleDiffDeg(float lhs, float rhs) {
  float diff = lhs - rhs;
  while (diff <= -180.0f) {
    diff += 360.0f;
  }
  while (diff > 180.0f) {
    diff -= 360.0f;
  }
  return fabsf(diff);
}

bool parsePointObject(const String& text, CartPoint& out) {
  bool valid = false;
  float angle = 0.0f;
  float distance = 0.0f;
  if (!extractBoolField(text, "valid", valid) || !valid) {
    return false;
  }
  if (!extractNumberField(text, "angle", angle) || !extractNumberField(text, "distance", distance)) {
    return false;
  }

  float d = distance + kDistanceOffsetM;
  if (d <= 0.0f || d < kDistMinM || d > kDistMaxM) {
    return false;
  }

  out.x = d * cosf(angle);
  out.y = d * sinf(angle);
  out.angle = angle;
  return true;
}

std::vector<CartPoint> getValidPoints(const String& laserscanJson) {
  std::vector<CartPoint> points;
  const int lpStart = laserscanJson.indexOf("\"laser_points\"");
  if (lpStart < 0) {
  return points;
  }

  const int arrayStart = laserscanJson.indexOf('[', lpStart);
  if (arrayStart < 0) {
    return points;
  }

  int depth = 0;
  int objectStart = -1;
  for (int i = arrayStart; i < static_cast<int>(laserscanJson.length()); ++i) {
    const char ch = laserscanJson[i];
    if (ch == '{') {
      if (depth == 0) {
        objectStart = i;
      }
      ++depth;
    } else if (ch == '}') {
      --depth;
      if (depth == 0 && objectStart >= 0) {
        const String objectText = laserscanJson.substring(objectStart, i + 1);
        CartPoint pt;
        if (parsePointObject(objectText, pt)) {
          points.push_back(pt);
        }
        objectStart = -1;
      }
    } else if (ch == ']' && depth == 0) {
      break;
    }
  }

  return points;
}

std::vector<std::vector<CartPoint>> clusterPoints(const std::vector<CartPoint>& cartPts) {
  if (cartPts.empty()) {
    return {};
  }

  std::vector<CartPoint> sortedPts = cartPts;
  std::sort(sortedPts.begin(), sortedPts.end(), [](const CartPoint& a, const CartPoint& b) {
    return a.angle < b.angle;
  });

  std::vector<std::vector<CartPoint>> clusters;
  std::vector<CartPoint> current;
  current.push_back(sortedPts[0]);

  for (size_t i = 1; i < sortedPts.size(); ++i) {
    const CartPoint& prev = sortedPts[i - 1];
    const CartPoint& cur = sortedPts[i];
    const float avgD = (hypotf(prev.x, prev.y) + hypotf(cur.x, cur.y)) / 2.0f;
    const float thresh = kClusterDistM + kAdaptiveFactor * avgD;
    if (hypotf(cur.x - prev.x, cur.y - prev.y) <= thresh) {
      current.push_back(cur);
    } else {
      clusters.push_back(current);
      current.clear();
      current.push_back(cur);
    }
  }

  if (!current.empty()) {
    clusters.push_back(current);
  }

  if (clusters.size() >= 2) {
    auto& first = clusters.front();
    auto& last = clusters.back();
    const float avgD = (hypotf(first.front().x, first.front().y) + hypotf(last.back().x, last.back().y)) / 2.0f;
    const float thresh = kClusterDistM + kAdaptiveFactor * avgD;
    if (hypotf(first.front().x - last.back().x, first.front().y - last.back().y) <= thresh) {
      std::vector<CartPoint> merged = last;
      merged.insert(merged.end(), first.begin(), first.end());
      clusters.front() = merged;
      clusters.pop_back();
    }
  }

  return clusters;
}

LegCandidate clusterGeometry(const std::vector<CartPoint>& pts) {
  LegCandidate geo;
  const int n = static_cast<int>(pts.size());
  geo.point_count = n;

  float sumX = 0.0f;
  float sumY = 0.0f;
  float minX = pts[0].x;
  float maxX = pts[0].x;
  float minY = pts[0].y;
  float maxY = pts[0].y;

  for (const auto& p : pts) {
    sumX += p.x;
    sumY += p.y;
    if (p.x < minX) minX = p.x;
    if (p.x > maxX) maxX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.y > maxY) maxY = p.y;
  }

  const float cx = sumX / n;
  const float cy = sumY / n;
  geo.cx_m = cx;
  geo.cy_m = cy;

  float avgR = 0.0f;
  float var = 0.0f;
  float maxPw = 0.0f;
  std::vector<float> dists;
  dists.reserve(pts.size());

  for (const auto& p : pts) {
    const float d = hypotf(p.x - cx, p.y - cy);
    dists.push_back(d);
    avgR += d;
  }
  avgR /= n;
  for (const float d : dists) {
    const float diff = d - avgR;
    var += diff * diff;
  }
  var /= n;

  for (size_t i = 0; i < pts.size(); ++i) {
    for (size_t j = i + 1; j < pts.size(); ++j) {
      const float dd = hypotf(pts[i].x - pts[j].x, pts[i].y - pts[j].y);
      if (dd > maxPw) {
        maxPw = dd;
      }
    }
  }

  const float spanX = maxX - minX;
  const float spanY = maxY - minY;
  const float minSpan = fminf(spanX, spanY);
  const float maxSpan = fmaxf(spanX, spanY);
  geo.width_m = maxPw > 0.005f ? maxPw : avgR * 2.5f;
  geo.aspect = minSpan > 0.001f ? (maxSpan / minSpan) : 99.0f;
  geo.circularity = avgR > 0.001f ? (sqrtf(var) / avgR) : 99.0f;
  geo.distance_m = hypotf(cx, cy);
  geo.angle_deg = angleUwbDeg(atan2f(cy, cx));
  return geo;
}

std::vector<LegCandidate> detectLegCandidates(const String& laserscanJson) {
  const std::vector<CartPoint> points = getValidPoints(laserscanJson);
  return detectLegCandidates(points);
}

std::vector<LegCandidate> detectLegCandidates(const std::vector<CartPoint>& points) {
  if (points.size() < 6U) {
    return {};
  }

  const auto clusters = clusterPoints(points);
  std::vector<LegCandidate> legs;
  for (const auto& cl : clusters) {
    if (cl.size() < static_cast<size_t>(kMinPoints) || cl.size() > static_cast<size_t>(kMaxPoints)) {
      continue;
    }

    const LegCandidate geo = clusterGeometry(cl);
    if (geo.distance_m < kDistMinM || geo.distance_m > kDistMaxM) {
      continue;
    }
    if (geo.aspect > kAspectMax) {
      continue;
    }
    if (geo.circularity > kCircularityMax) {
      continue;
    }
    if (geo.width_m < kLegWidthMin || geo.width_m > kLegWidthMax) {
      continue;
    }

    legs.push_back(geo);
  }

  return legs;
}

std::vector<PersonCandidate> pairLegs(const std::vector<LegCandidate>& legs) {
  if (legs.size() < 2U) {
    return {};
  }

  std::vector<PersonCandidate> people;
  std::vector<bool> used(legs.size(), false);

  std::vector<size_t> indexed(legs.size());
  for (size_t i = 0; i < legs.size(); ++i) {
    indexed[i] = i;
  }
  std::sort(indexed.begin(), indexed.end(), [&](size_t a, size_t b) {
    return legs[a].distance_m < legs[b].distance_m;
  });

  for (size_t i = 0; i < indexed.size(); ++i) {
    const size_t idxI = indexed[i];
    if (used[idxI]) {
      continue;
    }

    int bestJ = -1;
    float bestPref = 1e9f;
    float bestSep = 0.0f;

    for (size_t j = 0; j < indexed.size(); ++j) {
      const size_t idxJ = indexed[j];
      if (used[idxJ] || idxJ == idxI) {
        continue;
      }

      const float d = hypotf(legs[idxI].cx_m - legs[idxJ].cx_m, legs[idxI].cy_m - legs[idxJ].cy_m);
      if (d < kLegPairDistMin || d > kLegPairDistMax) {
        continue;
      }

      const float pref = fabsf(d - 0.30f);
      if (pref < bestPref) {
        bestPref = pref;
        bestJ = static_cast<int>(idxJ);
        bestSep = d;
      }
    }

    if (bestJ >= 0) {
      used[idxI] = true;
      used[static_cast<size_t>(bestJ)] = true;

      const LegCandidate& la = legs[idxI];
      const LegCandidate& lb = legs[static_cast<size_t>(bestJ)];
      PersonCandidate person;
      person.mid_x_m = (la.cx_m + lb.cx_m) / 2.0f;
      person.mid_y_m = (la.cy_m + lb.cy_m) / 2.0f;
      person.d_m = hypotf(person.mid_x_m, person.mid_y_m);
      person.theta_deg = angleUwbDeg(atan2f(person.mid_y_m, person.mid_x_m));
      person.leg_sep_m = bestSep;
      person.left = la;
      person.right = lb;
      people.push_back(person);
    }
  }

  return people;
}

size_t chooseBestPersonIndex(const std::vector<PersonCandidate>& people,
                             const PersonTarget* preferredTarget) {
  if (people.empty()) {
    return static_cast<size_t>(-1);
  }

  if (preferredTarget == nullptr) {
    return static_cast<size_t>(std::min_element(
        people.begin(),
        people.end(),
        [](const PersonCandidate& a, const PersonCandidate& b) { return a.d_m < b.d_m; }) -
                               people.begin());
  }

  size_t bestIndex = 0U;
  float bestScore = 1e9f;
  for (size_t i = 0; i < people.size(); ++i) {
    const PersonCandidate& p = people[i];
    const float distGap = fabsf(p.d_m - preferredTarget->distance_m);
    const float thetaGap = angleDiffDeg(p.theta_deg, preferredTarget->theta_deg);
    const float score = distGap / 0.80f + thetaGap / 45.0f;
    if (score < bestScore) {
      bestScore = score;
      bestIndex = i;
    }
  }
  return bestIndex;
}

bool parseBestTarget(const std::vector<CartPoint>& points, FollowTargetReport& out, const PersonTarget* preferredTarget) {
  const std::vector<LegCandidate> legs = detectLegCandidates(points);
  const std::vector<PersonCandidate> people = pairLegs(legs);

  std::vector<PersonCandidate> filtered;
  filtered.reserve(people.size());
  for (const auto& p : people) {
    if (p.d_m >= kPersonDistMin && p.d_m <= kPersonDistMax &&
        p.theta_deg >= kPersonThetaMin && p.theta_deg <= kPersonThetaMax) {
      filtered.push_back(p);
    }
  }

  out.valid = true;
  out.person_count = static_cast<uint16_t>(filtered.size());
  out.has_target = !filtered.empty();
  if (out.has_target) {
    const size_t bestIndex = chooseBestPersonIndex(filtered, preferredTarget);
    if (bestIndex < filtered.size()) {
      out.target.distance_m = filtered[bestIndex].d_m;
      out.target.theta_deg = filtered[bestIndex].theta_deg;
    }
  }
  return true;
}

bool parseBestTarget(const String& laserscanJson, FollowTargetReport& out, const PersonTarget* preferredTarget) {
  const std::vector<CartPoint> points = getValidPoints(laserscanJson);
  return parseBestTarget(points, out, preferredTarget);
}

}  // namespace

namespace {

bool readHttpLine(WiFiClient& client, String& line, unsigned long timeoutMs) {
  line = "";
  const unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (client.available() > 0) {
      const char c = static_cast<char>(client.read());
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        return true;
      }
      line += c;
    }
    if (!client.connected()) {
      return line.length() > 0U;
    }
    delay(1);
  }
  return line.length() > 0U;
}

bool readHttpBytes(WiFiClient& client, String& body, size_t byteCount, unsigned long timeoutMs) {
  body.reserve(body.length() + byteCount + 16U);
  const unsigned long start = millis();
  size_t remaining = byteCount;
  while (remaining > 0U && millis() - start < timeoutMs) {
    while (client.available() > 0 && remaining > 0U) {
      body += static_cast<char>(client.read());
      --remaining;
    }
    if (remaining == 0U) {
      return true;
    }
    if (!client.connected()) {
      break;
    }
    delay(1);
  }
  return remaining == 0U;
}

bool readChunkedBody(WiFiClient& client, String& body, unsigned long timeoutMs) {
  body = "";
  String line;
  while (readHttpLine(client, line, timeoutMs)) {
    line.trim();
    if (line.isEmpty()) {
      continue;
    }
    const size_t chunkSize = static_cast<size_t>(strtoul(line.c_str(), nullptr, 16));
    if (chunkSize == 0U) {
      while (readHttpLine(client, line, timeoutMs)) {
        if (line.isEmpty()) {
          break;
        }
      }
      return true;
    }
    if (!readHttpBytes(client, body, chunkSize, timeoutMs)) {
      return false;
    }
    if (!readHttpLine(client, line, timeoutMs)) {
      return false;
    }
  }
  return true;
}

String formatClockText() {
  time_t now = time(nullptr);
  if (now > 946684800) {
    struct tm tmInfo;
    localtime_r(&now, &tmInfo);
    char buf[32];
    strftime(buf, sizeof(buf), "%H:%M:%S", &tmInfo);
    const unsigned long msPart = millis() % 1000UL;
    char full[40];
    snprintf(full, sizeof(full), "%s.%03lu", buf, msPart);
    return String(full);
  }
  return String("ms=") + String(millis());
}

String formatDurationText(unsigned long startMs, unsigned long endMs) {
  const unsigned long diff = (endMs >= startMs) ? (endMs - startMs) : 0UL;
  return String(diff) + ".00ms";
}

struct StreamedLaserScan {
  int httpCode = 0;
  long contentLength = -1;
  bool chunked = false;
  std::vector<CartPoint> points;
};

struct LaserPointsBodyParser {
  explicit LaserPointsBodyParser(std::vector<CartPoint>& pointsOut) : points(pointsOut) {
    keyWindow.reserve(24);
    currentObject.reserve(256);
  }

  void feed(char c) {
    if (done) {
      return;
    }

    if (!seenLaserPointsKey) {
      keyWindow += c;
      if (keyWindow.length() > 24U) {
        keyWindow.remove(0, keyWindow.length() - 24U);
      }
      if (keyWindow.indexOf("\"laser_points\"") >= 0) {
        seenLaserPointsKey = true;
      }
      return;
    }

    if (!inArray) {
      if (c == '[') {
        inArray = true;
      }
      return;
    }

    if (!inObject) {
      if (c == '{') {
        inObject = true;
        braceDepth = 1;
        currentObject = "{";
      } else if (c == ']') {
        done = true;
      }
      return;
    }

    currentObject += c;
    if (c == '{') {
      ++braceDepth;
    } else if (c == '}') {
      --braceDepth;
      if (braceDepth == 0) {
        CartPoint pt;
        if (parsePointObject(currentObject, pt)) {
          points.push_back(pt);
        }
        currentObject = "";
        inObject = false;
      }
    }
  }

  bool finished() const {
    return done;
  }

  bool sawKey() const {
    return seenLaserPointsKey;
  }

  std::vector<CartPoint>& points;
  String keyWindow;
  String currentObject;
  bool seenLaserPointsKey = false;
  bool inArray = false;
  bool inObject = false;
  bool done = false;
  int braceDepth = 0;
};

bool readHttpBodyBytes(WiFiClient& client, LaserPointsBodyParser& parser, size_t byteCount, unsigned long timeoutMs) {
  const unsigned long start = millis();
  size_t remaining = byteCount;
  while (remaining > 0U && millis() - start < timeoutMs) {
    while (client.available() > 0 && remaining > 0U) {
      parser.feed(static_cast<char>(client.read()));
      --remaining;
      if (parser.finished()) {
        return true;
      }
    }
    if (remaining == 0U || parser.finished()) {
      return true;
    }
    if (!client.connected()) {
      break;
    }
    delay(1);
  }
  return remaining == 0U || parser.finished();
}

bool readHttpChunkedBody(WiFiClient& client, LaserPointsBodyParser& parser, unsigned long timeoutMs) {
  String line;
  while (readHttpLine(client, line, timeoutMs)) {
    line.trim();
    if (line.isEmpty()) {
      continue;
    }
    const size_t chunkSize = static_cast<size_t>(strtoul(line.c_str(), nullptr, 16));
    if (chunkSize == 0U) {
      while (readHttpLine(client, line, timeoutMs)) {
        if (line.isEmpty()) {
          break;
        }
      }
      return true;
    }

    if (!readHttpBodyBytes(client, parser, chunkSize, timeoutMs)) {
      return false;
    }

    if (!readHttpLine(client, line, timeoutMs)) {
      return false;
    }

    if (parser.finished()) {
      return true;
    }
  }
  return parser.finished();
}

bool fetchLaserPointsStreamed(const String& ip,
                             uint16_t port,
                             StreamedLaserScan& out,
                             unsigned long timeoutMs = 5000U) {
  out = StreamedLaserScan{};
  if (WiFi.status() != WL_CONNECTED) {
  return false;
  }

  WiFiClient client;
  client.setNoDelay(true);
  client.setTimeout(timeoutMs);
  if (!client.connect(ip.c_str(), port)) {
    return false;
  }

  String request;
  request.reserve(256);
  request += "GET /api/core/system/v1/laserscan HTTP/1.1\r\n";
  request += "Host: ";
  request += ip;
  request += ":";
  request += String(port);
  request += "\r\nAccept: application/json\r\nContent-Type: application/json\r\nConnection: close\r\nUser-Agent: python-requests/2.31.0\r\n\r\n";
  client.print(request);

  String statusLine;
  if (!readHttpLine(client, statusLine, timeoutMs)) {
    client.stop();
    return false;
  }

  const int firstSpace = statusLine.indexOf(' ');
  const int secondSpace = (firstSpace >= 0) ? statusLine.indexOf(' ', firstSpace + 1) : -1;
  if (firstSpace >= 0) {
    const String codeText = (secondSpace > firstSpace) ? statusLine.substring(firstSpace + 1, secondSpace)
                                                       : statusLine.substring(firstSpace + 1);
    out.httpCode = codeText.toInt();
  }

  String headerLine;
  while (readHttpLine(client, headerLine, timeoutMs)) {
    if (headerLine.isEmpty()) {
      break;
    }
    String lower = headerLine;
    lower.toLowerCase();
    if (lower.startsWith("content-length:")) {
      out.contentLength = headerLine.substring(headerLine.indexOf(':') + 1).toInt();
    } else if (lower.startsWith("transfer-encoding:") && lower.indexOf("chunked") >= 0) {
      out.chunked = true;
    }
  }

  LaserPointsBodyParser parser(out.points);
  bool ok = false;
  if (out.chunked) {
    ok = readHttpChunkedBody(client, parser, timeoutMs);
  } else if (out.contentLength >= 0) {
    ok = readHttpBodyBytes(client, parser, static_cast<size_t>(out.contentLength), timeoutMs);
  } else {
    const unsigned long start = millis();
    while (millis() - start < timeoutMs) {
      while (client.available() > 0) {
        parser.feed(static_cast<char>(client.read()));
        if (parser.finished()) {
          ok = true;
          client.stop();
          return true;
        }
      }
      if (!client.connected()) {
        ok = true;
        break;
      }
      delay(1);
    }
  }

  client.stop();
  return ok && parser.sawKey() && out.httpCode >= 200 && out.httpCode < 300;
}

}  // namespace

bool fetchLaserscan(const String& ip, String& response, uint16_t port) {
  if (WiFi.status() != WL_CONNECTED) {
  return false;
  }

  WiFiClient client;
  client.setTimeout(5000);
  if (!client.connect(ip.c_str(), port)) {
    return false;
  }

  const String url = String("/api/core/system/v1/laserscan");
  String request;
  request.reserve(256);
  request += "GET ";
  request += url;
  request += " HTTP/1.1\r\nHost: ";
  request += ip;
  request += ":";
  request += String(port);
  request += "\r\nAccept: application/json\r\nContent-Type: application/json\r\nConnection: close\r\nUser-Agent: python-requests/2.31.0\r\n\r\n";
  client.print(request);

  String statusLine;
  if (!readHttpLine(client, statusLine, 5000)) {
    client.stop();
    return false;
  }

  int httpCode = 0;
  {
    const int firstSpace = statusLine.indexOf(' ');
    const int secondSpace = (firstSpace >= 0) ? statusLine.indexOf(' ', firstSpace + 1) : -1;
    if (firstSpace >= 0) {
      const String codeText = (secondSpace > firstSpace) ? statusLine.substring(firstSpace + 1, secondSpace)
                                                         : statusLine.substring(firstSpace + 1);
      httpCode = codeText.toInt();
    }
  }

  bool chunked = false;
  long contentLength = -1;
  String headerLine;
  while (readHttpLine(client, headerLine, 5000)) {
    if (headerLine.isEmpty()) {
      break;
    }
    String lower = headerLine;
    lower.toLowerCase();
    if (lower.startsWith("content-length:")) {
      contentLength = headerLine.substring(headerLine.indexOf(':') + 1).toInt();
    } else if (lower.startsWith("transfer-encoding:") && lower.indexOf("chunked") >= 0) {
      chunked = true;
    }
  }

  response = "";
  bool bodyOk = false;
  if (chunked) {
    bodyOk = readChunkedBody(client, response, 5000);
  } else if (contentLength >= 0) {
    bodyOk = readHttpBytes(client, response, static_cast<size_t>(contentLength), 5000);
  } else {
    const unsigned long start = millis();
    while (millis() - start < 5000) {
      while (client.available() > 0) {
        response += static_cast<char>(client.read());
      }
      if (!client.connected()) {
        bodyOk = true;
        break;
      }
      delay(1);
    }
    if (client.available() == 0 && !client.connected()) {
      bodyOk = true;
    }
  }

  client.stop();
  return bodyOk && httpCode >= 200 && httpCode < 300;
}

bool parseFollowTargetFromLaserscan(const String& laserscanJson,
                                    FollowTargetReport& out,
                                    const PersonTarget* preferredTarget) {
  out = FollowTargetReport{};
  return parseBestTarget(laserscanJson, out, preferredTarget);
}

bool fetchFollowTarget(const String& ip,
                       FollowTargetReport& out,
                       uint16_t port,
                       const PersonTarget* preferredTarget) {
  StreamedLaserScan scan;
  if (!fetchLaserPointsStreamed(ip, port, scan)) {
    out = FollowTargetReport{};
    return false;
  }
  return parseBestTarget(scan.points, out, preferredTarget);
}

namespace {

struct P15LegView {
  float cx_m = 0.0f;
  float cy_m = 0.0f;
  float width_m = 0.0f;
  float distance_m = 0.0f;
  float angle_deg = 0.0f;
  int point_count = 0;
  float circularity = 0.0f;
};

struct P15PersonView {
  float d_m = 0.0f;
  float theta_deg = 0.0f;
  float mid_x_m = 0.0f;
  float mid_y_m = 0.0f;
  float leg_sep_m = 0.0f;
};

struct P15ReportData {
  std::vector<P15LegView> legs;
  std::vector<P15PersonView> people;
};

void appendJsonString(String& out, const String& value) {
  out += '"';
  for (size_t i = 0; i < value.length(); ++i) {
    const char ch = value[i];
    if (ch == '\\' || ch == '"') {
      out += '\\';
      out += ch;
    } else if (ch == '\n') {
      out += "\\n";
    } else if (ch == '\r') {
      out += "\\r";
    } else if (ch == '\t') {
      out += "\\t";
    } else {
      out += ch;
    }
  }
  out += '"';
}

void appendFloat(String& out, float value, uint8_t digits) {
  out += String(static_cast<double>(value), static_cast<unsigned int>(digits));
}

void appendLegJson(String& out, const P15LegView& leg) {
  out += "{\"cx_m\":";
  appendFloat(out, leg.cx_m, 4);
  out += ",\"cy_m\":";
  appendFloat(out, leg.cy_m, 4);
  out += ",\"width_m\":";
  appendFloat(out, leg.width_m, 4);
  out += ",\"distance_m\":";
  appendFloat(out, leg.distance_m, 4);
  out += ",\"angle_deg\":";
  appendFloat(out, leg.angle_deg, 1);
  out += ",\"point_count\":";
  out += String(leg.point_count);
  out += ",\"circularity\":";
  appendFloat(out, leg.circularity, 3);
  out += "}";
}

void appendPersonJson(String& out, const P15PersonView& person, int id) {
  out += "{\"id\":";
  out += String(id);
  out += ",\"d_m\":";
  appendFloat(out, person.d_m, 4);
  out += ",\"theta_deg\":";
  appendFloat(out, person.theta_deg, 1);
  out += ",\"mid_x_m\":";
  appendFloat(out, person.mid_x_m, 4);
  out += ",\"mid_y_m\":";
  appendFloat(out, person.mid_y_m, 4);
  out += ",\"leg_sep_cm\":";
  appendFloat(out, person.leg_sep_m * 100.0f, 1);
  out += "}";
}

bool buildP15ReportData(const std::vector<CartPoint>& points, P15ReportData& out) {
  const std::vector<LegCandidate> legs = detectLegCandidates(points);
  const std::vector<PersonCandidate> people = pairLegs(legs);

  out.legs.reserve(legs.size());
  for (const auto& leg : legs) {
    P15LegView view;
    view.cx_m = leg.cx_m;
    view.cy_m = leg.cy_m;
    view.width_m = leg.width_m;
    view.distance_m = leg.distance_m;
    view.angle_deg = leg.angle_deg;
    view.point_count = leg.point_count;
    view.circularity = leg.circularity;
    out.legs.push_back(view);
  }

  std::vector<PersonCandidate> filtered;
  filtered.reserve(people.size());
  for (const auto& p : people) {
    if (p.d_m >= kPersonDistMin && p.d_m <= kPersonDistMax &&
        p.theta_deg >= kPersonThetaMin && p.theta_deg <= kPersonThetaMax) {
      filtered.push_back(p);
    }
  }

  std::sort(filtered.begin(), filtered.end(), [](const PersonCandidate& a, const PersonCandidate& b) {
    return a.d_m < b.d_m;
  });

  out.people.reserve(filtered.size());
  for (const auto& p : filtered) {
    P15PersonView view;
    view.d_m = p.d_m;
    view.theta_deg = p.theta_deg;
    view.mid_x_m = p.mid_x_m;
    view.mid_y_m = p.mid_y_m;
    view.leg_sep_m = p.leg_sep_m;
    out.people.push_back(view);
  }

  return true;
}

bool buildP15ReportData(const String& laserscanJson, P15ReportData& out) {
  const std::vector<CartPoint> points = getValidPoints(laserscanJson);
  return buildP15ReportData(points, out);
}

bool buildP15ReportJsonInternal(const std::vector<CartPoint>& points,
                                const String& robotIp,
                                uint16_t robotPort,
                                String& jsonOut) {
  P15ReportData data;
  if (!buildP15ReportData(points, data)) {
    return false;
  }

  jsonOut = "";
  jsonOut.reserve(256 + data.people.size() * 128U + data.legs.size() * 128U);
  jsonOut += "{\"timestamp_ms\":";
  jsonOut += String(millis());
  jsonOut += ",\"status\":\"ok\",";
  jsonOut += "\"source\":{";
  jsonOut += "\"robot_ip\":";
  appendJsonString(jsonOut, robotIp);
  jsonOut += ",\"robot_port\":";
  jsonOut += String(robotPort);
  jsonOut += ",\"endpoint\":\"laserscan\"},";
  jsonOut += "\"person_count\":";
  jsonOut += String(data.people.size());
  jsonOut += ",\"leg_count\":";
  jsonOut += String(data.legs.size());
  jsonOut += ",\"people\":[";
  for (size_t i = 0; i < data.people.size(); ++i) {
    if (i > 0U) {
      jsonOut += ",";
    }
    appendPersonJson(jsonOut, data.people[i], static_cast<int>(i + 1U));
  }
  jsonOut += "],\"legs\":[";
  for (size_t i = 0; i < data.legs.size(); ++i) {
    if (i > 0U) {
      jsonOut += ",";
    }
    appendLegJson(jsonOut, data.legs[i]);
  }
  jsonOut += "]}";
  return true;
}

bool buildP15ReportJsonInternal(const String& laserscanJson,
                                const String& robotIp,
                                uint16_t robotPort,
                                String& jsonOut) {
  const std::vector<CartPoint> points = getValidPoints(laserscanJson);
  return buildP15ReportJsonInternal(points, robotIp, robotPort, jsonOut);
}

bool buildP15FollowTargetJsonInternal(const std::vector<CartPoint>& points, String& jsonOut) {
  P15ReportData data;
  if (!buildP15ReportData(points, data)) {
    return false;
  }

  jsonOut = "";
  jsonOut.reserve(96 + data.people.size() * 48U);
  jsonOut += "{\"status\":\"ok\",\"person_count\":";
  jsonOut += String(data.people.size());
  jsonOut += ",\"people\":[";
  for (size_t i = 0; i < data.people.size(); ++i) {
    if (i > 0U) {
      jsonOut += ",";
    }
    jsonOut += "{\"d_m\":";
    appendFloat(jsonOut, data.people[i].d_m, 4);
    jsonOut += ",\"theta_deg\":";
    appendFloat(jsonOut, data.people[i].theta_deg, 1);
    jsonOut += "}";
  }
  jsonOut += "]}";
  return true;
}

bool buildP15FollowTargetJsonInternal(const String& laserscanJson, String& jsonOut) {
  const std::vector<CartPoint> points = getValidPoints(laserscanJson);
  return buildP15FollowTargetJsonInternal(points, jsonOut);
}

bool buildP15TextResultInternal(const std::vector<CartPoint>& points, String& textOut) {
  P15ReportData data;
  if (!buildP15ReportData(points, data)) {
    return false;
  }

  if (data.people.empty()) {
    textOut = "person_count=0";
    return true;
  }

  const P15PersonView& t = data.people.front();
  textOut = "";
  textOut.reserve(64);
  textOut += "person_count=";
  textOut += String(data.people.size());
  textOut += " distance=";
  appendFloat(textOut, t.d_m, 3);
  textOut += "m angle=";
  appendFloat(textOut, t.theta_deg, 1);
  textOut += "deg";
  return true;
}

bool buildP15TextResultInternal(const String& laserscanJson, String& textOut) {
  const std::vector<CartPoint> points = getValidPoints(laserscanJson);
  return buildP15TextResultInternal(points, textOut);
}

}  // namespace

bool p15FetchLaserscan(const String& ip, String& response, uint16_t port) {
  return fetchLaserscan(ip, response, port);
}

bool p15BuildReportJson(const String& laserscanJson, String& jsonOut) {
  return buildP15ReportJsonInternal(laserscanJson, String("192.168.0.56"), 1448, jsonOut);
}

bool p15BuildReportJson(const String& laserscanJson, const String& robotIp, uint16_t robotPort, String& jsonOut) {
  return buildP15ReportJsonInternal(laserscanJson, robotIp, robotPort, jsonOut);
}

bool p15BuildFollowTargetJson(const String& laserscanJson, String& jsonOut) {
  return buildP15FollowTargetJsonInternal(laserscanJson, jsonOut);
}

bool p15PrintResult(const String& laserscanJson, Stream& out) {
  String text;
  if (!buildP15TextResultInternal(laserscanJson, text)) {
    return false;
  }
  out.println(text);
  return true;
}

bool p15PrintJsonOutput(const String& laserscanJson, Stream& out) {
  String json;
  if (!p15BuildReportJson(laserscanJson, json)) {
    return false;
  }
  out.println(json);
  return true;
}

bool p15PrintFollowOutput(const String& laserscanJson, Stream& out) {
  String json;
  if (!p15BuildFollowTargetJson(laserscanJson, json)) {
    return false;
  }
  out.println(json);
  return true;
}

bool p15FetchAndPrint(const String& ip, P15OutputFormat format, Stream& out, uint16_t port) {
  StreamedLaserScan scan;
  if (!fetchLaserPointsStreamed(ip, port, scan)) {
    out.println("[p15] ERROR: fetch_laserscan failed");
    return false;
  }

  String payload;
  switch (format) {
    case P15OutputFormat::kJson:
      if (!buildP15ReportJsonInternal(scan.points, ip, port, payload)) {
        return false;
      }
      out.println(payload);
      return true;
    case P15OutputFormat::kFollow:
      if (!buildP15FollowTargetJsonInternal(scan.points, payload)) {
        return false;
      }
      out.println(payload);
      return true;
    case P15OutputFormat::kText:
    default:
      if (!buildP15TextResultInternal(scan.points, payload)) {
        return false;
      }
      out.println(payload);
      return true;
  }
}

bool p15FetchAndPrintTimed(const String& ip, P15OutputFormat format, Stream& out, uint16_t port) {
  out.println("[p15] leg timing report");
  out.println("==============================================================");
  out.print("target: http://");
  out.print(ip);
  out.print(":");
  out.print(port);
  out.println("/api/core/system/v1/laserscan");
  out.println();
  out.println("[time]");
  out.println("--------------------------------------------------------------");
  out.print("request start:    ");
  out.println(formatClockText());

  const unsigned long requestStartMs = millis();
  StreamedLaserScan scan;
  const bool ok = fetchLaserPointsStreamed(ip, port, scan);
  const unsigned long requestEndMs = millis();

  out.print("request end:      ");
  out.println(formatClockText());
  out.print("analysis start:   ");
  out.println(formatClockText());

  const unsigned long analysisStartMs = millis();
  P15ReportData reportData;
  const bool parsedOk = ok && buildP15ReportData(scan.points, reportData);
  const unsigned long analysisEndMs = millis();

  out.print("analysis end:     ");
  out.println(formatClockText());
  out.println("--------------------------------------------------------------");
  out.print("fetch elapsed:    ");
  out.println(formatDurationText(requestStartMs, requestEndMs));
  out.print("analysis elapsed: ");
  out.println(formatDurationText(analysisStartMs, analysisEndMs));
  out.print("total elapsed:    ");
  out.println(formatDurationText(requestStartMs, analysisEndMs));
  out.println("--------------------------------------------------------------");
  out.println();
  out.println("[result]");

  if (!ok) {
    out.println(">>> request failed");
    return false;
  }

  if (!parsedOk) {
    out.println(">>> parse failed");
    return false;
  }

  if (reportData.people.empty()) {
    out.println(">>> result: no person");
    if (!reportData.legs.empty()) {
      out.print("    detected ");
      out.print(reportData.legs.size());
      out.println(" leg candidates but no pair");
      for (size_t i = 0; i < reportData.legs.size(); ++i) {
        const P15LegView& leg = reportData.legs[i];
        out.print("    leg");
        out.print(i + 1U);
        out.print(": distance=");
        out.print(leg.distance_m, 2);
        out.print("m, angle=");
        out.print(leg.angle_deg, 1);
        out.print("deg, width=");
        out.print(leg.width_m * 100.0f, 0);
        out.println("cm");
      }
    }
  } else {
    out.print(">>> result: detected ");
    out.print(reportData.people.size());
    out.println(" person(s)");
    for (size_t i = 0; i < reportData.people.size(); ++i) {
      const P15PersonView& p = reportData.people[i];
      out.println("  --------------------------------------------------------------");
      out.print("  person ");
      out.println(i + 1U);
      out.print("  d (distance) = ");
      out.print(p.d_m, 3);
      out.println(" m");
      out.print("  theta        = ");
      out.print(p.theta_deg, 1);
      out.println(" deg");
      out.print("  mid(x,y)     = (");
      out.print(p.mid_x_m, 3);
      out.print(", ");
      out.print(p.mid_y_m, 3);
      out.println(") m");
      out.print("  leg_sep      = ");
      out.print(p.leg_sep_m * 100.0f, 1);
      out.println(" cm");
    }
  }

  out.println();
  out.println("==============================================================");

  String payload;
  switch (format) {
    case P15OutputFormat::kJson:
      if (!buildP15ReportJsonInternal(scan.points, ip, port, payload)) {
        return false;
      }
      out.println(payload);
      return true;
    case P15OutputFormat::kFollow:
      if (!buildP15FollowTargetJsonInternal(scan.points, payload)) {
        return false;
      }
      out.println(payload);
      return true;
    case P15OutputFormat::kText:
    default:
      return true;
  }
}

}  // namespace leg_follow

