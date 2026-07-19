/*
  该程序作用是使 ESP32 连接 WiFi，并发送网络请求
  在线文档：https://docs.geeksman.com/esp32/Arduino/19.esp32-arduino-request.html
*/
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>


const char *ssid = "GeeksMan";
const char *password = "123456qq.";

String url = "http://apis.juhe.cn/simpleWeather/query";
String city = "上海";
String key = "a702b975fa48a063b1a57c938bafb47a";


void setup() {
  Serial.begin(9600);

  WiFi.disconnect(true);

  // 连接 WiFi
  WiFi.begin(ssid, password);

  Serial.print("正在连接 WiFi.");

  // 检测是否链接成功
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    }

  Serial.println("连接成功");

  // 创建 HTTPClient 对象
  HTTPClient http;

  // 指定访问 URL
  http.begin(url+"?city="+city+"&key="+key);

  // 接收 HTTP 响应状态码
  int http_code = http.GET();

  Serial.printf("HTTP 状态码：%d\n", http_code);

  // 获取响应正文
  String response = http.getString();
  Serial.print("响应数据：");
  Serial.println(response);

  // 关闭连接
  http.end();

  // 创建 DynamicJsonDocument 对象
  DynamicJsonDocument doc(1024);

  // 解析 JSON 数据
  deserializeJson(doc, response);

  // 从解析的 JSON 数据中获取值
  unsigned int temp = doc["result"]["realtime"]["temperature"].as<unsigned int>();
  String info = doc["result"]["realtime"]["info"].as<String>();
  int aqi = doc["result"]["realtime"]["aqi"].as<int>();

  Serial.printf("温度：%d, 天气：%s, 空气指数: %d\n", temp, info, aqi);
}


void loop() {

}
