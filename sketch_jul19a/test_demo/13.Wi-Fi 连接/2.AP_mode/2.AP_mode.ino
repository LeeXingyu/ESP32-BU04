/*
  该程序作用是使 ESP32 创建热点
  在线文档：https://docs.geeksman.com/esp32/Arduino/18.esp32-arduino-wifi.html
*/
#include <WiFi.h>


// 设置名称密码
const char * ssid = "ESP32_AP";
const char * password = "123456qq.";


void setup() {
  Serial.begin(9600);

  // 创建热点
  WiFi.softAP(ssid, password);

  // 打印 热点IP
  Serial.print("Wi-Fi 接入的IP：");
  Serial.println(WiFi.softAPIP());

}

void loop() {

}
