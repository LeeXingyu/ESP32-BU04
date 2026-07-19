/*
  该程序作用是使 ESP32 连接 WiFi，并在串口屏打印相关信息
  在线文档：https://docs.geeksman.com/esp32/Arduino/18.esp32-arduino-wifi.html
*/
#include <WiFi.h>

#define LED   2

// 定义 WiFi 名与密码
const char * ssid = "yuechuang24";
const char * password = "Yuechuang1";

void setup() {
  Serial.begin(115200);

  // 连接 WiFi
  WiFi.begin(ssid, password);

  Serial.print("正在连接 Wi-Fi");
  
  // 检测是否连接成功
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    }

  Serial.println("连接成功");
  Serial.print("IP 地址：");
  Serial.println(WiFi.localIP());

  // 通过板载 LED 反馈 WiFi 连接状态
  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);

  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);

  digitalWrite(LED, HIGH);
  delay(1500);
  digitalWrite(LED, LOW);
}

void loop() {
}
