/*
  该程序作用是通过串口助手与串口监视器进行数据交互
  在线文档：https://docs.geeksman.com/esp32/Arduino/27.esp32-arduino-uart.html
*/

void setup() {
  // 初始化串口通信波特率
  Serial.begin(9600);
  Serial2.begin(9600);
}

void loop() {
  // 从串口监视器读取输入数据
  if (Serial.available()) {
    char data = Serial.read();

    // 将数据发送到 UART2
    Serial2.write(data);
  }

  // 从 UART2 读取输入数据
  if (Serial2.available()) {
    char data = Serial2.read();

    // 将数据发送到 UART0
    Serial.write(data);
  }

}
