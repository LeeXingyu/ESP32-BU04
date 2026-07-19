#include <Arduino.h>

#include "src/bu04/Bu04Config.h"
#include "src/bu04/Bu04Console.h"
#include "src/bu04/Bu04Uart.h"
#include "src/net/Bu04PdoaBridge.h"

HardwareSerial Bu04CmdSerial(1);
HardwareSerial Bu04DataSerial(2);
Bu04Uart bu04Cmd(Bu04CmdSerial);
Bu04Uart bu04Data(Bu04DataSerial);
Bu04Console console(Serial);
Bu04PdoaBridge pdoaBridge(Serial);

void setup() {
  Serial.begin(bu04_demo::kUsbBaud);
  delay(500);
  Serial.print("Reset reason: ");
  Serial.println(static_cast<int>(esp_reset_reason()));

  Bu04CmdSerial.begin(bu04_demo::kCmdBaud, SERIAL_8N1, bu04_demo::kCmdRxPin, bu04_demo::kCmdTxPin);
  Bu04DataSerial.begin(bu04_demo::kDataBaud, SERIAL_8N1, bu04_demo::kDataRxPin, bu04_demo::kDataTxPin);
  console.begin();
  pdoaBridge.begin();

  Serial.println();
  Serial.println("ESP32 + BU04-Kit demo ready");
  Serial.println("USB Serial = command/debug console");
  Serial.println("UART1 on GPIO32/33 = BU04 command/debug port (remapped)");
  Serial.println("UART2 on GPIO16/17 = BU04 PDOA/TWR data port");
  Serial.println("Wiring:");
  Serial.println("  USB Serial Monitor -> ESP32 USB port");
  Serial.println("  ESP32 GPIO32 -> BU04 command RX");
  Serial.println("  ESP32 GPIO33 <- BU04 command TX");
  Serial.println("  ESP32 GPIO17 -> BU04 data RX");
  Serial.println("  ESP32 GPIO16 <- BU04 data TX");
  Serial.println("  GND          -> GND");
  Serial.println("  Power        -> 3.3V or 5V per module label");
  Serial.println("USB console defaults to passthrough mode.");
  Serial.println("Type 'pass off' to use local helper commands.");

  bu04Cmd.sendCommand("AT");
}

void loop() {
  console.update(bu04Cmd);
  pdoaBridge.update(bu04Data);
}
