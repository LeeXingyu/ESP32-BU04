#pragma once

#include <Arduino.h>

class Bu04Uart {
 public:
  explicit Bu04Uart(HardwareSerial& serial);

  void begin(uint32_t baudRate, int rxPin, int txPin);
  void sendCommand(const char* command);
  void sendCommand(const String& command);
  void sendLine(const String& line);

  bool readByte(char& out);
  bool readLine(String& line, uint32_t timeoutMs = 1000);
  size_t drainTo(Stream& out);
  bool ping(uint32_t timeoutMs = 1000);

 private:
  HardwareSerial& serial_;
};
