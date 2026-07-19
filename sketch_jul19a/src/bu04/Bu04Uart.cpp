#include "Bu04Uart.h"

Bu04Uart::Bu04Uart(HardwareSerial& serial) : serial_(serial) {}

void Bu04Uart::begin(uint32_t baudRate, int rxPin, int txPin) {
  serial_.begin(baudRate, SERIAL_8N1, rxPin, txPin);
}

void Bu04Uart::sendCommand(const char* command) {
  serial_.print(command);
  serial_.print("\r\n");
}

void Bu04Uart::sendCommand(const String& command) {
  serial_.print(command);
  serial_.print("\r\n");
}

void Bu04Uart::sendLine(const String& line) {
  serial_.print(line);
  serial_.print("\r\n");
}

bool Bu04Uart::readByte(char& out) {
  if (serial_.available() <= 0) {
    return false;
  }

  out = static_cast<char>(serial_.read());
  return true;
}

bool Bu04Uart::readLine(String& line, uint32_t timeoutMs) {
  line = "";
  const unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (serial_.available() > 0) {
      const char c = static_cast<char>(serial_.read());
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        return true;
      }
      line += c;
    }
    delay(1);
  }

  return false;
}

size_t Bu04Uart::drainTo(Stream& out) {
  size_t count = 0;
  while (serial_.available() > 0) {
    out.write(serial_.read());
    ++count;
  }
  return count;
}

bool Bu04Uart::ping(uint32_t timeoutMs) {
  sendCommand("AT");
  String reply;
  return readLine(reply, timeoutMs) && reply.length() > 0;
}
