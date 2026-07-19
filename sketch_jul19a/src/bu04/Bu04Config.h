#pragma once

#include <Arduino.h>

namespace bu04_demo {

// USB serial console.
constexpr uint32_t kUsbBaud = 115200;

// ESP32 UART1 wiring to BU04 command/debug port.
constexpr int kCmdRxPin = 33;  // ESP32 RX1 <- BU04 command TX
constexpr int kCmdTxPin = 32;  // ESP32 TX1 -> BU04 command RX

// ESP32 UART2 wiring to BU04 PDOA/TWR data port.
constexpr int kDataRxPin = 16;  // ESP32 RX2 <- BU04 data TX
constexpr int kDataTxPin = 17;  // ESP32 TX2 -> BU04 data RX

// Serial speeds.
constexpr uint32_t kCmdBaud = 115200;
constexpr uint32_t kDataBaud = 115200;
constexpr uint32_t kBu04Baud = 115200;

// Common BU04 AT command templates from the reference doc.
constexpr const char* kCmdSetCfgBase = "AT+SETCFG=0,1,1,1";
constexpr const char* kCmdSetCfgTag = "AT+SETCFG=0,0,1,1";
constexpr const char* kCmdSetUwbModePdoa = "AT+SETUWBMODE=1";
constexpr const char* kCmdSave = "AT+SAVE";
constexpr const char* kCmdRestore = "AT+RESTORE";
constexpr const char* kCmdAddTagTail = ",8834,1,64,0";

}  // namespace bu04_demo
