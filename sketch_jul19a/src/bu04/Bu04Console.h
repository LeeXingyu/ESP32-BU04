#pragma once

#include <Arduino.h>

#include "Bu04Uart.h"

class Bu04Console {
 public:
  explicit Bu04Console(Stream& usb);

  void begin();
  void update(Bu04Uart& device);

 private:
  void handleLine(Bu04Uart& device, const String& line);
  void printHelp();
  void sendRawAndReport(Bu04Uart& device, const String& command, const char* note = nullptr);
  void configureBase(Bu04Uart& device);
  void configureTagRole(Bu04Uart& device);
  void saveConfig(Bu04Uart& device);
  void restoreConfig(Bu04Uart& device);
  void addTag(Bu04Uart& device, const String& tagId);
  void setPassthrough(bool enabled);
  void setMirror(bool enabled);
  void printMode() const;

  Stream& usb_;
  String input_;
  bool passthroughMode_ = true;
  bool mirrorMode_ = true;
};
