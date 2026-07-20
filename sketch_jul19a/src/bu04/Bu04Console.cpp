#include "Bu04Console.h"

#include "Bu04Config.h"

Bu04Console::Bu04Console(Stream& usb) : usb_(usb) {}

void Bu04Console::begin() {
  input_.reserve(128);
}

void Bu04Console::update(Bu04Uart& device) {
  while (usb_.available() > 0) {
    const char c = static_cast<char>(usb_.read());
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (input_.length() > 0) {
        handleLine(device, input_);
        input_ = "";
      }
      continue;
    }

    input_ += c;
    if (input_.length() >= 120) {
      handleLine(device, input_);
      input_ = "";
    }
  }

  if (mirrorMode_) {
    device.drainTo(usb_);
  }
}

void Bu04Console::printHelp() {
  usb_.println("Commands:");
  usb_.println("  help           - show this help");
  usb_.println("  pass on        - forward every line to BU04");
  usb_.println("  pass off       - enable local command parsing");
  usb_.println("  mirror on      - mirror BU04 Serial1 replies to USB");
  usb_.println("  mirror off     - stop mirroring BU04 Serial1 replies");
  usb_.println("  mode           - show current mode");
  usb_.println("  ping           - send AT and read one line");
  usb_.println("  at <cmd>       - send raw AT command");
  usb_.println("  base           - AT+SETCFG=0,1,1,1 + AT+SETUWBMODE=1 + AT+SAVE");
  usb_.println("  tagrole        - AT+SETCFG=0,0,1,1 + AT+SETUWBMODE=1 + AT+SAVE");
  usb_.println("  uwbmode <n>    - send AT+SETUWBMODE=<n>");
  usb_.println("  save           - send AT+SAVE");
  usb_.println("  restore        - send AT+RESTORE");
  usb_.println("  addtag <id>    - send AT+ADDTAG=<id>,8834,1,64,0");
  usb_.println("  tag <id>       - alias of addtag");
  usb_.println("  dump           - drain all pending BU04 bytes");
  usb_.println("  echo <text>    - print locally");
  usb_.println();
  usb_.println("Any other non-empty line is forwarded raw to BU04.");
}

void Bu04Console::setPassthrough(bool enabled) {
  passthroughMode_ = enabled;
  usb_.print("Passthrough mode: ");
  usb_.println(enabled ? "ON" : "OFF");
}

void Bu04Console::setMirror(bool enabled) {
  mirrorMode_ = enabled;
  usb_.print("Mirror BU04 Serial1 to USB: ");
  usb_.println(enabled ? "ON" : "OFF");
}

void Bu04Console::printMode() const {
  usb_.print("Current mode: ");
  usb_.println(passthroughMode_ ? "PASSTHROUGH" : "LOCAL COMMANDS");
}

void Bu04Console::sendRawAndReport(Bu04Uart& device, const String& command, const char* note) {
  device.sendCommand(command);
  if (note != nullptr) {
    usb_.print(note);
    usb_.print(": ");
  }
  usb_.println(command);
}

void Bu04Console::configureBase(Bu04Uart& device) {
  sendRawAndReport(device, bu04_demo::kCmdSetCfgBase, "Sent");
  sendRawAndReport(device, bu04_demo::kCmdSetUwbModePdoa, "Sent");
  sendRawAndReport(device, bu04_demo::kCmdSave, "Sent");
}

void Bu04Console::configureTagRole(Bu04Uart& device) {
  sendRawAndReport(device, bu04_demo::kCmdSetCfgTag, "Sent");
  sendRawAndReport(device, bu04_demo::kCmdSetUwbModePdoa, "Sent");
  sendRawAndReport(device, bu04_demo::kCmdSave, "Sent");
}

void Bu04Console::saveConfig(Bu04Uart& device) {
  sendRawAndReport(device, bu04_demo::kCmdSave, "Sent");
}

void Bu04Console::restoreConfig(Bu04Uart& device) {
  sendRawAndReport(device, bu04_demo::kCmdRestore, "Sent");
}

void Bu04Console::addTag(Bu04Uart& device, const String& tagId) {
  if (tagId.length() == 0) {
    usb_.println("Usage: addtag <tag_id>");
    return;
  }

  const String command = String("AT+ADDTAG=") + tagId + bu04_demo::kCmdAddTagTail;
  sendRawAndReport(device, command, "Sent TAG add");
}

void Bu04Console::handleLine(Bu04Uart& device, const String& line) {
  if (line == "help") {
    printHelp();
    return;
  }

  if (line == "mode") {
    printMode();
    return;
  }

  if (line == "pass on") {
    setPassthrough(true);
    return;
  }

  if (line == "pass off") {
    setPassthrough(false);
    return;
  }

  if (line == "mirror on") {
    setMirror(true);
    return;
  }

  if (line == "mirror off") {
    setMirror(false);
    return;
  }

  if (passthroughMode_) {
    device.sendLine(line);
    usb_.println("Forwarded raw line to BU04.");
    return;
  }

  if (line == "ping") {
    usb_.println("Sending AT...");
    device.sendCommand("AT");
    String reply;
    if (device.readLine(reply, 1000)) {
      usb_.print("Reply: ");
      usb_.println(reply);
    } else {
      usb_.println("No reply");
    }
    return;
  }

  if (line.startsWith("at ")) {
    device.sendCommand(line.substring(3));
    usb_.println("Sent raw AT command.");
    return;
  }

  if (line == "base") {
    configureBase(device);
    return;
  }

  if (line == "tagrole") {
    configureTagRole(device);
    return;
  }

  if (line == "save") {
    saveConfig(device);
    return;
  }

  if (line == "restore") {
    restoreConfig(device);
    return;
  }

  if (line.startsWith("uwbmode ")) {
    const String mode = line.substring(8);
    if (mode.length() == 0) {
      usb_.println("Usage: uwbmode <n>");
      return;
    }

    sendRawAndReport(device, String("AT+SETUWBMODE=") + mode, "Sent UWB mode");
    return;
  }

  if (line.startsWith("addtag ")) {
    addTag(device, line.substring(7));
    return;
  }

  if (line.startsWith("tag ")) {
    addTag(device, line.substring(4));
    return;
  }

  if (line == "dump") {
    const size_t count = device.drainTo(usb_);
    usb_.print("Drained bytes: ");
    usb_.println(static_cast<unsigned long>(count));
    return;
  }

  if (line.startsWith("echo ")) {
    usb_.println(line.substring(5));
    return;
  }

  device.sendLine(line);
  usb_.println("Forwarded raw line to BU04.");
}
