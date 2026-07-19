# BU04 Module

This folder contains the BU04-specific code used by the root Arduino sketch.

- `Bu04Config.h` pins and baud rate configuration
- `Bu04Uart.*` UART transport and read helpers
- `Bu04Console.*` command parser for the USB debug port
- `Bu04TwrParser.*` TWR frame parser for `JSxxxx{...}` data

## Current wiring

- USB Serial: onboard USB-UART bridge to `Serial`
- BU04 command/debug UART1: ESP32 GPIO33 (RX), GPIO32 (TX)
- BU04 PDOA/TWR data UART2: ESP32 GPIO16 (RX), GPIO17 (TX)

## Runtime roles

- USB `Serial`: user debug and command input
- `Serial1`: BU04 command/debug path, remapped to GPIO33/32
- `Serial2`: BU04 PDOA/TWR data path, fixed on GPIO16/17
