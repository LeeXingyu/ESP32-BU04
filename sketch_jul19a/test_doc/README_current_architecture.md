# Current Architecture

This document summarizes the current Arduino project structure, runtime roles, and data flow for the BU04 + ESP32 demo.

## 1. Project Goal

The project is organized around one root sketch and two mutually exclusive runtime paths:

- `TCP` mode: receive BU04 PDOA/TWR data on `Serial2`, filter it, and forward condensed JSON to a TCP server.
- `FOLLOW` mode: receive BU04 PDOA/TWR data on `Serial2`, filter it, and drive Slamtec REST actions for following.

The active path is selected at compile time by `src/AppMode.h`.

## 2. Main Entry Point

The root sketch is:

- [`sketch_jul19a.ino`](../sketch_jul19a.ino)

It creates three serial roles:

- USB debug / command console on `Serial`
- BU04 command/debug port on `Serial1`
- BU04 PDOA/TWR data port on `Serial2`

Current wiring in code:

- `Serial1` is mapped to GPIO32/33
- `Serial2` is mapped to GPIO16/17

## 3. Compile-Time Mode Switch

The mode flag is:

- [`src/AppMode.h`](../src/AppMode.h)

Current behavior:

- `BU04_APP_USE_FOLLOW = 0` selects `src/net`
- `BU04_APP_USE_FOLLOW = 1` selects `src/follow`

Only one controller is instantiated in the sketch.

## 4. USB Console and BU04 Command Path

The USB console is handled by:

- [`src/bu04/Bu04Console.h`](../src/bu04/Bu04Console.h)
- [`src/bu04/Bu04Console.cpp`](../src/bu04/Bu04Console.cpp)

Its responsibilities are:

- accept typed commands from the USB serial monitor
- forward raw AT lines to BU04 when passthrough mode is enabled
- send helper commands such as `base`, `tagrole`, `addtag`, `restore`, `save`
- optionally mirror BU04 `Serial1` replies back to USB with `mirror on`

Useful console commands:

- `help`
- `pass on`
- `pass off`
- `mirror on`
- `mirror off`
- `mode`
- `ping`
- `at <cmd>`
- `base`
- `tagrole`
- `uwbmode <n>`
- `save`
- `restore`
- `addtag <id>`
- `tag <id>`
- `dump`
- `echo <text>`

## 5. BU04 UART Wrapper

The common UART wrapper is:

- [`src/bu04/Bu04Uart.h`](../src/bu04/Bu04Uart.h)
- [`src/bu04/Bu04Uart.cpp`](../src/bu04/Bu04Uart.cpp)

It provides:

- `begin(...)`
- `sendCommand(...)`
- `sendLine(...)`
- `readByte(...)`
- `readLine(...)`
- `drainTo(...)`
- `ping(...)`

This keeps low-level serial operations out of the higher-level controller logic.

## 6. TCP Path

The TCP path is implemented in:

- [`src/net/Bu04PdoaBridge.h`](../src/net/Bu04PdoaBridge.h)
- [`src/net/Bu04PdoaBridge.cpp`](../src/net/Bu04PdoaBridge.cpp)
- [`src/net/NetConfig.h`](../src/net/NetConfig.h)

### 6.1 Responsibilities

- connect WiFi STA
- connect to a fixed TCP server
- parse BU04 `JSxxxx{...}` framed data from `Serial2`
- extract BU04 fields
- average one group of 16 frames
- average the middle 10 frames for selected fields
- compute angle from filtered `Xcm/Ycm`
- keep a 3-item queue of outgoing samples
- send one JSON record to the TCP server every second

### 6.2 Parsed Fields

The TCP parser currently extracts:

- `a16`
- `R`
- `T`
- `D`
- `P`
- `Xcm`
- `Ycm`
- `O`
- `V`
- `X`
- `Y`
- `Z`

### 6.3 TCP Output Format

The current TCP payload is compact JSON with:

- `type`
- `a16`
- `T`
- `Xcm`
- `Ycm`
- `D`
- `angle`

### 6.4 TCP Diagnostics

The TCP path now emits periodic status logs with:

- received bytes
- parsed frames
- averaged groups
- sent packets
- queue depth
- parse failures
- idle time since the last input byte

## 7. Follow Path

The follow path is implemented in:

- [`src/follow/UwbFollowRest.h`](../src/follow/UwbFollowRest.h)
- [`src/follow/UwbFollowRest.cpp`](../src/follow/UwbFollowRest.cpp)
- [`src/follow/SlamtecRestClient.h`](../src/follow/SlamtecRestClient.h)
- [`src/follow/SlamtecRestClient.cpp`](../src/follow/SlamtecRestClient.cpp)
- [`src/follow/FollowConfig.h`](../src/follow/FollowConfig.h)

### 7.1 Responsibilities

- connect WiFi STA
- parse BU04 `JSxxxx{...}` frames from `Serial2`
- extract the fields needed for follow control
- filter `Xcm/Ycm`
- reject sudden outliers unless they repeat consistently
- estimate velocity
- compute a follow target point
- query Slamtec pose with REST
- sample coordinates at 1 Hz and send single-point `FollowPathPointsAction` requests
- lower follow speed ratio to reduce braking jerk
- monitor action status
- wait for the current follow-path action to finish before dispatching the next point

### 7.2 Current Follow Filtering

The follow path currently supports two filter modes:

- existing path: clamp jump + EMA + velocity smoothing
- optional path: Kalman filter

The active mode is controlled by:

- `BU04_FOLLOW_ENABLE_KALMAN_FILTER`

### 7.3 Outlier Guard

The follow path also has a raw-sample stability guard:

- it keeps the last stable raw samples
- it compares new raw points against a median reference
- a one-off jump is rejected
- repeated jumps can be accepted after a strike threshold

This is controlled by:

- `BU04_FOLLOW_ENABLE_OUTLIER_GUARD`
- `BU04_FOLLOW_OUTLIER_DISTANCE_CM`
- `BU04_FOLLOW_OUTLIER_ACCEPT_STRIKES`

### 7.4 Dispatch Throttling

The follow path has throttles for:

- minimum dispatch interval
- minimum dispatch distance
- live replan gap
- point spacing in the queue

## 8. WiFi and Logging

Current log intervals are intentionally relaxed to reduce spam.

Relevant knobs:

- `BU04_FOLLOW_WIFI_RECONNECT_MS`
- `BU04_FOLLOW_STATUS_LOG_MS`
- `kWifiReconnectMs`
- `kServerReconnectMs`
- `kStatusLogMs`

## 9. Known Assumptions

The current `SlamtecRestClient` parser uses lightweight string extraction instead of a full JSON parser.

That means it works best when the REST response is a simple JSON object containing top-level numeric fields such as:

- `x`
- `y`
- `yaw`

If the response shape changes to nested JSON, the parser should be upgraded.

## 10. Recommended Runtime Interpretation

Think of the project as three layers:

- transport layer: `Bu04Uart`, `Bu04Console`
- BU04 data layer: `src/net` or `src/follow`
- integration layer: `WiFi`, TCP server, or Slamtec REST

## 11. File Map

- [`sketch_jul19a.ino`](../sketch_jul19a.ino)
- [`src/AppMode.h`](../src/AppMode.h)
- [`src/bu04/Bu04Config.h`](../src/bu04/Bu04Config.h)
- [`src/bu04/Bu04Console.h`](../src/bu04/Bu04Console.h)
- [`src/bu04/Bu04Console.cpp`](../src/bu04/Bu04Console.cpp)
- [`src/bu04/Bu04Uart.h`](../src/bu04/Bu04Uart.h)
- [`src/bu04/Bu04Uart.cpp`](../src/bu04/Bu04Uart.cpp)
- [`src/net/Bu04PdoaBridge.h`](../src/net/Bu04PdoaBridge.h)
- [`src/net/Bu04PdoaBridge.cpp`](../src/net/Bu04PdoaBridge.cpp)
- [`src/net/NetConfig.h`](../src/net/NetConfig.h)
- [`src/follow/FollowConfig.h`](../src/follow/FollowConfig.h)
- [`src/follow/UwbFollowRest.h`](../src/follow/UwbFollowRest.h)
- [`src/follow/UwbFollowRest.cpp`](../src/follow/UwbFollowRest.cpp)
- [`src/follow/SlamtecRestClient.h`](../src/follow/SlamtecRestClient.h)
- [`src/follow/SlamtecRestClient.cpp`](../src/follow/SlamtecRestClient.cpp)
