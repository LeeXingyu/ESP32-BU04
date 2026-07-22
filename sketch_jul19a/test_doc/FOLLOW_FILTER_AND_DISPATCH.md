# Follow Filter and Dispatch Notes

This document focuses on the `follow` runtime path and the data-handling logic used before robot motion is dispatched.

## 1. Input Frame

The follow path reads BU04 data frames from `Serial2`.

The frame format currently expected by the parser is:

- frame prefix: `JS`
- 4 hex characters for payload length
- payload containing a `TWR` JSON object

The parser extracts:

- `a16`
- `T`
- `D`
- `Xcm`
- `Ycm`

Relevant code:

- [`src/follow/UwbFollowRest.cpp`](../src/follow/UwbFollowRest.cpp)

## 2. Why This Is Not Just Passthrough

The follow path is not a raw relay.

It performs:

- parsing
- input gating
- outlier rejection
- smoothing
- speed estimation
- queueing
- motion dispatch

The goal is to turn noisy UWB samples into stable follow targets.

## 3. Current Non-Kalman Filter Chain

When `BU04_FOLLOW_ENABLE_KALMAN_FILTER = 0`, the current chain is:

1. raw sample arrives
2. raw outlier guard runs
3. jump clamp limits sudden per-frame movement
4. EMA smooths `Xcm/Ycm`
5. velocity is estimated and smoothed
6. angle is computed
7. safe target point is generated
8. point is queued
9. `MoveToAction` is dispatched later

Relevant knobs:

- `BU04_FOLLOW_EMA_ALPHA`
- `BU04_FOLLOW_VELOCITY_ALPHA`
- `BU04_FOLLOW_CLAMP_MAX_JUMP_CM`

## 4. Raw Outlier Guard

The outlier guard is enabled by:

- `BU04_FOLLOW_ENABLE_OUTLIER_GUARD`

It uses:

- a 3-sample raw history
- a median reference
- a distance threshold
- a consecutive-strike counter

Behavior:

- if the new raw point is close enough to the recent stable pattern, it is accepted
- if it jumps too far once or twice, it is rejected
- if the jump persists for enough consecutive frames, the code resets stability and accepts the new region

Relevant knobs:

- `BU04_FOLLOW_OUTLIER_DISTANCE_CM`
- `BU04_FOLLOW_OUTLIER_ACCEPT_STRIKES`

## 5. Kalman Switch

The filter implementation is selected by:

- `BU04_FOLLOW_ENABLE_KALMAN_FILTER`

When enabled:

- `Xcm/Ycm` are processed by a lightweight 1D Kalman filter per axis
- position and velocity are estimated from the same chain

When disabled:

- the original clamp + EMA path is used

Relevant knobs:

- `BU04_FOLLOW_KALMAN_MEAS_NOISE_CM2`
- `BU04_FOLLOW_KALMAN_ACCEL_NOISE_CM2_PER_S4`
- `BU04_FOLLOW_KALMAN_INIT_POS_VAR`
- `BU04_FOLLOW_KALMAN_INIT_VEL_VAR`

## 6. Angle Logic

The angle is computed from the filtered values:

- `angle = atan2(xcm, ycm) * 180 / PI`

The code checks a track-angle limit to avoid following when the tag direction is too far off-axis.

Relevant knobs:

- `BU04_FOLLOW_MAX_TRACK_ANGLE_DEG`
- `BU04_FOLLOW_ANGLE_RECOVER_GAP_DEG`

## 7. Distance Safety

The `D` field is used as the primary distance gate.

Behavior:

- too close: stop or do not continue approaching
- within a safety threshold: cancel current action if needed

Relevant knobs:

- `BU04_FOLLOW_TOO_CLOSE_DISTANCE_M`
- `BU04_FOLLOW_SAFETY_STOP_DISTANCE_M`
- `BU04_FOLLOW_SLOW_DISTANCE_M`
- `BU04_FOLLOW_FAST_DISTANCE_M`

## 8. Point Queue

The follow path maintains a FIFO queue of target points.

Properties:

- queue size is controlled by `BU04_FOLLOW_QUEUE_SIZE`
- bootstrap dispatch waits until enough points are collected
- the queue drops the oldest point when full

Current queue-related knobs:

- `BU04_FOLLOW_QUEUE_SIZE`
- `BU04_FOLLOW_BOOTSTRAP_POINTS`
- `BU04_FOLLOW_LOOKAHEAD_POINTS`
- `BU04_FOLLOW_MIN_POINT_SPACING_CM`
- `BU04_FOLLOW_ENABLE_POINT_SPACING_GUARD`

## 9. Dispatch Throttling

To reduce robot jitter and repeated replans, the follow path now limits dispatches with:

- minimum dispatch distance
- minimum dispatch interval

Relevant knobs:

- `BU04_FOLLOW_MIN_SEND_DISTANCE_M`
- `BU04_FOLLOW_MIN_SEND_INTERVAL_MS`

The current values are intentionally conservative so the robot does not receive too many motion updates.

## 10. REST Integration

The follow path uses `SlamtecRestClient` to talk to the robot.

Responsibilities:

- get pose
- start move action
- check action state
- cancel current action

Important note:

- the current implementation uses simple string extraction for JSON fields
- this is fine for a flat response, but a full JSON parser would be safer if the response format becomes nested

Relevant file:

- [`src/follow/SlamtecRestClient.cpp`](../src/follow/SlamtecRestClient.cpp)

## 11. Logging and Debugging

The follow path prints periodic status logs rather than spamming the USB console.

The status log includes:

- total parsed frames
- queued points
- motion actions started
- rejected raw samples

Relevant knob:

- `BU04_FOLLOW_STATUS_LOG_MS`

## 12. How to Tune

Suggested tuning order:

1. adjust `BU04_FOLLOW_OUTLIER_DISTANCE_CM`
2. adjust `BU04_FOLLOW_OUTLIER_ACCEPT_STRIKES`
3. adjust `BU04_FOLLOW_EMA_ALPHA`
4. adjust `BU04_FOLLOW_MIN_SEND_INTERVAL_MS`
5. adjust `BU04_FOLLOW_MIN_SEND_DISTANCE_M`

If the robot follows too lazily:

- lower the send interval
- lower the send distance
- slightly raise `EMA_ALPHA`

If the robot jitters too much:

- raise the send interval
- raise the send distance
- lower `EMA_ALPHA`
- raise the outlier distance threshold slightly

## 13. Runtime Status Indicators

The follow status log is a good quick health check.

If you see:

- `rejected` rising quickly, the outlier guard may be too strict
- `queue` staying near zero, the follow target generation may be too conservative
- `actions` not increasing, the REST dispatch may be blocked by pose fetch or safety gates

## 14. Key Files

- [`src/follow/FollowConfig.h`](../src/follow/FollowConfig.h)
- [`src/follow/UwbFollowRest.h`](../src/follow/UwbFollowRest.h)
- [`src/follow/UwbFollowRest.cpp`](../src/follow/UwbFollowRest.cpp)
- [`src/follow/SlamtecRestClient.h`](../src/follow/SlamtecRestClient.h)
- [`src/follow/SlamtecRestClient.cpp`](../src/follow/SlamtecRestClient.cpp)

