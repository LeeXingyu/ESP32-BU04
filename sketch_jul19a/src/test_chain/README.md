# BU04 Test Chain

This folder contains an independent REST angle-test chain controlled by `src/AppMode.h`.

Modes:
- `BU04_TEST_CHAIN_MODE_PERIODIC_MOVE`
- `BU04_TEST_CHAIN_MODE_STRAIGHT_ONLY`
- `BU04_TEST_CHAIN_MODE_ROTATE_ONLY`

The test chain is isolated from `src/follow` and can be enabled without changing the follow controller.

Behavior:
- `PERIODIC_MOVE`: sends a fixed move command every few seconds.
- `STRAIGHT_ONLY`: accepts all BU04 frames, then converts them into straight forward motion by keeping `X` fixed and only changing `Y`.
- `ROTATE_ONLY`: samples one BU04 frame every 10 seconds, computes its angle, converts it to radians, and sends that angle for rotation testing.
