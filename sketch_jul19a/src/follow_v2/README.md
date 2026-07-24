# follow_v2

This folder contains an isolated follow controller for the new UWB tracking experiments.

Modes:
- `BU04_APP_MODE_FOLLOW_V2_1HZ`
- `BU04_APP_MODE_FOLLOW_V2_WINDOW`

The folder is independent from `src/follow`. When one of the `follow_v2` app modes is selected, the old follow implementation is not used.

Mode summary:
- `FOLLOW_V2_1HZ`: sample UWB at 1 Hz and dispatch a single `FollowPathPointsAction` point each cycle.
- `FOLLOW_V2_WINDOW`: keep a 30-point queue, bootstrap with 1 point for faster startup, commit samples in short 50 ms control windows, then apply angle gating, jump limiting, and Kalman smoothing on the newest valid sample before dispatching rolling 1-point startup actions and rolling 2-point windows with early reissue.
