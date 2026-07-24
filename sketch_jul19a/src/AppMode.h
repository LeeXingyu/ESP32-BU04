#pragma once

// 0 = TCP bridge under src/net
// 1 = REST follow controller under src/follow
// 2 = REST test chain under src/test_chain
// 3 = new REST follow controller under src/follow_v2 using 1 Hz single-point dispatch
// 4 = new REST follow controller under src/follow_v2 using rolling 5-point windows
// 5 = REST follow controller under src/follow_v3, modeled after uwb_follow1_1.ino
// 6 = independent TCP fusion debug reporter, samples BU04 at 15 Hz and reports angle pairs
#define BU04_APP_MODE_TCP 0
#define BU04_APP_MODE_FOLLOW 1
#define BU04_APP_MODE_TEST_CHAIN 2
#define BU04_APP_MODE_FOLLOW_V2_1HZ 3
#define BU04_APP_MODE_FOLLOW_V2_WINDOW 4
#define BU04_APP_MODE_FOLLOW_V3 5
#define BU04_APP_MODE_FUSION_DEBUG 6

// Change this macro to switch app entry.
#define BU04_APP_MODE BU04_APP_MODE_FOLLOW_V3
