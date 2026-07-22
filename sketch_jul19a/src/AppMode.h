#pragma once

// 0 = TCP bridge under src/net
// 1 = REST follow controller under src/follow
// 2 = REST test chain under src/test_chain
#define BU04_APP_MODE_TCP 0
#define BU04_APP_MODE_FOLLOW 1
#define BU04_APP_MODE_TEST_CHAIN 2

// Change this macro to switch app entry.
#define BU04_APP_MODE BU04_APP_MODE_FOLLOW
