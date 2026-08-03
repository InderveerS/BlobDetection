#pragma once
#include <stdint.h>
#include "detect_result.h"
#include "blob_filter.h"
#include "blob_detect.h"

// ================================================================
// WiFi debug telemetry.  ***NOT FOR COMPETITION.***
//
// Off at boot, every boot, unconditionally. It is deliberately NOT persisted
// to NVS - there is no combination of saved settings that can bring WiFi up on
// its own, so the only way it runs during a match is if somebody types the
// command during the match. Turning it on prints a loud warning.
//
// To remove it from the binary entirely for competition, build with
// -UENABLE_WIFI_DEBUG (see platformio.ini). Everything below then compiles to
// nothing and the commands report that it wasn't built in.
//
// Why core 0: WiFi's own tasks are pinned to core 0 by default in ESP-IDF, and
// the vision pipeline owns core 1. The HTTP servers are pinned to core 0 too,
// so all of this stays off the critical path. What it CANNOT avoid is sharing
// the PSRAM bus and the (core-shared) data cache with the camera DMA - so
// expect some frame-rate cost while streaming video. Telemetry alone is cheap.
//
// Two servers on two ports, which is the standard esp32-camera arrangement:
// an MJPEG handler blocks its server task for as long as the client is
// watching, so it gets its own port and can't stall the page or the JSON.
//
//   http://<ip>/         debug page (self-contained, no internet needed)
//   http://<ip>/data     JSON telemetry snapshot   <- always available
//   http://<ip>:81/stream  MJPEG video with blob overlay  <- best effort
// ================================================================

#define WIFI_DEBUG_AP_SSID  "teletubby-vision"
#define WIFI_DEBUG_AP_PASS  "enph253robot"      // >= 8 chars or the AP won't start

// Video preview is half-resolution (160x120). Encoding QVGA costs roughly four
// times as much for a preview you don't need at that size.
#define PREVIEW_W (FRAME_W / 2)
#define PREVIEW_H (FRAME_H / 2)
#define PREVIEW_JPEG_QUALITY 55       // 1-100, higher is better and slower
#define STREAM_DEFAULT_FPS    10

#ifdef ENABLE_WIFI_DEBUG

bool wifiDebugStartAP();
bool wifiDebugStartSTA(const char *ssid, const char *pass);
void wifiDebugStop();
void wifiDebugStatus();
bool wifiDebugRunning();

// Called from the vision loop every frame. Both are non-blocking: if the
// encoder is busy or no client is connected, they return immediately and the
// frame is simply skipped. Vision is never made to wait.
void wifiDebugPublish(const DetectResult &r, const FilterResult &f,
                      const BlobScanStats &st, float fps);
void wifiDebugOfferFrame(const uint8_t *frame);

void setStreamFps(int fps);
int  getStreamFps();

#else
// Competition build: wifi_debug.cpp isn't compiled at all, so the whole
// feature collapses to these inline no-ops. Callers stay #ifdef-free.
#include <Arduino.h>

inline void wifiDebugStatus() {
    Serial.println("[WIFI] Not in this build (competition firmware -- no radio compiled in).");
}
inline bool wifiDebugStartAP()                          { wifiDebugStatus(); return false; }
inline bool wifiDebugStartSTA(const char *, const char *) { wifiDebugStatus(); return false; }
inline void wifiDebugStop()                             {}
inline bool wifiDebugRunning()                          { return false; }
inline void wifiDebugPublish(const DetectResult &, const FilterResult &,
                             const BlobScanStats &, float) {}
inline void wifiDebugOfferFrame(const uint8_t *)        {}
inline void setStreamFps(int)                           {}
inline int  getStreamFps()                              { return 0; }
#endif
