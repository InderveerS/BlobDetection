#include <Arduino.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "camera_setup.h"
#include "frame_config.h"
#include "profiles.h"
#include "autotune.h"
#include "blob_detect.h"
#include "blob_filter.h"
#include "tracker.h"
#include "detect_result.h"
#include "color_lut.h"
#include "serial_commands.h"
#include "vision_output.h"
#include "vision_link.h"
#include "vision_nvs.h"
#include "fieldcheck.h"
#include "wifi_debug.h"

static uint32_t     g_lastFpsMs  = 0;
static int          g_frameCount = 0;
static float        g_fps        = 0;
static uint32_t     g_frameId    = 0;
static DetectResult g_result;

const DetectResult& latestResult() { return g_result; }

// A frame whose geometry doesn't match frame_config.h would make every pixel
// index in blobScan()/tuneProcessFrame() an out-of-bounds read. The sensor can
// silently fall back to a different framesize, so check rather than trust.
// Rate-limited because if it ever fires it fires on every single frame.
static bool frameIsSane(const camera_fb_t *fb) {
    if (fb->width == FRAME_W && fb->height == FRAME_H && fb->len >= FRAME_BYTES)
        return true;

    static uint32_t lastComplaintMs = 0;
    uint32_t now = millis();
    if (now - lastComplaintMs >= 1000) {
        lastComplaintMs = now;
        Serial.printf("[ERR] Frame is %ux%u len=%u, expected %dx%d len>=%u -- dropping.\n"
                      "      Camera fell back to another framesize; check cfg.frame_size "
                      "in camera_setup.cpp against frame_config.h.\n",
                      (unsigned)fb->width, (unsigned)fb->height, (unsigned)fb->len,
                      FRAME_W, FRAME_H, (unsigned)FRAME_BYTES);
    }
    return false;
}

static void reportStartupMemory() {
    Serial.printf("[MEM] internal free: %u B (largest block %u B)   PSRAM free: %u B\n",
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    uint32_t cov[COLOR_COUNT];
    lutCoverage(cov);
    Serial.printf("[LUT] built in %u us. Colour-space coverage (of 65536 RGB565 values):\n",
                  (unsigned)lutLastBuildMicros());
    for (int c = 0; c < COLOR_COUNT; c++) {
        Serial.printf("      %-7s %6u  (%.2f%%)\n",
            colorName((ColorId)c), (unsigned)cov[c], 100.0f * cov[c] / (float)LUT_SIZE);
    }
    Serial.println("      A colour claiming a large slice will pick up half the world -- retune it.");
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    // Belt and braces: the command reader is non-blocking, but this keeps any
    // other Stream call from parking the vision loop for a full second.
    Serial.setTimeout(5);
    delay(1000);
    Serial.println("\n================================");
    Serial.println("  Freenove ESP32-S3 Teletubby Vision");
    Serial.println("================================");
    Serial.println("Type 'help' for commands\n");

    if (!cameraInit()) {
        Serial.println("[HALTED] Camera failed. Check ribbon cable and power.");
        while (true) delay(1000);
    }

    CameraId cam = detectCameraId();
    Serial.printf("[OK] Camera ready: %s\n", cameraName(cam));

    profilesInit(cam);       // also builds the colour LUT
    filterSetDefaults();
    trackerSetDefaults();
    visionLinkBegin();       // starts the core-0 task; failure is non-fatal

    // Compiled-in defaults first, then anything saved for this sensor on top.
    if (visionNvsLoad())
        Serial.println("[NVS] Loaded saved settings for this camera.");
    else
        Serial.println("[NVS] No saved settings -- using compiled-in defaults.");

    reportStartupMemory();
    Serial.printf("Detection output: %s -- use 'verbose 1' or 'verbose 2' to see it.\n",
                  verbosityName(getVerbosity()));
    Serial.println("\n--- Ready. ---");

    g_lastFpsMs = millis();   // else the first FPS window includes all of setup()
}

static void printResult(const DetectResult &r, const BlobScanStats &st, const FilterResult &f) {
    Serial.printf("[%5.1ffps %3uus %2uc]", g_fps,
                  (unsigned)st.scanMicros, (unsigned)st.componentCount);

    for (int c = 0; c < COLOR_COUNT; c++) {
        const ColorDetection &d = r.color[c];
        if (!d.found) { Serial.printf("  %c:-", colorName((ColorId)c)[0]); continue; }
        Serial.printf("  %c:%3u@(%3d,%3d) %2ux%2u f%02u c%02u",
            colorName((ColorId)c)[0], d.confidence, d.cx, d.cy,
            (unsigned)(d.x1 - d.x0 + 1), (unsigned)(d.y1 - d.y0 + 1),
            d.fillPct, d.corePct);
        if (d.fragments > 1)  Serial.printf(" j%u", d.fragments);
        if (d.touchesEdge)    Serial.print(" E");
    }

    if (r.bestColor != NO_COLOR) Serial.printf("  best=%s", colorName((ColorId)r.bestColor));

    // Only surface rejections when something was actually thrown away.
    uint16_t rejTotal = 0;
    for (int i = 1; i < REJ_REASON_COUNT; i++) rejTotal = (uint16_t)(rejTotal + f.rejected[i]);
    if (rejTotal) {
        Serial.print("  rej:");
        for (int i = 1; i < REJ_REASON_COUNT; i++)
            if (f.rejected[i])
                Serial.printf(" %s=%u(max %upx)", rejectReasonName(i),
                              f.rejected[i], (unsigned)f.rejectedMaxPx[i]);
    }
    if (st.labelCapHit || st.runCapHit || st.outCapHit) Serial.print("  [CAP HIT]");
    Serial.println();
}

void loop() {
    handleSerialCommands();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[ERR] Frame grab failed");
        delay(100);
        return;
    }
    if (!frameIsSane(fb)) {
        esp_camera_fb_return(fb);
        delay(100);
        return;
    }

    g_frameCount++;
    uint32_t now = millis();
    if (now - g_lastFpsMs >= 1000) {
        g_fps        = g_frameCount * 1000.0f / (float)(now - g_lastFpsMs);
        g_frameCount = 0;
        g_lastFpsMs  = now;
    }

    if (tuneIsActive()) {
        tuneProcessFrame(fb);   // interactive - always prints, ignores verbosity
    } else {
        BlobScanStats st;
        FilterResult  f;

        BlobComponent *comps = blobScratch();
        int n = blobScan(fb->buf, lutData(), getScanRoi(),
                         comps, MAX_COMPONENTS, &st);
        blobFilter(comps, n, f);
        trackerUpdate(f, ++g_frameId, g_result);

        visionLinkPublish(g_result);   // non-blocking handoff to core 0
        fieldcheckSample(g_result, f, st);

        // Debug telemetry. Both no-op instantly when WiFi is off, which is
        // every boot unless somebody explicitly turned it on.
        wifiDebugPublish(g_result, f, st, g_fps);
        wifiDebugOfferFrame(fb->buf);

        if (shouldPrintDetection()) printResult(g_result, st, f);
    }

    esp_camera_fb_return(fb);
}
