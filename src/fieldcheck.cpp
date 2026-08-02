#include "fieldcheck.h"
#include "profiles.h"
#include <Arduino.h>

struct ColorStats {
    uint32_t confirmedFrames;
    uint32_t confSum, confMin, confMax;
    uint32_t fillSum, coreSum, pixelSum;
    uint32_t fragmentFrames;   // frames where the target arrived in pieces
    uint32_t edgeFrames;
};

static bool       g_active = false;
static uint32_t   g_endMs  = 0;
static uint32_t   g_startMs = 0;
static uint32_t   g_frames = 0;
static uint32_t   g_scanMicrosSum = 0;
static uint32_t   g_capHits = 0;
static uint32_t   g_rejected[REJ_REASON_COUNT];
static uint32_t   g_rejectedMaxPx[REJ_REASON_COUNT];
static ColorStats g_stats[COLOR_COUNT];

void fieldcheckStart(int seconds) {
    if (seconds < 1)  seconds = 1;
    if (seconds > 60) seconds = 60;

    g_active        = true;
    g_startMs       = millis();
    g_endMs         = g_startMs + (uint32_t)seconds * 1000u;
    g_frames        = 0;
    g_scanMicrosSum = 0;
    g_capHits       = 0;

    for (int r = 0; r < REJ_REASON_COUNT; r++) { g_rejected[r] = 0; g_rejectedMaxPx[r] = 0; }
    for (int c = 0; c < COLOR_COUNT; c++) {
        g_stats[c] = ColorStats{};
        g_stats[c].confMin = 999;
    }

    Serial.printf("\n[FIELDCHECK] Sampling for %d s -- hold the scene steady...\n", seconds);
}

bool fieldcheckActive() { return g_active; }

void fieldcheckAbort() {
    if (!g_active) return;
    g_active = false;
    Serial.println("[FIELDCHECK] Aborted.");
}

static void report() {
    uint32_t elapsed = millis() - g_startMs;
    Serial.println("\n================ FIELDCHECK ================");
    Serial.printf("%u frames in %u ms (%.1f fps), mean scan %u us\n",
        (unsigned)g_frames, (unsigned)elapsed,
        elapsed ? (g_frames * 1000.0f / elapsed) : 0.0f,
        (unsigned)(g_frames ? g_scanMicrosSum / g_frames : 0));

    Serial.println("\n  colour   seen%   conf(avg/min/max)  fill%  core%   px    frag  edge");
    for (int c = 0; c < COLOR_COUNT; c++) {
        const ColorStats &s = g_stats[c];
        uint32_t n = s.confirmedFrames;
        float seenPct = g_frames ? (100.0f * n / g_frames) : 0.0f;

        if (n == 0) {
            Serial.printf("  %-7s  %5.1f   -- never confirmed --\n", colorName((ColorId)c), seenPct);
            continue;
        }
        Serial.printf("  %-7s  %5.1f   %3u / %3u / %3u     %3u    %3u  %5u   %3u   %3u\n",
            colorName((ColorId)c), seenPct,
            (unsigned)(s.confSum / n), (unsigned)s.confMin, (unsigned)s.confMax,
            (unsigned)(s.fillSum / n), (unsigned)(s.coreSum / n),
            (unsigned)(s.pixelSum / n),
            (unsigned)s.fragmentFrames, (unsigned)s.edgeFrames);
    }

    Serial.println("\n  Candidates rejected, by gate:");
    uint32_t total = 0;
    for (int r = 1; r < REJ_REASON_COUNT; r++) total += g_rejected[r];
    if (total == 0) {
        Serial.println("    (none -- no gate fired at all this run)");
    } else {
        Serial.println("    gate            count   biggest one rejected");
        for (int r = 1; r < REJ_REASON_COUNT; r++)
            if (g_rejected[r])
                Serial.printf("    %-14s %6u   %u px  (%.0f%% of rejections)\n",
                              rejectReasonName(r), (unsigned)g_rejected[r],
                              (unsigned)g_rejectedMaxPx[r], 100.0f * g_rejected[r] / total);
        Serial.println("    -> 'biggest one rejected' is where to move the threshold.");
    }
    if (g_capHits)
        Serial.printf("  [!] %u frames hit an internal cap -- scene is unusually noisy.\n",
                      (unsigned)g_capHits);

    Serial.println("\n  Reading this:");
    Serial.println("   - Target in view      -> want seen% > 95 and a steady conf.");
    Serial.println("   - Course empty        -> want every colour 'never confirmed'.");
    Serial.println("     If something confirms, the gate list shows what ISN'T stopping it.");
    Serial.println("   - frag > 0            -> occlusion merging is doing real work.");
    Serial.println("============================================\n");
}

void fieldcheckSample(const DetectResult &r, const FilterResult &f, const BlobScanStats &st) {
    if (!g_active) return;

    g_frames++;
    g_scanMicrosSum += st.scanMicros;
    if (st.labelCapHit || st.runCapHit || st.outCapHit) g_capHits++;

    for (int i = 0; i < REJ_REASON_COUNT; i++) {
        g_rejected[i] += f.rejected[i];
        if (f.rejectedMaxPx[i] > g_rejectedMaxPx[i]) g_rejectedMaxPx[i] = f.rejectedMaxPx[i];
    }

    for (int c = 0; c < COLOR_COUNT; c++) {
        const ColorDetection &d = r.color[c];
        if (!d.found) continue;
        ColorStats &s = g_stats[c];
        s.confirmedFrames++;
        s.confSum  += d.confidence;
        if (d.confidence < s.confMin) s.confMin = d.confidence;
        if (d.confidence > s.confMax) s.confMax = d.confidence;
        s.fillSum  += d.fillPct;
        s.coreSum  += d.corePct;
        s.pixelSum += d.pixels;
        if (d.fragments > 1) s.fragmentFrames++;
        if (d.touchesEdge)   s.edgeFrames++;
    }

    if ((int32_t)(millis() - g_endMs) >= 0) {
        g_active = false;
        report();
    }
}
