#include "blob_detect.h"
#include "color_lut.h"
#include "frame_config.h"
#include <string.h>

// Timing uses micros() on target; under the native test build there is no
// Arduino, so fall back to zero rather than dragging in a clock.
#ifdef ARDUINO
  #include <Arduino.h>
  #define NOW_MICROS() micros()
#else
  #define NOW_MICROS() 0u
#endif

#define MAX_LABELS 1024
#define NO_LABEL   0

struct Run {
    int16_t  x0, x1;    // inclusive
    uint16_t core;      // core-flagged pixels within this run
    uint16_t label;
    uint8_t  color;     // 1..COLOR_COUNT; a 0-colour run is never stored
};

struct LabelAcc {
    uint32_t pixels;
    uint32_t corePixels;
    uint32_t sumX, sumY;
    int16_t  x0, y0, x1, y1;
    uint8_t  color;
};

// Static, not heap: guarantees internal SRAM and makes an over-budget
// configuration a link error rather than a runtime failure.
static Run      g_runsA[MAX_RUNS_PER_ROW];
static Run      g_runsB[MAX_RUNS_PER_ROW];
static LabelAcc g_acc[MAX_LABELS];
static uint16_t g_parent[MAX_LABELS];

static BlobComponent g_scratch[MAX_COMPONENTS];
BlobComponent* blobScratch() { return g_scratch; }

static int g_minRunLen = 3;

void setMinRunLength(int px) { g_minRunLen = (px < 1) ? 1 : px; }
int  getMinRunLength()       { return g_minRunLen; }

ScanRoi fullFrameRoi() {
    ScanRoi r = { 0, 0, (int16_t)(FRAME_W - 1), (int16_t)(FRAME_H - 1) };
    return r;
}

// ---- union-find with path halving ----
// Smaller index wins, so a component's root is always its first-created label.

static inline uint16_t ufFind(uint16_t x) {
    while (g_parent[x] != x) {
        g_parent[x] = g_parent[g_parent[x]];
        x = g_parent[x];
    }
    return x;
}

static inline uint16_t ufUnion(uint16_t a, uint16_t b) {
    a = ufFind(a);
    b = ufFind(b);
    if (a == b) return a;
    if (b < a) { uint16_t t = a; a = b; b = t; }
    g_parent[b] = a;
    return a;
}

static inline void accumulateRun(uint16_t label, const Run &r, int y) {
    LabelAcc &a  = g_acc[label];
    uint32_t len = (uint32_t)(r.x1 - r.x0 + 1);

    a.pixels     += len;
    a.corePixels += r.core;
    // Closed-form sum of x over [x0..x1]. (x0+x1)*len is always even, so the
    // halving is exact - no per-pixel loop needed for the centroid.
    a.sumX += (uint32_t)(((int32_t)r.x0 + r.x1) * (int32_t)len / 2);
    a.sumY += len * (uint32_t)y;

    if (r.x0 < a.x0) a.x0 = r.x0;
    if (r.x1 > a.x1) a.x1 = r.x1;
    if (y    < a.y0) a.y0 = (int16_t)y;
    if (y    > a.y1) a.y1 = (int16_t)y;
}

int blobScan(const uint8_t *frame, const uint8_t *lut, const ScanRoi &roi,
             BlobComponent *out, int maxOut, BlobScanStats *stats) {
    uint32_t t0 = NOW_MICROS();

    int rx0 = roi.x0 < 0 ? 0 : roi.x0;
    int ry0 = roi.y0 < 0 ? 0 : roi.y0;
    int rx1 = roi.x1 > FRAME_W - 1 ? FRAME_W - 1 : roi.x1;
    int ry1 = roi.y1 > FRAME_H - 1 ? FRAME_H - 1 : roi.y1;

    if (stats) {
        stats->componentCount = 0;
        stats->runsTotal      = 0;
        stats->scanMicros     = 0;
        stats->labelCapHit    = false;
        stats->runCapHit      = false;
        stats->outCapHit      = false;
    }
    if (rx1 < rx0 || ry1 < ry0) return 0;

    Run     *prev = g_runsA, *cur = g_runsB;
    int      nPrev = 0, nCur = 0;
    uint16_t nextLabel  = 1;            // 0 means "unlabelled"
    uint32_t runsTotal  = 0;
    bool     labelCapHit = false, runCapHit = false;

    for (int y = ry0; y <= ry1; y++) {
        const uint8_t *p = frame + ((size_t)y * FRAME_W + rx0) * 2;
        nCur = 0;

        // ---- run-length encode this row ----
        int x = rx0;
        while (x <= rx1) {
            uint8_t e = lut[((uint16_t)p[0] << 8) | p[1]];
            uint8_t c = lutColorOf(e);
            if (c == 0) { x++; p += 2; continue; }

            int      x0   = x;
            uint16_t core = 0;
            for (;;) {
                if (e & LUT_CORE_BIT) core++;
                x++; p += 2;
                if (x > rx1) break;
                e = lut[((uint16_t)p[0] << 8) | p[1]];
                if (lutColorOf(e) != c) break;
            }

            if (x - x0 < g_minRunLen)      continue;   // horizontal erosion
            if (nCur >= MAX_RUNS_PER_ROW) { runCapHit = true; continue; }

            Run &r = cur[nCur++];
            r.x0 = (int16_t)x0; r.x1 = (int16_t)(x - 1);
            r.core = core; r.color = c; r.label = NO_LABEL;
            runsTotal++;
        }

        // ---- link to the row above ----
        // Both lists are sorted by x and internally disjoint, so this is a
        // two-pointer merge: O(runs), not O(runs^2).
        int j = 0;
        for (int i = 0; i < nCur; i++) {
            Run     &r     = cur[i];
            uint16_t label = NO_LABEL;

            // Skip prev runs entirely left of r. Safe to advance j past them
            // permanently: later cur runs start further right still.
            // 8-connectivity, so a one-pixel diagonal gap still links.
            while (j < nPrev && prev[j].x1 < r.x0 - 1) j++;

            for (int k = j; k < nPrev && prev[k].x0 <= r.x1 + 1; k++) {
                if (prev[k].color != r.color || prev[k].label == NO_LABEL) continue;
                label = (label == NO_LABEL) ? ufFind(prev[k].label)
                                            : ufUnion(label, prev[k].label);
            }

            if (label == NO_LABEL) {
                if (nextLabel >= MAX_LABELS) { labelCapHit = true; continue; }
                label = nextLabel++;
                g_parent[label] = label;
                LabelAcc &a = g_acc[label];
                a.pixels = 0; a.corePixels = 0; a.sumX = 0; a.sumY = 0;
                a.x0 = FRAME_W; a.y0 = FRAME_H; a.x1 = -1; a.y1 = -1;
                a.color = r.color;
            }
            r.label = label;
            accumulateRun(label, r, y);
        }

        Run *swap = prev; prev = cur; cur = swap;
        nPrev = nCur;
    }

    // ---- fold every label's accumulator into its root ----
    // Roots have parent[l]==l and are never folded, so a single forward pass
    // is enough regardless of the order labels were merged in.
    for (uint16_t l = 1; l < nextLabel; l++) {
        uint16_t root = ufFind(l);
        if (root == l) continue;
        LabelAcc &a = g_acc[l];
        LabelAcc &b = g_acc[root];
        b.pixels     += a.pixels;
        b.corePixels += a.corePixels;
        b.sumX       += a.sumX;
        b.sumY       += a.sumY;
        if (a.x0 < b.x0) b.x0 = a.x0;
        if (a.y0 < b.y0) b.y0 = a.y0;
        if (a.x1 > b.x1) b.x1 = a.x1;
        if (a.y1 > b.y1) b.y1 = a.y1;
        a.pixels = 0;
    }

    // ---- emit roots ----
    int  n = 0;
    bool outCapHit = false;
    for (uint16_t l = 1; l < nextLabel; l++) {
        if (ufFind(l) != l) continue;
        LabelAcc &a = g_acc[l];
        if (a.pixels == 0) continue;
        if (n >= maxOut) { outCapHit = true; break; }

        BlobComponent &c = out[n++];
        c.color      = a.color;
        c.pixels     = a.pixels;
        c.corePixels = a.corePixels;
        c.x0 = a.x0; c.y0 = a.y0; c.x1 = a.x1; c.y1 = a.y1;
        c.cx = (int16_t)(a.sumX / a.pixels);
        c.cy = (int16_t)(a.sumY / a.pixels);
    }

    if (stats) {
        stats->componentCount = (uint16_t)n;
        stats->runsTotal      = runsTotal;
        stats->labelCapHit    = labelCapHit;
        stats->runCapHit      = runCapHit;
        stats->outCapHit      = outCapHit;
        stats->scanMicros     = NOW_MICROS() - t0;
    }
    return n;
}
