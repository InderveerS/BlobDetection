#include "tracker.h"

#define DEF_CONFIRM_M    3
#define DEF_CONFIRM_N    5
#define DEF_GATE_PX     60
#define DEF_MAX_MISS    10
#define DEF_ALPHA_PCT   40

// Centroids are held in 8.8 fixed point so the EMA doesn't quantise away on
// slow movement - integer smoothing at alpha 0.4 would otherwise never move
// the position by less than 2 px.
#define FP 256

struct Track {
    bool     active;
    bool     confirmed;
    uint16_t history;     // bit 0 = this frame, 1 = seen
    uint8_t  misses;
    int32_t  cxFp, cyFp;
    ColorCandidate last;
};

static Track g_track[COLOR_COUNT];

static int g_m       = DEF_CONFIRM_M;
static int g_n       = DEF_CONFIRM_N;
static int g_gatePx  = DEF_GATE_PX;
static int g_maxMiss = DEF_MAX_MISS;
static int g_alpha   = DEF_ALPHA_PCT;

static inline int popcount16(uint16_t v) {
    int c = 0;
    while (v) { v &= (uint16_t)(v - 1); c++; }
    return c;
}

void trackerReset() {
    for (int c = 0; c < COLOR_COUNT; c++) {
        g_track[c].active    = false;
        g_track[c].confirmed = false;
        g_track[c].history   = 0;
        g_track[c].misses    = 0;
        g_track[c].cxFp      = 0;
        g_track[c].cyFp      = 0;
    }
}

void trackerSetDefaults() {
    g_m       = DEF_CONFIRM_M;
    g_n       = DEF_CONFIRM_N;
    g_gatePx  = DEF_GATE_PX;
    g_maxMiss = DEF_MAX_MISS;
    g_alpha   = DEF_ALPHA_PCT;
    trackerReset();
}

void setTrackerConfirm(int m, int n) {
    if (n < 1)  n = 1;
    if (n > 16) n = 16;      // history is a uint16 bitfield
    if (m < 1)  m = 1;
    if (m > n)  m = n;
    g_m = m; g_n = n;
}
void getTrackerConfirm(int &m, int &n) { m = g_m; n = g_n; }

void setTrackerGatePx(int px)    { g_gatePx  = px < 1 ? 1 : px; }
int  getTrackerGatePx()          { return g_gatePx; }
void setTrackerMaxMiss(int f)    { g_maxMiss = f  < 0 ? 0 : f; }
int  getTrackerMaxMiss()         { return g_maxMiss; }
void setTrackerSmoothing(int a)  { g_alpha   = (a < 1) ? 1 : (a > 100 ? 100 : a); }
int  getTrackerSmoothing()       { return g_alpha; }

static inline bool withinGate(const Track &t, const ColorCandidate &c, int gate) {
    int dx = (int)(t.cxFp / FP) - c.cx;
    int dy = (int)(t.cyFp / FP) - c.cy;
    return (dx * dx + dy * dy) <= (gate * gate);
}

void trackerUpdate(const FilterResult &f, uint32_t frameId, DetectResult &out) {
    out.frameId   = frameId;
    out.bestColor = NO_COLOR;

    int bestConf = -1;

    for (int c = 0; c < COLOR_COUNT; c++) {
        Track                &t    = g_track[c];
        const ColorCandidate &cand = f.color[c];
        bool matched = false;

        if (cand.valid) {
            if (!t.active) {
                // First sighting: adopt it, but unconfirmed until M-of-N passes.
                t.active = true;
                t.cxFp   = (int32_t)cand.cx * FP;
                t.cyFp   = (int32_t)cand.cy * FP;
                matched  = true;
            } else if (withinGate(t, cand, g_gatePx)) {
                matched = true;
            } else if (t.misses >= g_maxMiss / 2) {
                // Track has been stale a while and a candidate has appeared
                // elsewhere. Re-acquire rather than stubbornly holding a
                // position the target has clearly left.
                t.cxFp     = (int32_t)cand.cx * FP;
                t.cyFp     = (int32_t)cand.cy * FP;
                t.history  = 0;
                t.confirmed = false;
                matched    = true;
            }
            // else: a valid candidate too far from an actively-tracked target.
            // Counted as a miss, which is what keeps a passing distraction from
            // yanking the track off a real teletubby.
        }

        t.history = (uint16_t)((t.history << 1) | (matched ? 1u : 0u));

        if (matched) {
            t.misses = 0;
            t.last   = cand;
            // EMA on the centroid.
            t.cxFp += ((int32_t)cand.cx * FP - t.cxFp) * g_alpha / 100;
            t.cyFp += ((int32_t)cand.cy * FP - t.cyFp) * g_alpha / 100;
        } else if (t.active) {
            if (t.misses < 255) t.misses++;
            if (t.misses > g_maxMiss) {
                t.active    = false;
                t.confirmed = false;
                t.history   = 0;
            }
        }

        // M-of-N over the last N frames.
        uint16_t mask = (uint16_t)((g_n >= 16) ? 0xFFFF : ((1u << g_n) - 1u));
        if (t.active && popcount16((uint16_t)(t.history & mask)) >= g_m)
            t.confirmed = true;

        // ---- publish ----
        ColorDetection &d = out.color[c];
        d.found = t.active && t.confirmed;
        if (d.found) {
            d.confidence  = t.last.confidence;
            d.cx          = (int16_t)(t.cxFp / FP);
            d.cy          = (int16_t)(t.cyFp / FP);
            d.x0          = t.last.x0; d.y0 = t.last.y0;
            d.x1          = t.last.x1; d.y1 = t.last.y1;
            d.pixels      = (uint16_t)(t.last.pixels > 65535 ? 65535 : t.last.pixels);
            d.fillPct     = t.last.fillPct;
            d.corePct     = t.last.corePct;
            d.fragments   = t.last.fragments;
            d.touchesEdge = t.last.touchesEdge;

            if ((int)d.confidence > bestConf) {
                bestConf      = d.confidence;
                out.bestColor = (uint8_t)c;
            }
        } else {
            d.confidence = 0;
            d.cx = d.cy = 0;
            d.x0 = d.y0 = d.x1 = d.y1 = 0;
            d.pixels = 0; d.fillPct = 0; d.corePct = 0;
            d.fragments = 0; d.touchesEdge = false;
        }
    }
}
