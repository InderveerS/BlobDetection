#include "autotune.h"
#include "hsv_convert.h"
#include "profiles.h"
#include "frame_config.h"
#include "color_lut.h"
#include "blob_detect.h"
#include <Arduino.h>
#include <algorithm>
#include <cstring>

// ================================================================
// SEARCH PRIORS — used only to decide which blob in frame we lock
// onto while tuning. NOT the saved detection range.
//
// "center" is the rough expected hue for that color; "halfWidth" is
// how far either side we'll consider a candidate pixel. Keep these
// generous enough to find the real object even if your camera's
// hue response is a bit off, but narrow enough that yellow tuning
// doesn't latch onto a warm-lit white background.
//
// NOTE: the prior also CLIPS what can be learned - a pixel outside
// center +/- halfWidth is never even considered, so an object whose true hue
// sits further off-centre yields a silently truncated range. tuneProcessFrame
// warns when the collected data is pressed against the prior boundary.
// ================================================================
struct ColorPrior { int center; int halfWidth; };
static const ColorPrior COLOR_PRIORS[COLOR_COUNT] = {
    /* RED    */ {  0, 20 },
    /* GREEN  */ { 60, 30 },
    /* PURPLE */ {145, 25 },
    /* YELLOW */ { 28, 15 },   // kept narrow on purpose - see README on yellow vs. white bg
};

// Low-saturation pixels are rejected while searching. This is the main defence
// against a warm-lit white/grey background reading as a yellowish hue - real
// plastic/fabric colour is almost always more saturated than that. It is also
// the effective floor on any saved sMin.
#define SEARCH_S_MIN           90
#define SEARCH_V_MIN           35
#define SEARCH_V_MAX          250
#define TUNE_MIN_COMPONENT_PX 250   // ignore tiny noise specks as "the blob"

// Padding applied to the percentile range. Hue needs a generous pad: RGB565's
// 5-bit red/blue quantise hue into multi-degree steps, so a tight pad produces
// ranges that miss pixels of the very object they were tuned on.
#define TUNE_PAD_H              4
#define TUNE_PAD_S             12
#define TUNE_PAD_V             15

// Percentiles rather than min/max: with absolute extremes, ONE specular
// highlight and ONE shadowed edge pixel would set the whole range, and the
// rolling median across frames couldn't rescue it because every sample would
// already be an extreme.
#define PCT_LO                  5
#define PCT_HI                 95

#define ROLL_N                  8   // rolling sample window size
#define LIVE_PRINT_INTERVAL_MS 250

// ---- circular hue helpers: rotate so the search window never straddles
// the 0/179 seam, so plain ordering works even for red. ----
static inline int shiftHue(int h, int center) {
    int sh = h - center + 90;
    while (sh < 0)   sh += 180;
    while (sh >= 180) sh -= 180;
    return sh;
}
static inline int unshiftHue(int shifted, int center) {
    int h = shifted + center - 90;
    while (h < 0)   h += 180;
    while (h >= 180) h -= 180;
    return h;
}

static bool     g_active = false;
static ColorId  g_color;
static uint32_t g_lastPrintMs = 0;
static bool     g_clipWarned  = false;

static int rbShLo[ROLL_N], rbShHi[ROLL_N];
static int rbSLo[ROLL_N],  rbSHi[ROLL_N];
static int rbVLo[ROLL_N],  rbVHi[ROLL_N];
static int rbCount = 0, rbNext = 0;

// Histograms over the winning component. Small enough to keep static, and far
// more robust than tracking extremes.
static uint32_t g_histH[180];
static uint32_t g_histS[256];
static uint32_t g_histV[256];

bool tuneIsActive() { return g_active; }

// The prior for the colour being tuned, expressed as an HsvRange so the shared
// LUT can do the candidate test.
static HsvRange priorRange(ColorId c) {
    const ColorPrior &p = COLOR_PRIORS[c];
    HsvRange r;
    r.hMin = ((p.center - p.halfWidth) % 180 + 180) % 180;
    r.hMax = ((p.center + p.halfWidth) % 180 + 180) % 180;
    r.sMin = SEARCH_S_MIN; r.sMax = 255;
    r.vMin = SEARCH_V_MIN; r.vMax = SEARCH_V_MAX;
    return r;
}

// Loads the search prior into the shared LUT, with every other colour disabled.
static void installPriorLut(ColorId c) {
    HsvRange profiles[COLOR_COUNT];
    for (int i = 0; i < COLOR_COUNT; i++) {
        if (i == (int)c) profiles[i] = priorRange(c);
        else {
            // sMin > sMax can never be satisfied, so this colour matches nothing.
            HsvRange none = {0, 0, 255, 0, 255, 0};
            profiles[i] = none;
        }
    }
    lutBuildCustom(profiles);
}

void tuneStart(ColorId c) {
    if (g_active) {
        Serial.printf("[TUNE] Switching from %s to %s -- previous session discarded.\n",
            colorName(g_color), colorName(c));
    }
    g_active = true;
    g_color = c;
    rbCount = 0; rbNext = 0;
    g_lastPrintMs = 0;
    g_clipWarned  = false;

    installPriorLut(c);

    Serial.printf("\n[TUNE] Started tuning %s on %s.\n", colorName(c), cameraName(getActiveCameraId()));
    Serial.println("       Hold the teletubby in view, anywhere in frame.");
    Serial.println("       Watch the live preview below; type 'save' once it looks stable,");
    Serial.println("       or 'cancel' to abort.\n");
}

static void endSession() {
    g_active = false;
    lutRebuild();   // restore the real detection table
}

void tuneCancel() {
    if (!g_active) { Serial.println("[TUNE] Not currently tuning."); return; }
    Serial.printf("[TUNE] Cancelled tuning %s. No changes made.\n", colorName(g_color));
    endSession();
}

// Value at the given percentile of a histogram.
static int percentileOf(const uint32_t *hist, int bins, uint32_t total, int pct) {
    if (total == 0) return 0;
    uint32_t target = (uint32_t)((uint64_t)total * (uint32_t)pct / 100u);
    uint32_t run = 0;
    for (int i = 0; i < bins; i++) {
        run += hist[i];
        if (run >= target) return i;
    }
    return bins - 1;
}

static int medianOf(const int *arr, int n) {
    int tmp[ROLL_N];
    for (int i = 0; i < n; i++) tmp[i] = arr[i];
    std::sort(tmp, tmp + n);
    return tmp[n / 2];
}

static HsvRange computeCandidateRange() {
    int shLo = medianOf(rbShLo, rbCount) - TUNE_PAD_H;
    int shHi = medianOf(rbShHi, rbCount) + TUNE_PAD_H;
    int sLo  = medianOf(rbSLo,  rbCount) - TUNE_PAD_S;
    int sHi  = medianOf(rbSHi,  rbCount) + TUNE_PAD_S;
    int vLo  = medianOf(rbVLo,  rbCount) - TUNE_PAD_V;
    int vHi  = medianOf(rbVHi,  rbCount) + TUNE_PAD_V;

    sLo = constrain(sLo, 0, 255); sHi = constrain(sHi, 0, 255);
    vLo = constrain(vLo, 0, 255); vHi = constrain(vHi, 0, 255);

    int center = COLOR_PRIORS[g_color].center;
    HsvRange r;
    r.hMin = unshiftHue(shLo, center);
    r.hMax = unshiftHue(shHi, center);
    r.sMin = sLo; r.sMax = sHi;
    r.vMin = vLo; r.vMax = vHi;
    return r;
}

void tuneProcessFrame(camera_fb_t *fb) {
    if (!g_active) return;

    const ColorPrior &prior = COLOR_PRIORS[g_color];
    const uint8_t    *buf   = fb->buf;
    const uint8_t    *lut   = lutData();

    // The LUT currently holds the search prior, so the scanner finds exactly
    // the pixels that fall inside it.
    BlobComponent *comps = blobScratch();
    int n = blobScan(buf, lut, fullFrameRoi(), comps, MAX_COMPONENTS, nullptr);

    int best = -1;
    for (int i = 0; i < n; i++)
        if (best < 0 || comps[i].pixels > comps[best].pixels) best = i;

    bool found = (best >= 0);
    uint32_t bestPx = found ? comps[best].pixels : 0;

    // A blob touching any frame edge is almost certainly bleeding into
    // background rather than showing a clean, isolated view of the toy --
    // don't let it pollute the calibration data.
    int edgeTouches = 0;
    if (found) {
        const BlobComponent &b = comps[best];
        edgeTouches = (b.x0 == 0) + (b.x1 == FRAME_W - 1) +
                      (b.y0 == 0) + (b.y1 == FRAME_H - 1);
    }

    bool clipped = false;

    if (found && bestPx >= TUNE_MIN_COMPONENT_PX && edgeTouches == 0) {
        // Second pass over the winning component's bounding box, building
        // histograms. Bounded and cheap - the bbox is a small part of a frame.
        const BlobComponent &b = comps[best];
        memset(g_histH, 0, sizeof(g_histH));
        memset(g_histS, 0, sizeof(g_histS));
        memset(g_histV, 0, sizeof(g_histV));
        uint32_t total = 0;

        for (int y = b.y0; y <= b.y1; y++) {
            const uint8_t *p = buf + ((size_t)y * FRAME_W + b.x0) * 2;
            for (int x = b.x0; x <= b.x1; x++, p += 2) {
                uint16_t px = ((uint16_t)p[0] << 8) | p[1];
                if (lutColorOf(lut[px]) == 0) continue;   // not a candidate
                int h, s, v;
                rgb565ToHsv(px, h, s, v);
                g_histH[shiftHue(h, prior.center)]++;
                g_histS[s]++;
                g_histV[v]++;
                total++;
            }
        }

        if (total > 0) {
            int shLo = percentileOf(g_histH, 180, total, PCT_LO);
            int shHi = percentileOf(g_histH, 180, total, PCT_HI);
            int sLo  = percentileOf(g_histS, 256, total, PCT_LO);
            int sHi  = percentileOf(g_histS, 256, total, PCT_HI);
            int vLo  = percentileOf(g_histV, 256, total, PCT_LO);
            int vHi  = percentileOf(g_histV, 256, total, PCT_HI);

            // If the measured hue is pressed against the prior window, the
            // prior is truncating what we can learn about this object.
            if (shLo <= (90 - prior.halfWidth) + 1 || shHi >= (90 + prior.halfWidth) - 1)
                clipped = true;

            rbShLo[rbNext] = shLo; rbShHi[rbNext] = shHi;
            rbSLo[rbNext]  = sLo;  rbSHi[rbNext]  = sHi;
            rbVLo[rbNext]  = vLo;  rbVHi[rbNext]  = vHi;
            rbNext = (rbNext + 1) % ROLL_N;
            if (rbCount < ROLL_N) rbCount++;
        }
    }

    if (clipped && !g_clipWarned) {
        g_clipWarned = true;
        Serial.printf("\n[TUNE] !! %s pixels are pressed against the search prior "
                      "(centre %d +/- %d).\n", colorName(g_color), prior.center, prior.halfWidth);
        Serial.println("       The saved range will be TRUNCATED. Widen halfWidth for this");
        Serial.println("       colour in COLOR_PRIORS (autotune.cpp) and re-tune.\n");
    }

    uint32_t now = millis();
    if (now - g_lastPrintMs < LIVE_PRINT_INTERVAL_MS) return;
    g_lastPrintMs = now;

    if (!found || bestPx < TUNE_MIN_COMPONENT_PX) {
        Serial.printf("[TUNE %s] no blob found yet (best %u px) -- reposition / check lighting\n",
            colorName(g_color), (unsigned)bestPx);
        return;
    }

    const BlobComponent &b = comps[best];
    const char *warn = (edgeTouches > 0)
        ? "  [WARN: blob touches frame edge -- excluded from calibration, likely background]" : "";

    Serial.printf("[TUNE %s] px:%-5u  bbox:(%d,%d)-(%d,%d)%s\n",
        colorName(g_color), (unsigned)b.pixels, b.x0, b.y0, b.x1, b.y1, warn);

    if (rbCount > 0) {
        HsvRange preview = computeCandidateRange();
        Serial.print("           would save -> ");
        printRange(preview);
        Serial.printf("           (%d/%d samples, %d-%dth percentile)\n",
                      rbCount, ROLL_N, PCT_LO, PCT_HI);
    }
}

void tuneSave() {
    if (!g_active) { Serial.println("[TUNE] Not currently tuning. Use 'tune <color>' first."); return; }
    if (rbCount == 0) {
        Serial.println("[TUNE] No valid samples yet -- get a blob found before saving.");
        return;
    }
    HsvRange r = computeCandidateRange();
    ColorId  c = g_color;

    endSession();                  // restore the detection LUT first...
    setColorProfile(c, r);         // ...then apply, which rebuilds it with the new range

    Serial.printf("[TUNE] Saved %s: ", colorName(c));
    printRange(r);
    Serial.println("       In RAM only. Use 'nvs save' to keep it across reboots,");
    Serial.println("       or 'dump' and paste into profiles_defaults.h to bake it in.");
}
