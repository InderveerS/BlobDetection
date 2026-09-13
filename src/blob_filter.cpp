#include "blob_filter.h"
#include "frame_config.h"

// Ballpark starting points. They want setting at the venue against the real
// course - `limits` and `fieldcheck` exist for exactly that.
#define DEF_MIN_AREA        300u
#define DEF_MAX_AREA      25000u     // ~1/3 of a QVGA frame: a wall or a person up close
#define DEF_MIN_FILL_PCT     35
#define DEF_MIN_ASPECT      30       // 0.30 w/h
#define DEF_MAX_ASPECT     300       // 3.00 w/h
#define DEF_MIN_CORE_PCT     25
#define DEF_MERGE_GAP        12
#define DEF_MIN_FRAGMENT     15

static ColorLimits g_limits[COLOR_COUNT];
static ScanRoi     g_roi;
static int         g_mergeGap      = DEF_MERGE_GAP;
static int         g_minFragmentPx = DEF_MIN_FRAGMENT;

const char* rejectReasonName(int r) {
    switch (r) {
        case REJ_NONE:     return "ok";
        case REJ_MIN_AREA: return "too-small";
        case REJ_MAX_AREA: return "too-big";
        case REJ_FILL:     return "low-fill";
        case REJ_ASPECT:   return "bad-aspect";
        case REJ_CORE:     return "edge-of-range";
        default:           return "?";
    }
}

void filterSetDefaults() {
    for (int c = 0; c < COLOR_COUNT; c++) {
        g_limits[c].minArea       = DEF_MIN_AREA;
        g_limits[c].maxArea       = DEF_MAX_AREA;
        g_limits[c].minFillPct    = DEF_MIN_FILL_PCT;
        g_limits[c].minAspectX100 = DEF_MIN_ASPECT;
        g_limits[c].maxAspectX100 = DEF_MAX_ASPECT;
        g_limits[c].minCorePct    = DEF_MIN_CORE_PCT;
    }
    g_roi           = fullFrameRoi();
    g_mergeGap      = DEF_MERGE_GAP;
    g_minFragmentPx = DEF_MIN_FRAGMENT;
}

ColorLimits& colorLimits(ColorId c) { return g_limits[c]; }

void    setScanRoi(const ScanRoi &r) { g_roi = r; }
ScanRoi getScanRoi()                 { return g_roi; }

void setMergeGap(int px) { g_mergeGap = px < 0 ? 0 : px; }
int  getMergeGap()       { return g_mergeGap; }

void setMinFragmentPx(int px) { g_minFragmentPx = px < 1 ? 1 : px; }
int  getMinFragmentPx()       { return g_minFragmentPx; }

// ---- fragment merging ----

static inline bool bboxNear(const BlobComponent &a, const BlobComponent &b, int gap) {
    return !(a.x1 + gap < b.x0 || b.x1 + gap < a.x0 ||
             a.y1 + gap < b.y0 || b.y1 + gap < a.y0);
}

static void mergeInto(BlobComponent &dst, const BlobComponent &src) {
    uint32_t total = dst.pixels + src.pixels;
    // Merging centroids as a pixel-weighted average is exact: each centroid is
    // sum/count, so the combined centroid is (c1*n1 + c2*n2)/(n1+n2).
    dst.cx = (int16_t)(((int32_t)dst.cx * (int32_t)dst.pixels +
                        (int32_t)src.cx * (int32_t)src.pixels) / (int32_t)total);
    dst.cy = (int16_t)(((int32_t)dst.cy * (int32_t)dst.pixels +
                        (int32_t)src.cy * (int32_t)src.pixels) / (int32_t)total);
    dst.pixels      = total;
    dst.corePixels += src.corePixels;
    if (src.x0 < dst.x0) dst.x0 = src.x0;
    if (src.y0 < dst.y0) dst.y0 = src.y0;
    if (src.x1 > dst.x1) dst.x1 = src.x1;
    if (src.y1 > dst.y1) dst.y1 = src.y1;
}

// Merges same-colour fragments whose bounding boxes come within `gap`.
//
// There is exactly one teletubby of each colour, so this can be aggressive:
// unlike the general case, there is no risk of fusing two distinct targets.
// The only thing it can wrongly absorb is a same-coloured background object
// that happens to sit adjacent to the real one, which the ROI already handles.
//
// O(n^2) per pass, but n is small: sub-minFragmentPx specks are dropped first,
// so only real fragments reach here.
static int mergeFragments(BlobComponent *comps, int n, int gap, uint8_t *fragCount) {
    for (int i = 0; i < n; i++) fragCount[i] = 1;
    if (gap <= 0) return n;

    bool changed = true;
    int  passes  = 0;
    while (changed && passes++ < 4) {
        changed = false;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; ) {
                if (comps[i].color == comps[j].color && bboxNear(comps[i], comps[j], gap)) {
                    mergeInto(comps[i], comps[j]);
                    fragCount[i] = (uint8_t)(fragCount[i] + fragCount[j]);
                    n--;
                    comps[j]     = comps[n];      // swap-remove
                    fragCount[j] = fragCount[n];
                    changed = true;
                } else {
                    j++;
                }
            }
        }
    }
    return n;
}

// ---- gating and scoring ----

static int gateCandidate(const BlobComponent &b, const ColorLimits &lim) {
    if (b.pixels < lim.minArea) return REJ_MIN_AREA;
    if (b.pixels > lim.maxArea) return REJ_MAX_AREA;

    int      w        = b.x1 - b.x0 + 1;
    int      h        = b.y1 - b.y0 + 1;
    uint32_t bboxArea = (uint32_t)w * (uint32_t)h;
    if (w <= 0 || h <= 0 || bboxArea == 0) return REJ_MIN_AREA;

    if (100u * b.pixels / bboxArea < lim.minFillPct) return REJ_FILL;

    int aspect = 100 * w / h;
    if (aspect < lim.minAspectX100 || aspect > lim.maxAspectX100) return REJ_ASPECT;

    if (100u * b.corePixels / b.pixels < lim.minCorePct) return REJ_CORE;

    return REJ_NONE;
}

static uint8_t scoreCandidate(const BlobComponent &b, const ColorLimits &lim, bool edge) {
    int      w        = b.x1 - b.x0 + 1;
    int      h        = b.y1 - b.y0 + 1;
    uint32_t bboxArea = (uint32_t)w * (uint32_t)h;

    int fillPct = bboxArea ? (int)(100u * b.pixels / bboxArea) : 0;
    int corePct = b.pixels ? (int)(100u * b.corePixels / b.pixels) : 0;

    // Fill: at the gate threshold this scores 0, a perfectly solid blob 100.
    int fillScore = 0;
    if (fillPct > lim.minFillPct && lim.minFillPct < 100)
        fillScore = 100 * (fillPct - lim.minFillPct) / (100 - lim.minFillPct);

    // Area: full marks in the middle of the plausible band, tapering towards
    // both edges. Sitting right at the limit is weak evidence either way.
    int areaScore = 0;
    if (b.pixels >= lim.minArea && b.pixels <= lim.maxArea) {
        uint32_t lo  = lim.minArea;
        uint32_t hi  = lim.maxArea;
        uint32_t mid = lo + (hi - lo) / 3;      // targets cluster nearer the low end
        if (b.pixels <= mid) {
            uint32_t span = (mid > lo) ? (mid - lo) : 1;
            areaScore = 50 + (int)(50u * (b.pixels - lo) / span);
        } else {
            uint32_t span = (hi > mid) ? (hi - mid) : 1;
            areaScore = 100 - (int)(50u * (b.pixels - mid) / span);
        }
        if (areaScore < 0)   areaScore = 0;
        if (areaScore > 100) areaScore = 100;
    }

    // Core fraction carries the most weight: it is the cleanest separator
    // between a real object and something that merely clips the range edge.
    int s = (corePct * 45 + fillScore * 25 + areaScore * 30) / 100;

    // Partially out of frame is still a real target - just less certain,
    // because area and fill are both understated when clipped.
    if (edge) s = s * 75 / 100;

    if (s < 0)   s = 0;
    if (s > 100) s = 100;
    return (uint8_t)(s * 255 / 100);
}

void blobFilter(BlobComponent *comps, int n, FilterResult &out) {
    for (int c = 0; c < COLOR_COUNT; c++) {
        out.color[c].valid      = false;
        out.color[c].confidence = 0;
        out.color[c].fragments  = 0;
    }
    for (int r = 0; r < REJ_REASON_COUNT; r++) { out.rejected[r] = 0; out.rejectedMaxPx[r] = 0; }
    out.mergedAway = 0;

    // Drop specks before merging: they cannot be a target on their own, and
    // letting them into the merge would drag bounding boxes outwards.
    int kept = 0;
    for (int i = 0; i < n; i++)
        if (comps[i].pixels >= (uint32_t)g_minFragmentPx) comps[kept++] = comps[i];

    static uint8_t fragCount[MAX_COMPONENTS];
    int m = mergeFragments(comps, kept, g_mergeGap, fragCount);
    out.mergedAway = (uint16_t)(kept - m);

    const ScanRoi &roi = g_roi;

    for (int i = 0; i < m; i++) {
        BlobComponent &b = comps[i];
        int c = (int)b.color - 1;
        if (c < 0 || c >= COLOR_COUNT) continue;

        const ColorLimits &lim = g_limits[c];
        int reason = gateCandidate(b, lim);
        if (reason != REJ_NONE) {
            out.rejected[reason]++;
            if (b.pixels > out.rejectedMaxPx[reason]) out.rejectedMaxPx[reason] = b.pixels;
            continue;
        }

        bool edge = (b.x0 <= roi.x0) || (b.y0 <= roi.y0) ||
                    (b.x1 >= roi.x1) || (b.y1 >= roi.y1);
        uint8_t conf = scoreCandidate(b, lim, edge);

        // Highest confidence wins, not largest area: a background object can
        // easily be the bigger blob.
        ColorCandidate &cand = out.color[c];
        if (cand.valid && conf <= cand.confidence) continue;

        uint32_t bboxArea = (uint32_t)(b.x1 - b.x0 + 1) * (uint32_t)(b.y1 - b.y0 + 1);
        cand.valid       = true;
        cand.confidence  = conf;
        cand.cx = b.cx;  cand.cy = b.cy;
        cand.x0 = b.x0;  cand.y0 = b.y0;
        cand.x1 = b.x1;  cand.y1 = b.y1;
        cand.pixels      = b.pixels;
        cand.fillPct     = (uint8_t)(bboxArea ? 100u * b.pixels / bboxArea : 0);
        cand.corePct     = (uint8_t)(b.pixels ? 100u * b.corePixels / b.pixels : 0);
        cand.fragments   = fragCount[i];
        cand.touchesEdge = edge;
    }
}
