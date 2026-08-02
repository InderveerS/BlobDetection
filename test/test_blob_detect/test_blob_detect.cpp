// Host-side tests for the connected-component scanner and the colour LUT.
//   pio test -e native
//
// These use a hand-built LUT rather than the real profiles: the point is to
// test the CCA geometry with unambiguous inputs, so "pixel value 1 is red,
// pixel value 2 is green, everything else is background" keeps the test cases
// readable.

#include <unity.h>
#include <string.h>
#include <stdlib.h>
#include "blob_detect.h"
#include "color_lut.h"
#include "frame_config.h"
#include "hsv_convert.h"

static uint8_t *g_frame = nullptr;   // FRAME_BYTES
static uint8_t *g_lut   = nullptr;   // LUT_SIZE

// Distinct RGB565 values used as paint. Values are arbitrary; the test LUT
// maps them explicitly.
#define PX_BG    0x0000
#define PX_RED   0xF800
#define PX_GREEN 0x07E0

static void clearFrame() {
    for (size_t i = 0; i < FRAME_PIXELS; i++) {
        g_frame[i * 2]     = (uint8_t)(PX_BG >> 8);
        g_frame[i * 2 + 1] = (uint8_t)(PX_BG & 0xFF);
    }
}

static void setPx(int x, int y, uint16_t v) {
    if (x < 0 || x >= FRAME_W || y < 0 || y >= FRAME_H) return;
    size_t i = ((size_t)y * FRAME_W + x) * 2;
    g_frame[i]     = (uint8_t)(v >> 8);
    g_frame[i + 1] = (uint8_t)(v & 0xFF);
}

static void fillRect(int x0, int y0, int x1, int y1, uint16_t v) {
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            setPx(x, y, v);
}

static int scan(BlobComponent *out, int maxOut, BlobScanStats *st) {
    return blobScan(g_frame, g_lut, fullFrameRoi(), out, maxOut, st);
}

void setUp(void) {
    clearFrame();
    setMinRunLength(3);
}
void tearDown(void) {}

// ---------------------------------------------------------------
// Two separate blobs of the same colour must stay separate. This is the case
// the old frame-wide-average detector got wrong: it reported one centroid in
// the empty space between them.
// ---------------------------------------------------------------
static void test_two_blobs_stay_separate(void) {
    fillRect(20, 20, 39, 39, PX_RED);      // 20x20 at (20,20), centre (29.5, 29.5)
    fillRect(200, 150, 219, 169, PX_RED);  // 20x20 at (200,150)

    BlobComponent c[16];
    BlobScanStats st;
    int n = scan(c, 16, &st);

    TEST_ASSERT_EQUAL_INT(2, n);
    TEST_ASSERT_EQUAL_UINT32(400, c[0].pixels);
    TEST_ASSERT_EQUAL_UINT32(400, c[1].pixels);

    // Centroids land inside their own blob, never between them.
    TEST_ASSERT_INT_WITHIN(1, 29,  c[0].cx);
    TEST_ASSERT_INT_WITHIN(1, 29,  c[0].cy);
    TEST_ASSERT_INT_WITHIN(1, 209, c[1].cx);
    TEST_ASSERT_INT_WITHIN(1, 159, c[1].cy);
    TEST_ASSERT_FALSE(st.labelCapHit || st.runCapHit || st.outCapHit);
}

// ---------------------------------------------------------------
// Different colours must never merge, even when adjacent.
// ---------------------------------------------------------------
static void test_adjacent_different_colours_do_not_merge(void) {
    fillRect(50, 50, 69, 69, PX_RED);
    fillRect(70, 50, 89, 69, PX_GREEN);   // directly touching

    BlobComponent c[16];
    int n = scan(c, 16, nullptr);

    TEST_ASSERT_EQUAL_INT(2, n);
    TEST_ASSERT_TRUE(c[0].color != c[1].color);
    TEST_ASSERT_EQUAL_UINT32(400, c[0].pixels);
    TEST_ASSERT_EQUAL_UINT32(400, c[1].pixels);
}

// ---------------------------------------------------------------
// A target behind a rock arrives as several fragments. At this stage they are
// still separate components (merging them is the filter's job); what matters
// here is that all the pixels survive and nothing is lost.
// ---------------------------------------------------------------
static void test_occluded_blob_yields_fragments(void) {
    fillRect(100, 100, 139, 119, PX_RED);   // one 40x20 blob
    fillRect(112, 100, 115, 119, PX_BG);    // two vertical occlusion bars
    fillRect(126, 100, 129, 119, PX_BG);

    BlobComponent c[16];
    int n = scan(c, 16, nullptr);

    TEST_ASSERT_EQUAL_INT(3, n);
    uint32_t total = 0;
    for (int i = 0; i < n; i++) total += c[i].pixels;
    TEST_ASSERT_EQUAL_UINT32(40u * 20u - 2u * 4u * 20u, total);
}

// ---------------------------------------------------------------
// Salt-and-pepper noise must never masquerade as a target.
//
// Note what this does NOT assert: that zero components survive. At 4000 random
// pixels in 76800 (5.2% density) the expected number of accidental 3-in-a-row
// runs is 76800 * 0.052^3 ~= 11, and that is what actually comes out. Run-length
// filtering alone cannot clear dense noise - it only guarantees the survivors
// are *tiny*. The area gate is what finishes the job, which is exactly why the
// filter stage has one. Asserting "0 components" here would be asserting
// something false about the algorithm.
// ---------------------------------------------------------------
static void test_noise_never_looks_like_a_target(void) {
    srand(12345);
    for (int i = 0; i < 4000; i++)
        setPx(rand() % FRAME_W, rand() % FRAME_H, PX_RED);

    BlobComponent c[MAX_COMPONENTS];
    BlobScanStats st;
    setMinRunLength(3);
    int n = scan(c, MAX_COMPONENTS, &st);

    // Whatever survives must be small enough that a modest area gate removes it.
    uint32_t largest = 0;
    for (int i = 0; i < n; i++)
        if (c[i].pixels > largest) largest = c[i].pixels;
    TEST_ASSERT_LESS_THAN_UINT32(30, largest);

    // With a realistic minimum area, nothing at all gets through.
    int survivors = 0;
    for (int i = 0; i < n; i++) if (c[i].pixels >= 50) survivors++;
    TEST_ASSERT_EQUAL_INT(0, survivors);

    TEST_ASSERT_FALSE(st.labelCapHit || st.runCapHit || st.outCapHit);
}

// ---------------------------------------------------------------
// A run of exactly minRunLength survives; one pixel shorter does not.
// Pins the boundary so the erosion can't silently drift.
// ---------------------------------------------------------------
static void test_min_run_length_boundary(void) {
    setMinRunLength(4);
    fillRect(10, 10, 13, 10, PX_RED);   // 4 wide - keep
    fillRect(10, 30, 12, 30, PX_RED);   // 3 wide - drop

    BlobComponent c[16];
    int n = scan(c, 16, nullptr);
    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_EQUAL_UINT32(4, c[0].pixels);
    TEST_ASSERT_EQUAL_INT(10, c[0].y0);
}

// ---------------------------------------------------------------
// Union-find stress: a comb forces many provisional labels on the top row that
// must all collapse into ONE component when the spine joins them. This is the
// case a naive single-pass labeller gets wrong.
// ---------------------------------------------------------------
static void test_comb_collapses_to_one_component(void) {
    const int teeth = 40;
    for (int t = 0; t < teeth; t++) {
        int x = 10 + t * 6;
        fillRect(x, 20, x + 3, 60, PX_RED);    // vertical tooth
    }
    fillRect(10, 60, 10 + teeth * 6, 63, PX_RED);   // spine joins them all

    BlobComponent c[MAX_COMPONENTS];
    BlobScanStats st;
    int n = scan(c, MAX_COMPONENTS, &st);

    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_FALSE(st.labelCapHit);
}

// ---------------------------------------------------------------
// 8-connectivity: a one-pixel diagonal step must keep a shape connected.
// 4-connectivity would split this into two components.
// ---------------------------------------------------------------
static void test_diagonal_link_is_connected(void) {
    fillRect(10, 10, 19, 12, PX_RED);
    fillRect(20, 13, 29, 15, PX_RED);   // starts one row down, one col right

    BlobComponent c[16];
    int n = scan(c, 16, nullptr);
    TEST_ASSERT_EQUAL_INT(1, n);
}

// ---------------------------------------------------------------
// ROI must exclude everything outside it. This is the cheapest defence against
// a same-coloured object in the background (a person's shirt above the course).
// ---------------------------------------------------------------
static void test_roi_excludes_outside(void) {
    fillRect(100, 10,  139, 40,  PX_RED);   // "background person", upper frame
    fillRect(100, 150, 139, 180, PX_RED);   // the actual target, lower frame

    BlobComponent c[16];
    ScanRoi roi = { 0, 120, FRAME_W - 1, FRAME_H - 1 };
    int n = blobScan(g_frame, g_lut, roi, c, 16, nullptr);

    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_GREATER_THAN_INT(120, c[0].cy);
}

// ---------------------------------------------------------------
// Blobs clipped by the ROI edge still report, with geometry clamped to the ROI.
// ---------------------------------------------------------------
static void test_blob_clipped_by_roi_edge(void) {
    fillRect(100, 100, 139, 200, PX_RED);

    BlobComponent c[16];
    ScanRoi roi = { 0, 120, FRAME_W - 1, 160 };
    int n = blobScan(g_frame, g_lut, roi, c, 16, nullptr);

    TEST_ASSERT_EQUAL_INT(1, n);
    TEST_ASSERT_EQUAL_INT(120, c[0].y0);
    TEST_ASSERT_EQUAL_INT(160, c[0].y1);
    TEST_ASSERT_EQUAL_UINT32(40u * 41u, c[0].pixels);
}

// ---------------------------------------------------------------
// An empty frame produces nothing and doesn't trip any cap.
// ---------------------------------------------------------------
static void test_empty_frame(void) {
    BlobComponent c[16];
    BlobScanStats st;
    int n = scan(c, 16, &st);
    TEST_ASSERT_EQUAL_INT(0, n);
    TEST_ASSERT_EQUAL_UINT32(0, st.runsTotal);
    TEST_ASSERT_FALSE(st.labelCapHit || st.runCapHit || st.outCapHit);
}

// ---------------------------------------------------------------
// The real LUT builder: the four colours must come out mutually exclusive, and
// a neutral grey must match nothing. Grey mattering here is the bit-replication
// fix - before it, white converted to S=4/H=120 and read as green.
// ---------------------------------------------------------------
static void test_lut_colours_are_mutually_exclusive(void) {
    HsvRange profiles[COLOR_COUNT] = {
        /* RED    */ {165,  10,  120, 255,   70, 255},
        /* GREEN  */ { 45,  80,   80, 255,   60, 255},
        /* PURPLE */ {130, 160,   80, 255,   50, 255},
        /* YELLOW */ { 20,  35,  110, 255,  100, 255},
    };
    uint8_t *lut = (uint8_t *)malloc(LUT_SIZE);
    TEST_ASSERT_NOT_NULL(lut);
    lutBuild(lut, profiles);

    // Every entry names at most one colour, by construction of the byte layout.
    for (uint32_t px = 0; px < LUT_SIZE; px++)
        TEST_ASSERT_LESS_OR_EQUAL_UINT8(COLOR_COUNT, lutColorOf(lut[px]));

    // Neutrals must be unclassified: pure white, pure black, mid grey.
    TEST_ASSERT_EQUAL_UINT8(0, lutColorOf(lut[0xFFFF]));
    TEST_ASSERT_EQUAL_UINT8(0, lutColorOf(lut[0x0000]));
    TEST_ASSERT_EQUAL_UINT8(0, lutColorOf(lut[0x8410]));   // ~50% grey

    // A saturated primary must land on a colour, and be flagged core.
    TEST_ASSERT_EQUAL_UINT8(COLOR_RED + 1, lutColorOf(lut[0xF800]));
    TEST_ASSERT_TRUE(lutIsCore(lut[0xF800]));

    free(lut);
}

// ---------------------------------------------------------------
// White must convert to zero saturation. Guards the RGB565->888 bit
// replication: a plain <<3 leaves white at (248,252,248) -> S=4, H=120.
// ---------------------------------------------------------------
static void test_white_has_zero_saturation(void) {
    int h, s, v;
    rgb565ToHsv(0xFFFF, h, s, v);
    TEST_ASSERT_EQUAL_INT(0, s);
    TEST_ASSERT_EQUAL_INT(255, v);
}

void registerFilterTrackerTests(void);   // test_filter_tracker.cpp

int main(int, char **) {
    g_frame = (uint8_t *)malloc(FRAME_BYTES);
    g_lut   = (uint8_t *)calloc(LUT_SIZE, 1);
    if (!g_frame || !g_lut) return 1;

    // Test LUT: two unambiguous colours, everything else background.
    g_lut[PX_RED]   = (uint8_t)(COLOR_RED   + 1) | LUT_CORE_BIT;
    g_lut[PX_GREEN] = (uint8_t)(COLOR_GREEN + 1) | LUT_CORE_BIT;

    UNITY_BEGIN();
    RUN_TEST(test_two_blobs_stay_separate);
    RUN_TEST(test_adjacent_different_colours_do_not_merge);
    RUN_TEST(test_occluded_blob_yields_fragments);
    RUN_TEST(test_noise_never_looks_like_a_target);
    RUN_TEST(test_min_run_length_boundary);
    RUN_TEST(test_comb_collapses_to_one_component);
    RUN_TEST(test_diagonal_link_is_connected);
    RUN_TEST(test_roi_excludes_outside);
    RUN_TEST(test_blob_clipped_by_roi_edge);
    RUN_TEST(test_empty_frame);
    RUN_TEST(test_lut_colours_are_mutually_exclusive);
    RUN_TEST(test_white_has_zero_saturation);
    registerFilterTrackerTests();
    int rc = UNITY_END();

    free(g_frame);
    free(g_lut);
    return rc;
}
