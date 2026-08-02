// Tests for the false-positive filter and the temporal tracker.
// Built and run by scripts/run_native_tests.ps1 alongside test_blob_detect.cpp.

#include <unity.h>
#include <string.h>
#include "blob_filter.h"
#include "tracker.h"
#include "detect_result.h"
#include "frame_config.h"

// Builds a solid rectangular component: fill 100%, core 100% unless told
// otherwise. Represents an ideal detection that should sail through the gates.
static BlobComponent mkComp(ColorId col, int x0, int y0, int w, int h,
                            int fillPct = 100, int corePct = 100) {
    BlobComponent b;
    b.color      = (uint8_t)(col + 1);
    b.x0 = (int16_t)x0;         b.y0 = (int16_t)y0;
    b.x1 = (int16_t)(x0+w-1);   b.y1 = (int16_t)(y0+h-1);
    uint32_t bbox = (uint32_t)w * (uint32_t)h;
    b.pixels     = bbox * (uint32_t)fillPct / 100u;
    b.corePixels = b.pixels * (uint32_t)corePct / 100u;
    b.cx = (int16_t)(x0 + w / 2);
    b.cy = (int16_t)(y0 + h / 2);
    return b;
}

void setUp_filter(void) {
    filterSetDefaults();
    trackerSetDefaults();
}

// ---------------------------------------------------------------
// A well-formed blob of plausible size is accepted.
// ---------------------------------------------------------------
static void test_good_blob_accepted(void) {
    setUp_filter();
    BlobComponent c[1] = { mkComp(COLOR_RED, 100, 100, 40, 30) };
    FilterResult f;
    blobFilter(c, 1, f);

    TEST_ASSERT_TRUE(f.color[COLOR_RED].valid);
    TEST_ASSERT_GREATER_THAN_UINT8(100, f.color[COLOR_RED].confidence);
    TEST_ASSERT_EQUAL_UINT8(100, f.color[COLOR_RED].fillPct);
}

// ---------------------------------------------------------------
// A person's t-shirt in the background: right colour, but far too large.
// The area band is what stops it.
// ---------------------------------------------------------------
static void test_oversized_object_rejected(void) {
    setUp_filter();
    BlobComponent c[1] = { mkComp(COLOR_RED, 10, 10, 300, 200) };
    FilterResult f;
    blobFilter(c, 1, f);

    TEST_ASSERT_FALSE(f.color[COLOR_RED].valid);
    TEST_ASSERT_EQUAL_UINT16(1, f.rejected[REJ_MAX_AREA]);
}

// ---------------------------------------------------------------
// A diffuse wash - a lit wall reading as a colour - covers a big bounding box
// but is sparse inside it. The fill gate is what stops it.
// ---------------------------------------------------------------
static void test_diffuse_wash_rejected_by_fill(void) {
    setUp_filter();
    BlobComponent c[1] = { mkComp(COLOR_YELLOW, 20, 20, 120, 100, /*fill*/ 12) };
    FilterResult f;
    blobFilter(c, 1, f);

    TEST_ASSERT_FALSE(f.color[COLOR_YELLOW].valid);
    TEST_ASSERT_EQUAL_UINT16(1, f.rejected[REJ_FILL]);
}

// ---------------------------------------------------------------
// A blob that only clips the edge of the profile's range - the signature of a
// marginal background match rather than a real object.
// ---------------------------------------------------------------
static void test_edge_of_range_rejected_by_core(void) {
    setUp_filter();
    BlobComponent c[1] = { mkComp(COLOR_GREEN, 60, 60, 40, 40, 100, /*core*/ 5) };
    FilterResult f;
    blobFilter(c, 1, f);

    TEST_ASSERT_FALSE(f.color[COLOR_GREEN].valid);
    TEST_ASSERT_EQUAL_UINT16(1, f.rejected[REJ_CORE]);
}

// ---------------------------------------------------------------
// Occlusion: a target split into three pieces by rocks must come back as ONE
// candidate whose bounding box spans all of them.
// ---------------------------------------------------------------
static void test_fragments_merge_back_into_one(void) {
    setUp_filter();
    setMergeGap(12);
    BlobComponent c[3] = {
        mkComp(COLOR_PURPLE, 100, 100, 20, 30),
        mkComp(COLOR_PURPLE, 126, 100, 18, 30),   // 6 px gap
        mkComp(COLOR_PURPLE, 150, 100, 20, 30),   // 6 px gap
    };
    FilterResult f;
    blobFilter(c, 3, f);

    const ColorCandidate &p = f.color[COLOR_PURPLE];
    TEST_ASSERT_TRUE(p.valid);
    TEST_ASSERT_EQUAL_UINT8(3, p.fragments);
    TEST_ASSERT_EQUAL_INT(100, p.x0);
    TEST_ASSERT_EQUAL_INT(169, p.x1);
    TEST_ASSERT_EQUAL_UINT16(2, f.mergedAway);
}

// ---------------------------------------------------------------
// merge 0 disables merging: the same fragments stay separate. This is the
// escape hatch if merging ever misbehaves at the venue.
// ---------------------------------------------------------------
static void test_merge_gap_zero_disables_merging(void) {
    setUp_filter();
    setMergeGap(0);
    colorLimits(COLOR_PURPLE).minArea = 100;
    BlobComponent c[2] = {
        mkComp(COLOR_PURPLE, 100, 100, 20, 30),
        mkComp(COLOR_PURPLE, 126, 100, 18, 30),
    };
    FilterResult f;
    blobFilter(c, 2, f);

    TEST_ASSERT_EQUAL_UINT16(0, f.mergedAway);
    TEST_ASSERT_TRUE(f.color[COLOR_PURPLE].valid);
    TEST_ASSERT_EQUAL_UINT8(1, f.color[COLOR_PURPLE].fragments);
}

// ---------------------------------------------------------------
// Fragments far apart are NOT merged - two separate things stay separate.
// ---------------------------------------------------------------
static void test_distant_fragments_not_merged(void) {
    setUp_filter();
    setMergeGap(12);
    BlobComponent c[2] = {
        mkComp(COLOR_RED, 10,  100, 30, 30),
        mkComp(COLOR_RED, 250, 100, 30, 30),
    };
    FilterResult f;
    blobFilter(c, 2, f);

    TEST_ASSERT_EQUAL_UINT16(0, f.mergedAway);
    TEST_ASSERT_EQUAL_UINT8(1, f.color[COLOR_RED].fragments);
}

// ---------------------------------------------------------------
// When two candidates of one colour both pass the gates, the higher-confidence
// one wins - NOT simply the larger. A background object is often the bigger
// blob, which is exactly the case this protects against.
// ---------------------------------------------------------------
static void test_highest_confidence_wins_not_largest(void) {
    setUp_filter();
    setMergeGap(0);
    BlobComponent c[2] = {
        mkComp(COLOR_RED, 10,  10,  90, 90, /*fill*/ 40, /*core*/ 30),  // big, marginal
        mkComp(COLOR_RED, 200, 150, 30, 30, /*fill*/100, /*core*/100),  // small, clean
    };
    FilterResult f;
    blobFilter(c, 2, f);

    TEST_ASSERT_TRUE(f.color[COLOR_RED].valid);
    TEST_ASSERT_EQUAL_INT(215, f.color[COLOR_RED].cx);   // the clean one
}

// ---------------------------------------------------------------
// Tracker: a single frame's sighting must NOT confirm. This is what stops a
// one-frame flash of colour from ever reaching the robot.
// ---------------------------------------------------------------
static void test_single_frame_does_not_confirm(void) {
    setUp_filter();
    BlobComponent c[1] = { mkComp(COLOR_RED, 100, 100, 40, 30) };
    FilterResult f;
    DetectResult r;

    blobFilter(c, 1, f);
    trackerUpdate(f, 1, r);
    TEST_ASSERT_FALSE(r.color[COLOR_RED].found);
    TEST_ASSERT_EQUAL_UINT8(NO_COLOR, r.bestColor);
}

// ---------------------------------------------------------------
// Tracker: three consecutive sightings confirm (default 3-of-5).
// ---------------------------------------------------------------
static void test_three_frames_confirm(void) {
    setUp_filter();
    BlobComponent c[1] = { mkComp(COLOR_RED, 100, 100, 40, 30) };
    FilterResult f;
    DetectResult r;

    for (int i = 1; i <= 3; i++) {
        BlobComponent tmp[1] = { c[0] };
        blobFilter(tmp, 1, f);
        trackerUpdate(f, i, r);
    }
    TEST_ASSERT_TRUE(r.color[COLOR_RED].found);
    TEST_ASSERT_EQUAL_UINT8(COLOR_RED, r.bestColor);
}

// ---------------------------------------------------------------
// Tracker: a confirmed target that disappears behind a rock must STAY
// confirmed across a short dropout. This is the asymmetric hysteresis - the
// robot must not give up on a target it just saw.
// ---------------------------------------------------------------
static void test_confirmed_survives_brief_occlusion(void) {
    setUp_filter();
    BlobComponent c[1] = { mkComp(COLOR_RED, 100, 100, 40, 30) };
    FilterResult f;
    DetectResult r;
    uint32_t frame = 0;

    for (int i = 0; i < 5; i++) {
        BlobComponent tmp[1] = { c[0] };
        blobFilter(tmp, 1, f);
        trackerUpdate(f, ++frame, r);
    }
    TEST_ASSERT_TRUE(r.color[COLOR_RED].found);

    // Fully hidden for 8 frames (~0.25 s at 30 fps) - still found.
    for (int i = 0; i < 8; i++) {
        blobFilter(nullptr, 0, f);
        trackerUpdate(f, ++frame, r);
        TEST_ASSERT_TRUE(r.color[COLOR_RED].found);
    }

    // Gone long enough (past maxMiss = 10) - now dropped.
    for (int i = 0; i < 6; i++) {
        blobFilter(nullptr, 0, f);
        trackerUpdate(f, ++frame, r);
    }
    TEST_ASSERT_FALSE(r.color[COLOR_RED].found);
}

// ---------------------------------------------------------------
// Tracker: intermittent noise that never sustains M-of-N never confirms.
// Alternating seen/unseen gives at most 3 of 5 ... so use 1-in-3, which is 2/5.
// ---------------------------------------------------------------
static void test_intermittent_noise_never_confirms(void) {
    setUp_filter();
    FilterResult f;
    DetectResult r;

    for (int i = 0; i < 30; i++) {
        if (i % 3 == 0) {
            BlobComponent c[1] = { mkComp(COLOR_GREEN, 50, 50, 40, 30) };
            blobFilter(c, 1, f);
        } else {
            blobFilter(nullptr, 0, f);
        }
        trackerUpdate(f, (uint32_t)i, r);
        TEST_ASSERT_FALSE(r.color[COLOR_GREEN].found);
    }
}

// ---------------------------------------------------------------
// Tracker: a candidate that teleports across the frame while a track is
// healthy is treated as a miss, not as the target moving. Keeps a passing
// distraction from yanking the track off a real teletubby.
// ---------------------------------------------------------------
static void test_out_of_gate_candidate_does_not_steal_track(void) {
    setUp_filter();
    FilterResult f;
    DetectResult r;
    uint32_t frame = 0;

    for (int i = 0; i < 5; i++) {
        BlobComponent c[1] = { mkComp(COLOR_RED, 40, 40, 40, 30) };
        blobFilter(c, 1, f);
        trackerUpdate(f, ++frame, r);
    }
    TEST_ASSERT_TRUE(r.color[COLOR_RED].found);
    int16_t heldX = r.color[COLOR_RED].cx;

    // Something red appears on the far side of the frame.
    BlobComponent far_[1] = { mkComp(COLOR_RED, 260, 190, 40, 30) };
    blobFilter(far_, 1, f);
    trackerUpdate(f, ++frame, r);

    TEST_ASSERT_TRUE(r.color[COLOR_RED].found);
    TEST_ASSERT_INT_WITHIN(3, heldX, r.color[COLOR_RED].cx);   // did not jump
}

// ---------------------------------------------------------------
// Tracker: the centroid is smoothed, so a jittery detection produces a stable
// reported position.
// ---------------------------------------------------------------
static void test_centroid_is_smoothed(void) {
    setUp_filter();
    FilterResult f;
    DetectResult r;
    uint32_t frame = 0;

    for (int i = 0; i < 10; i++) {
        int jitter = (i % 2) ? 6 : -6;
        BlobComponent c[1] = { mkComp(COLOR_RED, 100 + jitter, 100, 40, 30) };
        blobFilter(c, 1, f);
        trackerUpdate(f, ++frame, r);
    }
    TEST_ASSERT_TRUE(r.color[COLOR_RED].found);
    // Raw centroid alternates between 114 and 126; smoothed must sit between.
    TEST_ASSERT_INT_WITHIN(6, 120, r.color[COLOR_RED].cx);
}

void registerFilterTrackerTests(void) {
    RUN_TEST(test_good_blob_accepted);
    RUN_TEST(test_oversized_object_rejected);
    RUN_TEST(test_diffuse_wash_rejected_by_fill);
    RUN_TEST(test_edge_of_range_rejected_by_core);
    RUN_TEST(test_fragments_merge_back_into_one);
    RUN_TEST(test_merge_gap_zero_disables_merging);
    RUN_TEST(test_distant_fragments_not_merged);
    RUN_TEST(test_highest_confidence_wins_not_largest);
    RUN_TEST(test_single_frame_does_not_confirm);
    RUN_TEST(test_three_frames_confirm);
    RUN_TEST(test_confirmed_survives_brief_occlusion);
    RUN_TEST(test_intermittent_noise_never_confirms);
    RUN_TEST(test_out_of_gate_candidate_does_not_steal_track);
    RUN_TEST(test_centroid_is_smoothed);
}
