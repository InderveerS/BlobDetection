#include "serial_commands.h"
#include "profiles.h"
#include "autotune.h"
#include "color_types.h"
#include "frame_config.h"
#include "vision_output.h"
#include "blob_detect.h"
#include "blob_filter.h"
#include "tracker.h"
#include "vision_link.h"
#include "communicator.hpp"
#include "detect_result.h"
#include "camera_setup.h"
#include "vision_nvs.h"
#include "fieldcheck.h"
#include "wifi_debug.h"
#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

// ================================================================
// Non-blocking line reader.
//
// The old code used Serial.readStringUntil('\n'), which blocks for up to the
// Stream timeout (1000 ms by default) whenever a line arrives in pieces -
// Serial.available() only promises >=1 byte, not a whole line. One stray byte
// on the wire froze the vision loop for a full second. On a moving robot that
// is a collision.
//
// This drains whatever has arrived, reassembles lines itself, and returns
// immediately. Also drops String, so no heap churn on a long-running system.
// ================================================================
#define CMD_BUF_LEN 96

static char   g_buf[CMD_BUF_LEN];
static size_t g_len      = 0;
static bool   g_overflow = false;

// Returns a pointer to a trimmed, null-terminated command, or nullptr if no
// complete line is available yet. The returned pointer is valid until the next
// call, which is fine because the caller consumes it immediately.
static const char* readCommandLine() {
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\r') continue;
        if (c != '\n') {
            if (g_len + 1 >= CMD_BUF_LEN) g_overflow = true;   // keep draining to the newline
            else                          g_buf[g_len++] = c;
            continue;
        }

        // end of line
        if (g_overflow) {
            Serial.println("[ERR] Command too long -- ignored.");
            g_len = 0; g_overflow = false;
            continue;
        }
        g_buf[g_len] = '\0';
        while (g_len > 0 && isspace((unsigned char)g_buf[g_len - 1])) g_buf[--g_len] = '\0';
        size_t start = 0;
        while (g_buf[start] && isspace((unsigned char)g_buf[start])) start++;
        g_len = 0;
        if (g_buf[start] == '\0') continue;   // blank line
        return g_buf + start;
    }
    return nullptr;
}

// ================================================================
// Strict argument parsing.
//
// sscanf's return value was never checked before, so `set size` with no
// argument left px at 0 and set the minimum blob size to 0 - which makes
// every single frame report a detection. The trailing %c catches junk after
// the numbers, so `set h 5 10 banana` is rejected rather than half-applied.
// ================================================================
static bool parseOneInt(const char *s, int &a) {
    char extra;
    return sscanf(s, "%d %c", &a, &extra) == 1;
}

static bool parseTwoInts(const char *s, int &a, int &b) {
    char extra;
    return sscanf(s, "%d %d %c", &a, &b, &extra) == 2;
}

static bool parseFourInts(const char *s, int &a, int &b, int &c, int &d) {
    char extra;
    return sscanf(s, "%d %d %d %d %c", &a, &b, &c, &d, &extra) == 4;
}

// "<colour> <n> [<n>]" - shared by the per-colour limit commands.
static bool parseColorAndInts(const char *s, ColorId &c, int *vals, int count);

static bool inBounds(int lo, int hi, int min, int max, const char *what) {
    if (lo < min || lo > max || hi < min || hi > max) {
        Serial.printf("[ERR] %s must be within %d-%d (got %d and %d)\n", what, min, max, lo, hi);
        return false;
    }
    return true;
}

static bool parseColorName(const char *s, ColorId &out) {
    if (!strcmp(s, "red"))    { out = COLOR_RED;    return true; }
    if (!strcmp(s, "green"))  { out = COLOR_GREEN;  return true; }
    if (!strcmp(s, "purple")) { out = COLOR_PURPLE; return true; }
    if (!strcmp(s, "yellow")) { out = COLOR_YELLOW; return true; }
    return false;
}

static bool parseColorAndInts(const char *s, ColorId &c, int *vals, int count) {
    char name[16];
    int  consumed = 0;
    if (sscanf(s, "%15s%n", name, &consumed) != 1) return false;
    if (!parseColorName(name, c)) return false;

    const char *rest = s + consumed;
    if (count == 1) { char e; return sscanf(rest, "%d %c", &vals[0], &e) == 1; }
    if (count == 2) { char e; return sscanf(rest, "%d %d %c", &vals[0], &vals[1], &e) == 2; }
    return false;
}

static void printLimits() {
    Serial.printf("ROI: (%d,%d)-(%d,%d)   mergeGap: %d px   minFragment: %d px   minRun: %d px\n",
        getScanRoi().x0, getScanRoi().y0, getScanRoi().x1, getScanRoi().y1,
        getMergeGap(), getMinFragmentPx(), getMinRunLength());

    int m, n;
    getTrackerConfirm(m, n);
    Serial.printf("Tracker: confirm %d-of-%d   gate %d px   maxMiss %d frames   smoothing %d%%\n",
        m, n, getTrackerGatePx(), getTrackerMaxMiss(), getTrackerSmoothing());

    Serial.println("Per-colour limits:");
    Serial.println("         area(min-max)   fill%  core%  aspect(x100)");
    for (int c = 0; c < COLOR_COUNT; c++) {
        const ColorLimits &L = colorLimits((ColorId)c);
        Serial.printf("  %-7s %6u-%-6u  %4u   %4u   %u-%u\n",
            colorName((ColorId)c), (unsigned)L.minArea, (unsigned)L.maxArea,
            L.minFillPct, L.minCorePct, L.minAspectX100, L.maxAspectX100);
    }
}

static void printHelp() {
    Serial.println("Commands:");
    Serial.println("  color <red|green|purple|yellow>  select active detect color");
    Serial.println("  tune <red|green|purple|yellow>   start interactive auto-tune");
    Serial.println("  save                             save the current tune session");
    Serial.println("  cancel                           abort the current tune session");
    Serial.println("  set h <min> <max>                manual override, active color (H: 0-179, min>max wraps)");
    Serial.println("  set s <min> <max>                (S: 0-255)");
    Serial.println("  set v <min> <max>                (V: 0-255)");
    Serial.println("  set size <min_px>                min blob pixels (detection only, not tuned)");
    Serial.println("  verbose <0|1|2>                  0=silent 1=~10Hz 2=every frame");
    Serial.println("  reset <color|all>                reload compiled-in defaults");
    Serial.println("  dump                             print current camera's profiles as C code");
    Serial.println("  params                           show active color + range");
    Serial.println("  camera                           show detected camera");
    Serial.println("False-positive rejection:");
    Serial.println("  limits                           show ROI, per-color limits, tracker settings");
    Serial.println("  roi <x0> <y0> <x1> <y1> | full   restrict the scan (excludes background above course)");
    Serial.println("  area <color> <min> <max>         plausible blob size in pixels");
    Serial.println("  fill <color> <pct>               min pixels/bbox -- rejects diffuse washes");
    Serial.println("  core <color> <pct>               min share of pixels deep inside the range");
    Serial.println("  aspect <color> <min> <max>       width/height x100 (100 = square)");
    Serial.println("  merge <px>                       fuse occluded fragments within this gap (0=off)");
    Serial.println("  minrun <px>                      horizontal erosion, kills speckle");
    Serial.println("  minfrag <px>                     drop blobs below this before merging");
    Serial.println("  confirm <m> <n>                  require m of last n frames before 'found'");
    Serial.println("  gate <px>                        max jump between frames for the same target");
    Serial.println("  maxmiss <frames>                 misses tolerated before dropping a track");
    Serial.println("  smooth <pct>                     centroid EMA weight (100 = no smoothing)");
    Serial.println("Robot link (Serial1, core 0):");
    Serial.println("  link <on|off>                    enable/disable transmission to the other MCU");
    Serial.println("  linkrate <hz>                    transmissions/sec (lower = safer long wire)");
    Serial.println("  linkformat <compact|full>        compact = detected+colour; full = all geometry");
    Serial.println("  linkstatus                       pins, baud, rate, counters, sample line");
    Serial.println("Venue setup:");
    Serial.println("  fieldcheck [secs]                measure detection + which gates fire (default 10)");
    Serial.println("  lock                             settle, then freeze exposure/gain/white-balance");
    Serial.println("  unlock                           back to auto (ranges will drift again)");
    Serial.println("  mirror <on|off>                  horizontal flip -- check the sign of cx!");
    Serial.println("  camstatus                        exposure/gain/AWB lock state");
    Serial.println("  nvs <save|load|clear|status>     persist settings to flash for this camera");
    Serial.println("WiFi debug -- NOT FOR COMPETITION (off at every boot):");
    Serial.println("  wifi ap                          self-hosted network + web dashboard");
    Serial.println("  wifi sta <ssid> <pass>           join an existing 2.4GHz network");
    Serial.println("  wifi off                         radio down, full frame rate back");
    Serial.println("  wifistatus                       state and URLs");
    Serial.println("  streamfps <n>                    video target fps (1-15; lower = less cost)");
}

// Shared by `set h/s/v`: fetch, mutate one axis, store, echo.
static void applyAxis(char axis, int lo, int hi) {
    ColorId  active = getActiveDetectColor();
    HsvRange r      = getColorProfile(active);

    switch (axis) {
        // Hue deliberately allows min > max: that is how a wrapped range
        // (red, across the 0/179 seam) is represented.
        case 'h': if (!inBounds(lo, hi, 0, 179, "hue")) return; r.hMin = lo; r.hMax = hi; break;
        case 's': if (!inBounds(lo, hi, 0, 255, "saturation")) return;
                  if (lo > hi) { Serial.println("[ERR] S min must be <= max"); return; }
                  r.sMin = lo; r.sMax = hi; break;
        case 'v': if (!inBounds(lo, hi, 0, 255, "value")) return;
                  if (lo > hi) { Serial.println("[ERR] V min must be <= max"); return; }
                  r.vMin = lo; r.vMax = hi; break;
        default: return;
    }
    setColorProfile(active, r);
    Serial.printf("[SET %s] ", colorName(active));
    printRange(r);
}

void handleSerialCommands() {
    const char *cmd = readCommandLine();
    if (!cmd) return;

    ColorId c;

    if (!strcmp(cmd, "help")) {
        printHelp();
    }
    else if (!strcmp(cmd, "params")) {
        ColorId active = getActiveDetectColor();
        Serial.printf("[%s] active color: %s  ", cameraName(getActiveCameraId()), colorName(active));
        printRange(getColorProfile(active));
        Serial.printf("      min blob px: %d   verbose: %d (%s)\n",
                      getMinBlobPx(), getVerbosity(), verbosityName(getVerbosity()));
    }
    else if (!strcmp(cmd, "camera")) {
        Serial.printf("Camera: %s\n", cameraName(getActiveCameraId()));
    }
    else if (!strcmp(cmd, "dump"))   { dumpProfiles(); }
    else if (!strcmp(cmd, "save"))   { tuneSave(); }
    else if (!strcmp(cmd, "cancel")) { tuneCancel(); }
    else if (!strncmp(cmd, "color ", 6)) {
        if (parseColorName(cmd + 6, c)) {
            setActiveDetectColor(c);
            Serial.printf("[ACTIVE] %s  ", colorName(c));
            printRange(getColorProfile(c));
        } else {
            Serial.println("Unknown color. Options: red green purple yellow");
        }
    }
    else if (!strncmp(cmd, "tune ", 5)) {
        if (parseColorName(cmd + 5, c)) tuneStart(c);
        else Serial.println("Unknown color. Options: red green purple yellow");
    }
    else if (!strncmp(cmd, "reset ", 6)) {
        const char *arg = cmd + 6;
        if (!strcmp(arg, "all")) {
            resetAllToDefaults();
            Serial.println("[RESET] all colors reloaded from defaults");
        } else if (parseColorName(arg, c)) {
            resetColorToDefault(c);
            Serial.printf("[RESET] %s reloaded from defaults\n", colorName(c));
        } else {
            Serial.println("Usage: reset <red|green|purple|yellow|all>");
        }
    }
    else if (!strncmp(cmd, "verbose ", 8)) {
        int v;
        if (!parseOneInt(cmd + 8, v) || v < 0 || v > 2) {
            Serial.println("Usage: verbose <0|1|2>   (0=silent, 1=~10Hz, 2=every frame)");
        } else {
            setVerbosity(v);
            Serial.printf("[SET] verbose = %d (%s)\n", getVerbosity(), verbosityName(getVerbosity()));
        }
    }
    // `set size ` is checked before `set s ` would ever see it, and the
    // trailing space in "set s " means "set size 500" can't match it anyway.
    else if (!strncmp(cmd, "set size ", 9)) {
        int px;
        if (!parseOneInt(cmd + 9, px) || px < 1 || px > (int)FRAME_PIXELS) {
            Serial.printf("Usage: set size <min_px>   (1-%u)\n", (unsigned)FRAME_PIXELS);
        } else {
            setMinBlobPx(px);
            Serial.printf("[SET] min blob px = %d\n", getMinBlobPx());
        }
    }
    else if (!strncmp(cmd, "set h ", 6) || !strncmp(cmd, "set s ", 6) || !strncmp(cmd, "set v ", 6)) {
        int lo, hi;
        if (!parseTwoInts(cmd + 6, lo, hi)) {
            Serial.printf("Usage: set %c <min> <max>\n", cmd[4]);
        } else {
            applyAxis(cmd[4], lo, hi);
        }
    }
    else if (!strcmp(cmd, "limits")) {
        printLimits();
    }
    else if (!strncmp(cmd, "roi ", 4)) {
        const char *arg = cmd + 4;
        int x0, y0, x1, y1;
        if (!strcmp(arg, "full")) {
            setScanRoi(fullFrameRoi());
            Serial.println("[SET] ROI = full frame");
        } else if (!parseFourInts(arg, x0, y0, x1, y1)) {
            Serial.printf("Usage: roi <x0> <y0> <x1> <y1>   (0-%d, 0-%d) or 'roi full'\n",
                          FRAME_W - 1, FRAME_H - 1);
        } else if (x0 < 0 || y0 < 0 || x1 > FRAME_W - 1 || y1 > FRAME_H - 1 || x0 >= x1 || y0 >= y1) {
            Serial.printf("[ERR] ROI must satisfy 0<=x0<x1<=%d and 0<=y0<y1<=%d\n",
                          FRAME_W - 1, FRAME_H - 1);
        } else {
            ScanRoi r = { (int16_t)x0, (int16_t)y0, (int16_t)x1, (int16_t)y1 };
            setScanRoi(r);
            Serial.printf("[SET] ROI = (%d,%d)-(%d,%d)\n", x0, y0, x1, y1);
        }
    }
    else if (!strncmp(cmd, "area ", 5)) {
        int v[2];
        if (!parseColorAndInts(cmd + 5, c, v, 2) || v[0] < 1 || v[1] <= v[0] || v[1] > (int)FRAME_PIXELS) {
            Serial.printf("Usage: area <color> <min> <max>   (1 < min < max <= %u)\n",
                          (unsigned)FRAME_PIXELS);
        } else {
            colorLimits(c).minArea = (uint32_t)v[0];
            colorLimits(c).maxArea = (uint32_t)v[1];
            Serial.printf("[SET] %s area = %d-%d px\n", colorName(c), v[0], v[1]);
        }
    }
    else if (!strncmp(cmd, "fill ", 5)) {
        int v[1];
        if (!parseColorAndInts(cmd + 5, c, v, 1) || v[0] < 0 || v[0] > 99) {
            Serial.println("Usage: fill <color> <pct>   (0-99)");
        } else {
            colorLimits(c).minFillPct = (uint8_t)v[0];
            Serial.printf("[SET] %s min fill = %d%%\n", colorName(c), v[0]);
        }
    }
    else if (!strncmp(cmd, "aspect ", 7)) {
        int v[2];
        if (!parseColorAndInts(cmd + 7, c, v, 2) || v[0] < 1 || v[1] <= v[0] || v[1] > 2000) {
            Serial.println("Usage: aspect <color> <min> <max>   (width/height x100; 100 = square)");
            Serial.println("       e.g. 'aspect red 30 300' allows 0.30 to 3.00");
        } else {
            colorLimits(c).minAspectX100 = (uint16_t)v[0];
            colorLimits(c).maxAspectX100 = (uint16_t)v[1];
            Serial.printf("[SET] %s aspect = %.2f-%.2f (w/h)\n",
                          colorName(c), v[0] / 100.0f, v[1] / 100.0f);
        }
    }
    else if (!strncmp(cmd, "minfrag ", 8)) {
        int px;
        if (!parseOneInt(cmd + 8, px) || px < 1 || px > 1000) {
            Serial.println("Usage: minfrag <px>   (1-1000; blobs smaller than this are");
            Serial.println("       discarded before merging, so they can't drag a bbox out)");
        } else {
            setMinFragmentPx(px);
            Serial.printf("[SET] min fragment = %d px\n", getMinFragmentPx());
        }
    }
    else if (!strncmp(cmd, "core ", 5)) {
        int v[1];
        if (!parseColorAndInts(cmd + 5, c, v, 1) || v[0] < 0 || v[0] > 100) {
            Serial.println("Usage: core <color> <pct>   (0-100)");
        } else {
            colorLimits(c).minCorePct = (uint8_t)v[0];
            Serial.printf("[SET] %s min core = %d%%\n", colorName(c), v[0]);
        }
    }
    else if (!strncmp(cmd, "merge ", 6)) {
        int px;
        if (!parseOneInt(cmd + 6, px) || px < 0 || px > 100) {
            Serial.println("Usage: merge <px>   (0-100, 0 disables fragment merging)");
        } else {
            setMergeGap(px);
            Serial.printf("[SET] merge gap = %d px%s\n", px, px == 0 ? " (merging off)" : "");
        }
    }
    else if (!strncmp(cmd, "minrun ", 7)) {
        int px;
        if (!parseOneInt(cmd + 7, px) || px < 1 || px > 32) {
            Serial.println("Usage: minrun <px>   (1-32, 1 disables erosion)");
        } else {
            setMinRunLength(px);
            Serial.printf("[SET] min run length = %d px\n", getMinRunLength());
        }
    }
    else if (!strncmp(cmd, "confirm ", 8)) {
        int m, n;
        if (!parseTwoInts(cmd + 8, m, n) || m < 1 || n < 1 || n > 16 || m > n) {
            Serial.println("Usage: confirm <m> <n>   (1 <= m <= n <= 16)");
        } else {
            setTrackerConfirm(m, n);
            getTrackerConfirm(m, n);
            Serial.printf("[SET] confirm %d of last %d frames\n", m, n);
        }
    }
    else if (!strncmp(cmd, "gate ", 5)) {
        int px;
        if (!parseOneInt(cmd + 5, px) || px < 1 || px > FRAME_W) {
            Serial.printf("Usage: gate <px>   (1-%d)\n", FRAME_W);
        } else {
            setTrackerGatePx(px);
            Serial.printf("[SET] tracker gate = %d px\n", getTrackerGatePx());
        }
    }
    else if (!strncmp(cmd, "maxmiss ", 8)) {
        int f;
        if (!parseOneInt(cmd + 8, f) || f < 0 || f > 200) {
            Serial.println("Usage: maxmiss <frames>   (0-200)");
        } else {
            setTrackerMaxMiss(f);
            Serial.printf("[SET] max misses before dropping a track = %d\n", getTrackerMaxMiss());
        }
    }
    else if (!strncmp(cmd, "smooth ", 7)) {
        int a;
        if (!parseOneInt(cmd + 7, a) || a < 1 || a > 100) {
            Serial.println("Usage: smooth <pct>   (1-100, 100 = no smoothing)");
        } else {
            setTrackerSmoothing(a);
            Serial.printf("[SET] centroid smoothing = %d%%\n", getTrackerSmoothing());
        }
    }
    else if (!strncmp(cmd, "link ", 5)) {
        const char *arg = cmd + 5;
        if (!strcmp(arg, "on"))       { setLinkEnabled(true);  Serial.println("[LINK] transmit ON"); }
        else if (!strcmp(arg, "off")) { setLinkEnabled(false); Serial.println("[LINK] transmit OFF"); }
        else                            Serial.println("Usage: link <on|off>");
    }
    else if (!strncmp(cmd, "linkrate ", 9)) {
        int hz;
        if (!parseOneInt(cmd + 9, hz) || hz < 1 || hz > 100) {
            Serial.println("Usage: linkrate <hz>   (1-100; lower is safer on a long wire)");
        } else {
            setLinkRateHz(hz);
            Serial.printf("[LINK] rate = %d Hz\n", getLinkRateHz());
        }
    }
    else if (!strncmp(cmd, "linkformat ", 11)) {
        const char *arg = cmd + 11;
        if (!strcmp(arg, "compact"))   { setLinkFormat(LINK_FMT_COMPACT); Serial.println("[LINK] format = compact"); }
        else if (!strcmp(arg, "full")) { setLinkFormat(LINK_FMT_FULL);    Serial.println("[LINK] format = full"); }
        else Serial.println("Usage: linkformat <compact|full>");
    }
    else if (!strcmp(cmd, "linkstatus")) {
        Serial.printf("[LINK] %s  TX=%d RX=%d @ %u baud  %d Hz  format=%s  (core 0)\n",
            getLinkEnabled() ? "ON" : "OFF", LINK_TX_PIN, LINK_RX_PIN, (unsigned)LINK_BAUD,
            getLinkRateHz(), getLinkFormat() == LINK_FMT_COMPACT ? "compact" : "full");
        Serial.printf("       sent: %u   dropped(too long): %u\n",
            (unsigned)linkSentCount(), (unsigned)linkDroppedCount());
        Serial.printf("       last inbound: \"%s\"\n", linkLastInbound());
        char sample[Communicator::BUF_SIZE];
        int n = visionLinkFormat(latestResult(), sample, (int)sizeof(sample));
        if (n > 0) Serial.printf("       current line (%d chars): %s\n", n, sample);
    }
    else if (!strcmp(cmd, "lock"))      { cameraLock(); }
    else if (!strcmp(cmd, "unlock"))    { cameraUnlock(); }
    else if (!strcmp(cmd, "camstatus")) { cameraPrintStatus(); }
    else if (!strncmp(cmd, "mirror ", 7)) {
        const char *arg = cmd + 7;
        if (!strcmp(arg, "on") || !strcmp(arg, "off")) {
            bool on = !strcmp(arg, "on");
            cameraSetMirror(on);
            Serial.printf("[SET] hmirror %s -- verify: move a toy to your RIGHT and confirm cx increases.\n",
                          on ? "ON" : "off");
        } else {
            Serial.println("Usage: mirror <on|off>");
        }
    }
    else if (!strcmp(cmd, "fieldcheck")) { fieldcheckStart(10); }
    else if (!strncmp(cmd, "fieldcheck ", 11)) {
        int secs;
        if (!parseOneInt(cmd + 11, secs) || secs < 1 || secs > 60)
            Serial.println("Usage: fieldcheck [seconds]   (1-60, default 10)");
        else
            fieldcheckStart(secs);
    }
    else if (!strncmp(cmd, "nvs ", 4)) {
        const char *arg = cmd + 4;
        if (!strcmp(arg, "save"))        visionNvsSave();
        else if (!strcmp(arg, "clear"))  visionNvsClear();
        else if (!strcmp(arg, "status")) visionNvsStatus();
        else if (!strcmp(arg, "load")) {
            if (visionNvsLoad()) Serial.println("[NVS] Loaded.");
            else                 Serial.println("[NVS] Nothing stored for this camera.");
        }
        else Serial.println("Usage: nvs <save|load|clear|status>");
    }
    else if (!strcmp(cmd, "wifistatus")) { wifiDebugStatus(); }
    else if (!strncmp(cmd, "wifi ", 5)) {
        const char *arg = cmd + 5;
        if (!strcmp(arg, "ap"))       wifiDebugStartAP();
        else if (!strcmp(arg, "off")) wifiDebugStop();
        else if (!strncmp(arg, "sta ", 4)) {
            char ssid[40], pass[64], extra;
            int  got = sscanf(arg + 4, "%39s %63s %c", ssid, pass, &extra);
            if (got == 2)      wifiDebugStartSTA(ssid, pass);
            else if (got == 1) wifiDebugStartSTA(ssid, "");   // open network
            else Serial.println("Usage: wifi sta <ssid> <password>   (no spaces in either)");
        }
        else Serial.println("Usage: wifi <ap|sta <ssid> <pass>|off>");
    }
    else if (!strncmp(cmd, "streamfps ", 10)) {
        int fps;
        if (!parseOneInt(cmd + 10, fps) || fps < 1 || fps > 15) {
            Serial.println("Usage: streamfps <n>   (1-15)");
        } else {
            setStreamFps(fps);
            Serial.printf("[WIFI] video target = %d fps\n", getStreamFps());
        }
    }
    else {
        Serial.printf("Unknown: '%s'. Type 'help'.\n", cmd);
    }
}
