# Teletubby Vision

Colour-blob detection for the ENPH253 robot. Freenove ESP32-S3 WROOM CAM,
OV2640 or OV3660, QVGA RGB565.

---

## Competition day, in order

1. **`nvs status`** — check whether saved calibration loaded.
2. Set the **ROI** to exclude everything above the course:
   `roi 0 90 319 239`. This is the single most effective defence against a
   person in a matching-colour shirt, and it costs nothing.
3. **Tune each colour** under venue lighting: `tune red`, wait for the preview
   to settle, `save`. Repeat for green, purple, yellow.
4. **`lock`** — freezes exposure, gain and white balance. Do this *after*
   tuning. Auto white balance retunes the very thing your profiles measure.
5. **`fieldcheck 10`** with a target in view → want `seen%` above 95.
6. **`fieldcheck 10`** with the course empty, ideally with a teammate in a
   matching shirt standing behind it → want **every colour "never confirmed"**.
   Read the gate list to see *what* stopped it. If nothing fired, you're
   getting away with it by luck.
7. **`nvs save`** — persists everything for this camera.
8. **`verbose 0`** — silence the debug stream (it's the default anyway).

---

## Serial

**Baud is 921600** (see `platformio.ini`). At 115200 a ~90-char detection line
costs ~7.8 ms of blocking transmit, about a quarter of a 30 fps frame budget.

Per-frame output defaults to **off**. `verbose 1` (~10 Hz) or `verbose 2`
(every frame) while working. Tune previews, command replies and errors always
print.

---

## How detection works

```
frame ─► ROI + LUT classify + RLE ─► union-find CCA ─► fragment merge ─► gates ─► tracker ─► DetectResult
```

**The LUT.** RGB565 is 16 bits, so the whole colour space fits in a 64 KB
table. One byte lookup replaces the per-pixel HSV conversion and its two
integer divisions, and classifies all four colours at once. The table lives in
static internal SRAM — a PSRAM table would be slower than the arithmetic it
replaces, so the placement is guaranteed rather than hoped for.

Each entry holds a colour id plus a **core bit**: set when the pixel sits deep
inside the profile rather than on its edge. A real object is mostly core
pixels; a marginal background match clusters on the boundary. That gives a free
confidence signal at zero runtime cost.

Colours are made **mutually exclusive** at build time — a pixel matching two
profiles goes to the nearest by hue — so orange can't register as both red and
yellow.

**Connected components.** Per row, runs of one colour are RLE'd and linked to
overlapping same-colour runs in the row above via union-find, 8-connectivity.
One raster pass, all four colours, so two separate objects stay two separate
blobs instead of being averaged into one centroid between them.

**Rejecting false positives**, in order of how much work each actually does:

| Gate | Stops |
|---|---|
| ROI | Anything outside the course area — the background-person case |
| Area band | Objects too large or small to be a target at this distance |
| Fill ratio | Diffuse washes (a lit wall); weaker against a dense t-shirt |
| Core fraction | Matches that only clip the edge of the colour range |
| M-of-N temporal | Anything that doesn't persist — flashes, reflections |

Every rejection is counted by reason and reported by `fieldcheck`.

**Occlusion.** Same-colour fragments within `merge` pixels are fused back into
one candidate. There is exactly one teletubby per colour, so this is safe to do
aggressively — there's no risk of fusing two distinct targets. `merge 0`
disables it.

**Tracker.** 3-of-5 confirmation with a position gate, deliberately asymmetric:
fast to confirm, slow to drop (10 misses). A target hidden behind a rock for a
third of a second stays confirmed. A one-frame flash of colour never confirms
at all.

---

## Robot link

Continuous push on `Serial1` from a task pinned to **core 0** while vision owns
core 1, so the link can never stall a frame. Handoff is a single-slot mailbox
with overwrite semantics — the receiver always wants the newest result, and a
backlog of stale detections is worse than none on a moving robot.

**Tuned for a long wire, not for speed:** 57600 baud, 15 Hz. Plain TTL UART over
a long cable is the weak point — capacitance rounds the edges, there is no
differential noise rejection, and motor wiring couples straight in. A control
loop does not need 30 Hz of detections, and sending a short message infrequently
at a low baud leaves a large quiet window around each one.

Default (compact) format — "is something detected, and what colour":

```
V,<seq>,<mask>,<best>,<conf>,<cx>,<cy>*<xor>
```

- `seq` — `frameId & 0xFFFF`, so the peer can spot a stalled pipeline
- `mask` — bitmask of confirmed colours: bit0 red, bit1 green, bit2 purple,
  bit3 yellow. **0 means nothing detected.**
- `best` — colour index 0-3 of the highest-confidence detection, or 255
- `conf`, `cx`, `cy` — that detection's confidence and centroid
- `xor` — XOR of every character before `*`, two hex digits (NMEA style)

Worst case 29 characters: `V,65535,15,255,255,319,239*7A`

`linkformat full` switches to per-colour geometry
(`<conf>:<cx>:<cy>:<w>:<h>` for each of the four) if you need everything.

### Getting the wire right

**Verify the checksum on the receiver and count the failures.** That counter is
the only way to tell a working link from a marginal one — corrupted lines
otherwise parse into plausible-looking garbage. With it you can tune the baud
empirically: zero failures over a few minutes means you can raise `LINK_BAUD`
(115200, then 230400); any failures means drop it (38400, 19200). Change it on
**both** boards.

Also worth doing: a solid common ground between the two boards, run the ground
wire alongside the signal pair rather than relying on a shared chassis, and keep
the run away from motor leads.

Pins, baud and rate are `LINK_TX_PIN` / `LINK_RX_PIN` / `LINK_BAUD` /
`LINK_DEFAULT_HZ` at the top of `vision_link.h` — **check the pins against your
wiring.** `linkrate <hz>` and `linkformat <compact|full>` adjust at runtime.

---

## Tuning

`tune <colour>` searches for the largest connected blob roughly matching a
built-in hue prior, then measures the **5th–95th percentile** of hue,
saturation and value across it. Percentiles, not min/max: one specular
highlight and one shadowed edge pixel would otherwise set the entire range,
and averaging across frames can't fix that when every sample is already an
extreme.

The prior also *clips* what can be learned — a pixel outside `centre ±
halfWidth` is never considered. If the measured hue presses against that
boundary the preview says so; widen `halfWidth` in `COLOR_PRIORS`
(`autotune.cpp`) and re-tune.

A blob touching **any** frame edge is excluded from calibration — it's probably
bleeding into background rather than showing a clean view of the toy.

Saving: `save` puts it in RAM, `nvs save` persists it to flash for this camera,
`dump` prints a C block for `profiles_defaults.h` if you want it baked in.

### Yellow vs. a lit white background

Plain HSV thresholding can't fully separate a pale object from a white surface
under warm lighting. Defences: a saturation floor (`SEARCH_S_MIN`, 90) while
searching, a deliberately narrow hue prior for yellow, and largest-*connected*
blob selection. At runtime the fill-ratio and core-fraction gates carry this.

If it still latches onto the background, the fix is usually physical: tune
against a darker backdrop, or in the actual venue lighting — then `lock`.

---

## Commands

`help` lists everything. The ones that matter most:

| Command | Effect |
|---|---|
| `tune <colour>` / `save` / `cancel` | interactive calibration |
| `roi <x0> <y0> <x1> <y1>` / `roi full` | restrict the scan area |
| `fieldcheck [secs]` | measure detection rate and which gates fire |
| `lock` / `unlock` / `camstatus` | freeze exposure, gain, white balance |
| `mirror <on\|off>` | horizontal flip — **check the sign of cx** |
| `nvs <save\|load\|clear\|status>` | persist settings for this camera |
| `limits` | show ROI, per-colour gates, tracker settings |
| `area` / `fill` / `core` `<colour> …` | per-colour gate thresholds |
| `merge <px>` / `minrun <px>` | occlusion merging, speckle erosion |
| `confirm <m> <n>` / `gate` / `maxmiss` / `smooth` | tracker behaviour |
| `link <on\|off>` / `linkstatus` | robot UART link |
| `verbose <0\|1\|2>` | debug output level |

---

## WiFi debug telemetry — NOT for competition

A web dashboard for watching detection live, without a serial cable.

**It is off at every boot and is never persisted.** There is no saved setting
that can bring it up on its own; the only way it runs during a match is if
someone types the command during the match.

```
wifi ap                      # self-hosted network "teletubby-vision" / "enph253robot"
wifi sta <ssid> <pass>       # or join an existing 2.4 GHz network
wifi off
```

Then open the IP it prints:

| URL | What |
|---|---|
| `http://<ip>/` | dashboard — telemetry table + live video |
| `http://<ip>/data` | JSON snapshot, always available, cheap |
| `http://<ip>:81/stream` | MJPEG with blob overlay, best effort |

Telemetry costs almost nothing. **Video costs frame rate** — it JPEG-encodes a
half-resolution preview, and although that happens on core 0, it shares the
PSRAM bus and the (core-shared) data cache with the camera DMA. Default target
is 10 fps; `streamfps <1-15>` adjusts. The stream only does any work while a
browser is actually connected.

Two servers on two ports is deliberate: an MJPEG handler blocks its server task
for as long as the client watches, so it must not sit on the same task as the
page and the JSON.

### Building it out entirely

```
pio run -e competition -t upload
```

Same firmware with `wifi_debug.cpp` excluded from the build: no radio, no HTTP
server, no JPEG encoder. 394 KB flash / 153 KB RAM versus 851 KB / 186 KB for
the debug build.

Note it excludes the **file**, not just the `-D` flag. PlatformIO's dependency
finder text-scans for `#include` and will happily link the whole WiFi stack from
a line sitting inside a disabled `#ifdef`, so dropping the flag alone saves
nothing.

## Tests

Host-side tests cover the CCA, the LUT, the filter gates and the tracker —
including the occlusion, background-object and noise cases. Debugging
union-find on a robot via serial prints at 2 a.m. is miserable.

```
powershell -ExecutionPolicy Bypass -File scripts\run_native_tests.ps1
```

(`pio test -e native` does the same thing but needs a gcc that Windows doesn't
ship; the script drives Visual Studio's compiler instead.)

---

## File layout

```
include/
  camera_pins.h        pin defs (both cameras share the socket)
  frame_config.h       FRAME_W / FRAME_H, validated against every frame
  color_types.h        ColorId, CameraId, HsvRange
  hsv_convert.h        RGB565->HSV + wraparound-aware range check (no Arduino)
  color_lut.h          64 KB classification table
  blob_detect.h        RLE + union-find connected components
  blob_filter.h        ROI, gates, fragment merging, confidence
  tracker.h            M-of-N temporal confirmation
  detect_result.h      the public output struct
  vision_link.h        UART to the robot MCU (core 0)  <- PINS HERE
  communicator.hpp     copy of the main project's class; keep both in sync
  vision_nvs.h         flash persistence
  fieldcheck.h         venue measurement tool
  profiles_defaults.h  compiled-in ranges per camera
src/                   implementations; main.cpp is setup()/loop()
test/                  host-side unit tests
scripts/               run_native_tests.ps1
```

`hsv_convert.h`, `color_lut.cpp`, `blob_detect.cpp`, `blob_filter.cpp` and
`tracker.cpp` deliberately avoid `<Arduino.h>` and `<esp_camera.h>` so they can
be compiled and tested on a PC.
