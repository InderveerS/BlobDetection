#pragma once
#include <stdint.h>
#include "detect_result.h"

// ================================================================
// UART link to the main robot MCU, running on the OTHER CPU core.
//
// ---- pins: CHANGE THESE TWO after checking your wiring ----
// The camera occupies GPIO 4-13 and 15-18. GPIO 19/20 are USB D+/D- and 43/44
// are UART0 (the USB debug port), so all of those are unavailable.
#define LINK_TX_PIN   47
#define LINK_RX_PIN   48

// Deliberately slow. Plain TTL UART over a long wire is the weak link here:
// cable capacitance rounds the edges, there's no differential noise rejection,
// and motor wiring nearby couples straight in. Every halving of the baud rate
// roughly doubles the noise margin, and we simply do not need the bandwidth -
// see LINK_DEFAULT_HZ below.
//
// If the receiver reports zero checksum failures over a few minutes you can
// raise this (115200, then 230400). If you see failures, drop it (38400,
// 19200). Change it on BOTH boards.
#define LINK_BAUD  57600
// -----------------------------------------------------------
//
// Why core 0: the vision pipeline owns core 1 (the Arduino loop). Espressif
// documents that the ESP32-S3's data cache is shared between both cores and
// that PSRAM bandwidth suffers when a core and DMA hit external RAM at once -
// which is exactly our situation, since the camera DMA writes frame N+1 while
// we read frame N. So splitting the *pixel* work across cores would buy little
// and cost a lot of synchronisation. Splitting the *I/O* off costs almost
// nothing and guarantees the robot link can never stall a frame.
//
// Handoff is a single-slot mailbox with overwrite semantics: the consumer
// always wants the newest result, and a backlog of stale detections would be
// actively harmful on a moving robot.
//
// Transmission is decoupled from the frame rate: vision runs as fast as it
// likes, the link sends the NEWEST result at a fixed slow cadence. A control
// loop does not need 30 Hz of detections, and sending less often over a long
// wire means each message gets a large quiet window around it.
#define LINK_DEFAULT_HZ 15

// Wire format, one ASCII line per transmission. The peer's poll() keeps only
// the newest complete line, so everything must fit in one.
//
// COMPACT (default) - "is something detected, and what colour":
//
//   V,<seq>,<mask>,<best>,<conf>,<cx>,<cy>*<xor>
//
//   seq    frameId & 0xFFFF - lets the peer spot a stalled pipeline
//   mask   bitmask of confirmed colours: bit0 red, bit1 green,
//          bit2 purple, bit3 yellow. 0 = nothing detected.
//   best   colour index 0-3 of the highest-confidence detection, or 255
//   conf   that detection's confidence 0-255 (0 if none)
//   cx,cy  its centroid (0,0 if none)
//   xor    XOR of every character before '*', 2 hex digits (NMEA style)
//
//   Worst case 29 characters, e.g.  V,65535,15,255,255,319,239*7A
//
// FULL - adds per-colour geometry, for when you want everything:
//
//   V,<seq>,<best>,<red>,<green>,<purple>,<yellow>*<xor>
//   each colour: <confidence>:<cx>:<cy>:<width>:<height>
//
// ALWAYS verify the checksum on the receiving end and count the failures.
// That count is your signal for whether LINK_BAUD is too high for the wire -
// it is the only way to tell a noisy link from a working one.
//
// Confidence is shipped raw so the receiving MCU sets its own threshold -
// competition-day tuning then happens on whichever board is easier to reflash.
// ================================================================

enum LinkFormat { LINK_FMT_COMPACT = 0, LINK_FMT_FULL = 1 };

// Starts Serial1 and the core-0 link task. Safe to call once, from setup().
bool visionLinkBegin();

// Called from the vision loop each frame. Non-blocking: overwrites the mailbox
// and returns immediately.
void visionLinkPublish(const DetectResult &r);

void setLinkEnabled(bool on);
bool getLinkEnabled();

// Transmissions per second. Lower is safer on a long or noisy wire.
void setLinkRateHz(int hz);
int  getLinkRateHz();

void       setLinkFormat(LinkFormat f);
LinkFormat getLinkFormat();

// Diagnostics for `linkstatus`.
uint32_t    linkSentCount();
uint32_t    linkDroppedCount();   // lines refused because they wouldn't fit
const char* linkLastInbound();    // most recent line received from the peer

// Formats a result into the wire line. Exposed for testing.
int visionLinkFormat(const DetectResult &r, char *buf, int bufLen);
