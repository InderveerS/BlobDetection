#include "vision_link.h"
#include "communicator.hpp"
#include <Arduino.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define LINK_TASK_STACK 4096
#define LINK_TASK_PRIO  2
#define LINK_CORE       0     // vision owns core 1

static Communicator  g_comm(Serial1, LINK_RX_PIN, LINK_TX_PIN, LINK_BAUD);
static QueueHandle_t g_mailbox = nullptr;
static TaskHandle_t  g_task    = nullptr;
static volatile bool g_enabled = true;

static volatile uint32_t g_sent    = 0;
static volatile uint32_t g_dropped = 0;
static char              g_inbound[Communicator::BUF_SIZE] = {0};

static volatile int        g_intervalMs = 1000 / LINK_DEFAULT_HZ;
static volatile LinkFormat g_format     = LINK_FMT_COMPACT;

// Appends "*<xor>" over everything written so far. Returns the total length,
// or -1 if it wouldn't fit.
static int appendChecksum(char *buf, int len, int bufLen) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) sum ^= (uint8_t)buf[i];
    int n = snprintf(buf + len, (size_t)(bufLen - len), "*%02X", sum);
    if (n < 0 || len + n >= bufLen) return -1;
    return len + n;
}

int visionLinkFormat(const DetectResult &r, char *buf, int bufLen) {
    int len;

    if (g_format == LINK_FMT_COMPACT) {
        // One bit per colour, so the peer can answer "is anything detected,
        // and which colours" from a single small integer.
        unsigned mask = 0;
        for (int c = 0; c < COLOR_COUNT; c++)
            if (r.color[c].found) mask |= (1u << c);

        unsigned conf = 0;
        int      cx = 0, cy = 0;
        if (r.bestColor != NO_COLOR) {
            const ColorDetection &d = r.color[r.bestColor];
            conf = d.confidence; cx = d.cx; cy = d.cy;
        }

        len = snprintf(buf, (size_t)bufLen, "V,%u,%u,%u,%u,%d,%d",
                       (unsigned)(r.frameId & 0xFFFFu), mask,
                       (unsigned)r.bestColor, conf, cx, cy);
        if (len < 0 || len >= bufLen) return -1;
        return appendChecksum(buf, len, bufLen);
    }

    len = snprintf(buf, (size_t)bufLen, "V,%u,%u",
                   (unsigned)(r.frameId & 0xFFFFu), (unsigned)r.bestColor);
    if (len < 0 || len >= bufLen) return -1;

    for (int c = 0; c < COLOR_COUNT; c++) {
        const ColorDetection &d = r.color[c];
        int w = d.found ? (d.x1 - d.x0 + 1) : 0;
        int h = d.found ? (d.y1 - d.y0 + 1) : 0;
        int n = snprintf(buf + len, (size_t)(bufLen - len), ",%u:%d:%d:%d:%d",
                         (unsigned)(d.found ? d.confidence : 0),
                         d.found ? d.cx : 0, d.found ? d.cy : 0, w, h);
        if (n < 0 || len + n >= bufLen) return -1;
        len += n;
    }
    return appendChecksum(buf, len, bufLen);
}

static void linkTask(void *) {
    DetectResult r;
    bool         haveResult = false;
    uint32_t     lastSendMs = 0;
    char         line[Communicator::BUF_SIZE];

    for (;;) {
        // Take the newest result if one is waiting. The mailbox overwrites, so
        // whatever is in there is always the latest frame - intermediate
        // frames are intentionally skipped rather than queued. Stale
        // detections are worse than no detections on a moving robot.
        if (xQueueReceive(g_mailbox, &r, pdMS_TO_TICKS(10)) == pdTRUE) haveResult = true;

        uint32_t now = millis();
        if (haveResult && g_enabled && (int32_t)(now - lastSendMs) >= g_intervalMs) {
            lastSendMs = now;
            // Written only here (core 0), read only by `linkstatus` (core 1).
            // Aligned 32-bit access is atomic on Xtensa, so a plain assignment
            // is safe; `++` on a volatile is deprecated.
            int len = visionLinkFormat(r, line, (int)sizeof(line));
            if (len > 0 && g_comm.sendf("%s", line)) g_sent    = g_sent + 1;
            else                                     g_dropped = g_dropped + 1;
        }

        // Drain the RX side so the buffer can never overflow. Nothing acts on
        // inbound messages yet - the peer has no command protocol defined -
        // but the newest one is kept visible via `linkstatus`.
        if (g_comm.poll()) {
            strncpy(g_inbound, g_comm.lastMessage(), sizeof(g_inbound) - 1);
            g_inbound[sizeof(g_inbound) - 1] = '\0';
        }
    }
}

bool visionLinkBegin() {
    g_mailbox = xQueueCreate(1, sizeof(DetectResult));   // length 1: xQueueOverwrite requires it
    if (!g_mailbox) {
        Serial.println("[LINK] Failed to create mailbox -- robot link disabled.");
        return false;
    }

    g_comm.begin();

    BaseType_t ok = xTaskCreatePinnedToCore(linkTask, "vision_link", LINK_TASK_STACK,
                                            nullptr, LINK_TASK_PRIO, &g_task, LINK_CORE);
    if (ok != pdPASS) {
        Serial.println("[LINK] Failed to start link task -- robot link disabled.");
        vQueueDelete(g_mailbox);
        g_mailbox = nullptr;
        return false;
    }

    Serial.printf("[LINK] Serial1 up on TX=%d RX=%d @ %u baud, %d Hz, task on core %d.\n",
                  LINK_TX_PIN, LINK_RX_PIN, (unsigned)LINK_BAUD, getLinkRateHz(), LINK_CORE);
    Serial.println("       (vision runs on core 1, so the link can never stall a frame)");
    Serial.println("       Verify the checksum on the receiver and count failures -- that");
    Serial.println("       count tells you whether the baud is too high for your wire.");
    return true;
}

void visionLinkPublish(const DetectResult &r) {
    if (!g_mailbox) return;
    // Overwrite, never block: the consumer always wants the newest result, and
    // a queue of stale detections would be worse than none.
    xQueueOverwrite(g_mailbox, &r);
}

void setLinkEnabled(bool on) { g_enabled = on; }
bool getLinkEnabled()        { return g_enabled; }

void setLinkRateHz(int hz) {
    if (hz < 1)   hz = 1;
    if (hz > 100) hz = 100;
    g_intervalMs = 1000 / hz;
}
int getLinkRateHz() { return 1000 / (g_intervalMs < 1 ? 1 : g_intervalMs); }

void       setLinkFormat(LinkFormat f) { g_format = f; }
LinkFormat getLinkFormat()             { return g_format; }

uint32_t    linkSentCount()    { return g_sent; }
uint32_t    linkDroppedCount() { return g_dropped; }
const char* linkLastInbound()  { return g_inbound; }
