#include "wifi_debug.h"

// [env:competition] excludes this file from the build entirely; see
// platformio.ini for why the #ifdef alone isn't enough.
#ifdef ENABLE_WIFI_DEBUG

#include <Arduino.h>
#include <WiFi.h>
#include <esp_http_server.h>
#include "esp_heap_caps.h"
#include "img_converters.h"
#include "profiles.h"

static httpd_handle_t g_httpPage   = nullptr;   // port 80
static httpd_handle_t g_httpStream = nullptr;   // port 81
static bool           g_running    = false;
static int            g_streamFps  = STREAM_DEFAULT_FPS;

// ---- telemetry snapshot ----
// Written by the vision loop (core 1), read by HTTP handlers (core 0). Small
// enough that a spinlock costs nothing and is far cheaper than a mutex.
static portMUX_TYPE  g_telemetryMux = portMUX_INITIALIZER_UNLOCKED;
static DetectResult  g_telResult;
static FilterResult  g_telFilter;
static BlobScanStats g_telStats;
static float         g_telFps = 0;

// ---- video staging ----
// The vision loop downscales into here and hands off; the stream handler
// encodes from it. A mutex taken with zero timeout on the producer side means
// vision skips a frame rather than ever waiting on the encoder.
static uint8_t          *g_preview      = nullptr;     // PREVIEW_W*PREVIEW_H*2, PSRAM
static SemaphoreHandle_t g_previewMutex = nullptr;
static SemaphoreHandle_t g_frameReady   = nullptr;
static volatile int      g_streamClients = 0;
static uint32_t          g_lastOfferMs   = 0;
static DetectResult      g_previewResult;             // detections matching g_preview

static const uint16_t COLOR_RGB565[COLOR_COUNT] = {
    0xF800,   // red
    0x07E0,   // green
    0xF81F,   // purple -> magenta, far more visible than true purple
    0xFFE0,   // yellow
};

// ---------------- preview rendering ----------------

static inline void putPx(uint8_t *buf, int x, int y, uint16_t c) {
    if (x < 0 || x >= PREVIEW_W || y < 0 || y >= PREVIEW_H) return;
    size_t i = ((size_t)y * PREVIEW_W + x) * 2;
    buf[i]     = (uint8_t)(c >> 8);      // same big-endian order as the camera fb
    buf[i + 1] = (uint8_t)(c & 0xFF);
}

static void drawRect(uint8_t *buf, int x0, int y0, int x1, int y1, uint16_t c) {
    for (int x = x0; x <= x1; x++) { putPx(buf, x, y0, c); putPx(buf, x, y1, c); }
    for (int y = y0; y <= y1; y++) { putPx(buf, x0, y, c); putPx(buf, x1, y, c); }
}

static void drawCross(uint8_t *buf, int cx, int cy, uint16_t c) {
    for (int d = -4; d <= 4; d++) { putPx(buf, cx + d, cy, c); putPx(buf, cx, cy + d, c); }
}

// 2x2 nearest-neighbour downscale, straight byte copy so the RGB565 byte order
// is preserved exactly as fmt2jpg expects it.
static void downscaleInto(uint8_t *dst, const uint8_t *src) {
    for (int y = 0; y < PREVIEW_H; y++) {
        const uint8_t *s = src + ((size_t)(y * 2) * FRAME_W) * 2;
        uint8_t       *d = dst + ((size_t)y * PREVIEW_W) * 2;
        for (int x = 0; x < PREVIEW_W; x++) {
            d[0] = s[0];
            d[1] = s[1];
            d += 2;
            s += 4;              // skip a pixel horizontally
        }
    }
}

// Fixed ROI guide box, in full-frame coordinates. Hard-coded: it does not
// follow the `roi` command.
static const int GUIDE_X0 = 65, GUIDE_Y0 = 105, GUIDE_X1 = 255, GUIDE_Y1 = 239;

static void drawOverlay(uint8_t *buf, const DetectResult &r) {
    // Drawn first so the detections sit on top of it.
    drawRect(buf, GUIDE_X0 / 2, GUIDE_Y0 / 2, GUIDE_X1 / 2, GUIDE_Y1 / 2, 0x0000);

    for (int c = 0; c < COLOR_COUNT; c++) {
        const ColorDetection &d = r.color[c];
        if (!d.found) continue;
        // Detection coordinates are full-frame; the preview is half scale.
        drawRect(buf, d.x0 / 2, d.y0 / 2, d.x1 / 2, d.y1 / 2, COLOR_RGB565[c]);
        drawCross(buf, d.cx / 2, d.cy / 2, COLOR_RGB565[c]);
    }
}

// ---------------- producer side (called from the vision loop) ----------------

void wifiDebugPublish(const DetectResult &r, const FilterResult &f,
                      const BlobScanStats &st, float fps) {
    if (!g_running) return;
    portENTER_CRITICAL(&g_telemetryMux);
    g_telResult = r;
    g_telFilter = f;
    g_telStats  = st;
    g_telFps    = fps;
    portEXIT_CRITICAL(&g_telemetryMux);
}

void wifiDebugOfferFrame(const uint8_t *frame) {
    if (!g_running || !g_preview || g_streamClients <= 0) return;

    int interval = 1000 / (g_streamFps < 1 ? 1 : g_streamFps);
    uint32_t now = millis();
    if ((int32_t)(now - g_lastOfferMs) < interval) return;

    // Zero timeout: if the encoder still holds the buffer, drop this frame.
    // Vision must never block on the debug path.
    if (xSemaphoreTake(g_previewMutex, 0) != pdTRUE) return;
    g_lastOfferMs = now;
    downscaleInto(g_preview, frame);
    portENTER_CRITICAL(&g_telemetryMux);
    g_previewResult = g_telResult;
    portEXIT_CRITICAL(&g_telemetryMux);
    xSemaphoreGive(g_previewMutex);
    xSemaphoreGive(g_frameReady);
}

// ---------------- HTTP handlers ----------------

static esp_err_t dataHandler(httpd_req_t *req) {
    DetectResult  r;
    FilterResult  f;
    BlobScanStats st;
    float         fps;

    portENTER_CRITICAL(&g_telemetryMux);
    r = g_telResult; f = g_telFilter; st = g_telStats; fps = g_telFps;
    portEXIT_CRITICAL(&g_telemetryMux);

    char buf[1024];
    int  n = snprintf(buf, sizeof(buf),
        "{\"frameId\":%u,\"fps\":%.1f,\"scanUs\":%u,\"components\":%u,"
        "\"best\":%d,\"colors\":[",
        (unsigned)r.frameId, fps, (unsigned)st.scanMicros,
        (unsigned)st.componentCount,
        r.bestColor == NO_COLOR ? -1 : (int)r.bestColor);

    for (int c = 0; c < COLOR_COUNT && n > 0 && n < (int)sizeof(buf); c++) {
        const ColorDetection &d = r.color[c];
        n += snprintf(buf + n, sizeof(buf) - n,
            "%s{\"name\":\"%s\",\"found\":%s,\"conf\":%u,\"cx\":%d,\"cy\":%d,"
            "\"w\":%d,\"h\":%d,\"px\":%u,\"fill\":%u,\"core\":%u,\"frag\":%u,\"edge\":%s}",
            c ? "," : "", colorName((ColorId)c), d.found ? "true" : "false",
            d.confidence, d.cx, d.cy,
            d.found ? (d.x1 - d.x0 + 1) : 0, d.found ? (d.y1 - d.y0 + 1) : 0,
            (unsigned)d.pixels, d.fillPct, d.corePct, d.fragments,
            d.touchesEdge ? "true" : "false");
    }

    n += snprintf(buf + n, sizeof(buf) - n, "],\"rejected\":{");
    for (int i = 1; i < REJ_REASON_COUNT && n > 0 && n < (int)sizeof(buf); i++)
        n += snprintf(buf + n, sizeof(buf) - n, "%s\"%s\":%u",
                      i > 1 ? "," : "", rejectReasonName(i), f.rejected[i]);
    n += snprintf(buf + n, sizeof(buf) - n, "}}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, n);
}

#define PART_BOUNDARY "teletubbyframe"
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_PART =
    "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t streamHandler(httpd_req_t *req) {
    if (httpd_resp_set_type(req, STREAM_CONTENT_TYPE) != ESP_OK) return ESP_FAIL;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Only the stream server task touches this (httpd serialises handlers on
    // one task), and only the vision loop reads it. Aligned 32-bit access is
    // atomic on Xtensa; `++` on a volatile is deprecated.
    g_streamClients = g_streamClients + 1;
    esp_err_t res = ESP_OK;
    char      part[80];

    while (res == ESP_OK) {
        if (xSemaphoreTake(g_frameReady, pdMS_TO_TICKS(2000)) != pdTRUE) continue;
        if (!g_running) break;

        uint8_t *jpg     = nullptr;
        size_t   jpgLen  = 0;
        bool     ok      = false;

        // Hold the staging buffer only for the draw + encode. The producer's
        // zero-timeout take means it just skips frames while we're in here.
        if (xSemaphoreTake(g_previewMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            drawOverlay(g_preview, g_previewResult);
            ok = fmt2jpg(g_preview, (size_t)PREVIEW_W * PREVIEW_H * 2,
                         PREVIEW_W, PREVIEW_H, PIXFORMAT_RGB565,
                         PREVIEW_JPEG_QUALITY, &jpg, &jpgLen);
            xSemaphoreGive(g_previewMutex);
        }
        if (!ok) continue;

        int hl = snprintf(part, sizeof(part), STREAM_PART, (unsigned)jpgLen);
        res = httpd_resp_send_chunk(req, part, hl);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)jpg, jpgLen);
        free(jpg);
    }

    g_streamClients = g_streamClients - 1;
    return res;
}

static const char PAGE_HTML[] PROGMEM = R"HTML(<!doctype html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Teletubby Vision</title><style>
body{font:14px system-ui,sans-serif;background:#14161a;color:#e6e8ec;margin:0;padding:16px}
h1{font-size:16px;margin:0 0 12px;color:#9aa4b2;font-weight:600}
.wrap{display:flex;gap:16px;flex-wrap:wrap;align-items:flex-start}
img{width:480px;max-width:100%;image-rendering:pixelated;border:1px solid #2a2f38;border-radius:6px;background:#000}
table{border-collapse:collapse;font-variant-numeric:tabular-nums}
td,th{padding:4px 10px;border-bottom:1px solid #252a33;text-align:right}
th{color:#7d8794;font-weight:500;text-align:right}
td:first-child,th:first-child{text-align:left}
.on{color:#4ade80;font-weight:600}.off{color:#4b5563}
.hd{margin-bottom:8px;color:#7d8794}
.warn{background:#3b2a12;color:#fbbf24;padding:8px 12px;border-radius:6px;margin-bottom:12px}
</style></head><body>
<div class="warn">Debug telemetry &mdash; turn WiFi off for competition.</div>
<h1>Teletubby Vision</h1>
<div class="wrap">
<img id="v" alt="stream">
<div><div class="hd" id="hd">connecting...</div>
<table><thead><tr><th>colour</th><th>conf</th><th>cx</th><th>cy</th><th>w&times;h</th>
<th>px</th><th>fill</th><th>core</th><th>frag</th></tr></thead>
<tbody id="tb"></tbody></table>
<div class="hd" id="rej" style="margin-top:10px"></div></div>
</div>
<script>
document.getElementById('v').src='http://'+location.hostname+':81/stream';
async function tick(){
 try{
  const d=await (await fetch('/data',{cache:'no-store'})).json();
  document.getElementById('hd').textContent=
    'frame '+d.frameId+'  |  '+d.fps.toFixed(1)+' fps  |  scan '+d.scanUs+' us  |  '
    +d.components+' components  |  best: '+(d.best<0?'none':d.colors[d.best].name);
  document.getElementById('tb').innerHTML=d.colors.map(c=>
    '<tr><td class="'+(c.found?'on':'off')+'">'+c.name+'</td><td>'+(c.found?c.conf:'-')
    +'</td><td>'+(c.found?c.cx:'-')+'</td><td>'+(c.found?c.cy:'-')
    +'</td><td>'+(c.found?c.w+'&times;'+c.h:'-')+'</td><td>'+(c.found?c.px:'-')
    +'</td><td>'+(c.found?c.fill+'%':'-')+'</td><td>'+(c.found?c.core+'%':'-')
    +'</td><td>'+(c.found?(c.frag>1?c.frag:'')+(c.edge?' E':''):'-')+'</td></tr>').join('');
  const r=Object.entries(d.rejected).filter(([,v])=>v>0);
  document.getElementById('rej').textContent=r.length?
    'rejected: '+r.map(([k,v])=>k+'='+v).join('  '):'no candidates rejected';
 }catch(e){document.getElementById('hd').textContent='disconnected';}
}
setInterval(tick,250);tick();
</script></body></html>)HTML";

static esp_err_t pageHandler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

// ---------------- lifecycle ----------------

static bool startServers() {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.core_id       = 0;          // keep HTTP off the vision core
    cfg.server_port   = 80;
    cfg.ctrl_port     = 32768;
    cfg.max_uri_handlers = 4;

    httpd_uri_t pageUri = { "/",     HTTP_GET, pageHandler, nullptr };
    httpd_uri_t dataUri = { "/data", HTTP_GET, dataHandler, nullptr };

    if (httpd_start(&g_httpPage, &cfg) != ESP_OK) {
        Serial.println("[WIFI] Failed to start the page server.");
        return false;
    }
    httpd_register_uri_handler(g_httpPage, &pageUri);
    httpd_register_uri_handler(g_httpPage, &dataUri);

    // Separate server for MJPEG: the handler blocks for as long as a client is
    // watching, so it must not sit on the same task as the page and JSON.
    cfg.server_port = 81;
    cfg.ctrl_port   = 32769;
    httpd_uri_t streamUri = { "/stream", HTTP_GET, streamHandler, nullptr };
    if (httpd_start(&g_httpStream, &cfg) != ESP_OK) {
        Serial.println("[WIFI] Failed to start the stream server (telemetry still works).");
        g_httpStream = nullptr;
    } else {
        httpd_register_uri_handler(g_httpStream, &streamUri);
    }
    return true;
}

static bool allocPreview() {
    if (g_preview) return true;
    g_preview = (uint8_t *)heap_caps_malloc((size_t)PREVIEW_W * PREVIEW_H * 2,
                                            MALLOC_CAP_SPIRAM);
    if (!g_preview) {
        Serial.println("[WIFI] Could not allocate the preview buffer -- video disabled.");
        return false;
    }
    g_previewMutex = xSemaphoreCreateMutex();
    g_frameReady   = xSemaphoreCreateBinary();
    return g_previewMutex && g_frameReady;
}

static void announce(const IPAddress &ip) {
    Serial.println("\n**************************************************");
    Serial.println("*  WiFi DEBUG IS ON -- TURN IT OFF FOR COMPETITION *");
    Serial.println("**************************************************");
    Serial.printf("  page:      http://%s/\n", ip.toString().c_str());
    Serial.printf("  telemetry: http://%s/data\n", ip.toString().c_str());
    Serial.printf("  video:     http://%s:81/stream  (%d fps target)\n",
                  ip.toString().c_str(), g_streamFps);
    Serial.println("  Streaming video costs frame rate; telemetry alone is cheap.");
    Serial.println("  'wifi off' when you're done.\n");
}

bool wifiDebugStartAP() {
    if (g_running) { Serial.println("[WIFI] Already running."); return true; }
    allocPreview();

    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(WIFI_DEBUG_AP_SSID, WIFI_DEBUG_AP_PASS)) {
        Serial.println("[WIFI] softAP() failed.");
        WiFi.mode(WIFI_OFF);
        return false;
    }
    g_running = true;
    if (!startServers()) { wifiDebugStop(); return false; }

    Serial.printf("[WIFI] Access point \"%s\", password \"%s\"\n",
                  WIFI_DEBUG_AP_SSID, WIFI_DEBUG_AP_PASS);
    announce(WiFi.softAPIP());
    return true;
}

bool wifiDebugStartSTA(const char *ssid, const char *pass) {
    if (g_running) { Serial.println("[WIFI] Already running."); return true; }
    allocPreview();

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    Serial.printf("[WIFI] Joining \"%s\"", ssid);
    uint32_t until = millis() + 15000;
    while (WiFi.status() != WL_CONNECTED && (int32_t)(millis() - until) < 0) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WIFI] Could not connect -- check SSID/password (2.4 GHz only).");
        WiFi.mode(WIFI_OFF);
        return false;
    }
    g_running = true;
    if (!startServers()) { wifiDebugStop(); return false; }
    announce(WiFi.localIP());
    return true;
}

void wifiDebugStop() {
    if (!g_running) { Serial.println("[WIFI] Already off."); return; }
    g_running = false;
    // Wake any blocked stream handler so it can notice and unwind.
    if (g_frameReady) xSemaphoreGive(g_frameReady);
    delay(50);

    if (g_httpStream) { httpd_stop(g_httpStream); g_httpStream = nullptr; }
    if (g_httpPage)   { httpd_stop(g_httpPage);   g_httpPage   = nullptr; }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    g_streamClients = 0;
    Serial.println("[WIFI] Off. Radio down, vision back to full speed.");
}

void setStreamFps(int fps) {
    if (fps < 1)  fps = 1;
    if (fps > 15) fps = 15;
    g_streamFps = fps;
}
int getStreamFps() { return g_streamFps; }

void wifiDebugStatus() {
    if (!g_running) {
        Serial.println("[WIFI] OFF (this is the competition-safe state).");
        Serial.println("       'wifi ap' for a self-hosted network, or");
        Serial.println("       'wifi sta <ssid> <pass>' to join an existing 2.4 GHz one.");
        return;
    }
    IPAddress ip = (WiFi.getMode() == WIFI_AP) ? WiFi.softAPIP() : WiFi.localIP();
    Serial.printf("[WIFI] ON (%s)  ip=%s  stream clients=%d  target %d fps\n",
        WiFi.getMode() == WIFI_AP ? "access point" : "station",
        ip.toString().c_str(), g_streamClients, g_streamFps);
    Serial.printf("       page http://%s/   telemetry http://%s/data   video http://%s:81/stream\n",
        ip.toString().c_str(), ip.toString().c_str(), ip.toString().c_str());
    Serial.println("       *** turn this off for competition ***");
}

#endif // ENABLE_WIFI_DEBUG
