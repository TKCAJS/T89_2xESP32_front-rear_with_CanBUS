/**
 * T89 Main Node — Calibration Web Backend (skeleton)
 * ESP32-S3, Arduino framework (compiles in Arduino IDE and PlatformIO)
 *
 * Design goals: robust pit-side calibration, not racing.
 *   - JSON on the wire  <->  versioned C struct in RAM  <->  one CRC'd blob in NVS
 *   - Validate into a temp struct; only commit on full success
 *   - Read back from NVS after commit and echo it (closes the loop)
 *   - Blank/corrupt NVS always boots to compiled-in safe defaults
 *   - Web/WiFi pinned to core 1; CAN stays on core 0; shared via mutex
 *
 * PlatformIO lib_deps:
 *   bblanchon/ArduinoJson @ ^7
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// ---------------------------------------------------------------------------
//  Calibration data — flat scalars only
// ---------------------------------------------------------------------------
//  'version' first, 'crc' MUST stay last (CRC is computed over everything
//  before it). Bump CONFIG_VERSION whenever you change the field layout so
//  old/incompatible blobs are rejected and defaults are reloaded.
// ---------------------------------------------------------------------------
static const uint16_t CONFIG_VERSION = 1;

struct CalConfig {
  uint16_t version;

  // --- shift / clutch calibration ---
  uint16_t shiftUpMs;          // upshift relay pulse length
  uint16_t shiftDownMs;        // downshift relay pulse length
  uint16_t clutchEngageDeg;    // servo angle, clutch engaged
  uint16_t clutchDisengageDeg; // servo angle, clutch disengaged
  uint16_t clutchSettleMs;     // dwell after clutch move before shift
  uint16_t gearSettleMs;       // dwell after shift before next op
  uint16_t hallThreshold;      // clutch hall ADC threshold

  // --- integrity (must remain the last member) ---
  uint32_t crc;
};

// Compiled-in safe defaults. A node that loses its NVS must still be safe.
static void loadDefaults(CalConfig &c) {
  c.version            = CONFIG_VERSION;
  c.shiftUpMs          = 150;
  c.shiftDownMs        = 150;
  c.clutchEngageDeg    = 0;
  c.clutchDisengageDeg = 90;
  c.clutchSettleMs     = 200;
  c.gearSettleMs       = 300;
  c.hallThreshold      = 2000;
  c.crc                = 0; // filled in by sealConfig()
}

// Per-field bounds. Reject out-of-range writes rather than silently clamp.
// Returns "" on success, or a human-readable reason on failure.
static String validateConfig(const CalConfig &c) {
  if (c.shiftUpMs   < 20 || c.shiftUpMs   > 1000) return "shiftUpMs out of range (20-1000)";
  if (c.shiftDownMs < 20 || c.shiftDownMs > 1000) return "shiftDownMs out of range (20-1000)";
  if (c.clutchEngageDeg    > 180)                 return "clutchEngageDeg out of range (0-180)";
  if (c.clutchDisengageDeg > 180)                 return "clutchDisengageDeg out of range (0-180)";
  if (c.clutchSettleMs > 2000)                    return "clutchSettleMs out of range (0-2000)";
  if (c.gearSettleMs   > 2000)                    return "gearSettleMs out of range (0-2000)";
  if (c.hallThreshold  > 4095)                    return "hallThreshold out of range (0-4095)";
  return "";
}

// ---------------------------------------------------------------------------
//  CRC32 (portable, table-less) — over all bytes preceding the crc field
// ---------------------------------------------------------------------------
static uint32_t crc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc >> 1) ^ (0xEDB88320 & (-(int32_t)(crc & 1)));
  }
  return ~crc;
}

static void sealConfig(CalConfig &c) {
  c.version = CONFIG_VERSION;
  c.crc = crc32((const uint8_t *)&c, sizeof(CalConfig) - sizeof(c.crc));
}

static bool configValid(const CalConfig &c) {
  if (c.version != CONFIG_VERSION) return false;
  uint32_t want = crc32((const uint8_t *)&c, sizeof(CalConfig) - sizeof(c.crc));
  return want == c.crc;
}

// ---------------------------------------------------------------------------
//  NVS persistence — one blob, one key. Read back after write to confirm.
// ---------------------------------------------------------------------------
static Preferences prefs;
static const char *NVS_NAMESPACE = "t89cal";
static const char *NVS_KEY       = "cfg";

static bool nvsRead(CalConfig &out) {
  prefs.begin(NVS_NAMESPACE, true); // read-only
  size_t got = prefs.getBytesLength(NVS_KEY);
  bool ok = false;
  if (got == sizeof(CalConfig)) {
    CalConfig tmp;
    prefs.getBytes(NVS_KEY, &tmp, sizeof(tmp));
    if (configValid(tmp)) { out = tmp; ok = true; }
  }
  prefs.end();
  return ok;
}

static bool nvsWrite(const CalConfig &c) {
  prefs.begin(NVS_NAMESPACE, false); // read-write
  size_t n = prefs.putBytes(NVS_KEY, &c, sizeof(c));
  prefs.end();
  return n == sizeof(c);
}

// ---------------------------------------------------------------------------
//  Shared live config — guarded by a mutex (web task on core 1, CAN on core 0)
// ---------------------------------------------------------------------------
static CalConfig         gConfig;
static SemaphoreHandle_t gConfigMutex;

// ---------------------------------------------------------------------------
//  JSON <-> struct
// ---------------------------------------------------------------------------
static void configToJson(const CalConfig &c, JsonDocument &doc) {
  doc["version"]            = c.version;
  doc["shiftUpMs"]          = c.shiftUpMs;
  doc["shiftDownMs"]        = c.shiftDownMs;
  doc["clutchEngageDeg"]    = c.clutchEngageDeg;
  doc["clutchDisengageDeg"] = c.clutchDisengageDeg;
  doc["clutchSettleMs"]     = c.clutchSettleMs;
  doc["gearSettleMs"]       = c.gearSettleMs;
  doc["hallThreshold"]      = c.hallThreshold;
}

// Overlay only the keys present in the doc onto an existing config.
// Lets the UI send partial or full updates; the result is validated as a whole.
static void jsonOverlayConfig(const JsonDocument &doc, CalConfig &c) {
  if (doc["shiftUpMs"].is<uint16_t>())          c.shiftUpMs          = doc["shiftUpMs"];
  if (doc["shiftDownMs"].is<uint16_t>())        c.shiftDownMs        = doc["shiftDownMs"];
  if (doc["clutchEngageDeg"].is<uint16_t>())    c.clutchEngageDeg    = doc["clutchEngageDeg"];
  if (doc["clutchDisengageDeg"].is<uint16_t>()) c.clutchDisengageDeg = doc["clutchDisengageDeg"];
  if (doc["clutchSettleMs"].is<uint16_t>())     c.clutchSettleMs     = doc["clutchSettleMs"];
  if (doc["gearSettleMs"].is<uint16_t>())       c.gearSettleMs       = doc["gearSettleMs"];
  if (doc["hallThreshold"].is<uint16_t>())      c.hallThreshold      = doc["hallThreshold"];
}

// ---------------------------------------------------------------------------
//  Web server
// ---------------------------------------------------------------------------
static WebServer server(80);

static void sendConfigJson(const CalConfig &c, int code = 200) {
  JsonDocument doc;
  configToJson(c, doc);
  String out;
  serializeJson(doc, out);
  server.send(code, "application/json", out);
}

static void handleGetConfig() {
  CalConfig snap;
  xSemaphoreTake(gConfigMutex, portMAX_DELAY);
  snap = gConfig;
  xSemaphoreGive(gConfigMutex);
  sendConfigJson(snap);
}

static void handlePostConfig() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"empty body\"}");
    return;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, "application/json",
                String("{\"error\":\"bad json: ") + err.c_str() + "\"}");
    return;
  }

  // Work on a copy of the live config, overlay incoming fields, validate.
  CalConfig candidate;
  xSemaphoreTake(gConfigMutex, portMAX_DELAY);
  candidate = gConfig;
  xSemaphoreGive(gConfigMutex);

  jsonOverlayConfig(doc, candidate);

  String why = validateConfig(candidate);
  if (why.length()) {
    server.send(400, "application/json", String("{\"error\":\"") + why + "\"}");
    return;
  }

  // Seal (version + crc) and commit to NVS.
  sealConfig(candidate);
  if (!nvsWrite(candidate)) {
    server.send(500, "application/json", "{\"error\":\"nvs write failed\"}");
    return;
  }

  // Read back from NVS so the response reflects what actually landed.
  CalConfig stored;
  if (!nvsRead(stored)) {
    server.send(500, "application/json", "{\"error\":\"nvs readback failed\"}");
    return;
  }

  // Promote to live config.
  xSemaphoreTake(gConfigMutex, portMAX_DELAY);
  gConfig = stored;
  xSemaphoreGive(gConfigMutex);

  sendConfigJson(stored); // echo the persisted values
}

static void handlePostDefaults() {
  CalConfig def;
  loadDefaults(def);
  sealConfig(def);
  if (!nvsWrite(def)) {
    server.send(500, "application/json", "{\"error\":\"nvs write failed\"}");
    return;
  }
  CalConfig stored;
  nvsRead(stored);
  xSemaphoreTake(gConfigMutex, portMAX_DELAY);
  gConfig = stored;
  xSemaphoreGive(gConfigMutex);
  sendConfigJson(stored);
}

// ---------------------------------------------------------------------------
//  Web task — runs on core 1, isolated from CAN on core 0
// ---------------------------------------------------------------------------
static void webTask(void *param) {
  // SoftAP: deterministic IP, no dependency on pit WiFi infrastructure.
  WiFi.mode(WIFI_AP);
  WiFi.softAP("T89-CAL", "changeme123");  // <-- set your own credentials
  // Default SoftAP IP is 192.168.4.1

  server.on("/api/config",   HTTP_GET,  handleGetConfig);
  server.on("/api/config",   HTTP_POST, handlePostConfig);
  server.on("/api/defaults", HTTP_POST, handlePostDefaults);
  // TODO: server.serveStatic("/", LittleFS, "/index.html"); for the UI page
  server.begin();

  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ---------------------------------------------------------------------------
//  Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("T89 Main — calibration backend");

  gConfigMutex = xSemaphoreCreateMutex();

  // Load config; fall back to defaults on blank/corrupt/version-mismatch NVS.
  CalConfig boot;
  if (nvsRead(boot)) {
    Serial.println("Calibration loaded from NVS");
  } else {
    Serial.println("NVS invalid/empty -> loading defaults");
    loadDefaults(boot);
    sealConfig(boot);
    nvsWrite(boot);          // persist defaults so first boot is consistent
  }
  gConfig = boot;

  // Web/WiFi on core 1.
  xTaskCreatePinnedToCore(webTask, "web", 8192, nullptr, 1, nullptr, 1);

  // TODO: start your CAN/TWAI + shift logic here (this runs core 0 via loop()).
}

void loop() {
  // Core 0: CAN RX, heartbeat, shift logic.
  // Read calibration safely whenever you need it:
  //
  //   CalConfig cfg;
  //   xSemaphoreTake(gConfigMutex, portMAX_DELAY);
  //   cfg = gConfig;
  //   xSemaphoreGive(gConfigMutex);
  //   ... use cfg.shiftUpMs etc ...
  //
  delay(10);
}
