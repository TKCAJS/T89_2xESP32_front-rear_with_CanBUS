#pragma once
// CalConfig — versioned NVS blob for gearbox calibration scalars.
// One key ("cfg") in one namespace ("t89cfg"). All writes are transactional:
// validate -> seal -> write -> read-back -> promote, so a bad write never
// corrupts the live config. On first boot the blob is seeded from the current
// live globals so the NVS doesn't start with silent zeros.
//
// Thread safety: gCalMutex guards gCalConfig. Take it any time you read or
// write the global struct from a different context (ISR, separate task, etc.)

#include <Preferences.h>
#include <ArduinoJson.h>
#include <freertos/semphr.h>

static const uint16_t CAL_VERSION = 1;

struct CalConfig {
    uint16_t version;

    // --- relay timing ---
    uint16_t neutralDownMs;    // neutral / gear-1 down pulse (20-1000 ms)
    uint16_t neutralUpMs;      // neutral up pulse (20-1000 ms)
    uint16_t shiftUpMs;        // upshift relay pulse (20-1000 ms)
    uint16_t shiftDownMs;      // downshift relay pulse (20-1000 ms)

    // --- clutch servo angles ---
    uint16_t clutchIdlePos;    // lever released / clutch fully engaged (0-180 °)
    uint16_t clutchEngagePos;  // lever pulled / clutch disengaged (0-180 °)

    // --- integrity (MUST remain last member) ---
    uint32_t crc;
};

static void calLoadDefaults(CalConfig &c) {
    c.version        = CAL_VERSION;
    c.neutralDownMs  = 40;
    c.neutralUpMs    = 40;
    c.shiftUpMs      = 150;
    c.shiftDownMs    = 150;
    c.clutchIdlePos  = 0;
    c.clutchEngagePos = 180;
    c.crc            = 0;
}

// Returns "" on success, or a human-readable rejection reason.
// Reject rather than clamp — firmware is the authority.
static String calValidate(const CalConfig &c) {
    if (c.neutralDownMs  < 20 || c.neutralDownMs  > 1000) return "neutralDownMs out of range (20-1000)";
    if (c.neutralUpMs    < 20 || c.neutralUpMs    > 1000) return "neutralUpMs out of range (20-1000)";
    if (c.shiftUpMs      < 20 || c.shiftUpMs      > 1000) return "shiftUpMs out of range (20-1000)";
    if (c.shiftDownMs    < 20 || c.shiftDownMs    > 1000) return "shiftDownMs out of range (20-1000)";
    if (c.clutchIdlePos  > 180)                           return "clutchIdlePos out of range (0-180)";
    if (c.clutchEngagePos > 180)                          return "clutchEngagePos out of range (0-180)";
    return "";
}

// Portable table-less CRC32 (IEEE 802.3 polynomial, same as the reference .ino).
static uint32_t calCrc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1));
    }
    return ~crc;
}

static void calSeal(CalConfig &c) {
    c.version = CAL_VERSION;
    c.crc = calCrc32((const uint8_t *)&c, sizeof(CalConfig) - sizeof(c.crc));
}

static bool calConfigValid(const CalConfig &c) {
    if (c.version != CAL_VERSION) return false;
    uint32_t want = calCrc32((const uint8_t *)&c, sizeof(CalConfig) - sizeof(c.crc));
    return want == c.crc;
}

// ---------------------------------------------------------------------------
//  NVS persistence — one blob, one key
// ---------------------------------------------------------------------------
static const char *CAL_NVS_NS  = "t89cfg";
static const char *CAL_NVS_KEY = "cfg";

static bool calNvsRead(CalConfig &out) {
    Preferences p;
    p.begin(CAL_NVS_NS, true);
    bool ok = false;
    if (p.getBytesLength(CAL_NVS_KEY) == sizeof(CalConfig)) {
        CalConfig tmp;
        p.getBytes(CAL_NVS_KEY, &tmp, sizeof(tmp));
        if (calConfigValid(tmp)) { out = tmp; ok = true; }
    }
    p.end();
    return ok;
}

static bool calNvsWrite(const CalConfig &c) {
    Preferences p;
    p.begin(CAL_NVS_NS, false);
    size_t n = p.putBytes(CAL_NVS_KEY, &c, sizeof(c));
    p.end();
    return n == sizeof(c);
}

// ---------------------------------------------------------------------------
//  JSON <-> struct
// ---------------------------------------------------------------------------
static void calToJson(const CalConfig &c, JsonDocument &doc) {
    doc["version"]        = c.version;
    doc["neutralDownMs"]  = c.neutralDownMs;
    doc["neutralUpMs"]    = c.neutralUpMs;
    doc["shiftUpMs"]      = c.shiftUpMs;
    doc["shiftDownMs"]    = c.shiftDownMs;
    doc["clutchIdlePos"]  = c.clutchIdlePos;
    doc["clutchEngagePos"] = c.clutchEngagePos;
}

// Overlay only keys present in the JSON doc onto an existing config.
// Lets the UI send partial or full updates; the whole result is then validated.
static void calJsonOverlay(const JsonDocument &doc, CalConfig &c) {
    if (doc["neutralDownMs"].is<int>())   c.neutralDownMs  = doc["neutralDownMs"].as<int>();
    if (doc["neutralUpMs"].is<int>())     c.neutralUpMs    = doc["neutralUpMs"].as<int>();
    if (doc["shiftUpMs"].is<int>())       c.shiftUpMs      = doc["shiftUpMs"].as<int>();
    if (doc["shiftDownMs"].is<int>())     c.shiftDownMs    = doc["shiftDownMs"].as<int>();
    if (doc["clutchIdlePos"].is<int>())   c.clutchIdlePos  = doc["clutchIdlePos"].as<int>();
    if (doc["clutchEngagePos"].is<int>()) c.clutchEngagePos = doc["clutchEngagePos"].as<int>();
}

// ---------------------------------------------------------------------------
//  Shared live config + mutex
// ---------------------------------------------------------------------------
static CalConfig         gCalConfig;
static SemaphoreHandle_t gCalMutex = nullptr;

// Call once from setup(), AFTER loadConfig() has populated the live globals,
// so the first-boot NVS seed reflects real calibration rather than zeroes.
static void calInit(int liveNeutralDownMs, int liveNeutralUpMs,
                    int liveShiftUpMs, int liveShiftDownMs,
                    int liveClutchIdlePos, int liveClutchEngagePos) {
    gCalMutex = xSemaphoreCreateMutex();
    CalConfig boot;
    if (!calNvsRead(boot)) {
        boot.version        = CAL_VERSION;
        boot.neutralDownMs  = (uint16_t)liveNeutralDownMs;
        boot.neutralUpMs    = (uint16_t)liveNeutralUpMs;
        boot.shiftUpMs      = (uint16_t)liveShiftUpMs;
        boot.shiftDownMs    = (uint16_t)liveShiftDownMs;
        boot.clutchIdlePos  = (uint16_t)liveClutchIdlePos;
        boot.clutchEngagePos = (uint16_t)liveClutchEngagePos;
        calSeal(boot);
        calNvsWrite(boot);
        Serial.println("CalConfig: NVS empty/invalid — seeded from live config");
    } else {
        Serial.println("CalConfig: loaded from NVS");
    }
    gCalConfig = boot;
}
