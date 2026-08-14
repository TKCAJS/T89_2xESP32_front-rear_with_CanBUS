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
#include <nvs.h>            // nvs_get_stats — diagnosing write failures

static const uint16_t CAL_VERSION = 1;

// Clutch servo travel limits — physically measured, never exceed these.
// They live here rather than in T89_gearbox_206.cpp so calValidate() can enforce them:
// SimpleServo::write() constrains to this range silently, so a stored angle outside it
// would persist and read back fine while the servo quietly ignored it.
#define CLUTCH_SERVO_MIN  42
#define CLUTCH_SERVO_MAX  137

struct CalConfig {
    uint16_t version;

    // --- relay timing ---
    uint16_t neutralDownMs;    // neutral / gear-1 down pulse (20-1000 ms)
    uint16_t neutralUpMs;      // neutral up pulse (20-1000 ms)
    uint16_t shiftUpMs;        // upshift relay pulse (20-1000 ms)
    uint16_t shiftDownMs;      // downshift relay pulse (20-1000 ms)

    // --- clutch servo angles (CLUTCH_SERVO_MIN..MAX, not 0-180) ---
    uint16_t clutchIdlePos;    // lever released / clutch fully engaged
    uint16_t clutchFullyPull;  // lever pulled / clutch disengaged

    // --- integrity (MUST remain last member) ---
    uint32_t crc;
};

static void calLoadDefaults(CalConfig &c) {
    c.version        = CAL_VERSION;
    c.neutralDownMs  = 40;
    c.neutralUpMs    = 40;
    c.shiftUpMs      = 150;
    c.shiftDownMs    = 150;
    c.clutchIdlePos  = CLUTCH_SERVO_MIN;   // 0/180 would be outside real travel
    c.clutchFullyPull = CLUTCH_SERVO_MAX;
    c.crc            = 0;
}

// Returns "" on success, or a human-readable rejection reason.
// Reject rather than clamp — firmware is the authority.
static String calValidate(const CalConfig &c) {
    if (c.neutralDownMs  < 20 || c.neutralDownMs  > 1000) return "neutralDownMs out of range (20-1000)";
    if (c.neutralUpMs    < 20 || c.neutralUpMs    > 1000) return "neutralUpMs out of range (20-1000)";
    if (c.shiftUpMs      < 20 || c.shiftUpMs      > 1000) return "shiftUpMs out of range (20-1000)";
    if (c.shiftDownMs    < 20 || c.shiftDownMs    > 1000) return "shiftDownMs out of range (20-1000)";
    // Enforce the servo's real travel, not 0-180. SimpleServo::write() clamps to these
    // limits without complaint, so accepting a wider range would store an angle the
    // servo never actually reaches — stored value and real travel silently disagreeing.
    if (c.clutchIdlePos < CLUTCH_SERVO_MIN || c.clutchIdlePos > CLUTCH_SERVO_MAX)
        return "clutchIdlePos out of range (" + String(CLUTCH_SERVO_MIN) + "-" + String(CLUTCH_SERVO_MAX) + ")";
    if (c.clutchFullyPull < CLUTCH_SERVO_MIN || c.clutchFullyPull > CLUTCH_SERVO_MAX)
        return "clutchFullyPull out of range (" + String(CLUTCH_SERVO_MIN) + "-" + String(CLUTCH_SERVO_MAX) + ")";
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
    if (!p.begin(CAL_NVS_NS, false)) {
        Serial.println("CalConfig: NVS begin failed for namespace '" + String(CAL_NVS_NS) + "'");
        return false;
    }
    size_t n = p.putBytes(CAL_NVS_KEY, &c, sizeof(c));
    p.end();

    if (n != sizeof(c)) {
        // The web layer can only report "nvs write failed", which says nothing about
        // why. putBytes gives no error code, so dump the partition stats instead —
        // exhaustion (free_entries near zero) looks identical to any other failure
        // from the outside. This write is field-agnostic: it stores the whole struct,
        // so a failure is never specific to one setting.
        nvs_stats_t st;
        if (nvs_get_stats(NULL, &st) == ESP_OK) {
            Serial.printf("CalConfig: NVS write failed (%u of %u bytes). "
                          "entries used=%u free=%u total=%u, namespaces=%u\n",
                          (unsigned)n, (unsigned)sizeof(c),
                          (unsigned)st.used_entries, (unsigned)st.free_entries,
                          (unsigned)st.total_entries, (unsigned)st.namespace_count);
        } else {
            Serial.printf("CalConfig: NVS write failed (%u of %u bytes), stats unavailable\n",
                          (unsigned)n, (unsigned)sizeof(c));
        }
        return false;
    }
    return true;
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
    doc["clutchFullyPull"] = c.clutchFullyPull;
}

// Overlay only keys present in the JSON doc onto an existing config.
// Lets the UI send partial or full updates; the whole result is then validated.
static void calJsonOverlay(const JsonDocument &doc, CalConfig &c) {
    if (doc["neutralDownMs"].is<int>())   c.neutralDownMs  = doc["neutralDownMs"].as<int>();
    if (doc["neutralUpMs"].is<int>())     c.neutralUpMs    = doc["neutralUpMs"].as<int>();
    if (doc["shiftUpMs"].is<int>())       c.shiftUpMs      = doc["shiftUpMs"].as<int>();
    if (doc["shiftDownMs"].is<int>())     c.shiftDownMs    = doc["shiftDownMs"].as<int>();
    if (doc["clutchIdlePos"].is<int>())   c.clutchIdlePos  = doc["clutchIdlePos"].as<int>();
    if (doc["clutchFullyPull"].is<int>()) c.clutchFullyPull = doc["clutchFullyPull"].as<int>();
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
                    int liveClutchIdlePos, int liveClutchFullyPull) {
    gCalMutex = xSemaphoreCreateMutex();
    CalConfig boot;
    if (!calNvsRead(boot)) {
        boot.version        = CAL_VERSION;
        boot.neutralDownMs  = (uint16_t)liveNeutralDownMs;
        boot.neutralUpMs    = (uint16_t)liveNeutralUpMs;
        boot.shiftUpMs      = (uint16_t)liveShiftUpMs;
        boot.shiftDownMs    = (uint16_t)liveShiftDownMs;
        boot.clutchIdlePos  = (uint16_t)liveClutchIdlePos;
        boot.clutchFullyPull = (uint16_t)liveClutchFullyPull;
        calSeal(boot);
        calNvsWrite(boot);
        Serial.println("CalConfig: NVS empty/invalid — seeded from live config");
    } else {
        Serial.println("CalConfig: loaded from NVS");
    }
    gCalConfig = boot;
}
