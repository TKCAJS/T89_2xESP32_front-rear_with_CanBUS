# T89 Main Node — Calibration Web Interface: Implementation Brief

## Goal
Build a pit-only web calibration interface on the ESP32-S3 main node. It edits a
set of flat scalar calibration values and pushes them back to the controller's
NVS (non-volatile storage). Priority is **robust, predictable behaviour**, not
bandwidth or speed — it is never used during racing, only stationary in the pit.

## Environment / constraints
- Target: ESP32-S3, Arduino framework.
- Must compile in **both** Arduino IDE and PlatformIO. No code that only works in one.
- JSON library: **ArduinoJson v7** (`bblanchon/ArduinoJson @ ^7`). Use the v7 API
  (elastic `JsonDocument`, `is<T>()` checks) — do NOT use the removed
  `StaticJsonDocument`/`DynamicJsonDocument` v6 API.
- Web UI: **plain HTML + vanilla JS, no framework, no build step.** Single page,
  served from the device. No external CDN dependencies (pit has no internet).
- Persistence: ESP32 `Preferences` (NVS).
- Network: device runs as its own **SoftAP** (no dependency on pit WiFi).

## Architecture (already decided — implement to this)
Three distinct representations, kept separate:

```
JSON on the wire  <->  versioned C struct in RAM  <->  one CRC'd blob in NVS
```

- All calibration lives in a single `CalConfig` struct of flat scalars.
- The struct carries a `version` field (first) and a `crc` field (last).
- The whole struct is stored as **one blob under one NVS key** — one write,
  not scattered keys.
- On boot: read blob, check size + version + CRC. If any check fails, load
  compiled-in **safe defaults** and persist them. A blank/corrupt NVS must
  never brick the node.
- Writes are transactional: parse JSON into a *copy* of the live config,
  validate the whole thing against per-field bounds, and only on success seal
  (set version + CRC), write to NVS, **read back from NVS**, then promote to
  the live struct. The HTTP response echoes the persisted (read-back) values so
  the browser confirms what actually landed.
- Web/WiFi runs in a FreeRTOS task **pinned to core 1**; CAN/shift logic stays
  on core 0 (the `loop()`). The shared live config is guarded by a **mutex**.

## Reference implementation (the backend already exists)
`T89_main_calibration_web.ino` is a working backend skeleton that implements all
of the above. Treat it as the source of truth for the backend. It provides:

- `struct CalConfig` (flat scalars + `version` + `crc`)
- `loadDefaults()`, `validateConfig()` (per-field bounds, reject-don't-clamp)
- `crc32()`, `sealConfig()`, `configValid()`
- `nvsRead()` / `nvsWrite()` (single-blob Preferences)
- `configToJson()` / `jsonOverlayConfig()` (partial-or-full overlay)
- Mutex-guarded `gConfig`
- REST endpoints on a `WebServer` in a core-1 task:
  - `GET  /api/config`   → current calibration as JSON
  - `POST /api/config`   → validate → commit → read back → echo JSON
  - `POST /api/defaults` → restore factory defaults
- SoftAP setup (`T89-CAL`, default IP 192.168.4.1)

Current calibration fields (adjust names/bounds to match the real controller):
`shiftUpMs`, `shiftDownMs`, `clutchEngageDeg`, `clutchDisengageDeg`,
`clutchSettleMs`, `gearSettleMs`, `hallThreshold`.

## What to implement

### 1. Calibration web page (`data/index.html`)
- Single self-contained page (inline CSS + JS is fine), served from LittleFS via
  `server.serveStatic("/", LittleFS, "/index.html")`.
- On load, `GET /api/config` and populate one input per field.
- "Save" button → `POST /api/config` with the edited values as JSON; on the
  response, repopulate the form from the echoed (persisted) values so the user
  sees exactly what was stored.
- "Restore defaults" button → `POST /api/defaults`, repopulate from response.
- Client-side validation that **mirrors the firmware bounds exactly** (same
  min/max per field). Client validation is convenience only — the firmware
  remains the authority and rejects bad writes with a reason.
- Show success/error feedback from the HTTP status + JSON `error` field.
- No `<form>` submit/page reload — use `fetch()` and button `onclick` handlers.

### 2. LittleFS serving
- Initialise LittleFS in setup; if mount fails, log and continue (API still works).
- Wire up `serveStatic` for the page in the web task (there is a `TODO` marker
  for this in the reference `.ino`).
- Document how to upload the `data/` folder (PlatformIO `Upload Filesystem
  Image`, or the Arduino LittleFS uploader plugin).

### 3. Live readout endpoint (optional, do after the page works)
- `GET /api/live` returning current sensor values as JSON (e.g. clutch hall ADC,
  gear position, RPM) so the user can calibrate against live readings.
- Page polls it every ~200 ms with `fetch()` and displays the values. Use
  **polling, not SSE/WebSocket** — for flat-scalar calibration polling is the
  lower-risk option and avoids keeping a stream alive.
- The live values are read-only and must not touch the calibration struct.

### 4. Integration with the existing main controller
- The reference `.ino` is standalone. Merge its config/NVS/web logic into the
  real main-node firmware **without** disturbing the existing CAN/TWAI + shift
  logic on core 0.
- Replace the placeholder credentials and confirm the SoftAP SSID/password.
- Make the existing shift logic read calibration via the mutex-guarded snapshot
  pattern shown in `loop()`.

## Optional niceties (only if asked / time permits)
- JSON profile download/upload: `GET /api/config` already returns the full
  profile; add a "Download" (save JSON file) and "Upload" (POST the file) to the
  page so a known-good calibration can be backed up and restored.

## Definition of done
1. Page loads on the SoftAP, shows current values from NVS.
2. Editing + Save persists to NVS; a power-cycle retains the values.
3. Out-of-bounds input is rejected by the firmware with a clear message; no
   partial writes.
4. Wiping NVS (or first boot) yields safe defaults, not a crash.
5. The web/save path never stalls the core-0 CAN loop.
6. Compiles clean in PlatformIO (and Arduino IDE).
