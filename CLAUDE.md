# T89 2x ESP32 Gearbox Control

## Git: commit to `main`, never branch

Commit and push directly to `main`. **Do not create feature branches**, and do not
leave work sitting on one. This overrides the general "if on the default branch,
branch first" default — it does not apply to this project.

There is one developer working across several machines, with GitHub as the single
source of truth. Every machine sits on `main` and pulls `main`, so work parked on a
branch is invisible to all of them. This has already cost real time: in
`P4Display_node`, six commits including a verified touch-panel fix sat on a branch
while another machine pulled `main`, got none of it, and reflashed a build that
predated the fix. Touch stopped working and the work looked lost.

If a stray branch does exist, merge it back into `main` and delete it rather than
continuing on it.

At the start of a session, `git pull` on `main` first. If `main` looks older than
expected, check `git branch -r` for stray branches before concluding work is missing.

## Web UI

### One copy of the pages, two envs

Pages live in `src/main_node/data/` — **one copy, used by both envs.**

| Env | Serves pages from | Deploy |
|---|---|---|
| `main_node_integratedweb` | firmware image | **lead build** — rebuild + **Upload** |
| `main_node` | LittleFS | **Upload Filesystem Image** — no recompile |

`main_node_integratedweb` is the build to use. It compiles the pages into the firmware
with `board_build.embed_txtfiles`, so a single Upload ships firmware and pages together
and they cannot fall out of step. `main_node` is kept as the LittleFS fallback; it needs
a separate Upload Filesystem Image, and flashing it without one leaves the old pages in
place, which reads exactly like "my changes didn't apply".

There used to be a second, duplicate copy of the pages under
`src/main_node_integratedweb/data/` for the embedded build to compile. It drifted, and
it was expensive: that copy of `calibration.html` had been split to load a
`/calibration.js` that was never created and never routed, so it silently lost ~760
lines of JavaScript — the embedded build served a dead page while LittleFS served a
working one. Both envs now embed/serve the same directory. **Do not reintroduce a second
copy.**

### Embedded build gotchas (`main_node_integratedweb`)

- Needs `-DWEBINTERFACE_USE_EMBEDDED` in `build_flags`. Without it the build compiles
  the LittleFS path instead and serves nothing — `embed_txtfiles` alone does nothing.
- objcopy derives symbol names from the **file path**, so the externs are
  `_binary_src_main_node_data_<name>_html_start` / `_end`. Moving or renaming
  `src/main_node/data/` renames every symbol and breaks the link.
- Adding a page means three edits: the file, its `embed_txtfiles` line in
  `platformio.ini`, and its extern pair + route in `WebInterface.h`.
- Serve embedded bytes with `setContentLength()` + `sendContent()`. `sendHeader()`
  appends rather than replaces, and `send()` adds its own `Content-Length`, so setting
  it by hand emits two conflicting values and browsers reject the page. JSON endpoints
  are unaffected, so the tell is live data working while no page loads.

### Page roles

| Page | Role |
|---|---|
| `index.html` | Live status, gauges, bite-point voltage only. No config forms. |
| `calibration.html` | Live testing/discovery wizard. No NVS writes except piecewise zone. |
| `nvsconfig.html` | Single authority for all NVS saves. |
| `piecewise.html` | Visual hall/servo zone editor. Reads travel extents; writes only the piecewise zone. |

### Nav bar
All four pages share the same 4-button flex row at the top, **in this order**:
- **Home** — `#2196F3` blue
- **Calibration** — `#FF9800` orange
- **NVS Config** — `#9C27B0` purple
- **Piecewise** — `#4CAF50` green

Current page button is muted (`opacity:0.5; pointer-events:none;`).

### Save authority — nvsconfig.html
Uses `POST /api/config` (JSON) + firmware read-back. Fields: `neutralDownMs`, `neutralUpMs`, `shiftDownMs`, `shiftUpMs`, `clutchIdlePos`, `clutchFullyPull`. Stored as a CRC-sealed `CalConfig` blob in NVS namespace `t89cfg`.

Hall calibration uses capture buttons hitting `/cmd?action=captureHall*` while live-polling `/sensorData`.

### Piecewise zone — calibration.html only
Stored in the separate `gearbox` Preferences namespace, not in `CalConfig`. Saved via `/cmd?action=savePiecewiseZone&...`. Has bounds validation (Hall: 0-4095, Servo: 0-180) and read-back via `/configData`. Cannot move to `/api/config` without firmware changes.

### Firmware endpoints (WebInterface.h)

| Endpoint | Method | Purpose |
|---|---|---|
| `/api/config` | GET | Read current CalConfig as JSON |
| `/api/config` | POST | Write CalConfig (transactional, CRC-sealed, read-back verified) |
| `/api/defaults` | POST | Restore factory defaults |
| `/configData` | GET | Full config including piecewise fields and hall calibration |
| `/sensorData` | GET | Live sensor readings (hall, clutch voltage, RPM, MPH, temp, etc.) |
| `/cmd?action=...` | GET | Captures, servo control, piecewise save, curve type |
