# T89 2x ESP32 Gearbox Control

## Web UI

Pages live in `src/main_node/data/`. Deploy changes with PlatformIO **Upload Filesystem Image** — no recompile needed.

### Page roles

| Page | Role |
|---|---|
| `index.html` | Live status, gauges, bite-point voltage only. No config forms. |
| `calibration.html` | Live testing/discovery wizard. No NVS writes except piecewise zone. |
| `nvsconfig.html` | Single authority for all NVS saves. |

### Nav bar
All three pages share the same 3-button flex row at the top:
- **Home** — `#2196F3` blue
- **Calibration** — `#FF9800` orange
- **NVS Config** — `#9C27B0` purple

Current page button is muted (`opacity:0.5; pointer-events:none;`).

### Save authority — nvsconfig.html
Uses `POST /api/config` (JSON) + firmware read-back. Fields: `neutralDownMs`, `neutralUpMs`, `shiftDownMs`, `shiftUpMs`, `clutchIdlePos`, `clutchEngagePos`. Stored as a CRC-sealed `CalConfig` blob in NVS namespace `t89cfg`.

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
