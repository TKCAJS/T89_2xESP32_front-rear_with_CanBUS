#!/usr/bin/env python3
"""
webpreview.py - local stand-in for the main_node web server, so the pages in
src/main_node/data/ can be edited and reloaded without a build.

The lead env (main_node_integratedweb) bakes those files into the firmware via
board_build.embed_txtfiles, so on device every CSS tweak costs a full compile and
flash. This serves the same directory over HTTP using the same route table as
WebInterface::setupRoutes() - which is the part that matters, because the nav links
are extensionless routes ("/calibration", not "/calibration.html"). A plain static
file server (VS Code Live Preview included) 404s or rejects every one of them.

    python tools/webpreview.py                        # stub JSON, no hardware
    python tools/webpreview.py --device 192.168.4.1   # live data from the node

With --device the HTML stays local while every API call is proxied to the real
node, so layout work runs against real telemetry and still needs no flash.

Caveat: this exercises markup and layout only, not the embedded-serving path.
serveEmbeddedFile() trims a trailing NUL and sets Content-Length by hand, so a
truncation bug there still only shows up on device.
"""

import argparse
import json
import math
import mimetypes
import os
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(REPO_ROOT, "src", "main_node", "data")

# Mirrors the page routes in WebInterface::setupRoutes(). Keep in step when a page
# is added there, otherwise the new nav link works on device but not here.
PAGE_ROUTES = {
    "/": "index.html",
    "/index.html": "index.html",
    "/calibration": "calibration.html",
    "/piecewise": "piecewise.html",
    "/nvsconfig": "nvsconfig.html",
    "/hello": "hello.html",
}

# Everything the pages fetch(). Proxied when --device is given, stubbed otherwise.
API_ROUTES = (
    "/sensorData", "/configData", "/shiftStats", "/shiftLogs", "/cmd",
    "/api/config", "/api/defaults", "/update", "/updateHallCurve",
)

START = time.monotonic()

# Stub CalConfig, seeded from calLoadDefaults() in CalConfig.h. POST /api/config
# mutates it so the NVS page's write-then-read-back round trip behaves as on device.
stub_cal = {
    "version": 1,
    "neutralDownMs": 40,
    "neutralUpMs": 40,
    "shiftUpMs": 150,
    "shiftDownMs": 150,
    "clutchIdlePos": 42,     # CLUTCH_SERVO_DEFAULT_MIN
    "clutchFullyPull": 137,  # CLUTCH_SERVO_DEFAULT_MAX
}


def _wave():
    """0..1 triangle-ish sweep, ~10 s period - makes stubbed gauges actually move."""
    return (math.sin((time.monotonic() - START) * 0.6) + 1.0) / 2.0


def stub_sensor_data():
    """Key-for-key copy of WebInterface::handleSensorData()."""
    w = _wave()
    hall = int(300 + w * 2500)
    return {
        "shiftInProgress": False,
        "waitingForClutch": False,
        "wifiEnabled": True,
        "apIP": "192.168.4.1",
        "hallLeft": hall,
        "hallRight": int(hall * 0.95),
        "hallValue": hall,
        "clutchVoltage": round(0.8 + w * 2.4, 3),
        "clutchDisengaged": w > 0.7,
        "shiftSequenceState": 0,
        "currentGear": "N",
        "softwareVersion": 214.0,
        "servoPosition": round(stub_cal["clutchIdlePos"]
                               + w * (stub_cal["clutchFullyPull"] - stub_cal["clutchIdlePos"]), 1),
        "clutchDisengageV": 2.450,
        "clutchJustEngagedV": 1.780,
        "clutchJustEngaged": 0.4 < w < 0.7,
        "adsFault": False,
        "adsFailTotal": 0,
        "adsRecoveries": 0,
        "maxLoopUs": 1800,
        "loopStalls": 0,
        "currentRpm": round(1200 + w * 7000, 1),
        "currentTemp": round(72.0 + w * 26.0, 1),
        "pumpDuty": int(w * 100),
        "currentMph": 0,
        "shiftTimingActive": False,
        "hallCurveName": "piecewise",
        "hallCurveStrength": 1.0,
        "servoOverride": False,
    }


def stub_config_data():
    """Key-for-key copy of WebInterface::handleConfigData()."""
    lo = min(stub_cal["clutchIdlePos"], stub_cal["clutchFullyPull"])
    hi = max(stub_cal["clutchIdlePos"], stub_cal["clutchFullyPull"])
    return {
        "neutralDownMs": stub_cal["neutralDownMs"],
        "neutralUpMs": stub_cal["neutralUpMs"],
        "shiftDownMs": stub_cal["shiftDownMs"],
        "shiftUpMs": stub_cal["shiftUpMs"],
        "clutchIdlePos": stub_cal["clutchIdlePos"],
        "clutchFullyPull": stub_cal["clutchFullyPull"],
        "hallCurveType": 2,
        "hallCurveStrength": 1.00,
        "hallMin": 280,
        "hallMax": 2950,
        "clutchDisengageV": 2.450,
        "clutchJustEngagedV": 1.780,
        "clutchServoMin": lo,
        "clutchServoMax": hi,
        "hallBiteStart": 900,
        "hallBiteEnd": 2100,
        "servoBiteStart": 70,
        "servoBiteEnd": 110,
        "pwBlend": 25,
        "pin2RawMin": 310,
        "pin2RawMax": 2880,
        "nvsUsed": 214,
        "nvsFree": 416,
        "nvsTotal": 630,
        "nvsNs": 3,
    }


def stub_shift_stats():
    """Plain text, same layout as ShiftLogger::getStatistics()."""
    return ("Shift Statistics:\n"
            "Total Shifts: 42\n"
            "Failed Shifts: 1\n"
            "Success Rate: 97%\n"
            "Average Shift Time: 138ms\n"
            "Log Entries: 42/200")


def stub_shift_logs():
    """Same entry shape as ShiftLogger::logEntryToJson()."""
    gears = ["N", "1", "2", "3", "4", "5", "6"]
    logs = []
    for i in range(10):
        up = i % 3 != 2
        a, b = (i % 6, i % 6 + 1) if up else (i % 6 + 1, i % 6)
        logs.append({
            "timestamp": 1000 * (i + 1),
            "from": gears[a],
            "to": gears[b],
            "rpm": 6200 + i * 130,
            "time": 128 + (i * 7) % 40,
            "type": "upshift" if up else "downshift",
            "success": i != 4,
        })
    return logs


class Handler(BaseHTTPRequestHandler):
    device = None          # set from argv; None means stub mode
    server_version = "T89WebPreview/1.0"

    # -- response helpers ---------------------------------------------------

    def _send(self, code, ctype, body):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        # The whole point is edit-then-refresh, so never let anything cache.
        self.send_header("Cache-Control", "no-store, must-revalidate")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _send_json(self, obj, code=200):
        self._send(code, "application/json", json.dumps(obj))

    # -- routing ------------------------------------------------------------

    def do_GET(self):
        self._route()

    def do_HEAD(self):
        self._route()

    def do_POST(self):
        self._route()

    def _route(self):
        path = urlsplit(self.path).path

        if path in PAGE_ROUTES:
            return self._serve_file(PAGE_ROUTES[path])
        if path in API_ROUTES:
            if self.device:
                return self._proxy()
            return self._stub(path)
        # Anything else is a real asset request (favicon, images, any future
        # split-out .css/.js) - resolve it under data/ like LittleFS would.
        return self._serve_file(path.lstrip("/"))

    # -- static files -------------------------------------------------------

    def _serve_file(self, relpath):
        full = os.path.normpath(os.path.join(DATA_DIR, relpath))
        if not full.startswith(DATA_DIR) or not os.path.isfile(full):
            return self._send(404, "text/plain", "Not found: %s" % relpath)
        ctype = mimetypes.guess_type(full)[0] or "application/octet-stream"
        if ctype.startswith("text/") or ctype == "application/javascript":
            ctype += "; charset=utf-8"
        with open(full, "rb") as f:
            self._send(200, ctype, f.read())

    # -- proxy to a real node -----------------------------------------------

    def _proxy(self):
        url = "http://%s%s" % (self.device, self.path)
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length) if length else None
        req = urllib.request.Request(url, data=body, method=self.command)
        if self.headers.get("Content-Type"):
            req.add_header("Content-Type", self.headers["Content-Type"])
        try:
            # The node is single-threaded and shares loop() with the shift state
            # machine, so a stuck request must not hang the browser indefinitely.
            with urllib.request.urlopen(req, timeout=5) as r:
                self._send(r.status, r.headers.get("Content-Type", "text/plain"), r.read())
        except urllib.error.HTTPError as e:
            # Real status codes carry meaning here (423 = shift in progress).
            self._send(e.code, e.headers.get("Content-Type", "text/plain"), e.read())
        except Exception as e:
            self._send(502, "text/plain", "proxy to %s failed: %s" % (self.device, e))

    # -- stub responses -----------------------------------------------------

    def _stub(self, path):
        if path == "/sensorData":
            return self._send_json(stub_sensor_data())
        if path == "/configData":
            return self._send_json(stub_config_data())
        if path == "/shiftStats":
            return self._send(200, "text/plain", stub_shift_stats())
        if path == "/shiftLogs":
            return self._send_json(stub_shift_logs())
        if path == "/api/config":
            if self.command == "POST":
                length = int(self.headers.get("Content-Length") or 0)
                raw = self.rfile.read(length).decode("utf-8") if length else "{}"
                try:
                    patch = json.loads(raw)
                except ValueError as e:
                    return self._send_json({"error": "bad json: %s" % e}, 400)
                # Overlay only known keys, as calJsonOverlay() does, then echo the
                # whole config back - the page renders the read-back, not its input.
                for k in ("neutralDownMs", "neutralUpMs", "shiftUpMs", "shiftDownMs",
                          "clutchIdlePos", "clutchFullyPull"):
                    if k in patch:
                        stub_cal[k] = int(patch[k])
            return self._send_json(stub_cal)
        if path == "/api/defaults":
            stub_cal.update(version=1, neutralDownMs=40, neutralUpMs=40, shiftUpMs=150,
                            shiftDownMs=150, clutchIdlePos=42, clutchFullyPull=137)
            return self._send_json(stub_cal)
        # /cmd, /update, /updateHallCurve - the pages only show the text back.
        return self._send(200, "text/plain", "stub OK: %s" % self.path)

    def log_message(self, fmt, *args):
        # flush: stdout is block-buffered when this isn't a TTY (piped, or run from a
        # task runner), which otherwise swallows the whole log until the process exits.
        print("  %s" % (fmt % args), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--device", metavar="IP",
                    help="proxy API calls to a real main_node instead of stubbing them")
    args = ap.parse_args()

    if not os.path.isdir(DATA_DIR):
        raise SystemExit("data dir not found: %s" % DATA_DIR)

    Handler.device = args.device
    print("serving %s" % DATA_DIR)
    print("api    : %s" % ("proxy -> http://%s" % args.device if args.device else "stubbed"))
    print("open   : http://127.0.0.1:%d/   (Ctrl+C to stop)" % args.port, flush=True)
    # Threaded: the pages poll several endpoints at once, and a slow proxy call
    # would otherwise block the page load behind it.
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
