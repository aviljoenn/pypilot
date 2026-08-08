#!/usr/bin/env python3
"""

inno_web_remote.py — Browser-based Inno-Remote UI (port 8888).

Connects to inno-pilot-bridge on TCP port 8555 as a standard remote
client (same HELLO/PING/BTN/MODE/RUD protocol as the ESP32 handheld),
then serves a single-page web app over HTTP + Server-Sent Events.

No external dependencies — stdlib only.

Usage:
    python3 /usr/local/bin/inno_web_remote.py
    Open http://192.168.6.13:8888/ in a browser on the boat LAN.

Environment overrides:
    INNO_WEB_PORT     HTTP/SSE listen port (default: 8888)
    INNO_BRIDGE_HOST  Bridge host (default: 127.0.0.1)
    INNO_BRIDGE_PORT  Bridge TCP port (default: 8555)
"""

import json
import logging
import math
import os
import queue
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-5s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("web-remote")

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
WEB_PORT          = int(os.getenv("INNO_WEB_PORT", "8888"))
BRIDGE_HOST       = os.getenv("INNO_BRIDGE_HOST", "127.0.0.1")
BRIDGE_PORT       = int(os.getenv("INNO_BRIDGE_PORT", "8555"))
PING_PERIOD_S     = 2.0
RECONNECT_DELAY_S = 1.0
# Multi-browser command arbitration has been removed: every connected
# browser is always allowed to issue commands.
# Sent in HELLO handshake.  Bridge logs mismatch but stays connected.
INNOPILOT_VERSION = "v1.3.3_B7"

# Telegram notification config — JSON file with "token" and "chat_id" keys.
# If the file does not exist or is invalid, notifications are silently skipped.
TELEGRAM_CONF = "/home/innopilot/.pypilot/telegram.conf"

# ---------------------------------------------------------------------------
# Settings persistence — /var/lib/inno-pilot/settings.json
# ---------------------------------------------------------------------------
SETTINGS_FILE = "/var/lib/inno-pilot/settings.json"

# ---------------------------------------------------------------------------
# OTA update paths
# ---------------------------------------------------------------------------
REPO_DIR      = "/home/innopilot/inno-pilot"
DEPLOY_SCRIPT = "/home/innopilot/inno_deploy.sh"
OTA_LOG_FILE  = "/var/lib/inno-pilot/ota_log.txt"

_DEFAULT_SETTINGS: dict = {
    "network": {
        "ip_mode": "dhcp",          # "dhcp" or "static"
        "ip":      "",
        "mask":    "255.255.255.0",
        "gateway": "",
        "dns1":    "8.8.8.8",
        "dns2":    "8.8.4.4",
    },
    "wifi": {
        "ssid": "",
        "key":  "",
    },
    "vessel": {
        "name":             "",
        "type":             "sail",  # "sail" or "power"
        "rudder_range_deg": 35,      # full rudder throw each side (degrees)
    },
    "features": {
        "limit_switches":         False,
        "temp_sensor":            False,
        "pi_voltage_sensor":      False,
        "battery_voltage_sensor": False,
        "current_sensor":         False,
        "on_board_buttons":       False,
        "oled_sh1106":            False,
        "invert_clutch":          False,
        "invert_motor":           False,
        "remote_helm":            False,  # Enables REMOTE button on main screen; OFF by default
    },
    "autopilot": {
        "deadband_pct":           3.0,
        "pgain":                  1.0,  # proportional heading gain
        "off_course_alarm_deg":   20,   # degrees off-course before alert
        # Convention: pct=100=full port, pct=0=full stbd. Therefore
        # rudder_limit_port_pct is the *upper* allowed pct (default 100=no limit)
        # and rudder_limit_stbd_pct is the *lower* allowed pct (default 0=no limit).
        "rudder_limit_port_pct":  100,  # software port stop (%) — upper bound
        "rudder_limit_stbd_pct":  0,    # software stbd stop (%) — lower bound
    },
    "safety": {
        "auto_disengage_on_fault":  True,
        "comms_warn_threshold_pct": 10,  # CRC error rate % for WARN
        "comms_crit_threshold_pct": 25,  # CRC error rate % for CRIT
    },
    "notifications": {
        # 0 = send one health report at boot only.
        # >0 = repeat every N minutes (stats accumulated over the period).
        "health_interval_min": 0,
        # Seconds between gateway pings (0.5 = 2/s, 5 = every 5 s, etc.)
        "ping_interval_s": 0.5,
    },
}


def _load_settings() -> dict:
    """Return settings from file merged over built-in defaults."""
    import copy
    s = copy.deepcopy(_DEFAULT_SETTINGS)
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE) as fh:
                saved = json.load(fh)
            for sec, vals in saved.items():
                if isinstance(s.get(sec), dict) and isinstance(vals, dict):
                    s[sec].update(vals)
                else:
                    s[sec] = vals
        except Exception as exc:
            log.warning("Settings load failed: %s", exc)
    return s


def _persist_settings(settings: dict) -> None:
    """Write settings dict to file, creating parent dirs if needed."""
    try:
        os.makedirs(os.path.dirname(SETTINGS_FILE), exist_ok=True)
        with open(SETTINGS_FILE, "w") as fh:
            json.dump(settings, fh, indent=2)
        log.info("Settings saved \u2192 %s", SETTINGS_FILE)
    except Exception as exc:
        log.error("Settings save failed: %s", exc)


# ---------------------------------------------------------------------------
# Shared state — written by bridge thread, read by HTTP handlers
# ---------------------------------------------------------------------------
_state: dict = {
    "connected":  False,
    "mode":       "IDLE",   # IDLE | AP | MANUAL  (from bridge MODE telemetry)
    "ap":         0,
    "hdg":        None,     # float degrees
    "cmd":        None,     # float degrees (AP heading command)
    "rdr":        None,     # float degrees (actual rudder angle from pypilot)
    "rdr_pct":    None,     # float 0–100  (actual rudder position %)
    "rdr_cmd":    0,        # int: pypilot servo command direction +1=port, 0=neutral, -1=stbd (conv 2)
    "db":         3.0,
    "comms":        "OK",     # OK | WARN | CRIT
    "warn":         None,
    "bridge_ver":   None,
    "version":      INNOPILOT_VERSION,
    "ui_mode":      "on",     # web remote selector state: auto | remote | on | off
    "rudder_stall": False,    # True when motor commanded but rudder not moving
    "pypilot_ok":   True,     # False when pypilot process has lost its connection
    "debug":        False,    # True when bridge is in DEBUG log level
    "net_warn":     False,    # True when inno-health-notify reports packet-loss WARN
}
_state_lock = threading.Lock()

# One queue per connected SSE browser, tracked with client id.
_sse_subs: list[dict] = []
_sse_lock  = threading.Lock()

# Commands flowing from browser → bridge
_cmd_q: "queue.Queue[str]" = queue.Queue(maxsize=200)

# Responses to SETTINGS GET / SETTINGS SET received from the bridge.
# HTTP handler drains this queue before issuing a request, then waits up to
# SETTINGS_TIMEOUT_S for the bridge to reply.  Maxsize=4 catches stale replies.
_settings_resp_q: "queue.Queue[Optional[dict]]" = queue.Queue(maxsize=4)
SETTINGS_TIMEOUT_S = 4.0

# When cleared, the bridge thread drops the TCP connection and stops reconnecting.
# When cleared, the bridge thread drops the TCP connection and stops reconnecting.
# Set by default (ON is the startup mode); cleared only when the user selects OFF.
_bridge_active = threading.Event()
_bridge_active.set()


def _send_telegram(text: str) -> None:
    """Fire-and-forget Telegram message.  Reads token/chat_id from TELEGRAM_CONF.
    Runs in a daemon thread so it never blocks the HTTP response path.
    Silently does nothing if the config file is absent or malformed.
    Prepends vessel name from settings so messages are identifiable per boat.
    """
    import urllib.request
    import urllib.error

    # Prepend boat name for multi-boat identification
    try:
        vessel_name = _load_settings().get("vessel", {}).get("name", "").strip()
        if vessel_name:
            text = f"[{vessel_name}] {text}"
    except Exception:
        pass

    def _worker() -> None:
        try:
            with open(TELEGRAM_CONF, "r") as fh:
                cfg = json.loads(fh.read())
            token   = cfg["token"]
            chat_id = str(cfg["chat_id"])
        except Exception:
            return  # config missing or invalid — skip silently
        try:
            payload = json.dumps({"chat_id": chat_id, "text": text}).encode()
            req = urllib.request.Request(
                f"https://api.telegram.org/bot{token}/sendMessage",
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            urllib.request.urlopen(req, timeout=8)
        except Exception as exc:
            log.warning("Telegram notify failed: %s", exc)

    t = threading.Thread(target=_worker, daemon=True)
    t.start()


def _snap() -> dict:
    with _state_lock:
        return dict(_state)


def _update(**kw) -> None:
    """Update shared state and push a snapshot to every SSE subscriber."""
    with _state_lock:
        _state.update(kw)
        snap = dict(_state)
    _broadcast(snap)


def _broadcast(snap: dict) -> None:
    with _sse_lock:
        dead = []
        for sub in _sse_subs:
            try:
                sub["q"].put_nowait(snap)
            except queue.Full:
                dead.append(sub)
        for sub in dead:
            _sse_subs.remove(sub)
            log.debug("SSE: dropped slow client")









def _parse_bridge_line(line: str) -> None:
    """Parse one telemetry line received from the bridge."""
    parts = line.split()
    if not parts:
        return
    c = parts[0].upper()

    if c == "PONG":
        pass  # keepalive, no state change

    elif c == "HELLO":
        _update(bridge_ver=(parts[1] if len(parts) > 1 else "?"))

    elif c == "AP":
        try:
            _update(ap=int(parts[1]))
        except (IndexError, ValueError):
            pass

    elif c == "MODE":
        # Bridge sends MODE IDLE|AP|MANUAL every telemetry cycle
        if len(parts) > 1:
            mode = parts[1].upper()
            _update(mode=mode)

    elif c in ("HDG", "CMD", "RDR", "DB"):
        try:
            _update(**{c.lower(): float(parts[1])})
        except (IndexError, ValueError):
            pass

    elif c == "RDR_PCT":
        try:
            _update(rdr_pct=float(parts[1]))
        except (IndexError, ValueError):
            pass

    elif c == "RDR_CMD":
        try:
            _update(rdr_cmd=int(parts[1]))
        except (IndexError, ValueError):
            pass

    elif c == "COMMS":
        # COMMS OK | COMMS WARN <detail> | COMMS CRIT
        if len(parts) > 1:
            _update(comms=parts[1].upper())

    elif c == "WARN":
        _update(warn=(parts[1] if len(parts) > 1 else None))

    elif c == "SETTINGS":
        # Response to SETTINGS GET: "SETTINGS <json>"
        # Response to SETTINGS SET: "SETTINGS OK" or "SETTINGS ERR"
        if len(parts) < 2:
            return
        sub = parts[1].upper()
        if sub in ("OK", "ERR"):
            # Ack for a SET command — signal waiting HTTP handler
            try:
                _settings_resp_q.put_nowait({"_ack": sub})
            except queue.Full:
                pass
        else:
            # Must be JSON payload (SETTINGS GET response).
            # The entire remainder of the line (after "SETTINGS ") is JSON.
            json_str = line[len("SETTINGS "):].strip()
            try:
                data = json.loads(json_str)
                _settings_resp_q.put_nowait(data)
            except (json.JSONDecodeError, queue.Full) as exc:
                log.warning("SETTINGS response parse error: %s", exc)

    elif c == "RUDDER_STALL":
        # Bridge sends RUDDER_STALL 0|1 every telemetry cycle.
        try:
            _update(rudder_stall=bool(int(parts[1])))
        except (IndexError, ValueError):
            pass

    elif c == "PYPILOT":
        # Bridge sends PYPILOT OK|DEAD every telemetry cycle.
        if len(parts) > 1:
            _update(pypilot_ok=(parts[1].upper() == "OK"))

    elif c == "TEST_LINE":
        # Bridge relays a plain-text result line from the Nano test firmware.
        # Broadcast transiently — does NOT persist in _state (avoids replay on reconnect).
        text = " ".join(parts[1:])
        snap = _snap()
        snap["_test_line"] = text
        _broadcast(snap)

    elif c == "TEST_DONE":
        # Bridge signals that the Nano test run is complete.
        snap = _snap()
        snap["_test_done"] = True
        _broadcast(snap)

    else:
        log.debug("Bridge unknown: %s", line)


def bridge_client() -> None:
    """
    Persistent TCP client to inno-pilot-bridge.
    Reconnects automatically after any failure.
    When _bridge_active is cleared (mode OFF), drops the current connection
    and does not reconnect until the event is set again.
    Runs forever as a daemon thread.
    """
    while True:
        # Don't attempt connection while mode is OFF
        if not _bridge_active.is_set():
            time.sleep(0.2)
            continue

        sock: Optional[socket.socket] = None
        intentional_disconnect = False
        try:
            log.info("Connecting to bridge %s:%d", BRIDGE_HOST, BRIDGE_PORT)
            sock = socket.create_connection((BRIDGE_HOST, BRIDGE_PORT), timeout=5.0)
            sock.settimeout(0.1)
            _update(connected=True)
            sock.sendall(f"HELLO {INNOPILOT_VERSION}\n".encode())

            buf = b""
            last_ping = time.monotonic()

            while True:
                # Drop connection immediately if mode switched to OFF
                if not _bridge_active.is_set():
                    log.info("Bridge: mode OFF — dropping TCP connection")
                    intentional_disconnect = True
                    break

                now = time.monotonic()
                if now - last_ping >= PING_PERIOD_S:
                    sock.sendall(b"PING\n")
                    last_ping = now

                # Forward any queued commands from browser to bridge
                while True:
                    try:
                        sock.sendall((_cmd_q.get_nowait() + "\n").encode())
                    except queue.Empty:
                        break

                # Read telemetry from bridge
                try:
                    chunk = sock.recv(512)
                    if not chunk:
                        raise OSError("Bridge EOF")
                    buf += chunk
                except socket.timeout:
                    pass

                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    _parse_bridge_line(raw.decode(errors="replace").strip())

        except Exception as exc:
            log.warning("Bridge lost: %s", exc)

        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
            if intentional_disconnect:
                # Mode OFF — stay disconnected; state already set by _handle_command
                log.info("Bridge disconnected (mode OFF)")
            elif not _bridge_active.is_set():
                # OFF may have been requested while the socket was already down.
                # Preserve OFF state and stay idle until MODE ON/active command.
                _update(connected=False)
                log.info("Bridge idle (mode OFF)")
            else:
                # Unexpected loss while active — reset to IDLE and schedule reconnect
                _update(
                    connected=False, hdg=None, cmd=None,
                    rdr=None, rdr_pct=None, ap=0,
                    mode="IDLE", comms="OK", warn=None,
                    rudder_stall=False, pypilot_ok=True,
                )
                log.info("Reconnecting in %ds...", RECONNECT_DELAY_S)
                time.sleep(RECONNECT_DELAY_S)


# ---------------------------------------------------------------------------
# Ship's wheel SVG — generated once at startup
# ---------------------------------------------------------------------------

def _make_wheel_svg() -> str:
    """Return SVG markup for a wooden ship's wheel (8 spokes, brass hub)."""
    parts: list[str] = []

    # Drop shadow
    parts.append('<ellipse cx="102" cy="103" rx="86" ry="86" fill="rgba(0,0,0,0.28)"/>')

    # Rim — three layered stroked circles give a rounded 3-D wood appearance
    parts.append('<circle cx="100" cy="100" r="84" fill="none" stroke="#2A1408" stroke-width="20"/>')
    parts.append('<circle cx="100" cy="100" r="84" fill="none" stroke="#8B5E3C" stroke-width="14"/>')
    parts.append('<circle cx="100" cy="100" r="84" fill="none" stroke="#C49A6C" stroke-width="5"/>')
    parts.append('<circle cx="100" cy="100" r="76" fill="none" stroke="#1A0C04" stroke-width="1.5"/>')

    # 8 spokes + handle knobs at 45° intervals
    for i in range(8):
        a = math.radians(i * 45)
        ix = 100 + 22 * math.cos(a)
        iy = 100 + 22 * math.sin(a)
        ox = 100 + 73 * math.cos(a)
        oy = 100 + 73 * math.sin(a)
        kx = 100 + 80 * math.cos(a)
        ky = 100 + 80 * math.sin(a)

        if i == 6:
            # ── King spoke: Turk's Head rope-woven appearance ──
            # Dark shadow base
            parts.append(
                f'<line x1="{ix:.1f}" y1="{iy:.1f}" x2="{ox:.1f}" y2="{oy:.1f}" '
                f'stroke="#1A0C04" stroke-width="11" stroke-linecap="round"/>'
            )
            # Rope base (manila/cream)
            parts.append(
                f'<line x1="{ix:.1f}" y1="{iy:.1f}" x2="{ox:.1f}" y2="{oy:.1f}" '
                f'stroke="#C8922A" stroke-width="7" stroke-linecap="round"/>'
            )
            # Woven Turk's Head strand pattern — alternating X crossings along the spoke.
            # King spoke is vertical: (100,78) → (100,27), length 51 px.
            # 7 crossings spaced every 7 px give seamless tiling (strand half-height = 3.5 px).
            rw = 3.5   # half-width  of strand from spoke centre
            rh = 3.5   # half-height of each crossing
            s_dark = "#6B4010"   # shadow side of strand
            s_mid  = "#D4A050"   # lit side of strand
            s_hi   = "#F0D090"   # specular highlight
            for step in range(7):
                yc = 30.5 + step * 7.0
                if step % 2 == 0:
                    # "\" strand lies on top of "/" strand
                    parts.append(
                        f'<line x1="{100+rw:.1f}" y1="{yc-rh:.1f}" '
                        f'x2="{100-rw:.1f}" y2="{yc+rh:.1f}" '
                        f'stroke="{s_dark}" stroke-width="2.5" stroke-linecap="round"/>'
                    )
                    parts.append(
                        f'<line x1="{100-rw:.1f}" y1="{yc-rh:.1f}" '
                        f'x2="{100+rw:.1f}" y2="{yc+rh:.1f}" '
                        f'stroke="{s_mid}" stroke-width="2.5" stroke-linecap="round"/>'
                    )
                    parts.append(
                        f'<line x1="{100-rw+0.5:.1f}" y1="{yc-rh+0.5:.1f}" '
                        f'x2="{100+rw-1.0:.1f}" y2="{yc+rh-1.0:.1f}" '
                        f'stroke="{s_hi}" stroke-width="0.9" stroke-linecap="round" opacity="0.65"/>'
                    )
                else:
                    # "/" strand lies on top of "\" strand
                    parts.append(
                        f'<line x1="{100-rw:.1f}" y1="{yc-rh:.1f}" '
                        f'x2="{100+rw:.1f}" y2="{yc+rh:.1f}" '
                        f'stroke="{s_dark}" stroke-width="2.5" stroke-linecap="round"/>'
                    )
                    parts.append(
                        f'<line x1="{100+rw:.1f}" y1="{yc-rh:.1f}" '
                        f'x2="{100-rw:.1f}" y2="{yc+rh:.1f}" '
                        f'stroke="{s_mid}" stroke-width="2.5" stroke-linecap="round"/>'
                    )
                    parts.append(
                        f'<line x1="{100+rw-0.5:.1f}" y1="{yc-rh+0.5:.1f}" '
                        f'x2="{100-rw+1.0:.1f}" y2="{yc+rh-1.0:.1f}" '
                        f'stroke="{s_hi}" stroke-width="0.9" stroke-linecap="round" opacity="0.65"/>'
                    )
        else:
            # Normal spoke — layered for wood-grain depth
            parts.append(
                f'<line x1="{ix:.1f}" y1="{iy:.1f}" x2="{ox:.1f}" y2="{oy:.1f}" '
                f'stroke="#2A1408" stroke-width="9" stroke-linecap="round"/>'
            )
            parts.append(
                f'<line x1="{ix:.1f}" y1="{iy:.1f}" x2="{ox:.1f}" y2="{oy:.1f}" '
                f'stroke="#8B5E3C" stroke-width="5.5" stroke-linecap="round"/>'
            )
            parts.append(
                f'<line x1="{ix:.1f}" y1="{iy:.1f}" x2="{ox:.1f}" y2="{oy:.1f}" '
                f'stroke="#C09060" stroke-width="2" stroke-linecap="round"/>'
            )

        # Handle knob
        parts.append(
            f'<circle cx="{kx:.1f}" cy="{ky:.1f}" r="10" '
            f'fill="#2A1408" stroke="#1A0C04" stroke-width="1"/>'
        )
        parts.append(
            f'<circle cx="{kx:.1f}" cy="{ky:.1f}" r="7.5" fill="#7A5030"/>'
        )
        # Highlight
        parts.append(
            f'<circle cx="{kx - 1.5:.1f}" cy="{ky - 1.5:.1f}" r="3" '
            f'fill="#C09060" opacity="0.6"/>'
        )

    # Brass center hub
    parts.append('<circle cx="100" cy="100" r="23" fill="#2A1408" stroke="#1A0C04" stroke-width="2"/>')
    parts.append('<circle cx="100" cy="100" r="20" fill="#C49020"/>')
    parts.append('<circle cx="100" cy="100" r="14" fill="#DAA520" stroke="#8B6010" stroke-width="1"/>')
    parts.append('<circle cx="100" cy="100" r="7"  fill="#F5D060"/>')
    parts.append('<circle cx="97"  cy="97"  r="3"  fill="#FFF8C0" opacity="0.55"/>')

    return "\n        ".join(parts)


_WHEEL_SVG = _make_wheel_svg()

# ---------------------------------------------------------------------------
# Embedded single-page web app
# ---------------------------------------------------------------------------
_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Inno-Remote</title>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%}
body{
  background:#0d0f1e;
  display:flex;
  align-items:flex-start;
  justify-content:center;
  padding:10px 6px 24px;
  font-family:'Segoe UI',Roboto,Arial,sans-serif;
  min-height:100vh;
}

/* ── Remote outer case ── */
.remote{
  background:linear-gradient(160deg,#f5f5f5 0%,#dcdcdc 100%);
  border-radius:26px;
  box-shadow:
    0 30px 80px rgba(0,0,0,0.65),
    inset 0 1px 0 rgba(255,255,255,0.9),
    inset 0 -2px 6px rgba(0,0,0,0.12);
  padding:16px 14px 20px;
  width:340px;
  max-width:98vw;
  display:flex;
  flex-direction:column;
  gap:13px;
  -webkit-user-select:none;user-select:none;
}

/* ── OLED display ── */
.oled{
  background:#06081a;
  border-radius:9px;
  padding:4px 12px 4px;
  box-shadow:
    inset 0 3px 10px rgba(0,0,0,0.9),
    0 0 0 3px #1e2040,
    0 0 0 5px #2e3060,
    0 0 0 7px #131326;
  display:flex;
  flex-direction:column;
  gap:0;
  justify-content:space-between;
  height:138px; /* content-box height; rendered total ≈ 146px (est. base +50px) */
}
.oled-title{
  color:#00d4ff;
  font-size:1.2em;
  font-weight:700;
  text-align:center;
  letter-spacing:3px;
  text-shadow:0 0 10px #00bfff,0 0 22px rgba(0,191,255,0.35);
}

/* Rudder position bar */
.rdr-bar{
  position:relative;
  height:16px;
  display:flex;
  align-items:center;
  padding:0 4px;
}
.rdr-track{
  width:100%;
  height:2px;
  background:repeating-linear-gradient(90deg,
    #1a4060 0,#1a4060 4px,transparent 4px,transparent 8px);
  position:relative;
}
.rdr-center-tick{
  position:absolute;
  left:50%;
  top:-4px;
  width:2px;
  height:10px;
  background:#2a6090;
  transform:translateX(-50%);
}
.rdr-marker{
  position:absolute;
  width:12px;
  height:12px;
  background:#00d4ff;
  box-shadow:0 0 6px #00bfff;
  top:50%;
  left:50%;
  transform:translate(-50%,-50%);
  transition:left 0.12s ease;
}

/* Rudder command direction triangle */
.rdr-cmd-arrow{
  position:absolute;
  top:50%;
  transform:translateY(-50%);
  width:0;
  height:0;
  border-top:7px solid transparent;
  border-bottom:7px solid transparent;
  display:none;        /* hidden by default; shown by JS */
  transition:left 0.12s ease;   /* match rdr-marker so arrow glides in lockstep */
}
.rdr-cmd-arrow.port{
  border-right:12px solid #ff6a00;  /* orange, points left */
}
.rdr-cmd-arrow.stbd{
  border-left:12px solid #ff6a00;   /* orange, points right */
}

/* line-height:1.5 pre-reserves height for the 1.5× AP label so layout never shifts */
.oled-mode{color:#ccc;font-size:0.8em;letter-spacing:1px;line-height:1.5;display:flex;align-items:flex-end}
.oled-mode b{color:#00d4ff}
/* AP label in AUTO mode — 1.5× the surrounding text size */
.oled-mode .ap-label{font-size:1.5em;font-weight:700;color:#00d4ff;line-height:1}
.oled-data{
  color:#00d4ff;
  font-size:1.2em;
  font-weight:700;
  font-family:'Courier New',monospace;
  display:flex;
  justify-content:space-between;
}
.oled-btns{
  display:flex;
  justify-content:center;
  gap:4px;
  margin-top:1px;
}
.oled-btn{
  background:#0a1428;
  color:#7aabcc;
  border:1px solid #1e3850;
  border-radius:3px;
  font-size:0.7em;
  padding:2px 7px;
  cursor:pointer;
  font-family:'Courier New',monospace;
  touch-action:manipulation;
}
.oled-btn:active{background:#1a3060}
/* Bottom row of OLED screen: version left, conn centred */
.oled-status{
  display:flex;
  align-items:center;
  position:relative;
  font-family:'Courier New',monospace;
  letter-spacing:1px;
}
#o-ver{
  font-size:0.63em;
  color:#6ee0ff;
}
#o-conn{
  position:absolute;
  left:50%;
  transform:translateX(-50%);
  font-size:0.945em;
  font-weight:700;
  white-space:nowrap;
}
.ok{color:#00cc70}
.warn{color:#ffaa00}
.crit{color:#ff3030;animation:blink .45s step-end infinite}
@keyframes blink{50%{opacity:0}}

/* ── Physical button row ── */
.btn-row{
  display:flex;
  justify-content:space-between;
  gap:5px;
}
.hw-btn{
  flex:1;
  height:50px;
  border:none;
  border-radius:8px;
  font-size:1.2em;
  font-weight:800;
  cursor:pointer;
  color:#fff;
  box-shadow:0 5px 0 rgba(0,0,0,0.38),0 6px 10px rgba(0,0,0,0.28);
  transition:transform .07s,box-shadow .07s;
  display:flex;align-items:center;justify-content:center;
  touch-action:manipulation;
}
.hw-btn:active{transform:translateY(4px);box-shadow:0 1px 0 rgba(0,0,0,0.38),0 2px 4px rgba(0,0,0,0.28)}
.hw-btn.b1{background:linear-gradient(180deg,#ff5a5a,#c00000)}
.hw-btn.b2{background:linear-gradient(180deg,#cacaca,#909090);color:#111}
.hw-btn.b3{background:linear-gradient(180deg,#444444,#111111)}
.hw-btn.b4{background:linear-gradient(180deg,#55cc55,#1a8a1a)}
.hw-btn.b5{background:linear-gradient(180deg,#55cc55,#1a8a1a)}

/* ── STOP + toggle row ── */
.action-row{
  display:flex;
  align-items:center;
  justify-content:space-between;
  padding:0 10px;
}

/* STOP button */
.stop-wrap{
  position:relative;
  width:96px;
  height:96px;
}
.stop-ring{
  position:absolute;
  inset:0;
  border-radius:50%;
  background:radial-gradient(circle at 35% 28%,#dddddd,#9a9a9a 55%,#686868);
  box-shadow:0 4px 14px rgba(0,0,0,0.5),inset 0 1px 0 rgba(255,255,255,0.55);
}
.stop-btn{
  position:absolute;
  inset:8px;
  border-radius:50%;
  border:none;
  cursor:pointer;
  background:radial-gradient(circle at 38% 28%,#ff7070,#cc0000 55%,#7a0000);
  color:#fff;
  font-size:1.05em;
  font-weight:900;
  letter-spacing:2px;
  box-shadow:0 6px 0 #550000,0 8px 18px rgba(0,0,0,0.5),
             inset 0 2px 4px rgba(255,255,255,0.18);
  transition:transform .07s,box-shadow .07s;
  touch-action:manipulation;
}
.stop-btn:active{
  transform:translateY(4px);
  box-shadow:0 2px 0 #550000,0 4px 8px rgba(0,0,0,0.5),
             inset 0 2px 4px rgba(255,255,255,0.18);
}

/* ── 4-position mode radio selector ── */
.mode-radio-group{display:flex;flex-direction:column;gap:12px;justify-content:center}
.mode-radio{display:flex;align-items:center;gap:10px;cursor:pointer;user-select:none;-webkit-user-select:none}
.mrd-dot{
  width:22px;height:22px;border-radius:50%;
  border:2px solid #888;background:#ccc;
  position:relative;flex-shrink:0;
  transition:border-color .18s,background .18s;
}
.mrd-dot::after{
  content:'';position:absolute;inset:4px;border-radius:50%;
  background:#007aff;transform:scale(0);
  transition:transform .22s cubic-bezier(.34,1.56,.64,1);
}
.mode-radio.active .mrd-dot{border-color:#007aff;background:#d8eaff}
.mode-radio.active .mrd-dot::after{transform:scale(1)}
.mrd-lbl{
  font-size:0.7em;font-weight:600;color:#999;
  letter-spacing:0.5px;line-height:1.1;
  transition:color .15s,font-weight .15s;
}
.mode-radio.active .mrd-lbl{color:#111;font-weight:800}
.mode-radio:hover .mrd-lbl{color:#444}
.mode-radio.disabled{pointer-events:none;cursor:default;opacity:0.28}
/* OFF mode: blank the OLED content area */
.oled.blank-mode .oled-title,
.oled.blank-mode .rdr-bar,
.oled.blank-mode .oled-mode,
.oled.blank-mode .oled-data,
.oled.blank-mode .oled-status{visibility:hidden}

/* ── Ship's wheel ── */
.wheel-section{
  display:flex;
  flex-direction:column;
  align-items:center;
  gap:5px;
}
.wheel-wrap{
  width:210px;
  height:210px;
  cursor:grab;
  touch-action:none;
  filter:saturate(0.3) brightness(0.5);
  transition:filter .35s;
}
.wheel-wrap.active{
  filter:saturate(1) brightness(1);
}
.wheel-wrap.active:active{cursor:grabbing}
#wheel-svg{width:100%;height:100%;display:block}

/* ── Nudge buttons (port/stbd, flank the helm wheel) ── */
.wheel-nudge-row{
  display:flex;
  align-items:center;
  gap:10px;
}
.nudge-btn{
  width:52px;height:52px;
  border-radius:10px;
  background:#12192e;
  border:2px solid #2e5490;
  color:#5bb8ff;
  font-size:1.5em;
  line-height:1;
  cursor:pointer;
  touch-action:manipulation;
  flex-shrink:0;
  transition:background .12s,opacity .2s,border-color .2s;
  -webkit-user-select:none;user-select:none;
}
.nudge-btn:active{background:#0d2a50;border-color:#5bb8ff}
.nudge-btn:disabled{
  opacity:0.22;
  cursor:not-allowed;
  border-color:#2a2a2a;
  color:#444;
}

/* ── "No bridge" overlay ── */
.no-bridge-overlay{
  position:fixed;
  inset:0;
  background:rgba(6,8,28,.88);
  display:flex;
  flex-direction:column;
  align-items:center;
  justify-content:center;
  gap:14px;
  z-index:200;
  pointer-events:none;
  -webkit-backdrop-filter:blur(5px);
  backdrop-filter:blur(5px);
}
.no-bridge-overlay h2{
  font-size:1.8em;
  color:#ff4040;
  letter-spacing:3px;
}
.no-bridge-overlay p{color:#aaa;font-size:.9em}
.no-bridge-overlay.hidden{display:none}

@media(max-width:350px){
  .remote{width:100vw;border-radius:0;padding:10px 8px 16px}
}

/* ── Gear / settings button ─────────────────────────────────────────── */
.settings-footer{display:flex;justify-content:flex-start;padding:0 4px 2px;gap:8px}
.gear-btn{
  width:38px;height:38px;border-radius:50%;
  background:#111;border:1.5px solid #333;
  color:#fff;font-size:1.3em;line-height:1;
  cursor:pointer;display:flex;align-items:center;justify-content:center;
  box-shadow:0 3px 8px rgba(0,0,0,.55),inset 0 1px 0 rgba(255,255,255,.07);
  transition:transform .1s,background .15s,border-color .15s;
  touch-action:manipulation;
}
.gear-btn:active{transform:scale(.88)}
.gear-btn.settings-open{
  background:#0d2a40;border-color:#00bfff;color:#00d4ff;
  box-shadow:0 0 8px rgba(0,191,255,.45),0 3px 8px rgba(0,0,0,.4);
}

/* ── Settings warning toast ─────────────────────────────────────────── */
.sw-toast{
  position:fixed;top:50%;left:50%;
  transform:translate(-50%,-50%) scale(.85);
  background:rgba(155,50,0,.97);color:#fff;
  padding:11px 20px;border-radius:9px;
  font-size:.88em;font-weight:700;text-align:center;line-height:1.5;
  opacity:0;pointer-events:none;z-index:500;
  transition:opacity .18s,transform .18s;
  box-shadow:0 6px 24px rgba(0,0,0,.7);
}
.sw-toast.visible{opacity:1;transform:translate(-50%,-50%) scale(1)}

/* ── Settings overlay panel ─────────────────────────────────────────── */
.sov{
  position:fixed;inset:0;background:rgba(4,6,22,.92);
  display:flex;align-items:center;justify-content:center;
  z-index:400;
  -webkit-backdrop-filter:blur(4px);backdrop-filter:blur(4px);
}
.sov.hidden{display:none}
.spanel{
  background:linear-gradient(170deg,#0e1226,#080b18);
  border:1px solid #1e2a50;border-radius:13px;
  box-shadow:0 24px 70px rgba(0,0,0,.85),0 0 0 1px rgba(0,191,255,.08);
  width:318px;max-width:96vw;max-height:88vh;
  display:flex;flex-direction:column;overflow:hidden;
}
.shdr{
  background:linear-gradient(90deg,#0c1830,#101e40);
  border-bottom:1px solid #1a2840;
  padding:9px 14px 7px;flex-shrink:0;
}
.shdr-title{
  color:#00d4ff;font-size:1.0em;font-weight:700;
  letter-spacing:3px;text-shadow:0 0 10px rgba(0,191,255,.4);
  display:block;
}
.shdr-hint{
  color:#354860;font-size:.62em;font-family:'Courier New',monospace;
  letter-spacing:.5px;margin-top:2px;display:block;
}
/* Status/feedback line shown below the header hint while loading/saving settings */
.shdr-status{font-size:.67em;font-family:'Courier New',monospace;margin-top:3px;
  min-height:1em;transition:color .2s;display:block;}
.shdr-status.s-ok  {color:#00cc50;}
.shdr-status.s-warn{color:#f0a000;}
.shdr-status.s-err {color:#cc3030;}
.shdr-status.s-pend{color:#6090b0;}
.sbody{overflow-y:auto;padding:6px 12px 10px;flex:1;overscroll-behavior:contain}
.ss-title{
  color:#3090b8;font-size:.68em;font-weight:700;letter-spacing:2px;
  margin:12px 0 4px;padding-bottom:3px;border-bottom:1px solid #152030;
}
.ss-title:first-child{margin-top:4px}
.sf-row{
  display:flex;align-items:center;justify-content:space-between;
  padding:5px 5px;border-radius:5px;border:1px solid transparent;gap:6px;
  transition:border-color .13s,background .13s;
}
.sf-row.focused{
  background:rgba(0,80,160,.20);border-color:#0080c0;
  box-shadow:0 0 7px rgba(0,128,192,.28);
}
.sf-row.sf-hidden{display:none}
.sf-lbl{color:#90a8c0;font-size:.76em;min-width:108px;flex-shrink:0}
.sf-eval{
  color:#00d4ff;font-size:.8em;font-family:'Courier New',monospace;
  font-weight:700;min-width:64px;text-align:right;
}
.sf-inp{
  background:#080f20;border:1px solid #1e3450;border-radius:4px;
  color:#00d4ff;font-family:'Courier New',monospace;font-size:.79em;
  padding:3px 6px;width:130px;outline:none;
}
.sf-inp:focus{border-color:#0090d0;box-shadow:0 0 5px rgba(0,144,208,.35)}
.sf-bool{display:flex;gap:4px}
.sf-bb{
  padding:3px 9px;border-radius:4px;border:1px solid #1e3450;
  background:#080f20;color:#4a6a80;font-size:.72em;font-weight:700;
  cursor:pointer;touch-action:manipulation;font-family:'Courier New',monospace;
  transition:background .1s,color .1s,border-color .1s;
}
.sf-bb.bb-on {background:#082010;border-color:#00a030;color:#00cc50}
.sf-bb.bb-off{background:#200808;border-color:#a02020;color:#cc3030}
.sftr{border-top:1px solid #152030;padding:7px 12px;display:flex;gap:8px;flex-shrink:0}
.sftr-btn{
  flex:1;padding:7px 0;border-radius:6px;border:none;
  font-size:.8em;font-weight:700;cursor:pointer;letter-spacing:1.5px;
  touch-action:manipulation;
}
.sftr-btn.sv {background:linear-gradient(180deg,#0d4020,#061808);color:#00cc50;border:1px solid #0a4020}
.sftr-btn.cx {background:linear-gradient(180deg,#280c0c,#140404);color:#cc3030;border:1px solid #3a1010}

/* ── DEBUG toggle button (oval pill, replaces old circular Test button) ── */
.dbg-btn{
  height:38px;padding:0 14px;border-radius:19px;
  background:#111;border:1.5px solid #333;
  color:#e8a020;font-size:.72em;font-weight:700;line-height:1;
  cursor:pointer;display:flex;align-items:center;justify-content:center;
  box-shadow:0 3px 8px rgba(0,0,0,.55),inset 0 1px 0 rgba(255,255,255,.07);
  transition:transform .1s,background .15s,border-color .15s;
  touch-action:manipulation;letter-spacing:.5px;white-space:nowrap;
}
.dbg-btn:active{transform:scale(.88)}
.dbg-btn.dbg-active{
  background:#1a0a00;border-color:#e8a020;color:#f0c040;
  box-shadow:0 0 8px rgba(232,160,32,.45),0 3px 8px rgba(0,0,0,.4);
  animation:dbg-flash 1s ease-in-out infinite;
}
@keyframes dbg-flash{0%,100%{opacity:1}50%{opacity:.35}}

/* ── TEST overlay ────────────────────────────────────────────────────── */
.tov{
  position:fixed;inset:0;background:rgba(4,6,22,.92);
  display:flex;align-items:center;justify-content:center;
  z-index:400;
  -webkit-backdrop-filter:blur(4px);backdrop-filter:blur(4px);
}
.tov.hidden{display:none}
.tpanel{
  background:linear-gradient(170deg,#0e1226,#080b18);
  border:1px solid #2a1a00;border-radius:13px;
  box-shadow:0 24px 70px rgba(0,0,0,.85),0 0 0 1px rgba(232,160,32,.08);
  width:318px;max-width:96vw;max-height:88vh;
  display:flex;flex-direction:column;overflow:hidden;
}
.thdr{
  background:linear-gradient(90deg,#1a1000,#201800);
  border-bottom:1px solid #2a1a00;
  padding:9px 14px 7px;flex-shrink:0;
}
.thdr-title{
  color:#e8a020;font-size:1.0em;font-weight:700;
  letter-spacing:3px;text-shadow:0 0 10px rgba(232,160,32,.4);
  display:block;
}
.thdr-hint{
  color:#504030;font-size:.62em;font-family:'Courier New',monospace;
  letter-spacing:.5px;margin-top:2px;display:block;
}
.tbody{overflow-y:auto;padding:6px 12px 10px;flex:1;overscroll-behavior:contain}

/* Catalogue list */
.tcat-item{
  display:flex;align-items:center;gap:8px;
  padding:7px 6px;border-radius:6px;border:1px solid #1a1200;
  margin-bottom:4px;cursor:pointer;touch-action:manipulation;
  transition:background .12s,border-color .12s;
}
.tcat-item:active,.tcat-item:hover{background:rgba(232,160,32,.08);border-color:#604010}
.tcat-num{
  background:#1a1200;color:#e8a020;font-size:.7em;font-weight:700;
  font-family:'Courier New',monospace;border-radius:4px;
  padding:3px 6px;min-width:22px;text-align:center;flex-shrink:0;
}
.tcat-name{color:#c0a060;font-size:.76em;line-height:1.35;flex:1}
.tcat-arrow{color:#504030;font-size:1.0em;flex-shrink:0}

/* Detail view */
.tdet-view{display:none}
.tdet-view.active{display:block}
.tcat-view.hidden{display:none}
.tdet-name{
  color:#e8a020;font-size:.88em;font-weight:700;letter-spacing:.5px;
  margin-bottom:8px;line-height:1.3;
}
.tdet-section{
  color:#6080a0;font-size:.6em;font-weight:700;letter-spacing:2px;
  margin:8px 0 3px;text-transform:uppercase;
}
.tdet-text{color:#90a8b8;font-size:.72em;line-height:1.55;margin-bottom:4px;white-space:pre-wrap}
.tdet-fw{
  background:#0d0f1e;border:1px solid #1a1200;border-radius:4px;
  color:#e8a020;font-family:'Courier New',monospace;font-size:.67em;
  padding:4px 8px;margin-bottom:6px;white-space:pre-wrap;word-break:break-all;
}
.tdet-confirm{
  display:flex;gap:8px;margin-top:10px;
}
.tdet-btn{
  flex:1;padding:8px 0;border-radius:6px;border:none;
  font-size:.78em;font-weight:700;cursor:pointer;letter-spacing:1.5px;
  touch-action:manipulation;
}
.tdet-btn.yes{background:linear-gradient(180deg,#0d4020,#061808);color:#00cc50;border:1px solid #0a4020}
.tdet-btn.no {background:linear-gradient(180deg,#280c0c,#140404);color:#cc3030;border:1px solid #3a1010}

/* Results view */
.tres-view{display:none}
.tres-view.active{display:block}
.tres-name{color:#e8a020;font-size:.85em;font-weight:700;margin-bottom:6px;line-height:1.3}
.tres-log{
  background:#06081a;border-radius:6px;border:1px solid #0a0e24;
  padding:7px 8px;height:220px;overflow-y:auto;
  font-family:'Courier New',monospace;font-size:.67em;color:#00cc80;
  line-height:1.5;white-space:pre-wrap;word-break:break-all;
}
.tres-status{
  color:#e8a020;font-size:.7em;text-align:center;
  margin-top:5px;font-family:'Courier New',monospace;letter-spacing:1px;
}
.tres-b3{
  display:none;width:100%;margin-top:6px;
  padding:8px 0;border-radius:6px;border:1px solid #0a4020;
  background:linear-gradient(180deg,#0d4020,#061808);color:#00cc50;
  font-size:.78em;font-weight:700;cursor:pointer;letter-spacing:1.5px;
  touch-action:manipulation;
}
.tres-b3.visible{display:block}

/* Shared footer */
.tftr{border-top:1px solid #1a1200;padding:7px 12px;display:flex;gap:8px;flex-shrink:0}
.tftr-btn{
  flex:1;padding:7px 0;border-radius:6px;border:none;
  font-size:.78em;font-weight:700;cursor:pointer;letter-spacing:1.5px;
  touch-action:manipulation;
}
.tftr-btn.back{background:linear-gradient(180deg,#1a1000,#0d0800);color:#a07020;border:1px solid #2a1800}
.tftr-btn.close{background:linear-gradient(180deg,#280c0c,#140404);color:#cc3030;border:1px solid #3a1010}

/* ── Packet-loss warning banner ─────────────────────────────────────────── */
#net-warn-banner{display:none;position:fixed;top:0;left:0;right:0;z-index:1500;
  background:#7a2000;color:#ffe0c0;font-size:0.82em;font-weight:bold;
  text-align:center;padding:5px 8px;letter-spacing:0.03em}
#net-warn-banner.visible{display:block}

/* ── Boat-name setup modal ──────────────────────────────────────────────── */
.bnm-overlay{position:fixed;inset:0;background:rgba(0,0,0,.80);display:none;
  align-items:center;justify-content:center;z-index:2000}
.bnm-overlay.visible{display:flex}
.bnm-box{background:#1a1a2e;border:1px solid #0090d0;border-radius:10px;
  padding:28px 24px;max-width:320px;width:90%;text-align:center}
.bnm-title{font-size:1.1em;font-weight:bold;color:#0090d0;margin-bottom:10px}
.bnm-body{font-size:0.82em;color:#aaa;margin-bottom:18px;line-height:1.4}
.bnm-inp{width:100%;box-sizing:border-box;padding:9px 10px;background:#0d1117;
  border:1px solid #444;border-radius:5px;color:#e0e0e0;font-size:1em;
  margin-bottom:8px;outline:none}
.bnm-inp:focus{border-color:#0090d0}
.bnm-err{color:#e05050;font-size:0.78em;min-height:1.1em;margin-bottom:10px}
.bnm-btn{background:#0090d0;color:#fff;border:none;border-radius:5px;
  padding:9px 28px;font-size:0.95em;cursor:pointer;font-weight:bold}
.bnm-btn:active{opacity:.75}
/* ── Software update modal ──────────────────────────────────────────────── */
.upd-overlay{position:fixed;inset:0;background:rgba(0,0,0,.80);display:none;
  align-items:center;justify-content:center;z-index:2001}
.upd-overlay.visible{display:flex}
.upd-box{background:#1a1a2e;border:1px solid #0090d0;border-radius:10px;
  padding:24px 22px;max-width:380px;width:92%;text-align:left}
.upd-title{font-size:1.1em;font-weight:bold;color:#0090d0;margin-bottom:12px;text-align:center}
.upd-body{font-size:0.80em;color:#ccc;margin-bottom:16px;line-height:1.5;
  max-height:200px;overflow-y:auto;white-space:pre-wrap;font-family:monospace}
.upd-actions{display:flex;gap:10px;justify-content:center}
.upd-btn{border:none;border-radius:5px;padding:8px 22px;font-size:0.92em;
  cursor:pointer;font-weight:bold}
.upd-install{background:#0090d0;color:#fff}
.upd-close{background:#444;color:#ccc}
.upd-btn:active{opacity:.75}
/* Terminal log pane inside the update modal */
.upd-box.terminal{max-width:540px}
#upd-log{display:none;background:#0a0a0a;color:#00e000;font-family:'Courier New',monospace;
  font-size:0.70em;padding:10px;border-radius:5px;max-height:320px;overflow-y:auto;
  white-space:pre-wrap;word-break:break-all;margin-bottom:12px;min-height:80px}
</style>
</head>
<body>

<!-- Disconnected overlay (visible until SSE delivers connected=true) -->
<div class="no-bridge-overlay" id="overlay">
  <h2>NO BRIDGE</h2>
  <p>Connecting to inno-pilot-bridge\u2026</p>
  <p id="dots">\u25cf</p>
</div>

<div class="remote">

  <!-- OLED display panel -->
  <div class="oled">
    <div class="oled-title">Inno-Web-Remote</div>

    <!-- Rudder position bar -->
    <div class="rdr-bar">
      <div class="rdr-track">
        <div class="rdr-center-tick"></div>
        <div class="rdr-marker" id="rdr-marker"></div>
        <div class="rdr-cmd-arrow port" id="rdr-arrow-port"></div>
        <div class="rdr-cmd-arrow stbd" id="rdr-arrow-stbd"></div>
      </div>
    </div>

    <div class="oled-data">
      <span>CMD:&nbsp;<span id="o-cmd">---</span>&deg;</span>
      <span>Head:&nbsp;<span id="o-hdg">---</span>&deg;</span>
    </div>

    <div class="oled-mode" id="oled-mode-row">MODE: <b id="o-mode">IDLE</b></div>

    <div class="oled-status">
      <span id="o-ver">---</span>
      <span id="o-conn" class="warn">CONNECTING\u2026</span>
    </div>

  </div>

  <!-- Physical button row -->
  <div class="btn-row">
    <button class="hw-btn b1" data-cmd="BTN -10">&laquo;</button>
    <button class="hw-btn b2" data-cmd="BTN -1">&lsaquo;</button>
    <button class="hw-btn b3" data-cmd="BTN TOGGLE">|</button>
    <button class="hw-btn b4" data-cmd="BTN +1">&rsaquo;</button>
    <button class="hw-btn b5" data-cmd="BTN +10">&raquo;</button>
  </div>

  <!-- STOP button + 3-position mode toggle -->
  <div class="action-row">
    <div class="stop-wrap">
      <div class="stop-ring"></div>
      <button class="stop-btn" id="stop-btn">STOP</button>
    </div>

    <div class="mode-radio-group" id="mode-radio-group">
      <div class="mode-radio" data-action="auto">
        <span class="mrd-dot"></span><span class="mrd-lbl">AUTO</span>
      </div>
      <div class="mode-radio disabled" data-action="remote">
        <span class="mrd-dot"></span><span class="mrd-lbl">REMOTE</span>
      </div>
      <div class="mode-radio active" data-action="on">
        <span class="mrd-dot"></span><span class="mrd-lbl">ON</span>
      </div>
      <div class="mode-radio" data-action="off">
        <span class="mrd-dot"></span><span class="mrd-lbl">OFF</span>
      </div>
    </div>
  </div>

  <!-- Ship's wheel (rotatable, active in MANUAL mode) -->
  <!-- Nudge buttons (port/stbd) flank the wheel; active in AUTO mode only -->
  <div class="wheel-section">
    <div class="wheel-nudge-row">
      <button class="nudge-btn" id="nudge-port" title="Port nudge — hold to extend (min 500 ms)" disabled>&#9664;</button>
      <div class="wheel-wrap" id="wheel-wrap">
        <svg id="wheel-svg" viewBox="0 0 200 200" xmlns="http://www.w3.org/2000/svg">
          $$WHEEL_SVG$$
        </svg>
      </div>
      <button class="nudge-btn" id="nudge-stbd" title="Stbd nudge — hold to extend (min 500 ms)" disabled>&#9654;</button>
    </div>
  </div>

  <!-- Settings gear button — only active while toggle is in OFF position -->
  <!-- DEBUG toggle button -->
  <div class="settings-footer">
    <button class="gear-btn" id="gear-btn" title="Settings (OFF mode only)">&#9881;</button>
    <button class="dbg-btn" id="dbg-btn" title="Toggle bridge debug logging">Debug</button>
  </div>

</div><!-- .remote -->

<!-- Settings warning toast -->
<div class="sw-toast" id="sw-toast">Settings only<br>available in OFF mode</div>

<!-- Settings overlay -->
<div class="sov hidden" id="sov">
  <div class="spanel">
    <div class="shdr">
      <span class="shdr-title">&#9881; SETTINGS</span>
      <span class="shdr-hint">Tap a field to edit &nbsp;&bull;&nbsp; SAVE to apply</span>
      <span class="shdr-status" id="sov-status"></span>
    </div>
    <div class="sbody" id="sbody">

      <div class="ss-title">NETWORK</div>
      <div class="sf-row" data-sfid="ip_mode">
        <span class="sf-lbl">IP Mode</span>
        <span class="sf-eval" id="sf-ip_mode">DHCP</span>
      </div>
      <div class="sf-row sf-hidden" data-sfid="ip">
        <span class="sf-lbl">IP Address</span>
        <input class="sf-inp" type="text" id="sf-ip" placeholder="192.168.x.x">
      </div>
      <div class="sf-row sf-hidden" data-sfid="mask">
        <span class="sf-lbl">Subnet Mask</span>
        <input class="sf-inp" type="text" id="sf-mask" placeholder="255.255.255.0">
      </div>
      <div class="sf-row sf-hidden" data-sfid="gateway">
        <span class="sf-lbl">Gateway</span>
        <input class="sf-inp" type="text" id="sf-gateway" placeholder="192.168.x.1">
      </div>
      <div class="sf-row sf-hidden" data-sfid="dns1">
        <span class="sf-lbl">DNS 1</span>
        <input class="sf-inp" type="text" id="sf-dns1" placeholder="8.8.8.8">
      </div>
      <div class="sf-row sf-hidden" data-sfid="dns2">
        <span class="sf-lbl">DNS 2</span>
        <input class="sf-inp" type="text" id="sf-dns2" placeholder="8.8.4.4">
      </div>

      <div class="ss-title">WI-FI</div>
      <div class="sf-row" data-sfid="ssid">
        <span class="sf-lbl">SSID</span>
        <input class="sf-inp" type="text" id="sf-ssid" placeholder="Network name">
      </div>
      <div class="sf-row" data-sfid="key">
        <span class="sf-lbl">Password</span>
        <input class="sf-inp" type="password" id="sf-key" placeholder="Wi-Fi key">
      </div>

      <div class="ss-title">VESSEL</div>
      <div class="sf-row" data-sfid="name">
        <span class="sf-lbl">Boat Name</span>
        <input class="sf-inp" type="text" id="sf-name" placeholder="My Boat">
      </div>
      <div class="sf-row" data-sfid="type">
        <span class="sf-lbl">Vessel Type</span>
        <span class="sf-eval" id="sf-type">SAIL</span>
      </div>
      <div class="sf-row" data-sfid="rudder_range_deg">
        <span class="sf-lbl">Rudder Range (&#176;)</span>
        <input class="sf-inp" type="number" id="sf-rudder_range_deg" min="10" max="100" step="1">
      </div>

      <div class="ss-title">CONNECTIONS &amp; FEATURES</div>
      <div class="sf-row" data-sfid="limit_switches">
        <span class="sf-lbl">Limit Switches</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="limit_switches" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="limit_switches" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="temp_sensor">
        <span class="sf-lbl">Temp Sensor</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="temp_sensor" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="temp_sensor" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="pi_voltage_sensor">
        <span class="sf-lbl">Pi Voltage</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="pi_voltage_sensor" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="pi_voltage_sensor" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="battery_voltage_sensor">
        <span class="sf-lbl">Battery Voltage</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="battery_voltage_sensor" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="battery_voltage_sensor" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="current_sensor">
        <span class="sf-lbl">Current Sensor</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="current_sensor" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="current_sensor" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="on_board_buttons">
        <span class="sf-lbl">On-Board Buttons</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="on_board_buttons" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="on_board_buttons" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="oled_sh1106">
        <span class="sf-lbl" title="Enable if the left-most column of pixels appears on the right edge of the screen. This switches the OLED driver from SSD1306 to SH1106 mode, which corrects the 2-pixel column offset used by some 1.3&quot; OLED modules.">OLED SH1106 Mode &#9432;</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="oled_sh1106" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="oled_sh1106" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="invert_clutch">
        <span class="sf-lbl" title="Enable if the clutch engages and disengages in reverse. This inverts the Nano pin 11 output level for builds where the clutch relay is wired active-LOW instead of active-HIGH.">Invert Clutch Signal &#9432;</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="invert_clutch" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="invert_clutch" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="invert_motor">
        <span class="sf-lbl" title="Enable if the rudder moves in the wrong direction. This will invert the H-bridge drive signals so port and starboard are swapped in hardware.">Invert Motor Direction &#9432;</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="invert_motor" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="invert_motor" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="remote_helm">
        <span class="sf-lbl" title="Allow the web remote to directly steer the rudder via the REMOTE button. Default OFF — enable only when actively helming from this screen.">Remote Helm Control &#9432;</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="remote_helm" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="remote_helm" data-bval="false">OFF</button>
        </div>
      </div>

      <div class="ss-title">AUTOPILOT</div>
      <div class="sf-row" data-sfid="deadband_pct">
        <span class="sf-lbl">Deadband (%)</span>
        <input class="sf-inp" type="number" id="sf-deadband_pct" min="0.5" max="20" step="0.5">
      </div>
      <div class="sf-row" data-sfid="pgain">
        <span class="sf-lbl">P-Gain</span>
        <input class="sf-inp" type="number" id="sf-pgain" min="0.1" max="5.0" step="0.1">
      </div>
      <div class="sf-row" data-sfid="off_course_alarm_deg">
        <span class="sf-lbl">Off-Course Alarm (&#176;)</span>
        <input class="sf-inp" type="number" id="sf-off_course_alarm_deg" min="5" max="90" step="1">
      </div>
      <div class="sf-row" data-sfid="rudder_limit_port_pct">
        <span class="sf-lbl">Port Limit (%)</span>
        <input class="sf-inp" type="number" id="sf-rudder_limit_port_pct" min="55" max="100" step="1">
      </div>
      <div class="sf-row" data-sfid="rudder_limit_stbd_pct">
        <span class="sf-lbl">Stbd Limit (%)</span>
        <input class="sf-inp" type="number" id="sf-rudder_limit_stbd_pct" min="0" max="45" step="1">
      </div>

      <div class="ss-title">SAFETY</div>
      <div class="sf-row" data-sfid="auto_disengage_on_fault">
        <span class="sf-lbl">Auto-Disengage</span>
        <div class="sf-bool">
          <button class="sf-bb" data-boolid="auto_disengage_on_fault" data-bval="true">ON</button>
          <button class="sf-bb" data-boolid="auto_disengage_on_fault" data-bval="false">OFF</button>
        </div>
      </div>
      <div class="sf-row" data-sfid="comms_warn_threshold_pct">
        <span class="sf-lbl">Comms WARN (%)</span>
        <input class="sf-inp" type="number" id="sf-comms_warn_threshold_pct" min="1" max="50" step="1">
      </div>
      <div class="sf-row" data-sfid="comms_crit_threshold_pct">
        <span class="sf-lbl">Comms CRIT (%)</span>
        <input class="sf-inp" type="number" id="sf-comms_crit_threshold_pct" min="5" max="90" step="1">
      </div>

      <div class="ss-title">NOTIFICATIONS</div>
      <div class="sf-row" data-sfid="health_interval_min">
        <span class="sf-lbl" title="0 = one health report at boot only. Set minutes for repeated reports with stats accumulated over the period.">Health Report (min) &#9432;</span>
        <input class="sf-inp" type="number" id="sf-health_interval_min" min="0" max="1440" step="5">
      </div>
      <div class="sf-row" data-sfid="ping_interval_s">
        <span class="sf-lbl" title="Seconds between gateway pings. 0.5 = 2 per second. Increase if router rate-limits ICMP.">Ping Interval (s) &#9432;</span>
        <input class="sf-inp" type="number" id="sf-ping_interval_s" min="0.5" max="30" step="0.5">
      </div>

      <div class="ss-title">TESTS</div>
      <div class="sf-row">
        <span class="sf-lbl">Hardware Tests</span>
        <button class="sf-bb" id="settings-open-tests" style="padding:3px 14px">OPEN &#8250;</button>
      </div>

      <div class="ss-title">SYSTEM</div>
      <div class="sf-row">
        <span class="sf-lbl" title="Check GitHub for new commits and optionally install.">Software Update &#9432;</span>
        <button class="sf-bb" id="sf-check-update" style="padding:3px 14px">CHECK &#8250;</button>
      </div>

    </div><!-- .sbody -->
    <div class="sftr">
      <button class="sftr-btn sv"  id="sov-save">SAVE</button>
      <button class="sftr-btn cx" id="sov-cancel">CANCEL</button>
    </div>
  </div><!-- .spanel -->
</div><!-- .sov -->

<!-- TEST overlay -->
<div class="tov hidden" id="tov">
  <div class="tpanel">
    <div class="thdr">
      <span class="thdr-title" id="tov-title">&#x2697; HARDWARE TESTS</span>
      <span class="thdr-hint" id="tov-hint">Select a test to view description</span>
    </div>
    <div class="tbody" id="tbody">

      <!-- View 1: Test catalogue list (populated by JS) -->
      <div id="tcat-view"></div>

      <!-- View 2: Test detail + YES/NO confirm -->
      <div class="tdet-view" id="tdet-view">
        <div class="tdet-name"    id="tdet-name"></div>
        <div class="tdet-section">Description</div>
        <div class="tdet-text"    id="tdet-desc"></div>
        <div class="tdet-section">Purpose</div>
        <div class="tdet-text"    id="tdet-purpose"></div>
        <div class="tdet-section">Expected Output</div>
        <div class="tdet-text"    id="tdet-output"></div>
        <div class="tdet-section">Required Firmware</div>
        <div class="tdet-fw"      id="tdet-fw"></div>
        <div class="tdet-confirm">
          <button class="tdet-btn yes" id="tdet-yes">YES &mdash; RUN</button>
          <button class="tdet-btn no"  id="tdet-no">NO &mdash; BACK</button>
        </div>
      </div>

      <!-- View 2b: RAM Test config -->
      <div class="tdet-view" id="tram-view">
        <div class="tdet-name">1. RAM Test</div>
        <div class="tdet-section">What it does</div>
        <div class="tdet-text">Sweeps the rudder continuously left and right between &#177;N&#176; of centre to stress-test the hydraulic ram. Uses the standard motor_simple firmware &#8212; no reflashing needed.</div>
        <div class="tdet-section">Sweep amplitude</div>
        <div style="display:flex;align-items:center;gap:10px;padding:6px 0 10px">
          <input type="number" id="ram-deg-inp" min="1" max="35" value="10"
                 style="width:64px;background:#080f20;border:1px solid #1e3450;border-radius:4px;color:#00d4ff;font-family:'Courier New',monospace;font-size:.95em;padding:4px 6px;outline:none;text-align:center">
          <span style="color:#90a8c0;font-size:.78em">degrees each side of centre<br><span id="ram-deg-hint" style="color:#354860"></span></span>
        </div>
        <div class="tdet-confirm">
          <button class="tdet-btn yes" id="tram-yes">PROCEED</button>
          <button class="tdet-btn no"  id="tram-no">BACK</button>
        </div>
      </div>

      <!-- View 3: Results / output streaming -->
      <div class="tres-view" id="tres-view">
        <div class="tres-name"   id="tres-name"></div>
        <div class="tres-log"    id="tres-log"></div>
        <div class="tres-status" id="tres-status">WAITING&hellip;</div>
        <button class="tres-b3"  id="tres-b3">SEND B3 (TRIGGER)</button>
      </div>

    </div><!-- .tbody -->
    <div class="tftr">
      <button class="tftr-btn back"  id="tov-back"  style="display:none">&#8592; BACK</button>
      <button class="tftr-btn close" id="tov-close">CLOSE</button>
    </div>
  </div><!-- .tpanel -->
</div><!-- .tov -->

<!-- Packet-loss warning banner: shown when net_warn is true in SSE state -->
<div id="net-warn-banner">NETWORK: Packet loss elevated &mdash; check WiFi / router</div>

<!-- Software update modal -->
<div id="upd-overlay" class="upd-overlay">
  <div class="upd-box" id="upd-box">
    <div class="upd-title">Software Update</div>
    <div id="upd-body" class="upd-body">Checking...</div>
    <pre id="upd-log"></pre>
    <div class="upd-actions">
      <button class="upd-btn upd-install" id="upd-install">INSTALL</button>
      <button class="upd-btn upd-close"   id="upd-close">CLOSE</button>
    </div>
  </div>
</div>

<!-- Boat-name setup modal: shown on load when vessel.name is not configured -->
<div id="bnm-overlay" class="bnm-overlay">
  <div class="bnm-box">
    <div class="bnm-title">Welcome to Inno-Pilot</div>
    <div class="bnm-body">Enter your boat name to continue.<br>
      This identifies your vessel in all notifications.</div>
    <input class="bnm-inp" type="text" id="bnm-input"
           placeholder="e.g. Windseeker" maxlength="32" autocomplete="off">
    <div class="bnm-err" id="bnm-err"></div>
    <button class="bnm-btn" id="bnm-save">SAVE</button>
  </div>
</div>

<script>
'use strict';

// ── Shared state ──────────────────────────────────────────────────────────
var gMode      = 'IDLE';
var gConnected = false;
var gApOn      = false;      // true when AP is actually engaged (d.ap === 1)
var gTogglePos = 'on';       // mode radio position: 'auto' | 'remote' | 'on' | 'off'
var gHdg       = null;       // latest heading from bridge telemetry (degrees)
var gCmd       = null;       // latest AP heading command from bridge telemetry (degrees)
var wheelAngle = 0;      // accumulated rotation in degrees, clamped to ±MAX_DEG
var isDragging = false;
var prevPtrAngle = null;
var lastRudSend  = 0;
var MAX_DEG = 150;       // ±150° maps 0..100% rudder
var gManualRudPct = 50.0;  // commanded rudder % in MANUAL mode (0–100%)
var gRdrPct       = null;  // latest rdr_pct from bridge telemetry
var gJogTimer     = null;  // setInterval handle for hold-jog repeat
var gHdgLastSeen  = Date.now();  // timestamp of last non-null HDG from bridge (grace period on load)
var gJogHoldTimer = null;  // setTimeout handle for jog hold-delay
var gSettings     = {};    // settings loaded from /settings endpoint
var gDebugActive  = false; // true when bridge is in DEBUG log level
var gProgressES   = null;  // EventSource for /update/progress SSE stream
var gSettingsOpen = false; // true while settings panel is visible
var gTestOpen     = false; // true while test modal is visible
var gSelectedTest = null;  // id of test currently shown in detail/results view
var gTestRunning  = false; // true while a test is streaming results
var gRamTestDeg   = 10;    // last configured RAM test amplitude (degrees)

// ── Command sender ────────────────────────────────────────────────────────
function sendCmd(cmd) {
  return fetch('/command', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({cmd: cmd})
  }).then(function(resp) {
    return resp.json().catch(function() { return {}; }).then(function(body) {
      return {ok: resp.ok, status: resp.status, body: body};
    });
  }).catch(function() {
    return {ok: false, status: 0, body: {}};
  });
}


// ── Hardware test catalogue ───────────────────────────────────────────────
var TESTS = [
  { id: 1,
    name: 'RAM Test',
    desc: 'Sweeps the rudder continuously between \u00b1N\u00b0 of centre to stress-test the hydraulic ram. Runs on the standard motor_simple firmware \u2014 no reflashing needed.',
    purpose: 'Verify the ram moves freely and reliably across its full working range under continuous load.',
    output: 'Live rudder position on the bar. Start/stop with B3. Emergency stop with STOP button.',
    fw: 'motor_simple.ino (standard firmware \u2014 already flashed)',
    trigger: 'B3 to start / stop'
  },
  { id: 2,
    name: 'Binary Search \u2014 Min Starting PWM',
    desc: 'Bisects the PWM range (0\u2013255) in both STBD and PORT directions to find the lowest duty cycle that actually moves the rudder from standstill.',
    purpose: 'Establish the motor deadband floor. Below this PWM the hydraulic valve does not crack open. Required before any closed-loop control tuning.',
    output: 'MIN_STBD and MIN_PORT PWM thresholds with motor current at each threshold. Example: MIN_STBD=118  MIN_PORT=122  CURRENT=0.8A',
    fw: 'pwm_test.ino  (default compile \u2014 no extra #define)',
    trigger: 'Automatic on power-up'
  },
  { id: 2,
    name: 'Speed Sweep',
    desc: 'Drives the rudder at fixed PWM steps (100 \u2192 255, \u223c15 per step) for 600 ms each. Records counts/second and motor current at each step.',
    purpose: 'Map motor speed as a function of PWM. Used to build the braking-time lookup table and characterise the full actuator range.',
    output: 'Table with \u223c10 rows: PWM | cps (counts/s) | amps.',
    fw: 'pwm_test.ino  (default compile \u2014 no extra #define)',
    trigger: 'Automatic'
  },
  { id: 4,
    name: 'Coast / Ramp-Down Test',
    desc: 'For PWM 150, 190, and 255: (A) hard-cuts the motor at a trigger point and measures overshoot; (B) applies a proportional ramp-down and measures the improvement side-by-side.',
    purpose: 'Quantify overshoot from hard-cut versus speed-proportional ramp braking. Determines whether ramp braking is worth implementing for this vessel.',
    output: '3 rows: PWM | coast_counts | hard_error | ramp_error. Side-by-side comparison for each speed.',
    fw: 'pwm_test.ino  (default compile \u2014 no extra #define)',
    trigger: 'Automatic'
  },
  { id: 5,
    name: 'Comprehensive Characterisation Table',
    desc: 'For each of 15 PWM levels (120 \u2192 255): measures coast count on hard-cut, minimum pulse width (binary search), speed (cps), and reverse-brake ms needed for dead-stop.',
    purpose: 'Produce the master reference table. This is the single most important calibration run \u2014 its output feeds all control algorithm parameters.',
    output: '15-row table: PWM | coast_ms | min_pulse_ms | cps | brake_ms. Full hydraulic characterisation for this vessel.',
    fw: 'pwm_test.ino  (default compile \u2014 no extra #define)',
    trigger: 'Automatic'
  },
  { id: 6,
    name: 'Interactive Target Test',
    desc: 'Drives 450-count runs alternating STBD/PORT (6 total). B3 triggers each run. Two algorithms: ALG 0 = hard-cut baseline; ALG 1 = speed-aware braking (replica of motor_simple B5). Reports overshoot, drive time, speed at brake trigger, brake ms used.',
    purpose: 'Validate the closed-loop stopping algorithm against a fixed target under realistic conditions. Directly measures real-world overshoot.',
    output: '6 rows: dir | target | actual | overshoot | drive_ms | brake_ms | speed@trigger.',
    fw: 'pwm_test.ino  #define RUN_INTERACTIVE_TEST',
    trigger: 'B3 button to start each run'
  },
  { id: 7,
    name: 'Burst Sweep Test',
    desc: '24 burst durations (10\u2013600 ms) \xd7 2 directions = 48 fully automatic runs after one B3 press. PWM=255, hard-cut. Records ADC start/final and net counts per burst.',
    purpose: 'Map the hydraulic cracking threshold \u2014 below what pulse duration does the rudder not move? Reveals burst duration vs. displacement relationship.',
    output: '48 rows: dir | duration_ms | start_adc | final_adc | net_counts. Identifies minimum effective pulse width.',
    fw: 'pwm_test.ino  #define RUN_BURST_SWEEP_TEST',
    trigger: 'B3 once to start \u2014 then fully automatic'
  },
  { id: 8,
    name: 'Fine Burst Test',
    desc: 'Steps 15\u201325 ms in 1 ms increments, 10 reps per step, both directions. PWM=255, hard-cut. Characterises fine-pulse resolution at the low end of burst duration.',
    purpose: 'Find the minimum repeatable pulse that moves the rudder by at least 1 ADC count. Critical for precision micro-corrections in closed-loop control.',
    output: '220 rows (10 reps \xd7 11 steps \xd7 2 dirs): dir | duration_ms | counts_moved. Statistical view of fine-pulse repeatability.',
    fw: 'pwm_test.ino  #define RUN_FINE_BURST_TEST',
    trigger: 'B3 once to start \u2014 then fully automatic'
  },
  { id: 9,
    name: 'Remote Control Test (RCT)',
    desc: 'Bridge-integrated positioning trial with 3 sub-modes: ADJUST (B2/B4 scroll settings, B1/B5 change value, B3 save); TEST (bridge sends TGT <pct>, Nano chases target using 7 RCT settings); RATIFY (real-time remote steering). Settings persist to EEPROM.',
    purpose: 'End-to-end validation of the full closed-loop positioning system from web remote through bridge to Nano, including EEPROM-persisted tuning parameters.',
    output: 'Real-time telemetry: TGT | RDR_PCT | overshoot | PWM. EEPROM parameter dump on ADJUST save. B3 cycles: ADJUST \u2192 TEST \u2192 RATIFY.',
    fw: 'pwm_test.ino  #define RUN_REMOTE_CONTROL_TEST  (bridge-integrated \u2014 full TCP protocol)',
    trigger: 'B3 cycles modes; B1/B2/B4/B5 adjust parameters'
  }
];

// ── Debug toggle ──────────────────────────────────────────────────────────

function setDebugState(active) {
  gDebugActive = !!active;
  var btn = document.getElementById('dbg-btn');
  btn.textContent = gDebugActive ? 'Debugging' : 'Debug';
  btn.classList.toggle('dbg-active', gDebugActive);
}

function toggleDebug() {
  fetch('/debug', { method: 'POST' })
    .then(function(r) { return r.json(); })
    .then(function(d) { if (d.debug !== undefined) setDebugState(d.debug); })
    .catch(function() {});  // silently ignore network errors
}

// ── Test modal functions ──────────────────────────────────────────────────

function openTestMenu() {
  gTestOpen = true;
  document.getElementById('tov').classList.remove('hidden');
  showTestCatalogue();
}

function closeTestMenu() {
  gTestOpen     = false;
  gTestRunning  = false;
  gSelectedTest = null;
  document.getElementById('tov').classList.add('hidden');
  document.getElementById('tram-view').classList.remove('active');
}

function showTestCatalogue() {
  document.getElementById('tov-title').textContent = '\u2697 HARDWARE TESTS';
  document.getElementById('tov-hint').textContent  = 'Select a test to view description';
  document.getElementById('tram-view').classList.remove('active');
  // Populate catalogue list on first call (idempotent)
  var catView = document.getElementById('tcat-view');
  catView.innerHTML = '';
  TESTS.forEach(function(t) {
    var item = document.createElement('div');
    item.className = 'tcat-item';
    item.innerHTML =
      '<span class="tcat-num">' + t.id + '</span>' +
      '<span class="tcat-name">' + t.name + '</span>' +
      '<span class="tcat-arrow">\u203a</span>';
    item.addEventListener('click', function() { selectTest(t.id); });
    item.addEventListener('touchstart', function(e) {
      e.preventDefault(); selectTest(t.id);
    }, {passive: false});
    catView.appendChild(item);
  });
  catView.classList.remove('hidden');
  document.getElementById('tdet-view').classList.remove('active');
  document.getElementById('tres-view').classList.remove('active');
  document.getElementById('tov-back').style.display = 'none';
}

function showRamTestConfig() {
  gSelectedTest = 1;
  var rng = (gSettings && gSettings.vessel && gSettings.vessel.rudder_range_deg)
            ? gSettings.vessel.rudder_range_deg : 35;
  document.getElementById('ram-deg-inp').max   = rng;
  document.getElementById('ram-deg-inp').value = Math.min(gRamTestDeg, rng);
  document.getElementById('ram-deg-hint').textContent = 'max ' + rng + '° (travel limit)';
  document.getElementById('tov-title').textContent = 'RAM Test';
  document.getElementById('tov-hint').textContent  = 'Set sweep amplitude then press PROCEED';
  document.getElementById('tcat-view').classList.add('hidden');
  document.getElementById('tdet-view').classList.remove('active');
  document.getElementById('tres-view').classList.remove('active');
  document.getElementById('tram-view').classList.add('active');
  document.getElementById('tov-back').style.display = '';
}

function selectTest(id) {
  if (id === 1) { showRamTestConfig(); return; }
  var t = TESTS.find(function(x) { return x.id === id; });
  if (!t) return;
  gSelectedTest = id;
  document.getElementById('tdet-name').textContent    = t.id + '.  ' + t.name;
  document.getElementById('tdet-desc').textContent    = t.desc;
  document.getElementById('tdet-purpose').textContent = t.purpose;
  document.getElementById('tdet-output').textContent  = t.output;
  document.getElementById('tdet-fw').textContent      = t.fw;
  document.getElementById('tov-title').textContent    = 'TEST ' + t.id;
  document.getElementById('tov-hint').textContent     = 'Trigger: ' + t.trigger;
  document.getElementById('tcat-view').classList.add('hidden');
  document.getElementById('tdet-view').classList.add('active');
  document.getElementById('tres-view').classList.remove('active');
  document.getElementById('tov-back').style.display = '';
}

function runTest(id) {
  var t = TESTS.find(function(x) { return x.id === id; });
  if (!t) return;
  gTestRunning = true;
  document.getElementById('tres-name').textContent   = t.id + '.  ' + t.name;
  document.getElementById('tres-log').textContent    = '';
  document.getElementById('tres-status').textContent = 'Sending TEST ' + id + ' to Nano\u2026';
  // Show B3 button for tests that require it (ids 6-8 after renumber)
  var needsB3 = (id >= 6);
  document.getElementById('tres-b3').classList.toggle('visible', needsB3);
  document.getElementById('tdet-view').classList.remove('active');
  document.getElementById('tres-view').classList.add('active');
  document.getElementById('tov-back').style.display = 'none';
  // Forward TEST command through bridge to Nano
  sendCmd('TEST ' + id);
}

function appendTestLine(line) {
  if (!gTestOpen) return;
  var log = document.getElementById('tres-log');
  if (!log) return;
  log.textContent += line + '\\n';
  log.scrollTop = log.scrollHeight;
  document.getElementById('tres-status').textContent = 'RUNNING\u2026';
}

function testDone() {
  gTestRunning = false;
  var status = document.getElementById('tres-status');
  if (status) status.textContent = 'TEST COMPLETE';
  document.getElementById('tres-b3').classList.remove('visible');
  document.getElementById('tov-back').style.display = '';
}

// Test button and modal event wiring
document.getElementById('dbg-btn').addEventListener('click', toggleDebug);
document.getElementById('dbg-btn').addEventListener('touchstart', function(e) {
  e.preventDefault(); toggleDebug();
}, {passive: false});
document.getElementById('tov-close').addEventListener('click', closeTestMenu);
document.getElementById('tov-back').addEventListener('click', function() {
  // Back always returns to catalogue (from detail or results)
  showTestCatalogue();
});
document.getElementById('tdet-yes').addEventListener('click', function() {
  if (gSelectedTest) runTest(gSelectedTest);
});
document.getElementById('tdet-no').addEventListener('click', showTestCatalogue);
document.getElementById('tram-no').addEventListener('click', showTestCatalogue);
document.getElementById('tram-yes').addEventListener('click', function() {
  var rng = (gSettings && gSettings.vessel && gSettings.vessel.rudder_range_deg)
            ? gSettings.vessel.rudder_range_deg : 35;
  var deg = parseInt(document.getElementById('ram-deg-inp').value, 10) || 10;
  deg = Math.max(1, Math.min(deg, rng));
  gRamTestDeg = deg;
  closeTestMenu();
  // Switch toggle to auto position so B3 is labelled correctly
  if (gTogglePos !== 'auto') handleToggleAction('auto');
  sendCmd('RAM_SETUP ' + deg);
});
document.getElementById('tres-b3').addEventListener('click', function() {
  // Send B3 (BTN TOGGLE) — trigger for tests 5, 6, 7
  sendCmd('BTN TOGGLE');
});

// ── SSE subscription ──────────────────────────────────────────────────────
var es = new EventSource('/events');
es.onmessage = function(e) { updateUI(JSON.parse(e.data)); };
es.onerror   = function()  { setConnected(false); };

// ── UI updater ────────────────────────────────────────────────────────────
function updateUI(d) {
  gConnected = !!d.connected;
  gMode      = d.mode || 'IDLE';
  gApOn      = !!d.ap;
  gRdrPct    = d.rdr_pct != null ? d.rdr_pct : null;
  if (d.hdg != null) { gHdg = d.hdg; gHdgLastSeen = Date.now(); }
  if (d.cmd != null) gCmd = d.cmd;
  if (d.ui_mode) gTogglePos = d.ui_mode;
  if (d.debug !== undefined) setDebugState(d.debug);
  if (d.net_warn !== undefined) {
    document.getElementById('net-warn-banner').classList.toggle('visible', !!d.net_warn);
  }

  // Suppress overlay when connected, or when disconnect is intentional (OFF).
  // This also handles occasional mode/ui-mode desync by treating the selector
  // position as authoritative for intentional offline state.
  setConnected(gConnected || gMode === 'OFF' || gTogglePos === 'off');

  // OLED mode line — AUTO: show AP ON/OFF with oversized "AP"; other modes: plain label
  var modeRow = document.getElementById('oled-mode-row');
  var b3btn   = document.querySelector('.hw-btn.b3');
  if (gMode === 'RAM_ON') {
    modeRow.innerHTML = 'MODE: <b style="color:#ff6a00">RAM Test ON</b>';
    if (b3btn) b3btn.textContent = 'Stop';
  } else if (gMode === 'RAM_OFF') {
    modeRow.innerHTML = 'MODE: <b style="color:#e09000">RAM Test OFF</b>';
    if (b3btn) b3btn.textContent = 'Run';
  } else if (gMode === 'AP' && gApOn) {
    modeRow.innerHTML = 'MODE: <span class="ap-label">AP</span>\u00a0ON';
    if (b3btn) b3btn.textContent = 'Off';  // AP is engaged; pressing will disengage
  } else {
    modeRow.innerHTML = 'MODE: <b id="o-mode">' + (gMode || 'IDLE') + '</b>';
    if (b3btn) b3btn.textContent = gTogglePos === 'auto' ? 'On' : 'Go';  // AUTO: engage; other modes: generic
  }

  // Heading / CMD — always displayed; integer degrees, right-aligned in 3-char field
  document.getElementById('o-hdg').textContent =
    d.hdg != null ? String(Math.round(d.hdg)).padStart(3) : '---';
  document.getElementById('o-cmd').textContent =
    d.cmd != null ? String(Math.round(d.cmd)).padStart(3) : '---';

  // Rudder position bar (uses rdr_pct; falls back to 50% centre)
  // Convention: rdr_pct=100=port (left of bar), rdr_pct=0=stbd (right of bar)
  // — see conv. 4 (linear graphic: port=left, stbd=right).
  var pct = d.rdr_pct != null ? Math.max(0, Math.min(100, d.rdr_pct)) : 50;
  document.getElementById('rdr-marker').style.left = (100 - pct) + '%';

  // Servo command direction triangle: track the marker position so the arrow sits
  // directly against the position block rather than fixed at the bar ends.
  // markerLeft is the same coordinate used for rdr-marker; port arrow sits 18px to
  // the left (6px half-marker + 12px arrow width), stbd sits 6px to the right.
  var arrowPort = document.getElementById('rdr-arrow-port');
  var arrowStbd = document.getElementById('rdr-arrow-stbd');
  var markerLeft = (100 - pct) + '%';
  arrowPort.style.left = 'calc(' + markerLeft + ' - 18px)';
  arrowStbd.style.left = 'calc(' + markerLeft + ' + 6px)';
  arrowPort.style.display = (d.rdr_cmd ===  1) ? 'block' : 'none';
  arrowStbd.style.display = (d.rdr_cmd === -1) ? 'block' : 'none';

  // Version + status line (priority order: stall > no-bridge > motor-fault > no-heading > comms-warn > ok)
  document.getElementById('o-ver').textContent = d.bridge_ver || d.version || '---';
  var connEl = document.getElementById('o-conn');
  var comms  = (d.comms || 'OK').toUpperCase();
  if (d.connected && d.rudder_stall) {
    // Highest priority: motor commanded but rudder position not changing
    connEl.textContent = 'RUDDER NOT RESPONDING';
    connEl.className   = 'crit';
  } else if (!d.connected && gTogglePos !== 'off') {
    connEl.textContent = 'NO BRIDGE';
    connEl.className   = 'crit';
  } else if (d.connected && comms === 'CRIT') {
    // Motor driver CRC error rate at critical level
    connEl.textContent = 'MOTOR FAULT';
    connEl.className   = 'crit';
  } else if (d.connected && d.pypilot_ok === false) {
    // pypilot process has lost its internal connection
    connEl.textContent = 'NO HEADING';
    connEl.className   = 'warn';
  } else if (d.connected && comms === 'WARN') {
    connEl.textContent = 'COMMS WARN';
    connEl.className   = 'warn';
  } else if (d.connected) {
    connEl.textContent = 'CONNECTED';
    connEl.className   = 'ok';
  } else {
    connEl.textContent = 'OFFLINE';
    connEl.className   = 'warn';   // amber — intentional disconnect (OFF mode)
  }

  // Mode radio selector — driven by physical position, not bridge mode
  setToggle(gTogglePos);
  // Blank the OLED content panel when in OFF mode
  document.querySelector('.oled').classList.toggle('blank-mode', gTogglePos === 'off');

  // Wheel active state + position sync
  var ww = document.getElementById('wheel-wrap');
  if (gMode === 'MANUAL') {
    ww.classList.add('active');
    // In MANUAL: keep wheel at drag position, don't override with telemetry
  } else {
    ww.classList.remove('active');
    // Sync wheel visual to actual rudder position.
    // Convention 3: CW (positive CSS rotate) = stbd. With rdr_pct=100=port,
    // we negate so port (high pct) maps to CCW.
    if (d.rdr_pct != null) {
      wheelAngle = -(d.rdr_pct - 50) / 50 * MAX_DEG;
      document.getElementById('wheel-svg').style.transform =
        'rotate(' + wheelAngle + 'deg)';
    }
  }

  // Test result streaming — bridge relays TEST_LINE / TEST_DONE from Nano
  if (d._test_line) appendTestLine(d._test_line);
  if (d._test_done) testDone();
}

function setConnected(ok) {
  document.getElementById('overlay').classList.toggle('hidden', ok);
}

function setToggle(m) {
  // Sync animated radio button active state
  document.querySelectorAll('.mode-radio').forEach(function(el) {
    el.classList.toggle('active', el.dataset.action === m);
  });

  // Nudge buttons: enabled only in AUTO mode (covers both IDLE and AP ON)
  var nudgeEnabled = (m === 'auto');
  document.getElementById('nudge-port').disabled = !nudgeEnabled;
  document.getElementById('nudge-stbd').disabled = !nudgeEnabled;

  // Physical button labels — shown in AUTO and REMOTE modes only
  var btns = [
    document.querySelector('.hw-btn.b1'),
    document.querySelector('.hw-btn.b2'),
    document.querySelector('.hw-btn.b3'),
    document.querySelector('.hw-btn.b4'),
    document.querySelector('.hw-btn.b5'),
  ];
  if (m === 'auto') {
    btns[0].textContent = '-10';
    btns[1].textContent = '-1';
    btns[2].textContent = gApOn ? 'Off' : 'On';  // reflect live AP state
    btns[3].textContent = '+1';
    btns[4].textContent = '+10';
  } else if (m === 'remote') {
    btns[0].innerHTML = '&laquo;';
    btns[1].innerHTML = '&lsaquo;';
    btns[2].innerHTML = '|';
    btns[3].innerHTML = '&rsaquo;';
    btns[4].innerHTML = '&raquo;';
  } else {
    // OFF and ON: no button labels
    btns.forEach(function(b) { b.textContent = ''; });
  }
}

// ── Manual-mode jog helpers ───────────────────────────────────────────────
// jogRudder: adjust gManualRudPct by deltaPct, sync wheel visual, send RUD.
function jogRudder(deltaPct) {
  gManualRudPct = Math.max(0, Math.min(100, gManualRudPct + deltaPct));
  wheelAngle = -((gManualRudPct - 50) / 50 * MAX_DEG);
  wheelSvg.style.transform = 'rotate(' + wheelAngle + 'deg)';
  sendCmd('RUD ' + gManualRudPct.toFixed(1));
}

// startJog: immediate first jog, then after 300 ms hold delay repeat at intervalMs.
function startJog(deltaPct, intervalMs) {
  stopJog();
  jogRudder(deltaPct);
  gJogHoldTimer = setTimeout(function() {
    gJogHoldTimer = null;
    gJogTimer = setInterval(function() {
      if (gMode !== 'MANUAL') { stopJog(); return; }
      // Stop repeating when the limit is already reached
      var next = gManualRudPct + deltaPct;
      if (next < 0 || next > 100) { stopJog(); return; }
      jogRudder(deltaPct);
    }, intervalMs);
  }, 300);
}

function stopJog() {
  if (gJogTimer     !== null) { clearInterval(gJogTimer);     gJogTimer     = null; }
  if (gJogHoldTimer !== null) { clearTimeout(gJogHoldTimer);  gJogHoldTimer = null; }
}

// ── Button wiring ─────────────────────────────────────────────────────────
// MANUAL mode: B1=port 10% jog (hold=continuous), B2=port 1° jog (hold=500ms),
//              B3=centre rudder, B4=stbd 1° (hold=500ms), B5=stbd 10% (hold=continuous).
// AUTO/IDLE mode: send BTN commands as before.
(function() {
  var btnCfg = [
    { cls: 'b1', cmd: 'BTN -10',     delta: +10, interval: 200 },
    { cls: 'b2', cmd: 'BTN -1',      delta: +1,  interval: 500 },
    { cls: 'b3', cmd: 'BTN TOGGLE',  delta: null },
    { cls: 'b4', cmd: 'BTN +1',      delta: -1,  interval: 500 },
    { cls: 'b5', cmd: 'BTN +10',     delta: -10, interval: 200 },
  ];

  btnCfg.forEach(function(cfg) {
    var el = document.querySelector('.hw-btn.' + cfg.cls);
    if (!el) return;

    function onPress() {
      if (gMode === 'MANUAL') {
        if (cfg.delta === null) {
          // B3: centre rudder immediately
          stopJog();
          gManualRudPct = 50.0;
          wheelAngle    = 0;
          wheelSvg.style.transform = 'rotate(0deg)';
          sendCmd('RUD 50.0');
        } else {
          startJog(cfg.delta, cfg.interval);
        }
      } else if (cfg.cls === 'b3' && gTogglePos === 'auto' && !gApOn) {
        // GO in AUTO mode while AP is off: lock onto current heading then engage.
        sendCmd('AP_ENGAGE_AT_HDG').then(function(res) {
          if (!res.ok || res.body.ok === false) {
            var msg = (res.body && res.body.error) ? res.body.error : 'AP engage failed';
            showWarning(msg);
          }
        });
      } else {
        sendCmd(cfg.cmd);
      }
    }

    function onRelease() {
      // Only stop jog for directional buttons; centre has no hold behaviour
      if (gMode === 'MANUAL' && cfg.delta !== null) stopJog();
    }

    el.addEventListener('mousedown',   function(e) { e.preventDefault(); onPress(); });
    el.addEventListener('mouseup',     onRelease);
    el.addEventListener('mouseleave',  onRelease);
    el.addEventListener('touchstart',  function(e) { e.preventDefault(); onPress(); }, {passive: false});
    el.addEventListener('touchend',    onRelease);
    el.addEventListener('touchcancel', onRelease);
  });
}());

document.getElementById('stop-btn').addEventListener('click', function() {
  sendCmd('ESTOP');
});
document.getElementById('stop-btn').addEventListener('touchstart', function(e) {
  e.preventDefault(); sendCmd('ESTOP');
}, {passive: false});

// ── Nudge buttons — hold-to-run, active in AUTO mode only ──
// Press sends NUDGE PORT/STBD once; release sends NUDGE STOP after 100 ms debounce.
// The debounce prevents a fast tap delivering START+STOP so close together that the
// bridge processes them before the motor has engaged (causes the "hesitation" symptom).
// gNudgeActive stays true until the deferred STOP is actually sent, so a second press
// within the 100 ms window is correctly rejected rather than starting a second nudge.
(function() {
  var gNudgeActive   = false;  // true while a nudge is in progress (including stop debounce)
  var gNudgeStopTimer = null;  // setTimeout handle for the deferred NUDGE STOP

  function startNudge(dir) {
    // Guard: reject if not in AUTO mode (belt-and-suspenders alongside disabled attr)
    if (gTogglePos !== 'auto') return;
    if (gNudgeActive) return;
    // Cancel any pending stop from a previous release (should not happen given the
    // gNudgeActive guard, but be safe)
    if (gNudgeStopTimer) { clearTimeout(gNudgeStopTimer); gNudgeStopTimer = null; }
    gNudgeActive = true;
    sendCmd('NUDGE ' + dir);
  }

  function stopNudge() {
    if (!gNudgeActive) return;
    // 100 ms debounce: delay STOP so the bridge has time to engage the motor before
    // the STOP arrives.  gNudgeActive stays true until the STOP is sent, blocking a
    // second nudge start during this window.
    if (gNudgeStopTimer) return;  // already waiting
    gNudgeStopTimer = setTimeout(function() {
      gNudgeStopTimer = null;
      gNudgeActive = false;
      sendCmd('NUDGE STOP');
    }, 100);
  }

  var portBtn = document.getElementById('nudge-port');
  var stbdBtn = document.getElementById('nudge-stbd');

  portBtn.addEventListener('mousedown',   function()  { startNudge('PORT'); });
  portBtn.addEventListener('mouseup',     stopNudge);
  portBtn.addEventListener('mouseleave',  stopNudge);
  portBtn.addEventListener('touchstart',  function(e) { e.preventDefault(); startNudge('PORT'); }, {passive: false});
  portBtn.addEventListener('touchend',    function(e) { e.preventDefault(); stopNudge(); }, {passive: false});
  portBtn.addEventListener('touchcancel', stopNudge);

  stbdBtn.addEventListener('mousedown',   function()  { startNudge('STBD'); });
  stbdBtn.addEventListener('mouseup',     stopNudge);
  stbdBtn.addEventListener('mouseleave',  stopNudge);
  stbdBtn.addEventListener('touchstart',  function(e) { e.preventDefault(); startNudge('STBD'); }, {passive: false});
  stbdBtn.addEventListener('touchend',    function(e) { e.preventDefault(); stopNudge(); }, {passive: false});
  stbdBtn.addEventListener('touchcancel', stopNudge);
}());

// Mode radio button clicks
document.querySelectorAll('.mode-radio').forEach(function(el) {
  el.addEventListener('click', function() { handleToggleAction(el.dataset.action); });
  el.addEventListener('touchstart', function(e) {
    e.preventDefault(); handleToggleAction(el.dataset.action);
  }, {passive: false});
});

function handleToggleAction(action) {
  if (action === 'auto') {
    // Enter AUTO-ready state; AP starts OFF.  User presses Go to engage/disengage AP.
    gTogglePos = 'auto';
    if (gMode !== 'AP') sendCmd('MODE AUTO');

  } else if (action === 'off') {
    if (gApOn)                    sendCmd('BTN TOGGLE');  // disengage AP before disconnect
    if (gTogglePos === 'remote')  sendCmd('MODE AUTO');   // exit REMOTE before disconnect
    gTogglePos = 'off';
    // Delay gives the preceding command time to reach the bridge before the TCP
    // connection is dropped by MODE OFF (bridge loop polls every ~100 ms).
    setTimeout(function() { sendCmd('MODE OFF'); }, 300);

  } else if (action === 'on') {
    // Observer mode: connect (or reconnect), show telemetry, no steering control.
    if (gApOn)                    sendCmd('BTN TOGGLE');  // disengage AP
    if (gTogglePos === 'remote')  sendCmd('MODE AUTO');   // exit REMOTE steering → IDLE
    gTogglePos = 'on';
    // Send MODE ON to wake the bridge thread if it was idle due to mode OFF;
    // the server handles it locally without forwarding a control command.
    sendCmd('MODE ON');

  } else if (action === 'remote') {
    // REMOTE mode: helm wheel lights up, skipper commands the rudder.
    if (gMode === 'MANUAL') {
      gTogglePos = 'remote';
      return;
    }
    sendCmd('MODE MANUAL').then(function(res) {
      if (!res.ok) return;
      gTogglePos = 'remote';
      // Sync wheel graphic to actual rudder position (same sign as telemetry update).
      // Do NOT send an initial RUD command: the Nano seeds its own target from its
      // ADC on MANUAL_MODE_CODE receipt, so any RUD at this point would override
      // that correct seed with a potentially sign-inverted value.
      gManualRudPct = gRdrPct !== null ? gRdrPct : 50.0;
      // Same sign as telemetry sync: gManualRudPct=100 (port) → CCW.
      wheelAngle = -(gManualRudPct - 50) / 50 * MAX_DEG;
      document.getElementById('wheel-svg').style.transform = 'rotate(' + wheelAngle + 'deg)';
    });
  }
}

// ── Ship's wheel drag interaction ─────────────────────────────────────────
var wheelWrap = document.getElementById('wheel-wrap');
var wheelSvg  = document.getElementById('wheel-svg');

function wheelCenter() {
  var r = wheelWrap.getBoundingClientRect();
  return {x: r.left + r.width / 2, y: r.top + r.height / 2};
}

function ptrAngleDeg(cx, cy) {
  var c = wheelCenter();
  return Math.atan2(cy - c.y, cx - c.x) * 180 / Math.PI;
}

function wheelStart(cx, cy) {
  isDragging   = true;
  prevPtrAngle = ptrAngleDeg(cx, cy);
}

function wheelMove(cx, cy) {
  if (!isDragging || gMode !== 'MANUAL') return;
  var curr  = ptrAngleDeg(cx, cy);
  var delta = curr - prevPtrAngle;
  // Unwrap angle discontinuity at ±180°
  if (delta >  180) delta -= 360;
  if (delta < -180) delta += 360;
  prevPtrAngle = curr;

  wheelAngle = Math.max(-MAX_DEG, Math.min(MAX_DEG, wheelAngle + delta));
  wheelSvg.style.transform = 'rotate(' + wheelAngle + 'deg)';

  var pct = (-wheelAngle + MAX_DEG) / (2 * MAX_DEG) * 100;
  gManualRudPct = pct;  // keep button jog state in sync with wheel drag

  // Send RUD at max 5 Hz
  var now = Date.now();
  if (now - lastRudSend >= 200) {
    sendCmd('RUD ' + pct.toFixed(1));
    lastRudSend = now;
  }
}

function wheelEnd() {
  isDragging   = false;
  prevPtrAngle = null;
}

// Mouse events
wheelWrap.addEventListener('mousedown', function(e) {
  e.preventDefault(); wheelStart(e.clientX, e.clientY);
});
window.addEventListener('mousemove', function(e) { wheelMove(e.clientX, e.clientY); });
window.addEventListener('mouseup',   function()  { wheelEnd(); });

// Touch events
wheelWrap.addEventListener('touchstart', function(e) {
  e.preventDefault(); wheelStart(e.touches[0].clientX, e.touches[0].clientY);
}, {passive: false});
window.addEventListener('touchmove', function(e) {
  if (isDragging) { e.preventDefault(); wheelMove(e.touches[0].clientX, e.touches[0].clientY); }
}, {passive: false});
window.addEventListener('touchend', function() { wheelEnd(); });

// ── Settings panel ───────────────────────────────────────────────────────────
// Field registry — drives population, B-button navigation, and bool/enum control.
// type: 'text'|'password'|'number'|'bool'|'enum'
// onVal/offVal: stored value toggled by tapping the enum field.
// dep: {id, val} — field hidden unless named field equals val.
var SF = [
  // Network
  {id:'ip_mode', sec:'network', type:'enum',     onVal:'static', offVal:'dhcp'},
  {id:'ip',      sec:'network', type:'text',     dep:{id:'ip_mode', val:'static'}},
  {id:'mask',    sec:'network', type:'text',     dep:{id:'ip_mode', val:'static'}},
  {id:'gateway', sec:'network', type:'text',     dep:{id:'ip_mode', val:'static'}},
  {id:'dns1',    sec:'network', type:'text',     dep:{id:'ip_mode', val:'static'}},
  {id:'dns2',    sec:'network', type:'text',     dep:{id:'ip_mode', val:'static'}},
  // Wi-Fi
  {id:'ssid',    sec:'wifi',    type:'text'},
  {id:'key',     sec:'wifi',    type:'password'},
  // Vessel
  {id:'name',             sec:'vessel',   type:'text'},
  {id:'type',             sec:'vessel',   type:'enum',   onVal:'sail', offVal:'power'},
  {id:'rudder_range_deg', sec:'vessel',   type:'number'},
  // Features
  {id:'limit_switches',         sec:'features', type:'bool'},
  {id:'temp_sensor',            sec:'features', type:'bool'},
  {id:'pi_voltage_sensor',      sec:'features', type:'bool'},
  {id:'battery_voltage_sensor', sec:'features', type:'bool'},
  {id:'current_sensor',         sec:'features', type:'bool'},
  {id:'on_board_buttons',       sec:'features', type:'bool'},
  {id:'oled_sh1106',            sec:'features', type:'bool'},
  {id:'invert_clutch',          sec:'features', type:'bool'},
  {id:'invert_motor',           sec:'features', type:'bool'},
  {id:'remote_helm',            sec:'features', type:'bool'},
  // Autopilot
  {id:'deadband_pct',           sec:'autopilot', type:'number'},
  {id:'pgain',                  sec:'autopilot', type:'number'},
  {id:'off_course_alarm_deg',   sec:'autopilot', type:'number'},
  {id:'rudder_limit_port_pct',  sec:'autopilot', type:'number'},
  {id:'rudder_limit_stbd_pct',  sec:'autopilot', type:'number'},
  // Safety
  {id:'auto_disengage_on_fault',  sec:'safety', type:'bool'},
  {id:'comms_warn_threshold_pct', sec:'safety', type:'number'},
  {id:'comms_crit_threshold_pct', sec:'safety', type:'number'},
  // Notifications
  {id:'health_interval_min', sec:'notifications', type:'number'},
  {id:'ping_interval_s',     sec:'notifications', type:'number'},
];

function sfGet(f) { return (gSettings[f.sec] || {})[f.id]; }
function sfSet(f, v) {
  if (!gSettings[f.sec]) gSettings[f.sec] = {};
  gSettings[f.sec][f.id] = v;
}

// Returns visible (non-hidden) field list, honouring dep conditions.
function sfVisibleList() {
  return SF.filter(function(f) {
    if (!f.dep) return true;
    var d = SF.find(function(x) { return x.id === f.dep.id; });
    return d && String(sfGet(d)) === f.dep.val;
  });
}

// Show/hide rows for fields with dep conditions.
function sfSyncVisibility() {
  SF.forEach(function(f) {
    if (!f.dep) return;
    var d = SF.find(function(x) { return x.id === f.dep.id; });
    var show = d && String(sfGet(d)) === f.dep.val;
    var row = document.querySelector('.sf-row[data-sfid="' + f.id + '"]');
    if (row) row.classList.toggle('sf-hidden', !show);
  });
}

// Render ON/OFF button styles for a bool field.
function sfBoolRender(fid, isOn) {
  document.querySelectorAll('.sf-bb[data-boolid="' + fid + '"]').forEach(function(b) {
    b.classList.remove('bb-on', 'bb-off');
    if (b.dataset.bval === 'true'  &&  isOn) b.classList.add('bb-on');
    if (b.dataset.bval === 'false' && !isOn) b.classList.add('bb-off');
  });
}

// Populate all controls from gSettings.
function sfApplyToUI() {
  SF.forEach(function(f) {
    var v = sfGet(f);
    if (v === undefined || v === null) return;
    if (f.type === 'bool') {
      sfBoolRender(f.id, !!v);
    } else if (f.type === 'enum') {
      var el = document.getElementById('sf-' + f.id);
      if (el) el.textContent = String(v).toUpperCase();
    } else {
      var el = document.getElementById('sf-' + f.id);
      if (el) el.value = v;
    }
  });
  applyRemoteHelmSetting();
}

// Enable or disable the REMOTE mode-radio button based on the remote_helm setting.
function applyRemoteHelmSetting() {
  var enabled = !!(gSettings && gSettings.features && gSettings.features.remote_helm);
  var remBtn = document.querySelector('.mode-radio[data-action="remote"]');
  if (remBtn) remBtn.classList.toggle('disabled', !enabled);
}

// Read text/number/password inputs back into gSettings.
// (bool and enum values are written directly to gSettings on interaction.)
function sfCollect() {
  SF.forEach(function(f) {
    if (f.type === 'bool' || f.type === 'enum') return;
    var el = document.getElementById('sf-' + f.id);
    if (!el) return;
    if (!gSettings[f.sec]) gSettings[f.sec] = {};
    gSettings[f.sec][f.id] = (f.type === 'number') ? (parseFloat(el.value) || 0) : el.value;
  });
}


// Update the status line inside the settings header.
// type: 'ok' (green) | 'warn' (amber) | 'err' (red) | 'pend' (blue-grey) | '' (clear)
function setSovStatus(msg, type) {
  var el = document.getElementById('sov-status');
  el.textContent = msg;
  el.className = 'shdr-status' + (type ? ' s-' + type : '');
}

// showWarning: display a transient warning toast in the centre of the screen.
function showWarning(msg) {
  var t = document.getElementById('sw-toast');
  t.innerHTML = msg;
  t.classList.add('visible');
  setTimeout(function() { t.classList.remove('visible'); }, 2500);
}

function openSettings() {
  if (gTogglePos !== 'off') {
    showWarning('Settings only<br>available in OFF mode');
    return;
  }
  gSettingsOpen = true;
  document.getElementById('gear-btn').classList.add('settings-open');
  document.getElementById('sov').classList.remove('hidden');
  setToggle('off'); // re-renders buttons (override inside setToggle shows settings labels)
  setSovStatus('Loading\u2026', 'pend');
  fetch('/settings')
    .then(function(r) { return r.json(); })
    .then(function(s) {
      // _source is injected by server to indicate origin; strip before storing
      var src = s['_source'] || 'unknown';
      delete s['_source'];
      gSettings = s;
      sfApplyToUI();
      sfSyncVisibility();
      if (src === 'bridge') {
        setSovStatus('\u2713 Loaded from bridge', 'ok');
      } else {
        setSovStatus('\u26a0 Using local settings \u2014 bridge unavailable', 'warn');
      }
    })
    .catch(function() { setSovStatus('\u2717 Failed to load settings', 'err'); });
}

// Internal: tears down the settings overlay without any save action.
function _doClosePanel() {
  gSettingsOpen = false;
  document.getElementById('gear-btn').classList.remove('settings-open');
  document.getElementById('sov').classList.add('hidden');
  setToggle('off'); // restore button labels (empty in OFF mode, since gSettingsOpen is now false)
}

function closeSettings(save) {
  if (save) {
    sfCollect();
    setSovStatus('Saving\u2026', 'pend');
    // Abort controller: guarantees .catch() fires within SAVE_TIMEOUT_MS even if
    // the server is slow (Pi Zero SD write + bridge round-trip can take up to ~6 s).
    var ctrl = new AbortController();
    var abortTimer = setTimeout(function() { ctrl.abort(); }, 9000);
    // Keep panel open while the POST is in-flight so the user sees feedback.
    fetch('/settings', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify(gSettings),
      signal: ctrl.signal
    })
    .then(function(r) { clearTimeout(abortTimer); return r.json(); })
    .then(function(d) {
      var msg, type;
      if (d.ok && d.via === 'bridge') {
        msg = '\u2713 Saved via bridge'; type = 'ok';
      } else if (d.ok) {
        // Saved to local file — bridge was unavailable or timed out
        msg = '\u26a0 Saved locally \u2014 bridge unavailable'; type = 'warn';
      } else {
        msg = '\u2717 Save failed'; type = 'err';
      }
      setSovStatus(msg, type);
      applyRemoteHelmSetting();
      setTimeout(_doClosePanel, 1800);
    })
    .catch(function(err) {
      clearTimeout(abortTimer);
      var msg = (err && err.name === 'AbortError')
        ? '\u2717 Save timed out'
        : '\u2717 Save failed \u2014 network error';
      setSovStatus(msg, 'err');
      setTimeout(_doClosePanel, 2000);
    });
    return; // panel stays open until timeout above fires
  }
  _doClosePanel();
}

// Settings event wiring
document.getElementById('gear-btn').addEventListener('click', openSettings);
document.getElementById('gear-btn').addEventListener('touchstart', function(e) {
  e.preventDefault(); openSettings();
}, {passive:false});
document.getElementById('sov-save').addEventListener('click',   function() { closeSettings(true); });
document.getElementById('sov-cancel').addEventListener('click', function() { closeSettings(false); });
document.getElementById('settings-open-tests').addEventListener('click', function() {
  closeSettings(false);
  openTestMenu();
});

// Bool buttons: direct click updates gSettings immediately
document.querySelectorAll('.sf-bb').forEach(function(b) {
  b.addEventListener('click', function() {
    var fid = b.dataset.boolid;
    var val = b.dataset.bval === 'true';
    var f = SF.find(function(x) { return x.id === fid; });
    if (f) { sfSet(f, val); sfBoolRender(fid, val); }
  });
});

// Field row click: focus the input for text/number/password fields
document.querySelectorAll('.sf-row').forEach(function(row) {
  row.addEventListener('click', function(e) {
    var inp = row.querySelector('.sf-inp');
    if (inp && e.target !== inp) inp.focus();
  });
});
// Enum field: tap the displayed value to toggle between the two options
document.querySelectorAll('.sf-eval').forEach(function(el) {
  el.style.cursor = 'pointer';
  el.addEventListener('click', function(e) {
    e.stopPropagation();
    var row = el.parentElement;
    while (row && !row.classList.contains('sf-row')) { row = row.parentElement; }
    if (!row) return;
    var fid = row.dataset.sfid;
    var f = SF.find(function(x) { return x.id === fid; });
    if (!f || f.type !== 'enum') return;
    var nv = (sfGet(f) === f.onVal) ? f.offVal : f.onVal;
    sfSet(f, nv);
    el.textContent = nv.toUpperCase();
    sfSyncVisibility();
  });
});

// ── Boat-name modal ───────────────────────────────────────────────────────
function _bnmShow() {
  document.getElementById('bnm-overlay').classList.add('visible');
  setTimeout(function() { document.getElementById('bnm-input').focus(); }, 80);
}
function _bnmHide() {
  document.getElementById('bnm-overlay').classList.remove('visible');
}
function _bnmSave() {
  var name = document.getElementById('bnm-input').value.trim();
  var err  = document.getElementById('bnm-err');
  if (!name) { err.textContent = 'Boat name is required.'; return; }
  err.textContent = '';
  if (!gSettings.vessel) gSettings.vessel = {};
  gSettings.vessel.name = name;
  fetch('/settings', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(gSettings)
  })
  .then(function(r) { return r.json(); })
  .then(function(d) {
    if (d.ok) {
      _bnmHide();
      // Reflect the saved name in the settings panel input if it is open
      var el = document.getElementById('sf-name');
      if (el) el.value = name;
    } else {
      err.textContent = 'Save failed — try again.';
    }
  })
  .catch(function() { err.textContent = 'Save failed — try again.'; });
}
document.getElementById('bnm-save').addEventListener('click', _bnmSave);
document.getElementById('bnm-input').addEventListener('keydown', function(e) {
  if (e.key === 'Enter') _bnmSave();
});

// Fetch settings on page load to check vessel name; show modal if not set.
fetch('/settings')
  .then(function(r) { return r.json(); })
  .then(function(s) {
    delete s['_source'];
    gSettings = s;
    applyRemoteHelmSetting();
    var name = (s.vessel && s.vessel.name) ? s.vessel.name.trim() : '';
    if (!name) _bnmShow();
  })
  .catch(function() {
    // If settings fetch fails at startup, still show the modal as a fallback.
    _bnmShow();
  });

// ── Software update modal ─────────────────────────────────────────────────
function _updShow() { document.getElementById('upd-overlay').classList.add('visible'); }
function _updHide() { document.getElementById('upd-overlay').classList.remove('visible'); }

function _updCheck() {
  var body = document.getElementById('upd-body');
  var inst = document.getElementById('upd-install');
  body.textContent = 'Fetching from GitHub...';
  inst.style.display = 'none';
  document.getElementById('upd-close').textContent = 'CLOSE';
  _updShow();
  fetch('/update/check')
    .then(function(r) { return r.json(); })
    .then(function(d) {
      if (d.error) {
        body.textContent = 'Error: ' + d.error;
      } else if (d.up_to_date) {
        body.textContent = 'Already up to date.\\n\\nBranch : ' + d.branch + '\\nCommit : ' + d.sha;
      } else {
        body.textContent = 'Updates available on branch ' + d.branch + ':\\n\\n'
          + d.commits.join('\\n');
        inst.style.display = '';
      }
    })
    .catch(function(e) { body.textContent = 'Check failed: ' + e; });
}

function _updInstall() {
  var body = document.getElementById('upd-body');
  var inst = document.getElementById('upd-install');
  inst.style.display = 'none';
  body.textContent = 'Starting update...';
  fetch('/update/start', {method: 'POST'})
    .then(function(r) { return r.json(); })
    .then(function(d) {
      if (d.error) {
        body.textContent = 'Failed to start update:\\n' + d.error;
        inst.style.display = '';
      } else {
        _startProgressStream();
      }
    })
    .catch(function(e) { body.textContent = 'Request failed: ' + e; });
}

// Switch the update modal to terminal view and stream deploy output via SSE.
// The EventSource sends Last-Event-ID on reconnect so the server resumes from
// the correct line even after inno-pilot-web-remote is restarted mid-deploy.
function _startProgressStream() {
  var logEl   = document.getElementById('upd-log');
  var body    = document.getElementById('upd-body');
  var actions = document.querySelector('#upd-overlay .upd-actions');
  var box     = document.getElementById('upd-box');

  body.style.display    = 'none';
  actions.style.display = 'none';
  box.classList.add('terminal');
  logEl.textContent  = '';
  logEl.style.display = 'block';

  if (gProgressES) { gProgressES.close(); gProgressES = null; }

  gProgressES = new EventSource('/update/progress');

  gProgressES.onmessage = function(e) {
    var d;
    try { d = JSON.parse(e.data); } catch(ex) { return; }
    if (d.line !== undefined) {
      logEl.textContent += d.line + '\\n';
      logEl.scrollTop = logEl.scrollHeight;
    }
    if (d.done) {
      gProgressES.close();
      gProgressES = null;
      if (d.ok) {
        logEl.textContent += '\\n>>> Deployment complete.  Waiting for service to restart...\\n';
      } else {
        logEl.textContent += '\\n>>> DEPLOYMENT FAILED — check log above.\\n';
        var closeBtn = document.getElementById('upd-close');
        closeBtn.textContent = 'CLOSE';
        actions.style.display = '';
      }
      logEl.scrollTop = logEl.scrollHeight;
      _waitForReconnect();
    }
  };

  // onerror fires when the connection drops (service restarting).
  // The SSE spec auto-retries with Last-Event-ID, so lines resume once
  // the new service instance is up — no manual reconnect needed.
  gProgressES.onerror = function() {
    if (gProgressES && gProgressES.readyState === EventSource.CONNECTING) {
      logEl.textContent += '[service restarting — reconnecting...]\\n';
      logEl.scrollTop = logEl.scrollHeight;
    }
  };
}

// Poll GET / every 3 s until the restarted web-remote answers, then reload.
function _waitForReconnect() {
  setTimeout(function _poll() {
    fetch('/', {cache: 'no-store'})
      .then(function(r) {
        if (r.ok) { window.location.reload(); }
        else { setTimeout(_poll, 3000); }
      })
      .catch(function() { setTimeout(_poll, 3000); });
  }, 3000);
}

document.getElementById('sf-check-update').addEventListener('click', function(e) {
  e.stopPropagation();  // prevent sf-row click from focusing a non-existent input
  _updCheck();
});
document.getElementById('upd-install').addEventListener('click', _updInstall);
document.getElementById('upd-close').addEventListener('click', _updHide);

// ── No-bridge overlay animation ───────────────────────────────────────────
var dotN = 1;
setInterval(function() {
  dotN = (dotN % 3) + 1;
  var d = document.getElementById('dots');
  if (d) d.textContent = '\u25cf'.repeat(dotN);
}, 550);
</script>

</body>
</html>""".replace("$$WHEEL_SVG$$", _WHEEL_SVG)

# ---------------------------------------------------------------------------
# HTTP request handler
# ---------------------------------------------------------------------------

class _Handler(BaseHTTPRequestHandler):

    # HTTP/1.1 is required for Server-Sent Events (persistent connection).
    # Python's BaseHTTPRequestHandler defaults to HTTP/1.0, which browsers
    # reject for EventSource — the SSE stream never delivers and the
    # "NO BRIDGE" overlay stays permanently.
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # suppress default access log noise
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_page()
        elif self.path == "/events":
            self._serve_sse()
        elif self.path == "/health":
            self._serve_health()
        elif self.path == "/settings":
            self._serve_settings()
        elif self.path == "/update/check":
            self._handle_update_check()
        elif self.path == "/update/progress":
            self._handle_update_progress()
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == "/command":
            self._handle_command()
        elif self.path == "/settings":
            self._handle_settings_post()
        elif self.path == "/debug":
            self._handle_debug_toggle()
        elif self.path == "/update/start":
            self._handle_update_start()
        else:
            self.send_error(404)

    def _client_id(self) -> str:
        # Prefer first x-forwarded-for entry when behind a proxy.
        xff = self.headers.get("X-Forwarded-For", "").strip()
        if xff:
            return xff.split(",")[0].strip()
        return self.client_address[0]

    def _send_json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    # ---- GET / ----

    def _serve_page(self) -> None:
        body = _HTML.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    # ---- GET /events (SSE) ----

    def _sse_chunk(self, data: bytes) -> None:
        """Write one HTTP/1.1 chunked-transfer frame and flush.

        HTTP/1.1 streaming responses require either Content-Length or
        Transfer-Encoding: chunked so the browser knows each frame boundary.
        Without chunked encoding Chrome buffers the entire SSE stream and
        onmessage never fires, leaving the 'NO BRIDGE' overlay permanently.
        """
        self.wfile.write(f"{len(data):x}\r\n".encode())
        self.wfile.write(data)
        self.wfile.write(b"\r\n")
        self.wfile.flush()

    def _serve_sse(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type",       "text/event-stream")
        self.send_header("Cache-Control",      "no-cache")
        self.send_header("Connection",         "keep-alive")
        self.send_header("Transfer-Encoding",  "chunked")   # required for HTTP/1.1 streaming
        self.send_header("X-Accel-Buffering",  "no")        # disable nginx buffering
        self.end_headers()

        client_id = self._client_id()
        client_q: queue.Queue = queue.Queue(maxsize=60)
        sub = {"id": client_id, "q": client_q}
        with _sse_lock:
            _sse_subs.append(sub)

        try:
            # Send current state immediately on connect
            snap = _snap()
            self._sse_chunk(f"data: {json.dumps(snap)}\n\n".encode())

            while True:
                try:
                    event = client_q.get(timeout=15)
                    self._sse_chunk(f"data: {json.dumps(event)}\n\n".encode())
                except queue.Empty:
                    # SSE keepalive comment (prevents proxy/browser timeout)
                    self._sse_chunk(b": ka\n\n")

        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with _sse_lock:
                try:
                    _sse_subs.remove(sub)
                except ValueError:
                    pass

    # ---- POST /command ----

    def _handle_command(self) -> None:
        length = int(self.headers.get("Content-Length", 0))
        body   = self.rfile.read(length) if length else b""
        try:
            cmd_str: str = str(json.loads(body)["cmd"])
        except (json.JSONDecodeError, KeyError, ValueError, TypeError):
            self.send_response(400)
            self.end_headers()
            return

        tok = cmd_str.upper().split()

        # ── MODE OFF: intentional TCP disconnect ──────────────────────────────
        # Handled entirely locally — do not forward to bridge.
        if len(tok) >= 2 and tok[0] == "MODE" and tok[1] == "OFF":
            _bridge_active.clear()          # signal bridge thread to drop connection
            # Drain any pending commands so nothing sneaks through after disconnect
            while True:
                try:
                    _cmd_q.get_nowait()
                except queue.Empty:
                    break
            _update(
                connected=False, hdg=None, cmd=None,
                rdr=None, rdr_pct=None, ap=0,
                mode="OFF", comms="OK", warn=None, ui_mode="off",
            )
            self._send_json(200, {"ok": True})
            return

        # ── MODE ON: reconnect and resume live sync ────────────────────────────
        # Re-enables the bridge thread without forwarding any mode command to
        # the bridge itself.
        if len(tok) >= 2 and tok[0] == "MODE" and tok[1] == "ON":
            _bridge_active.set()          # wake bridge thread if idle (was mode OFF)
            _update(ui_mode="on")
            self._send_json(200, {"ok": True})
            return

        # ── Any active command: ensure bridge is (re)connected ────────────────
        # This wakes the bridge thread if it was idle due to mode OFF.
        _bridge_active.set()

        # Optimistic local state update for instant UI feedback before
        # bridge confirms via telemetry (bridge updates within ~200 ms)
        if tok and tok[0] == "ESTOP":
            _update(mode="IDLE", ap=0, ui_mode="on")
        elif len(tok) >= 2 and tok[0] == "MODE":
            if tok[1] == "MANUAL":
                _update(mode="MANUAL", ui_mode="remote")
            elif tok[1] == "AUTO":
                _update(mode="IDLE", ui_mode="auto")
        elif len(tok) >= 2 and tok[0] == "BTN" and tok[1] == "TOGGLE":
            with _state_lock:
                cur_ap   = _state["ap"]
                cur_mode = _state["mode"]
            if cur_mode != "MANUAL":
                if cur_ap:
                    _update(ap=0, mode="IDLE")
                else:
                    _update(ap=1, mode="AP")

        # ── AP_ENGAGE_AT_HDG: lock CMD to current heading then engage AP ──────
        # Reads hdg and cmd atomically, computes the shortest-path delta, and
        # queues BTN <delta> + BTN TOGGLE back-to-back so the bridge sees them
        # as a consecutive pair with no commands interleaved.
        # Note: a bridge-side AP_ENGAGE_AT_HDG command that does this in a
        # single operation would be strictly more atomic; this is the best we
        # can do from the web-remote side without changing the bridge.
        if tok and tok[0] == "AP_ENGAGE_AT_HDG":
            with _state_lock:
                hdg = _state["hdg"]
                cmd = _state["cmd"]
            if hdg is None:
                self._send_json(200, {"ok": False, "error": "No heading signal—compass not ready"})
                return
            if cmd is None:
                self._send_json(200, {"ok": False, "error": "AP heading unknown—try again"})
                return
            # Shortest-path delta so the heading command wraps correctly at 0/360.
            delta = ((hdg - cmd) + 180.0) % 360.0 - 180.0
            try:
                _cmd_q.put_nowait(f"BTN {delta:.1f}")
                _cmd_q.put_nowait("BTN TOGGLE")
            except queue.Full:
                log.warning("AP_ENGAGE_AT_HDG: cmd queue full")
                self._send_json(200, {"ok": False, "error": "Command queue full—try again"})
                return
            # Optimistic local state update (bridge confirms via telemetry ~200 ms later)
            _update(ap=1, mode="AP")
            log.info("AP_ENGAGE_AT_HDG: hdg=%.1f cmd=%.1f delta=%.1f -> engage", hdg, cmd, delta)
            self._send_json(200, {"ok": True})
            return

        # Forward command to bridge via _cmd_q (bridge thread reads this)
        try:
            _cmd_q.put_nowait(cmd_str)
        except queue.Full:
            log.warning("Command queue full, dropping: %s", cmd_str)

        self._send_json(200, {"ok": True})

    # ---- GET /health ----

    def _serve_health(self) -> None:
        snap = _snap()
        body = json.dumps({
            "status":           "ok",
            "bridge_connected": snap["connected"],
            "mode":             snap["mode"],
        }).encode()
        self.send_response(200)
        self.send_header("Content-Type",   "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    # ---- GET /settings ----

    def _serve_settings(self) -> None:
        """Fetch current settings from the bridge via TCP.

        Sends SETTINGS GET through the command queue and waits for the bridge
        to reply with SETTINGS <json>.  Falls back to the local settings file
        when the bridge is not connected or the response times out.

        Works even when mode is OFF: temporarily wakes the bridge thread,
        waits up to 2 s for the connection to establish, then restores the
        OFF state after the operation completes.
        """
        # Drain any stale responses from a previous request
        while True:
            try:
                _settings_resp_q.get_nowait()
            except queue.Empty:
                break

        # Temporarily wake bridge thread if mode is OFF so settings can be
        # fetched via the live bridge regardless of autopilot mode state.
        was_active = _bridge_active.is_set()
        if not was_active:
            _bridge_active.set()
            # Wait up to 2 s for the bridge thread to (re)connect
            deadline = time.monotonic() + 2.0
            while not _snap()["connected"] and time.monotonic() < deadline:
                time.sleep(0.05)

        data: Optional[dict] = None
        try:
            if _snap()["connected"]:
                try:
                    _cmd_q.put_nowait("SETTINGS GET")
                except queue.Full:
                    log.warning("Settings GET: cmd_q full, falling back to file")
                else:
                    try:
                        resp = _settings_resp_q.get(timeout=SETTINGS_TIMEOUT_S)
                        # If it's a dict without an _ack key it's the GET response
                        if isinstance(resp, dict) and "_ack" not in resp:
                            data = resp
                            log.debug("Settings fetched from bridge (%d keys)", len(resp))
                        else:
                            log.warning("Settings GET: unexpected response %s", resp)
                    except queue.Empty:
                        log.warning("Settings GET: bridge timed out, using local file")
        finally:
            # Restore OFF state if the bridge thread was idle before this request
            if not was_active:
                _bridge_active.clear()

        source = "bridge" if data is not None else "local"
        if data is None:
            data = _load_settings()

        # Shallow copy so we can inject _source without mutating the live dict.
        # JS strips _source before storing in gSettings so it is never re-POSTed.
        resp_data = dict(data)
        resp_data["_source"] = source
        body = json.dumps(resp_data).encode()
        self.send_response(200)
        self.send_header("Content-Type",   "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control",  "no-cache")
        self.end_headers()
        self.wfile.write(body)

    # ---- POST /debug ----

    def _handle_debug_toggle(self) -> None:
        """Send SIGUSR1 to the bridge process to toggle DEBUG/INFO log level.

        Reads the bridge MainPID from systemd so we never have to hard-code it.
        Updates _state["debug"] and logs a timestamped marker to the bridge
        journal so the toggle is visible in the log stream.
        """
        import os
        import signal as _signal
        try:
            pid_str = subprocess.check_output(
                ["systemctl", "show", "inno-pilot-bridge",
                 "--property=MainPID", "--value"],
                text=True, timeout=3
            ).strip()
            pid = int(pid_str)
            if pid <= 0:
                self._send_json(500, {"ok": False, "error": "Bridge not running"})
                return
            os.kill(pid, _signal.SIGUSR1)
            new_state = not _snap()["debug"]
            _update(debug=new_state)
            log.info("=== Debug mode %s via web remote ===", "ON" if new_state else "OFF")
            _send_telegram(f"Inno-Pilot: Debug mode {'ON' if new_state else 'OFF'}")
            self._send_json(200, {"ok": True, "debug": new_state})
        except Exception as exc:
            self._send_json(500, {"ok": False, "error": str(exc)})

    # ---- GET /update/check ----

    def _handle_update_check(self) -> None:
        """Fetch from GitHub and report whether the local branch is behind.

        Runs git fetch (blocking, ~2-5 s on a good connection) then compares
        HEAD vs FETCH_HEAD.  Returns JSON:
          {"up_to_date": true,  "branch": "...", "sha": "abcd123"}
          {"up_to_date": false, "branch": "...", "commits": ["hash msg", ...],
           "current_sha": "abcd123"}
        """
        try:
            branch = subprocess.check_output(
                ["git", "-C", REPO_DIR, "rev-parse", "--abbrev-ref", "HEAD"],
                text=True, timeout=5, stderr=subprocess.DEVNULL,
            ).strip()
            subprocess.check_output(
                ["git", "-C", REPO_DIR, "fetch", "origin", branch],
                text=True, timeout=30, stderr=subprocess.STDOUT,
            )
            local_sha = subprocess.check_output(
                ["git", "-C", REPO_DIR, "rev-parse", "HEAD"],
                text=True, timeout=5, stderr=subprocess.DEVNULL,
            ).strip()
            remote_sha = subprocess.check_output(
                ["git", "-C", REPO_DIR, "rev-parse", "FETCH_HEAD"],
                text=True, timeout=5, stderr=subprocess.DEVNULL,
            ).strip()
            if local_sha == remote_sha:
                self._send_json(200, {
                    "up_to_date": True, "branch": branch, "sha": local_sha[:7],
                })
                return
            commits_out = subprocess.check_output(
                ["git", "-C", REPO_DIR, "log",
                 "HEAD..FETCH_HEAD", "--oneline", "--no-decorate"],
                text=True, timeout=5, stderr=subprocess.DEVNULL,
            ).strip()
            commits = [l for l in commits_out.splitlines() if l.strip()]
            self._send_json(200, {
                "up_to_date": False,
                "branch": branch,
                "commits": commits,
                "current_sha": local_sha[:7],
            })
        except subprocess.TimeoutExpired:
            self._send_json(504, {"error": "git fetch timed out — check network"})
        except subprocess.CalledProcessError as exc:
            self._send_json(500, {"error": f"git error: {(exc.output or '').strip()}"})
        except Exception as exc:
            self._send_json(500, {"error": str(exc)})

    # ---- POST /update/start ----

    def _handle_update_start(self) -> None:
        """Launch inno_deploy.sh as a systemd transient service.

        Uses systemd-run --no-block so the deploy process lives in its own
        cgroup and survives inno-pilot-web-remote being stopped mid-deploy.
        Sends a Telegram notification before handing off and returns immediately.
        """
        try:
            branch = subprocess.check_output(
                ["git", "-C", REPO_DIR, "rev-parse", "--abbrev-ref", "HEAD"],
                text=True, timeout=5, stderr=subprocess.DEVNULL,
            ).strip()
        except Exception:
            branch = "master"

        log.info("OTA update triggered via web remote — branch=%s version=%s",
                 branch, INNOPILOT_VERSION)
        _send_telegram(f"OTA update started — {INNOPILOT_VERSION}")

        # Truncate the log so /update/progress streams a fresh run each time.
        try:
            os.makedirs(os.path.dirname(OTA_LOG_FILE), exist_ok=True)
            with open(OTA_LOG_FILE, "w"):
                pass
        except OSError:
            pass

        try:
            # systemd-run creates a transient service in its own cgroup.
            # The web-remote is stopped by the deploy script; this ensures
            # the deploy process is not in the same cgroup and is not killed.
            # Timestamp suffix avoids unit-name collision if a previous run
            # failed and its transient unit is still loaded in a failed state.
            unit_name = f"inno-deploy-ota-{int(time.time())}"
            subprocess.Popen(
                ["sudo", "systemd-run",
                 "--no-block",
                 f"--unit={unit_name}",
                 "--description=Inno-Pilot OTA update",
                 "--setenv=HOME=/home/innopilot",
                 "--setenv=USER=innopilot",
                 "bash", DEPLOY_SCRIPT, branch],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL,
            )
            self._send_json(200, {"ok": True, "branch": branch})
        except Exception as exc:
            log.error("Failed to launch OTA deploy: %s", exc)
            _send_telegram(f"OTA update FAILED to start: {exc}")
            self._send_json(500, {"error": str(exc)})

    # ---- GET /update/progress (SSE) ----

    def _handle_update_progress(self) -> None:
        """Stream OTA deploy output line-by-line as Server-Sent Events.

        Reads OTA_LOG_FILE and emits each line as a JSON SSE event.  The event
        ID is the zero-based line number so browsers can resume mid-stream on
        reconnect (e.g. after the web-remote itself is restarted by the deploy
        script) by sending Last-Event-ID in the reconnect request.

        Emits a terminal {"done": true, "ok": bool} event when the deploy
        script writes its exit marker and then closes the connection.
        """
        try:
            last_id = int(self.headers.get("Last-Event-Id", "-1") or "-1")
            start_line = max(0, last_id + 1)
        except (ValueError, TypeError):
            start_line = 0

        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Transfer-Encoding", "chunked")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        # Send a fast retry so the browser reconnects quickly after a restart.
        try:
            self._sse_chunk(b"retry: 2000\n\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

        sent_up_to = start_line - 1
        while True:
            try:
                with open(OTA_LOG_FILE, "r", errors="replace") as f:
                    lines = f.read().splitlines()
            except (FileNotFoundError, OSError):
                lines = []

            for i in range(max(0, sent_up_to + 1), len(lines)):
                line = lines[i]
                payload = json.dumps({"line": line})
                chunk = f"id: {i}\ndata: {payload}\n\n".encode("utf-8")
                try:
                    self._sse_chunk(chunk)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
                sent_up_to = i

                done_marker: Optional[bool] = None
                if "=== COMPLETE ===" in line:
                    done_marker = True
                elif "=== FAILED" in line:
                    done_marker = False

                if done_marker is not None:
                    done_payload = json.dumps({"done": True, "ok": done_marker})
                    done_chunk = f"data: {done_payload}\n\n".encode("utf-8")
                    try:
                        self._sse_chunk(done_chunk)
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        pass
                    return

            time.sleep(0.4)

    # ---- POST /settings ----

    def _handle_settings_post(self) -> None:
        """Send new settings to the bridge via TCP; bridge saves and applies them.

        Falls back to writing the local settings file when not connected.
        """
        length = int(self.headers.get("Content-Length", 0))
        raw    = self.rfile.read(length) if length else b""
        try:
            settings = json.loads(raw)
            if not isinstance(settings, dict):
                raise ValueError("expected JSON object")
        except Exception as exc:
            log.warning("Settings POST: bad request — %s", exc)
            self.send_response(400)
            self.end_headers()
            return

        saved_via_bridge = False

        # Temporarily wake bridge thread if mode is OFF so settings can be
        # saved via the live bridge regardless of autopilot mode state.
        was_active = _bridge_active.is_set()
        if not was_active:
            _bridge_active.set()
            # Wait up to 2 s for the bridge thread to (re)connect
            deadline = time.monotonic() + 2.0
            while not _snap()["connected"] and time.monotonic() < deadline:
                time.sleep(0.05)

        try:
            if _snap()["connected"]:
                # Drain stale responses
                while True:
                    try:
                        _settings_resp_q.get_nowait()
                    except queue.Empty:
                        break

                # Compact JSON — must be a single line (no embedded newlines)
                json_line = json.dumps(settings, separators=(",", ":"), ensure_ascii=True)
                try:
                    _cmd_q.put_nowait(f"SETTINGS SET {json_line}")
                except queue.Full:
                    log.warning("Settings POST: cmd_q full, falling back to file")
                else:
                    try:
                        resp = _settings_resp_q.get(timeout=SETTINGS_TIMEOUT_S)
                        if isinstance(resp, dict) and resp.get("_ack") == "OK":
                            saved_via_bridge = True
                            log.debug("Settings saved via bridge")
                        else:
                            log.warning("Settings POST: bridge replied %s", resp)
                    except queue.Empty:
                        log.warning("Settings POST: bridge timed out, falling back to file")
        finally:
            # Restore OFF state if the bridge thread was idle before this request
            if not was_active:
                _bridge_active.clear()

        # Always write local file — keeps it in sync with bridge-saved state so
        # the page-load settings fetch returns a current copy before reconnect.
        _persist_settings(settings)

        via = "bridge" if saved_via_bridge else "local"
        body = json.dumps({"ok": True, "via": via}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))  # required for HTTP/1.1 keep-alive
        self.end_headers()
        self.wfile.write(body)


# ---------------------------------------------------------------------------
# Journal monitor — tracks inno-health-notify for packet-loss state changes
# ---------------------------------------------------------------------------

def _net_warn_monitor() -> None:
    """Daemon thread: tails the inno-health-notify journal and updates net_warn.

    Uses clean-slate semantics (no history replay) so web-remote restart always
    starts from net_warn=False.  The health-notify service itself is the source
    of truth; its state is logged and persists across web-remote restarts.
    """
    try:
        proc = subprocess.Popen(
            ["journalctl", "-u", "inno-health-notify",
             "--no-pager", "-o", "cat", "-n", "0", "-f"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        log.info("Net-warn monitor: tailing inno-health-notify journal")
        for line in proc.stdout:
            line = line.strip()
            if not line:
                continue
            if "PACKET_LOSS_WARN" in line:
                _update(net_warn=True)
                log.warning("NET_WARN: Packet loss warning received from health monitor")
            elif "PACKET_LOSS_CLEAR" in line:
                _update(net_warn=False)
                log.info("NET_WARN: Packet loss cleared — network OK")
    except Exception as exc:
        log.warning("Net-warn monitor thread error: %s", exc)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    # Start bridge client thread
    t = threading.Thread(target=bridge_client, daemon=True, name="bridge-client")
    t.start()
    log.info("Bridge client thread started (target: %s:%d)", BRIDGE_HOST, BRIDGE_PORT)

    # Start journal monitor thread for packet-loss network warnings
    m = threading.Thread(target=_net_warn_monitor, daemon=True, name="net-warn-monitor")
    m.start()
    log.info("Net-warn monitor thread started")

    # Start HTTP server
    server = ThreadingHTTPServer(("", WEB_PORT), _Handler)
    log.info("Web remote listening on http://0.0.0.0:%d/", WEB_PORT)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("Shutting down")


if __name__ == "__main__":
    main()
