# Changelog

All notable changes to the Inno-Pilot system are documented here.
Format follows [Keep a Changelog](https://keepachangelog.com/).
Version applies to all three components (Bridge, Nano, Remote) simultaneously and the version of all components must always match regardless if a change was made in a component or not.

## [Unreleased]

### Fixed
- **pypilot web UI / install** (`install.sh`, `inno_deploy.sh`,
  `compute_module/pypilot/scripts/debian/etc/systemd/system/pypilot_web.service`):
  the pypilot web UI on port **8000** — which hosts the compass/heel
  **calibration** interface — was never installed. `install.sh` deliberately
  skipped `pypilot_web.service` ("not used by Inno-Pilot"), and `inno_deploy.sh`
  only *started* the unit if it already existed, so port 8000 was down on every Pi.
  - `install.sh` now installs and enables `pypilot_web.service` alongside
    `pypilot.service`.
  - `inno_deploy.sh` Step 4b now installs/enables the unit on redeploy, so Pis
    provisioned before this change pick it up without a full reinstall.
  - `inno_deploy.sh` Step 4b now does a **two-pass** `setup.py install`.
    dependencies.py drops a `pyproject.toml` that (on setuptools 78 / Python
    3.13) makes `build_py` skip the pure-Python pypilot source, so a single pass
    installs only the C extensions and never updates `web.py` et al. — meaning
    edits to pypilot Python source silently never reach the Pi on a redeploy.
    The second pass (pyproject.toml removed + `deps` marker touched, bypassing
    dependencies.py) actually installs the Python modules. This mirrors
    install.sh's workaround but runs every redeploy (install.sh only ran it when
    pypilot was entirely missing).
  - The unit runs as `User=innopilot` (upstream ships it with no `User=` line, so
    it would run as root with the wrong `~/.pypilot`) and uses an absolute
    `ExecStart` path.

### Notes
- This change touches the **pypilot** subsystem and the installers only; it does
  **not** modify any of the version-synced inno-remote components (Bridge, Nano,
  Remote), so no `INNOPILOT_VERSION` bump is required.

## [v1.3.3_B7] — 2026-06-26 — ADC mux-settle fix (spurious PiV HIGH) + 5V calibration

### Fixed
- **Nano firmware** (`motor_simple.ino`, `read_adc_avg()`): added ADC sample/hold
  settling (3 discarded reads after each channel switch). Without it the
  high-impedance dividers (A3 Pi/5V, A0 Vin) suffered ADC-mux **crosstalk**: A3 (5V
  rail) intermittently read ~14 V (≈ Vin's pin voltage), tripping a **spurious
  `PiV HIGH` alarm** (buzzer + OLED) once FEATURE_PI_VOLTAGE was enabled. Confirmed
  via the new 5V telemetry — ~15 % of samples read 14.2 V, the rest a correct ~5.18 V
  (14 V on a 5 V rail is impossible — it would destroy the Pi/Nano).
- **Nano firmware**: `PI_VSENSE_SCALE` calibrated 5.25 → **5.17** (multimeter 5.1 V
  vs reported 5.18 V). Per-board value.

### Changed
- **All version-synced components** (Nano, Bridge, Web remote): v1.3.3_B6 → v1.3.3_B7.

### Hardware impact
- Adds ~1.2 ms per `read_adc_avg()` call (3 settle reads); negligible at 5 Hz, no
  change to motor drive. Makes A3/A0/A1 readings trustworthy — prerequisite for a
  meaningful brown-out load test.

## [v1.3.3_B6] — 2026-06-26 — 5V-rail telemetry + Vin calibration (brown-out diagnosis)

### Added
- **Nano firmware + Bridge**: 5V logic-rail telemetry. The Nano samples the shared
  Pi/Nano 5V bus (A3) each telemetry cycle and sends it as `PI_VOLTAGE_CODE` (0xB4,
  volts×100); the bridge logs it at DEBUG (`Nano 5V rail: X.XX V`). Lets us watch the
  logic rail under motor load to diagnose a suspected brown-out/reset of the Nano
  during steering. Forwarded to pypilot like other inno-pilot codes (pypilot ignores
  codes it doesn't recognise, so no guard needed).

### Fixed
- **Nano firmware** (`motor_simple.ino`): `VOLTAGE_SCALE` corrected for Malu (.13)
  from 3.323 to **4.855** (bench-calibrated: multimeter 12.985 V vs reported 8.887 V).
  The old value under-read Vin by ~35% (a true 13 V showed as ~8.9 V — below the
  10.9 V under-voltage threshold; harmless only because battery-voltage faults are
  disabled on Malu). Per-board value; recalibrate per unit.

### Changed
- **All version-synced components** (Nano, Bridge, Web remote): v1.3.3_B5 → v1.3.3_B6.

### Hardware impact
- Adds one ADC read (A3) per telemetry cycle (~5 Hz); negligible loop-time impact, no
  change to motor drive. Corrected `VOLTAGE_SCALE` makes the reported Vin accurate —
  relevant if battery-voltage faults are later enabled.

## [v1.3.3_B5] — 2026-06-23 — Fix: remote-manual (REMOTE) rudder runaway — axis sign

### Fixed
- **Nano firmware** (`servo_motor_control/arduino/motor_simple/motor_simple.ino`):
  remote-manual ("REMOTE" helm) steering ran the rudder hard-over to the
  commanded-side end-stop instead of servoing to the commanded angle. The B49
  angle-based loop assumed `pilot_rudder_deg10` was `+port / -stbd`, but the bridge
  transmits the Nano `-rudder.angle` (`PILOT_RUDDER_CODE`) plus matching limits, so
  the axis is actually `+stbd / -port`. That single inverted assumption made the
  position loop **positive-feedback** (error grew as it drove) and pointed the
  end-limit guard at the wrong end, so nothing halted travel. Confirmed by
  instrumented bench test on Malu (192.168.6.13): a small port command drove the
  rudder to the port stop (~+91°, well past the ±42° calibrated limit).
  - Rebuilt the remote-manual block on the correct `+stbd / -port` axis — target
    mapping, position error, drive direction, and end-limit guard are now consistent,
    so the loop converges and holds at the commanded angle.
  - Added a **convention-independent absolute end backstop** (`over_stbd_end` /
    `over_port_end`): the motor is never driven past a calibrated end regardless of
    the directional logic — defence-in-depth against any future sign error.
  - Motor direction mapping unchanged (bench-confirmed correct: port cmd → port
    motion); only the position-loop sign and limit-end mapping were wrong.
- **Deploy/install** (`inno_deploy.sh`, `install.sh`): the two-pass `setup.py install`
  marker-file shuffle now uses `sudo` (`sudo rm -f pyproject.toml; sudo touch deps;
  sudo rm -f deps`). Pass 1 runs under sudo and creates those files as root, so the
  earlier plain `touch deps` failed with "Permission denied" and aborted the deploy
  mid-way (after services were already stopped).

### Changed
- **All version-synced components** (Nano, Bridge, Web remote): v1.3.3_B4 →
  v1.3.3_B5 (`INNOPILOT_VERSION` / `INNOPILOT_BUILD_NUM`). The ESP32 hardware-remote
  OTA binary is not rebuilt here — it is not used for the bench validation (the web
  remote on port 8888 drives the test).

### Hardware impact
- Changes which H-bridge direction is commanded for a given rudder error in
  remote-manual mode, and adds an absolute end-stop cutout. Requires a Nano reflash
  and bench re-test before sea use: verify port/stbd convergence and that travel
  halts at the calibrated limits.

## [v1.3.3_B3] — 2026-05-08 — Fix: deploy script aborts on root-owned nano_sketch.sha256

### Fixed
- **Deploy** (`inno_deploy.sh`): After a successful Nano flash the script
  records the firmware hash via `echo > "$NANO_HASH_FILE"` (no sudo).
  If a previous deploy was run with `sudo`, the hash file ends up
  `root:root`, and the next unprivileged deploy fails with
  `Permission denied` *after* the Nano has already been reflashed —
  leaving the post-flash steps (boot wait, bridge restart, version
  verification) unrun and the system in a half-deployed state.
  The script now detects an unwritable hash file and repairs ownership
  via `sudo chown` to the current user before writing.

### Changed
- **All** (Bridge, Web, Nano, Remote): version bump v1.3.3_B2 → v1.3.3_B3.

## [v1.3.3_B2] — 2026-05-08 — Fix: OTA URL `127.0.0.1` due to bridge/network startup race

### Fixed
- **Bridge** (`inno_pilot_bridge.py`): `_local_ip()` was called at module
  import time and its result frozen in `OTA_SERVER_HOST`. On a Pi Zero on
  Wi-Fi the bridge service starts ~80 s before `network-online.target`, so
  the UDP probe to `8.8.8.8` raised `ENETUNREACH`, the except branch fell
  back to `127.0.0.1`, and the bridge advertised an unreachable OTA URL
  (`http://127.0.0.1:8556/inno_remote.bin`) to every remote that connected
  with a version mismatch — even after the network came up.
  - Replaced with a lazy `ota_host()` helper that resolves the LAN IP at
    OTA-offer time and caches the first non-loopback result. `_local_ip()`
    now returns `None` (instead of `127.0.0.1`) when no route is available
    so the cache is not poisoned.
  - If the network is still unreachable when an OTA is offered, the bridge
    logs a WARNING and re-tries on the next remote HELLO.

### Changed
- **All** (Bridge, Web, Nano, Remote): version bump v1.3.3_B1 → v1.3.3_B2.

## [v1.2.0_B36] — 2026-04-14 — OLED SH1106 toggle + EEPROM settings storage

### Added
- **Web** (`inno_web_remote.py`): "OLED SH1106 Mode" toggle in CONNECTIONS & FEATURES
  settings section with hover tooltip explaining the pixel-shift symptom.
- **Bridge** (`inno_pilot_bridge.py`): All settings now serialized to Nano EEPROM on
  save and on bridge startup. Nano rebooted via DTR pulse after EEPROM write so
  settings take effect immediately.
- **Nano** (`motor_simple.ino`): EEPROM read on boot — loads feature flags (including
  OLED type) before display init. OLED init selects SH1106_128x64 or Adafruit128x64
  profile based on the stored flag.
- **Nano** (`motor_simple.ino`): EEPROM_WRITE_CODE (0x53) handler for receiving
  settings bytes from bridge.

### Changed
- **All** (Nano, Bridge, Web): Version bump v1.2.0_B34 → v1.2.0_B36.
  B35 fixes (OTA IP auto-detect, Nano cold-boot DTR pulse, Flask Markup) are
  included but were committed previously without a version string bump.

## [v1.2.0_B35] — 2026-04-13 — Fix: Pi5/Bookworm reinstall issues (OTA IP, Nano cold-boot UART, Flask Markup)

### Fixed
- **Bridge** (`inno_pilot_bridge.py`): Removed hardcoded `OTA_SERVER_HOST = "192.168.6.13"`.
  Replaced with a `_local_ip()` function that probes the routing table via a UDP
  connect (no data sent) to determine the Pi's own LAN IP at startup. This means the
  bridge works correctly on any Pi regardless of its fixed IP address, and OTA update
  push works without manual edits after install.
- **Bridge** (`inno_pilot_bridge.py`): Added explicit DTR pulse on bridge startup to
  ensure the Nano's UART initialises correctly on cold boot. On cold boot the CH340
  USB-UART DTR line starts LOW; `open_serial_no_reset()` clears HUPCL but causes no
  DTR transition, so the Nano's serial TX never drove the line. The bridge sent frames
  continuously but received nothing until a DTR reset occurred. The fix pulses DTR
  HIGH→LOW (150 ms) immediately after opening the port; HUPCL is already cleared so
  this is the only reset the Nano will see during normal bridge operation. The
  subsequent `time.sleep(2.0)` gives the Nano comfortable margin to finish `setup()`
  before the bridge starts sending RCT settings.
- **Web** (`compute_module/pypilot/web/web.py`): Fixed `ImportError` crash-loop on
  Flask >= 2.3. `flask.Markup` was removed in Flask 2.3 and moved to `markupsafe`.
  The import is now guarded with a try/except so pypilot_web starts correctly on
  Bookworm (Flask 3.x) and still works on older installs.

## [v1.2.0_B34] — 2026-04-13 — Fix: settings SAVE permanently stuck on "Saving…" (missing Content-Length)

### Fixed
- **Web Remote**: Settings POST response now includes a `Content-Length` header.
  The handler class uses `protocol_version = "HTTP/1.1"` so the browser keeps the
  connection alive and requires either `Content-Length` or `Transfer-Encoding:
  chunked` to know when the response body ends.  Without it `r.json()` in the
  browser hung forever — the 200 status and headers were received immediately but
  the body never "arrived".  The GET (`_serve_settings`) and command POST
  (`_handle_command` via `_send_json`) already sent Content-Length correctly; only
  the settings POST was missing it.  Root cause found via Playwright debug tracing
  (browser showed HTTP 200 received; server logs showed response fully sent; body
  never delivered to JS).

## [v1.2.0_B33] — 2026-04-13 — Fix: "Saving…" status stuck indefinitely on slow bridge response

### Fixed
- **Web Remote**: Settings SAVE POST now has a 9-second `AbortController` timeout.
  Previously there was no client-side fetch timeout — if the Python HTTP handler
  blocked longer than expected (bridge reconnect 2 s + SETTINGS_TIMEOUT_S 4 s = 6 s
  worst case, or indefinitely on an unhandled server error), the "Saving…" status
  would remain on-screen forever. The AbortController guarantees `.catch()` fires
  within 9 s with a "✗ Save timed out" message so the panel always closes.

## [v1.2.0_B32] — 2026-04-13 — Fix: Settings panel always shows "bridge unavailable" in OFF mode

### Fixed
- **Web Remote**: Settings GET and SET now work regardless of autopilot mode.
  Previously, the bridge client thread dropped its TCP connection when mode was
  OFF, so `_snap()["connected"]` returned False and both `_serve_settings` and
  `_handle_settings_post` silently fell back to the local file — showing the
  "⚠ Using local settings — bridge unavailable" warning even when the bridge
  was fully operational.  Fixed by temporarily setting `_bridge_active` before
  each settings operation, waiting up to 2 s for the bridge thread to
  (re)connect, then restoring the OFF state in a `finally` block.

## [v1.2.0_B31] — 2026-04-13 — Feat: Settings panel status/feedback line

### Added
- **Web Remote**: Status/feedback line inside the settings panel header (`#sov-status`).
  Shows load and save outcomes so the user always knows whether settings came from the
  bridge or the local fallback file, and whether a save succeeded.
  - **Opening settings**: "Loading…" (blue-grey) while the `GET /settings` request is in
    flight; then "✓ Loaded from bridge" (green) or "⚠ Using local settings — bridge
    unavailable" (amber) depending on which path the server used.
  - **Saving settings**: "Saving…" (blue-grey) while the `POST /settings` request is in
    flight; panel stays open until the response arrives, then shows "✓ Saved via bridge"
    (green), "⚠ Saved locally — bridge unavailable" (amber), or "✗ Save failed" (red).
    Panel auto-closes 1.8 s after a successful save (2 s after a network error).
  - **Load failure**: "✗ Failed to load settings" (red) on network error.
- **Web Remote**: `setSovStatus(msg, type)` JS helper manages the status line.
- **Web Remote**: `_doClosePanel()` JS helper extracted from `closeSettings` to support
  the deferred-close save flow.

### Changed
- **Web Remote** (`_serve_settings`): Response JSON now includes a `_source` key
  (`"bridge"` or `"local"`) so JS can show the correct status. JS strips this key before
  storing settings in `gSettings` so it is never re-POSTed to the bridge.
- **Web Remote** (`_handle_settings_post`): Response JSON changed from `{"ok":true}` to
  `{"ok":true,"via":"bridge"|"local"}` so JS can distinguish the two save paths.
- **Nano**: Version constant bumped to B31 (no logic changes).

## [v1.2.0_B29] — 2026-04-05 — Feat: 4-mode animated radio button selector (OFF / ON / REMOTE / AUTO)

### Changed
- **Web Remote**: Replaced 3-position vertical lever toggle (OFF / AUTO / MANUAL) with a
  4-position animated radio button group (AUTO / REMOTE / ON / OFF, top-to-bottom).
  Each option has an animated round radio dot with a label beside it.
- **Web Remote**: Mode semantics redefined:
  - **OFF** — OLED content area blanked, TCP connection to bridge dropped (port freed for ESP32 handheld)
  - **ON** — observer mode; connects to bridge, shows all telemetry, helm wheel follows rudder position sensor (dark/inactive appearance), no control
  - **REMOTE** — skipper steers; helm wheel lights up, wheel reflects commanded rudder position, boat driven by web remote (was "MANUAL")
  - **AUTO** — Pypilot heading hold; helm wheel dark but follows actual rudder movements, all metrics displayed
- **Web Remote**: `MODE ON` command handled server-side (wakes bridge thread without forwarding a control command)
- **Web Remote**: Wheel label updated to "drag wheel in REMOTE mode"

## [v1.2.0_B28] — 2026-04-05 — Fix: On-Board Buttons setting not rendering or responding in web UI

### Fixed
- **Web Remote**: `on_board_buttons` was missing from the `SF` field descriptor array —
  the UI rendered the row but had no JS registration for it, so the ON/OFF buttons showed
  no active state on open and click events were silently ignored. Added
  `{id:'on_board_buttons', sec:'features', type:'bool'}` to `SF` alongside the other
  feature flags.

## [v1.2.0_B27] — 2026-04-05 — Fix: motor activates in HAND mode on units without buttons

### Fixed
- **Nano**: Motor falsely activated in HAND mode when helm moved by hand on units with no
  physical buttons wired (e.g. .12 / Pi5 boat). Root cause: floating A6 pin combined with
  ADC S/H crosstalk from the preceding A2 rudder read caused `decode_button_from_adc()` to
  return BTN_B1, setting `manual_override=true` and driving the motor STBD (confirmed via
  MOTOR_REASON_CODE = `manual_phys(btn)` with `manual_ov=1`).

### Added
- **Nano**: `FEATURE_ON_BOARD_BUTTONS` (0x20) — when this feature flag is OFF, the Nano
  skips `analogRead(A6)` entirely and forces `raw_b = BTN_NONE`, eliminating the floating
  pin false-positive. Default OFF so units without buttons are safe out of the box.
- **Bridge**: `FEATURE_ON_BOARD_BUTTONS` constant and `on_board_buttons` settings key;
  included in `send_features_to_nano()` and logged at startup.
- **Web Remote**: "On-Board Buttons" ON/OFF toggle in SETTINGS → CONNECTIONS & FEATURES.
  Set ON for the .13 unit (has buttons), leave OFF for the .12 unit (no buttons).

## [v1.2.0_B20] — 2026-04-02 — Version sync across all components

### Changed
- **All**: Synchronised version to `v1.2.0_B20` across Bridge, Nano (motor_simple.ino), and ESP32 Remote — Bridge was at B15, Nano and Remote were at B7

## [v1.2.0_B1] — 2026-03-27 — OTA Update Pipeline + Version Reset

### Added
- **Remote**: OTA firmware update via TCP — on TCP connect, HELLO version mismatch triggers automatic OTA download from bridge HTTP server (`http://192.168.6.13:8556/inno_remote.bin`), flashes inactive OTA partition, and reboots; OLED shows "OTA UPDATE / Updating..." during transfer, "OTA FAILED / Check bridge" on error
- **Remote**: Task watchdog unregisters during OTA download to prevent false 10 s timeout during transfer
- **Bridge**: OTA HTTP file server on port 8556 — serves `inno_remote.bin` from `/var/lib/inno-pilot/ota/` in a daemon thread; returns 404 if binary not yet deployed (safe at startup)
- **Bridge**: HELLO handler now sends `OTA <url>` after `HELLO <ver>` reply when remote version mismatches and binary is present
- **Deploy**: `deploy_inno_pilot_glue.sh` now copies `ota/inno_remote.bin` from repo to `/var/lib/inno-pilot/ota/` on Pi
- **Repo**: `inno-remote/firmware/inno_remote/ota/` directory tracked in git; `.gitignore` exception allows committing the OTA-ready `.bin`

### Changed
- Version scheme reset from `v0.2.0_B21` (Bridge/Nano) / `v0.2.0_B19` (Remote) to `v1.2.0_B1` across all three components — build counter restarts at 1

## [v0.2.0_B21] — 2026-03-27 — OLED Fault Area Moved to Rows 1-2, Helm Always at Row 3

### Changed
- **Nano**: All fault/warning messages moved from rows 2-3 to rows 1-2 — 2X messages span rows 1+2, 1X messages use row 1
- **Nano**: Helm field (row 3) is now always drawn unconditionally — `row3_taken` guard removed entirely
- **Bridge**: Build bumped to B21 to match Nano

## [v0.2.0_B20] — 2026-03-27 — OLED Layout Compaction: Remove Diagnostics Row, Move Helm/Cmd/Rudder Up

### Changed
- **Nano**: Removed Rx diagnostics row (was row 3: `Rx:N Er:N A:N.N`) — no longer shown in normal or offline state
- **Nano**: Helm mode moved from row 4 to row 3 (guarded: hidden during 2X fault messages that span rows 2-3)
- **Nano**: Cmd/HDG row moved from row 5 to row 4
- **Nano**: Graphical rudder bar moved from row 6 to row 5; track now filled with `-` instead of spaces (`P---I---S`)
- **Nano**: Row 6 always cleared (was rudder bar); row 7 V/A/T unchanged
- **Nano**: Offline path now simply clears rows 3-6 (no diagnostics text in middle rows)
- **Bridge**: Build bumped to B20 to match Nano

## [v0.2.0_B19] — 2026-03-27 — Remote Safety: TCP Indicator, Comms Display, Watchdog, Buzzer, Version Handshake

### Added
- **Remote**: No-bridge connecting screen — when TCP is not established in normal mode, OLED shows "NO BRIDGE / Connecting..." instead of simulated instrument values; simulation is now strictly a demo-mode feature (hold ESTOP at boot)
- **Remote**: COMMS fault display — parses `COMMS WARN` / `COMMS CRIT` from bridge; mode line becomes `MODE: AUTO [W]` on WARN and `!COMMS FAULT!` on CRIT
- **Remote**: Buzzer drive — GPIO 8 (NPN transistor) now driven for three events:
  - ESTOP press: double-beep (100 ms on / 60 ms off / 100 ms on), highest priority
  - TCP bridge lost mid-session: triple short pulse alarm
  - COMMS CRIT active: continuous 500 ms on/off slow beep (matches Nano pattern)
- **Remote**: Task watchdog — `esp_task_wdt_add(NULL)` registered in `app_main`; fed by `esp_task_wdt_reset()` every 20 ms loop; 10 s timeout + panic in `sdkconfig.defaults`
- **Remote**: VERSION handshake — sends `HELLO v0.2.0_B19` on every new TCP connection; logs mismatch warning if bridge replies with a different version
- **Remote**: `tcp_client_set_connect_callback()` API — fires once per new TCP connection, used for HELLO send
- **Bridge**: HELLO command handler — parses remote version, logs match/mismatch at INFO/WARNING, replies `HELLO <bridge_version>`

### Changed
- All components bumped to B19 (Nano: version constant only, no logic changes)

## [v0.2.0_B18] — 2026-03-27 — OLED Simplification & Graphical Rudder Bar

### Changed
- **Nano**: OLED Row 1 removed — Pi online/offline status replaced by version-mismatch-by-exception warning in rows 2-3 (flashing `!VER MISMATCH!` + build numbers when bridge ≠ Nano build)
- **Nano**: Boot splash is now non-blocking — `loop()` runs during the 3 s splash (serial RX uninterrupted); old `delay(3000)` removed
- **Nano**: Row 4 replaces AP/Clutch dual-field with single `Helm:` field: `HAND` (manual jog), `AUTO` (AP engaged), `REMOTE` (TCP remote manual)
- **Nano**: Row 5 heading values always 3-digit zero-padded (e.g. `005`); `HDG:` right-justified on the display
- **Nano**: Row 6 replaced with graphical rudder bar — `P` on left, `S` on right, `I` indicator proportionally placed within [port_lim, stbd_lim]; `I` blinks over `P` or `S` when rudder exceeds a limit, with `?Rud>Limit` warning in rows 2-3
- **Nano**: Rows 2-3 warning priority updated: steer_loss > hw_fault > ver_mismatch > comms_crit > comms_warn > rud_overshoot > ap_pressed
- **Nano**: Removed unused `rudder_deg` local in `oled_draw()` (was never read)
- **Bridge**: Build bumped to B18 to match Nano

## [v0.2.0_B17] — 2026-03-27 — Fix CRC Errors: RX Buffer + OLED Throttle

### Fixed
- **Nano**: Root cause of sustained ~20 CRC errors/10s identified — OLED I2C draw blocking `loop()` long enough to overflow the 64-byte Arduino serial RX buffer during bridge telemetry bursts
- **Nano**: Increased serial RX buffer from 64 to 128 bytes via `--build-property "build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=128"` (required on every compile)
- **Nano**: Reduced OLED refresh from 200ms (5 Hz) to 1000ms (1 Hz) — eliminates buffer pressure overlap with 5 Hz telemetry bursts
- **Nano**: Partial OLED update — rows 0-1 (version/Pi status) skip I2C writes when content unchanged, saving ~25% of draw time

## [v0.2.0_B16] — 2026-03-27 — Comms Error Diagnostic Logging

### Added
- **Nano**: `COMMS_ERR_DETAIL_CODE` (0xED) — sends corrupt frame CODE byte + received CRC to bridge on CRC error, rate-limited to 5/s (200 ms throttle, latest-wins 1-slot)
- **Bridge**: Volatile diagnostic log at `/tmp/inno_pilot_comms_diag.log` — 256 KB rotating, ms-precision timestamps, does not survive reboot
- **Bridge**: `COMMS_ERR_DETAIL_CODE` parser — extracts corrupt code + CRC, logs with err_window/crit_s context
- **Bridge**: Bridge-side CRC errors (Nano→Bridge direction) also logged to the same diagnostic file

## [v0.2.0_B15] — 2026-03-26 — Comms-Fault Detection & Safety System

### Added
- **Nano**: Sliding-window CRC error rate monitor — 10 × 1-second buckets, cached sum, evaluated every loop
- **Nano**: Two-level fault thresholds: WARN (≥5 errors/10 s) and CRITICAL (≥15/10 s held 3 consecutive seconds)
- **Nano**: Autonomous AP disengage (one-shot latch) on CRITICAL — motor neutral, clutch off, 0ms latency
- **Nano**: 1 Hz comms buzzer alarm (500ms on/off) — distinct from 100ms hw-alarm rapid beeps
- **Nano**: OLED comms-fault slots: `!COMMS ERR!` (flashing 2×) at CRITICAL, `?Comms:N err/10s` (static 1×) at WARN
- **Nano**: PTM two-press extended to silence comms buzzer (first press silence, second press STOP)
- **Nano**: `COMMS_DIAG_CODE` (0xEC) telemetry at 1 Hz — lo byte = err_window_sum, hi byte = crit_consec_s
- **Nano**: New flag bits `COMMS_WARN_FAULT=0x1000`, `COMMS_CRIT_FAULT=0x2000` in FLAGS word
- **Bridge**: Parse `COMMS_WARN_FAULT`/`COMMS_CRIT_FAULT` from FLAGS — auto-disengage AP via pypilot set_q on CRITICAL
- **Bridge**: `COMMS_DIAG_CODE` handler — logs elevated error rates at INFO, clean at DEBUG
- **Bridge**: CRC validation on Nano→bridge frames in `extract_wrapped_frames()` — corrupt frames dropped, not forwarded to pypilot
- **Bridge**: `COMMS OK` / `COMMS WARN N` / `COMMS CRIT` TCP telemetry to remote at 5 Hz
- **Bridge**: Relay diagnostics — drop-rate percentage logged every 30 s, warns at >5%

### Changed
- OLED fault priority: steer_loss > hw_fault > **comms_crit** > **comms_warn** > ap_pressed_warn > overlay
- Buzzer priority: steer_loss > hw_alarm > **comms_fault** (500ms/1Hz) > ap_pressed_warn

## [v0.2.0_B14] — 2026-03-26 — AP State Cleanup + Probe Refactor

### Changed
- **Bridge**: Removed `AP_ENABLED_CODE` (0xE1) send — AP engage/disengage is now derived from pypilot's native `COMMAND_CODE`/`DISENGAGE_CODE` relay frames (single source of truth)
- **Bridge**: Removed `ap.enabled` subscription from pypilot worker thread (redundant after above)
- **Bridge**: Relay loop now inspects forwarded pypilot frames to keep `bstate.mode` accurate when pypilot changes AP state externally
- **Bridge**: `probe_nano_port()` now uses `send_nano_frame()` — removes hand-rolled wrap/build duplication
- **Nano**: Removed `AP_ENABLED_CODE` handler; `ap_enabled_remote` set true on first `COMMAND_CODE`, cleared on `DISENGAGE_CODE`

## [v0.2.0_B11] — 2026-03-26 — Serial Reliability + HUPCL Fix

### Added
- **Bridge**: B8 — CRC-validated pypilot relay: wraps outbound pypilot frames in bridge framing + CRC before forwarding to Nano; prevents Nano CRC-error flood from raw pypilot data
- **Bridge**: B9 — Structured DEBUG logging for relay diagnostics (SIGUSR1 toggle)
- **Nano**: B10 — RX debug counters on OLED (CRC errors, raw bytes, valid frames)

### Fixed
- **Bridge**: B11 — HUPCL disabled on serial open via `termios.tcsetattr(~termios.HUPCL)` — prevents Nano reset (DTR-drop) when the port is closed. Documented in `CLAUDE.md` and top-level `CLAUDE.md`.

## [v0.2.0_B7] — 2026-03-25 — Remote Integration (Tasks #2–#10)

### Added
- **Bridge**: Mode state machine (`IDLE / AP / MANUAL`), `handle_remote_disconnect()` with steer-loss safety
- **Bridge**: TCP server on port 8555 handles `MODE MANUAL/AUTO`, `RUD xx.x`, `BTN TOGGLE/±1/±10`, `ESTOP`
- **Bridge**: Parses `FLAGS_CODE` and `BUZZER_STATE_CODE` from Nano, relays to Remote as TCP text
- **Bridge**: Sends `RDR_PCT`, `MODE`, `WARN AP_PRESSED` telemetry to Remote
- **Nano**: `MANUAL_MODE_CODE` (0xE9), `MANUAL_RUD_TARGET_CODE` (0xE8), `WARNING_CODE` (0xEA), `BUZZER_STATE_CODE` (0xEB)
- **Nano**: Two-press STOP logic — first press silences active alarm, second press is full emergency stop
- **Nano**: Remote manual steering via bang-bang motor drive to ADC target with deadband
- **Nano**: Steer-loss OLED display (flashing `!STEER LOSS`), `?AP Pressed?` warning (5s)
- **Nano**: Buzzer priority chain: steer-loss > hw-fault > ap-warn, with state reporting to bridge
- **Nano**: OLED row 4 shows `MAN: ON` when in remote MANUAL mode
- **Remote**: `on_bridge_rx()` TCP callback — parses `AP`, `HDG`, `CMD`, `RDR`, `RDR_PCT`, `FLAGS`, `MODE`, `WARN AP_PRESSED`
- **Remote**: Mode switch sends `MODE MANUAL` / `MODE AUTO` on physical transition
- **Remote**: Buttons send `BTN TOGGLE`, `BTN ±1/±10` in AUTO mode
- **Remote**: STOP button sends `ESTOP` to bridge on every press
- **Remote**: Pot sends `RUD xx.x` (0–100%) in MANUAL mode, throttled to 1% change or 200ms
- **Remote**: Bridge telemetry overrides local simulation when connected
- **Remote**: `?AP Rejected?` warning overlay on OLED (5s auto-expire)

### Changed
- `MANUAL_MODE_CODE` reassigned from 0xE7 to 0xE9 (0xE7 conflicts with pypilot `RESET_CODE`)
- Bridge `process_remote_line()` refactored to modify `BridgeState` directly
- Bridge pypilot worker thread with `time.sleep(0.01)` to prevent CPU starvation on Pi Zero (fix from B6)

### Fixed
- Pi: Offline issue (pypilot thread consuming 92% CPU on single-core Pi Zero, starving main loop)

## [v0.1.0] — 2026-03-20 — Baseline

First tagged baseline. Captures the working state before inno-remote integration.

### Components
- **Bridge** (`compute_module/glue/inno_pilot_bridge.py`) — pypilot ↔ Nano relay via PTY/socat
- **Nano** (`servo_motor_control/arduino/motor_simple/motor_simple.ino`) — motor controller with clutch, OLED, buzzer
- **Remote** (`inno-remote/firmware/inno_remote/`) — ESP32-C3 handheld, demo/simulator mode only

### What works
- pypilot autopilot with heading hold via bridge relay
- Serial CRC-8 framed protocol (6-byte frames, 38400 baud) between bridge and Nano
- Motor H-bridge drive with current sensing and stall protection
- Clutch engage/disengage with buzzer feedback
- Rudder position sensing via potentiometer (ADC)
- Port and starboard limit detection
- OLED status display on Nano (heading, rudder, AP state)
- Remote: Wi-Fi STA with auto-reconnect, OLED UI, button ladder, demo/simulation mode
- Deployment via `deploy_inno_pilot_glue.sh` on Pi

### Known limitations
- Remote is not integrated (demo mode only, no TCP connection)
- No OTA update capability for remote
- No version checking between components
- Single installation config (no per-boat provisioning)
