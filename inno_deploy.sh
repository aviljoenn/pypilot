#!/bin/bash
# =============================================================================
# Inno-Pilot full release deployment script
# Resides at ~/inno_deploy.sh on the Pi (OUTSIDE the repo to prevent git pull
# from modifying this script while it is running).
#
# Usage:
#   bash ~/inno_deploy.sh [branch]
#
# If branch is omitted, the current branch of ~/inno-pilot is used.
#
# What this script does:
#   0. First-run provisioning (Tailscale, Telegram config, /var/lib/inno-pilot)
#   1. git pull the repo
#      → if inno_deploy.sh changed in the pull, re-exec the new version and exit
#   2. Pre-compile Nano firmware (while services are still up — no port needed)
#   3. Stop all inno-pilot + pypilot services
#   4. Deploy glue (bridge, web-remote, systemd units, OTA binary)
#   5. Flash the Nano via arduino-cli
#   6. Wait for the Nano to boot after flash
#   7. Start all services in dependency order
#   8. Print a final status summary
# =============================================================================

set -euo pipefail

# --------------- Configurable paths / names ---------------
REPO_DIR="$HOME/inno-pilot"
GLUE_DEPLOY="$REPO_DIR/compute_module/glue/deploy_inno_pilot_glue.sh"
NANO_SKETCH_DIR="$REPO_DIR/servo_motor_control/arduino/motor_simple"
ARDUINO_CLI="/usr/local/bin/arduino-cli"
# Defaults — overridden by /var/lib/inno-pilot/board.conf if present.  The
# defaults match the historical Inno-Pilot Nano (CH340 USB-serial) so existing
# .12 / .13 fleet keeps working until detect_arduino.sh has been run on them.
NANO_PORT="/dev/ttyUSB0"
NANO_FQBN="arduino:avr:nano"
NANO_BUILD_FLAG="build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=128"
if [ -f /usr/local/sbin/board_conf.sh ] && [ -f /var/lib/inno-pilot/board.conf ]; then
    # shellcheck disable=SC1091
    source /usr/local/sbin/board_conf.sh
    if load_board_conf 2>/dev/null; then
        NANO_PORT="$INNO_BOARD_PORT"
        NANO_FQBN="$INNO_BOARD_FQBN"
        NANO_BUILD_FLAG="$INNO_BOARD_BUILD_FLAGS"
    fi
fi
NANO_BOOT_WAIT_S=6    # seconds to wait for Nano to finish its setup() splash after flash
BRIDGE_SETTLE_S=3     # seconds to let the bridge stabilise before starting pypilot
TELEGRAM_CONF="/home/innopilot/.pypilot/telegram.conf"
# Hash of last successfully flashed sketch — used to skip unchanged firmware.
# Delete this file to force a reflash regardless of source state.
NANO_HASH_FILE="/var/lib/inno-pilot/nano_sketch.sha256"
# All deploy output is tee'd here so the web-remote can stream it live to the browser.
OTA_LOG_FILE="/var/lib/inno-pilot/ota_log.txt"
# ----------------------------------------------------------

log()  { echo "[$(date '+%H:%M:%S')] $*"; }
die()  { echo "[ERROR] $*" >&2; exit 1; }
info() { echo; echo ">>> $*"; echo; }

# --------------- arduino-cli invocation ---------------
# Pi .12 (Pi5): arduino:avr core lives under /root/.arduino15; must use sudo.
# Pi .13 (Pi Zero): core lives under innopilot's own ~/.arduino15; no sudo needed.
# Detect by checking the core hardware directory on disk — avoids triggering a
# network index download (which would hang silently and OOM the Pi).
if [ -d "$HOME/.arduino15/packages/arduino/hardware/avr" ]; then
    ARDUINO=("$ARDUINO_CLI")
    ARDUINO15_ROOT="$HOME/.arduino15"
    log "arduino-cli: using user core (~/.arduino15)"
else
    ARDUINO=(sudo "$ARDUINO_CLI")
    ARDUINO15_ROOT="/root/.arduino15"
    log "arduino-cli: user core absent, falling back to sudo (root core)"
fi
# ------------------------------------------------------

# --------------- ARMv6 avrdude compatibility fix ---------------
# arduino-cli downloads avrdude compiled for armhf (ARMv7+). On ARMv6 (Pi Zero),
# executing an armhf binary causes SIGILL ("illegal instruction"). The system
# avrdude (apt package avrdude) is compiled for ARMv6 and works correctly.
# This function replaces the bundled binary with a symlink to the system one.
# The symlink is idempotent and survives reboots; it is only overwritten if
# arduino-cli explicitly reinstalls its avrdude package.
fix_avrdude_for_armv6() {
    local bundled_bin bundled_conf
    bundled_bin=$(find "$ARDUINO15_ROOT/packages/arduino/tools/avrdude" \
                  -name avrdude \( -type f -o -type l \) 2>/dev/null | head -1)
    if [[ -z "$bundled_bin" ]]; then
        # Core tools not yet downloaded — nothing to fix; upload would fail anyway.
        log "avrdude compat: bundled avrdude not found in $ARDUINO15_ROOT — skipping fix."
        return
    fi

    # Ensure the system avrdude package is present.
    if ! command -v avrdude &>/dev/null; then
        log "avrdude compat: system avrdude not installed — installing via apt ..."
        sudo apt-get install -y avrdude
    fi
    local system_avrdude system_conf
    system_avrdude=$(command -v avrdude)
    # System avrdude config is at /etc/avrdude.conf
    system_conf="/etc/avrdude.conf"

    # --- binary ---
    if [[ -L "$bundled_bin" ]]; then
        log "avrdude compat: binary already symlinked ($(readlink "$bundled_bin")) — OK."
    else
        log "avrdude compat: symlinking binary -> $system_avrdude"
        if [[ "$bundled_bin" == /root/* ]]; then
            sudo ln -sf "$system_avrdude" "$bundled_bin"
        else
            ln -sf "$system_avrdude" "$bundled_bin"
        fi
    fi

    # --- config file ---
    # arduino-cli passes the bundled etc/avrdude.conf explicitly via -C. The 8.0.0
    # config format is incompatible with the system avrdude 7.x. Symlink it to the
    # system config so the versions stay in sync.
    bundled_conf="$(dirname "$(dirname "$bundled_bin")")/etc/avrdude.conf"
    if [[ -L "$bundled_conf" ]]; then
        log "avrdude compat: config already symlinked ($(readlink "$bundled_conf")) — OK."
    elif [[ ! -f "$system_conf" ]]; then
        log "avrdude compat: WARNING — system config $system_conf not found; upload may fail."
    else
        log "avrdude compat: symlinking config -> $system_conf"
        if [[ "$bundled_conf" == /root/* ]]; then
            sudo ln -sf "$system_conf" "$bundled_conf"
        else
            ln -sf "$system_conf" "$bundled_conf"
        fi
    fi

    log "avrdude compat: fix applied."
}
# ------------------------------------------------------

# Wrap all executable logic in main() so that bash reads the entire file into
# memory before starting execution. This is necessary for the self-update re-exec:
# without the wrapper, bash reads the script in chunks from disk and if the file
# is overwritten mid-execution it reads the new content at the wrong offset,
# causing "syntax error near unexpected token" on whatever lands at that position.
main() {

# Guard: refuse to run on the dev workstation
case "$(uname -m)" in
  arm* | aarch64) ;;
  *) die "This script is for the Raspberry Pi, not the dev workstation (arch: $(uname -m))." ;;
esac

# Determine branch
BRANCH="${1:-$(git -C "$REPO_DIR" rev-parse --abbrev-ref HEAD)}"

# ---------------------------------------------------------------------------
# OTA log file — pipe all output into OTA_LOG_FILE so the web-remote can
# stream progress live.  Truncate on a fresh run; skip on self-update re-exec
# so the re-exec'd instance inherits the already-open tee fd without creating
# a second tee process or clobbering the log mid-stream.
# ---------------------------------------------------------------------------
if [ "${_OTA_REEXEC:-0}" != "1" ]; then
    mkdir -p "$(dirname "$OTA_LOG_FILE")"
    > "$OTA_LOG_FILE"
    exec > >(tee -a "$OTA_LOG_FILE") 2>&1
fi
_ota_exit_marker() { [ "$1" -eq 0 ] && echo "=== COMPLETE ===" || echo "=== FAILED (exit $1) ==="; }
trap '_ota_exit_marker $?' EXIT

# ---------------------------------------------------------------------------
info "Step 0 — First-run provisioning checks"
# ---------------------------------------------------------------------------
# All checks are idempotent: already-configured items are skipped silently.
# Items that require a manual action pause and prompt the user before continuing.

# ---- /var/lib/inno-pilot directory ----
if [[ ! -d "/var/lib/inno-pilot" ]]; then
    log "Creating /var/lib/inno-pilot ..."
    sudo mkdir -p /var/lib/inno-pilot
    sudo chown innopilot:innopilot /var/lib/inno-pilot
    log "/var/lib/inno-pilot created."
else
    log "Data dir /var/lib/inno-pilot: exists — OK."
fi

# ---- Tailscale ----
if ! command -v tailscale &>/dev/null; then
    log "Tailscale not found — installing ..."
    curl -fsSL https://tailscale.com/install.sh | sh
    log "Tailscale installed."
fi

# tailscale status exits non-zero and prints "Logged out" / error when not connected
if tailscale status &>/dev/null 2>&1 && ! tailscale status 2>&1 | grep -qi "logged out"; then
    ts_ip=$(tailscale ip -4 2>/dev/null || echo "n/a")
    log "Tailscale: connected — IP $ts_ip — OK."
else
    log ""
    log "======================================================"
    log "  ACTION REQUIRED: Tailscale is not authenticated."
    log "  Running 'sudo tailscale up' — open the URL it"
    log "  prints in a browser on any device and log in."
    log "======================================================"
    log ""
    # tailscale up prints the auth URL to stdout; run it and let the user see it
    sudo tailscale up || true
    log ""
    read -rp "Press ENTER once you have completed Tailscale login ... "
    if tailscale status &>/dev/null 2>&1 && ! tailscale status 2>&1 | grep -qi "logged out"; then
        ts_ip=$(tailscale ip -4 2>/dev/null || echo "unknown")
        log "Tailscale: authenticated — IP $ts_ip"
    else
        log "WARNING: Tailscale still does not appear connected — continuing anyway."
        log "         Re-run this script after completing Tailscale auth to verify."
    fi
fi

# ---- Telegram config ----
if [[ -f "$TELEGRAM_CONF" ]]; then
    log "Telegram: config exists at $TELEGRAM_CONF — OK."
else
    log ""
    log "======================================================"
    log "  ACTION REQUIRED: Telegram config not found."
    log "  Enter your bot credentials to enable notifications."
    log "  Leave blank to skip (you can create the file later)."
    log "======================================================"
    log ""
    read -rp "  Bot token (press ENTER to skip): " tg_token
    if [[ -n "$tg_token" ]]; then
        read -rp "  Chat ID   : " tg_chat_id
        if [[ -n "$tg_chat_id" ]]; then
            mkdir -p "$(dirname "$TELEGRAM_CONF")"
            printf '{"token": "%s", "chat_id": "%s"}\n' "$tg_token" "$tg_chat_id" \
                > "$TELEGRAM_CONF"
            chmod 600 "$TELEGRAM_CONF"
            # Ensure the innopilot user owns the conf dir and file
            sudo chown -R innopilot:innopilot "$(dirname "$TELEGRAM_CONF")" 2>/dev/null || true
            log "Telegram config written to $TELEGRAM_CONF"
        else
            log "WARNING: No chat ID entered — Telegram config skipped."
        fi
    else
        log "Telegram config skipped — create $TELEGRAM_CONF manually to enable notifications."
    fi
fi

# ---------------------------------------------------------------------------
info "Step 1 — git pull (branch: $BRANCH)"
# ---------------------------------------------------------------------------
cd "$REPO_DIR"
# When running as root (OTA path), git 2.35+ blocks access to repos owned by
# another user unless the directory is explicitly marked safe.
git config --global --add safe.directory "$REPO_DIR" 2>/dev/null || true
git pull origin "$BRANCH"
log "Pull complete."

# ---------------------------------------------------------------------------
# Self-update re-exec: if the pull brought a new version of this deploy script,
# copy it over ~/inno_deploy.sh and re-exec so the rest of the deployment runs
# under the new logic.  The new instance will find no diff on its own re-exec
# check and proceed normally from Step 0 onwards.
# ---------------------------------------------------------------------------
REPO_DEPLOY="$REPO_DIR/inno_deploy.sh"
SELF="$HOME/inno_deploy.sh"
if [[ -f "$REPO_DEPLOY" ]] && ! diff -q "$REPO_DEPLOY" "$SELF" >/dev/null 2>&1; then
    log "Deploy script updated in this pull — relaunching new version ..."
    cp "$REPO_DEPLOY" "$SELF"
    chmod 755 "$SELF"
    # Disable EXIT trap so this (parent) instance does not write the completion
    # marker — the re-exec'd instance will write it after its own EXIT.
    # Export _OTA_REEXEC so the child skips re-truncating and re-opening the log.
    trap - EXIT
    export _OTA_REEXEC=1
    exec bash "$SELF" "$@"
fi
log "Deploy script is current — continuing."

# ---------------------------------------------------------------------------
# Nano firmware change detection
# Hash all source files in the sketch directory. If the hash matches the last
# successful flash, skip compile and flash entirely to save ~2 minutes of
# downtime on glue-only deploys.  Delete NANO_HASH_FILE to force a reflash.
# ---------------------------------------------------------------------------
_nano_src_hash() {
    # Sort paths so hash is order-independent; include the FQBN + build flag so
    # a toolchain change can be forced by clearing the hash file manually.
    (
        echo "$NANO_FQBN $NANO_BUILD_FLAG"
        find "$NANO_SKETCH_DIR" -maxdepth 1 \( -name "*.ino" -o -name "*.h" -o -name "*.cpp" \) \
            | sort | xargs sha256sum
    ) | sha256sum | awk '{print $1}'
}

NANO_CURRENT_HASH=$(_nano_src_hash)
if [[ -f "$NANO_HASH_FILE" ]] && [[ "$(cat "$NANO_HASH_FILE")" == "$NANO_CURRENT_HASH" ]]; then
    FLASH_NANO=false
    log "Nano firmware unchanged — compile and flash will be skipped."
else
    FLASH_NANO=true
    log "Nano firmware changed (or first run) — will compile and flash."
fi

# ---------------------------------------------------------------------------
info "Step 2 — Pre-compile Nano firmware (services still running)"
# ---------------------------------------------------------------------------
# Compile does not touch /dev/ttyUSB0, so we can do this before stopping services
# to minimise the downtime window.
if $FLASH_NANO; then
    cd "$NANO_SKETCH_DIR"
    log "Compiling $NANO_SKETCH_DIR ..."
    "${ARDUINO[@]}" compile \
        --fqbn "$NANO_FQBN" \
        --build-property "$NANO_BUILD_FLAG" \
        .
    log "Nano firmware compiled OK."
else
    log "Skipping compile (firmware unchanged)."
fi

# ---------------------------------------------------------------------------
info "Step 3 — Stop services (consumers first, providers last)"
# ---------------------------------------------------------------------------
stop_svc() {
    local svc="$1"
    if systemctl list-unit-files "${svc}.service" &>/dev/null \
       && systemctl is-active --quiet "$svc" 2>/dev/null; then
        log "  Stopping $svc ..."
        sudo systemctl stop "$svc"
    else
        log "  $svc — already stopped or not installed, skipping."
    fi
}

stop_svc pypilot_web
stop_svc pypilot
stop_svc inno-health-notify
stop_svc inno-pilot-web-remote
stop_svc inno-pilot-bridge
stop_svc inno-pilot-fixlink
stop_svc inno-pilot-socat

# ---------------------------------------------------------------------------
info "Step 4 — Deploy glue (bridge, web-remote, systemd units, OTA binary)"
# ---------------------------------------------------------------------------
bash "$GLUE_DEPLOY"
log "Glue deploy complete."

# ---------------------------------------------------------------------------
info "Step 4b — Install pypilot Python package (web templates, JS, core)"
# ---------------------------------------------------------------------------
# The pypilot web interface files (templates/, static/) are part of the pypilot
# Python package and are only picked up by the running service after a fresh
# `setup.py install`. A plain `git pull` updates the repo but not the installed
# package — this step closes that gap.
#
# Two-pass install (see the same workaround in install.sh Phase 3):
# dependencies.py — run inside `setup.py install` — drops a pyproject.toml into
# this dir, and on setuptools 78 / Python 3.13 that pyproject.toml overrides
# setup(packages=…), so build_py skips the *pure-Python* pypilot source entirely
# (only the C extensions and pypilot_data land in dist-packages).  install.sh
# only re-runs the workaround when pypilot is *missing*; on a redeploy the import
# always succeeds, so without this second pass any edit to pypilot Python source
# (e.g. web/web.py — async_mode, the Flask Markup fix) never reaches the Pi and
# the running service keeps using the stale installed copy.
#
# Pass 1 fetches deps and builds the C extensions.  Pass 2 bypasses
# dependencies.py — by removing the pyproject.toml it dropped and touching the
# `deps` success-marker it checks for — so build_py uses setup(packages=…) and
# actually installs/updates the pypilot Python modules and entry-point scripts.
PYPILOT_DIR="$REPO_DIR/compute_module/pypilot"
cd "$PYPILOT_DIR"
log "Running setup.py install in $PYPILOT_DIR (pass 1 — deps + C extensions) ..."
sudo python3 setup.py install --quiet
log "Re-running setup.py install (pass 2 — pypilot Python source, deps bypassed) ..."
# Use sudo for the marker-file shuffle: pass 1 ran under sudo, so dependencies.py
# created `deps` (and pyproject.toml) as root.  A plain `touch deps` as the
# invoking user then fails with "Permission denied" on the root-owned file,
# aborting the deploy mid-way (services already stopped in Step 3).  Same
# root-owned-file hazard the NANO_HASH_FILE guard in Step 5 handles.
sudo rm -f pyproject.toml
sudo touch deps
sudo python3 setup.py install --quiet
sudo rm -f deps
log "pypilot package installed."

# Ensure the pypilot_web.service unit (calibration web UI on port 8000) is
# installed and enabled.  install.sh sets this up on a fresh build, but Pis
# provisioned before pypilot_web was made mandatory won't have the unit — so a
# redeploy must create it too, otherwise Step 7 silently skips starting it.
# The repo unit already pins User=innopilot, so no in-place patching is needed.
PYPILOT_WEB_UNIT_SRC="$PYPILOT_DIR/scripts/debian/etc/systemd/system/pypilot_web.service"
PYPILOT_WEB_UNIT_DST="/etc/systemd/system/pypilot_web.service"
if [ -f "$PYPILOT_WEB_UNIT_SRC" ]; then
    log "Installing/enabling pypilot_web.service (calibration web UI on port 8000) ..."
    sudo install -m 644 "$PYPILOT_WEB_UNIT_SRC" "$PYPILOT_WEB_UNIT_DST"
    sudo systemctl daemon-reload
    sudo systemctl enable pypilot_web.service
else
    log "WARNING: pypilot_web.service template not found at $PYPILOT_WEB_UNIT_SRC — calibration web UI will not be installed"
fi

# ---------------------------------------------------------------------------
info "Step 5 — Flash Nano firmware"
# ---------------------------------------------------------------------------
if $FLASH_NANO; then
    # On ARMv6 (Pi Zero) arduino-cli bundles an armhf avrdude that crashes with
    # SIGILL. Replace it with a symlink to the system avrdude before upload.
    if [[ "$(uname -m)" == "armv6l" ]]; then
        fix_avrdude_for_armv6
    fi

    # NOTE: HUPCL is not relevant here because arduino-cli handles the reset pulse
    # intentionally (needed for programming). After upload completes and arduino-cli
    # closes the port, the Nano will reset once more — that is the boot we wait for
    # in Step 6.
    #
    # Re-stop serial-port services before flashing. inno-pilot-bridge has a systemd
    # Restart= policy and can auto-restart itself during the long Step 4b pypilot
    # install, so it may be activating again by the time we reach Step 5. A bridge
    # that holds or has recently held /dev/ttyUSB0 corrupts the avrdude bootloader
    # dialogue, causing "programmer is out of sync".
    log "Re-stopping bridge/socat to ensure serial port is free ..."
    sudo systemctl stop inno-pilot-bridge inno-pilot-socat 2>/dev/null || true
    sleep 2

    # Upload with one automatic retry — a transient sync error on the first
    # attempt is rare but recoverable; a second failure is a real problem.
    cd "$NANO_SKETCH_DIR"
    log "Flashing Nano on $NANO_PORT ..."
    upload_ok=false
    for attempt in 1 2; do
        if "${ARDUINO[@]}" upload \
               -p "$NANO_PORT" \
               --fqbn "$NANO_FQBN" \
               .; then
            upload_ok=true
            break
        fi
        if [[ "$attempt" -eq 1 ]]; then
            log "Upload attempt 1 failed — waiting 3s and retrying ..."
            sleep 3
        fi
    done
    $upload_ok || die "Nano upload failed after 2 attempts."
    log "Flash complete."

    # Record successful flash so future deploys can skip unchanged firmware.
    # Repair ownership if a previous deploy ran via sudo and left the file
    # root-owned — otherwise this redirect fails with "Permission denied"
    # and the script aborts after a successful flash.
    if [[ -e "$NANO_HASH_FILE" && ! -w "$NANO_HASH_FILE" ]]; then
        log "Repairing ownership of $NANO_HASH_FILE (not writable as $(id -un)) ..."
        sudo chown "$(id -un):$(id -gn)" "$NANO_HASH_FILE"
    fi
    echo "$NANO_CURRENT_HASH" > "$NANO_HASH_FILE"
    log "Nano firmware hash saved."
else
    log "Skipping flash (firmware unchanged)."
fi

# ---------------------------------------------------------------------------
info "Step 6 — Post-flash Nano boot wait"
# ---------------------------------------------------------------------------
# The Nano runs a ~3 s OLED splash in setup(). The bridge needs the Nano to be
# ready before sending FEATURES_CODE and HELLO, so give it ample time here.
if $FLASH_NANO; then
    log "Waiting ${NANO_BOOT_WAIT_S}s for Nano to complete setup() after flash ..."
    sleep "$NANO_BOOT_WAIT_S"
    log "Nano should be ready."
else
    log "Skipping boot wait (no flash performed)."
fi

# ---------------------------------------------------------------------------
info "Step 7 — Start services (providers first, consumers last)"
# ---------------------------------------------------------------------------
start_svc() {
    local svc="$1"
    if systemctl list-unit-files "${svc}.service" &>/dev/null; then
        log "  Starting $svc ..."
        sudo systemctl start "$svc"
    else
        log "  $svc — unit not installed, skipping."
    fi
}

# socat must be up before fixlink and bridge use the PTY pair
start_svc inno-pilot-socat

# fixlink is a one-shot that redirects the USB symlink; it will go inactive after
# running — that is expected and not an error
start_svc inno-pilot-fixlink

# bridge grabs the real USB port and opens the TCP server on port 8555
start_svc inno-pilot-bridge

# web remote connects to the bridge's TCP server
start_svc inno-pilot-web-remote

# health notify runs independently — no ordering dependency on autopilot
start_svc inno-health-notify

# Let the bridge stabilise (FEATURES_CODE + HELLO exchange with Nano) before
# autopilot tries to grab the PTY
log "  Waiting ${BRIDGE_SETTLE_S}s for bridge to stabilise ..."
sleep "$BRIDGE_SETTLE_S"

# autopilot connects via the PTY symlink created by socat + fixlink
start_svc pypilot

# pypilot_web (port 8000) hosts the calibration UI and is now installed on every
# Inno-Pilot by Step 4b above, so it should always be present.  The guard remains
# as a safety net in case the unit failed to install.
if systemctl list-unit-files pypilot_web.service &>/dev/null; then
    start_svc pypilot_web
else
    log "WARNING: pypilot_web.service missing — calibration UI (port 8000) will not start"
fi

# ---------------------------------------------------------------------------
info "Step 8 — Service status summary"
# ---------------------------------------------------------------------------
# Give all services a moment to finish activating before checking status.
# inno-pilot-web-remote in particular takes a few seconds to establish its
# TCP connection to the bridge and reach the "active" steady state.
log "  Waiting 6s for services to reach steady state ..."
sleep 6
SVCS=(inno-pilot-socat inno-pilot-fixlink inno-pilot-bridge inno-pilot-web-remote inno-health-notify pypilot pypilot_web)
all_ok=true
for svc in "${SVCS[@]}"; do
    if ! systemctl list-unit-files "${svc}.service" &>/dev/null; then
        log "  $svc: (not installed)"
        continue
    fi
    status=$(systemctl is-active "$svc" 2>/dev/null || true)
    # fixlink is one-shot; inactive after running is normal
    if [[ "$svc" == "inno-pilot-fixlink" && "$status" == "inactive" ]]; then
        log "  $svc: inactive (one-shot — OK)"
    elif [[ "$status" == "active" ]]; then
        log "  $svc: active ✓"
    else
        log "  $svc: $status  ← CHECK THIS"
        all_ok=false
    fi
done

echo
if $all_ok; then
    log "=== Deployment complete — all services nominal ==="

    # Telegram completion notification (best-effort — never blocks or fails deploy)
    python3 - 2>/dev/null <<'PYEOF' || true
import json, urllib.request
TELEGRAM_CONF = "/home/innopilot/.pypilot/telegram.conf"
SETTINGS_FILE = "/var/lib/inno-pilot/settings.json"
try:
    conf = json.load(open(TELEGRAM_CONF))
    token, chat_id = conf["token"], str(conf["chat_id"])
    try:
        name = json.load(open(SETTINGS_FILE)).get("vessel", {}).get("name", "").strip()
        prefix = f"[{name}] " if name else ""
    except Exception:
        prefix = ""
    text = f"{prefix}Update complete — all services nominal"
    data = json.dumps({"chat_id": chat_id, "text": text}).encode()
    req = urllib.request.Request(
        f"https://api.telegram.org/bot{token}/sendMessage",
        data=data, headers={"Content-Type": "application/json"})
    urllib.request.urlopen(req, timeout=10)
except Exception:
    pass
PYEOF
else
    log "=== Deployment finished — one or more services need attention (see above) ==="
    exit 1
fi

} # end main()

main "$@"
