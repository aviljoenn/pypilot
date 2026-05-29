#!/usr/bin/env bash
# Inno-Pilot bootstrap installer
#
# One-command fresh install on any Raspberry Pi running Raspberry Pi OS Lite
# (Bullseye or Bookworm, 32-bit or 64-bit):
#
#   curl -fsSL https://raw.githubusercontent.com/aviljoenn/inno-pilot/master/install.sh | bash
#
# Safe to re-run: each phase is idempotent.

set -euo pipefail

# Prevent apt from opening interactive pagers (apt-listchanges, debconf, etc.)
export DEBIAN_FRONTEND=noninteractive

REPO_URL="https://github.com/aviljoenn/inno-pilot.git"
REPO_DIR="$HOME/inno-pilot"
LOG="$HOME/inno-pilot-install.log"

# ── helpers ───────────────────────────────────────────────────────────────────

step() { echo; echo "==> $*"; }
info() { echo "    $*"; }
die()  { echo "ERROR: $*" >&2; exit 1; }

# Tee all output to log file
exec > >(tee -a "$LOG") 2>&1
echo "=== Inno-Pilot install started $(date) ==="

# ── detect OS / hardware ──────────────────────────────────────────────────────

step "Detecting hardware and OS"

PI_MODEL="unknown"
if [ -f /proc/device-tree/model ]; then
    PI_MODEL="$(tr -d '\0' </proc/device-tree/model)"
fi
info "Pi model : $PI_MODEL"

OS_CODENAME="$(. /etc/os-release && echo "${VERSION_CODENAME:-unknown}" | tr '[:upper:]' '[:lower:]')"
PYTHON_VER="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
info "OS       : $OS_CODENAME"
info "Python   : $PYTHON_VER"

# ── Phase 0 — interactive setup (all manual prompts up front) ────────────────
#
# Goal: every prompt the operator must respond to happens in this phase, so
# from Phase 1 onwards the install runs unattended.  Three pieces:
#   0a  passwordless sudo for innopilot (one password prompt, then never again)
#   0b  Tailscale binary install + auth URL + ENTER acknowledgement
#   0c  Telegram bot token / chat ID (optional, ENTER to skip)
#
# When run in a non-interactive context (no /dev/tty — e.g. plink -batch
# automation), the interactive prompts are skipped and Tailscale auth +
# Telegram conf can be completed manually after the install reboots.

step "Phase 0: interactive setup (sudo NOPASSWD, Tailscale, Telegram)"

if ( : < /dev/tty ) 2>/dev/null; then
    INTERACTIVE=true
else
    INTERACTIVE=false
    info "No controlling terminal — interactive prompts will be skipped."
fi

# 0a — passwordless sudo for innopilot.  The first sudo invocation below is
# the ONE password prompt the operator sees during the entire install (and
# only when NOPASSWD wasn't already configured — e.g. default `pi` user has
# it pre-configured).  After this, every later sudo runs without prompting,
# so the long Phase 3 (~30 min) won't trip over sudo's 15-minute auth cache.
SUDOERS_FILE="/etc/sudoers.d/010-innopilot-nopasswd"
if sudo -n true 2>/dev/null; then
    info "Passwordless sudo already configured — OK"
else
    info "Configuring passwordless sudo for innopilot (one-time password prompt)"
    sudo bash -c "echo 'innopilot ALL=(ALL) NOPASSWD: ALL' > $SUDOERS_FILE && chmod 440 $SUDOERS_FILE"
    info "NOPASSWD configured — no further password prompts during install."
fi

# 0b — Tailscale binary install (always) + auth (interactive only).
# Done BEFORE Phase 1's apt-update so all interactive bits are at the top
# and the operator can walk away once Phase 0 finishes.
if ! command -v tailscale >/dev/null 2>&1; then
    info "Installing Tailscale (apt repo + binary + service)"
    curl -fsSL https://tailscale.com/install.sh | sh
fi

if $INTERACTIVE; then
    if tailscale status >/dev/null 2>&1 && ! tailscale status 2>&1 | grep -qi "logged out"; then
        ts_ip=$(tailscale ip -4 2>/dev/null || echo "n/a")
        info "Tailscale: already authenticated — IP $ts_ip"
    else
        echo
        echo "======================================================"
        echo "  ACTION 1 of 2: Tailscale authentication"
        echo "  'sudo tailscale up' will print a URL — open it in"
        echo "  your browser, log in, then press ENTER here."
        echo "======================================================"
        echo
        sudo tailscale up || true
        echo
        read -rp "Press ENTER once Tailscale login is complete ... " _ack < /dev/tty
        if tailscale status >/dev/null 2>&1 && ! tailscale status 2>&1 | grep -qi "logged out"; then
            ts_ip=$(tailscale ip -4 2>/dev/null || echo "unknown")
            info "Tailscale: authenticated — IP $ts_ip"
        else
            info "WARNING: Tailscale still not connected — continuing anyway"
        fi
    fi
else
    info "Tailscale binary installed; auth deferred (run 'sudo tailscale up' after reboot)"
fi

# 0c — Telegram conf (optional, interactive only).  Written to ~/.pypilot/
# which is created here if needed.
TELEGRAM_CONF="$HOME/.pypilot/telegram.conf"
if $INTERACTIVE && [ ! -f "$TELEGRAM_CONF" ]; then
    echo
    echo "======================================================"
    echo "  ACTION 2 of 2: Telegram bot credentials (optional)"
    echo "  Used by inno-health-notify for OTA / failure alerts."
    echo "  Press ENTER at the token prompt to skip."
    echo "======================================================"
    echo
    read -rp "  Bot token (or ENTER to skip): " tg_token < /dev/tty
    if [ -n "$tg_token" ]; then
        read -rp "  Chat ID                    : " tg_chat_id < /dev/tty
        if [ -n "$tg_chat_id" ]; then
            mkdir -p "$(dirname "$TELEGRAM_CONF")"
            printf '{"token": "%s", "chat_id": "%s"}\n' "$tg_token" "$tg_chat_id" \
                > "$TELEGRAM_CONF"
            chmod 600 "$TELEGRAM_CONF"
            info "Telegram config written to $TELEGRAM_CONF"
        else
            info "No chat ID entered — Telegram skipped"
        fi
    else
        info "Telegram skipped (write $TELEGRAM_CONF manually later to enable)"
    fi
elif [ -f "$TELEGRAM_CONF" ]; then
    info "Telegram config already at $TELEGRAM_CONF — OK"
fi

info "Phase 0 complete — running unattended phases 1-7"

# ── Phase 1 — base packages ───────────────────────────────────────────────────

step "Phase 1: installing base packages"

# On a freshly-flashed Raspberry Pi OS SD card, unattended-upgrades typically
# runs on first boot and holds the dpkg lock for several minutes.  Tell apt to
# wait (up to 10 minutes) for the lock to release rather than fail immediately
# with "Could not get lock /var/lib/dpkg/lock-frontend".  Applies to every
# apt-get call in this Phase via the -o DPkg::Lock::Timeout option.
APT_OPTS=(-o DPkg::Lock::Timeout=600)

sudo apt-get "${APT_OPTS[@]}" update -y
sudo apt-get "${APT_OPTS[@]}" upgrade -y

# Packages common to all Pi OS versions
COMMON_PKGS=(
    git python3-pip python3-dev
    python3-numpy python3-scipy
    python3-serial python3-smbus
    i2c-tools socat
    libgles2-mesa-dev libgles2 libgbm-dev
    python3-setuptools
    avrdude
)

# Bookworm / Trixie (Debian 12/13) ship Flask 3.x via apt and block pip (PEP 668).
# Install web-stack packages via apt up front so dependencies.py never needs pip.
# Bullseye (Debian 11) still uses the old pip-based path.
case "$OS_CODENAME" in
    bookworm|trixie)
        EXTRA_PKGS=(
            python3-flask python3-flask-socketio python3-socketio
            python3-importlib-metadata
        )
        ;;
    *)
        # Bullseye or unknown: pigpio available; flask-socketio via pip in dependencies.py
        EXTRA_PKGS=(pigpio python3-flask)
        ;;
esac

# shellcheck disable=SC2086
sudo apt-get "${APT_OPTS[@]}" install -y "${COMMON_PKGS[@]}" "${EXTRA_PKGS[@]}"

step "Phase 1: enabling I2C"
sudo raspi-config nonint do_i2c 0

# ── Phase 2 — clone repo ──────────────────────────────────────────────────────

step "Phase 2: cloning inno-pilot repo"

if [ -d "$REPO_DIR/.git" ]; then
    info "Repo already exists — pulling latest master"
    git -C "$REPO_DIR" fetch origin
    git -C "$REPO_DIR" checkout master
    git -C "$REPO_DIR" pull --ff-only origin master
else
    git clone "$REPO_URL" "$REPO_DIR"
fi

# ── Phase 3 — install pypilot ─────────────────────────────────────────────────

step "Phase 3: installing pypilot (this takes several minutes on slow Pi hardware)"

cd "$REPO_DIR/compute_module/pypilot"
sudo python3 setup.py install

# dependencies.py (run inside setup.py above) installs pypilot_data, which copies a
# pyproject.toml into this directory.  On setuptools 78 (Trixie / Python 3.13) that
# pyproject.toml is treated as authoritative and overrides setup(packages=…), causing
# build_py to skip the pypilot Python source entirely — only the C extensions land in
# dist-packages.
#
# Naive remediation (rm pyproject.toml + re-run) loses the race: setup.py re-imports
# dependencies, which re-runs install_data.sh, which re-creates pyproject.toml BEFORE
# setup() is called.  The data idempotency check `os.path.exists('ui/compass.png')` no
# longer matches upstream pypilot_data's current layout (compass.png moved into a
# pypilot_data/ subdir), so install_data.sh runs unconditionally on every pass.
#
# Workaround: tell setup.py to skip `import dependencies` entirely by creating a stub
# `deps` file (the same marker dependencies.py writes on full success).  The first pass
# above has already fetched RTIMULib2 and pypilot_data, so the second pass only needs
# to install the pypilot Python modules and entry-point scripts.
if ! python3 -c "import pypilot.autopilot" 2>/dev/null; then
    info "pypilot Python modules missing — bypassing dependencies.py and retrying"
    rm -f pyproject.toml
    touch deps
    sudo python3 setup.py install
    rm -f deps
fi

# Hard verification — fail the install loudly here rather than letting Phase 6 crash
# on a partial pypilot.  /usr/local/bin/pypilot is the entry-point script systemd
# launches; if it's missing the autopilot will never start.
#
# The check must run with cwd outside the source tree.  Python prepends '' (cwd)
# to sys.path when running with -c, and the source tree contains pypilot/__init__.py
# but no compiled _linebuffer.so — so importing from source dir would falsely fail
# with "cannot import name '_linebuffer'".  At runtime systemd starts /usr/local/bin/pypilot
# with cwd=/, which resolves correctly via dist-packages.
( cd / && python3 -c "import pypilot.autopilot" ) \
    || die "pypilot Python package failed to install — autopilot would not start"
[ -x /usr/local/bin/pypilot ] \
    || die "pypilot entry-point script /usr/local/bin/pypilot was not created"

# Upstream pypilot ships pypilot.service in scripts/debian/etc/systemd/system/
# but its README expects the integrator to `sudo cp -r etc /` manually.  Our
# install.sh has to do that step explicitly, otherwise systemctl reports
# "Unit pypilot.service could not be found" after install and the autopilot
# never starts on boot.
#
# We install pypilot.service (the autopilot core) and pypilot_web.service (the
# web UI on port 8000 — it hosts the compass/heel calibration interface, which
# Inno-Pilot relies on, so it must always be present).  pypilot_hat and
# pypilot_boatimu are not used by Inno-Pilot and would only add maintenance
# burden if installed.
#
# The shipped pypilot.service hard-codes User=pi, which doesn't exist on
# Inno-Pilot Pis (we use innopilot).  Patch the User line in place during
# install.  (The pypilot_web.service unit in the repo already specifies
# User=innopilot, so it needs no patching.)
PYPILOT_UNIT_SRC="$REPO_DIR/compute_module/pypilot/scripts/debian/etc/systemd/system/pypilot.service"
PYPILOT_UNIT_DST="/etc/systemd/system/pypilot.service"
if [ -f "$PYPILOT_UNIT_SRC" ]; then
    info "Installing pypilot.service unit (with User=innopilot)"
    sudo install -m 644 "$PYPILOT_UNIT_SRC" "$PYPILOT_UNIT_DST"
    sudo sed -i 's/^User=pi$/User=innopilot/' "$PYPILOT_UNIT_DST"
    sudo systemctl daemon-reload
    sudo systemctl enable pypilot.service
else
    info "WARNING: pypilot.service template not found at $PYPILOT_UNIT_SRC — autopilot will not auto-start"
fi

# pypilot_web.service — the web UI on port 8000 that serves the calibration
# interface.  Installed and enabled the same way as pypilot.service above.  The
# repo unit already pins User=innopilot and async_mode=threading (see web.py),
# so no in-place patching is needed.  daemon-reload runs again so systemd picks
# up the newly-copied unit before enable.
PYPILOT_WEB_UNIT_SRC="$REPO_DIR/compute_module/pypilot/scripts/debian/etc/systemd/system/pypilot_web.service"
PYPILOT_WEB_UNIT_DST="/etc/systemd/system/pypilot_web.service"
if [ -f "$PYPILOT_WEB_UNIT_SRC" ]; then
    info "Installing pypilot_web.service unit (calibration web UI on port 8000)"
    sudo install -m 644 "$PYPILOT_WEB_UNIT_SRC" "$PYPILOT_WEB_UNIT_DST"
    sudo systemctl daemon-reload
    sudo systemctl enable pypilot_web.service
else
    info "WARNING: pypilot_web.service template not found at $PYPILOT_WEB_UNIT_SRC — calibration web UI will not auto-start"
fi

# ── Phase 4 — arduino-cli ────────────────────────────────────────────────────

step "Phase 4: installing arduino-cli"

if command -v arduino-cli >/dev/null 2>&1; then
    info "arduino-cli already installed: $(arduino-cli version)"
else
    # BINDIR controls where the arduino-cli installer places the binary.
    # env-var prefix on a piped command (VAR=x cmd | sh) only applies to the
    # left side of the pipe (curl), NOT to the right side (sh).  Download the
    # installer to a file first so we can pass BINDIR to sh directly.
    ARDUINO_TMP="$(mktemp -d)"
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
        -o "$ARDUINO_TMP/arduino-cli-install.sh"
    BINDIR="$ARDUINO_TMP" sh "$ARDUINO_TMP/arduino-cli-install.sh"
    sudo mv "$ARDUINO_TMP/arduino-cli" /usr/local/bin/arduino-cli
    rm -rf "$ARDUINO_TMP"
    arduino-cli core update-index
    arduino-cli core install arduino:avr
fi

# Arduino libraries required by motor_simple.ino.  Install (or update) every
# time so a re-run of install.sh repairs a partial library set.  arduino-cli
# stores these under ~/Arduino/libraries for the running user (innopilot);
# Phase 4c also runs as innopilot, so the paths line up.
step "Phase 4: installing Arduino libraries (SSD1306Ascii, OneWire, DallasTemperature)"
arduino-cli lib update-index
arduino-cli lib install "SSD1306Ascii" "OneWire" "DallasTemperature"

# ── Phase 4b — auto-detect connected Arduino ──────────────────────────────────

step "Phase 4b: detecting connected Arduino (writes /var/lib/inno-pilot/board.conf)"

# Run detect_arduino.sh from the repo so the consumer scripts deployed in
# Phase 5 can source board.conf.  Probe is non-destructive (avrdude -n no-write
# bootloader handshake).  Requires avrdude (Phase 1) and arduino-cli (Phase 4)
# already installed.  Tolerates "no Arduino plugged in yet" by continuing —
# detect_arduino.sh exit 1 is recoverable (operator can plug in the Nano and
# re-run inno_deploy.sh, which handles missing board.conf gracefully).
sudo mkdir -p /var/lib/inno-pilot
sudo chown innopilot:innopilot /var/lib/inno-pilot 2>/dev/null || true
if sudo bash "$REPO_DIR/compute_module/glue/detect_arduino.sh" --verbose; then
    info "Arduino detected — board.conf written"
else
    info "WARNING: Arduino detection failed (exit $?). Plug in the Nano and run:"
    info "    sudo /usr/local/sbin/detect_arduino.sh --verbose"
    info "before the first inno_deploy.sh run."
fi

# ── Phase 4c — compile + flash Nano firmware ─────────────────────────────────

step "Phase 4c: compiling and flashing Nano firmware"

# Skips gracefully if Phase 4b didn't produce a board.conf (no Arduino plugged
# in).  Inno-pilot services have not been started yet at this point, so
# /dev/ttyUSB0 is free and we don't need a stop/start dance.
if [ ! -f /var/lib/inno-pilot/board.conf ]; then
    info "WARNING: no /var/lib/inno-pilot/board.conf — skipping Nano flash."
    info "After plugging in the Nano, run: sudo /usr/local/sbin/detect_arduino.sh && bash ~/inno_deploy.sh"
else
    # shellcheck disable=SC1091
    source /var/lib/inno-pilot/board.conf
    NANO_SKETCH_DIR="$REPO_DIR/servo_motor_control/arduino/motor_simple"

    # arduino:avr core may be under root (~/.arduino15 absent for innopilot)
    # or innopilot's home, depending on how Phase 4 ran.  Detect by file system.
    if [ -d "$HOME/.arduino15/packages/arduino/hardware/avr" ]; then
        ARDUINO_CMD=(arduino-cli)
        ARDUINO15_ROOT="$HOME/.arduino15"
        info "Using user arduino:avr core ($ARDUINO15_ROOT)"
    else
        ARDUINO_CMD=(sudo arduino-cli)
        ARDUINO15_ROOT="/root/.arduino15"
        info "Using root arduino:avr core ($ARDUINO15_ROOT)"
    fi

    # ARMv6 avrdude compat fix: arduino-cli ships an armhf avrdude that SIGILLs
    # on Pi Zero / Pi 1.  Symlink the bundled binary + config to the system
    # avrdude (installed via apt in Phase 1) so upload works.
    if [ "$(uname -m)" = "armv6l" ]; then
        info "ARMv6 detected — applying avrdude compatibility fix"
        BUNDLED_AVRDUDE="$(sudo find "$ARDUINO15_ROOT/packages/arduino/tools/avrdude" \
                          -name avrdude \( -type f -o -type l \) 2>/dev/null | head -1)"
        SYS_AVRDUDE="$(command -v avrdude || true)"
        if [ -n "$BUNDLED_AVRDUDE" ] && [ -n "$SYS_AVRDUDE" ] && [ ! -L "$BUNDLED_AVRDUDE" ]; then
            BUNDLED_CONF="$(dirname "$(dirname "$BUNDLED_AVRDUDE")")/etc/avrdude.conf"
            sudo ln -sf "$SYS_AVRDUDE" "$BUNDLED_AVRDUDE"
            [ -f /etc/avrdude.conf ] && sudo ln -sf /etc/avrdude.conf "$BUNDLED_CONF"
            info "Symlinked bundled avrdude -> $SYS_AVRDUDE"
        fi
    fi

    info "Compiling $NANO_SKETCH_DIR (FQBN=$INNO_BOARD_FQBN)"
    cd "$NANO_SKETCH_DIR"
    "${ARDUINO_CMD[@]}" compile \
        --fqbn "$INNO_BOARD_FQBN" \
        --build-property "$INNO_BOARD_BUILD_FLAGS" \
        .

    # Free $INNO_BOARD_PORT before upload.  inno-pilot-bridge.service has
    # `Restart=always RestartSec=2`, so a plain `stop` races: the bridge is
    # back within 2s and holds the port, causing avrdude "not in sync resp=0x1c".
    # Mask the unit to suppress auto-restart for the duration of the flash,
    # then unmask afterwards.  On a fresh install the units don't exist yet,
    # so mask/stop are no-ops (the `|| true` makes them tolerant).
    info "Masking bridge/socat to prevent restart race during flash"
    sudo systemctl mask inno-pilot-bridge inno-pilot-socat 2>/dev/null || true
    sudo systemctl stop inno-pilot-bridge inno-pilot-socat 2>/dev/null || true
    sleep 2

    info "Compile OK; uploading to $INNO_BOARD_PORT"
    "${ARDUINO_CMD[@]}" upload \
        -p "$INNO_BOARD_PORT" \
        --fqbn "$INNO_BOARD_FQBN" \
        .
    info "Nano flashed successfully"

    # Restore normal startup behaviour — Phase 5 will start them via systemd
    # after the deploy is complete (or the final reboot will).
    sudo systemctl unmask inno-pilot-bridge inno-pilot-socat 2>/dev/null || true

    # Record sketch hash so inno_deploy.sh's hash-skip logic doesn't reflash
    # unnecessarily on the next deploy run.  Algorithm matches _nano_src_hash()
    # in inno_deploy.sh.
    NANO_HASH=$(
        ( echo "$INNO_BOARD_FQBN $INNO_BOARD_BUILD_FLAGS"
          find "$NANO_SKETCH_DIR" -maxdepth 1 \( -name "*.ino" -o -name "*.h" -o -name "*.cpp" \) \
              | sort | xargs sha256sum
        ) | sha256sum | awk '{print $1}'
    )
    sudo mkdir -p /var/lib/inno-pilot
    echo "$NANO_HASH" | sudo tee /var/lib/inno-pilot/nano_sketch.sha256 >/dev/null
    info "Sketch hash recorded — inno_deploy.sh will skip reflash on unchanged sketch"
fi

# ── Phase 5 — deploy glue layer ───────────────────────────────────────────────

step "Phase 5: deploying inno-pilot glue layer"

cd "$REPO_DIR"
bash compute_module/glue/deploy_inno_pilot_glue.sh

# Place inno_deploy.sh outside the repo so future `git pull` cannot modify it
# mid-execution (the script self-updates by copying the repo version over this
# one and re-execing — see comments at the top of inno_deploy.sh).  Without
# this step a fresh install has no working OTA / manual-deploy path.
install -m 755 "$REPO_DIR/inno_deploy.sh" "$HOME/inno_deploy.sh"
info "inno_deploy.sh placed at $HOME/inno_deploy.sh"

# ── Phase 6 — initialise pypilot config directory ────────────────────────────

step "Phase 6: initialising pypilot config"

if [ ! -d "$HOME/.pypilot" ]; then
    info "Starting pypilot briefly to generate default config..."
    python3 -m pypilot.autopilot &
    PYPILOT_PID=$!
    sleep 6
    kill "$PYPILOT_PID" 2>/dev/null || true
    wait "$PYPILOT_PID" 2>/dev/null || true
else
    info "~/.pypilot already exists, skipping init run"
fi

# Ensure ~/.pypilot exists (guards against pypilot failing to start above).
# servodevice itself is written by deploy_inno_pilot_glue.sh which sources
# board.conf — keeping a single writer prevents the two from drifting apart.
mkdir -p "$HOME/.pypilot"

# ── Phase 7 — fix stale pypilot_client.conf ──────────────────────────────────

case "$OS_CODENAME" in
    bookworm|trixie)
        step "Phase 7: resetting pypilot_client.conf to localhost ($OS_CODENAME)"
        mkdir -p "$HOME/.pypilot"
        echo '{"host":"127.0.0.1","port":23322}' > "$HOME/.pypilot/pypilot_client.conf"
        info "pypilot_client.conf set to 127.0.0.1"
        ;;
esac

# ── Done ──────────────────────────────────────────────────────────────────────

echo
echo "============================================================"
echo " Inno-Pilot installation complete!"
echo " Log saved to: $LOG"
echo
echo " Detected Arduino:"
if [ -f /var/lib/inno-pilot/board.conf ]; then
    grep -E '^INNO_BOARD_(VARIANT|FQBN|PORT)=' /var/lib/inno-pilot/board.conf | sed 's/^/   /'
else
    echo "   (no board.conf — plug in the Nano and run sudo /usr/local/sbin/detect_arduino.sh)"
fi
echo
echo " Flash the Nano with the matching FQBN via:"
echo "   bash ~/inno_deploy.sh"
echo
echo " Rebooting in 10 seconds — Ctrl-C to cancel."
echo "============================================================"

sleep 10
sudo reboot
