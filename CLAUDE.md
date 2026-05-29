# CLAUDE.md
Audience: Claude Code (and other AI coding agents) working in this repository.

## Prime directive
1) **Do not break working systems.** Keep changes minimal, focused, and testable.
2) **GitHub is the source of truth.** Modify repo files, open a PR, and describe what changed.
3) **Preserve human intent.** This repo runs on real hardware; avoid sweeping refactors.

---

## Coding style & preferences (very important)
### Comments
- **Do NOT remove existing Python comments** unless they are wrong or dangerously misleading.
- If a comment must change: prefer **fixing** it (or expanding it) rather than deleting it.
- Adding comments to improve human readability is mandatory. Keep them practical.

### Python
- Prefer **clear, explicit code** over cleverness.
- Add **type hints** for new/changed public functions where reasonable.
- Prefer docstrings for modules/classes with non-trivial behavior.
- Avoid adding heavy dependencies unless explicitly requested.

### Arduino / ESP32 (C++ / sketches)
- Keep changes small and hardware-safe.
- Avoid timing-sensitive behavior changes unless explicitly requested.
- Prefer constants, enums, and clear pin naming.
- If touching IO pins, power, ADC scaling, or interrupts: **explain assumptions** in the PR.

### Testing of code
- Code must be compiled, started and executed at the least.
- Attempt must be made to automatically simulate user inputs where possible.
- If automatically simulating user inputs is not possible, the user must be guided and prompted on what to do to complete testing.

---

## Repo hygiene rules
- **Do not rename/move files** unless explicitly requested.
- Avoid formatting-only PRs.
- Keep PRs small (ideally one concern per PR).
- Never commit secrets, tokens, WiFi credentials, or private keys.

---

## What "done" looks like
A PR is "done" when it includes:
- A clear summary + rationale
- Tests/build steps run (or why not)
- Any new/changed docs needed (README/docs)
- Release notes updated

---

## How to run / verify (use what exists; don't invent tooling)
Before adding new tooling, check what the repo already uses:
- Look for: `pyproject.toml`, `requirements.txt`, `setup.cfg`, `tox.ini`, `.pre-commit-config.yaml`,
  `platformio.ini`, `arduino-cli.yaml`, `Makefile`, GitHub Actions workflows.

### Python: preferred order
1) If `pyproject.toml` + Poetry:
   - Install: `poetry install`
   - Run: `poetry run pytest -q` (or existing commands)
2) If `requirements.txt`:
   - Install: `python -m venv .venv && . .venv/bin/activate && pip install -r requirements.txt`
   - Run: `pytest -q` (or existing commands)

### Python quality gates (only if present in repo)
- Formatting: `ruff format .` or `black .`
- Lint: `ruff check .` or `flake8`
- Types: `mypy .`

**Rule:** If the repo already uses a tool, use it. If not, don't add one unless asked.

---

## Firmware build / check
### PlatformIO (if `platformio.ini` exists)
- Build: `pio run`
- If tests exist: `pio test`

### Arduino CLI (if used in repo)
- Compile (example): `arduino-cli compile --fqbn <FQBN> <sketch_dir>`
- Do not guess FQBNs. Look for existing CI scripts/docs.

**Rule:** If you can't build firmware due to missing toolchain/board config, state that clearly in the PR and keep changes conservative.

> **Facts about the Inno-Pilot Raspberry Pi** 
> Every Inno-Pilot constructed may use different hardware types, E.g. Pi5, Pi Zero, Arduino nano, Pi Pico etc.
> Every Inno-Pilot constructed may be on different IP subnets with different IP addresses
> `arduino-cli` is installed at `/usr/local/bin/arduino-cli`
> with the `arduino:avr` core. The Nano is on `/dev/ttyUSB0`.
> Compile: `arduino-cli compile --fqbn arduino:avr:nano --build-property "build.extra_flags=-DSERIAL_RX_BUFFER_SIZE=128" .`
> Upload:  `arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano .`
> The `SERIAL_RX_BUFFER_SIZE=128` flag is **required** — the default 64-byte buffer overflows
> during bridge telemetry bursts while the OLED I2C draw blocks `loop()`.
> Stop `inno-pilot-bridge`, `inno-pilot-socat`, `pypilot` services before flashing; restart after.

---

## Change strategy
When asked to implement something:
1) Identify the smallest set of files to change.
2) Implement with guardrails (validation, bounds checks, sane defaults).
3) Update docs/comments where it prevents future mistakes.
4) Run the best available checks/tests.
5) Open PR with a high-signal description.
6) Check all components of inno-remote that might be versioned, like the nano sketch, the bridge, updated OTA binary that also lands at /var/lib/inno-pilot/ota/ on the Pi, inno-remote and inno-web-remote. Keep the version numbers of all those components in sync and push/flash the same version number to all components.

---

## PR description template (use this structure)
**What**
- (1–3 bullets)

**Why**
- (short rationale)

**How**
- Key implementation notes (include hardware assumptions if any)

**Notes / Follow-ups**
- Any TODOs, edge cases, or recommended next steps

---

## Hardware-safety flags (call these out explicitly)
If changes touch any of the following, add a dedicated "Hardware impact" section in the PR:
- Pin mappings / GPIO modes
- PWM frequency/duty behavior
- ADC scaling, voltage dividers, calibration constants
- Power control, relays, motors, H-bridges
- Interrupts, watchdogs, real-time loops
- Serial/I2C/SPI protocol timing

---

## Known hardware gotcha: Nano reset via HUPCL

**The Arduino Nano resets whenever `/dev/ttyUSB0` is closed by any process.**

The Nano's RESET pin is wired to DTR through a 100 nF RC differentiator (standard
Arduino Uno/Nano design). Linux serial ports have the HUPCL flag set by default,
which drops DTR whenever the file descriptor is closed — even if `dtr=False` was
set while the port was open.

**Symptoms:** Nano shows high CRC error counts, bridge receives zero bytes from Nano
after any process closes the port (including a previous bridge instance, a flash
tool, or `stty`). The Nano is stuck in its `setup()` splash delay (~3 s) and can't
communicate.

**Fix already in place:** `open_serial_no_reset()` in `inno_pilot_bridge.py` clears
HUPCL via `termios.tcsetattr` immediately after opening the port.

**If you ever add a new tool that opens `/dev/ttyUSB0`** (diagnostic scripts,
sniffers, etc.), either:
1. Run `stty -F /dev/ttyUSB0 -hupcl` before closing, **or**
2. Open with `pyserial` and apply the same `termios` fix, **or**
3. Accept that the Nano will reset and allow ≥ 5 s before expecting frames.

---

---

## Known install issue: pypilot_client.conf stale IP after fresh install

After installing pypilot on a Pi that was previously connected to another pypilot
instance (or moved to a different network), `~/.pypilot/pypilot_client.conf` may
contain a stale IP address.  pypilot will then try to connect to the wrong host
instead of its own local instance.

**Symptom:** pypilot log shows connection attempts to an IP that is not this Pi.
OTA server unreachable because the bridge also picked up the wrong host from a
similar config.

**Fix:**
```bash
echo '{"host":"127.0.0.1","port":23322}' > ~/.pypilot/pypilot_client.conf
sudo systemctl restart pypilot
```

This file is auto-generated by pypilot's zeroconf discovery and will be overwritten
again if zeroconf finds a remote pypilot.  After a fresh install, always check and
reset it before the first reboot.

The OTA server host in the bridge was separately fixed (auto-detected via
`_local_ip()`) and no longer depends on this file.

---

## Known outstanding issue: pypilot EBUSY on /dev/ttyINNOPILOT (PTY)

pypilot's servo subprocess cannot open `/dev/ttyINNOPILOT` (errno 16, EBUSY) after
the bridge or pypilot restarts.

**Root cause (diagnosed, not yet fixed):** socat holds the master side of the PTY
pair open continuously.  When any process sets `TIOCEXCL` (exclusive mode) on the
slave PTY and then closes it, the exclusive flag persists on the device as long as
socat holds the master open.  pypilot's servo subprocess retries the open in a loop
and never succeeds until the service is restarted.

**Impact:** The autopilot servo is not connected to pypilot's control loop after a
bridge restart without a full service restart.

**Workaround:** `sudo systemctl restart pypilot inno-pilot-bridge inno-pilot-socat`
followed by ~5 s wait.  A full system reboot also clears the condition.

**Status:** Unresolved.  The fix likely requires either (a) not setting TIOCEXCL on
the slave PTY, or (b) having socat recreate the PTY pair on restart.

---

## Known issue: pypilot_web Flask 2.3+ incompatibility (fixed in source)

`flask.Markup` was removed in Flask 2.3 and moved to `markupsafe`.  On Raspberry Pi
OS Bookworm (Flask 3.x), the original `web/web.py` crashes with:
```
ImportError: cannot import name 'Markup' from 'flask'
```
**Fix is in the source** (`compute_module/pypilot/web/web.py`) with a try/except
import guard.  When deploying to a new Pi, run `setup.py install` from the repo so
the fixed file is used.  If you install pypilot from upstream (not the inno-pilot
fork), you will need to patch `web/web.py` manually.

---

## Known gotcha: editing pypilot Python source needs a TWO-PASS `setup.py install`

**A single `setup.py install` does NOT deploy edits to pypilot's pure-Python
source** (e.g. `web/web.py`, `autopilot.py`, anything under `compute_module/pypilot/`).
Only the C extensions and `pypilot_data` get copied; the Python modules are silently
skipped, so the running service keeps using the **old installed copy**.

**Why:** `dependencies.py` (invoked inside `setup.py install`) clones `pypilot_data`,
which drops a `pyproject.toml` into `compute_module/pypilot/`.  On setuptools 78 /
Python 3.13 (Bookworm/Trixie) that `pyproject.toml` is treated as authoritative and
overrides `setup(packages=…)`, so `build_py` skips the pypilot package source.

**Symptom:** you edit `web/web.py` (or any pypilot module), deploy, the service
restarts fine and reports `active` — but the change isn't there (diff the installed
copy under `/usr/local/lib/python3.*/dist-packages/pypilot/` against the repo and it
still shows the old value).  This was discovered while iterating on `web/web.py`:
a single-pass redeploy kept silently running the previously-installed copy.

**Fix (already in place):** both `install.sh` (Phase 3) and `inno_deploy.sh`
(Step 4b) do a **two-pass** install — pass 1 fetches deps + builds C extensions,
then pass 2 bypasses `dependencies.py` (`rm -f pyproject.toml; touch deps`) so
`build_py` uses `setup(packages=…)` and actually installs the Python modules.
Verify pass 2 ran by looking for `copying web/web.py -> build/...` in the log.

**If you add another deploy/install path that touches pypilot**, replicate the
two-pass pattern or your pure-Python edits will not land.

---

## When uncertain
- Prefer asking a clarifying question in the PR description or as a comment rather than guessing.
- If you must assume: **state assumptions explicitly** and keep the change minimal.
