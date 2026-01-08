# Inno-Pilot Architecture

## What Inno-Pilot is
Inno-Pilot is a complete autopilot product made of:
- **Hardware** (power, sensors, actuator drive, buttons, display, enclosure)
- **Compute module** (currently Raspberry Pi Zero running the autopilot logic)
- **Servo/motor controller** (currently Arduino Nano controlling the hydraulic actuator + clutch)
- **Glue** (the integration layer that makes all parts behave as one product)

The autopilot logic is based on **Sean D’Epagnier’s pypilot** (used as an internal component). Inno-Pilot adds:
- hardware packaging + wiring conventions
- a dedicated servo controller design
- a button/display UX
- a “glue” layer to integrate buttons + telemetry without hacking upstream pypilot

Repository layout (high level):
- `hardware/` – physical build, wiring, modules, power, mounting
- `servo_motor_control/` – Arduino sketches & servo controller logic
- `compute_module/` – Pi Zero (or other SBC) setup, pypilot component, glue layer
  - `compute_module/pypilot/` – pypilot component (upstream-derived)
  - `compute_module/glue/` – Inno-Pilot-Glue: bridge + deploy scripts + services

---

## System components

### 1) Compute module (Pi Zero)
Role:
- runs **pypilot core** and **pypilot_web**
- interfaces to IMU (LSM9DS1 over I2C)
- interfaces to the servo controller (Arduino Nano over USB serial)
- hosts “glue” processes (bridge + PTY plumbing)

Operating requirements:
- can run **offline** (no Wi-Fi) and still steer in compass mode
- when network exists, can integrate with a central boat computer (SignalK/OpenCPN)

### 2) Servo/motor controller (Arduino Nano)
Role:
- drives IBT-2 H-bridge (hydraulic pump motor direction + enable)
- drives the **hydraulic clutch**
- reads the **rudder position pot**
- reads local sensors (12V bus V, current sensor, controller temperature)
- handles **local buttons**, OLED, and buzzer
- implements basic hard-stop safety at pot extremes (last-ditch protection)

### 3) Central navigation computer (optional)
Example in this build:
- `lysmarine` box hosts **SignalK + OpenCPN**
- autopilot subscribes to GPS/routes/wind/etc when enabled
- Inno-Pilot publishes its telemetry to SignalK (no cloud required)

---

## Data flow (runtime)

### Core servo protocol path (must be rock solid)
pypilot expects to talk to a “servo device” over a serial port using 4-byte CRC frames.

To preserve that, Inno-Pilot uses a PTY pair + a bridge:

pypilot <-> /dev/ttyINNOPILOT (PTY A)
|
| (socat creates PTY A <-> PTY B)
|
bridge <-> /dev/ttyINNOPILOT_BRIDGE (PTY B) <-> Arduino Nano (/dev/ttyUSB0)


### Why this “glue” exists
We want:
- **pypilot <-> Nano protocol** to stay unchanged (for compatibility and stability)
- **Nano buttons** to control pypilot’s **API** (ap.enabled, heading changes, etc.)
- **pypilot telemetry** (heading, command, rudder, limits) to appear on the Nano OLED

So the bridge:
- forwards frames **both directions** (transparent)
- listens for custom Nano button frames (0xE0) and converts them into pypilot API `set(...)`
- injects “state frames” into Nano (0xE1..0xE6) so OLED shows pypilot truth

---

## Boot sequence (systemd)
We want “power on → autopilot ready” without manual SSH sessions.

Target boot order:
1. `inno-pilot-socat.service` – creates PTY pair:
   - `/dev/ttyINNOPILOT`
   - `/dev/ttyINNOPILOT_BRIDGE`
2. `inno-pilot-fixlink.service` – redirects `/dev/serial/by-id/...` used by pypilot to point at `/dev/ttyINNOPILOT`
3. `inno-pilot-bridge.service` – starts bridge (Nano <-> PTY + API integration)
4. `pypilot.service` and `pypilot_web.service` – start pypilot after glue exists

---

## Safety model (high level)
Primary safety: pypilot controls rudder angle using its calibrated rudder model.
Secondary safety (Nano): absolute “never exceed” protection:
- if rudder pot approaches raw ADC extremes (near 0 or 1023), stop motor/clutch and raise fault
- if pypilot-reported rudder angle exceeds pypilot-reported limits beyond tolerance, stop and fault
- faults produce audible alarm + OLED message; PTM can silence/stop

---

## Update/deploy philosophy
- GitHub is the single source of truth
- On the Pi Zero: `git pull` + deploy script(s) + restart services
- Avoid hand-editing system files on the Pi (systemd units and scripts live in repo and are installed by deploy scripts)

