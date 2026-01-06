# Inno-Pilot Glue – Pi Zero ↔ Nano ↔ pypilot

This directory contains the **Inno-Pilot-Glue**: everything that connects the
Inno-Pilot servo controller (Nano) to pypilot on the compute module (Pi Zero).

## What this glue does

- Keeps pypilot talking to the same **USB by-id** name it expects:
  `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
- Moves the original by-id symlink to:
  `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0.real`
- Creates a PTY pair using `socat`:
  - `/dev/ttyINNOPILOT`
  - `/dev/ttyINNOPILOT_BRIDGE`
- Repoints the by-id name to `/dev/ttyINNOPILOT` (the PTY)
- Runs a bridge process that:
  - Connects the real Nano USB (`...port0.real`) to `/dev/ttyINNOPILOT_BRIDGE`
  - Forwards all servo frames unchanged
  - Listens for `BUTTON_EVENT_CODE` (0xE0) frames from the Nano and translates
    them into pypilot API calls:
    - `ap.enabled` toggle
    - `ap.heading_command` ±1/±10 degrees

In short: pypilot still sees its normal servo device, but we have inserted a
transparent bridge in the middle that adds Inno-Pilot control head features.

## Files

- `inno_pilot_bridge.py`  
  Bridge between Nano and PTY. Forwards all traffic and handles button
  events via `pypilotClient`.

- `inno_pilot_fix_symlink.sh`  
  Moves the original by-id symlink to `.real` and points the original name at
  `/dev/ttyINNOPILOT`.

- `inno-pilot-socat.service`  
  systemd unit to create the PTY pair `/dev/ttyINNOPILOT` and
  `/dev/ttyINNOPILOT_BRIDGE`.

- `inno-pilot-fixlink.service`  
  systemd oneshot to run `inno_pilot_fix_symlink.sh` after socat starts.

- `inno-pilot-bridge.service`  
  systemd unit to run `inno_pilot_bridge.py` as user `aviljoen` (group `dialout`).

- `deploy_inno_pilot_glue.sh`  
  Deployment helper that copies scripts into `/usr/local/bin` and
  `/usr/local/sbin`, installs/updates the systemd units, and ensures
  `pypilot.service` starts after the glue.

- `AGENTS.md`  
  Guidance for future humans/agents on how to maintain this glue layer.

## Deployment (on the Pi Zero)

After updating the repo:

```bash
sudo systemctl stop pypilot pypilot_web
sudo systemctl stop inno-pilot-bridge inno-pilot-fixlink inno-pilot-socat || true

cd ~/inno-pilot
git pull origin master

./compute_module/glue/deploy_inno_pilot_glue.sh

sudo reboot
```

After reboot:

- inno-pilot-socat creates PTYs
- inno-pilot-fixlink adjusts the by-id symlink
- inno-pilot-bridge connects Nano ↔ PTY
- pypilot starts and uses the PTY via the by-id name
