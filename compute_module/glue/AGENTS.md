# Inno-Pilot Glue – Agent Guide

This file is for **future maintainers** (humans or AI) working on the
Inno-Pilot glue on the Pi Zero.

## Principles

- The **repository is the source of truth**.
- Do **not** hand-edit:
  - `/usr/local/bin/inno_pilot_bridge.py`
  - `/usr/local/sbin/inno_pilot_fix_symlink.sh`
  - `/etc/systemd/system/inno-pilot-*.service`
  - `/etc/systemd/system/pypilot.service.d/override.conf`
- Instead:
  - Edit files under `compute_module/glue/` in the repo.
  - Commit and push to GitHub.
  - On the Pi, `git pull` and run the deploy script.

## Update / deployment workflow

When changing the glue (bridge logic, units, etc.):

1. **Edit in the repo**  
   Make changes under `compute_module/glue/` (this directory).

2. **Commit and push**  
   From your dev machine or the Pi:

   ```bash
   git add compute_module/glue
   git commit -m "Update Inno-Pilot glue"
   git push origin master
   ```

3. **To deploy on the Compute Module**  
   Run the following on the Pi:

   ```bash
   sudo systemctl stop pypilot pypilot_web
   sudo systemctl stop inno-pilot-bridge inno-pilot-fixlink inno-pilot-socat || true

   cd ~/inno-pilot
   git pull origin master

   ./compute_module/glue/deploy_inno_pilot_glue.sh

   sudo reboot
   ```
