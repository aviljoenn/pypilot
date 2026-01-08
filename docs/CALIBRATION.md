# Inno-Pilot Calibration

This is a practical “do this, then that” calibration sequence.

## 0) Baseline checks (before calibration)
- Confirm services are running:
  - `inno-pilot-socat`
  - `inno-pilot-fixlink`
  - `inno-pilot-bridge`
  - `pypilot`
  - `pypilot_web`
- Confirm the servo device used by pypilot points to the by-id path:
  - `~/.pypilot/servodevice`
- Confirm Nano is reachable through the glue path:
  - pypilot should probe and find the servo device
- Confirm OLED is stable and readings update (Vin/I/Temp always bottom line)

---

## 1) IMU calibration (pypilot truth)
IMU: LSM9DS1 on I2C.

### Goals
- stable heading with low drift
- compass distortions resolved
- correct axis alignment

### Steps (high level)
- Use pypilot web UI calibration pages:
  - accelerate/gyro checks
  - compass calibration (full coverage)
  - alignment adjustments (imu.alignmentQ)
- Keep IMU away from magnetic sources as much as practical.

---

## 2) Rudder calibration (pypilot truth)
**Principle:** Inno-Pilot sails by pypilot’s calibrated rudder model. Nano does not “invent” rudder angles.

### Goals
- pypilot knows true rudder angle
- pypilot knows min/max angle (port/stbd limits)
- Nano displays pypilot port|actual|stbd (all from pypilot)

### Steps
1. Ensure rudder pot is wired correctly to Nano (A2) and stable.
2. In pypilot web UI:
   - run rudder calibration procedure (move rudder through range)
   - set/confirm `rudder.range` (e.g. 40°)
   - verify `rudder.offset`, `rudder.scale`, `rudder.nonlinearity`
3. Confirm the UI rudder angle matches physical reality.

### Nano “last-ditch” safety
The Nano should only hard-stop if:
- raw ADC approaches 0 or 1023 (pot wiring/calibration failure)
- pypilot-reported rudder exceeds pypilot-reported limits beyond tolerance

When tripped:
- stop motor + clutch
- raise fault
- show “Calibration required” message

---

## 3) Voltage calibration
### A0: main supply voltage
- adjust `VOLTAGE_SCALE` so OLED Vin matches multimeter at the supply input

### A3: Pi Zero 5V bus
- adjust `PI_VSENSE_SCALE` so OLED 5V bus matches multimeter on the Pi 5V rail

Fault thresholds:
- set `PI_VOLT_HIGH_FAULT` / `PI_VOLT_LOW_FAULT` to your real acceptable range
- verify fault alarm behaviour + PTM acknowledgement

---

## 4) Current sensor calibration
Your current sensor ADC response is not perfectly linear, so Inno-Pilot uses a calibration table (piecewise linear interpolation).

### Steps
1. Measure current with a multimeter at several loads.
2. Record corresponding ADC values on A1.
3. Update `CURR_ADC[]` and `CURR_A[]` arrays.
4. Verify the OLED current matches reality across low and higher currents.

---

## 5) Temperature calibration
- DS18B20 on D12
- confirm readings are plausible
- set `MAX_CONTROLLER_TEMP_C` to a conservative safe value (e.g. 50°C)
- verify overtemp fault stops motor/clutch and signals alarm

---

## 6) Button ladder calibration (A6)
- display raw ADC values for each button
- choose thresholds at midpoints between observed values
- confirm:
  - each button is stable (debounced)
  - no “ghost presses”

---

## 7) Save points (where “truth” lives)
- pypilot stores calibration in:
  - `~/.pypilot/pypilot.conf`
  - and `~/.pypilot/pypilot.conf.bak`
- Treat that file as a calibrated artifact (backup it).

