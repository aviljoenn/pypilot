# Inno-Pilot Pinouts (V2)

This document describes the **default V2** pinout. Some pins are optional (limit switches).

## Arduino Nano pinout

### Motor driver (IBT-2 / BTS7960)
- **D2**  → IBT-2 **RPWM** (direction)
- **D3**  → IBT-2 **LPWM** (direction)
- **D9**  → IBT-2 **R_EN + L_EN bridged** (enable / “PWM gate”)
  - In V2 hydraulic pump mode this is effectively **ON/OFF** (often fixed at 255)

IBT-2 power:
- IBT-2 V+ → 12V supply
- IBT-2 GND → 0V (must share common ground with Nano and sensors)
- IBT-2 motor outputs → hydraulic pump motor

### Clutch
- **D11** → clutch driver input
  - **HIGH = clutch engaged**
  - **LOW  = clutch disengaged**

### Emergency stop (PTM)
- **D4** → PTM button (wired to GND)
  - Configure as `INPUT_PULLUP`
  - Press = LOW

### Buzzer (via transistor)
- **D10** → base resistor (e.g. 2.2k) → NPN transistor base
- Transistor emitter → GND
- Transistor collector → buzzer negative
- Buzzer positive → +12V
- Nano GND must be tied to 12V GND

### OLED (I2C)
- **A4 (SDA)** → OLED SDA
- **A5 (SCL)** → OLED SCL
- OLED VCC → +5V
- OLED GND → GND

### DS18B20 (1-Wire)
- **D12** → DS18B20 DQ
- DS18B20 VDD → +5V
- DS18B20 GND → GND
- 4.7k pull-up from DQ to +5V

### Rudder position pot
- **A2** → pot wiper
- pot ends → +5V and GND (ensure stable reference)

### Analog sensors (ADC)
- **A0** → main supply voltage sense (via divider)
- **A1** → current sensor output
- **A3** → Pi Zero 5V bus sense (via divider)
- **A6** → 5-button resistor ladder input (analog-only pin)

### Optional limit switches (disabled by default in V2)
- **D7** → Port limit switch (NC to GND recommended), `INPUT_PULLUP`
- **D8** → Stbd limit switch (NC to GND recommended), `INPUT_PULLUP`

V2 default: **D7/D8 left free**, `LIMIT_SWITCHES_ACTIVE=false`.

---

## Pi Zero pinout (compute module)

### IMU (LSM9DS1)
- Uses Pi I2C bus (commonly `/dev/i2c-1`)
- LSM9DS1 typically appears at:
  - **0x6B** (accel/gyro)
  - **0x1E** (mag)
- Wiring:
  - Pi SDA/SCL → IMU SDA/SCL
  - Pi 3V3 → IMU VCC (confirm board voltage requirements)
  - Pi GND → IMU GND

### Servo controller (Nano)
- Connected via USB serial (CH340), e.g. `/dev/ttyUSB0`
- Inno-Pilot glue creates PTYs:
  - `/dev/ttyINNOPILOT`
  - `/dev/ttyINNOPILOT_BRIDGE`

---

## Wiring cautions (hard learned)
- **Common ground** is mandatory between:
  - Nano, IBT-2, clutch driver, current sensor, voltage dividers, Pi
- Avoid backfeeding +5V between devices unless you *really* know the power path.
- If you see Pi reboots when plugging Nano, treat it as **power integrity / backfeed** first, not software.

