#!/usr/bin/env python3
# Pypilot motor controller test console for Raspberry Pi
# Talks to the Arduino Nano running motor_IBT2_pypilot.ino
#
# - Opens a serial port to the Nano
# - Decodes and prints telemetry frames
# - Provides a menu to send all defined commands
#
# Requires: pip install pyserial

import sys
import time
import threading

try:
    import serial  # pyserial
except ImportError:
    print("This script requires the 'pyserial' package. Install with:")
    print("  pip install pyserial")
    sys.exit(1)

# ---------------- Protocol constants ----------------

# Serial settings (motor.ino uses Serial.begin(38400 * DIV_CLOCK), DIV_CLOCK=2)
DEFAULT_BAUD = 76800
DEFAULT_PORT_CANDIDATES = [
    "/dev/ttyUSB0",
    "/dev/ttyACM0",
    "/dev/serial0",
]

# CRC-8 configuration.
# NOTE: This assumes crc.h uses CRC-8 with polynomial 0x07 and init 0x00.
# If your Nano does not accept commands, check crc.h and adjust CRC_POLY/CRC_INIT.
CRC_POLY = 0x07
CRC_INIT = 0x00

# Command codes (from motor.ino)
COMMAND_CODE                 = 0xC7
RESET_CODE                   = 0xE7
MAX_CURRENT_CODE             = 0x1E
MAX_CONTROLLER_TEMP_CODE     = 0xA4
MAX_MOTOR_TEMP_CODE          = 0x5A
RUDDER_RANGE_CODE            = 0xB6  # not used by stock firmware
RUDDER_MIN_CODE              = 0x2B
RUDDER_MAX_CODE              = 0x4D
REPROGRAM_CODE               = 0x19
DISENGAGE_CODE               = 0x68
MAX_SLEW_CODE                = 0x71
EEPROM_READ_CODE             = 0x91
EEPROM_WRITE_CODE            = 0x53
CLUTCH_PWM_AND_BRAKE_CODE    = 0x36

# Result codes (from motor.ino)
CURRENT_CODE                 = 0x1C
VOLTAGE_CODE                 = 0xB3
CONTROLLER_TEMP_CODE         = 0xF9
MOTOR_TEMP_CODE              = 0x48
RUDDER_SENSE_CODE            = 0xA7
FLAGS_CODE                   = 0x8F
EEPROM_VALUE_CODE            = 0x9A

# Flag bits (from motor.ino)
FLAG_BITS = [
    (0x0001, "SYNC"),
    (0x0002, "OVERTEMP_FAULT"),
    (0x0004, "OVERCURRENT_FAULT"),
    (0x0008, "ENGAGED"),
    (0x0010, "INVALID"),
    (0x0020, "PORT_PIN_FAULT"),
    (0x0040, "STARBOARD_PIN_FAULT"),
    (0x0080, "BADVOLTAGE_FAULT"),
    (0x0100, "MIN_RUDDER_FAULT"),
    (0x0200, "MAX_RUDDER_FAULT"),
    (0x0400, "CURRENT_RANGE"),
    (0x0800, "BAD_FUSES"),
    (0x8000, "REBOOTED"),
]

# ---------------- CRC and framing helpers ----------------

def crc8(data: bytes, poly: int = CRC_POLY, init: int = CRC_INIT) -> int:
    """Compute CRC-8 over data (same algorithm family as motor.ino's crc.h)."""
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def make_frame(code: int, value: int) -> bytes:
    """Build a 4-byte command frame: [code, value_lo, value_hi, crc8]."""
    if not 0 <= code <= 0xFF:
        raise ValueError("code must be 0..255")
    if not 0 <= value <= 0xFFFF:
        raise ValueError("value must be 0..65535")
    lo = value & 0xFF
    hi = (value >> 8) & 0xFF
    body = bytes([code, lo, hi])
    c = crc8(body)
    return body + bytes([c])


# ---------------- Telemetry state ----------------

telemetry = {
    "flags": None,
    "current_raw": None,
    "current_A": None,
    "voltage_raw": None,
    "voltage_V": None,
    "ctrl_temp_raw": None,
    "ctrl_temp_C": None,
    "motor_temp_raw": None,
    "motor_temp_C": None,
    "rudder_raw": None,
    "rudder_normalized": None,
    "last_eeprom": None,  # (addr, value)
    "last_update": None,
}

print_lock = threading.Lock()
running_flag = {"run": True}


# ---------------- Serial reader thread ----------------

def decode_flags(value: int) -> str:
    if value is None:
        return "None"
    names = [name for bit, name in FLAG_BITS if value & bit]
    if names:
        return ", ".join(names)
    return "0"


def handle_frame(code: int, value: int) -> None:
    now = time.time()
    with print_lock:
        if code == FLAGS_CODE:
            telemetry["flags"] = value
            telemetry["last_update"] = now
            desc = decode_flags(value)
            print(f"[RX] FLAGS 0x{value:04X} [{desc}]")

        elif code == CURRENT_CODE:
            telemetry["current_raw"] = value
            telemetry["current_A"] = value / 100.0  # 10 mA units
            telemetry["last_update"] = now
            print(f"[RX] CURRENT {value/100.0:.2f} A (raw={value})")

        elif code == VOLTAGE_CODE:
            telemetry["voltage_raw"] = value
            telemetry["voltage_V"] = value / 100.0  # 10 mV units
            telemetry["last_update"] = now
            print(f"[RX] VOLTAGE {value/100.0:.2f} V (raw={value})")

        elif code == CONTROLLER_TEMP_CODE:
            telemetry["ctrl_temp_raw"] = value
            telemetry["ctrl_temp_C"] = value / 100.0
            telemetry["last_update"] = now
            print(f"[RX] CTRL TEMP {value/100.0:.2f} °C (raw={value})")

        elif code == MOTOR_TEMP_CODE:
            telemetry["motor_temp_raw"] = value
            telemetry["motor_temp_C"] = value / 100.0
            telemetry["last_update"] = now
            print(f"[RX] MOTOR TEMP {value/100.0:.2f} °C (raw={value})")

        elif code == RUDDER_SENSE_CODE:
            telemetry["rudder_raw"] = value
            if value == 0xFFFF:
                telemetry["rudder_normalized"] = None
                rdesc = "INVALID (0xFFFF)"
            else:
                telemetry["rudder_normalized"] = value / 65535.0
                rdesc = f"{telemetry['rudder_normalized']*100.0:.1f}% of ADC span"
            telemetry["last_update"] = now
            print(f"[RX] RUDDER raw=0x{value:04X} ({value}) -> {rdesc}")

        elif code == EEPROM_VALUE_CODE:
            addr = value & 0xFF
            val = (value >> 8) & 0xFF
            telemetry["last_eeprom"] = (addr, val)
            telemetry["last_update"] = now
            print(f"[RX] EEPROM addr=0x{addr:02X} value=0x{val:02X} ({val})")

        else:
            print(f"[RX] code=0x{code:02X} value=0x{value:04X} ({value})")


def reader_thread(port: serial.Serial) -> None:
    buf = bytearray()
    while running_flag["run"]:
        try:
            data = port.read(64)
        except serial.SerialException as e:
            with print_lock:
                print(f"[ERR] Serial error: {e}")
            break

        if not data:
            continue

        # NEW: show raw bytes as they arrive
        with print_lock:
            print(f"[RAW RX] {data.hex()}")

        buf.extend(data)

        # Sliding 4-byte window to find valid frames
        while len(buf) >= 4:
            code = buf[0]
            lo = buf[1]
            hi = buf[2]
            crc_rx = buf[3]
            crc_calc = crc8(buf[0:3])
            if crc_calc == crc_rx:
                value = lo | (hi << 8)
                handle_frame(code, value)
                del buf[:4]
            else:
                # shift window by one byte and try again
                del buf[0:1]


# ---------------- Command sending helpers ----------------

def send_frame(port: serial.Serial, code: int, value: int) -> None:
    frame = make_frame(code, value)
    with print_lock:
        print(f"[TX] code=0x{code:02X} value={value} -> {frame.hex()}")
    port.write(frame)
    port.flush()


def prompt_int(prompt: str, default=None, min_val=None, max_val=None) -> int:
    while True:
        txt = prompt
        if default is not None:
            txt += f" [{default}]"
        txt += ": "
        s = input(txt).strip()
        if not s and default is not None:
            return default
        try:
            v = int(s, 0)  # allow hex with 0x prefix
        except ValueError:
            print("Please enter a valid integer.")
            continue
        if min_val is not None and v < min_val:
            print(f"Value must be >= {min_val}")
            continue
        if max_val is not None and v > max_val:
            print(f"Value must be <= {max_val}")
            continue
        return v


def show_status() -> None:
    with print_lock:
        print("\n--- Latest telemetry ---")
        print(f"Flags     : {telemetry['flags']} (decoded: {decode_flags(telemetry['flags'])})")
        if telemetry["current_A"] is not None:
            print(f"Current   : {telemetry['current_A']:.2f} A (raw={telemetry['current_raw']})")
        if telemetry["voltage_V"] is not None:
            print(f"Voltage   : {telemetry['voltage_V']:.2f} V (raw={telemetry['voltage_raw']})")
        if telemetry["ctrl_temp_C"] is not None:
            print(f"Ctrl temp : {telemetry['ctrl_temp_C']:.2f} °C")
        if telemetry["motor_temp_C"] is not None:
            print(f"Motor temp: {telemetry['motor_temp_C']:.2f} °C")
        if telemetry["rudder_raw"] is not None:
            if telemetry["rudder_normalized"] is None:
                print(f"Rudder    : INVALID (raw=0x{telemetry['rudder_raw']:04X})")
            else:
                print(
                    f"Rudder    : raw=0x{telemetry['rudder_raw']:04X} "
                    f"({telemetry['rudder_raw']}) "
                    f"~ {telemetry['rudder_normalized']*100.0:.1f}% of full scale"
                )
        if telemetry["last_eeprom"] is not None:
            addr, val = telemetry["last_eeprom"]
            print(f"EEPROM    : last read addr=0x{addr:02X} value=0x{val:02X} ({val})")
        print("------------------------")

# ---------------- Main menu ----------------

MENU_TEXT = """
Commands:
 1) Send motor command (COMMAND_CODE, 0..2000, 1000 = neutral)
 2) DISENGAGE (release motor + clutch)
 3) RESET overcurrent fault
 4) Set MAX_CURRENT (in amps)
 5) Set MAX_CONTROLLER_TEMP (°C)
 6) Set MAX_MOTOR_TEMP (°C)
 7) Set MAX_SLEW (speed + slow)
 8) Set RUDDER_MIN to current rudder reading
 9) Set RUDDER_MAX to current rudder reading
10) Set RUDDER_MIN to explicit value
11) Set RUDDER_MAX to explicit value
12) Set CLUTCH_PWM and BRAKE flag
13) EEPROM READ range
14) EEPROM WRITE one byte
15) Send REPROGRAM (jump to bootloader)  ***DANGEROUS***
 s) Show latest telemetry snapshot
 q) Quit
"""


def main() -> None:
    # Pick serial port
    print("=== Pypilot motor controller test console ===")
    print("Known serial device guesses:")
    for p in DEFAULT_PORT_CANDIDATES:
        print(f"  - {p}")
    port_name = input(f"Enter serial port [default {DEFAULT_PORT_CANDIDATES[0]}]: ").strip()
    if not port_name:
        port_name = DEFAULT_PORT_CANDIDATES[0]

    try:
        ser = serial.Serial(port_name, DEFAULT_BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"Error opening {port_name}: {e}")
        sys.exit(1)


    print(f"Opened {ser.port} at {ser.baudrate} bps")
    print("Press Ctrl+C or choose 'q' to exit.")

    # Start reader thread
    t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t.start()

    try:
        while True:
            print(MENU_TEXT)
            choice = input("Select option: ").strip().lower()
            if choice == "q":
                break

            elif choice == "1":
                val = prompt_int(
                    "Enter command value (0..2000, 1000 = neutral)",
                    default=1000,
                    min_val=0,
                    max_val=2000,
                )
                send_frame(ser, COMMAND_CODE, val)

            elif choice == "2":
                send_frame(ser, DISENGAGE_CODE, 0)

            elif choice == "3":
                # Reset overcurrent fault
                send_frame(ser, RESET_CODE, 0)

            elif choice == "4":
                amps = prompt_int("Enter MAX_CURRENT in amps (e.g. 20)", default=20, min_val=0)
                value = int(amps * 100)  # 10 mA units
                send_frame(ser, MAX_CURRENT_CODE, value)

            elif choice == "5":
                temp_c = prompt_int("Enter MAX_CONTROLLER_TEMP in °C", default=70, min_val=0)
                value = int(temp_c * 100)  # hundredths of °C
                send_frame(ser, MAX_CONTROLLER_TEMP_CODE, value)

            elif choice == "6":
                temp_c = prompt_int("Enter MAX_MOTOR_TEMP in °C", default=70, min_val=0)
                value = int(temp_c * 100)
                send_frame(ser, MAX_MOTOR_TEMP_CODE, value)

            elif choice == "7":
                fast = prompt_int("Enter max_slew_speed (1..250)", default=50, min_val=1, max_val=250)
                slow = prompt_int("Enter max_slew_slow (1..250)", default=75, min_val=1, max_val=250)
                # Pack as bytes in low/high
                value = (slow << 8) | (fast & 0xFF)
                send_frame(ser, MAX_SLEW_CODE, value)

            elif choice == "8":
                # RUDDER_MIN to current reading
                rr = telemetry.get("rudder_raw")
                if rr is None:
                    print("No rudder reading yet. Wait for RUDDER_SENSE frames first.")
                else:
                    print(f"Using current rudder_raw={rr} (0x{rr:04X}) as RUDDER_MIN")
                    send_frame(ser, RUDDER_MIN_CODE, rr)

            elif choice == "9":
                rr = telemetry.get("rudder_raw")
                if rr is None:
                    print("No rudder reading yet. Wait for RUDDER_SENSE frames first.")
                else:
                    print(f"Using current rudder_raw={rr} (0x{rr:04X}) as RUDDER_MAX")
                    send_frame(ser, RUDDER_MAX_CODE, rr)

            elif choice == "10":
                rr = prompt_int("Enter explicit RUDDER_MIN raw value (0..65535)", min_val=0, max_val=65535)
                send_frame(ser, RUDDER_MIN_CODE, rr)

            elif choice == "11":
                rr = prompt_int("Enter explicit RUDDER_MAX raw value (0..65535)", min_val=0, max_val=65535)
                send_frame(ser, RUDDER_MAX_CODE, rr)

            elif choice == "12":
                pwm = prompt_int(
                    "Enter clutch PWM (0..255, <250 enables PWM hold)",
                    default=192,
                    min_val=0,
                    max_val=255,
                )
                brake = prompt_int("Use brake when stopped? (0 = no, 1 = yes)", default=0, min_val=0, max_val=1)
                value = (brake << 8) | (pwm & 0xFF)
                send_frame(ser, CLUTCH_PWM_AND_BRAKE_CODE, value)

            elif choice == "13":
                start = prompt_int("EEPROM READ start address (0..254)", default=0, min_val=0, max_val=254)
                end = prompt_int("EEPROM READ end address (0..255)", default=16, min_val=0, max_val=255)
                if end < start:
                    print("End must be >= start")
                else:
                    value = (end << 8) | (start & 0xFF)
                    send_frame(ser, EEPROM_READ_CODE, value)

            elif choice == "14":
                addr = prompt_int("EEPROM WRITE address (0..255)", min_val=0, max_val=255)
                val = prompt_int("EEPROM WRITE value (0..255)", min_val=0, max_val=255)
                value = (val << 8) | (addr & 0xFF)
                send_frame(ser, EEPROM_WRITE_CODE, value)

            elif choice == "15":
                print("WARNING: REPROGRAM_CODE will jump the Nano into the bootloader.")
                print("Only send this if you know what you are doing and are ready to reflash firmware.")
                confirm = input("Type 'YES' to confirm: ").strip()
                if confirm == "YES":
                    send_frame(ser, REPROGRAM_CODE, 0)
                else:
                    print("Cancelled.")

            elif choice == "s":
                show_status()

            elif choice == "":
                continue

            else:
                print("Unknown option.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        running_flag["run"] = False
        time.sleep(0.2)
        try:
            ser.close()
        except Exception:
            pass
        print("Exiting.")


if __name__ == "__main__":
    main()
