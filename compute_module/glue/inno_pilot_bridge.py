#!/usr/bin/env python3
import time
import serial
from pypilot.client import pypilotClient

# Serial devices
# Real Nano USB port: original by-id symlink moved to .real
NANO_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0.real"
# PTY side connected to pypilot via socat
PILOT_PORT = "/dev/ttyINNOPILOT_BRIDGE"
BAUD = 38400

# Button event protocol from Nano (motor_simple.ino)
BUTTON_EVENT_CODE = 0xE0

BTN_EVT_NONE = 0
BTN_EVT_MINUS10 = 1
BTN_EVT_MINUS1 = 2
BTN_EVT_TOGGLE = 3
BTN_EVT_PLUS10 = 4
BTN_EVT_PLUS1 = 5


def handle_button_event(client, ev):
    """
    Translate Nano button events into pypilot AP actions.
    Nano encodes events as a 16-bit value:
      1: -10 degrees
      2: -1  degree
      3: AP toggle
      4: +10 degrees
      5: +1  degree
    """
    def get_heading():
        try:
            return client.get('ap.heading_command')
        except Exception:
            return None

    try:
        if ev == BTN_EVT_TOGGLE:
            enabled = client.get('ap.enabled')
            if enabled is None:
                return
            new_value = not bool(enabled)
            client.set('ap.enabled', new_value)
            print("Inno-Pilot: AP TOGGLE ->", new_value)

        elif ev in (BTN_EVT_MINUS10, BTN_EVT_MINUS1,
                    BTN_EVT_PLUS10, BTN_EVT_PLUS1):

            heading = get_heading()
            if heading is None:
                return

            if ev == BTN_EVT_MINUS10:
                delta = -10.0
            elif ev == BTN_EVT_MINUS1:
                delta = -1.0
            elif ev == BTN_EVT_PLUS1:
                delta = 1.0
            elif ev == BTN_EVT_PLUS10:
                delta = 10.0
            else:
                delta = 0.0

            new_heading = heading + delta
            client.set('ap.heading_command', new_heading)
            print(f"Inno-Pilot: heading += {delta} -> {new_heading}")

    except Exception as e:
        # Don't crash the bridge if pypilot is down; just log and continue
        print("Inno-Pilot: button event handling error:", e)


def main():
    # Connect to pypilot on localhost
    client = pypilotClient()

    while True:
        try:
            nano = serial.Serial(NANO_PORT, BAUD, timeout=0.01)
            pilot = serial.Serial(PILOT_PORT, BAUD, timeout=0.01)
            print("Inno-Pilot: bridge connected to Nano and PTY")

            # Simple state machine for scanning button frames in the Nano stream
            btn_state = 0
            btn_lo = 0
            btn_ev = 0

            while True:
                # ----- Data from pypilot -> Nano -----
                try:
                    data_from_pilot = pilot.read(64)
                    if data_from_pilot:
                        nano.write(data_from_pilot)
                except Exception as e:
                    print("Inno-Pilot: error reading from pypilot side:", e)
                    time.sleep(0.5)

                # ----- Data from Nano -> pypilot -----
                try:
                    data_from_nano = nano.read(64)
                    if data_from_nano:
                        # Always forward raw bytes unchanged
                        pilot.write(data_from_nano)

                        # Scan Nano stream for BUTTON_EVENT_CODE frames
                        for b in data_from_nano:
                            if btn_state == 0:
                                if b == BUTTON_EVENT_CODE:
                                    btn_state = 1
                            elif btn_state == 1:
                                # low byte of event
                                btn_lo = b
                                btn_state = 2
                            elif btn_state == 2:
                                # high byte of event
                                btn_ev = btn_lo | (b << 8)
                                btn_state = 3
                            elif btn_state == 3:
                                # CRC byte for the button frame – ignore CRC, fire event
                                print("Inno-Pilot: button event from Nano:", btn_ev)
                                handle_button_event(client, btn_ev)
                                btn_state = 0

                except Exception as e:
                    print("Inno-Pilot: error reading from Nano side:", e)
                    time.sleep(0.5)

                time.sleep(0.01)

        except Exception as e:
            print("Inno-Pilot: bridge failed to open ports:", e)
            time.sleep(2.0)  # backoff and retry


if __name__ == "__main__":
    main()
