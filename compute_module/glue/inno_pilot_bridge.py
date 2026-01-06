#!/usr/bin/env python3
#
# Inno-Pilot bridge:
# - Forwards servo protocol between Nano and pypilot
# - Interprets BUTTON_EVENT_CODE frames from Nano
# - Uses pypilotClient API (watch/receive/set) to control ap.enabled / ap.heading_command

import time
import serial

from pypilot.client import pypilotClient  # uses compute_module/pypilot/pypilot/client.py

# Serial devices
NANO_PORT  = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0.real"  # real Nano servo
PILOT_PORT = "/dev/ttyINNOPILOT_BRIDGE"                               # side connected to pypilot via socat
BAUD       = 38400

DEBUG_NANO_STREAM = True  # set to False later if logs get too noisy

# Button event protocol from Nano (motor_simple.ino)
BUTTON_EVENT_CODE = 0xE0

BTN_EVT_NONE    = 0
BTN_EVT_MINUS10 = 1
BTN_EVT_MINUS1  = 2
BTN_EVT_TOGGLE  = 3
BTN_EVT_PLUS10  = 4
BTN_EVT_PLUS1   = 5

# Global autopilot client state
pilot_client = None
ap_enabled = None
ap_heading_command = None


def connect_pilot():
    """Connect to pypilot server and start watching key values."""
    global pilot_client, ap_enabled, ap_heading_command

    # Reset local cache when reconnecting
    ap_enabled = None
    ap_heading_command = None

    try:
        c = pypilotClient()  # defaults to localhost + DEFAULT_PORT
        if not c.connect(True):
            print("Inno-Pilot: failed to connect to pypilot server")
            pilot_client = None
            return False

        # Watch the values we care about.
        # True (or 0) means "send every update as soon as it changes".
        c.watch('ap.enabled', True)
        c.watch('ap.heading_command', True)

        pilot_client = c
        print("Inno-Pilot: connected to pypilot server and watching ap.enabled/ap.heading_command")
        return True
    except Exception as e:
        print("Inno-Pilot: error connecting to pypilot:", e)
        pilot_client = None
        return False


def pump_pilot():
    """
    Poll pypilot client and update local copies of values.

    This uses pypilotClient.receive(), which internally calls poll()
    to send any pending watch updates and pull new value updates.
    """
    global pilot_client, ap_enabled, ap_heading_command

    if not pilot_client or not pilot_client.connection:
        return

    try:
        msgs = pilot_client.receive(0)  # receive() calls poll(timeout)
        for name, value in msgs.items():
            if name == 'ap.enabled':
                # pypilot uses JSON booleans; value may be True/False already.
                ap_enabled = bool(value)
            elif name == 'ap.heading_command':
                # heading_command is a float in radians or degrees depending on mode;
                # here we just treat it as a float and apply relative deltas.
                try:
                    ap_heading_command = float(value)
                except (TypeError, ValueError):
                    ap_heading_command = None
    except Exception as e:
        print("Inno-Pilot: error polling pypilot:", e)


def handle_button_event(ev):
    """Translate Nano button events into pypilot AP actions via pypilotClient.set."""
    global pilot_client, ap_enabled, ap_heading_command

    if not pilot_client or not pilot_client.connection:
        print("Inno-Pilot: no pypilot connection; button event ignored:", ev)
        return

    # ----- AP toggle -----
    if ev == BTN_EVT_TOGGLE:
        if ap_enabled is None:
            print("Inno-Pilot: AP toggle ignored (ap.enabled unknown)")
            return

        new_val = not ap_enabled
        try:
            pilot_client.set('ap.enabled', new_val)
            ap_enabled = new_val
            print("Inno-Pilot: AP TOGGLE ->", new_val)
        except Exception as e:
            print("Inno-Pilot: error setting ap.enabled:", e)
        return

    # ----- Heading step commands -----
    if ev in (BTN_EVT_MINUS10, BTN_EVT_MINUS1, BTN_EVT_PLUS10, BTN_EVT_PLUS1):
        if ap_heading_command is None:
            print("Inno-Pilot: heading step ignored (ap.heading_command unknown)")
            return

        if   ev == BTN_EVT_MINUS10: delta = -10.0
        elif ev == BTN_EVT_MINUS1:  delta =  -1.0
        elif ev == BTN_EVT_PLUS1:   delta =   1.0
        elif ev == BTN_EVT_PLUS10:  delta =  10.0
        else:                       delta =   0.0

        new_heading = float(ap_heading_command) + delta
        try:
            pilot_client.set('ap.heading_command', new_heading)
            ap_heading_command = new_heading
            print(f"Inno-Pilot: heading += {delta} -> {new_heading}")
        except Exception as e:
            print("Inno-Pilot: error setting ap.heading_command:", e)
        return

    print("Inno-Pilot: unknown button event", ev)


def main():
    global pilot_client

    # Connect to pypilot once at startup; if it fails, we'll retry inside the loop.
    connect_pilot()

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
                # Keep pypilot values up to date (and reconnect if needed)
                if pilot_client and pilot_client.connection:
                    pump_pilot()
                else:
                    connect_pilot()

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

                        if DEBUG_NANO_STREAM:
                            print(
                                "Inno-Pilot: nano->pilot:",
                                " ".join(f"{b:02X}" for b in data_from_nano),
                            )

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
                                # CRC byte – ignore CRC, fire event
                                print("Inno-Pilot: button event from Nano:", btn_ev)
                                handle_button_event(btn_ev)
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
