#!/usr/bin/env python3
import time
import serial
from pypilot.client import pypilotClient  # no .get() API; use watch/receive/set

# Serial devices
NANO_PORT  = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0.real"  # real Nano USB serial
PILOT_PORT = "/dev/ttyINNOPILOT_BRIDGE"                               # PTY side that talks to pypilot
BAUD       = 38400

# Nano -> Bridge button event protocol
BUTTON_EVENT_CODE = 0xE0
BTN_EVT_MINUS10 = 1
BTN_EVT_MINUS1  = 2
BTN_EVT_TOGGLE  = 3
BTN_EVT_PLUS10  = 4
BTN_EVT_PLUS1   = 5

# Bridge -> Nano state protocol
AP_ENABLED_CODE = 0xE1  # value: 0/1

# How often to refresh AP state to Nano (keeps Nano "online" even if servo frames are quiet)
AP_STATE_PERIOD_S = 0.5

def crc8_msb(data: bytes, poly: int = 0x31, init: int = 0xFF) -> int:
    """CRC-8 MSB-first, poly 0x31, init 0xFF (matches your Arduino crc8)."""
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ poly
            else:
                crc = (crc << 1) & 0xFF
    return crc & 0xFF

def build_frame(code: int, value: int) -> bytes:
    lo = value & 0xFF
    hi = (value >> 8) & 0xFF
    body = bytes([code, lo, hi])
    return body + bytes([crc8_msb(body)])

def clamp_heading(deg: float) -> float:
    # keep in 0..360 range
    deg = deg % 360.0
    if deg < 0:
        deg += 360.0
    return deg

def main():
    # pypilot TCP client (no direct get(); we watch and receive updates)
    client = pypilotClient()
    client.watch('ap.enabled', True)
    client.watch('ap.heading_command', True)

    ap_enabled = None
    heading_cmd = None

    # Open serial ports
    nano = serial.Serial(NANO_PORT, BAUD, timeout=0.01)
    pilot = serial.Serial(PILOT_PORT, BAUD, timeout=0.01)

    # Buffer for Nano->pilot frames
    frame_buf = bytearray()

    last_ap_sent = None
    last_ap_sent_ts = 0.0

    while True:
        now = time.monotonic()

        # ---- Pump pypilot client + update cached values ----
        msgs = client.receive(0)  # non-blocking
        if 'ap.enabled' in msgs:
            try:
                ap_enabled = bool(msgs['ap.enabled'])
            except Exception:
                pass

        if 'ap.heading_command' in msgs:
            try:
                heading_cmd = float(msgs['ap.heading_command'])
            except Exception:
                pass

        # ---- Push AP enabled state down to Nano (keepalive + on change) ----
        if ap_enabled is not None:
            need_send = (last_ap_sent is None) or (ap_enabled != last_ap_sent) or ((now - last_ap_sent_ts) >= AP_STATE_PERIOD_S)
            if need_send:
                frame = build_frame(AP_ENABLED_CODE, 1 if ap_enabled else 0)
                nano.write(frame)
                last_ap_sent = ap_enabled
                last_ap_sent_ts = now
                # optional debug:
                # print(f"Inno-Pilot: sent AP state to Nano: {int(ap_enabled)}")

        # ---- Data from pypilot -> Nano ----
        data_from_pilot = pilot.read(256)
        if data_from_pilot:
            nano.write(data_from_pilot)

        # ---- Data from Nano -> pypilot ----
        data_from_nano = nano.read(256)
        if data_from_nano:
            frame_buf.extend(data_from_nano)

            # Frames are always 4 bytes in this protocol
            while len(frame_buf) >= 4:
                f = bytes(frame_buf[:4])
                del frame_buf[:4]

                code = f[0]
                value = f[1] | (f[2] << 8)

                # forward raw frame to pypilot unchanged
                pilot.write(f)

                # handle button events
                if code == BUTTON_EVENT_CODE:
                    # print(f"Inno-Pilot: button event from Nano: {value}")
                    try:
                        if value == BTN_EVT_TOGGLE:
                            # Toggle ap.enabled using cached state (fallback: enable if unknown)
                            target = True if ap_enabled is None else (not ap_enabled)
                            client.set('ap.enabled', target)

                        elif value in (BTN_EVT_MINUS10, BTN_EVT_MINUS1, BTN_EVT_PLUS10, BTN_EVT_PLUS1):
                            if heading_cmd is None:
                                continue
                            delta = 0.0
                            if value == BTN_EVT_MINUS10: delta = -10.0
                            elif value == BTN_EVT_MINUS1: delta = -1.0
                            elif value == BTN_EVT_PLUS1: delta = 1.0
                            elif value == BTN_EVT_PLUS10: delta = 10.0

                            client.set('ap.heading_command', clamp_heading(heading_cmd + delta))

                    except Exception as e:
                        print("Inno-Pilot: button event handling error:", e)

        time.sleep(0.01)

if __name__ == "__main__":
    main()
