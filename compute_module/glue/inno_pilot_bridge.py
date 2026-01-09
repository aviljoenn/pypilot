#!/usr/bin/env python3
import os
import time
import serial
from pypilot.client import pypilotClient  # watch/receive/set

# Serial devices
NANO_PORT  = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0.real"  # real Nano USB serial (fallback)
PILOT_PORT = "/dev/ttyINNOPILOT_BRIDGE"                               # PTY side that talks to pypilot
BAUD       = 38400

# Bridge -> Nano state protocol
AP_ENABLED_CODE = 0xE1  # value: 0/1
# Bridge -> Nano telemetry (pypilot truth)
PILOT_HEADING_CODE     = 0xE2  # imu.heading * 10  (uint16)
PILOT_COMMAND_CODE     = 0xE3  # ap.heading_command * 10 (uint16)
PILOT_RUDDER_CODE      = 0xE4  # rudder.angle * 10 (int16 -> two's complement in uint16)
PILOT_RUDDER_PORT_CODE = 0xE5  # +rudder.range * 10 (int16)
PILOT_RUDDER_STBD_CODE = 0xE6  # -rudder.range * 10 (int16)
# Optional: send pypilot-calibrated rudder limits too
PILOT_RUDDER_PORT_LIM_CODE = 0xE5  # +limit * 10 (signed int16)
PILOT_RUDDER_STBD_LIM_CODE = 0xE6  # -limit * 10 (signed int16)

# Nano -> Bridge button event protocol
BUTTON_EVENT_CODE = 0xE0
BTN_EVT_MINUS10 = 1
BTN_EVT_MINUS1  = 2
BTN_EVT_TOGGLE  = 3
BTN_EVT_PLUS10  = 4
BTN_EVT_PLUS1   = 5

# Bridge -> Nano telemetry frames (all are 4-byte frames with crc)
AP_ENABLED_CODE     = 0xE1  # value: 0/1
PILOT_HEADING_CODE  = 0xE2  # imu.heading * 10  (uint16)
PILOT_COMMAND_CODE  = 0xE3  # ap.heading_command * 10 (uint16)
PILOT_RUDDER_CODE   = 0xE4  # rudder.angle * 10 (int16 in two's complement)

# How often to refresh telemetry to Nano
AP_STATE_PERIOD_S = 0.5
TELEM_PERIOD_S    = 0.2  # heading/command/rudder

# Bridge <-> Nano framing (camouflage to prevent direct pypilot probe)
BRIDGE_MAGIC1 = 0xA5
BRIDGE_MAGIC2 = 0x5A
BRIDGE_HELLO_CODE = 0xF0
BRIDGE_HELLO_ACK_CODE = 0xF1
BRIDGE_HELLO_VALUE = 0xBEEF
PROBE_INITIAL_DELAY_S = 1.0
PROBE_RETRIES = 5
PROBE_RETRY_DELAY_S = 3.0

def crc8_msb(data: bytes, poly: int = 0x31, init: int = 0xFF) -> int:
    """CRC-8 MSB-first, poly 0x31, init 0xFF (matches Arduino crc8)."""
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ poly
            else:
                crc = (crc << 1) & 0xFF
    return crc & 0xFF

def build_frame(code: int, value_u16: int) -> bytes:
    lo = value_u16 & 0xFF
    hi = (value_u16 >> 8) & 0xFF
    body = bytes([code, lo, hi])
    return body + bytes([crc8_msb(body)])

def wrap_frame(frame: bytes) -> bytes:
    return bytes([BRIDGE_MAGIC1, BRIDGE_MAGIC2]) + frame

def send_nano_frame(nano: serial.Serial, code: int, value_u16: int) -> None:
    nano.write(wrap_frame(build_frame(code, value_u16)))

def extract_wrapped_frames(buf: bytearray) -> list[bytes]:
    frames: list[bytes] = []
    while len(buf) >= 2:
        if buf[0] != BRIDGE_MAGIC1 or buf[1] != BRIDGE_MAGIC2:
            del buf[0]
            continue
        if len(buf) < 6:
            break
        frame = bytes(buf[2:6])
        del buf[:6]
        frames.append(frame)
    return frames

def probe_nano_port(port: str, timeout_s: float = 0.5) -> bool:
    try:
        with serial.Serial(port, BAUD, timeout=0.1) as probe:
            time.sleep(PROBE_INITIAL_DELAY_S)
            for attempt in range(PROBE_RETRIES):
                probe.reset_input_buffer()
                probe.write(wrap_frame(build_frame(BRIDGE_HELLO_CODE, BRIDGE_HELLO_VALUE)))
                buf = bytearray()
                deadline = time.monotonic() + timeout_s
                while time.monotonic() < deadline:
                    chunk = probe.read(64)
                    if chunk:
                        buf.extend(chunk)
                        for frame in extract_wrapped_frames(buf):
                            code = frame[0]
                            value = frame[1] | (frame[2] << 8)
                            if code == BRIDGE_HELLO_ACK_CODE and value == BRIDGE_HELLO_VALUE:
                                return True
                if attempt < (PROBE_RETRIES - 1):
                    time.sleep(PROBE_RETRY_DELAY_S)
            return False
    except Exception:
        return False

def find_nano_port() -> str:
    by_id = "/dev/serial/by-id"
    candidates: list[str] = []
    if os.path.isdir(by_id):
        for entry in sorted(os.listdir(by_id)):
            path = os.path.join(by_id, entry)
            if os.path.islink(path):
                if os.path.realpath(path) == "/dev/ttyINNOPILOT":
                    continue
                candidates.append(path)
    if NANO_PORT not in candidates and os.path.exists(NANO_PORT):
        candidates.append(NANO_PORT)

    for candidate in candidates:
        if probe_nano_port(candidate):
            return candidate

    raise RuntimeError(f"Unable to find Nano on serial ports: {candidates}")

def to_u16_signed(v: int) -> int:
    return v & 0xFFFF

def clamp_heading(deg: float) -> float:
    deg = deg % 360.0
    if deg < 0:
        deg += 360.0
    return deg

def enc_deg10_u16(deg: float) -> int:
    """0..360 deg -> 0..3600 (uint16)."""
    d = clamp_heading(float(deg))
    v = int(round(d * 10.0))
    if v < 0: v = 0
    if v > 3600: v = 3600
    return v & 0xFFFF

def enc_deg10_i16(deg: float) -> int:
    """Signed degrees -> int16 in tenths."""
    v = int(round(float(deg) * 10.0))
    if v < -32768: v = -32768
    if v >  32767: v =  32767
    return v & 0xFFFF  # two's complement for Nano side

def main():
    # pypilot TCP client
    client = pypilotClient()

    # pypilot truth we want on the Nano
    client.watch('ap.enabled', True)
    client.watch('imu.heading', True)
    client.watch('ap.heading_command', True)
    client.watch('rudder.angle', True)
    # Periodic watch so updated calibration shows up without a restart.
    client.watch('rudder.range', 1.0)
    client.watch('ap.heading', True)

    ap_enabled = None
    ap_heading = None
    heading_cmd = None
    heading = None
    rudder_angle = None
    imu_heading  = None
    rudder_range = None
    pilot_heading = None
    pilot_rudder  = None

    # Open serial ports
    nano_port = find_nano_port()
    nano  = serial.Serial(nano_port,  BAUD, timeout=0.01)
    pilot = serial.Serial(PILOT_PORT, BAUD, timeout=0.01)

    # Buffers for framed transport
    nano_buf = bytearray()
    pilot_buf = bytearray()

    last_ap_sent = None
    last_ap_sent_ts = 0.0
    last_telem_ts = 0.0

    while True:
        now = time.monotonic()

        # ---- Pump pypilot client + update cached values ----
        msgs = client.receive(0)  # non-blocking
        if msgs:
            print(f"[pypilot] {msgs}")
        if "ap.enabled" in msgs:
            try:
                ap_enabled = bool(msgs["ap.enabled"])
            except Exception:
                pass
                
        if 'ap.heading' in msgs:
            try:
                ap_heading = float(msgs['ap.heading'])
            except Exception:
                pass
                
        if "ap.heading_command" in msgs:
            try:
                heading_cmd = float(msgs["ap.heading_command"])
            except Exception:
                pass

        if "imu.heading" in msgs:
            try:
                heading = float(msgs["imu.heading"])
            except Exception:
                pass

        if "rudder.angle" in msgs:
            try:
                rudder_angle = float(msgs["rudder.angle"])
            except Exception:
                pass

        if 'rudder.range' in msgs:
            try: 
                rudder_range = float(msgs['rudder.range'])
            except Exception: 
                pass

        # ---- Push AP enabled state down to Nano (keepalive + on change) ----
        if ap_enabled is not None:
            need_send = (last_ap_sent is None) or (ap_enabled != last_ap_sent) or ((now - last_ap_sent_ts) >= AP_STATE_PERIOD_S)
            if need_send:
                send_nano_frame(nano, AP_ENABLED_CODE, 1 if ap_enabled else 0)
                last_ap_sent = ap_enabled
                last_ap_sent_ts = now
                
        # heading (0xE2) and command (0xE3) in deg*10
        if ap_heading is not None:
            send_nano_frame(nano, PILOT_HEADING_CODE, int(round(ap_heading * 10)) & 0xFFFF)
        
        if heading_cmd is not None:
            send_nano_frame(nano, PILOT_COMMAND_CODE, int(round(heading_cmd * 10)) & 0xFFFF)
        
        # rudder angle (0xE4) signed deg*10
        if rudder_angle is not None:
            send_nano_frame(nano, PILOT_RUDDER_CODE, int(round(rudder_angle * 10)) & 0xFFFF)
        
        # limits derived from rudder.range (0xE5 port, 0xE6 stbd)
        if rudder_range is not None:
            port_lim = abs(rudder_range)
            stbd_lim = -abs(rudder_range)
            send_nano_frame(nano, PILOT_RUDDER_PORT_LIM_CODE, enc_deg10_i16(port_lim))
            send_nano_frame(nano, PILOT_RUDDER_STBD_LIM_CODE, enc_deg10_i16(stbd_lim))
    
        # ---- Push extra telemetry down to Nano (periodic) ----
        # Keep this lightweight; we send at most once per loop tick here.
        # If values are None, we just skip that frame.
        if 'last_telem_ts' not in locals():
            last_telem_ts = 0.0

        TELEM_PERIOD_S = 0.2  # 5 Hz
        if (now - last_telem_ts) >= TELEM_PERIOD_S:
            last_telem_ts = now

            # helper packers
            def u16(v: int) -> int:
                return max(0, min(65535, int(v)))

            def deg10_heading(x: float) -> int:
                # Normalize to 0..360 and scale by 10
                x = x % 360.0
                if x < 0:
                    x += 360.0
                return u16(round(x * 10.0))

            def s16_deg10(x: float) -> int:
                # signed int16 *10, returned as uint16 two's complement
                v = int(round(x * 10.0))
                if v < -32768: v = -32768
                if v >  32767: v =  32767
                return v & 0xFFFF

            # imu.heading
            if pilot_heading is not None:
                send_nano_frame(nano, PILOT_HEADING_CODE, deg10_heading(pilot_heading))

            # ap.heading_command
            if heading_cmd is not None:
                send_nano_frame(nano, PILOT_COMMAND_CODE, deg10_heading(heading_cmd))

            # rudder.angle
            if pilot_rudder is not None:
                send_nano_frame(nano, PILOT_RUDDER_CODE, s16_deg10(pilot_rudder))



        
        # ---- Push heading/command/rudder telemetry ----
        if (now - last_telem_ts) >= TELEM_PERIOD_S:
            last_telem_ts = now

            if heading is not None:
                send_nano_frame(nano, PILOT_HEADING_CODE, enc_deg10_u16(heading))

            if heading_cmd is not None:
                send_nano_frame(nano, PILOT_COMMAND_CODE, enc_deg10_u16(heading_cmd))

            if rudder_angle is not None:
                send_nano_frame(nano, PILOT_RUDDER_CODE, enc_deg10_i16(rudder_angle))

        # ---- Data from pypilot -> Nano ----
        data_from_pilot = pilot.read(256)
        if data_from_pilot:
            pilot_buf.extend(data_from_pilot)
            while len(pilot_buf) >= 4:
                raw_frame = bytes(pilot_buf[:4])
                del pilot_buf[:4]
                nano.write(wrap_frame(raw_frame))

        # ---- Data from Nano -> pypilot ----
        try:
            data_from_nano = nano.read(256)
        except serial.serialutil.SerialException as exc:
            print("ERROR: Nano serial read failed.")
            print(f"Exception: {exc}")
            print(f"NANO_PORT: {NANO_PORT}")
            print(f"PILOT_PORT: {PILOT_PORT}")
            print(f"BAUD: {BAUD}")
            print(f"Nano is_open: {getattr(nano, 'is_open', None)}")
            print(f"Nano in_waiting: {getattr(nano, 'in_waiting', None)}")
            print(f"Pilot is_open: {getattr(pilot, 'is_open', None)}")
            print(f"Pilot in_waiting: {getattr(pilot, 'in_waiting', None)}")
            raise SystemExit(1) from exc
        if data_from_nano:
            nano_buf.extend(data_from_nano)

            for f in extract_wrapped_frames(nano_buf):
                code = f[0]
                value = f[1] | (f[2] << 8)

                # forward raw frame to pypilot unchanged
                pilot.write(f)

                # handle button events
                if code == BUTTON_EVENT_CODE:
                    try:
                        if value == BTN_EVT_TOGGLE:
                            target = True if ap_enabled is None else (not ap_enabled)
                            client.set("ap.enabled", target)
                            ap_enabled = target  # optimistic local cache

                        elif value in (BTN_EVT_MINUS10, BTN_EVT_MINUS1, BTN_EVT_PLUS10, BTN_EVT_PLUS1):
                            if heading_cmd is None:
                                continue
                            delta = 0.0
                            if   value == BTN_EVT_MINUS10: delta = -10.0
                            elif value == BTN_EVT_MINUS1:  delta = -1.0
                            elif value == BTN_EVT_PLUS1:   delta =  1.0
                            elif value == BTN_EVT_PLUS10:  delta = 10.0

                            new_cmd = clamp_heading(heading_cmd + delta)
                            client.set("ap.heading_command", new_cmd)
                            heading_cmd = new_cmd  # optimistic cache

                    except Exception as e:
                        print("Inno-Pilot: button event handling error:", e, flush=True)

        time.sleep(0.01)

if __name__ == "__main__":
    main()
