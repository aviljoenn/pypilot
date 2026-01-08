#!/usr/bin/env python3
import time
import serial
from pypilot.client import pypilotClient  # watch/receive/set

# Serial devices
NANO_PORT  = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0.real"  # real Nano USB serial
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
    nano  = serial.Serial(NANO_PORT,  BAUD, timeout=0.01)
    pilot = serial.Serial(PILOT_PORT, BAUD, timeout=0.01)

    # Buffer for Nano->pilot frames
    frame_buf = bytearray()

    last_ap_sent = None
    last_ap_sent_ts = 0.0
    last_telem_ts = 0.0

    while True:
        now = time.monotonic()

        # ---- Pump pypilot client + update cached values ----
        msgs = client.receive(0)  # non-blocking
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
                nano.write(build_frame(AP_ENABLED_CODE, 1 if ap_enabled else 0))
                last_ap_sent = ap_enabled
                last_ap_sent_ts = now
                
        # heading (0xE2) and command (0xE3) in deg*10
        if ap_heading is not None:
            nano.write(build_frame(PILOT_HEADING_CODE, int(round(ap_heading * 10)) & 0xFFFF))
        
        if heading_cmd is not None:
            nano.write(build_frame(PILOT_COMMAND_CODE, int(round(heading_cmd * 10)) & 0xFFFF))
        
        # rudder angle (0xE4) signed deg*10
        if rudder_angle is not None:
            nano.write(build_frame(PILOT_RUDDER_CODE, int(round(rudder_angle * 10)) & 0xFFFF))
        
        # limits derived from rudder.range (0xE5 port, 0xE6 stbd)
        if rudder_range is not None:
            port_lim = abs(rudder_range)
            stbd_lim = -abs(rudder_range)
            nano.write(build_frame(PILOT_RUDDER_PORT_LIM_CODE, enc_deg10_i16(port_lim)))
            nano.write(build_frame(PILOT_RUDDER_STBD_LIM_CODE, enc_deg10_i16(stbd_lim)))
    
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
                nano.write(build_frame(PILOT_HEADING_CODE, deg10_heading(pilot_heading)))

            # ap.heading_command
            if heading_cmd is not None:
                nano.write(build_frame(PILOT_COMMAND_CODE, deg10_heading(heading_cmd)))

            # rudder.angle
            if pilot_rudder is not None:
                nano.write(build_frame(PILOT_RUDDER_CODE, s16_deg10(pilot_rudder)))



        
        # ---- Push heading/command/rudder telemetry ----
        if (now - last_telem_ts) >= TELEM_PERIOD_S:
            last_telem_ts = now

            if heading is not None:
                nano.write(build_frame(PILOT_HEADING_CODE, enc_deg10_u16(heading)))

            if heading_cmd is not None:
                nano.write(build_frame(PILOT_COMMAND_CODE, enc_deg10_u16(heading_cmd)))

            if rudder_angle is not None:
                nano.write(build_frame(PILOT_RUDDER_CODE, enc_deg10_i16(rudder_angle)))

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
