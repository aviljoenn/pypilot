// motor_simple.ino
// Minimal pypilot-compatible motor controller core
// - 16 MHz Nano, Serial @ 38400
// - Uses crc.h (CRC-8 poly 0x31, init 0xFF)
// - Receives framed 4-byte packets: [MAGIC1, MAGIC2, code, value_lo, value_hi, crc]
// - Sends periodic FLAGS and RUDDER frames with the same framing
// - Drives IBT-2 H-bridge + clutch based on COMMAND / DISENGAGE
// - Obeys limit switches + rudder pot min/max
// - B26: MOTOR_REASON_CODE diagnostic — sent once when D9 goes LOW→HIGH, encodes which
//        code branch activated the motor plus key state variables for root-cause tracing.
// - B27: FEATURE_ON_BOARD_BUTTONS (0x20) — gate button ADC read behind this flag to
//        prevent floating A6 crosstalk from falsely activating motor in HAND mode.

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include "crc.h"    // your existing CRC-8 table + crc8()

enum ButtonID : uint8_t;

// ---- Inno-Pilot version (must match bridge + remote) ----
const char INNOPILOT_VERSION[] = "v1.3.3_B5";
const uint16_t INNOPILOT_BUILD_NUM = 5;  // increment with each push during development

// Boot / online timing (user-tweakable)
bool ap_enabled_remote = false;        // true when AP engaged (set by COMMAND_CODE, cleared by DISENGAGE_CODE)
bool ap_display        = false;        // what OLED shows (can be briefly “optimistic”)
unsigned long ap_display_override_until_ms = 0;
const unsigned long PI_BOOT_EST_MS    = 98000UL;  // 60s estimate, tweak later
const unsigned long ONLINE_SPLASH_MS  = 3000UL;   // 3s "On-line" splash
bool pi_online_at_boot = false;

// Bridge->Nano: extra telemetry (tenths of degrees)
const uint8_t PILOT_HEADING_CODE = 0xE2; // ap.heading * 10 (uint16)
const uint8_t PILOT_COMMAND_CODE = 0xE3; // ap.heading_command * 10 (uint16)
const uint8_t PILOT_RUDDER_CODE  = 0xE4; // rudder.angle * 10 (int16, two's complement)
// Bridge framing + handshake (prevents direct pypilot probe binding to Nano)
const uint8_t BRIDGE_MAGIC1 = 0xA5;
const uint8_t BRIDGE_MAGIC2 = 0x5A;
const uint8_t BRIDGE_HELLO_CODE = 0xF0;
const uint8_t BRIDGE_HELLO_ACK_CODE = 0xF1;
const uint8_t BRIDGE_VERSION_CODE = 0xF2;  // Bridge -> Nano: build number
// Bridge->Nano: pypilot rudder limits (tenths of degrees)
const uint8_t PILOT_RUDDER_DIRB_LIM_CODE = 0xE5; // Dir-B limit * 10 (int16)
const uint8_t PILOT_RUDDER_DIRA_LIM_CODE = 0xE6; // Dir-A limit * 10 (int16)
// Remote manual control (Bridge->Nano)
const uint8_t MANUAL_RUD_TARGET_CODE = 0xE8; // remote rudder target 0..1000
const uint8_t MANUAL_MODE_CODE       = 0xE9; // 1=enter MANUAL, 0=exit MANUAL
// Warnings from bridge (Bridge->Nano)
const uint8_t WARNING_CODE           = 0xEA; // warning subtype
const uint8_t WARN_NONE              = 0;
const uint8_t WARN_AP_PRESSED        = 1;    // AP button pressed while remote not in MANUAL
const uint8_t WARN_STEER_LOSS        = 2;    // TCP dropped in MANUAL mode
// Nano->Bridge: buzzer state reporting
const uint8_t BUZZER_STATE_CODE      = 0xEB; // 1=buzzer on, 0=off

// Nano->Bridge: H-bridge pin state change (diagnostic, on-change only)
// value bits: [2]=D9/EN(PWM)  [1]=D3/LPWM  [0]=D2/RPWM
// Sent every time any of these three pins changes after update_motor_from_command().
const uint8_t PIN_STATE_CODE = 0xE1;

// Nano->Bridge: motor activation reason (B26 diagnostic).
// Sent ONCE when D9 transitions LOW->HIGH (motor first starts).
// value bits [3:0]: reason code
//   1 = manual_override (physical button jog on Nano)
//   2 = delta_jog (stale COMMAND: !ap_active && command_recent && |delta|>DEADBAND)
//   3 = ap_active (autopilot COMMAND driving motor)
//   4 = remote_manual RM_DRIVING (TCP remote manual mode, driving)
//   5 = remote_manual RM_BRAKING (TCP remote manual mode, reverse-brake pulse)
// value bit 4: ap_enabled_remote at time of activation
// value bit 5: command_recent at time of activation
// value bit 6: pi_alive at time of activation
// value bit 7: manual_override (physical button state) at time of activation
// value bits [15:8]: last_command_val >> 3  (≈0-250; 125 = neutral 1000; map back: ×8)
const uint8_t MOTOR_REASON_CODE = 0xEE;

// Reason codes for MOTOR_REASON_CODE
const uint8_t MRSN_MANUAL_PHYS = 1;  // manual_override=true (physical button)
const uint8_t MRSN_DELTA_JOG   = 2;  // stale command delta path
const uint8_t MRSN_AP          = 3;  // ap_active autopilot drive
const uint8_t MRSN_RM_DRIVE    = 4;  // remote manual RM_DRIVING
const uint8_t MRSN_RM_BRAKE    = 5;  // remote manual RM_BRAKING

// Pending reason value: set by update_motor_from_command() before each activation,
// consumed by loop() when D9 transitions LOW->HIGH.
uint16_t g_motor_reason = 0;

// Bridge->Nano: feature enable bitmask (0xEF)
// Sent by bridge on startup and on every settings change.
const uint8_t FEATURES_CODE          = 0xEF; // Bridge->Nano: uint8 feature bitmask
const uint8_t FEATURE_LIMIT_SWITCHES  = 0x01; // use D7/D8 NC limit switches
const uint8_t FEATURE_TEMP_SENSOR     = 0x02; // DS18B20 overtemp fault detection
const uint8_t FEATURE_PI_VOLTAGE      = 0x04; // A3 Pi supply voltage fault detection
const uint8_t FEATURE_BATTERY_VOLTAGE = 0x08; // A0 main Vin over/under-voltage faults
const uint8_t FEATURE_CURRENT_SENSOR  = 0x10; // A1 current sensor telemetry
const uint8_t FEATURE_ON_BOARD_BUTTONS = 0x20; // physical button ladder on A6 is wired
const uint8_t FEATURE_OLED_SH1106     = 0x40; // OLED uses SH1106 controller (132-col, offset 2)
const uint8_t FEATURE_INVERT_CLUTCH   = 0x80; // Clutch relay is active-LOW (invert pin 11 logic)

// Second feature-flags byte — EEPROM offset 23 (layout v2).
// Not sent via FEATURES_CODE at runtime; EEPROM-only until logic is wired in.
const uint8_t FEATURE2_INVERT_MOTOR   = 0x01; // Invert H-bridge direction (storage only for now)

// ---- EEPROM settings layout ----
const uint8_t EEPROM_MAGIC1          = 0xAA;
const uint8_t EEPROM_MAGIC2          = 0x55;
const uint8_t EEPROM_LAYOUT_VERSION  = 2;

// ---- arduino_servo EEPROM mirror (B47) ----
// 32-byte opaque region that mirrors the arduino_servo_data struct as serialised
// by the Pi-side arduino_servo driver (ARM gcc layout, includes 1 padding byte
// before rudder_offset so that int16_t is 2-byte aligned).
// Key struct byte offsets (ARM-padded):
//   8-9:  rudder_offset      (int16_t, tobase255s(v × 64))
//   10-11: rudder_scale      (int16_t, tobase255s(v × 8))
//   12-13: rudder_nonlinearity (int16_t, tobase255s(v × 8))
//   25-30: signature "arsv27" (validity sentinel)
// Motor_simple does not decode these fields — it stores/returns raw bytes.
// On first boot (EEPROM = 0xFF) the signature check fails inside arduino_servo,
// which then pushes the live calibration via EEPROM_WRITE_CODE before succeeding.
const uint16_t ASRV_EEPROM_BASE = 180;
const uint8_t  ASRV_EEPROM_SIZE = 32;

// Cached telemetry from pypilot (for OLED)
bool     pilot_heading_valid = false;
uint16_t pilot_heading_deg10 = 0;

bool     pilot_command_valid = false;
uint16_t pilot_command_deg10 = 0;

bool     pilot_rudder_valid  = false;
int16_t  pilot_rudder_deg10  = 0;

// Rudder limits from pypilot (tenths of degrees)
bool     pilot_dirb_lim_valid = false;
int16_t  pilot_dirb_lim_deg10 = 0;

bool     pilot_dira_lim_valid = false;
int16_t  pilot_dira_lim_deg10 = 0;

bool any_serial_rx = false;
unsigned long last_serial_rx_ms = 0;
bool pi_online           = false;    // true once we see first valid frame
unsigned long boot_start_ms    = 0;  // reference after splash
unsigned long splash_until_ms  = 0;  // non-blocking 3 s boot splash timer (0 = not active)
unsigned long pi_online_time_ms = 0; // when we first saw Pi online
// NEW: track if Pi was ever online and when we last heard from it
bool pi_ever_online       = false;
unsigned long last_pi_frame_ms = 0;
const unsigned long PI_OFFLINE_TIMEOUT_MS = 5000UL;  // 5s no frames => offline
uint16_t bridge_build_num = 0;     // received from bridge via BRIDGE_VERSION_CODE
bool     bridge_build_valid = false;

// ---- Pins ----
const uint8_t LED_PIN          = 13;
const uint8_t RUDDER_PIN       = A2;

const uint8_t PTM_PIN          = 4;
const unsigned long PTM_DEBOUNCE_MS = 30;
const uint8_t BUZZER_PIN       = 10;

const uint8_t PIN_DS18B20      = 12;
const uint8_t PIN_PI_VSENSE    = A3;
const uint8_t PIN_VOLTAGE      = A0;
const uint8_t PIN_CURRENT      = A1;
const uint8_t BUTTON_ADC_PIN   = A6;   // analog-only pin for button ladder

// IBT-2 (BTS7960) pins
const uint8_t HBRIDGE_RPWM_PIN = 2;   // RPWM
const uint8_t HBRIDGE_LPWM_PIN = 3;   // LPWM
const uint8_t HBRIDGE_PWM_PIN  = 9;   // EN (R_EN + L_EN tied together)

// Clutch pin. Default active-HIGH: HIGH = engaged, LOW = disengaged.
// Set FEATURE_INVERT_CLUTCH to reverse for active-LOW relay wiring.
const uint8_t CLUTCH_PIN       = 11;

// Limit switches (NC -> GND, HIGH = tripped / broken)
const uint8_t DIRB_LIMIT_PIN   = 7;   // Dir-B end limit switch
const uint8_t DIRA_LIMIT_PIN   = 8;   // Dir-A end limit switch

const float ADC_VREF              = 5.00f;
const float VOLTAGE_SCALE         = 3.323f; // Change to 3.323f for .12 and 5.156f for .13

// New sensor calibration from ADC readings:
// @ 0 A  : ADC = 430 -> V0 ≈ 2.103 V
// @ 10 A : ADC = 417 -> V10 ≈ 2.039 V
// drop ≈ 0.064 V over 10 A => ~6.4 mV/A

const uint8_t ADC_SAMPLES         = 16;

// Rudder ADC two-stage filter:
//   Stage 1 (per loop): 8 reads — 2 dummy (S/H settle) + 6 real.
//                       Drop highest and lowest of the 6; average remaining 4.
//   Stage 2 (FIFO MA): sliding window of RUDDER_FIFO_SIZE Stage-1 values.
//                       Running sum keeps the average O(1) per update.
// Combined noise reduction: σ / (√4 × √RUDDER_FIFO_SIZE) = σ / 8.
// Tune RUDDER_FIFO_SIZE to trade smoothness vs step-response lag.
const uint8_t RUDDER_FIFO_SIZE    = 16;

const float PI_VSENSE_SCALE = 5.25f;
const float PI_VOLT_HIGH_FAULT = 5.40f;
const float PI_VOLT_LOW_FAULT  = 4.80f;
const float MAX_CONTROLLER_TEMP_C = 50.0f;

// Manual jog state (used when AP is disengaged)
bool  manual_override = false;  // true while a manual jog is active
int8_t manual_dir     = 0;      // -1 = Dir-B, +1 = Dir-A, 0 = none

// Remote manual mode state (from Bridge via TCP)
bool     remote_manual_active     = false;  // true when Bridge is in MANUAL mode
uint16_t manual_rud_target_0_1000 = 500;    // remote rudder target 0=full Dir-A, 1000=full Dir-B
                                            // (canonical convention: 1000=port end, 0=stbd end;
                                            // matches web/bridge pct=100=port)

// ---- Remote-manual brake state machine ----
// States: 0 = DRIVING (toward target), 1 = BRAKING (timed reverse pulse), 2 = SETTLED (in deadband)
enum RemoteManualState : uint8_t { RM_DRIVING = 0, RM_BRAKING = 1, RM_SETTLED = 2 };
RemoteManualState rm_state         = RM_SETTLED;
unsigned long     rm_brake_start_ms = 0;      // millis() when brake pulse began
unsigned long     rm_brake_dur_ms  = 0;       // computed brake duration for this pulse

// ---- Rudder speed estimation ----
// Differentiates smoothed ADC over a ~50 ms window.
const unsigned long SPEED_WINDOW_MS = 50;
int           speed_prev_adc  = 511;          // ADC snapshot at start of window
unsigned long speed_prev_ms   = 0;            // millis() at snapshot
int16_t       rudder_speed_cps = 0;           // signed: +ve = increasing ADC (Dir-A)

// Minimum speed (cps) below which we just coast instead of active braking.
// Below ~60 cps coast distance is within deadband.
const int16_t BRAKE_MIN_SPEED_CPS = 30000;  // braking disabled: threshold unreachable. To re-enable braking later, restore the value to 60.
// AP-pressed warning (Bridge rejected AP toggle in MANUAL mode)
bool          ap_pressed_warn_active = false;
unsigned long ap_pressed_warn_ms     = 0;
const unsigned long AP_PRESSED_WARN_MS = 5000UL; // 5s OLED flash duration
const unsigned long CLUTCH_SETTLE_MS   = 50UL;   // ms to wait after clutch engages before allowing motor EN
unsigned long ap_warn_beep_end_ms    = 0;         // when single beep ends
// Steer-loss warning (remote TCP dropped while in MANUAL mode)
bool steer_loss_active   = false;
bool steer_loss_silenced = false;
// Buzzer state tracking for change reporting to bridge
bool last_buzzer_state = false;

// Current calibration points: ADC reading vs measured amps
const uint8_t  CURR_N = 8;
const uint16_t CURR_ADC[CURR_N] PROGMEM = {430, 427, 426, 424, 423, 422, 420, 417};
const float    CURR_A[CURR_N]   PROGMEM = {0.0f, 0.11f, 0.65f, 1.70f, 2.23f, 2.33f, 4.00f, 5.30f};

static inline uint16_t curr_adc_at(uint8_t idx) {
  return pgm_read_word(&CURR_ADC[idx]);
}

static inline float curr_a_at(uint8_t idx) {
  return pgm_read_float(&CURR_A[idx]);
}

// ---- DS18B20 scheduling ----
const unsigned long TEMP_PERIOD_MS = 1000;
const unsigned long TEMP_CONV_MS = 200;

// ---- OLED ----
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR  0x3C

OneWire oneWire(PIN_DS18B20);
DallasTemperature tempSensors(&oneWire);
SSD1306AsciiWire display;
bool oled_ok = false;
unsigned long oled_last_init_ms = 0;
const unsigned long OLED_INIT_RETRY_MS = 1000UL;

// ---- Rudder calibration (from motor_limit_test) ----
// Raw ADC counts (0..1023)
// Hard safety endpoints for the rudder pot. These should be near the ADC rails so
// manual jog only stops at the mechanical ends if everything else fails.
const int RUDDER_ADC_DIRB_END  = 1;     // ADC at Dir-B end (lower = Dir-B)
const int RUDDER_ADC_DIRA_END  = 1022;  // ADC at Dir-A end (higher = Dir-A)
const int RUDDER_ADC_MARGIN    = 10;    // safety margin on each end
const int RUDDER_ADC_END_HYST  = 8;    // extra counts to CLEAR end-latch (prevents flicker)

// Tolerance for centring (not used here but handy later)
const int RUDDER_CENTRE_ADC    = (RUDDER_ADC_DIRB_END + RUDDER_ADC_DIRA_END) / 2;
const int RUDDER_CENTRE_TOL    = 5;

// Feature enable flags — set by bridge via FEATURES_CODE on startup and on settings change.
// Default 0x00: all features OFF until bridge configures them.
// Safe-start behaviour: no limit-switch reads, no sensor fault alarms before bridge connects.
uint8_t feature_flags = 0x00;
// Second feature byte (EEPROM offset 23, layout v2). Loaded by eeprom_load_settings().
// Bridge does not send this at runtime; it is EEPROM-only until direction-inversion is wired in.
uint8_t feature_flags_2 = 0x00;
bool    g_invert_motor  = false;

// Deadband for AP command dead zone and PWM scaling (in delta units, 0–1000).
// Loaded from EEPROM at boot (= deadband_pct × 10). Falls back to 20 if EEPROM absent.
uint16_t g_deadband = 20;

// ---- Command codes (same as motor.ino) ----
enum commands {
  COMMAND_CODE              = 0xC7,
  RESET_CODE                = 0xE7,
  MAX_CURRENT_CODE          = 0x1E,
  MAX_CONTROLLER_TEMP_CODE  = 0xA4,
  MAX_MOTOR_TEMP_CODE       = 0x5A,
  RUDDER_RANGE_CODE         = 0xB6,
  RUDDER_MIN_CODE           = 0x2B,
  RUDDER_MAX_CODE           = 0x4D,
  REPROGRAM_CODE            = 0x19,
  DISENGAGE_CODE            = 0x68,
  MAX_SLEW_CODE             = 0x71,
  EEPROM_READ_CODE              = 0x91,
  EEPROM_WRITE_CODE             = 0x53,  // arduino_servo protocol: lo=struct_addr, hi=data
  CLUTCH_PWM_AND_BRAKE_CODE     = 0x36,
  BRIDGE_SETTINGS_WRITE_CODE    = 0xFA   // bridge settings push (B47+): hi=addr, lo=data
};

// ---- Forward declarations ----
// Explicit declarations are required for functions called before they are defined.
// arduino-cli normally auto-generates these via ctags, but ctags may not be
// available on all build hosts (e.g. aarch64 Pi 5 after a fresh install).
uint16_t read_rudder_scaled();
int read_rudder_adc();
void service_rudder_adc();
bool dirb_limit_switch_hit();
bool dira_limit_switch_hit();
bool oled_try_init(bool allow_blocking_splash);
void send_frame(uint8_t code, uint16_t value);
void show_overlay(const char *text);
void send_button_event(uint16_t ev);

// ---- Result codes ----
enum results {
  CURRENT_CODE         = 0x1C,
  VOLTAGE_CODE         = 0xB3,
  CONTROLLER_TEMP_CODE = 0xF9,
  MOTOR_TEMP_CODE      = 0x48,
  RUDDER_SENSE_CODE    = 0xA7,
  FLAGS_CODE           = 0x8F,
  EEPROM_VALUE_CODE    = 0x9A
};

// ---- Flags (subset, matching your Python decode) ----
enum {
  SYNC                = 0x0001,
  OVERTEMP_FAULT      = 0x0002,
  OVERCURRENT_FAULT   = 0x0004,
  ENGAGED             = 0x0008,
  INVALID             = 0x0010,
  DIRB_PIN_FAULT      = 0x0020,
  DIRA_PIN_FAULT      = 0x0040,
  BADVOLTAGE_FAULT    = 0x0080,
  // Bit values are pypilot-protocol constants and must not change. Local names
  // were renamed (Fix #6a) so the Nano source reads in the canonical convention:
  // larger value = port end, smaller value = stbd end.
  STBD_END_FAULT      = 0x0100,   // = pypilot MIN_RUDDER_FAULT — Dir-A (high ADC) end
  PORT_END_FAULT      = 0x0200,   // = pypilot MAX_RUDDER_FAULT — Dir-B (low ADC) end
  CURRENT_RANGE       = 0x0400,
  BAD_FUSES           = 0x0800,
  COMMS_WARN_FAULT    = 0x1000,  // error rate elevated (>= 5 errors in 10-second window)
  COMMS_CRIT_FAULT    = 0x2000,  // error rate unsafe (>= 15/10s held 3s) — AP auto-disengaged
  REBOOTED            = 0x8000
};

// Button IDs from the A6 resistor ladder
enum ButtonID : uint8_t {
  BTN_NONE = 0,
  BTN_B1,  // -10 deg
  BTN_B2,  // -1  deg
  BTN_B3,  // AP toggle
  BTN_B4,  // +1 deg
  BTN_B5   // +10  deg
};

// Optional: button event codes to send to Pi via servo protocol
const uint8_t BUTTON_EVENT_CODE = 0xE0;  // new result code, must be handled by Pi-side helper

enum ButtonEvent {
  BTN_EVT_NONE    = 0,
  BTN_EVT_MINUS10 = 1,
  BTN_EVT_MINUS1  = 2,
  BTN_EVT_TOGGLE  = 3,
  BTN_EVT_PLUS10  = 4,
  BTN_EVT_PLUS1   = 5,
  BTN_EVT_STOP    = 6
};

// Overlay for big transient messages (e.g. -10, +1, AP: ON)
bool overlay_active = false;
char overlay_text[8] = "";
unsigned long overlay_start_ms = 0;
const unsigned long OVERLAY_DURATION_MS = 600UL;  // 0.6s display

// ---- RX state ----
uint8_t  in_bytes[3];
uint8_t  sync_b        = 0;
uint8_t  in_sync_count = 0;
uint8_t  bridge_magic_state = 0;

// ---- Diagnostic counters (shown on OLED row 2) ----
uint16_t rx_good_count   = 0;  // CRC-valid frames processed
uint16_t rx_crc_err_count = 0;  // CRC mismatches
uint16_t rx_sync_count   = 0;  // frames discarded during initial sync

// ---- Comms-fault rate detection (sliding 10-second window) ----
const uint8_t  COMMS_DIAG_CODE         = 0xEC;    // Nano->Bridge: lo=err_window_sum, hi=crit_consec_s
const uint8_t  COMMS_ERR_DETAIL_CODE   = 0xED;    // Nano->Bridge: lo=corrupt_code, hi=rx_crc
const unsigned long ERR_DETAIL_MIN_MS  = 200UL;   // rate limit: max 5 detail frames/second
const uint8_t  COMMS_ERR_BUCKETS    = 10;      // 10 x 1-second buckets = 10-second window
const uint8_t  COMMS_WARN_THRESH    = 5;       // errors in window to enter WARN state
const uint8_t  COMMS_CRIT_THRESH    = 15;      // errors in window to enter CRITICAL state
const uint8_t  COMMS_CRIT_HOLD_S    = 3;       // consecutive seconds above CRIT before disengage
const unsigned long COMMS_BUCKET_MS = 1000UL;

uint8_t       err_buckets[COMMS_ERR_BUCKETS];  // per-second error count (circular buffer)
uint8_t       err_bucket_idx    = 0;           // index of current bucket
unsigned long err_bucket_ms     = 0;           // start time of current bucket
uint8_t       err_window_sum    = 0;           // cached sum of all 10 buckets
uint8_t       crit_consec_s     = 0;           // consecutive seconds above CRIT threshold

bool          comms_warn_active      = false;
bool          comms_crit_active      = false;
bool          comms_fault_disengaged = false;  // true after autonomous AP disengage on CRIT
bool          comms_fault_silenced   = false;  // true after user silences comms buzzer via PTM

// ---- Error detail forwarding (1-slot, latest-wins, rate-limited to bridge) ----
bool          err_detail_pending     = false;   // true when an unsent error detail is queued
uint16_t      err_detail_value       = 0;       // packed: lo=code byte, hi=received CRC
unsigned long err_detail_last_ms     = 0;       // last time an error detail frame was sent

// ---- State / telemetry ----
uint16_t flags            = REBOOTED;   // reported once then cleared
uint16_t last_command_val = 1000;       // 0..2000, 1000 = neutral
unsigned long last_command_ms = 0;
uint16_t rudder_raw       = 0;          // 0..65535 (scaled)
int      rudder_adc_last  = 0;          // 0..1023, inverted (higher = Dir-A)

// ---- Rudder ADC FIFO moving-average state ----
// service_rudder_adc() writes here; all other code reads rudder_adc_smoothed.
uint16_t rdr_fifo[16];                 // ring buffer — size matches RUDDER_FIFO_SIZE
uint8_t  rdr_fifo_idx         = 0;    // next write position
uint32_t rdr_fifo_sum         = 0;    // running sum for O(1) average
bool     rdr_fifo_ready       = false; // false until first Stage-1 pre-fills the FIFO
int      rudder_adc_smoothed  = 511;  // published value, inverted (higher = Dir-A)
int      rudder_adc_raw_disp  = 511;  // non-inverted, for OLED debug row

float pi_voltage_v = 0.0f;
bool  pi_overvolt_fault = false;
bool  pi_undervolt_fault = false;
bool  pi_fault = false;
bool  pi_fault_alarm_silenced = false;

// Main supply (Vin) voltage faults
const float VIN_LOW_FAULT_V  = 10.9f;   // below this: undercharge / brown-out
const float VIN_HIGH_FAULT_V = 15.1f;   // above this: overcharge risk
float    vin_v            = 0.0f;
bool     vin_low_fault    = false;
bool     vin_high_fault   = false;
uint32_t vin_low_since_ms = 0;  // millis() when Vin first dropped below threshold; 0 = not low

float temp_c = NAN;
bool temp_pending = false;
unsigned long temp_cycle_ms = 0;
unsigned long temp_req_ms = 0;

float current_ema_a = 0.0f;
bool  current_ema_init = false;
const float CURRENT_EMA_ALPHA = 0.20f;
// ---- Current sensor debug (for calibration) ----
uint16_t current_debug_adc = 0;
float    current_debug_v   = 0.0f;

// ---- Telemetry timing ----
unsigned long last_flags_ms  = 0;
unsigned long last_rudder_ms = 0;
unsigned long last_current_ms = 0;
unsigned long last_voltage_ms = 0;
unsigned long last_temp_ms = 0;
const unsigned long FLAGS_PERIOD_MS  = 200; // 5 Hz
const unsigned long RUDDER_PERIOD_MS = 200; // 5 Hz
const unsigned long CURRENT_PERIOD_MS = 200; // 5 Hz
const unsigned long VOLTAGE_PERIOD_MS = 200; // 5 Hz
const unsigned long TEMP_PERIOD_SEND_MS = 500; // 2 Hz

void show_overlay(const char *text) {
  strncpy(overlay_text, text, sizeof(overlay_text));
  overlay_text[sizeof(overlay_text) - 1] = '\0';
  overlay_active = true;
  overlay_start_ms = millis();
}

ButtonID decode_button_from_adc(int adc) {
  // These thresholds are starting guesses. You'll tune them based on real ADC values.
  // NONE: high ADC, buttons bring it down in steps.
  if (adc > 865) return BTN_NONE;
  if (adc > 608) return BTN_B5;  // +10 deg
  if (adc > 419) return BTN_B4;  // +1 deg
  if (adc > 257) return BTN_B3;  // AP toggle
  if (adc > 139) return BTN_B2;  // -1 deg
  return BTN_B1;                 // -10 deg
}

void send_button_event(uint16_t ev) {
  // Reuse send_frame(): code=BUTTON_EVENT_CODE, 16-bit value = ev
  send_frame(BUTTON_EVENT_CODE, ev);
}

void handle_button(ButtonID b) {
  if (b == BTN_NONE) return;

  switch (b) {
    case BTN_B1:  // -10
      show_overlay("-10");
      send_button_event(BTN_EVT_MINUS10);
      break;

    case BTN_B2:  // -1
      show_overlay("-1");
      send_button_event(BTN_EVT_MINUS1);
      break;

    case BTN_B3: { // AP On or Off (optimistic local UI, remote truth follows)
      ap_display = !ap_display;
      ap_display_override_until_ms = millis() + 2000UL; // 2s grace for remote confirm
      if (!ap_display) {
        // Toggling AP off — cut motor immediately, don't wait for bridge DISENGAGE.
        // When DISENGAGE arrives within the override window, ap_enabled_remote
        // is already false so the display block is a no-op; override expires normally.
        ap_enabled_remote = false;
      }
      show_overlay(ap_display ? "AP: ON" : "AP: OFF");
      send_button_event(BTN_EVT_TOGGLE);
      break;
    }

    case BTN_B4:  // +1
      show_overlay("+1");
      send_button_event(BTN_EVT_PLUS1);
      break;

    case BTN_B5:  // +10
      show_overlay("+10");
      send_button_event(BTN_EVT_PLUS10);
      break;

    default:
      break;
  }
}

// Helper: send a framed 4-byte packet [MAGIC1, MAGIC2, code, value_lo, value_hi, crc]
void send_frame(uint8_t code, uint16_t value) {
  uint8_t body[3];
  body[0] = code;
  body[1] = value & 0xFF;
  body[2] = (value >> 8) & 0xFF;
  uint8_t c = crc8(body, 3);
  Serial.write(BRIDGE_MAGIC1);
  Serial.write(BRIDGE_MAGIC2);
  Serial.write(body, 3);
  Serial.write(c);
}

// ---- Comms-fault sliding-window helpers ----

// Record one CRC error into the current 1-second bucket.
void comms_err_record() {
  if (err_buckets[err_bucket_idx] < 255) {
    err_buckets[err_bucket_idx]++;
    if (err_window_sum < 255) err_window_sum++;
  }
}

// Advance the circular bucket window every 1 second.
// Also maintains crit_consec_s (consecutive seconds above CRIT threshold).
void comms_err_bucket_tick(unsigned long now) {
  if (err_bucket_ms == 0) {
    // First call: initialise (global array is already zero-initialised)
    err_bucket_ms = now;
    return;
  }
  while (now - err_bucket_ms >= COMMS_BUCKET_MS) {
    err_bucket_ms += COMMS_BUCKET_MS;
    // Move to next slot: subtract outgoing bucket then zero it
    err_bucket_idx = (err_bucket_idx + 1) % COMMS_ERR_BUCKETS;
    if (err_window_sum >= err_buckets[err_bucket_idx]) {
      err_window_sum -= err_buckets[err_bucket_idx];
    } else {
      err_window_sum = 0;
    }
    err_buckets[err_bucket_idx] = 0;
    // Evaluate CRIT hold counter once per second
    if (err_window_sum >= COMMS_CRIT_THRESH) {
      if (crit_consec_s < 255) crit_consec_s++;
    } else {
      crit_consec_s = 0;
    }
  }
}

uint16_t read_adc_avg(uint8_t pin, uint8_t samples) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += (uint16_t)analogRead(pin);
    delayMicroseconds(200);
  }
  return (uint16_t)(sum / samples);
}

float adc_to_volts(uint16_t adc) {
  return (adc * ADC_VREF) / 1023.0f;
}

float read_voltage_v() {
  uint16_t adc = read_adc_avg(PIN_VOLTAGE, ADC_SAMPLES);
  float v_adc = adc_to_volts(adc);
  return v_adc * VOLTAGE_SCALE;
}

float read_current_a() {
  uint16_t adc = read_adc_avg(PIN_CURRENT, ADC_SAMPLES);

    // debug tracking
  current_debug_adc = adc;
  current_debug_v   = adc_to_volts(adc);

  // Above "zero" point → treat as 0 A
  if (adc >= curr_adc_at(0)) {
    return curr_a_at(0);
  }
  // Below or equal to lowest calibration → clamp to max
  if (adc <= curr_adc_at(CURR_N - 1)) {
    return curr_a_at(CURR_N - 1);
  }

  // Find the segment CURR_ADC[i] >= adc >= CURR_ADC[i+1]
  for (uint8_t i = 0; i < CURR_N - 1; i++) {
    uint16_t a0 = curr_adc_at(i);
    uint16_t a1 = curr_adc_at(i + 1);

    if (adc <= a0 && adc >= a1) {
      
      float x0 = (float)a0;
      float x1 = (float)a1;
      float y0 = curr_a_at(i);
      float y1 = curr_a_at(i + 1);

      // Since ADC decreases with current, invert the fraction:
      float t = (x0 - (float)adc) / (x0 - x1);   // 0 at x0, 1 at x1
      float I = y0 + t * (y1 - y0);
      if (I < 0.0f) I = 0.0f;
      return I;
    }
  }

  // Fallback (shouldn't hit if table covers full range)
  return 0.0f;
}

float smooth_current_for_display(float a_instant) {
  if (!current_ema_init) {
    current_ema_a = a_instant;
    current_ema_init = true;
    return current_ema_a;
  }

  current_ema_a = (CURRENT_EMA_ALPHA * a_instant) +
                  ((1.0f - CURRENT_EMA_ALPHA) * current_ema_a);
  return current_ema_a;
}

float read_pi_voltage_v() {
  uint16_t adc = read_adc_avg(PIN_PI_VSENSE, ADC_SAMPLES);
  float v_adc = adc_to_volts(adc);
  return v_adc * PI_VSENSE_SCALE;
}

void temp_service(unsigned long now) {
  if (!temp_pending) {
    if (temp_cycle_ms == 0 || (now - temp_cycle_ms >= TEMP_PERIOD_MS)) {
      tempSensors.requestTemperatures();
      temp_req_ms = now;
      temp_cycle_ms = now;
      temp_pending = true;
    }
  }

  if (temp_pending && (now - temp_req_ms >= TEMP_CONV_MS)) {
    float t = tempSensors.getTempCByIndex(0);
    if (t > -55.0f && t < 125.0f) {
      temp_c = t;
    }
    temp_pending = false;
  }
}



void oled_draw() {
  if (!oled_ok) {
    return;
  }

  const unsigned long now = millis();

  // Non-blocking boot splash: hold the display on the splash screen until timer expires
  if (splash_until_ms) {
    if (now < splash_until_ms) return;
    splash_until_ms = 0;   // expired — fall through to first normal draw
  }

  // ----------------------------
  // Measurements used everywhere
  // ----------------------------
  float vin        = read_voltage_v();
  float ia_instant = read_current_a();
  float ia         = smooth_current_for_display(ia_instant);

  bool temp_valid  = (temp_c == temp_c) && (temp_c > -55.0f) && (temp_c < 125.0f);

  // Format bottom line: Vin / I / Temp (ALWAYS at y=52)
  char vnum[10], inum[10], tnum[10];
  char vbuf[12], ibuf[12], tbuf[12];

  // Vin
  dtostrf(vin, 0, 1, vnum);
  strncpy(vbuf, vnum, sizeof(vbuf));
  vbuf[sizeof(vbuf) - 1] = '\0';
  strncat(vbuf, "V", sizeof(vbuf) - strlen(vbuf) - 1);

  // Current
  dtostrf(ia, 0, 2, inum);
  strncpy(ibuf, inum, sizeof(ibuf));
  ibuf[sizeof(ibuf) - 1] = '\0';
  strncat(ibuf, "A", sizeof(ibuf) - strlen(ibuf) - 1);

  // Temp
  if (!temp_valid) {
    strncpy(tbuf, "--.-C", sizeof(tbuf));
    tbuf[sizeof(tbuf) - 1] = '\0';
  } else {
    dtostrf(temp_c, 0, 1, tnum);
    strncpy(tbuf, tnum, sizeof(tbuf));
    tbuf[sizeof(tbuf) - 1] = '\0';
    strncat(tbuf, "C", sizeof(tbuf) - strlen(tbuf) - 1);
  }

  // ----------------------------
  // Controller status state
  // ----------------------------
  unsigned long elapsed_boot  = now - boot_start_ms;
  unsigned long since_last_pi = pi_ever_online ? (now - last_pi_frame_ms) : 0;

  bool pi_timed_out       = pi_ever_online && (since_last_pi > PI_OFFLINE_TIMEOUT_MS);
  bool pi_never_seen      = !pi_ever_online;
  bool within_boot_window = pi_never_seen && (elapsed_boot < PI_BOOT_EST_MS);
  bool boot_offline       = pi_never_seen && (elapsed_boot >= PI_BOOT_EST_MS);

  // Full clear only on display state transition (avoids per-frame flicker)
  // States: 0=boot, 1=offline, 2=online, 3=overlay
  static uint8_t prev_state = 0xFF;
  uint8_t cur_state;
  if (overlay_active && (now - overlay_start_ms < OVERLAY_DURATION_MS)) {
    cur_state = 3;
  } else if (within_boot_window) {
    cur_state = 0;
  } else if (boot_offline || pi_timed_out) {
    cur_state = 1;
  } else {
    cur_state = 2;
  }
  if (cur_state != prev_state) {
    display.clear();
    prev_state = cur_state;
  }

  // ----------------------------
  // Partial-update: skip rows 0-1 when their content hasn't changed.
  // Saves ~25% of I2C time (two of eight rows).
  // ----------------------------
  // Rows 0-1 content now depends only on cur_state (Row 1 is blank)
  uint8_t top_ctx = cur_state;
  static uint8_t prev_top_ctx = 0xFF;
  bool top_changed = (top_ctx != prev_top_ctx);
  prev_top_ctx = top_ctx;

  // ----------------------------
  // Draw
  // ----------------------------
  display.setFont(System5x7);
  display.set1X();

  // --- Rows 0-1: only redraw when context changed (or during boot — progress %) ---
  if (top_changed || within_boot_window) {
    // --- Row 0: heading (always name + version; boot shows progress instead) ---
    display.setCursor(0, 0);
    if (within_boot_window) {
      uint8_t pct = (uint8_t)((elapsed_boot * 100UL) / PI_BOOT_EST_MS);
      if (pct > 100) pct = 100;
      display.print(F("Inno-Cntl:Boot "));
      display.print(pct);
      display.print(F("%"));
    } else {
      display.print(F("Inno-Ctrl "));
      display.print(INNOPILOT_VERSION);
    }
    display.clearToEOL();

    // --- Row 1: fault/warning area — cleared on state change; fault section overwrites each frame ---
    display.setCursor(0, 1);
    display.clearToEOL();
  }

  // --- Pre-calculate rudder overshoot (used in rows 1-2 warning) ---
  bool rudder_overshoot_active = false;
  {
    bool lims_ok = pilot_rudder_valid && pilot_dirb_lim_valid && pilot_dira_lim_valid
                   && (pilot_dira_lim_deg10 > pilot_dirb_lim_deg10);
    if (lims_ok) {
      if (pilot_rudder_deg10 < pilot_dirb_lim_deg10 ||
          pilot_rudder_deg10 > pilot_dira_lim_deg10) {
        rudder_overshoot_active = true;
      }
    }
  }

  // --- Rows 1-2: fault/warning display ---
  // Priority: steer_loss > hw_fault > ver_mismatch > comms_crit > comms_warn > rud_overshoot > ap_warn > overlay
  // 2X messages span rows 1-2; 1X messages use row 1 only. Row 3 is always free for Helm.
  {
    bool any_hw_fault = pi_fault || (flags & OVERTEMP_FAULT) || vin_low_fault || vin_high_fault;
    bool hw_fault_shown = any_hw_fault && !pi_fault_alarm_silenced;

    // Expire ap_pressed_warn after 5s
    if (ap_pressed_warn_active && (now - ap_pressed_warn_ms >= AP_PRESSED_WARN_MS)) {
      ap_pressed_warn_active = false;
    }

    if (steer_loss_active && !steer_loss_silenced) {
      // STEER LOSS: flash 2X "!STEER LOSS" every 400ms
      static bool sl_vis = true;
      static unsigned long sl_flash_ms = 0;
      if (now - sl_flash_ms >= 400UL) {
        sl_flash_ms = now;
        sl_vis = !sl_vis;
      }
      if (sl_vis) {
        display.set2X();
        const char *msg = "!STEER LOSS";
        int16_t x = (SCREEN_WIDTH - (int16_t)strlen(msg) * 12) / 2;
        if (x < 0) x = 0;
        display.setCursor(x, 1);
        display.print(msg);
        display.clearToEOL();
        display.set1X();
      } else {
        display.setCursor(0, 1); display.clearToEOL();
        display.setCursor(0, 2); display.clearToEOL();
      }
    } else if (hw_fault_shown) {
      // Hardware fault: flash 2X message every 400ms
      static bool fault_vis = true;
      static unsigned long fault_flash_ms = 0;
      if (now - fault_flash_ms >= 400UL) {
        fault_flash_ms = now;
        fault_vis = !fault_vis;
      }
      if (fault_vis) {
        // Priority: OVERTEMP > PiV > Vin
        const char *msg;
        if      (flags & OVERTEMP_FAULT) msg = "!OVERTEMP!";
        else if (pi_overvolt_fault)      msg = "!PiV HIGH!";
        else if (pi_undervolt_fault)     msg = "!PiV LOW!";
        else if (vin_high_fault)         msg = "!Vin HIGH!";
        else                             msg = "!Vin LOW!";
        display.set2X();
        int16_t x = (SCREEN_WIDTH - (int16_t)strlen(msg) * 12) / 2;
        if (x < 0) x = 0;
        display.setCursor(x, 1);
        display.print(msg);
        display.clearToEOL();
        display.set1X();
      } else {
        display.setCursor(0, 1); display.clearToEOL();
        display.setCursor(0, 2); display.clearToEOL();
      }
    } else if (bridge_build_valid && bridge_build_num != INNOPILOT_BUILD_NUM) {
      // Version mismatch: flash 1X warning — all components must run the same build
      static bool vm_vis = true;
      static unsigned long vm_flash_ms = 0;
      if (now - vm_flash_ms >= 500UL) { vm_flash_ms = now; vm_vis = !vm_vis; }
      if (vm_vis) {
        display.setCursor(0, 1);
        display.print(F("!VER MISMATCH!"));
        display.clearToEOL();
        char vmbuf[22];
        snprintf(vmbuf, sizeof(vmbuf), "Pi:B%u Nano:B%u", bridge_build_num, INNOPILOT_BUILD_NUM);
        display.setCursor(0, 2);
        display.print(vmbuf);
        display.clearToEOL();
      } else {
        display.setCursor(0, 1); display.clearToEOL();
        display.setCursor(0, 2); display.clearToEOL();
      }
    } else if (comms_crit_active && !comms_fault_silenced) {
      // COMMS CRITICAL: flash 2X "!COMMS ERR!" every 400ms
      static bool cf_vis = true;
      static unsigned long cf_flash_ms = 0;
      if (now - cf_flash_ms >= 400UL) {
        cf_flash_ms = now;
        cf_vis = !cf_vis;
      }
      if (cf_vis) {
        display.set2X();
        const char *msg = "!COMMS ERR!";
        int16_t x = (SCREEN_WIDTH - (int16_t)strlen(msg) * 12) / 2;
        if (x < 0) x = 0;
        display.setCursor(x, 1);
        display.print(msg);
        display.clearToEOL();
        display.set1X();
      } else {
        display.setCursor(0, 1); display.clearToEOL();
        display.setCursor(0, 2); display.clearToEOL();
      }
    } else if (comms_warn_active) {
      // COMMS WARN: static 1X error rate summary
      display.setCursor(0, 1);
      char wbuf[22];
      snprintf(wbuf, sizeof(wbuf), "?Comms:%u err/10s", err_window_sum);
      display.print(wbuf);
      display.clearToEOL();
      display.setCursor(0, 2); display.clearToEOL();
    } else if (rudder_overshoot_active) {
      // Rudder exceeds soft limits: static 1X warning
      display.setCursor(0, 1);
      display.print(F("?Rud>Limit"));
      display.clearToEOL();
      display.setCursor(0, 2); display.clearToEOL();
    } else if (ap_pressed_warn_active) {
      // AP-pressed warning: 1X "?AP Pressed?" (no flashing)
      display.setCursor(0, 1);
      display.print(F("?AP Pressed?"));
      display.clearToEOL();
      display.setCursor(0, 2); display.clearToEOL();
    } else if (overlay_active && (now - overlay_start_ms < OVERLAY_DURATION_MS)) {
      // Overlay (big transient button feedback) on rows 1-2
      display.set2X();
      uint8_t len = strlen(overlay_text);
      int16_t x = (SCREEN_WIDTH - (int16_t)len * 12) / 2;
      if (x < 0) x = 0;
      display.setCursor(x, 1);
      display.print(overlay_text);
      display.clearToEOL();
      display.set1X();
    } else {
      if (!overlay_active || (now - overlay_start_ms >= OVERLAY_DURATION_MS)) {
        overlay_active = false;
        display.setCursor(0, 1); display.clearToEOL();
        display.setCursor(0, 2); display.clearToEOL();
      }
    }
  }

  // --- Row 7: V/A/T (always shown) ---
  display.setCursor(0, 7);
  display.print(vbuf);
  display.print(F("  "));
  display.print(ibuf);
  display.print(F("  "));
  display.print(tbuf);
  display.clearToEOL();

  // --- Boot window: show Nano + Bridge versions in the middle rows, then done ---
  if (within_boot_window) {
    // Row 4: Nano version centred
    {
      char buf[22];
      snprintf(buf, sizeof(buf), "Nano: %s", INNOPILOT_VERSION);
      int16_t tw = strlen(buf) * 6;
      display.setCursor((SCREEN_WIDTH - tw) / 2, 4);
      display.print(buf);
      display.clearToEOL();
    }
    // Row 5: Bridge version centred
    {
      char buf[22];
      if (bridge_build_valid) {
        snprintf(buf, sizeof(buf), "Bridge: v0.2.0_B%u", bridge_build_num);
      } else {
        snprintf(buf, sizeof(buf), "Bridge: waiting...");
      }
      int16_t tw = strlen(buf) * 6;
      display.setCursor((SCREEN_WIDTH - tw) / 2, 5);
      display.print(buf);
      display.clearToEOL();
    }
    return;
  }

  // Offline: clear middle rows and return
  if (boot_offline || pi_timed_out) {
    display.setCursor(0, 3); display.clearToEOL();
    display.setCursor(0, 4); display.clearToEOL();
    display.setCursor(0, 5); display.clearToEOL();
    display.setCursor(0, 6); display.clearToEOL();
    return;
  }

  // --- Online-only rows ---

  // Row 3: Helm mode — HAND (manual jog), AUTO (AP engaged), REMOTE (TCP remote manual)
  display.setCursor(0, 3);
  if (remote_manual_active) {
    display.print(F("Helm: REMOTE"));
  } else if (ap_display) {
    display.print(F("Helm: AUTO"));
  } else {
    display.print(F("Helm: HAND"));
  }
  display.clearToEOL();

  // Row 4: Cmd (left, 3-digit leading zeros) | HDG right-justified (3-digit leading zeros)
  {
    char row5[22];
    char tmp[8];
    memset(row5, ' ', 21);
    row5[21] = '\0';

    // Left: "Cmd:NNN"
    if (pilot_command_valid) {
      uint16_t deg = ((pilot_command_deg10 + 5) / 10) % 360;
      snprintf(tmp, sizeof(tmp), "Cmd:%03u", (unsigned)deg);
    } else {
      strncpy(tmp, "Cmd:---", sizeof(tmp));
    }
    memcpy(row5, tmp, 7);

    // Right: "HDG:NNN" — right-justified (cols 14-20 of the 21-char row)
    if (pilot_heading_valid) {
      uint16_t deg = ((pilot_heading_deg10 + 5) / 10) % 360;
      snprintf(tmp, sizeof(tmp), "HDG:%03u", (unsigned)deg);
    } else {
      strncpy(tmp, "HDG:---", sizeof(tmp));
    }
    memcpy(row5 + 14, tmp, 7);

    display.setCursor(0, 4);
    display.print(row5);
    display.clearToEOL();
  }

  display.setCursor(0, 5); display.clearToEOL();

  // --- Row 6: rudder position — label left, ADC count right-justified ---
  {
    char row6[22];
    memset(row6, ' ', 21);
    row6[21] = '\0';
    const char *label = "Rudder Position:";
    memcpy(row6, label, 16);
    char val[6];
    snprintf(val, sizeof(val), "%d", rudder_adc_raw_disp);
    uint8_t vlen = (uint8_t)strlen(val);
    memcpy(row6 + 21 - vlen, val, vlen);
    display.setCursor(0, 6);
    display.print(row6);
    display.clearToEOL();
  }
}

// ---- EEPROM settings loader ----
// Reads the binary settings block written by the bridge and extracts
// feature_flags and feature_flags_2 so all hardware options are known before pin init.
void eeprom_load_settings() {
  if (EEPROM.read(0) != EEPROM_MAGIC1 || EEPROM.read(1) != EEPROM_MAGIC2) return;
  if (EEPROM.read(2) != EEPROM_LAYOUT_VERSION) return;

  // Walk the variable-length tail to find total block length.
  // Fixed part is 45 bytes (offsets 0-44); v2 added feature_flags_2 at offset 23,
  // shifting the network block from 23–43 to 24–44 and variable strings to 45+.
  uint16_t pos = 45;
  for (uint8_t i = 0; i < 3; i++) {            // SSID, WiFi key, vessel name
    uint8_t slen = EEPROM.read(pos);
    pos += 1 + slen;
    if (pos > 250) return;                      // sanity guard
  }
  // pos now points at the CRC byte
  uint16_t block_len = pos;                     // bytes 0..(pos-1) are payload

  // Validate CRC8 over the entire payload
  uint8_t crc = 0xFF;
  for (uint16_t i = 0; i < block_len; i++) {
    crc = crc8_byte(crc, EEPROM.read(i));
  }
  if (crc != EEPROM.read(block_len)) return;    // CRC mismatch — ignore

  // Extract feature flags (offset 3)
  feature_flags = EEPROM.read(3);

  // Extract deadband (offsets 13-14, uint16 little-endian = deadband_pct × 10).
  // Clamp to 5–200 (0.5–20%) to match the web UI limits and prevent zero-deadband runaway.
  uint16_t db = (uint16_t)EEPROM.read(13) | ((uint16_t)EEPROM.read(14) << 8);
  if (db >= 5 && db <= 200) g_deadband = db;

  // Extract second feature-flags byte (offset 23, layout v2).
  feature_flags_2 = EEPROM.read(23);
  g_invert_motor  = (feature_flags_2 & FEATURE2_INVERT_MOTOR) != 0;
}

bool oled_try_init(bool allow_blocking_splash) {
  // Probe I2C to check if OLED is present
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() != 0) {
    oled_ok = false;
    return false;
  }

  const DevType* dev = (feature_flags & FEATURE_OLED_SH1106) ? &SH1106_128x64 : &Adafruit128x64;
  display.begin(dev, OLED_ADDR);
  display.setFont(System5x7);
  oled_ok = true;

  display.clear();

  // Splash: big Inno-Pilot
  display.set2X();
  display.setCursor(0, 1);  // row 1 (y=8)
  display.println(F("Inno-Pilot"));

  display.set1X();
  display.setCursor(6, 4);  // row 4 (y=32), shifted left to fit on screen
  display.print(F("Version "));
  display.println(INNOPILOT_VERSION);

  // Non-blocking 3 s splash: oled_draw() will return early until the timer expires
  if (allow_blocking_splash && !pi_online_at_boot) {
    splash_until_ms = millis() + 3000UL;
  }

  return true;
}

// Read rudder pot and scale to 0..65535 for telemetry
uint16_t read_rudder_scaled() {
  int a = read_rudder_adc();   // 0..1023 (inverted so higher = Dir-A)
  rudder_adc_last = a;              // save raw for limit logic
  return (uint16_t)a * 64;          // 0..~65472
}

int read_rudder_adc() {
  // Return the pre-averaged, channel-settled value maintained by
  // service_rudder_adc(). Already inverted: higher counts = Dir-A.
  return rudder_adc_smoothed;
}

// Called once per main-loop iteration (end of loop, after all other ADC work).
//
// Stage 1 — synchronous burst of 8 reads, every loop:
//   2 dummy reads  : settle S/H cap after any preceding channel (A6/A0/A1/A3)
//   6 real reads   : take min/max on the fly (no array)
//   drop min + max : trimmed mean of 4 remaining values → stage1 (0..1023)
//   Cost: 8 × ~104 µs = ~832 µs per loop.
//
// Stage 2 — 16-slot FIFO moving average (O(1) running sum):
//   Insert stage1, evict oldest; published value = running_sum / RUDDER_FIFO_SIZE.
//   On first call, pre-fill all slots so output is immediately valid.
//   Combined noise floor: σ / (√4 × √16) = σ / 8.
void service_rudder_adc() {
  // --- Stage 1: trimmed burst ---
  (void)analogRead(RUDDER_PIN);   // dummy 1 — S/H cap settle
  (void)analogRead(RUDDER_PIN);   // dummy 2 — belt-and-braces
  uint16_t mn = 1023, mx = 0, sum6 = 0;
  for (uint8_t i = 0; i < 6; i++) {
    uint16_t r = (uint16_t)analogRead(RUDDER_PIN);
    if (r < mn) mn = r;
    if (r > mx) mx = r;
    sum6 += r;
  }
  uint16_t stage1 = (sum6 - mn - mx) / 4;  // trimmed mean of 4 middle values

  // --- Stage 2: FIFO moving average ---
  if (!rdr_fifo_ready) {
    // Pre-fill all slots with the first valid reading for instant stable output.
    for (uint8_t i = 0; i < RUDDER_FIFO_SIZE; i++) rdr_fifo[i] = stage1;
    rdr_fifo_sum   = (uint32_t)stage1 * RUDDER_FIFO_SIZE;
    rdr_fifo_idx   = 0;
    rdr_fifo_ready = true;
  } else {
    rdr_fifo_sum            -= rdr_fifo[rdr_fifo_idx];  // evict oldest
    rdr_fifo[rdr_fifo_idx]   = stage1;                  // insert newest
    rdr_fifo_sum            += stage1;
    rdr_fifo_idx = (rdr_fifo_idx + 1) % RUDDER_FIFO_SIZE;
  }

  // Publish: FIFO average, inverted so higher = Dir-A.
  uint16_t avg_raw    = (uint16_t)(rdr_fifo_sum / RUDDER_FIFO_SIZE);
  rudder_adc_smoothed = 1023 - (int)avg_raw;  // inverted for limit/motor logic
  rudder_adc_raw_disp = (int)avg_raw;          // non-inverted for OLED debug row
}

// ---- Limit logic helpers ----
// Reads the NC limit switch pins only when the feature is enabled by the bridge.
bool dirb_limit_switch_hit() {
  if (!(feature_flags & FEATURE_LIMIT_SWITCHES)) {
    return false;  // feature disabled: use pot-based soft limits only
  }
  // NC -> GND, so HIGH = open/tripped/broken
  return digitalRead(DIRB_LIMIT_PIN) == HIGH;
}

bool dira_limit_switch_hit() {
  if (!(feature_flags & FEATURE_LIMIT_SWITCHES)) {
    return false;
  }
  return digitalRead(DIRA_LIMIT_PIN) == HIGH;
}

// ---- Motor + clutch drive based on last_command_val & flags ----
void update_motor_from_command() {
  // Always update rudder ADC for limit logic
  int a = read_rudder_adc();   // 0..1023 (inverted so higher = Dir-A)
  rudder_adc_last = a;

  unsigned long now = millis();
  bool pi_alive = pi_ever_online && (now - last_pi_frame_ms <= PI_OFFLINE_TIMEOUT_MS);
  bool ap_active = ap_enabled_remote && pi_alive;
  bool command_recent = (now - last_command_ms <= PI_OFFLINE_TIMEOUT_MS);

// Helper: build g_motor_reason value for MOTOR_REASON_CODE diagnostic frame.
// Encodes the activation branch + key state flags + approx last_command_val.
#define SET_MOTOR_REASON(rsn) \
  g_motor_reason = (uint16_t)((rsn) \
    | (ap_enabled_remote ? 0x10 : 0) \
    | (command_recent    ? 0x20 : 0) \
    | (pi_alive          ? 0x40 : 0) \
    | (manual_override   ? 0x80 : 0)) \
    | ((uint16_t)((last_command_val >> 3) & 0xFF) << 8)
  bool pilot_limits_ok = pi_alive &&
                         pilot_rudder_valid &&
                         pilot_dirb_lim_valid &&
                         pilot_dira_lim_valid &&
                         (pilot_dirb_lim_deg10 < pilot_dira_lim_deg10);
  const int16_t PILOT_LIMIT_HYST_DEG10 = 5;  // 0.5 deg hysteresis to avoid jitter
  bool at_dirb_pilot_enter = pilot_limits_ok && (pilot_rudder_deg10 <= pilot_dirb_lim_deg10);
  bool at_dira_pilot_enter = pilot_limits_ok && (pilot_rudder_deg10 >= pilot_dira_lim_deg10);
  int16_t dirb_pilot_exit = pilot_dirb_lim_deg10 + PILOT_LIMIT_HYST_DEG10;
  int16_t dira_pilot_exit = pilot_dira_lim_deg10 - PILOT_LIMIT_HYST_DEG10;
  bool at_dirb_pilot_hold = pilot_limits_ok && (pilot_rudder_deg10 <= dirb_pilot_exit);
  bool at_dira_pilot_hold = pilot_limits_ok && (pilot_rudder_deg10 >= dira_pilot_exit);

  int16_t delta = (int16_t)last_command_val - 1000;   // -1000..+1000
  int16_t db    = (int16_t)g_deadband;                // cast once for signed comparisons

  bool manual_jog_active = manual_override;
  int8_t manual_jog_dir = manual_dir;
  if (!manual_override && !ap_active && !remote_manual_active && pi_alive && command_recent) {
    // Conv 1: larger value = port. delta > 0 means port; Dir-B = manual_jog_dir -1.
    if (delta > db) {
      manual_jog_active = true;
      manual_jog_dir = -1;
    } else if (delta < -db) {
      manual_jog_active = true;
      manual_jog_dir = +1;
    }
  }

  // Clutch engages when AP is enabled remotely OR manual jog is active OR remote MANUAL mode.
  // Safety: clutch forced OFF during pi_fault or overtemp.
  bool clutch_should = (manual_jog_active || ap_active || remote_manual_active) &&
                       !pi_fault &&
                       !(flags & OVERTEMP_FAULT);

  // FEATURE_INVERT_CLUTCH: relay is active-LOW, so invert the pin level.
  bool clutch_pin_high = clutch_should ^ (bool)(feature_flags & FEATURE_INVERT_CLUTCH);
  digitalWrite(CLUTCH_PIN, clutch_pin_high ? HIGH : LOW);

  // Clutch settling: hold EN=0 for CLUTCH_SETTLE_MS after clutch engages.
  // Prevents motor current from flowing before the relay/solenoid is fully closed,
  // which causes the initial vibration pulse observed on first AP engagement.
  static bool          s_clutch_was_on    = false;
  static unsigned long s_clutch_engage_ms = 0;
  if (clutch_pin_high && !s_clutch_was_on) {
    s_clutch_engage_ms = now;
  }
  s_clutch_was_on = clutch_pin_high;
  bool clutch_settled = !clutch_pin_high ||
                        (now - s_clutch_engage_ms >= CLUTCH_SETTLE_MS);

  // Direction-change guard: zero EN before switching H-bridge direction pins.
  // Without this, the opposite output briefly fires whenever the code transitions
  // between directions while EN is already HIGH (e.g. manual-jog → AP drive or
  // AP direction reversal), producing the opposite-LED flash on button release.
  static int8_t last_drive_dir = 0;  // 0=stopped, +1=Dir-A (LPWM=HIGH), -1=Dir-B (RPWM=HIGH)

  // If in a fault state, don't drive the motor at all
  if (pi_fault || (flags & OVERTEMP_FAULT)) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    last_drive_dir = 0;
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // Enter thresholds (near the ends)
  bool at_dirb_enter = dirb_limit_switch_hit() ||
                       (a <= (RUDDER_ADC_DIRB_END + RUDDER_ADC_MARGIN));

  bool at_dira_enter = dira_limit_switch_hit() ||
                       (a >= (RUDDER_ADC_DIRA_END - RUDDER_ADC_MARGIN));

  // Exit thresholds (must move further away before we clear the latch)
  // Dir-B end is low ADC: we "hold" Dir-B end while ADC <= dirb_exit
  // Dir-A end is high ADC: we "hold" Dir-A end while ADC >= dira_exit
  int dirb_exit = RUDDER_ADC_DIRB_END + (RUDDER_ADC_MARGIN + RUDDER_ADC_END_HYST);
  int dira_exit = RUDDER_ADC_DIRA_END - (RUDDER_ADC_MARGIN + RUDDER_ADC_END_HYST);

  bool at_dirb_hold = dirb_limit_switch_hit() || (a <= dirb_exit);
  bool at_dira_hold = dira_limit_switch_hit() || (a >= dira_exit);

  // Latch ensures only one end is active; hysteresis prevents flicker/reset on jitter.
  static uint8_t end_latch = 0;  // 0 none, 1 Dir-B, 2 Dir-A

  switch (end_latch) {
    case 0:
      if (at_dirb_enter && !at_dira_enter) {
        end_latch = 1;
      } else if (at_dira_enter && !at_dirb_enter) {
        end_latch = 2;
      } else if (at_dirb_enter && at_dira_enter) {
        // If both appear "true" (noise / overlap), pick the closer end.
        int dist_dirb = abs(a - RUDDER_ADC_DIRB_END);
        int dist_dira = abs(a - RUDDER_ADC_DIRA_END);
        end_latch = (dist_dirb <= dist_dira) ? 1 : 2;
      }
      break;

    case 1:
      // Stay latched until we've moved away past the EXIT threshold
      if (!at_dirb_hold) end_latch = 0;
      break;

    case 2:
      if (!at_dira_hold) end_latch = 0;
      break;
  }

  bool at_dirb_end = (end_latch == 1) && at_dirb_hold;
  bool at_dira_end = (end_latch == 2) && at_dira_hold;

  // Pilot limit latch mirrors the ADC end latch to avoid jitter around calibrated limits.
  static uint8_t pilot_latch = 0;  // 0 none, 1 Dir-B, 2 Dir-A

  if (!pilot_limits_ok) {
    pilot_latch = 0;
  } else {
    switch (pilot_latch) {
      case 0:
        if (at_dirb_pilot_enter && !at_dira_pilot_enter) {
          pilot_latch = 1;
        } else if (at_dira_pilot_enter && !at_dirb_pilot_enter) {
          pilot_latch = 2;
        } else if (at_dirb_pilot_enter && at_dira_pilot_enter) {
          int dist_dirb = abs(pilot_rudder_deg10 - pilot_dirb_lim_deg10);
          int dist_dira = abs(pilot_rudder_deg10 - pilot_dira_lim_deg10);
          pilot_latch = (dist_dirb <= dist_dira) ? 1 : 2;
        }
        break;

      case 1:
        if (!at_dirb_pilot_hold) pilot_latch = 0;
        break;

      case 2:
        if (!at_dira_pilot_hold) pilot_latch = 0;
        break;
    }
  }

  bool at_dirb_pilot = (pilot_latch == 1) && at_dirb_pilot_hold;
  bool at_dira_pilot = (pilot_latch == 2) && at_dira_pilot_hold;

  // Update rudder fault flags (as before)
  if (at_dirb_end) {
    flags |= PORT_END_FAULT;
  } else {
    flags &= ~PORT_END_FAULT;
  }

  if (at_dira_end) {
    flags |= STBD_END_FAULT;
  } else {
    flags &= ~STBD_END_FAULT;
  }

  // ---- Remote MANUAL mode: drive at 255 PWM with active reverse-braking ----
  //
  // Speed-aware braking eliminates the coast overshoot that plagued the old
  // bang-bang controller.  The motor always runs at full duty (255) but a
  // timed reverse pulse decelerates the pump so it stops near the target
  // centre.  The 21-count deadband absorbs residual error.
  //
  // Brake formula (linear fit to measured coast/speed/brake_ms table):
  //   brake_ms  = 0.54 * speed_cps + 72
  //   brake_dist ≈ speed_cps * brake_ms / 2000  (half-speed assumption)
  //
  // State machine: RM_DRIVING → RM_BRAKING → RM_SETTLED
  //   RM_DRIVING : full 255 PWM toward target
  //   RM_BRAKING : timed reverse pulse running; no re-evaluation until done
  //   RM_SETTLED : inside deadband, motor off
  //
  if (remote_manual_active) {
    // ---- Angle-based remote-manual position control (B49) ----
    // Previously this loop steered on the raw rudder ADC against hard-coded
    // endpoints (RUDDER_ADC_DIRA_END/DIRB_END = 1022/1) assuming "higher ADC =
    // Dir-A/stbd".  On a boat whose rudder pot is wired/oriented opposite to that
    // assumption (or whose travel does not span the full ADC) the error sign
    // inverts and the rudder runs hard-over to the stop (positive-feedback
    // runaway) — it never holds the commanded angle.
    //
    // Fix: steer on pypilot's CALIBRATED rudder angle (pilot_rudder_deg10, tenths
    // of a degree, +port / -stbd) and the calibrated limits.  These are correct on
    // any boat regardless of pot polarity/range, so the loop converges and holds.
    // Raw ADC is kept ONLY for the (magnitude-only) speed estimate that sizes the
    // brake pulse.
    //
    // Motor convention (reference build): Dir-A (LPWM HIGH) moves the rudder to
    // STARBOARD (angle decreases); Dir-B (RPWM HIGH) moves to PORT (angle
    // increases).  FEATURE2_INVERT_MOTOR swaps the H-bridge sense for boats wired
    // the other way.  Hardware impact: this changes which pin fires for a given
    // commanded angle — verify on the bench before sea use.
    //
    // Fail-safe: without a trustworthy calibrated position (pilot_limits_ok) we must
    // not drive blind, so the motor is held off.  This means remote-manual steering
    // now depends on pypilot publishing rudder.angle (it always runs as a service).
    if (!pilot_limits_ok) {
      analogWrite(HBRIDGE_PWM_PIN, 0);
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      last_drive_dir = 0;
      rm_state = RM_SETTLED;
      return;
    }

    // ---- Axis convention (CRITICAL — Fix #B49b) ----
    // The bridge transmits the Nano -rudder.angle (PILOT_RUDDER_CODE) plus matching
    // limits, so on the pilot_rudder_deg10 axis  + = STARBOARD, - = PORT:
    //   pilot_dira_lim_deg10 = +range -> the STBD (+) end
    //   pilot_dirb_lim_deg10 = -range -> the PORT (-) end
    // pilot_limits_ok guarantees pilot_dirb_lim_deg10 < pilot_dira_lim_deg10.
    // The original B49 code assumed the opposite (+port/-stbd); that single inverted
    // assumption made the loop positive-feedback and ran the rudder hard-over to the
    // commanded-side stop.  This axis now matches the rest of the firmware (the
    // rudder-overshoot check and at_dira_pilot/at_dirb_pilot).
    int16_t stbd_end_deg10 = pilot_dira_lim_deg10;   // most positive = stbd (+) end
    int16_t port_end_deg10 = pilot_dirb_lim_deg10;   // most negative = port (-) end
    int32_t span_deg10     = (int32_t)stbd_end_deg10 - port_end_deg10;  // > 0

    // Map target 0..1000 onto the calibrated range:
    //   0 = full STBD (+ end), 1000 = full PORT (- end), 500 = centre.
    int16_t target_deg10 = (int16_t)(stbd_end_deg10 -
        span_deg10 * (int32_t)manual_rud_target_0_1000 / 1000);

    // Position error (tenths deg): >0 => move toward STBD (increase pilot_rudder_deg10),
    //                              <0 => move toward PORT (decrease pilot_rudder_deg10).
    int16_t error     = target_deg10 - pilot_rudder_deg10;
    int16_t abs_error = (error >= 0) ? error : -error;

    // Angle deadband from the operator deadband_pct (g_deadband = pct*10):
    //   deadband_deg10 = span_deg10 * pct/100 = span_deg10 * g_deadband / 1000
    int16_t REMOTE_DEADBAND = (int16_t)(span_deg10 * (int32_t)g_deadband / 1000);
    if (REMOTE_DEADBAND < 8) REMOTE_DEADBAND = 8;   // ~0.8 deg floor: avoid hunting

    // Reduce PWM within this band of target to limit overshoot between the
    // ~5 Hz pilot position updates.
    int16_t SLOW_ZONE_DEG10 = (int16_t)(REMOTE_DEADBAND * 4);
    const uint8_t RM_SLOW_PWM = 160;

    bool invert_motor = (feature_flags_2 & FEATURE2_INVERT_MOTOR) != 0;

    // --- Local-ADC speed estimate (magnitude only) for brake-pulse sizing ---
    if (now - speed_prev_ms >= SPEED_WINDOW_MS) {
      int delta_adc = a - speed_prev_adc;
      unsigned long dt = now - speed_prev_ms;
      rudder_speed_cps = (int16_t)((long)delta_adc * 1000L / (long)dt);
      speed_prev_adc = a;
      speed_prev_ms  = now;
    }
    int16_t abs_speed = (rudder_speed_cps >= 0) ? rudder_speed_cps : -rudder_speed_cps;
    // brake_ms = 0.54*speed + 72 (integer-friendly): reverse-pulse duration.
    unsigned long est_brake_ms = (unsigned long)((54L * abs_speed + 50) / 100) + 72UL;

    // --- State machine ---
    switch (rm_state) {

      case RM_BRAKING: {
        // Timed reverse pulse in progress — hold until duration expires
        if (now - rm_brake_start_ms >= rm_brake_dur_ms) {
          // Brake pulse complete — cut motor
          analogWrite(HBRIDGE_PWM_PIN, 0);
          last_drive_dir = 0;
          digitalWrite(HBRIDGE_RPWM_PIN, LOW);
          digitalWrite(HBRIDGE_LPWM_PIN, LOW);
          // Check where we ended up
          rm_state = (abs_error <= REMOTE_DEADBAND) ? RM_SETTLED : RM_DRIVING;
        }
        // else: keep brake pulse running (pins already set when entering BRAKING)
        break;
      }

      case RM_SETTLED: {
        // In deadband — stay put unless target moves outside
        if (abs_error > REMOTE_DEADBAND) {
          rm_state = RM_DRIVING;
          // fall through to DRIVING below
        } else {
          // Motor off, hold position via hydraulic lock
          analogWrite(HBRIDGE_PWM_PIN, 0);
          last_drive_dir = 0;
          digitalWrite(HBRIDGE_RPWM_PIN, LOW);
          digitalWrite(HBRIDGE_LPWM_PIN, LOW);
          break;
        }
      }
      // intentional fall-through from SETTLED when error exceeds deadband
      // fall through

      case RM_DRIVING: {
        // dir: +1 = drive STBD (increase angle), -1 = drive PORT (decrease angle)
        int8_t dir = 0;
        if      (error >  REMOTE_DEADBAND) dir = +1;   // need stbd
        else if (error < -REMOTE_DEADBAND) dir = -1;   // need port

        // Respect calibrated pilot limits + physical end switches for the end we
        // would drive into (axis: + = stbd = dira, - = port = dirb).  The over_*_end
        // terms are a convention-independent ABSOLUTE backstop: even if the steering
        // sign were wrong again, the motor can never be driven past a calibrated end.
        bool over_stbd_end = (pilot_rudder_deg10 >= pilot_dira_lim_deg10);  // at/over + end
        bool over_port_end = (pilot_rudder_deg10 <= pilot_dirb_lim_deg10);  // at/over - end
        bool blocked_stbd = at_dira_pilot || over_stbd_end || dira_limit_switch_hit();
        bool blocked_port = at_dirb_pilot || over_port_end || dirb_limit_switch_hit();
        if (dir > 0 && blocked_stbd) dir = 0;
        if (dir < 0 && blocked_port) dir = 0;

        if (dir == 0) {
          // Within deadband or at a limit — stop and hold.
          analogWrite(HBRIDGE_PWM_PIN, 0);
          last_drive_dir = 0;
          digitalWrite(HBRIDGE_RPWM_PIN, LOW);
          digitalWrite(HBRIDGE_LPWM_PIN, LOW);
          rm_state = RM_SETTLED;
          break;
        }

        // If moving fast and inside the pre-brake zone, fire a reverse pulse
        // (opposite to travel) to kill ram momentum, then settle.
        if (abs_speed >= BRAKE_MIN_SPEED_CPS &&
            abs_error <= (int16_t)(REMOTE_DEADBAND * 2)) {
          bool brake_toward_port = (dir > 0);                  // opposite to travel (dir>0 = driving stbd)
          bool rpwm_high = brake_toward_port ^ invert_motor;   // ref: port = RPWM HIGH
          int8_t dd = rpwm_high ? -1 : +1;                     // +1 = LPWM HIGH (file convention)
          if (dd != last_drive_dir && last_drive_dir != 0) {
            analogWrite(HBRIDGE_PWM_PIN, 0);                   // EN off before direction change
          }
          digitalWrite(HBRIDGE_RPWM_PIN, rpwm_high ? HIGH : LOW);
          digitalWrite(HBRIDGE_LPWM_PIN, rpwm_high ? LOW  : HIGH);
          last_drive_dir    = dd;
          rm_brake_dur_ms   = est_brake_ms;
          rm_brake_start_ms = now;
          rm_state          = RM_BRAKING;
          SET_MOTOR_REASON(MRSN_RM_BRAKE);                    // B26: record activation reason
          analogWrite(HBRIDGE_PWM_PIN, clutch_settled ? 255 : 0);
          break;
        }

        // Drive toward target.  Full duty far out; reduced inside the slow zone to
        // limit overshoot between pilot position updates.
        {
          bool drive_toward_port = (dir < 0);
          bool rpwm_high = drive_toward_port ^ invert_motor;   // ref: port = RPWM HIGH
          int8_t dd = rpwm_high ? -1 : +1;                     // +1 = LPWM HIGH (file convention)
          if (dd != last_drive_dir && last_drive_dir != 0) {
            analogWrite(HBRIDGE_PWM_PIN, 0);                   // EN off before direction change
          }
          digitalWrite(HBRIDGE_RPWM_PIN, rpwm_high ? HIGH : LOW);
          digitalWrite(HBRIDGE_LPWM_PIN, rpwm_high ? LOW  : HIGH);
          last_drive_dir = dd;
          SET_MOTOR_REASON(MRSN_RM_DRIVE);                    // B26: record activation reason
          uint8_t pwm = (abs_error <= SLOW_ZONE_DEG10) ? RM_SLOW_PWM : 255;
          analogWrite(HBRIDGE_PWM_PIN, clutch_settled ? pwm : 0);
        }
        break;
      }
    }
    return;
  } else {
    // Not in remote-manual mode — reset state machine so next entry starts clean
    rm_state = RM_SETTLED;
  }

  // ---- Manual override branch (AP disengaged) ----
  if (manual_jog_active) {
    // Respect limits
    if (manual_jog_dir < 0 && (at_dirb_end || at_dirb_pilot)) {
      // Trying to jog further to Dir-B, but at/near Dir-B end → stop
      analogWrite(HBRIDGE_PWM_PIN, 0);
      last_drive_dir = 0;
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      return;
    }
    if (manual_jog_dir > 0 && (at_dira_end || at_dira_pilot)) {
      // Trying to jog further to Dir-A, but at/near Dir-A end → stop
      analogWrite(HBRIDGE_PWM_PIN, 0);
      last_drive_dir = 0;
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      return;
    }

    const uint8_t duty = 255;  // full duty for now; you can tune later

    if (manual_jog_dir > 0) {
      // Dir-A: increase ADC
      // Record activation reason: distinguish physical-button from delta-jog
      SET_MOTOR_REASON(manual_override ? MRSN_MANUAL_PHYS : MRSN_DELTA_JOG);  // B26
      if (last_drive_dir == -1) analogWrite(HBRIDGE_PWM_PIN, 0);  // EN off before direction change
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, HIGH);
      last_drive_dir = +1;
    } else if (manual_jog_dir < 0) {
      // Dir-B: decrease ADC
      SET_MOTOR_REASON(manual_override ? MRSN_MANUAL_PHYS : MRSN_DELTA_JOG);  // B26
      if (last_drive_dir == +1) analogWrite(HBRIDGE_PWM_PIN, 0);  // EN off before direction change
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      digitalWrite(HBRIDGE_RPWM_PIN, HIGH);
      last_drive_dir = -1;
    } else {
      // No direction -> stop
      analogWrite(HBRIDGE_PWM_PIN, 0);
      last_drive_dir = 0;
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      return;
    }

    analogWrite(HBRIDGE_PWM_PIN, clutch_settled ? duty : 0);
    return;  // do not fall through to autopilot logic
  }

  // ---- Existing autopilot logic below (unchanged) ----

  // If AP is not active (remote ap.enabled false or Pi offline), don't drive motor
  if (!ap_active) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    last_drive_dir = 0;
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // ----- Autopilot motor drive from last_command_val -----
  // last_command_val: 0..2000, 1000 = stop
  // Deadband around neutral
  if (delta > -db && delta < db) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    last_drive_dir = 0;
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // Don't drive further into soft limits.
  // Conv 1: delta > 0 = port (Dir-B); delta < 0 = stbd (Dir-A).
  if (delta > 0 && (at_dirb_end || at_dirb_pilot)) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    last_drive_dir = 0;
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }
  if (delta < 0 && (at_dira_end || at_dira_pilot)) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    last_drive_dir = 0;
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // Duty mapping (keep MIN_DUTY = MAX_DUTY = 255 for your hydraulic pump: ON/OFF only)
  // If you ever experiment with PWM later, set MIN_DUTY somewhere >= 180 and MAX_DUTY = 255.
  const uint8_t MIN_DUTY = 255;
  const uint8_t MAX_DUTY = 255;

  int16_t abs_delta = (delta >= 0) ? delta : -delta;
  if (abs_delta > 1000) abs_delta = 1000;

  uint8_t duty = MIN_DUTY;
  if (MAX_DUTY == MIN_DUTY) {
    duty = MIN_DUTY;
  } else {
    int16_t span = 1000 - db;
    if (span < 1) span = 1;
    int16_t effective = abs_delta - db;
    if (effective < 0) effective = 0;

    duty = MIN_DUTY + (uint8_t)((effective * (MAX_DUTY - MIN_DUTY)) / span);
  }

  // Direction: conv 1: delta > 0 = large value = port => Dir-B (RPWM); delta < 0 = stbd => Dir-A (LPWM)
  SET_MOTOR_REASON(MRSN_AP);  // B26: record activation reason (autopilot path)
  {
    int8_t ap_new_dir = (delta > 0) ? -1 : +1;
    if (ap_new_dir != last_drive_dir && last_drive_dir != 0) {
      analogWrite(HBRIDGE_PWM_PIN, 0);  // EN off before direction change
    }
    if (delta > 0) {
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      digitalWrite(HBRIDGE_RPWM_PIN, HIGH);
    } else {
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, HIGH);
    }
    last_drive_dir = ap_new_dir;
  }

  analogWrite(HBRIDGE_PWM_PIN, clutch_settled ? duty : 0);
}

// Process one CRC-valid frame
void process_packet() {
  flags |= SYNC;

  unsigned long now = millis();

  // Mark Pi controller online on first valid frame
  if (!pi_online) {
    pi_online = true;
    pi_ever_online = true;
    pi_online_time_ms = now;
  }

  // Update last time we heard from the Pi
  last_pi_frame_ms = now;
  
  uint16_t value = in_bytes[1] | (in_bytes[2] << 8);
  uint8_t  code  = in_bytes[0];

  switch (code) {
    case COMMAND_CODE:
      // 0..2000, 1000 = neutral
      last_command_val = value;
      last_command_ms = now;
      // Receiving COMMAND_CODE implies AP is engaged
      if (!ap_enabled_remote) {
        ap_enabled_remote = true;
        flags |= ENGAGED;
        if (ap_display_override_until_ms == 0) {
          ap_display = true;
        } else if (ap_display == true) {
          ap_display_override_until_ms = 0;
        }
      }
      break;

    case DISENGAGE_CODE:
      flags &= ~ENGAGED;
      // DISENGAGE_CODE means AP is off
      if (ap_enabled_remote) {
        ap_enabled_remote = false;
        if (ap_display_override_until_ms == 0) {
          ap_display = false;
        } else if (ap_display == false) {
          ap_display_override_until_ms = 0;
        }
      }
      // Reset stale command so the delta-based manual-jog check in
      // update_motor_from_command() cannot trigger from the last AP command.
      // Without this, a non-neutral last_command_val + command_recent=true
      // causes the clutch to engage and the motor to run for up to 5 s after
      // AP is disengaged, even though ap_display shows HAND (bug: motor steers
      // when remote is set to OFF).
      last_command_val = 1000;   // neutral — delta = 0 < DEADBAND
      last_command_ms  = 0;      // command_recent = false immediately
      break;

    case RESET_CODE:
      flags &= ~OVERCURRENT_FAULT;
      break;

    case PILOT_HEADING_CODE:
      pilot_heading_deg10 = value;
      pilot_heading_valid = true;
      break;

    case PILOT_COMMAND_CODE:
      pilot_command_deg10 = value;
      pilot_command_valid = true;
      break;

    case PILOT_RUDDER_CODE:
      pilot_rudder_deg10 = (int16_t)value; // interpret two's complement
      pilot_rudder_valid = true;
      break;

    case PILOT_RUDDER_DIRB_LIM_CODE:
      pilot_dirb_lim_deg10 = (int16_t)value;
      pilot_dirb_lim_valid = true;
      break;
    
    case PILOT_RUDDER_DIRA_LIM_CODE:
      pilot_dira_lim_deg10 = (int16_t)value;
      pilot_dira_lim_valid = true;
      break;

    case MANUAL_MODE_CODE:
      if (value != 0 && !remote_manual_active) {
        // Entering MANUAL: initialise speed estimator and brake state machine
        speed_prev_adc    = rudder_adc_smoothed;
        speed_prev_ms     = millis();
        rudder_speed_cps  = 0;
        rm_state          = RM_SETTLED;
        // Seed target from the Nano's own ADC so the motor holds still on entry.
        // Anchored at Dir-A (high ADC) so pos=0 at Dir-A end and pos=1000 at
        // Dir-B end, matching manual_rud_target_0_1000's canonical convention
        // (1000=port end, 0=stbd end).
        {
          long pos = (long)(RUDDER_ADC_DIRA_END - rudder_adc_smoothed) * 1000L
                     / (RUDDER_ADC_DIRA_END - RUDDER_ADC_DIRB_END);
          if (pos < 0)    pos = 0;
          if (pos > 1000) pos = 1000;
          manual_rud_target_0_1000 = (uint16_t)pos;
        }
      }
      remote_manual_active = (value != 0);
      if (!remote_manual_active) {
        // Exiting MANUAL: clear steer-loss state
        steer_loss_active   = false;
        steer_loss_silenced = false;
      }
      break;

    case MANUAL_RUD_TARGET_CODE:
      manual_rud_target_0_1000 = value;
      break;

    case WARNING_CODE:
      if ((uint8_t)value == WARN_AP_PRESSED) {
        ap_pressed_warn_active = true;
        ap_pressed_warn_ms     = millis();
        ap_warn_beep_end_ms    = millis() + 200UL;  // single 200ms beep
      } else if ((uint8_t)value == WARN_STEER_LOSS) {
        steer_loss_active   = true;
        steer_loss_silenced = false;
      } else if ((uint8_t)value == WARN_NONE) {
        steer_loss_active      = false;
        steer_loss_silenced    = false;
        ap_pressed_warn_active = false;
      }
      break;

    case BRIDGE_HELLO_CODE:
      send_frame(BRIDGE_HELLO_ACK_CODE, 0xBEEF);
      break;

    case BRIDGE_VERSION_CODE:
      bridge_build_num = value;
      bridge_build_valid = true;
      break;

    case FEATURES_CODE: {
      // Bridge configures which sensors/alarms are active.
      uint8_t new_flags = (uint8_t)(value & 0xFF);
      bool limit_was_off = !(feature_flags & FEATURE_LIMIT_SWITCHES);
      bool limit_now_on  = (new_flags & FEATURE_LIMIT_SWITCHES);
      feature_flags = new_flags;
      // Configure limit switch pins the first time they are enabled.
      if (limit_was_off && limit_now_on) {
        pinMode(DIRB_LIMIT_PIN, INPUT_PULLUP);  // NC -> GND
        pinMode(DIRA_LIMIT_PIN, INPUT_PULLUP);
      }
      break;
    }

    case EEPROM_WRITE_CODE: {
      // arduino_servo protocol encoding: lo byte = struct byte addr, hi byte = data.
      // Writes into the arduino_servo EEPROM mirror region only; addr is bounds-checked.
      uint8_t addr = (uint8_t)(value & 0xFF);
      uint8_t data = (uint8_t)(value >> 8);
      if (addr < ASRV_EEPROM_SIZE) {
        EEPROM.update(ASRV_EEPROM_BASE + addr, data);
      }
      break;
    }

    case BRIDGE_SETTINGS_WRITE_CODE: {
      // Bridge settings push encoding (B47+): hi byte = EEPROM addr, lo byte = data.
      // Writes into the bridge settings region (addresses 0–175).
      uint8_t addr = (uint8_t)(value >> 8);
      uint8_t data = (uint8_t)(value & 0xFF);
      EEPROM.update(addr, data);
      break;
    }

    case EEPROM_READ_CODE: {
      // arduino_servo requests a range of struct bytes to verify its local copy.
      // Respond with one EEPROM_VALUE_CODE frame per byte: lo=struct_addr, hi=stored_byte.
      // arduino_servo processes frames in even/odd pairs; sequential iteration satisfies this.
      uint8_t start = (uint8_t)(value & 0xFF);
      uint8_t end   = (uint8_t)(value >> 8);
      for (uint8_t a = start; a < end && a < ASRV_EEPROM_SIZE; a++) {
        send_frame(EEPROM_VALUE_CODE,
                   (uint16_t)a | ((uint16_t)EEPROM.read(ASRV_EEPROM_BASE + a) << 8));
      }
      break;
    }

    default:
      // Unhandled commands ignored for now
      break;
  }
}

void setup() {
  // Read EEPROM first so feature_flags (clutch polarity, OLED type, etc.) is
  // known before any pin is configured. No peripherals needed — EEPROM.read()
  // works immediately after reset.
  eeprom_load_settings();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(RUDDER_PIN, INPUT);  // pot

  // IBT-2 pins
  pinMode(HBRIDGE_RPWM_PIN, OUTPUT);
  pinMode(HBRIDGE_LPWM_PIN, OUTPUT);
  pinMode(HBRIDGE_PWM_PIN, OUTPUT);
  digitalWrite(HBRIDGE_RPWM_PIN, LOW);
  digitalWrite(HBRIDGE_LPWM_PIN, LOW);
  analogWrite(HBRIDGE_PWM_PIN, 0);   // motor off

  // Clutch — pre-load PORT latch with the disengaged level BEFORE enabling the
  // output driver so the pin never glitches to the engaged state even for one
  // clock cycle. Polarity is read from EEPROM above; default (no EEPROM) is LOW.
  digitalWrite(CLUTCH_PIN, (feature_flags & FEATURE_INVERT_CLUTCH) ? HIGH : LOW);
  pinMode(CLUTCH_PIN, OUTPUT);

  // Limit switch pins: configured when FEATURES_CODE arrives from bridge.
  // At boot feature_flags=0x00 so these pins are not activated yet.
  // See FEATURES_CODE handling in process_packet() for runtime enable.

  pinMode(PTM_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.begin(38400);
  
  // Quick check: is Pi/pypilot already talking?
  // Look for any incoming serial for ~200ms
  {
    unsigned long t0 = millis();
    while (millis() - t0 < 200UL) {
      if (Serial.available()) {
        pi_online_at_boot = true;
        break;
      }
    }
  }

  tempSensors.begin();
  tempSensors.setResolution(10);
  tempSensors.setWaitForConversion(false);
  temp_cycle_ms = 0;

  Wire.begin();
  oled_last_init_ms = millis();
  oled_try_init(true);

  // after splash, start boot timer reference
  boot_start_ms = millis();

  // Startup blink so we know this firmware is running
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);

  unsigned long now = millis();
  last_flags_ms  = now;
  last_rudder_ms = now;
  last_current_ms = now;
  last_voltage_ms = now;
  last_temp_ms = now;
}

void loop() {
  unsigned long now = millis();

  // DS18B20 temperature service (feature-gated)
  if (feature_flags & FEATURE_TEMP_SENSOR) {
    temp_service(now);
  }
  if (!oled_ok && (now - oled_last_init_ms >= OLED_INIT_RETRY_MS)) {
    oled_last_init_ms = now;
    oled_try_init(false);
  }
  // Expire optimistic AP display override if remote didn't confirm in time
  if (ap_display_override_until_ms && now > ap_display_override_until_ms) {
    ap_display_override_until_ms = 0;
    ap_display = ap_enabled_remote;  // revert to truth
  }
  
  static unsigned long last_pi_ms = 0;
  if (now - last_pi_ms >= 200) {
    last_pi_ms = now;

    // Pi supply voltage fault detection (feature-gated)
    if (feature_flags & FEATURE_PI_VOLTAGE) {
      pi_voltage_v       = read_pi_voltage_v();
      pi_overvolt_fault  = (pi_voltage_v > PI_VOLT_HIGH_FAULT);
      pi_undervolt_fault = (pi_voltage_v < PI_VOLT_LOW_FAULT);
      pi_fault           = pi_overvolt_fault || pi_undervolt_fault;
    } else {
      // Feature disabled: suppress all Pi voltage faults
      pi_overvolt_fault  = false;
      pi_undervolt_fault = false;
      pi_fault           = false;
    }

    // Main supply (Vin) voltage fault detection (feature-gated)
    if (feature_flags & FEATURE_BATTERY_VOLTAGE) {
      vin_v          = read_voltage_v();
      // 300 ms debounce: startup inrush sags the 12V bus for ~200 ms; the fault must
      // not latch during normal motor engagement. Only a genuinely sustained undervoltage
      // (battery going flat) holds below VIN_LOW_FAULT_V for the full 300 ms window.
      // It clears immediately when voltage recovers.
      if (vin_v < VIN_LOW_FAULT_V) {
        if (vin_low_since_ms == 0) vin_low_since_ms = millis();
        vin_low_fault = ((millis() - vin_low_since_ms) >= 300UL);
      } else {
        vin_low_since_ms = 0;
        vin_low_fault    = false;
      }
      vin_high_fault = (vin_v > VIN_HIGH_FAULT_V);
    } else {
      // Feature disabled: suppress Vin over/under-voltage faults
      vin_low_fault    = false;
      vin_high_fault   = false;
      vin_low_since_ms = 0;
    }

    // Reset fault silence latch once all active faults have cleared
    if (!pi_fault && !(flags & OVERTEMP_FAULT) && !vin_low_fault && !vin_high_fault) {
      pi_fault_alarm_silenced = false;
    }
  }

  static bool ptm_prev = false;
  static bool ptm_stable_pressed = false;
  static bool ptm_last_raw = false;
  static unsigned long ptm_last_change_ms = 0;
  bool ptm_raw_pressed = (digitalRead(PTM_PIN) == LOW);
  if (ptm_raw_pressed != ptm_last_raw) {
    ptm_last_raw = ptm_raw_pressed;
    ptm_last_change_ms = now;
  }
  if (now - ptm_last_change_ms >= PTM_DEBOUNCE_MS) {
    ptm_stable_pressed = ptm_raw_pressed;
  }
  bool ptm_edge = ptm_stable_pressed && !ptm_prev;
  ptm_prev = ptm_stable_pressed;

  if (ptm_edge && (pi_fault || (flags & OVERTEMP_FAULT) || vin_low_fault || vin_high_fault)) {
    pi_fault_alarm_silenced = true;
  }

  if (ptm_edge) {
    // Two-press STOP: first press silences active alarm if any; second (or first if no alarm) is full stop
    bool alarm_was_active = (steer_loss_active && !steer_loss_silenced)
                          || ap_pressed_warn_active
                          || (comms_crit_active && !comms_fault_silenced);

    if (alarm_was_active) {
      // First press: silence alarm only, AP stays engaged
      steer_loss_silenced    = true;
      ap_pressed_warn_active = false;
      comms_fault_silenced   = true;
      show_overlay("SIL");
    } else {
      // Full emergency stop
      flags &= ~ENGAGED;
      ap_enabled_remote      = false;  // stop motor immediately — don't wait for bridge DISENGAGE
      last_command_val       = 1000;
      remote_manual_active   = false;
      steer_loss_active      = false;
      steer_loss_silenced    = false;
      ap_pressed_warn_active = false;

      // Make OLED show OFF immediately (remote truth will follow)
      ap_display = false;
      ap_display_override_until_ms = millis() + 2000UL;

      // Tell the bridge/pypilot to disable ap.enabled
      send_button_event(BTN_EVT_STOP);

      // Show big STOP overlay
      show_overlay("STOP");
    }
  }

// ----- Button ladder on A6 (B1..B5) -----
static ButtonID last_stable_button = BTN_NONE;
static ButtonID last_raw_button    = BTN_NONE;
static unsigned long btn_last_change_ms = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 60UL;

// Only read the button ladder when the feature is enabled.
// When FEATURE_ON_BOARD_BUTTONS is OFF (no buttons wired), A6 floats and
// ADC crosstalk from the preceding A2 rudder read causes false BTN_B1/B2
// detections that falsely activate the motor in HAND mode (bug fixed B27).
ButtonID raw_b;
if (feature_flags & FEATURE_ON_BOARD_BUTTONS) {
  int btn_adc = analogRead(BUTTON_ADC_PIN);
  raw_b = decode_button_from_adc(btn_adc);
} else {
  raw_b = BTN_NONE;  // no buttons wired — skip floating A6 read
}

if (raw_b != last_raw_button) {
  last_raw_button = raw_b;
  btn_last_change_ms = now;
}

ButtonID stable_b = last_stable_button;
if (now - btn_last_change_ms >= BUTTON_DEBOUNCE_MS) {
  stable_b = raw_b;
}

// Reset manual override by default; we’ll set it again below if needed
manual_override = false;
manual_dir      = 0;

// Treat AP as engaged only when remote ap.enabled is true AND Pi is alive
bool pi_alive = pi_ever_online && (now - last_pi_frame_ms <= PI_OFFLINE_TIMEOUT_MS);
bool ap_engaged = ap_enabled_remote && pi_alive;

// On a change of stable button state, act accordingly
if (stable_b != last_stable_button) {
  if (stable_b != BTN_NONE) {

    // B3 must always work (it is the AP toggle)
    if (stable_b == BTN_B3) {
      handle_button(stable_b);

    // Other buttons only send events when AP is engaged
    } else if (ap_engaged) {
      handle_button(stable_b);
    }
  }

  last_stable_button = stable_b;
}

// If AP is disengaged and not in remote MANUAL mode, use buttons as manual jog
if (!ap_engaged && !remote_manual_active) {
  if (stable_b == BTN_B1 || stable_b == BTN_B2) {
    // Define B1/B2 as Dir-A jog (positive direction)
    manual_override = true;
    manual_dir      = +1;
  } else if (stable_b == BTN_B4 || stable_b == BTN_B5) {
    // Define B4/B5 as Dir-B jog (negative direction)
    manual_override = true;
    manual_dir      = -1;
  }
}

  // Overtemp fault detection (feature-gated — only when temp sensor is enabled)
  bool temp_valid = (temp_c == temp_c) && (temp_c > -55.0f) && (temp_c < 125.0f);
  if ((feature_flags & FEATURE_TEMP_SENSOR) && temp_valid && temp_c > MAX_CONTROLLER_TEMP_C) {
    flags |= OVERTEMP_FAULT;
  } else if (!(feature_flags & FEATURE_TEMP_SENSOR)) {
    flags &= ~OVERTEMP_FAULT;  // feature disabled: suppress overtemp fault
  } else {
    flags &= ~OVERTEMP_FAULT;
  }

  // Buzzer logic (priority: steer_loss > hw_fault > comms_fault > ap_warn > silent)
  bool hw_alarm_active   = pi_fault || (flags & OVERTEMP_FAULT) || vin_low_fault || vin_high_fault;
  bool hw_alarm_silenced = pi_fault_alarm_silenced;
  bool buzzer_on = false;

  if (steer_loss_active && !steer_loss_silenced) {
    // Continuous rapid 100ms beeping for steer loss (life-safety)
    static unsigned long sl_buzz_last = 0;
    static bool sl_buzz_on = false;
    if (now - sl_buzz_last >= 100UL) {
      sl_buzz_last = now;
      sl_buzz_on = !sl_buzz_on;
    }
    buzzer_on = sl_buzz_on;
  } else if (hw_alarm_active && !hw_alarm_silenced) {
    // Rapid 100ms beeping for hardware faults
    static unsigned long hw_buzz_last = 0;
    static bool hw_buzz_on = false;
    if (now - hw_buzz_last >= 100UL) {
      hw_buzz_last = now;
      hw_buzz_on = !hw_buzz_on;
    }
    buzzer_on = hw_buzz_on;
  } else if (comms_crit_active && !comms_fault_silenced) {
    // Slow 500ms on/off (1 Hz) for comms fault — distinguishable from rapid 100ms hw alarm
    static unsigned long cf_buzz_last = 0;
    static bool cf_buzz_on = false;
    if (now - cf_buzz_last >= 500UL) {
      cf_buzz_last = now;
      cf_buzz_on = !cf_buzz_on;
    }
    buzzer_on = cf_buzz_on;
  } else if (ap_pressed_warn_active && (now < ap_warn_beep_end_ms)) {
    // Single 200ms beep for AP-pressed warning
    buzzer_on = true;
  }

  digitalWrite(BUZZER_PIN, buzzer_on ? HIGH : LOW);

  // Report buzzer state change to bridge
  if (buzzer_on != last_buzzer_state) {
    last_buzzer_state = buzzer_on;
    send_frame(BUZZER_STATE_CODE, buzzer_on ? 1 : 0);
  }

  // ---- Comms-fault rate evaluation ----
  comms_err_bucket_tick(now);

  if (err_window_sum >= COMMS_CRIT_THRESH && crit_consec_s >= COMMS_CRIT_HOLD_S) {
    comms_warn_active = true;
    comms_crit_active = true;
  } else if (err_window_sum >= COMMS_WARN_THRESH) {
    comms_warn_active = true;
    comms_crit_active = false;
  } else {
    // Rate cleared: reset latches so next fault re-arms fully
    if (comms_warn_active || comms_crit_active) {
      comms_fault_disengaged = false;
      comms_fault_silenced   = false;
    }
    comms_warn_active = false;
    comms_crit_active = false;
  }

  if (comms_warn_active) flags |= COMMS_WARN_FAULT; else flags &= ~COMMS_WARN_FAULT;
  if (comms_crit_active) flags |= COMMS_CRIT_FAULT; else flags &= ~COMMS_CRIT_FAULT;

  // Autonomous AP disengage on CRITICAL (Nano-side safety plane, ~0ms latency)
  if (comms_crit_active && !comms_fault_disengaged) {
    ap_enabled_remote      = false;
    flags                 &= ~ENGAGED;
    last_command_val       = 1000;  // neutral
    ap_display             = false;
    comms_fault_disengaged = true;
  }

  if (pi_fault || vin_low_fault || vin_high_fault) {
    flags |= BADVOLTAGE_FAULT;
  } else {
    flags &= ~BADVOLTAGE_FAULT;
  }

  if ((flags & OVERTEMP_FAULT) || pi_fault || vin_low_fault || vin_high_fault) {
    flags &= ~ENGAGED;
    last_command_val = 1000;
  }

  // --- RX: parse incoming framed packets ---
  while (Serial.available()) {
    uint8_t c = Serial.read();
    any_serial_rx = true;
    last_serial_rx_ms = now;

    if (bridge_magic_state == 0) {
      if (c == BRIDGE_MAGIC1) {
        bridge_magic_state = 1;
      }
      continue;
    } else if (bridge_magic_state == 1) {
      if (c == BRIDGE_MAGIC2) {
        bridge_magic_state = 2;
        sync_b = 0;
      } else {
        bridge_magic_state = 0;
      }
      continue;
    }

    if (sync_b < 3) {
      in_bytes[sync_b++] = c;
    } else {
      // We have three bytes in in_bytes, this is the 4th (CRC)
      uint8_t crc_rx   = c;
      uint8_t crc_calc = crc8(in_bytes, 3);
      if (crc_rx == crc_calc) {
        // CRC-valid frame
        if (in_sync_count >= 1) {
          process_packet();
          rx_good_count++;
        } else {
          in_sync_count++;
          rx_sync_count++;
        }
        flags &= ~INVALID;
      } else {
        // CRC invalid: mark INVALID and resync to magic header
        flags |= INVALID;
        in_sync_count = 0;
        rx_crc_err_count++;
        comms_err_record();  // feed sliding-window rate detector
        // Capture error detail for forwarding to bridge (latest-wins, 1-slot)
        err_detail_value   = (uint16_t)in_bytes[0] | ((uint16_t)crc_rx << 8);
        err_detail_pending = true;
      }

      sync_b = 0;
      bridge_magic_state = 0;

      // Only process one frame per loop() iteration
      break;
    }
  }

  // --- Telemetry: periodic FLAGS and RUDDER ---
  if (now - last_flags_ms >= FLAGS_PERIOD_MS) {
    uint16_t out_flags = flags;
    flags &= ~REBOOTED;           // REBOOTED appears once
    send_frame(FLAGS_CODE, out_flags);
    last_flags_ms = now;
  }

  if (now - last_rudder_ms >= RUDDER_PERIOD_MS) {

    rudder_raw = read_rudder_scaled();
    send_frame(RUDDER_SENSE_CODE, rudder_raw);
    last_rudder_ms = now;
  }

  // Current sensor telemetry (feature-gated)
  if ((feature_flags & FEATURE_CURRENT_SENSOR) && (now - last_current_ms >= CURRENT_PERIOD_MS)) {
    // Stub: report artificial current to prevent pypilot DRIVER_TIMEOUT while
    // real current sensor (ADS1115) is unavailable/miscalibrated.
    // Restore by replacing the two lines below with: float current_a = read_current_a();
    bool motor_moving = (last_command_val != 1000);
    float current_a = motor_moving ? 1.8f : 0.8f;
    int scaled = (int)(current_a * 100.0f + 0.5f);
    if (scaled < 0) {
      scaled = 0;
    } else if (scaled > 65535) {
      scaled = 65535;
    }
    send_frame(CURRENT_CODE, (uint16_t)scaled);
    last_current_ms = now;
  }

  if (now - last_voltage_ms >= VOLTAGE_PERIOD_MS) {
    float voltage_v = read_voltage_v();
    int scaled = (int)(voltage_v * 100.0f + 0.5f);
    if (scaled < 0) {
      scaled = 0;
    } else if (scaled > 65535) {
      scaled = 65535;
    }
    send_frame(VOLTAGE_CODE, (uint16_t)scaled);
    last_voltage_ms = now;
  }

  // Temperature sensor telemetry (feature-gated)
  if ((feature_flags & FEATURE_TEMP_SENSOR) && (now - last_temp_ms >= TEMP_PERIOD_SEND_MS)) {
    bool temp_valid_send = (temp_c == temp_c) && (temp_c > -55.0f) && (temp_c < 125.0f);
    if (temp_valid_send) {
      int scaled = (int)(temp_c * 100.0f + 0.5f);
      if (scaled < 0) {
        scaled = 0;
      } else if (scaled > 65535) {
        scaled = 65535;
      }
      send_frame(CONTROLLER_TEMP_CODE, (uint16_t)scaled);
    }
    last_temp_ms = now;
  }

  static unsigned long last_comms_diag_ms = 0;
  if (now - last_comms_diag_ms >= 1000UL) {
    // Pack: low byte = err_window_sum, high byte = crit_consec_s
    uint16_t diag = (uint16_t)err_window_sum | ((uint16_t)crit_consec_s << 8);
    send_frame(COMMS_DIAG_CODE, diag);
    last_comms_diag_ms = now;
  }

  // Send error detail frame to bridge (rate-limited to max 5/s)
  if (err_detail_pending && (now - err_detail_last_ms >= ERR_DETAIL_MIN_MS)) {
    send_frame(COMMS_ERR_DETAIL_CODE, err_detail_value);
    err_detail_pending = false;
    err_detail_last_ms = now;
  }

  // --- Motor + clutch control with limit logic ---
  update_motor_from_command();

  // ---- H-bridge pin-state telemetry (on-change, debug diagnostics) ----
  // Sent whenever D2/D3/D9 change — lets the bridge log exact motor on/off
  // transitions and direction without needing external instrumentation.
  // digitalRead() on D9 works correctly here because we only ever write
  // analogWrite(D9, 0) or analogWrite(D9, 255) (MIN_DUTY == MAX_DUTY == 255).
  // B26: also send MOTOR_REASON_CODE once when D9 transitions LOW->HIGH.
  {
    static uint8_t last_pin_state = 0xFF;  // 0xFF = invalid sentinel, forces first send
    uint8_t ps = (uint8_t)((digitalRead(HBRIDGE_PWM_PIN)  ? 0x04 : 0)
                          | (digitalRead(HBRIDGE_LPWM_PIN) ? 0x02 : 0)
                          | (digitalRead(HBRIDGE_RPWM_PIN) ? 0x01 : 0));
    if (ps != last_pin_state) {
      // On D9 LOW->HIGH transition: send reason code before pin-state frame
      // so the bridge receives the cause before the effect.
      bool d9_was_off = (last_pin_state != 0xFF) && !(last_pin_state & 0x04);
      bool d9_now_on  = (ps & 0x04);
      if (d9_was_off && d9_now_on) {
        send_frame(MOTOR_REASON_CODE, g_motor_reason);
      }
      send_frame(PIN_STATE_CODE, ps);
      last_pin_state = ps;
    }
  }

  // Collect one settled A2 sample into the running average.  Placed here so
  // update_motor_from_command() uses the previous iteration's published value
  // (latency ≤ one loop iteration) and all other ADC channel work is already
  // complete, guaranteeing a channel switch into A2 every call.
  service_rudder_adc();

  static unsigned long last_draw = 0;
  if (oled_ok && (now - last_draw >= 1000)) {
    last_draw = now;
    oled_draw();
  }
}


















