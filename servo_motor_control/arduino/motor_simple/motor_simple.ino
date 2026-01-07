// motor_simple.ino
// Minimal pypilot-compatible motor controller core
// - 16 MHz Nano, Serial @ 38400
// - Uses crc.h (CRC-8 poly 0x31, init 0xFF)
// - Receives 4-byte frames: [code, value_lo, value_hi, crc]
// - Sends periodic FLAGS and RUDDER frames
// - Drives IBT-2 H-bridge + clutch based on COMMAND / DISENGAGE
// - Obeys limit switches + rudder pot min/max

#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "crc.h"    // your existing CRC-8 table + crc8()

// ---- Inno-Pilot version ----
const char INNOPILOT_VERSION[] = "V2c";

// Boot / online timing (user-tweakable)
const uint8_t AP_ENABLED_CODE = 0xE1;  // Bridge->Nano: ap.enabled state (0/1)
bool ap_enabled_remote = false;        // truth from Pi bridge
bool ap_display        = false;        // what OLED shows (can be briefly “optimistic”)
unsigned long ap_display_override_until_ms = 0;
const unsigned long PI_BOOT_EST_MS    = 98000UL;  // 60s estimate, tweak later
const unsigned long ONLINE_SPLASH_MS  = 3000UL;   // 3s "On-line" splash
bool pi_online_at_boot = false;

// Bridge->Nano: extra telemetry (tenths of degrees)
const uint8_t PILOT_HEADING_CODE = 0xE2; // ap.heading * 10 (uint16)
const uint8_t PILOT_COMMAND_CODE = 0xE3; // ap.heading_command * 10 (uint16)
const uint8_t PILOT_RUDDER_CODE  = 0xE4; // rudder.angle * 10 (int16, two's complement)
const uint8_t PILOT_RUDDER_PORT_LIM_CODE = 0xE5; // port limit deg * 10 (int16)
const uint8_t PILOT_RUDDER_STBD_LIM_CODE = 0xE6; // stbd limit deg * 10 (int16)

// Cached telemetry from pypilot (for OLED)
bool     pilot_heading_valid = false;
uint16_t pilot_heading_deg10 = 0;

bool     pilot_command_valid = false;
uint16_t pilot_command_deg10 = 0;

bool     pilot_rudder_valid  = false;
int16_t  pilot_rudder_deg10  = 0;

bool     pilot_port_lim_valid = false;
int16_t  pilot_port_lim_deg10 = 0;

bool     pilot_stbd_lim_valid = false;
int16_t  pilot_stbd_lim_deg10 = 0;

bool any_serial_rx = false;
unsigned long last_serial_rx_ms = 0;
bool pi_online           = false;    // true once we see first valid frame
unsigned long boot_start_ms    = 0;  // reference after splash
unsigned long pi_online_time_ms = 0; // when we first saw Pi online
// NEW: track if Pi was ever online and when we last heard from it
bool pi_ever_online       = false;
unsigned long last_pi_frame_ms = 0;
const unsigned long PI_OFFLINE_TIMEOUT_MS = 5000UL;  // 5s no frames => offline

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

// Clutch pin (active-HIGH: HIGH = engaged)
const uint8_t CLUTCH_PIN       = 11;

// Limit switches (NC -> GND, HIGH = tripped / broken)
const uint8_t PORT_LIMIT_PIN   = 7;
const uint8_t STBD_LIMIT_PIN   = 8;

const float ADC_VREF              = 5.00f;
const float VOLTAGE_SCALE         = 5.156f;

// New sensor calibration from ADC readings:
// @ 0 A  : ADC = 430 -> V0 ≈ 2.103 V
// @ 10 A : ADC = 417 -> V10 ≈ 2.039 V
// drop ≈ 0.064 V over 10 A => ~6.4 mV/A

const uint8_t ADC_SAMPLES         = 16;

const float PI_VSENSE_SCALE = 5.25f;
const float PI_VOLT_HIGH_FAULT = 5.40f;
const float PI_VOLT_LOW_FAULT  = 4.80f;
const float MAX_CONTROLLER_TEMP_C = 50.0f;

// Manual jog state (used when AP is disengaged)
bool  manual_override = false;  // true while a manual jog is active
int8_t manual_dir     = 0;      // -1 = starboard, +1 = port, 0 = none

// Current calibration points: ADC reading vs measured amps
const uint8_t  CURR_N = 8;
const uint16_t CURR_ADC[CURR_N] = {430, 427, 426, 424, 423, 422, 420, 417};
const float    CURR_A[CURR_N]   = {0.0f, 0.11f, 0.65f, 1.70f, 2.23f, 2.33f, 4.00f, 5.30f};

// ---- DS18B20 scheduling ----
const unsigned long TEMP_PERIOD_MS = 1000;
const unsigned long TEMP_CONV_MS = 200;

// ---- OLED ----
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR  0x3C

OneWire oneWire(PIN_DS18B20);
DallasTemperature tempSensors(&oneWire);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oled_ok = false;

// ---- Rudder calibration (from motor_limit_test) ----
// Raw ADC counts (0..1023)
const int RUDDER_ADC_PORT_END  = 711;   // ADC at port end
const int RUDDER_ADC_STBD_END  = 153;   // ADC at starboard end
const int RUDDER_ADC_MARGIN    = 10;    // safety margin on each end
const int RUDDER_ADC_END_HYST  = 8;    // extra counts to CLEAR end-latch (prevents flicker)

// Tolerance for centring (not used here but handy later)
const int RUDDER_CENTRE_ADC    = (RUDDER_ADC_PORT_END + RUDDER_ADC_STBD_END) / 2;
const int RUDDER_CENTRE_TOL    = 5;

// Rudder angle display range (approx ±40°)
const float RUDDER_RANGE_DEG = 40.0f;

// Enable/disable physical limit switches on D7/D8. (Optional)
// true  -> use NC limit switches on PORT_LIMIT_PIN / STBD_LIMIT_PIN.
// false -> ignore switch pins, use only pot-based soft limits.
const bool LIMIT_SWITCHES_ACTIVE = false;

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
  EEPROM_READ_CODE          = 0x91,
  EEPROM_WRITE_CODE         = 0x53,
  CLUTCH_PWM_AND_BRAKE_CODE = 0x36
};

// ---- Forward declarations ----
uint16_t read_rudder_scaled();
bool port_limit_switch_hit();
bool stbd_limit_switch_hit();

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
  PORT_PIN_FAULT      = 0x0020,
  STARBOARD_PIN_FAULT = 0x0040,
  BADVOLTAGE_FAULT    = 0x0080,
  MIN_RUDDER_FAULT    = 0x0100,   // starboard end
  MAX_RUDDER_FAULT    = 0x0200,   // port end
  CURRENT_RANGE       = 0x0400,
  BAD_FUSES           = 0x0800,
  REBOOTED            = 0x8000
};

// Button IDs from the A6 resistor ladder
enum ButtonID {
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

// ---- State / telemetry ----
uint16_t flags            = REBOOTED;   // reported once then cleared
uint16_t last_command_val = 1000;       // 0..2000, 1000 = neutral
uint16_t rudder_raw       = 0;          // 0..65535 (scaled)
int      rudder_adc_last  = 0;          // 0..1023

float pi_voltage_v = 0.0f;
bool  pi_overvolt_fault = false;
bool  pi_undervolt_fault = false;
bool  pi_fault = false;
bool  pi_fault_alarm_silenced = false;

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

// Helper: send a 4-byte frame [code, value_lo, value_hi, crc]
void send_frame(uint8_t code, uint16_t value) {
  uint8_t body[3];
  body[0] = code;
  body[1] = value & 0xFF;
  body[2] = (value >> 8) & 0xFF;
  uint8_t c = crc8(body, 3);
  Serial.write(body, 3);
  Serial.write(c);
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
  if (adc >= CURR_ADC[0]) {
    return CURR_A[0];
  }
  // Below or equal to lowest calibration → clamp to max
  if (adc <= CURR_ADC[CURR_N - 1]) {
    return CURR_A[CURR_N - 1];
  }

  // Find the segment CURR_ADC[i] >= adc >= CURR_ADC[i+1]
  for (uint8_t i = 0; i < CURR_N - 1; i++) {
    uint16_t a0 = CURR_ADC[i];
    uint16_t a1 = CURR_ADC[i + 1];

    if (adc <= a0 && adc >= a1) {
      float x0 = (float)a0;
      float x1 = (float)a1;
      float y0 = CURR_A[i];
      float y1 = CURR_A[i + 1];

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

void oled_print_right(uint8_t y, const char* text) {
  uint8_t len = (uint8_t)strlen(text);
  int16_t x = SCREEN_WIDTH - (int16_t)(len * 6);
  if (x < 0) {
    x = 0;
  }
  display.setCursor(x, y);
  display.print(text);
}

void oled_draw() {
  if (!oled_ok) {
    return;
  }

  const unsigned long now = millis();

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

  // Rudder angle (rough local mapping for now)
  float rudder_deg = 0.0f;
  {
    float centre   = 0.5f * (RUDDER_ADC_PORT_END + RUDDER_ADC_STBD_END);
    float halfSpan = 0.5f * (RUDDER_ADC_PORT_END - RUDDER_ADC_STBD_END);
    if (halfSpan < 1.0f) halfSpan = 1.0f;
    rudder_deg = (rudder_adc_last - centre) * (RUDDER_RANGE_DEG / halfSpan);
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

  // ----------------------------
  // Draw
  // ----------------------------
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Line 1: permanent controller status
  display.setCursor(0, 0);

  if (within_boot_window) {
    uint8_t pct = (uint8_t)((elapsed_boot * 100UL) / PI_BOOT_EST_MS);
    if (pct > 100) pct = 100;
    // tight formatting so it fits: "Inno-Cntl:Booting100%"
    display.print(F("Inno-Cntl:Booting"));
    display.print(pct);
    display.print(F("%"));
  } else if (boot_offline || pi_timed_out) {
    display.print(F("Inno-Cntl: Off-line"));
  } else {
    display.print(F("Inno-Cntl: On-line"));
  }

  // Overlay (big transient button feedback) – leave bottom line off during overlay
  if (overlay_active) {
    if (now - overlay_start_ms < OVERLAY_DURATION_MS) {
      display.setTextSize(3);

      uint8_t len = strlen(overlay_text);
      int16_t char_w = 6 * 3;
      int16_t text_w = len * char_w;
      int16_t x = (SCREEN_WIDTH - text_w) / 2;
      if (x < 0) x = 0;

      // below the status line
      int16_t y = 20;
      display.setCursor(x, y);
      display.println(overlay_text);

      display.display();
      return;
    } else {
      overlay_active = false;
    }
  }

  // Always keep V/A/C pinned to the bottom (even during boot/offline)
  display.setCursor(0, 52);
  display.print(vbuf);
  display.print(F("  "));
  display.print(ibuf);
  display.print(F("  "));
  display.print(tbuf);

  // If booting/offline, don’t shuffle other lines around—just show bottom line.
  if (within_boot_window || boot_offline || pi_timed_out) {
    display.display();
    return;
  }

  // Online-only info at fixed positions (no jumping)
  // y=12: faults (optional)
  const uint8_t LINE2_Y = 12;
  if (pi_fault || (flags & OVERTEMP_FAULT)) {
    display.setCursor(0, 12);
    display.print(F("FAULT: "));
    if (pi_overvolt_fault) {
      display.print(F("PiV HIGH"));
    } else if (pi_undervolt_fault) {
      display.print(F("PiV LOW"));
    } else if (flags & OVERTEMP_FAULT) {
      display.print(F("TEMP"));
    }
  }

  // y=24: AP + clutch
  display.setCursor(0, 24);
  display.print(F("AP: "));
  display.print(ap_display ? F("ON") : F("Off"));
  display.print(F("  Clutch: "));
  display.print(digitalRead(CLUTCH_PIN) == HIGH ? F("ON") : F("OFF"));

  // Heading / Command
  display.setCursor(0, LINE2_Y + 20);
  display.print(F("Head: "));
  if (pilot_heading_valid) display.print(pilot_heading_deg10 / 10.0f, 1);
  else display.print(F("--.-"));
  display.print(F(" Cmd: "));
  if (pilot_command_valid) display.print(pilot_command_deg10 / 10.0f, 1);
  else display.print(F("--.-"));

  // Rudder compare (Nano vs Pypilot)
  display.setCursor(0, LINE2_Y + 30);
  display.print(F("Rud N:"));
  display.print(rudder_deg, 1);
  display.print(F(" P:"));
  if (pilot_rudder_valid) display.print(pilot_rudder_deg10 / 10.0f, 1);
  else display.print(F("--.-"));

  display.display();
}

// Read rudder pot and scale to 0..65535 for telemetry
uint16_t read_rudder_scaled() {
  int a = analogRead(RUDDER_PIN);   // 0..1023
  rudder_adc_last = a;              // save raw for limit logic
  return (uint16_t)a * 64;          // 0..~65472
}

// ---- Limit logic helpers ----
// V2: if LIMIT_SWITCHES_ACTIVE is false, ignore the switch pins completely.
bool port_limit_switch_hit() {
  if (!LIMIT_SWITCHES_ACTIVE) {
    return false;
  }
  // NC -> GND, so HIGH = open/tripped/broken
  return digitalRead(PORT_LIMIT_PIN) == HIGH;
}

bool stbd_limit_switch_hit() {
  if (!LIMIT_SWITCHES_ACTIVE) {
    return false;
  }
  return digitalRead(STBD_LIMIT_PIN) == HIGH;
}

// ---- Motor + clutch drive based on last_command_val & flags ----
void update_motor_from_command() {
  // Always update rudder ADC for limit logic
  int a = analogRead(RUDDER_PIN);   // 0..1023
  rudder_adc_last = a;
  
  unsigned long now = millis();
  bool pi_alive = pi_ever_online && (now - last_pi_frame_ms <= PI_OFFLINE_TIMEOUT_MS);
  bool ap_active = ap_enabled_remote && pi_alive;

  // Clutch engages when AP is enabled remotely OR manual jog is active.
  // Safety: clutch forced OFF during pi_fault or overtemp.
  bool clutch_should = (manual_override || ap_active) &&
                       !pi_fault &&
                       !(flags & OVERTEMP_FAULT);

  digitalWrite(CLUTCH_PIN, clutch_should ? HIGH : LOW);

  // If in a fault state, don't drive the motor at all
  if (pi_fault || (flags & OVERTEMP_FAULT)) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // Enter thresholds (near the ends)
  bool at_port_enter = port_limit_switch_hit() ||
                       (a >= (RUDDER_ADC_PORT_END - RUDDER_ADC_MARGIN));

  bool at_stbd_enter = stbd_limit_switch_hit() ||
                       (a <= (RUDDER_ADC_STBD_END + RUDDER_ADC_MARGIN));

  // Exit thresholds (must move further away before we clear the latch)
  // PORT end is high ADC: we "hold" port-end while ADC >= port_exit
  // STBD end is low  ADC: we "hold" stbd-end while ADC <= stbd_exit
  int port_exit = RUDDER_ADC_PORT_END - (RUDDER_ADC_MARGIN + RUDDER_ADC_END_HYST);
  int stbd_exit = RUDDER_ADC_STBD_END + (RUDDER_ADC_MARGIN + RUDDER_ADC_END_HYST);

  bool at_port_hold = port_limit_switch_hit() || (a >= port_exit);
  bool at_stbd_hold = stbd_limit_switch_hit() || (a <= stbd_exit);

  // Latch ensures only one end is active; hysteresis prevents flicker/reset on jitter.
  static uint8_t end_latch = 0;  // 0 none, 1 port, 2 stbd

  switch (end_latch) {
    case 0:
      if (at_port_enter && !at_stbd_enter) {
        end_latch = 1;
      } else if (at_stbd_enter && !at_port_enter) {
        end_latch = 2;
      } else if (at_port_enter && at_stbd_enter) {
        // If both appear "true" (noise / overlap), pick the closer end.
        int dist_port = abs(a - RUDDER_ADC_PORT_END);
        int dist_stbd = abs(a - RUDDER_ADC_STBD_END);
        end_latch = (dist_port <= dist_stbd) ? 1 : 2;
      }
      break;

    case 1:
      // Stay latched until we've moved away past the EXIT threshold
      if (!at_port_hold) end_latch = 0;
      break;

    case 2:
      if (!at_stbd_hold) end_latch = 0;
      break;
  }

  bool at_port_end = (end_latch == 1) && at_port_hold;
  bool at_stbd_end = (end_latch == 2) && at_stbd_hold;

  // Update rudder fault flags (as before)
  if (at_port_end) {
    flags |= MAX_RUDDER_FAULT;
  } else {
    flags &= ~MAX_RUDDER_FAULT;
  }

  if (at_stbd_end) {
    flags |= MIN_RUDDER_FAULT;
  } else {
    flags &= ~MIN_RUDDER_FAULT;
  }

  // ---- Manual override branch (AP disengaged) ----
  if (manual_override) {
    // Respect limits
    if (manual_dir > 0 && at_port_end) {
      // Trying to jog further to port, but at/near port end → stop
      analogWrite(HBRIDGE_PWM_PIN, 0);
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      return;
    }
    if (manual_dir < 0 && at_stbd_end) {
      // Trying to jog further to stbd, but at/near stbd end → stop
      analogWrite(HBRIDGE_PWM_PIN, 0);
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      return;
    }

    const uint8_t duty = 255;  // full duty for now; you can tune later

    if (manual_dir > 0) {
      // PORT: increase ADC
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, HIGH);
    } else if (manual_dir < 0) {
      // STBD: decrease ADC
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      digitalWrite(HBRIDGE_RPWM_PIN, HIGH);
    } else {
      // No direction -> stop
      analogWrite(HBRIDGE_PWM_PIN, 0);
      digitalWrite(HBRIDGE_RPWM_PIN, LOW);
      digitalWrite(HBRIDGE_LPWM_PIN, LOW);
      return;
    }

    analogWrite(HBRIDGE_PWM_PIN, duty);
    return;  // do not fall through to autopilot logic
  }

  // ---- Existing autopilot logic below (unchanged) ----

  // If AP is not active (remote ap.enabled false or Pi offline), don't drive motor
  if (!ap_active) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // ----- Autopilot motor drive from last_command_val -----
  // last_command_val: 0..2000, 1000 = stop
  int16_t delta = (int16_t)last_command_val - 1000;   // -1000..+1000

  // Deadband around neutral
  const int16_t DEADBAND = 20;
  if (delta > -DEADBAND && delta < DEADBAND) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // Don't drive further into soft limits
  if (delta > 0 && at_port_end) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }
  if (delta < 0 && at_stbd_end) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
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
    int16_t span = 1000 - DEADBAND;
    if (span < 1) span = 1;
    int16_t effective = abs_delta - DEADBAND;
    if (effective < 0) effective = 0;

    duty = MIN_DUTY + (uint8_t)((effective * (MAX_DUTY - MIN_DUTY)) / span);
  }

  // Direction: delta > 0 => PORT (increase ADC), delta < 0 => STBD (decrease ADC)
  if (delta > 0) {
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, HIGH);
  } else {
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    digitalWrite(HBRIDGE_RPWM_PIN, HIGH);
  }

  analogWrite(HBRIDGE_PWM_PIN, duty);
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
      break;

    case DISENGAGE_CODE:
      flags &= ~ENGAGED;
      break;

    case RESET_CODE:
      flags &= ~OVERCURRENT_FAULT;
      break;

    case AP_ENABLED_CODE: {
      bool en = (value != 0);
      ap_enabled_remote = en;
      if (en) {
        flags |= ENGAGED;
      } else {
        flags &= ~ENGAGED;
        last_command_val = 1000;  // neutral when AP disabled
      }
      
      // If we are not in an override window, follow remote immediately.
      // If we ARE overriding (user just pressed B3), cancel override once remote matches.
      if (ap_display_override_until_ms == 0) {
        ap_display = en;
      } else if (ap_display == en) {
        ap_display_override_until_ms = 0;
      }
      break;
    }
    
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

    case PILOT_RUDDER_PORT_LIM_CODE:
      pilot_port_lim_deg10 = (int16_t)value;
      pilot_port_lim_valid = true;
      break;
    
    case PILOT_RUDDER_STBD_LIM_CODE:
      pilot_stbd_lim_deg10 = (int16_t)value;
      pilot_stbd_lim_valid = true;
      break;

    
    default:
      // Unhandled commands ignored for now
      break;
  }
}

void setup() {
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

  // Clutch
  pinMode(CLUTCH_PIN, OUTPUT);
  digitalWrite(CLUTCH_PIN, LOW);    // clutch disengaged (active-HIGH)

  // Limits
  if (LIMIT_SWITCHES_ACTIVE) {
    pinMode(PORT_LIMIT_PIN, INPUT_PULLUP);  // NC -> GND
    pinMode(STBD_LIMIT_PIN, INPUT_PULLUP);
  }

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
  oled_ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oled_ok) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Splash: big Inno-Pilot
    display.setTextSize(2);
    display.setCursor(0, 10);
    display.println(F("Inno-Pilot"));

    display.setTextSize(1);
    display.setCursor(30, 36);
    display.print(F("Version "));
    display.println(INNOPILOT_VERSION);

    display.display();
    // Only block for the full splash if the Pi doesn't appear online yet
    if (!pi_online_at_boot) {
      delay(3000);   // Pi booting: OK to block
    }
}

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

  temp_service(now);
  // Expire optimistic AP display override if remote didn't confirm in time
  if (ap_display_override_until_ms && now > ap_display_override_until_ms) {
    ap_display_override_until_ms = 0;
    ap_display = ap_enabled_remote;  // revert to truth
  }
  
  static unsigned long last_pi_ms = 0;
  if (now - last_pi_ms >= 200) {
    last_pi_ms = now;
    pi_voltage_v = read_pi_voltage_v();
    pi_overvolt_fault = (pi_voltage_v > PI_VOLT_HIGH_FAULT);
    pi_undervolt_fault = (pi_voltage_v < PI_VOLT_LOW_FAULT);
    pi_fault = pi_overvolt_fault || pi_undervolt_fault;
    if (!pi_fault) {
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

  if (ptm_edge && pi_fault) {
    pi_fault_alarm_silenced = true;
  }

  if (ptm_edge) {
    // Emergency stop: force AP off
    flags &= ~ENGAGED;
    last_command_val = 1000;

    // Make OLED show OFF immediately (remote truth will follow)
    ap_display = false;
    ap_display_override_until_ms = millis() + 2000UL;

    // Tell the bridge/pypilot to disable ap.enabled
    send_button_event(BTN_EVT_STOP);

    // Show big STOP overlay
    show_overlay("STOP");
  }

// ----- Button ladder on A6 (B1..B5) -----
static ButtonID last_stable_button = BTN_NONE;
static ButtonID last_raw_button    = BTN_NONE;
static unsigned long btn_last_change_ms = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 60UL;

int btn_adc = analogRead(BUTTON_ADC_PIN);
ButtonID raw_b = decode_button_from_adc(btn_adc);

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

// If AP is disengaged, use buttons as manual jog
if (!ap_engaged) {
  if (stable_b == BTN_B1 || stable_b == BTN_B2) {
    // Define B1/B2 as starboard jog (negative direction)
    manual_override = true;
    manual_dir      = -1;
  } else if (stable_b == BTN_B4 || stable_b == BTN_B5) {
    // Define B4/B5 as port jog (positive direction)
    manual_override = true;
    manual_dir      = +1;
  }
}

  bool temp_valid = (temp_c == temp_c) && (temp_c > -55.0f) && (temp_c < 125.0f);
  if (temp_valid && temp_c > MAX_CONTROLLER_TEMP_C) {
    flags |= OVERTEMP_FAULT;
  } else {
    flags &= ~OVERTEMP_FAULT;
  }

  bool alarm_active = pi_fault || (flags & OVERTEMP_FAULT);
  bool alarm_silenced = pi_fault_alarm_silenced && !(flags & OVERTEMP_FAULT);
  if (alarm_active && !alarm_silenced) {
    static unsigned long buzz_last = 0;
    static bool buzz_on = false;
    unsigned long period = buzz_on ? 500UL : 250UL;
    if (now - buzz_last >= period) {
      buzz_last = now;
      buzz_on = !buzz_on;
      digitalWrite(BUZZER_PIN, buzz_on ? HIGH : LOW);
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  if (pi_fault) {
    flags |= BADVOLTAGE_FAULT;
  } else {
    flags &= ~BADVOLTAGE_FAULT;
  }

  if ((flags & OVERTEMP_FAULT) || pi_fault) {
    flags &= ~ENGAGED;
    last_command_val = 1000;
  }

  // --- RX: parse incoming pypilot-style frames ---
  while (Serial.available()) {
    uint8_t c = Serial.read();
    any_serial_rx = true;
    last_serial_rx_ms = now;

    if (sync_b < 3) {
      in_bytes[sync_b++] = c;
    } else {
      // We have three bytes in in_bytes, this is the 4th (CRC)
      uint8_t crc_rx   = c;
      uint8_t crc_calc = crc8(in_bytes, 3);
      if (crc_rx == crc_calc) {
        // CRC-valid frame
        if (in_sync_count >= 2) {
          process_packet();
        } else {
          in_sync_count++;
        }
        sync_b = 0;
        flags &= ~INVALID;
      } else {
        // CRC invalid: mark INVALID, shift window by 1 byte
        flags |= INVALID;
        in_sync_count = 0;
        in_bytes[0] = in_bytes[1];
        in_bytes[1] = in_bytes[2];
        in_bytes[2] = c;
      }

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

  if (now - last_current_ms >= CURRENT_PERIOD_MS) {
    float current_a = read_current_a();
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

  if (now - last_temp_ms >= TEMP_PERIOD_SEND_MS) {
    bool temp_valid = (temp_c == temp_c) && (temp_c > -55.0f) && (temp_c < 125.0f);
    if (temp_valid) {
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

  // --- Motor + clutch control with limit logic ---
  update_motor_from_command();

  static unsigned long last_draw = 0;
  if (oled_ok && (now - last_draw >= 200)) {
    last_draw = now;
    oled_draw();
  }
}








