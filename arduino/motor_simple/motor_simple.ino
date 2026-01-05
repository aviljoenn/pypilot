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
const char INNOPILOT_VERSION[] = "V2";

// Boot / online timing (user-tweakable)
const unsigned long PI_BOOT_EST_MS    = 60000UL;  // 60s estimate, tweak later
const unsigned long ONLINE_SPLASH_MS  = 3000UL;   // 3s "On-line" splash

bool any_serial_rx = false;
unsigned long last_serial_rx_ms = 0;
bool pi_online           = false;    // true once we see first valid frame
unsigned long boot_start_ms    = 0;  // reference after splash
unsigned long pi_online_time_ms = 0; // when we first saw Pi online

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

// Tolerance for centring (not used here but handy later)
const int RUDDER_CENTRE_ADC    = (RUDDER_ADC_PORT_END + RUDDER_ADC_STBD_END) / 2;
const int RUDDER_CENTRE_TOL    = 5;

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

  unsigned long now = millis();

  // Measurements
  float vin        = read_voltage_v();
  float ia_instant = read_current_a();
  float ia         = smooth_current_for_display(ia_instant);
  float piv        = pi_voltage_v;  // maintained in loop

  bool temp_valid  = (temp_c == temp_c) && (temp_c > -55.0f) && (temp_c < 125.0f);

  // Boot/online state
  unsigned long elapsed       = now - boot_start_ms;
  bool show_online            = pi_online && (now - pi_online_time_ms < ONLINE_SPLASH_MS);
  bool pi_offline             = !pi_online;
  bool in_boot_window         = pi_offline && (elapsed < PI_BOOT_EST_MS);
  bool offline_timeout        = pi_offline && (elapsed >= PI_BOOT_EST_MS);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // ----- Header: "Inno-Pilot V2" in bold-ish -----
  display.setCursor(0, 0);
  display.print(F("Inno-Pilot "));
  display.print(INNOPILOT_VERSION);

  // Draw again 1px lower to fake bold
  display.setCursor(0, 1);
  display.print(F("Inno-Pilot "));
  display.print(INNOPILOT_VERSION);

  const uint8_t LINE2_Y = 10;   // Y for second logical line

  // ----- Online splash (Pi controller online) -----
  if (show_online) {
    display.setCursor(0, LINE2_Y);
    display.println(F("Inno-Controller"));

    display.setCursor(0, LINE2_Y + 10);
    display.println(F("On-line..."));

    display.display();
    return;
  }

  // ----- Booting indication (Pi offline, within boot estimate) -----
  if (in_boot_window) {
    uint8_t pct = (uint8_t)((elapsed * 100UL) / PI_BOOT_EST_MS);

    display.setCursor(0, LINE2_Y);
    display.println(F("Inno-Controller"));

    display.setCursor(0, LINE2_Y + 10);
    display.print(F("Booting "));
    display.print(pct);
    display.println(F("%"));

    display.display();
    return;
  }

  // ----- Controller offline (boot time exceeded, still no Pi) -----
  if (offline_timeout) {
    display.setCursor(0, LINE2_Y);
    display.println(F("Inno-Controller"));

    display.setCursor(0, LINE2_Y + 10);
    display.println(F("OFFLINE"));

    // Still show Vin/I so user sees life
    display.setCursor(0, LINE2_Y + 20);
    display.print(F("Vin: "));
    display.print(vin, 1);
    display.print(F("V  I: "));
    display.print(ia, 2);
    display.print(F("A"));

    display.display();
    return;
  }

  // ----- Normal telemetry display (Pi online, no online splash) -----

  // Fault line (Pi voltage and/or overtemp)
  if (pi_fault || (flags & OVERTEMP_FAULT)) {
    display.setCursor(0, LINE2_Y);
    display.print(F("FAULT: "));
    if (pi_overvolt_fault) {
      display.print(F("PiV HIGH"));
    } else if (pi_undervolt_fault) {
      display.print(F("PiV LOW"));
    } else if (flags & OVERTEMP_FAULT) {
      display.print(F("TEMP"));
    }
  }

  // Engaged / Clutch
  display.setCursor(0, LINE2_Y + 10);
  display.print(F("Eng: "));
  display.print((flags & ENGAGED) ? F("YES") : F("NO"));
  display.print(F("  Cl: "));
  display.print(digitalRead(CLUTCH_PIN) == HIGH ? F("ON") : F("OFF"));

  // Main Vin & current
  display.setCursor(0, LINE2_Y + 20);
  display.print(F("Vin: "));
  display.print(vin, 1);
  display.print(F("V  I: "));
  display.print(ia, 2);
  display.print(F("A"));

  // Controller temperature
  display.setCursor(0, LINE2_Y + 30);
  display.print(F("CtlT: "));
  if (temp_valid) {
    display.print(temp_c, 1);
    display.print(F("C"));
  } else {
    display.print(F("--.-C"));
  }

  // Pi 5V rail
  display.setCursor(0, LINE2_Y + 40);
  display.print(F("PiV: "));
  display.print(piv, 2);
  display.print(F("V"));

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
  // Clutch: engaged whenever ENGAGED flag is set
  if (flags & ENGAGED) {
    digitalWrite(CLUTCH_PIN, HIGH);   // active-HIGH -> engage
  } else {
    digitalWrite(CLUTCH_PIN, LOW);  // disengage
  }

  // Always update rudder ADC for limit logic
  int a = analogRead(RUDDER_PIN);   // 0..1023
  rudder_adc_last = a;

  // Determine end-of-travel based on ADC and limit switches
  bool at_port_end = port_limit_switch_hit() ||
                     (a >= (RUDDER_ADC_PORT_END - RUDDER_ADC_MARGIN));
  bool at_stbd_end = stbd_limit_switch_hit() ||
                     (a <= (RUDDER_ADC_STBD_END + RUDDER_ADC_MARGIN));

  // Update rudder fault flags
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

  // If not engaged, stop motor
  if (!(flags & ENGAGED)) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // Map last_command_val 0..2000 (1000 = neutral) to direction + duty
  int16_t delta    = (int16_t)last_command_val - 1000;   // -1000..+1000
  const int16_t deadband = 20;                           // small neutral zone

  // Stop if within deadband
  if (delta > -deadband && delta < deadband) {
    analogWrite(HBRIDGE_PWM_PIN, 0);
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // Gating: don't drive further into limits
  if (delta > 0 && at_port_end) {
    // Trying to move PORT but at/near port end -> stop
    analogWrite(HBRIDGE_PWM_PIN, 0);
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  if (delta < 0 && at_stbd_end) {
    // Trying to move STBD but at/near stbd end -> stop
    analogWrite(HBRIDGE_PWM_PIN, 0);
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    return;
  }

  // Compute PWM duty:
  // - Use MIN_DUTY as the slowest commanded speed that still moves the motor
  // - Use MAX_DUTY = 255 as full speed
  // - Map |delta| in [deadband..1000] to [MIN_DUTY..MAX_DUTY]
  const uint8_t MIN_DUTY = 255;
  const uint8_t MAX_DUTY = 255;

  int16_t abs_delta = delta > 0 ? delta : -delta;
  if (abs_delta > 1000) abs_delta = 1000;

  uint8_t duty;
  if (abs_delta <= deadband) {
    duty = MIN_DUTY;  // safety fallback
  } else {
    int16_t span      = 1000 - deadband;
    int16_t effective = abs_delta - deadband;   // 0..span
    duty = MIN_DUTY + (uint8_t)((effective * (MAX_DUTY - MIN_DUTY)) / span);
  }

  // Direction:
  // From your tests: ADC at port end is HIGHER than at stbd end.
  if (delta > 0) {
    // Positive: move toward PORT (increase ADC)
    digitalWrite(HBRIDGE_RPWM_PIN, LOW);
    digitalWrite(HBRIDGE_LPWM_PIN, HIGH);
  } else {
    // Negative: move toward STBD (decrease ADC)
    digitalWrite(HBRIDGE_LPWM_PIN, LOW);
    digitalWrite(HBRIDGE_RPWM_PIN, HIGH);
  }

  analogWrite(HBRIDGE_PWM_PIN, duty);
}

// Process one CRC-valid frame
void process_packet() {
  flags |= SYNC;

  // Mark Pi controller online on first valid frame
  if (!pi_online) {
    pi_online = true;
    pi_online_time_ms = millis();
  }

  uint16_t value = in_bytes[1] | (in_bytes[2] << 8);
  uint8_t  code  = in_bytes[0];

  switch (code) {
    case COMMAND_CODE:
      // 0..2000, 1000 = neutral
      last_command_val = value;
      if (value != 1000) {
        flags |= ENGAGED;
      }
      break;

    case DISENGAGE_CODE:
      flags &= ~ENGAGED;
      break;

    case RESET_CODE:
      flags &= ~OVERCURRENT_FAULT;
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
    display.setCursor(10, 10);
    display.println(F("Inno-Pilot"));

    display.setTextSize(1);
    display.setCursor(36, 36);
    display.print(F("Version "));
    display.println(INNOPILOT_VERSION);

    display.display();
    delay(3000);   // splash for 3 seconds (was 1.5s)
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
    flags &= ~ENGAGED;
    last_command_val = 1000;
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
















