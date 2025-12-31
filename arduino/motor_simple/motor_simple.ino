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
#include "crc.h"    // your existing CRC-8 table + crc8()

// ---- Pins ----
const uint8_t LED_PIN          = 13;
const uint8_t RUDDER_PIN       = A4;

// IBT-2 (BTS7960) pins
const uint8_t HBRIDGE_RPWM_PIN = 2;   // RPWM
const uint8_t HBRIDGE_LPWM_PIN = 3;   // LPWM
const uint8_t HBRIDGE_PWM_PIN  = 9;   // EN (R_EN + L_EN tied together)

// Clutch pin (active-LOW: LOW = engaged)
const uint8_t CLUTCH_PIN       = 11;

// Limit switches (NC -> GND, HIGH = tripped / broken)
const uint8_t PORT_LIMIT_PIN   = 7;
const uint8_t STBD_LIMIT_PIN   = 8;

// ---- Rudder calibration (from motor_limit_test) ----
// Raw ADC counts (0..1023)
const int RUDDER_ADC_PORT_END  = 711;   // ADC at port end
const int RUDDER_ADC_STBD_END  = 153;   // ADC at starboard end
const int RUDDER_ADC_MARGIN    = 10;    // safety margin on each end

// Tolerance for centring (not used here but handy later)
const int RUDDER_CENTRE_ADC    = (RUDDER_ADC_PORT_END + RUDDER_ADC_STBD_END) / 2;
const int RUDDER_CENTRE_TOL    = 5;

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

// ---- Telemetry timing ----
unsigned long last_flags_ms  = 0;
unsigned long last_rudder_ms = 0;
const unsigned long FLAGS_PERIOD_MS  = 200; // 5 Hz
const unsigned long RUDDER_PERIOD_MS = 200; // 5 Hz

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

// Read rudder pot and scale to 0..65535 for telemetry
uint16_t read_rudder_scaled() {
  int a = analogRead(RUDDER_PIN);   // 0..1023
  rudder_adc_last = a;              // save raw for limit logic
  return (uint16_t)a * 64;          // 0..~65472
}

// ---- Limit logic helpers ----
bool port_limit_switch_hit() {
  // NC -> GND, so HIGH = open/tripped/broken
  return digitalRead(PORT_LIMIT_PIN) == HIGH;
}

bool stbd_limit_switch_hit() {
  return digitalRead(STBD_LIMIT_PIN) == HIGH;
}

// ---- Motor + clutch drive based on last_command_val & flags ----
void update_motor_from_command() {
  // Clutch: engaged whenever ENGAGED flag is set
  if (flags & ENGAGED) {
    digitalWrite(CLUTCH_PIN, LOW);   // active-LOW -> engage
  } else {
    digitalWrite(CLUTCH_PIN, HIGH);  // disengage
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
  digitalWrite(CLUTCH_PIN, HIGH);    // clutch disengaged (active-LOW)

  // Limits
  pinMode(PORT_LIMIT_PIN, INPUT_PULLUP);  // NC -> GND
  pinMode(STBD_LIMIT_PIN, INPUT_PULLUP);

  Serial.begin(38400);

  // Startup blink so we know this firmware is running
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);

  unsigned long now = millis();
  last_flags_ms  = now;
  last_rudder_ms = now;
}

void loop() {
  // --- RX: parse incoming pypilot-style frames ---
  while (Serial.available()) {
    uint8_t c = Serial.read();

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
  unsigned long now = millis();

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

  // --- Motor + clutch control with limit logic ---
  update_motor_from_command();
}
