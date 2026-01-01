#define SSD1306_NO_SPLASH

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ==============================
// Hardware pinout (matches IBT-2 Nano wiring)
// ==============================
const uint8_t PIN_RPWM   = 2;   // D2 -> RPWM (ON/OFF)
const uint8_t PIN_LPWM   = 3;   // D3 -> LPWM (ON/OFF)
const uint8_t PIN_EN     = 9;   // D9 -> R_EN + L_EN bridged
const uint8_t PIN_CLUTCH = 11;  // D11 -> clutch driver input
const uint8_t PIN_PTM    = 4;   // D4 -> PTM push-button (to GND, INPUT_PULLUP)

// Rudder position sensor
const uint8_t PIN_RUDDER = A2;  // potentiometer feedback input

// ==============================
// OLED
// ==============================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oled_ok = false;

// ==============================
// Control constants
// ==============================
const uint16_t RUDDER_MIN_LIMIT = 0;
const uint16_t RUDDER_MAX_LIMIT = 1023;
const uint8_t ADC_SAMPLES = 8;
const uint8_t CHANGE_THRESHOLD = 1;     // minimum delta to consider movement
const uint16_t CENTER_TOLERANCE = 2;    // stop within +/- this value
const unsigned long STALL_TIMEOUT_MS = 700;
const unsigned long DISPLAY_PERIOD_MS = 200;
const unsigned long TELEMETRY_PERIOD_MS = 250;
const unsigned long PTM_HOLD_MS = 3000;

// ==============================
// State
// ==============================
enum MotionState : uint8_t { STOPPED = 0, MOVING_NEG = 1, MOVING_POS = 2 };
enum RunState : uint8_t { WAIT_START = 0, FIND_LIMIT_A = 1, FIND_LIMIT_B = 2, CENTERING = 3, DONE = 4, ERROR = 5 };

MotionState motion = STOPPED;
RunState run_state = WAIT_START;

uint16_t last_reading = 0;
unsigned long last_change_ms = 0;
bool last_reading_valid = false;

uint16_t min_reading = 1023;
uint16_t max_reading = 0;
uint16_t center_target = 512;
uint16_t limit_a = 0;
uint16_t limit_b = 0;
bool limit_a_valid = false;
bool limit_b_valid = false;

unsigned long last_display_ms = 0;
unsigned long last_telemetry_ms = 0;
unsigned long ptm_press_start_ms = 0;

String last_event = "";

// ==============================
// Motor and clutch helpers
// ==============================
void clutch_on() {
  digitalWrite(PIN_CLUTCH, HIGH);
}

void clutch_off() {
  digitalWrite(PIN_CLUTCH, LOW);
}

void motor_stop() {
  digitalWrite(PIN_EN, LOW);
  digitalWrite(PIN_RPWM, LOW);
  digitalWrite(PIN_LPWM, LOW);
  motion = STOPPED;
}

void motor_move_negative() {
  digitalWrite(PIN_LPWM, LOW);
  digitalWrite(PIN_RPWM, HIGH);
  digitalWrite(PIN_EN, HIGH);
  motion = MOVING_NEG;
}

void motor_move_positive() {
  digitalWrite(PIN_RPWM, LOW);
  digitalWrite(PIN_LPWM, HIGH);
  digitalWrite(PIN_EN, HIGH);
  motion = MOVING_POS;
}

// ==============================
// ADC helpers
// ==============================
uint16_t read_rudder() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    sum += (uint16_t)analogRead(PIN_RUDDER);
    delayMicroseconds(200);
  }
  return (uint16_t)(sum / ADC_SAMPLES);
}

// ==============================
// Display and telemetry
// ==============================
const __FlashStringHelper *motion_str(MotionState m) {
  switch (m) {
    case MOVING_NEG: return F("NEG");
    case MOVING_POS: return F("POS");
    default: return F("STOP");
  }
}

const __FlashStringHelper *state_str(RunState s) {
  switch (s) {
    case WAIT_START: return F("WAIT PTM");
    case FIND_LIMIT_A: return F("FIND A");
    case FIND_LIMIT_B: return F("FIND B");
    case CENTERING: return F("CENTER");
    case DONE:      return F("DONE");
    default:        return F("ERROR");
  }
}

void update_display(uint16_t reading) {
  if (!oled_ok) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(F("State: "));
  display.print(state_str(run_state));

  display.setCursor(0, 10);
  display.print(F("Motion: "));
  display.print(motion_str(motion));

  display.setCursor(0, 20);
  display.print(F("Rudder: "));
  display.print(reading);

  display.setCursor(0, 30);
  display.print(F("Min: "));
  display.print(min_reading);
  display.print(F(" Max: "));
  display.print(max_reading);

  display.setCursor(0, 40);
  display.print(F("Center: "));
  display.print(center_target);

  display.setCursor(0, 50);
  display.print(F("Event: "));
  display.print(last_event);

  display.display();
}

void telemetry(uint16_t reading) {
  Serial.print(F("state="));
  Serial.print(state_str(run_state));
  Serial.print(F(" motion="));
  Serial.print(motion_str(motion));
  Serial.print(F(" rudder="));
  Serial.print(reading);
  Serial.print(F(" min="));
  Serial.print(min_reading);
  Serial.print(F(" max="));
  Serial.print(max_reading);
  Serial.print(F(" center="));
  Serial.print(center_target);
  if (last_event.length()) {
    Serial.print(F(" event="));
    Serial.print(last_event);
  }
  Serial.println();
}

void setup() {
  Serial.begin(38400);

  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_CLUTCH, OUTPUT);
  pinMode(PIN_RUDDER, INPUT);
  pinMode(PIN_PTM, INPUT_PULLUP);

  motor_stop();
  clutch_off();

  oled_ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oled_ok) {
    display.clearDisplay();
    display.display();
  }

  last_event = "BOOT";
  uint16_t reading = read_rudder();
  last_reading = reading;
  last_reading_valid = true;
  last_change_ms = millis();
  run_state = WAIT_START;
}

void loop() {
  unsigned long now = millis();
  uint16_t reading = read_rudder();
  bool ptm_pressed = (digitalRead(PIN_PTM) == LOW);

  while (Serial.available() > 0) {
    Serial.read();
  }

  if (ptm_pressed) {
    if (ptm_press_start_ms == 0) {
      ptm_press_start_ms = now;
    }
  } else {
    ptm_press_start_ms = 0;
  }

  if (run_state == WAIT_START) {
    if (ptm_press_start_ms > 0 && (now - ptm_press_start_ms >= PTM_HOLD_MS)) {
      last_event = "START";
      limit_a_valid = false;
      limit_b_valid = false;
      min_reading = 1023;
      max_reading = 0;
      clutch_on();
      motor_move_negative();
      run_state = FIND_LIMIT_A;
      last_change_ms = now;
      last_reading = reading;
      ptm_press_start_ms = 0;
    }
  }

  if ((run_state == FIND_LIMIT_A || run_state == FIND_LIMIT_B || run_state == CENTERING) && ptm_pressed) {
    motor_stop();
    clutch_off();
    run_state = ERROR;
    last_event = "PTM_STOP";
  }

  if (last_reading_valid) {
    if ((uint16_t)abs((int)reading - (int)last_reading) >= CHANGE_THRESHOLD) {
      last_change_ms = now;
      last_reading = reading;
    }
  } else {
    last_reading = reading;
    last_reading_valid = true;
    last_change_ms = now;
  }

  bool at_limit_value = (reading == RUDDER_MIN_LIMIT || reading == RUDDER_MAX_LIMIT);
  bool stalled = (motion != STOPPED) && (now - last_change_ms > STALL_TIMEOUT_MS);

  if (motion != STOPPED && (at_limit_value || stalled)) {
    motor_stop();
    last_event = at_limit_value ? "LIMIT" : "STALL";

    if (run_state == FIND_LIMIT_A) {
      limit_a = reading;
      limit_a_valid = true;
      run_state = FIND_LIMIT_B;
      motor_move_positive();
      last_change_ms = now;
      last_reading = reading;
    } else if (run_state == FIND_LIMIT_B) {
      limit_b = reading;
      limit_b_valid = true;
      min_reading = min(limit_a, limit_b);
      max_reading = max(limit_a, limit_b);
      center_target = (uint16_t)((min_reading + max_reading) / 2);
      run_state = CENTERING;
      if (reading > center_target) {
        motor_move_negative();
      } else if (reading < center_target) {
        motor_move_positive();
      } else {
        run_state = DONE;
        clutch_off();
      }
      last_change_ms = now;
      last_reading = reading;
    } else if (run_state == CENTERING) {
      run_state = ERROR;
      clutch_off();
    }
  }

  if (run_state == CENTERING && motion != STOPPED) {
    if ((reading >= center_target - CENTER_TOLERANCE) &&
        (reading <= center_target + CENTER_TOLERANCE)) {
      motor_stop();
      run_state = DONE;
      last_event = "CENTERED";
      clutch_off();
    }
  }

  if ((motion == STOPPED) && (run_state == CENTERING)) {
    if (reading > center_target + CENTER_TOLERANCE) {
      motor_move_negative();
    } else if (reading < center_target - CENTER_TOLERANCE) {
      motor_move_positive();
    }
    last_change_ms = now;
    last_reading = reading;
  }

  if (run_state == FIND_LIMIT_A || run_state == FIND_LIMIT_B || run_state == CENTERING) {
    if (reading < min_reading) {
      min_reading = reading;
    }
    if (reading > max_reading) {
      max_reading = reading;
    }
  }

  if (oled_ok && (now - last_display_ms >= DISPLAY_PERIOD_MS)) {
    update_display(reading);
    last_display_ms = now;
  }

  if (now - last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
    telemetry(reading);
    last_telemetry_ms = now;
  }
}
