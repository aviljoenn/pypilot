#define SSD1306_NO_SPLASH

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <string.h>   // for strlen, strncpy, strncat

// ==============================
// Hardware pinout (your wiring)
// ==============================

// IBT-2 motor driver:
// - D2 / D3: direction/enable (RPWM/LPWM style ON/OFF pins)
// - D9: common PWM enable (R_EN + L_EN bridged in your wiring)
const uint8_t PIN_RPWM   = 2;   // D2 -> RPWM (ON/OFF)
const uint8_t PIN_LPWM   = 3;   // D3 -> LPWM (ON/OFF)
const uint8_t PIN_EN     = 9;   // D9 -> R_EN + L_EN bridged

// Clutch (HIGH = engaged):
const uint8_t PIN_CLUTCH = 11;  // D11 -> clutch driver input

// PTM button and Buzzer:
const uint8_t PIN_PTM    = 4;   // D4 -> PTM push-button (to GND, INPUT_PULLUP)
const uint8_t PIN_BUZZER = 10;  // D10 -> buzzer transistor base via resistor

// DS18B20 temperature sensor:
const uint8_t PIN_DS18B20 = 12; // D12 -> DS18B20 DQ (with 4.7k pull-up to 5V)

OneWire oneWire(PIN_DS18B20);
DallasTemperature tempSensors(&oneWire);

// ==============================
// Pi Zero 5V voltage monitor
// ==============================

// A3 is fed from a voltage divider on the Pi Zero's 5V rail
const uint8_t PIN_PI_VSENSE = A3;

// Scale ADC reading (0–1023) to Pi 5V voltage:
//   V_pi = V_adc * PI_VSENSE_SCALE
// Example: if divider halves 5V to ~2.5V -> scale ≈ 2.0
const float PI_VSENSE_SCALE = 2.00f;  // <-- tune this to your divider

// Fault thresholds:
const float PI_VOLT_HIGH_FAULT = 5.30f; // over-voltage fault
const float PI_VOLT_LOW_FAULT  = 4.70f; // under-voltage fault

float pi_voltage_v       = 0.0f;
bool  pi_overvolt_fault  = false;
bool  pi_undervolt_fault = false;
bool  pi_fault           = false;
bool  pi_fault_alarm_silenced = false;

// ==============================
// DS18B20 non-blocking scheduling
// ==============================
float temp_c = NAN;
bool temp_pending = false;
unsigned long temp_cycle_ms = 0;
unsigned long temp_req_ms   = 0;

const unsigned long TEMP_PERIOD_MS = 1000; // update once per second
const unsigned long TEMP_CONV_MS   = 200;  // 10-bit conversion ~187ms

// ==============================
// Analog measurements (A0, A1)
// ==============================

const uint8_t PIN_VOLTAGE = A0; // main system voltage sense
const uint8_t PIN_CURRENT = A1; // current sense

// ADC reference (Nano default assumes ~5V Vcc):
const float ADC_VREF = 5.00f;

// Voltage scaling from your divider on A0:
//   V_in = V_adc * VOLTAGE_SCALE
// Example: Rtop = 100k, Rbottom = 20k => scale ≈ 6.0
const float VOLTAGE_SCALE = 5.156f;   // <-- tune to your divider

// Current sensor calibration (ACS758LCB-050B-ish):
// Adjust these with real measurements.
const float CURRENT_ZERO_V         = 2.50f;   // zero-current offset (V) ~ Vcc/2
const float CURRENT_SENS_V_PER_A   = 0.040f;  // 40 mV/A at 5V
const bool  CURRENT_V_DROPS_WITH_A = true;    // true if V drops when I rises

const uint8_t ADC_SAMPLES = 16;  // averaging

// Current smoothing:
float current_ema_a    = 0.0f;
bool  current_ema_init = false;
const float CURRENT_EMA_ALPHA = 0.20f;

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
// Motion / control state
// ==============================

enum MotionState : uint8_t { STOPPED=0, EXTENDING=1, RETRACTING=2 };
MotionState motion = STOPPED;

unsigned long run_timeout_ms = 1000;
unsigned long last_cmd_ms    = 0;

bool clutch_enabled = true;
bool clutch_hold    = true;
unsigned long clutch_hold_ms          = 3000;
unsigned long clutch_release_deadline = 0;

static inline const __FlashStringHelper* motion_str(MotionState s) {
  switch (s) {
    case EXTENDING:  return F("EXTEND");
    case RETRACTING: return F("RETRACT");
    default:         return F("STOP");
  }
}

// ==============================
// Helpers
// ==============================

void clutch_on()  { if (clutch_enabled) digitalWrite(PIN_CLUTCH, HIGH); }
void clutch_off() { digitalWrite(PIN_CLUTCH, LOW); }

void motor_off_hard() {
  digitalWrite(PIN_EN,   LOW);
  digitalWrite(PIN_RPWM, LOW);
  digitalWrite(PIN_LPWM, LOW);
  motion = STOPPED;

  if (clutch_enabled && clutch_hold) {
    clutch_release_deadline = millis() + clutch_hold_ms;
  } else {
    clutch_release_deadline = 0;
    clutch_off();
  }
}

void motor_extend_on() {
  digitalWrite(PIN_LPWM, LOW);
  digitalWrite(PIN_RPWM, HIGH);
  digitalWrite(PIN_EN,   HIGH);
  motion = EXTENDING;

  clutch_on();
  clutch_release_deadline = 0;
}

void motor_retract_on() {
  digitalWrite(PIN_RPWM, LOW);
  digitalWrite(PIN_LPWM, HIGH);
  digitalWrite(PIN_EN,   HIGH);
  motion = RETRACTING;

  clutch_on();
  clutch_release_deadline = 0;
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
  float v = adc_to_volts(adc);

  float delta = v - CURRENT_ZERO_V;

  if (CURRENT_V_DROPS_WITH_A) delta = -delta;

  float a = delta / CURRENT_SENS_V_PER_A;

  if (a < 0.0f) a = 0.0f;
  return a;
}

float smooth_current_for_display(float a_instant) {
  if (!current_ema_init) {
    current_ema_a    = a_instant;
    current_ema_init = true;
    return current_ema_a;
  }

  current_ema_a = (CURRENT_EMA_ALPHA * a_instant) +
                  ((1.0f - CURRENT_EMA_ALPHA) * current_ema_a);

  return current_ema_a;
}

float read_pi_voltage_v() {
  uint16_t adc = read_adc_avg(PIN_PI_VSENSE, ADC_SAMPLES);
  float v_adc  = adc_to_volts(adc);
  return v_adc * PI_VSENSE_SCALE;
}

// Right-align text on OLED at given y
void oled_print_right(uint8_t y, const char* text) {
  uint8_t len = (uint8_t)strlen(text);
  int16_t x = SCREEN_WIDTH - (int16_t)(len * 6); // ~6px width per char
  if (x < 0) x = 0;
  display.setCursor(x, y);
  display.print(text);
}

void temp_service(unsigned long now) {
  if (!temp_pending) {
    if (temp_cycle_ms == 0 || (now - temp_cycle_ms >= TEMP_PERIOD_MS)) {
      tempSensors.requestTemperatures();   // non-blocking
      temp_req_ms   = now;
      temp_cycle_ms = now;
      temp_pending  = true;
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

// ==============================
// OLED drawing
// ==============================

void oled_draw() {
  if (!oled_ok) return;

  // Read measurements
  float vin = read_voltage_v();
  float ia_instant = read_current_a();
  float ia = smooth_current_for_display(ia_instant);

  // Format main voltage & current
  char vnum[10], inum[10];
  char vbuf[12], ibuf[12];

  dtostrf(vin, 0, 1, vnum);
  dtostrf(ia,  0, 1, inum);

  strncpy(vbuf, vnum, sizeof(vbuf));
  vbuf[sizeof(vbuf)-1] = '\0';
  strncat(vbuf, "V", sizeof(vbuf) - strlen(vbuf) - 1);

  strncpy(ibuf, inum, sizeof(ibuf));
  ibuf[sizeof(ibuf)-1] = '\0';
  strncat(ibuf, "A", sizeof(ibuf) - strlen(ibuf) - 1);

  // Temp formatting
  char tnum[10], tbuf[12];
  bool temp_valid = (temp_c == temp_c) && (temp_c > -55.0f) && (temp_c < 125.0f);
  if (!temp_valid) {
    strncpy(tbuf, "--.-C", sizeof(tbuf));
    tbuf[sizeof(tbuf)-1] = '\0';
  } else {
    dtostrf(temp_c, 0, 1, tnum);
    strncpy(tbuf, tnum, sizeof(tbuf));
    tbuf[sizeof(tbuf)-1] = '\0';
    strncat(tbuf, "C", sizeof(tbuf) - strlen(tbuf) - 1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Title
  display.setCursor(0, 0);
  display.println(F("IBT-2 + Clutch"));

  // Fault indicator
  if (pi_fault) {
    display.setCursor(0, 10);
    if (pi_overvolt_fault) {
      display.println(F("FAULT: PI V HIGH"));
    } else if (pi_undervolt_fault) {
      display.println(F("FAULT: PI V LOW"));
    }
  }

  display.setCursor(0, 22);
  display.print(F("Motion: "));
  display.println(motion_str(motion));

  display.setCursor(0, 32);
  display.print(F("EN: "));
  display.println(digitalRead(PIN_EN) ? F("HIGH") : F("LOW"));

  display.setCursor(0, 42);
  display.print(F("Clutch: "));
  display.println(digitalRead(PIN_CLUTCH) ? F("ON") : F("OFF"));

  display.setCursor(0, 52);
  display.print(F("Pi V: "));
  display.print(pi_voltage_v, 2);
  display.println(F("V"));

  // Right column: main Vin, current, temp
  oled_print_right(0,  vbuf);  // main Vin
  oled_print_right(10, ibuf);  // current
  oled_print_right(20, tbuf);  // enclosure temp

  display.display();
}

// ==============================
// Serial help
// ==============================

void print_help_short() {
  Serial.println();
  Serial.println(F("Cmd: e=extend r=retract s=stop"));
  Serial.println(F("0..9 timeout=N*250ms, c=clutch, h=hold"));
  Serial.println(F("? help"));
  Serial.println();
}

// ==============================
// Setup / Loop
// ==============================

void setup() {
  pinMode(PIN_EN,     OUTPUT);
  pinMode(PIN_RPWM,   OUTPUT);
  pinMode(PIN_LPWM,   OUTPUT);
  pinMode(PIN_CLUTCH, OUTPUT);

  pinMode(PIN_PTM,    INPUT_PULLUP);  // PTM to GND
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  clutch_off();
  motor_off_hard();

  Serial.begin(115200);
  Serial.println(F("=== Nano IBT-2 TEST SKETCH ==="));
  Serial.println(F("Boot: SAFE OFF"));
  print_help_short();

  // DS18B20 init
  tempSensors.begin();
  tempSensors.setResolution(10);           // 10-bit for speed
  tempSensors.setWaitForConversion(false); // non-blocking
  temp_cycle_ms = 0;                       // force immediate first request

  Wire.begin();

  oled_ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oled_ok) {
    Serial.println(F("OLED INIT FAILED (0x3C)"));
  } else {
    Serial.println(F("OLED INIT OK (0x3C)"));
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("OLED OK"));
    display.println(F("Boot: SAFE OFF"));
    display.display();
  }

  delay(800);
  motor_off_hard();
}

void loop() {
  const unsigned long now = millis();

  // Keep DS18B20 updated non-blocking
  temp_service(now);

  // Debug heartbeat once per second
  static unsigned long last_debug = 0;
  if (now - last_debug >= 1000) {
    last_debug = now;
    Serial.println(F("DEBUG: loop alive"));
  }

  // --- Pi 5V monitoring ---
  pi_voltage_v       = read_pi_voltage_v();
  pi_overvolt_fault  = (pi_voltage_v > PI_VOLT_HIGH_FAULT);
  pi_undervolt_fault = (pi_voltage_v < PI_VOLT_LOW_FAULT);
  pi_fault           = pi_overvolt_fault || pi_undervolt_fault;

  if (!pi_fault) {
    pi_fault_alarm_silenced = false;
  }

  // --- PTM handling with fault ack ---
  static bool ptm_prev = false;
  bool ptm_pressed = (digitalRead(PIN_PTM) == LOW); // button to GND
  bool ptm_edge    = ptm_pressed && !ptm_prev;
  ptm_prev = ptm_pressed;

  if (ptm_edge && pi_fault) {
    motor_off_hard();
    clutch_off();
    pi_fault_alarm_silenced = true;
    Serial.println(F("PTM: PI voltage fault acknowledged, motor/clutch off"));
  }

  // --- Buzzer control for PI fault (500ms on / 250ms off) ---
  if (pi_fault && !pi_fault_alarm_silenced) {
    static unsigned long buzz_last = 0;
    static bool          buzz_on   = false;
    unsigned long period = buzz_on ? 500UL : 250UL;
    if (now - buzz_last >= period) {
      buzz_last = now;
      buzz_on   = !buzz_on;
      digitalWrite(PIN_BUZZER, buzz_on ? HIGH : LOW);
    }
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }

  // --- Serial command handling ---
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n' || ch == ' ') continue;

    Serial.print(F("RX cmd: "));
    Serial.println(ch);

    last_cmd_ms = now;

    if (ch == 'e' || ch == 'E') {
      Serial.println(F("CMD: EXTEND"));
      motor_extend_on();
    } else if (ch == 'r' || ch == 'R') {
      Serial.println(F("CMD: RETRACT"));
      motor_retract_on();
    } else if (ch == 's' || ch == 'S') {
      Serial.println(F("CMD: STOP"));
      motor_off_hard();
    } else if (ch >= '0' && ch <= '9') {
      uint8_t n = (uint8_t)(ch - '0');
      run_timeout_ms = (unsigned long)n * 250UL;
      Serial.print(F("Timeout="));
      Serial.print(run_timeout_ms);
      Serial.println(F("ms"));
      if (run_timeout_ms == 0) motor_off_hard();
    } else if (ch == 'c' || ch == 'C') {
      // Toggle clutch enable
      clutch_enabled = !clutch_enabled;
      Serial.print(F("Clutch enabled="));
      Serial.println(clutch_enabled ? F("YES") : F("NO"));
      if (!clutch_enabled) clutch_off();
    } else if (ch == 'h' || ch == 'H') {
      // Toggle clutch hold
      clutch_hold = !clutch_hold;
      Serial.print(F("Clutch hold="));
      Serial.println(clutch_hold ? F("YES") : F("NO"));
      if (!clutch_hold && motion == STOPPED) clutch_off();
    } else if (ch == '?') {
      print_help_short();
    } else {
      Serial.println(F("Unknown cmd"));
      print_help_short();
    }
  }

  // Timeout stop
  if (motion != STOPPED && run_timeout_ms > 0) {
    if (now - last_cmd_ms >= run_timeout_ms) {
      Serial.println(F("Timeout -> STOP"));
      motor_off_hard();
    }
  }

  // Clutch release after hold time when stopped
  if (motion == STOPPED && clutch_release_deadline != 0) {
    if ((long)(now - clutch_release_deadline) >= 0) {
      clutch_release_deadline = 0;
      Serial.println(F("Clutch hold expired -> OFF"));
      clutch_off();
    }
  }

  // OLED refresh
  static unsigned long last_draw = 0;
  if (oled_ok && (now - last_draw >= 200)) {
    last_draw = now;
    oled_draw();
  }
}
