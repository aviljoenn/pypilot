#define SSD1306_NO_SPLASH

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ==============================
// Hardware pinout (your wiring)
// ==============================
// IBT-2:
const uint8_t PIN_RPWM   = 2;   // D2 -> RPWM (ON/OFF)
const uint8_t PIN_LPWM   = 3;   // D3 -> LPWM (ON/OFF)
const uint8_t PIN_EN     = 9;   // D9 -> R_EN + L_EN bridged

// PTM button and Buzzer
const uint8_t PIN_PTM    = 4;   // D4 -> PTM push-button (to GND, use INPUT_PULLUP)
const uint8_t PIN_BUZZER = 10;  // D10 -> buzzer transistor base via resistor

// Clutch (your hardware: HIGH = engaged):
const uint8_t PIN_CLUTCH = 11;  // D11 -> clutch driver input

// ==============================
// DS18B20 Temperature (DQ on D12)
// ==============================
const uint8_t PIN_DS18B20 = 12;   // D12 -> DS18B20 DQ (with 4.7k pull-up to VDD)

OneWire oneWire(PIN_DS18B20);
DallasTemperature tempSensors(&oneWire);

// Last valid temperature (C)
float temp_c = NAN;

// Non-blocking conversion scheduling
bool temp_pending = false;
unsigned long temp_cycle_ms = 0;
unsigned long temp_req_ms   = 0;

// Update temperature once per second
const unsigned long TEMP_PERIOD_MS = 1000;

// 10-bit conversion ~187ms, give it margin
const unsigned long TEMP_CONV_MS = 200;

// Analog measurements:
const uint8_t PIN_VOLTAGE = A0; // Voltage sense (ADC input)
const uint8_t PIN_CURRENT = A1; // Current sense  (ADC input)

// --- Current smoothing (display) ---
float current_ema_a = 0.0f;
bool  current_ema_init = false;

// 0.10 = heavy smoothing (slower), 0.30 = lighter smoothing (faster)
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
// Calibration (EDIT THESE)
// ==============================
// ADC reference voltage (default Nano uses Vcc ~ 5.0V; set this to your measured 5V for best results)
const float ADC_VREF = 5.00f;

// Voltage divider scaling:
// If A0 measures the divided voltage, then:
//   V_in = V_adc * VOLTAGE_SCALE
// Example: If divider is 100k (top) and 20k (bottom), scale = (100k+20k)/20k = 6.0
const float VOLTAGE_SCALE = 5.156f;   // <-- set to your divider ratio

// Current sensor scaling (default set for ACS758LCB-050B style behavior)
// Many ACS sensors output Vcc/2 at 0A and have a sensitivity (V per A).
const float CURRENT_ZERO_V = 2.10f;      // calibrated zero point (Volts) - your setup
const float CURRENT_SENS_V_PER_A = 0.0111f; // calibrated sensitivity (V per Amp) ~11.1mV/A
const bool  CURRENT_V_DROPS_WITH_A = true;  // true if voltage goes DOWN when current goes UP

// Display formatting:
const uint8_t ADC_SAMPLES = 16;       // simple averaging for noise reduction

// ==============================
// Control state
// ==============================
enum MotionState : uint8_t { STOPPED=0, EXTENDING=1, RETRACTING=2 };
MotionState motion = STOPPED;

unsigned long run_timeout_ms = 1000;
unsigned long last_cmd_ms = 0;

bool clutch_enabled = true;
bool clutch_hold = true;
unsigned long clutch_hold_ms = 3000;
unsigned long clutch_release_deadline = 0;

static inline const __FlashStringHelper* motion_str(MotionState s) {
  switch (s) {
    case EXTENDING: return F("EXTEND");
    case RETRACTING: return F("RETRACT");
    default: return F("STOP");
  }
}

// ==============================
// Helpers
// ==============================
void clutch_on()  { if (clutch_enabled) digitalWrite(PIN_CLUTCH, HIGH); }
void clutch_off() { digitalWrite(PIN_CLUTCH, LOW); }

void motor_off_hard() {
  digitalWrite(PIN_EN, LOW);
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
  digitalWrite(PIN_EN, HIGH);
  motion = EXTENDING;

  clutch_on();
  clutch_release_deadline = 0;
}

void motor_retract_on() {
  digitalWrite(PIN_RPWM, LOW);
  digitalWrite(PIN_LPWM, HIGH);
  digitalWrite(PIN_EN, HIGH);
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

float smooth_current_for_display(float a_instant)
{
  if (!current_ema_init) {
    current_ema_a = a_instant;
    current_ema_init = true;
    return current_ema_a;
  }

  current_ema_a = (CURRENT_EMA_ALPHA * a_instant) +
                  ((1.0f - CURRENT_EMA_ALPHA) * current_ema_a);

  return current_ema_a;
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

  // If current makes voltage drop, invert delta
  if (CURRENT_V_DROPS_WITH_A) delta = -delta;

  float a = delta / CURRENT_SENS_V_PER_A;

  // Clamp: never below 0A
  if (a < 0.0f) a = 0.0f;

  return a;
}

// Right-align text on OLED at a given y
void oled_print_right(uint8_t y, const char* text) {
  // Font at size=1 is ~6px wide per char (including spacing)
  // This is accurate enough for small right-aligned readouts.
  uint8_t len = (uint8_t)strlen(text);
  int16_t x = SCREEN_WIDTH - (int16_t)(len * 6);
  if (x < 0) x = 0;
  display.setCursor(x, y);
  display.print(text);
}

void oled_draw() {
  if (!oled_ok) return;

  // Read measurements
  float vin = read_voltage_v();
  float ia_instant = read_current_a();
  float ia = smooth_current_for_display(ia_instant);

  // Format strings (keep short for right corner)
  char vnum[10];
  char inum[10];
  char vbuf[12];
  char ibuf[12];

  // Convert floats to strings (width=0, 1 decimal)
  dtostrf(vin, 0, 1, vnum);
  dtostrf(ia,  0, 1, inum);

  // Append units
  strncpy(vbuf, vnum, sizeof(vbuf));
  vbuf[sizeof(vbuf)-1] = '\0';
  strncat(vbuf, "V", sizeof(vbuf) - strlen(vbuf) - 1);

  strncpy(ibuf, inum, sizeof(ibuf));
  ibuf[sizeof(ibuf)-1] = '\0';
  strncat(ibuf, "A", sizeof(ibuf) - strlen(ibuf) - 1);

  // Temp formatting
  char tnum[10];
  char tbuf[12];

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

  // Left side status
  display.setCursor(0, 0);
  display.println(F("IBT-2 + Clutch"));

  display.setCursor(0, 12);
  display.print(F("Motion: "));
  display.println(motion_str(motion));

  display.setCursor(0, 22);
  display.print(F("EN: "));
  display.println(digitalRead(PIN_EN) ? F("HIGH") : F("LOW"));

  display.setCursor(0, 32);
  display.print(F("Clutch: "));
  display.println(digitalRead(PIN_CLUTCH) ? F("ON") : F("OFF"));

  display.setCursor(0, 42);
  display.print(F("Tmo: "));
  display.print(run_timeout_ms);
  display.println(F("ms"));

  display.setCursor(0, 54);
  display.println(F("e r s 0-9 ?"));

  // Top-right readouts
  oled_print_right(0,  vbuf);  // Voltage at top right
  oled_print_right(10, ibuf);  // Current just below (right side)
  oled_print_right(20, tbuf);  // Temp below current

  display.display();
}

void print_help_short() {
  Serial.println();
  Serial.println(F("Cmd: e=extend r=retract s=stop"));
  Serial.println(F("0..9 timeout=N*250ms, c=clutch, h=hold"));
  Serial.println(F("? help"));
  Serial.println();
}

void temp_service(unsigned long now)
{
  // Kick off a conversion periodically (non-blocking)
  if (!temp_pending) {
    if (temp_cycle_ms == 0 || (now - temp_cycle_ms >= TEMP_PERIOD_MS)) {
      tempSensors.requestTemperatures();   // returns immediately (non-blocking)
      temp_req_ms = now;
      temp_cycle_ms = now;
      temp_pending = true;
    }
  }

  // After conversion time has elapsed, read the result
  if (temp_pending && (now - temp_req_ms >= TEMP_CONV_MS)) {
    float t = tempSensors.getTempCByIndex(0);

    // DEVICE_DISCONNECTED_C is typically -127
    if (t > -55.0f && t < 125.0f) {
      temp_c = t;
    }

    temp_pending = false;
  }
}


// ==============================
// Setup / Loop
// ==============================
void setup() {
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  pinMode(PIN_CLUTCH, OUTPUT);

  // PTM + Buzzer
  pinMode(PIN_PTM, INPUT_PULLUP);  // button to GND
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);   // buzzer off

  clutch_off();
  motor_off_hard();

  Serial.begin(115200);
  Serial.println(F("Boot: SAFE OFF"));
  print_help_short();

    // DS18B20 init (non-blocking)
  tempSensors.begin();
  tempSensors.setResolution(10);           // 10-bit for fast updates
  tempSensors.setWaitForConversion(false); // IMPORTANT: don't block your loop
  temp_cycle_ms = 0;                       // force first request

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

  // Keep DS18B20 updated without blocking
  temp_service(now);

  // Serial command handling
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n' || ch == ' ') continue;

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
      clutch_enabled = !clutch_enabled;
      Serial.print(F("Clutch enabled="));
      Serial.println(clutch_enabled ? F("YES") : F("NO"));
      if (!clutch_enabled) clutch_off();
    } else if (ch == 'h' || ch == 'H') {
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

  // PTM + buzzer test: buzzer on while PTM is pressed
  bool ptm_pressed = (digitalRead(PIN_PTM) == LOW);  // button to GND
  digitalWrite(PIN_BUZZER, ptm_pressed ? HIGH : LOW);

}

