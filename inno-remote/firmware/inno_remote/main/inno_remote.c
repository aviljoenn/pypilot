#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#include "u8g2.h"
#include "u8g2_esp32_hal.h"

#include "wifi_status.h"
#include <stdarg.h>

static const char *TAG = "INNO_REMOTE";

// ========================
// OLED PINS (as built)
// ========================
#define PIN_OLED_SCK   4
#define PIN_OLED_MOSI  5
#define PIN_OLED_CS    6
#define PIN_OLED_DC    7
#define PIN_OLED_RES   10

// ========================
// INPUT PINS (as built)
// ========================
#define PIN_MODE_AUTO      20   // active-low -> AUTO
#define PIN_MODE_MANUAL    21   // active-low -> MANUAL
#define PIN_ESTOP          3    // active-low STOP
#define PIN_ADC_LADDER     0    // ADC ladder sense
#define PIN_ADC_POT        1    // ADC pot wiper

// ADC channels for ESP32-C3 (GPIO0=CH0, GPIO1=CH1)
#define ADC_UNIT_USED      ADC_UNIT_1
#define ADC_CH_LADDER      ADC_CHANNEL_0
#define ADC_CH_POT         ADC_CHANNEL_1

#define SCREEN_W 128
#define SCREEN_H  64

// Button responsiveness
#define LOOP_MS             20
#define BUTTON_DEBOUNCE_MS  15

// Rudder limits (demo)
#define MAX_RUDDER_DEG      40.0f

typedef enum {
    BTN_NONE = -1,
    BTN_1 = 0,
    BTN_2 = 1,
    BTN_3 = 2,
    BTN_4 = 3,
    BTN_5 = 4,
} btn_t;

static u8g2_t u8g2;
static adc_oneshot_unit_handle_t adc_handle;

// ------------------------------ Helpers ------------------------------------

void wifi_sta_start(void);

static void init_gpio_inputs(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_MODE_AUTO) |
                        (1ULL << PIN_MODE_MANUAL) |
                        (1ULL << PIN_ESTOP),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_USED,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CH_LADDER, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CH_POT, &chan_cfg));
}

static int adc_read_raw(adc_channel_t ch)
{
    int raw = 0;
    if (adc_oneshot_read(adc_handle, ch, &raw) != ESP_OK) return -1;
    return raw;
}

static void init_oled_u8g2(void)
{
    u8g2_esp32_hal_t hal = U8G2_ESP32_HAL_DEFAULT;

    hal.bus.spi.clk  = PIN_OLED_SCK;
    hal.bus.spi.mosi = PIN_OLED_MOSI;
    hal.bus.spi.cs   = PIN_OLED_CS;
    hal.dc           = PIN_OLED_DC;
    hal.reset        = PIN_OLED_RES;

    u8g2_esp32_hal_init(hal);

    u8g2_Setup_sh1106_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8g2_esp32_spi_byte_cb,
        u8g2_esp32_gpio_and_delay_cb
    );

    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
}

static inline bool mode_is_auto(bool auto_low, bool manual_low)
{
    return (auto_low && !manual_low);
}
static inline bool mode_is_manual(bool auto_low, bool manual_low)
{
    return (manual_low && !auto_low);
}

static const char* mode_str(bool auto_low, bool manual_low)
{
    if (mode_is_auto(auto_low, manual_low)) return "AUTO";
    if (mode_is_manual(auto_low, manual_low)) return "MANUAL";
    return "ERR";
}

static int wrap360(int a)
{
    a %= 360;
    if (a < 0) a += 360;
    return a;
}

static float wrap360f(float a)
{
    while (a >= 360.0f) a -= 360.0f;
    while (a <   0.0f) a += 360.0f;
    return a;
}

static float shortest_delta_deg(float current, float target)
{
    float d = target - current;
    while (d >= 180.0f) d -= 360.0f;
    while (d <  -180.0f) d += 360.0f;
    return d;
}

// Ladder decode using ratio to “no-press” baseline
static btn_t decode_ladder(float ratio)
{
    if (ratio > 0.85f) return BTN_NONE;
    if (ratio > 0.59f)  return BTN_1;  // 22k
    if (ratio > 0.41f)  return BTN_2;  // 10k
    if (ratio > 0.25f)  return BTN_3;  // 4.7k
    if (ratio > 0.136f) return BTN_4;  // 2.2k
    return BTN_5;                      // 1k
}

static void draw_title_centered(const char *s, int baseline_y)
{
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    int w = u8g2_GetStrWidth(&u8g2, s);
    int x = (SCREEN_W - w) / 2;
    if (x < 0) x = 0;
    u8g2_DrawStr(&u8g2, x, baseline_y, s);
}

static void draw_centered_line_6x10(const char *s, int baseline_y)
{
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    int w = u8g2_GetStrWidth(&u8g2, s);
    int x = (SCREEN_W - w) / 2;
    if (x < 0) x = 0;
    u8g2_DrawStr(&u8g2, x, baseline_y, s);
}

static void draw_left_right_6x10(const char *left, const char *right, int baseline_y)
{
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, baseline_y, left);

    int rw = u8g2_GetStrWidth(&u8g2, right);
    int rx = SCREEN_W - rw;
    if (rx < 0) rx = 0;
    u8g2_DrawStr(&u8g2, rx, baseline_y, right);
}

static void draw_button_box(int x, int y, int w, int h, const char *label, bool on)
{
    if (on) {
        u8g2_DrawBox(&u8g2, x, y, w, h);
        u8g2_SetDrawColor(&u8g2, 0);
    } else {
        u8g2_DrawFrame(&u8g2, x, y, w, h);
        u8g2_SetDrawColor(&u8g2, 1);
    }

    // Button text: vertically centered
    u8g2_SetFont(&u8g2, u8g2_font_5x8_tf);
    int ascent  = u8g2_GetAscent(&u8g2);
    int descent = u8g2_GetDescent(&u8g2);     // negative
    int font_h  = ascent - descent;

    int tw = u8g2_GetStrWidth(&u8g2, label);
    int tx = x + (w - tw) / 2;
    int ty = y + (h - font_h) / 2 + ascent + 1;   // baseline for vertical centering

    u8g2_DrawStr(&u8g2, tx, ty, label);
    u8g2_SetDrawColor(&u8g2, 1);
}

static void draw_dotted_line(int x0, int y0, int x1, int y1, int period, int on_len)
{
    // Simple Bresenham with "period" pattern, drawing first "on_len" pixels per period
    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    int step = 0;

    while (1) {
        int p = step % period;
        if (p < on_len) {
            u8g2_DrawPixel(&u8g2, x0, y0);
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
        step++;
    }
}

static void draw_rudder_bar(int x, int y, int w, int h, float rudder_norm)
{
    // Frame
    u8g2_DrawFrame(&u8g2, x, y, w, h);

    int inner_left   = x + 1;
    int inner_right  = x + w - 2;
    int inner_top    = y + 1;
    int inner_bottom = y + h - 2;

    int center_x = x + (w - 1) / 2;

    // Center tick at bottom
    u8g2_DrawVLine(&u8g2, center_x, y + h - 5, 4);

    // Diagonal dotted guide lines:
    // from bottom middle (inside) to top corners (inside)
    draw_dotted_line(center_x, inner_bottom, inner_left,  inner_top,  3, 1);
    draw_dotted_line(center_x, inner_bottom, inner_right, inner_top,  3, 1);

    // Clamp and map rudder position
    if (rudder_norm >  1.0f) rudder_norm =  1.0f;
    if (rudder_norm < -1.0f) rudder_norm = -1.0f;

    int travel = ((w - 1) / 2) - 2;
    int line_x = center_x + (int)lroundf(rudder_norm * travel);

    if (line_x < inner_left)  line_x = inner_left;
    if (line_x > inner_right) line_x = inner_right;

    // Position indicator: 6 pixels thick, centered on line_x
    // For even width, "center" is between the two middle columns.
    // This draws columns: line_x-3, -2, -1, 0, +1, +2
    int xs[6] = { line_x - 3, line_x - 2, line_x - 1, line_x, line_x + 1, line_x + 2 };

    for (int i = 0; i < 6; i++) {
        int xi = xs[i];
        if (xi < inner_left)  xi = inner_left;
        if (xi > inner_right) xi = inner_right;
        u8g2_DrawVLine(&u8g2, xi, inner_top, (h - 2));
    }
}

// STOP: smaller font, snug border
static void draw_stop_snug(int y_top, bool stop_on)
{
    const char *label = "STOP";

    u8g2_SetFont(&u8g2, u8g2_font_5x8_tf);

    int ascent  = u8g2_GetAscent(&u8g2);
    int descent = u8g2_GetDescent(&u8g2);
    int font_h  = ascent - descent;

    int tw = u8g2_GetStrWidth(&u8g2, label);

    const int pad_x = 3;
    const int pad_y = 1;

    int frame_w = tw + 2 * pad_x;
    int frame_h = font_h + 2 * pad_y;

    int x = (SCREEN_W - frame_w) / 2;
    if (x < 0) x = 0;

    if (stop_on) {
        u8g2_DrawBox(&u8g2, x, y_top, frame_w, frame_h);
        u8g2_SetDrawColor(&u8g2, 0);
    } else {
        u8g2_DrawFrame(&u8g2, x, y_top, frame_w, frame_h);
        u8g2_SetDrawColor(&u8g2, 1);
    }

    int text_x = x + pad_x;
    int text_y = y_top + pad_y + ascent;
    u8g2_DrawStr(&u8g2, text_x, text_y, label);

    u8g2_SetDrawColor(&u8g2, 1);
}

static void draw_wifi_icon_top_right(void)
{
    wifi_ui_status_t st = wifi_ui_get_status();

    // Define a fixed "icon box" in the top-right so everything is clipped by design.
    // Box size: 18x12 pixels
    const int box_w = 18;
    const int box_h = 12;
    const int box_x = SCREEN_W - box_w;   // 128-18 = 110
    const int box_y = 0;

    // Dot center inside the box (always)
    const int cx = box_x + (box_w / 2);   // ~119
    const int cy = box_y + (box_h - 2);   // ~10 (keeps dot visible, not on the border)

    // --- No association: draw an X INSIDE the icon box (no negative coords) ---
    if (!st.assoc) {
        // Make the X narrower (~1/3 less width), 1px shorter top+bottom,
        // and bold by drawing it 3 times offset to the right.
        const int x0 = box_x + 6;
        const int x1 = box_x + box_w - 7;
        const int y0 = box_y + 2;               // shave off top row
        const int y1 = box_y + box_h - 3;       // shave off bottom row

        for (int dx = 0; dx < 3; dx++) {
            u8g2_DrawLine(&u8g2, x0 + dx, y0, x1 + dx, y1);
            u8g2_DrawLine(&u8g2, x0 + dx, y1, x1 + dx, y0);
        }
        return;
}

    // --- Associated: dot always ---
    u8g2_DrawDisc(&u8g2, cx, cy, 1, U8G2_DRAW_ALL);

    // --- Associated but no IP: "!" on the left, moved 4px RIGHT and 1px UP ---
    if (st.assoc && !st.has_ip) {
        u8g2_SetFont(&u8g2, u8g2_font_5x8_tf);
        // was cx-13 -> shift right by 4 => cx-9
        u8g2_DrawStr(&u8g2, cx - 9, cy + 1, "!");
    }

    // 0..3 arcs
    int n = st.arcs;
    if (n < 0) n = 0;
    if (n > 3) n = 3;

    // Radii for 3 arcs (fit inside the box)
    const int r1 = 3;
    const int r2 = 5;
    const int r3 = 7;

    // Draw upper semicircle as pixels, BUT skip dy==0 endpoints
    // so we never plot pixels on the dot baseline under the arc ends.
    #define DRAW_ARC_UPPER(r) do {                                \
        for (int dx = -(r); dx <= (r); dx++) {                   \
            int inside_i = (r)*(r) - dx*dx;                      \
            if (inside_i <= 0) continue; /* skip endpoints */    \
            int dy = (int)lroundf(sqrtf((float)inside_i));       \
            int x = cx + dx;                                     \
            int y = cy - dy;                                     \
            /* keep pixels inside icon box */                    \
            if (x < box_x || x >= box_x + box_w) continue;       \
            if (y < box_y || y >= box_y + box_h) continue;       \
            u8g2_DrawPixel(&u8g2, x, y);                         \
        }                                                        \
    } while(0)

    if (n >= 1) DRAW_ARC_UPPER(r1);
    if (n >= 2) DRAW_ARC_UPPER(r2);
    if (n >= 3) DRAW_ARC_UPPER(r3);

    #undef DRAW_ARC_UPPER
}

static bool demo_mode_boot_check(void)
{
    // Hold STOP for this long during boot to enter demo mode
    const int hold_ms = 800;
    const int step_ms = 20;
    const int steps = hold_ms / step_ms;

    for (int i = 0; i < steps; i++) {
        // STOP is active-low
        if (gpio_get_level(PIN_ESTOP) != 0) {
            return false; // released -> not demo
        }
        vTaskDelay(pdMS_TO_TICKS(step_ms));
    }
    return true; // held for the full duration
}

// =========================
// Log ring buffer (100 lines)
// =========================
#define LOG_CAPACITY   100
#define LOG_LINE_MAX   160  // truncate long lines

static char s_log_lines[LOG_CAPACITY][LOG_LINE_MAX];
static uint16_t s_log_head = 0;   // next write position
static uint16_t s_log_count = 0;  // number of valid lines (<= LOG_CAPACITY)
static uint32_t s_log_gen = 0;    // increments on every stored line

static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;
static int (*s_orig_vprintf)(const char *fmt, va_list ap) = NULL;

static void log_store_line_locked(const char *line)
{
    // strip trailing CR/LF
    char tmp[LOG_LINE_MAX];
    size_t n = strnlen(line, LOG_LINE_MAX - 1);
    while (n > 0 && (line[n-1] == '\n' || line[n-1] == '\r')) n--;

    // copy + null terminate
    memcpy(tmp, line, n);
    tmp[n] = '\0';

    // store
    strncpy(s_log_lines[s_log_head], tmp, LOG_LINE_MAX - 1);
    s_log_lines[s_log_head][LOG_LINE_MAX - 1] = '\0';

    s_log_head = (s_log_head + 1) % LOG_CAPACITY;
    if (s_log_count < LOG_CAPACITY) s_log_count++;
    s_log_gen++;
}

static void log_store_line(const char *line)
{
    portENTER_CRITICAL(&s_log_mux);
    log_store_line_locked(line);
    portEXIT_CRITICAL(&s_log_mux);
}

static int log_vprintf(const char *fmt, va_list ap)
{
    // format into a buffer for storing
    char buf[LOG_LINE_MAX * 2];
    va_list ap_copy;
    va_copy(ap_copy, ap);
    vsnprintf(buf, sizeof(buf), fmt, ap_copy);
    va_end(ap_copy);

    // split on newlines and store each line (skip empty)
    char *p = buf;
    while (*p) {
        char *nl = strchr(p, '\n');
        if (nl) *nl = '\0';
        if (*p) log_store_line(p);
        if (!nl) break;
        p = nl + 1;
    }

    // still print to console
    if (s_orig_vprintf) return s_orig_vprintf(fmt, ap);
    return vprintf(fmt, ap);
}

static void log_capture_init(void)
{
    s_orig_vprintf = esp_log_set_vprintf(log_vprintf);
}

static uint16_t log_count(void)
{
    uint16_t c;
    portENTER_CRITICAL(&s_log_mux);
    c = s_log_count;
    portEXIT_CRITICAL(&s_log_mux);
    return c;
}

static uint32_t log_gen(void)
{
    uint32_t g;
    portENTER_CRITICAL(&s_log_mux);
    g = s_log_gen;
    portEXIT_CRITICAL(&s_log_mux);
    return g;
}

static void log_copy_line(uint16_t idx_from_oldest, char *out, size_t out_sz)
{
    if (out_sz == 0) return;
    out[0] = '\0';

    portENTER_CRITICAL(&s_log_mux);

    uint16_t c = s_log_count;
    if (idx_from_oldest >= c) {
        portEXIT_CRITICAL(&s_log_mux);
        return;
    }

    uint16_t start = (s_log_head + LOG_CAPACITY - c) % LOG_CAPACITY;
    uint16_t pos = (start + idx_from_oldest) % LOG_CAPACITY;

    strncpy(out, s_log_lines[pos], out_sz - 1);
    out[out_sz - 1] = '\0';

    portEXIT_CRITICAL(&s_log_mux);
}

static int log_max_len(void)
{
    int maxlen = 0;
    portENTER_CRITICAL(&s_log_mux);

    uint16_t c = s_log_count;
    uint16_t start = (s_log_head + LOG_CAPACITY - c) % LOG_CAPACITY;
    for (uint16_t i = 0; i < c; i++) {
        uint16_t pos = (start + i) % LOG_CAPACITY;
        int len = (int)strnlen(s_log_lines[pos], LOG_LINE_MAX - 1);
        if (len > maxlen) maxlen = len;
    }

    portEXIT_CRITICAL(&s_log_mux);
    return maxlen;
}

// ------------------------------ Main ---------------------------------------

void app_main(void)
{
    init_gpio_inputs();
    log_capture_init();
    bool demo_mode = demo_mode_boot_check();
    ESP_LOGI(TAG, "Boot mode: %s", demo_mode ? "DEMO" : "NORMAL");
    init_adc();
    init_oled_u8g2();

    wifi_sta_start();

    // ===== Log View state =====
    bool log_view = false;
    bool log_ignore_stop_until_release = false;
    bool log_pinned_bottom = true;
    int  log_top = 0;            // index of first visible line (0=oldest)
    int  log_horiz = 0;          // character offset
    int  log_cached_maxlen = 0;
    uint32_t log_last_gen = 0;

    bool prev_stop_on = false;
    int  stop_tap_count = 0;
    uint32_t stop_window_start_ms = 0;
    uint32_t last_stop_press_ms = 0;

    const uint32_t STOP_TAP_WINDOW_MS = 4000;   // 5 taps within 4s total
    const uint32_t STOP_TAP_GAP_MS = 900;     // max gap between taps
    const uint32_t STOP_TAP_MIN_GAP_MS = 150;   // ignore bounce/double-edges
    const int STOP_TAP_COUNT = 5;

    // AUTO state
    bool  ap_on = false;
    int   command_deg = 0;     // starts at 0 on reboot
    float head_deg = 0.0f;     // shared across modes

    // Rudder state (deg)
    float rudder_deg = 0.0f;
    float rudder_target_deg = 0.0f;

    // Manual jog state
    float manual_rudder_deg = 0.0f;
    bool  manual_using_jog = false;

    // Tuning
    const float TURN_RATE_FULL_DEG_PER_SEC = 60.0f;
    const float ERROR_FOR_FULL_RUDDER_DEG  = 45.0f;
    const float RUDDER_TAU_SEC             = 0.08f;
    const float SNAP_DEG                   = 0.6f;

    // Baselines / filters
    float ladder_idle = 0.0f;
    bool  ladder_idle_init = false;

    float pot_filt = 0.0f;
    bool  pot_init = false;
    float pot_last = 0.0f;

    // Debounce
    btn_t candidate_btn = BTN_NONE;
    TickType_t candidate_since = 0;
    btn_t stable_btn = BTN_NONE;
    btn_t prev_stable_btn = BTN_NONE;

    // STOP state
    bool stop_on = false;

    // Mode transition tracking
    bool prev_manual = false;

    ESP_LOGI(TAG, "UI start (debounce=%dms, loop=%dms)", BUTTON_DEBOUNCE_MS, LOOP_MS);

    const float dt = (float)LOOP_MS / 1000.0f;

    while (1) {
        bool auto_low   = (gpio_get_level(PIN_MODE_AUTO) == 0);
        bool manual_low = (gpio_get_level(PIN_MODE_MANUAL) == 0);
        bool in_auto    = mode_is_auto(auto_low, manual_low);
        bool in_manual  = mode_is_manual(auto_low, manual_low);

        stop_on = (gpio_get_level(PIN_ESTOP) == 0);

        int raw_ladder = adc_read_raw(ADC_CH_LADDER);
        int raw_pot    = adc_read_raw(ADC_CH_POT);

        if (raw_pot >= 0) {
            if (!pot_init) { pot_filt = (float)raw_pot; pot_last = pot_filt; pot_init = true; }
            pot_filt = pot_filt + 0.35f * ((float)raw_pot - pot_filt);
        }

        // Mode transition into manual: seed jog from current rudder
        if (in_manual && !prev_manual) {
            manual_rudder_deg = rudder_deg;
            manual_using_jog = false;
        }
        prev_manual = in_manual;

        // Safety/state rules
        if (in_manual || (!in_auto && !in_manual) || stop_on) {
            ap_on = false;
        }

        // Ladder baseline + decode
        btn_t sample_btn = BTN_NONE;
        if (raw_ladder >= 0) {
            if (!ladder_idle_init) { ladder_idle = (float)raw_ladder; ladder_idle_init = true; }

            float ratio_now = (ladder_idle > 1.0f) ? ((float)raw_ladder / ladder_idle) : 1.0f;
            if (ratio_now > 0.90f) {
                ladder_idle = ladder_idle + 0.02f * ((float)raw_ladder - ladder_idle);
            }

            float ratio = (ladder_idle > 1.0f) ? ((float)raw_ladder / ladder_idle) : 1.0f;
            sample_btn = decode_ladder(ratio);
        }

        // Debounce ladder (time-based)
        TickType_t now = xTaskGetTickCount();
        if (sample_btn != candidate_btn) {
            candidate_btn = sample_btn;
            candidate_since = now;
        } else {
            uint32_t ms = (uint32_t)((now - candidate_since) * portTICK_PERIOD_MS);
            if (ms >= BUTTON_DEBOUNCE_MS) {
                stable_btn = candidate_btn;
            }
        }

        // Press event (edge)
        bool pressed_event = (stable_btn != prev_stable_btn) && (stable_btn != BTN_NONE);
        if (stable_btn != prev_stable_btn) prev_stable_btn = stable_btn;

        // If STOP is active: freeze everything, ignore inputs and motion
        if (!stop_on) {
            if (pressed_event) {
                if (in_auto) {
                    if (stable_btn == BTN_3) {
                        ap_on = !ap_on;
                    } else {
                        switch (stable_btn) {
                            case BTN_1: command_deg = wrap360(command_deg - 10); break;
                            case BTN_2: command_deg = wrap360(command_deg - 1);  break;
                            case BTN_4: command_deg = wrap360(command_deg + 1);  break;
                            case BTN_5: command_deg = wrap360(command_deg + 10); break;
                            default: break;
                        }
                    }
                } else if (in_manual) {
                    manual_using_jog = true;
                    switch (stable_btn) {
                        case BTN_1: manual_rudder_deg -= 10.0f; break; // <<
                        case BTN_2: manual_rudder_deg -= 1.0f;  break; // <
                        case BTN_3: manual_rudder_deg  = 0.0f;  break; // |
                        case BTN_4: manual_rudder_deg += 1.0f;  break; // >
                        case BTN_5: manual_rudder_deg += 10.0f; break; // >>
                        default: break;
                    }
                    if (manual_rudder_deg >  MAX_RUDDER_DEG) manual_rudder_deg =  MAX_RUDDER_DEG;
                    if (manual_rudder_deg < -MAX_RUDDER_DEG) manual_rudder_deg = -MAX_RUDDER_DEG;
                }
            }
        }

        // ===== STOP press edge detect =====
        uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        bool stop_edge_press = (stop_on && !prev_stop_on);
        bool stop_edge_release = (!stop_on && prev_stop_on);
        prev_stop_on = stop_on;
        if (log_ignore_stop_until_release && stop_edge_release) {
            log_ignore_stop_until_release = false;
        }

        // ===== Enter Log View: STOP tapped 5x within 4s window =====
        if (!log_view && !log_ignore_stop_until_release && stop_edge_press) {

            // ignore bounce / too-fast repeats
            if (last_stop_press_ms != 0 && (now_ms - last_stop_press_ms) < STOP_TAP_MIN_GAP_MS) {
                // ignore
            } else {
                last_stop_press_ms = now_ms;

                // start or restart the 4s window
                if (stop_tap_count == 0 || (now_ms - stop_window_start_ms) > STOP_TAP_WINDOW_MS) {
                    stop_window_start_ms = now_ms;
                    stop_tap_count = 0;
                }

                stop_tap_count++;

                // Debug so you can see it working in monitor
                ESP_LOGI(TAG, "STOP taps: %d/%d", stop_tap_count, STOP_TAP_COUNT);

                if (stop_tap_count >= STOP_TAP_COUNT && (now_ms - stop_window_start_ms) <= STOP_TAP_WINDOW_MS) {

                    ESP_LOGI(TAG, "Entering LOG VIEW");

                    log_view = true;
                    log_ignore_stop_until_release = true;  // prevent instant exit on the 5th press
                    log_pinned_bottom = true;
                    log_last_gen = log_gen();
                    log_cached_maxlen = log_max_len();

                    // start at bottom (newest visible, newest at bottom line)
                    int lines_per_screen = 10; // 4x6 font on 64px tall
                    int total = (int)log_count();
                    int bottom_top = total - lines_per_screen;
                    if (bottom_top < 0) bottom_top = 0;
                    log_top = bottom_top;

                    stop_tap_count = 0;
                    stop_window_start_ms = 0;
                }
            }
        }

        // ===== Log View Mode =====
        if (log_view) {

            // arm STOP-to-exit only after user releases STOP
            if (log_ignore_stop_until_release && stop_edge_release) {
                log_ignore_stop_until_release = false;
            }

            if (!log_ignore_stop_until_release && stop_edge_press) {
                log_view = false;

                // Reset tap state so this exit press can't immediately retrigger entry
                stop_tap_count = 0;
                stop_window_start_ms = 0;
                last_stop_press_ms = 0;
                log_ignore_stop_until_release = true;

                // Require a clean STOP release before we allow tap-to-enter again
                log_ignore_stop_until_release = true;

                // Also reset horizontal scroll
                log_horiz = 0;
                continue;
            }

            // Update cached lengths when logs change; auto-follow bottom if pinned
            uint32_t g = log_gen();
            if (g != log_last_gen) {
                log_last_gen = g;
                log_cached_maxlen = log_max_len();

                int lines_per_screen = 10;
                int total = (int)log_count();
                int bottom_top = total - lines_per_screen;
                if (bottom_top < 0) bottom_top = 0;

                if (log_pinned_bottom) {
                    log_top = bottom_top;
                } else {
                    // clamp if buffer rolled
                    if (log_top > bottom_top) log_top = bottom_top;
                }
            }

            // POT horizontal scroll (0..max_offset chars)
            int cols = 32; // ~128px / 4px per char with 4x6 font
            int max_off = log_cached_maxlen - cols;
            if (max_off < 0) max_off = 0;

            int potv = raw_pot;
            if (potv < 0) potv = 0;
            if (potv > 4095) potv = 4095;
            log_horiz = (max_off == 0) ? 0 : (int)((potv * (long)max_off + 2047) / 4095);

            // Button controls (ladder buttons)
            // BTN_1: page up, BTN_2: line up, BTN_3: bottom, BTN_4: line down, BTN_5: page down
            if (pressed_event) {
                int lines_per_screen = 10;
                int total = (int)log_count();
                int bottom_top = total - lines_per_screen;
                if (bottom_top < 0) bottom_top = 0;

                if (stable_btn == BTN_1) {
                    log_top -= lines_per_screen;
                } else if (stable_btn == BTN_2) {
                    log_top -= 1;
                } else if (stable_btn == BTN_3) {
                    log_top = bottom_top;
                    log_pinned_bottom = true;
                } else if (stable_btn == BTN_4) {
                    log_top += 1;
                } else if (stable_btn == BTN_5) {
                    log_top += lines_per_screen;
                }

                if (log_top < 0) log_top = 0;
                if (log_top > bottom_top) log_top = bottom_top;

                // pinned if at bottom
                log_pinned_bottom = (log_top == bottom_top);
            }

            // ===== Render Log View =====
            u8g2_ClearBuffer(&u8g2);
            u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);

            int ascent = u8g2_GetAscent(&u8g2);
            int descent = u8g2_GetDescent(&u8g2);   // negative
            int line_h = ascent - descent;
            if (line_h <= 0) line_h = 6;

            int lines_per_screen = SCREEN_H / line_h;
            if (lines_per_screen < 1) lines_per_screen = 1;

            char linebuf[LOG_LINE_MAX];

            for (int i = 0; i < lines_per_screen; i++) {
                int idx = log_top + i;
                if (idx >= (int)log_count()) break;

                log_copy_line((uint16_t)idx, linebuf, sizeof(linebuf));

                // Apply horizontal scroll in characters
                int len = (int)strlen(linebuf);
                const char *p = linebuf;
                if (log_horiz < len) p = linebuf + log_horiz;
                else p = "";

                int y = i * line_h + ascent;
                u8g2_DrawStr(&u8g2, 0, y, p);
            }

            u8g2_SendBuffer(&u8g2);
            vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
            continue; // skip normal UI while in log view
        }



        if (!stop_on) {
            // Determine rudder target
            if (in_auto) {
                if (ap_on) {
                    float err = shortest_delta_deg(head_deg, (float)command_deg);
                    if (fabsf(err) < SNAP_DEG) {
                        head_deg = (float)command_deg;
                        rudder_target_deg = 0.0f;
                    } else {
                        float norm = err / ERROR_FOR_FULL_RUDDER_DEG;
                        if (norm >  1.0f) norm =  1.0f;
                        if (norm < -1.0f) norm = -1.0f;
                        rudder_target_deg = norm * MAX_RUDDER_DEG;
                    }
                } else {
                    rudder_target_deg = 0.0f;
                }
            } else if (in_manual) {
                if (pot_init) {
                    float pot_deg = ((pot_filt / 4095.0f) * 2.0f - 1.0f) * MAX_RUDDER_DEG;

                    if (fabsf(pot_filt - pot_last) > 35.0f) {
                        manual_using_jog = false;
                    }
                    pot_last = pot_filt;

                    if (!manual_using_jog) {
                        manual_rudder_deg = pot_deg;
                    }
                }

                if (manual_rudder_deg >  MAX_RUDDER_DEG) manual_rudder_deg =  MAX_RUDDER_DEG;
                if (manual_rudder_deg < -MAX_RUDDER_DEG) manual_rudder_deg = -MAX_RUDDER_DEG;

                rudder_target_deg = manual_rudder_deg;
            } else {
                rudder_target_deg = 0.0f;
            }

            // Rudder smoothing
            float tau = in_manual ? 0.03f : RUDDER_TAU_SEC;
            float alpha = 1.0f - expf(-dt / tau);
            rudder_deg = rudder_deg + alpha * (rudder_target_deg - rudder_deg);

            // Heading response driven by rudder
            float rudder_norm = rudder_deg / MAX_RUDDER_DEG;
            if (rudder_norm >  1.0f) rudder_norm =  1.0f;
            if (rudder_norm < -1.0f) rudder_norm = -1.0f;

            float turn_rate = rudder_norm * TURN_RATE_FULL_DEG_PER_SEC;
            float prev_head = head_deg;
            head_deg = wrap360f(head_deg + turn_rate * dt);

            if (in_auto && ap_on) {
                float eb = shortest_delta_deg(prev_head, (float)command_deg);
                float ea = shortest_delta_deg(head_deg, (float)command_deg);
                if ((eb * ea) < 0.0f && fabsf(ea) < 2.0f) {
                    head_deg = (float)command_deg;
                }
            }
        }

        // ---------------- UI DRAW (line order fixed) ----------------
        const int Y_TITLE_BASE = 9;

        const int Y_BAR_TOP = 10;
        const int H_BAR = 12;

        const int Y_MODE_BASE = 31;
        const int Y_INFO_BASE = 41;

        const int Y_BTNS_TOP = 42;
        const int H_BTNS = 12;

        const int Y_STOP_TOP = 54; // STOP box will be <= 10px tall

        u8g2_ClearBuffer(&u8g2);

        // Line 1
        draw_title_centered(demo_mode ? "DEMO-Mode" : "Inno-Remote", Y_TITLE_BASE);

        // Line 2 (rudder graph)
        float rudder_norm_ui = rudder_deg / MAX_RUDDER_DEG;
        draw_rudder_bar(0, Y_BAR_TOP, SCREEN_W, H_BAR, rudder_norm_ui);

        // Line 3 (MODE centered)
        char mode_line[24];
        snprintf(mode_line, sizeof(mode_line), "MODE: %s", mode_str(auto_low, manual_low));
        draw_centered_line_6x10(mode_line, Y_MODE_BASE);

        // Line 4 (Head/Cmnd or Head/POT)
        int head_disp = wrap360((int)lroundf(head_deg));
        if (in_auto) {
            char left[16], right[16];
            snprintf(left,  sizeof(left),  "Head:%03d", head_disp);
            snprintf(right, sizeof(right), "Cmnd:%03d", wrap360(command_deg));
            draw_left_right_6x10(left, right, Y_INFO_BASE);
        } else if (in_manual) {
            char left[16], right[16];
            snprintf(left,  sizeof(left),  "Head:%03d", head_disp);
            snprintf(right, sizeof(right), "POT:%4d", (raw_pot < 0 ? 0 : raw_pot));
            draw_left_right_6x10(left, right, Y_INFO_BASE);
        } else {
            draw_centered_line_6x10("MODE ERROR", Y_INFO_BASE);
        }

        // Line 5 (buttons row)
        const int gap = 1;
        const int box_w = 24;
        const int total_w = 5 * box_w + 4 * gap;
        int start_x = (SCREEN_W - total_w) / 2;
        if (start_x < 0) start_x = 0;

        const char *lbl1, *lbl2, *lbl3, *lbl4, *lbl5;
        bool latch3 = false;

        if (in_manual) {
            lbl1 = "<<"; lbl2 = "<"; lbl3 = "|"; lbl4 = ">"; lbl5 = ">>";
        } else {
            lbl1 = "-10"; lbl2 = "-1"; lbl3 = "AP"; lbl4 = "+1"; lbl5 = "+10";
            latch3 = ap_on;
        }

        bool b1_on = (stable_btn == BTN_1);
        bool b2_on = (stable_btn == BTN_2);
        bool b3_on = latch3 || (stable_btn == BTN_3);
        bool b4_on = (stable_btn == BTN_4);
        bool b5_on = (stable_btn == BTN_5);

        if (stop_on) b3_on = (stable_btn == BTN_3);

        draw_button_box(start_x + 0*(box_w+gap), Y_BTNS_TOP, box_w, H_BTNS, lbl1, b1_on);
        draw_button_box(start_x + 1*(box_w+gap), Y_BTNS_TOP, box_w, H_BTNS, lbl2, b2_on);
        draw_button_box(start_x + 2*(box_w+gap), Y_BTNS_TOP, box_w, H_BTNS, lbl3, b3_on);
        draw_button_box(start_x + 3*(box_w+gap), Y_BTNS_TOP, box_w, H_BTNS, lbl4, b4_on);
        draw_button_box(start_x + 4*(box_w+gap), Y_BTNS_TOP, box_w, H_BTNS, lbl5, b5_on);

        // Line 6 (STOP snug)
        draw_stop_snug(Y_STOP_TOP, stop_on);

        draw_wifi_icon_top_right();
        u8g2_SendBuffer(&u8g2);

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}
