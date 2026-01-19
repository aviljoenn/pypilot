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

// ------------------------------ Main ---------------------------------------

void app_main(void)
{
    init_gpio_inputs();
    init_adc();
    init_oled_u8g2();

    wifi_sta_start();

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
        draw_title_centered("Inno-Remote", Y_TITLE_BASE);

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
