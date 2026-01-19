#include "wifi_status.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_log.h"

static const char *TAG = "WIFI_UI";

static volatile bool s_assoc = false;
static volatile bool s_has_ip = false;
static float s_rssi_smooth = -90.0f;     // start low
static volatile int s_arcs = 0;

static int rssi_to_arcs(float rssi_dbm)
{
    // Tune later if you want; these are sane defaults
    if (rssi_dbm >= -60.0f) return 3;
    if (rssi_dbm >= -70.0f) return 2;
    if (rssi_dbm >= -80.0f) return 1;
    return 0;
}

wifi_ui_status_t wifi_ui_get_status(void)
{
    wifi_ui_status_t st;
    st.assoc = s_assoc;
    st.has_ip = s_has_ip;
    st.rssi_dbm = (int)(s_rssi_smooth + (s_rssi_smooth >= 0 ? 0.5f : -0.5f));
    st.arcs = s_arcs;
    return st;
}

void wifi_ui_set_assoc(bool assoc)
{
    s_assoc = assoc;
    if (!assoc) {
        s_has_ip = false;
        s_arcs = 0;
    }
}

void wifi_ui_set_has_ip(bool has_ip)
{
    s_has_ip = has_ip;
}

static void rssi_task(void *arg)
{
    while (1) {
        if (s_assoc) {
            int rssi = 0;
            esp_err_t err = esp_wifi_sta_get_rssi(&rssi);
            if (err == ESP_OK) {
                // Smooth so arcs don't flicker
                s_rssi_smooth = 0.80f * s_rssi_smooth + 0.20f * (float)rssi;
                s_arcs = rssi_to_arcs(s_rssi_smooth);
            } else {
                // If temporarily unavailable, don't change state aggressively
                ESP_LOGD(TAG, "esp_wifi_sta_get_rssi err=%d", (int)err);
            }
        } else {
            s_arcs = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void wifi_ui_start_rssi_task(void)
{
    xTaskCreate(rssi_task, "wifi_rssi", 3072, NULL, 3, NULL);
}
