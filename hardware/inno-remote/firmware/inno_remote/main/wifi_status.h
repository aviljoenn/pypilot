#pragma once
#include <stdbool.h>

typedef struct {
    bool assoc;
    bool has_ip;
    int  rssi_dbm;      // smoothed, e.g. -62
    int  arcs;          // 0..3
} wifi_ui_status_t;

wifi_ui_status_t wifi_ui_get_status(void);
void wifi_ui_set_assoc(bool assoc);
void wifi_ui_set_has_ip(bool has_ip);
void wifi_ui_start_rssi_task(void);   // call once after wifi start
