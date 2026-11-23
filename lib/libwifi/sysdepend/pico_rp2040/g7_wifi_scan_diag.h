/* Read-only Wi-Fi discovery results; the CYW43 owner alone scans. */
#ifndef G7_WIFI_SCAN_DIAG_H
#define G7_WIFI_SCAN_DIAG_H
#include <stdint.h>
#define G7_SCAN_MAX_APS 8U
#define G7_SCAN_SSID_CAP 33U
typedef struct {
    char ssid[G7_SCAN_SSID_CAP];
    int32_t rssi;
    int32_t channel;
    int32_t auth;
} G7_WIFI_AP;
typedef struct {
    uint32_t generation;
    uint32_t active;
    uint32_t complete;
    int32_t request_rc;
    uint32_t seen_total;
    uint32_t stored;
    uint32_t target_seen;
    G7_WIFI_AP ap[G7_SCAN_MAX_APS];
} G7_WIFI_SCAN;
void g7_wifi_scan_snapshot(G7_WIFI_SCAN *out);
#endif
