/**
 * Wi-Fi settings for the bench (practice) build. Change them here.
 *
 * Default: the robot creates its OWN network (access point). Join it from the
 * phone/laptop and open http://192.168.4.1 — no router needed, works anywhere.
 *
 * Optional: set WIFI_STA_SSID to join an existing network instead. If it does
 * not connect within WIFI_STA_TIMEOUT_MS, the robot falls back to its own AP.
 */
#pragma once

#define WIFI_AP_SSID        "SYROBIX-BENCH"
#define WIFI_AP_PASS        "syrobix2026"     // >= 8 characters. CHANGE IT.
#define WIFI_AP_CHANNEL     6

#define WIFI_STA_SSID       ""                // "" = do not try, use the AP
#define WIFI_STA_PASS       ""
#define WIFI_STA_TIMEOUT_MS 8000

#define WIFI_HTTP_PORT      80
#define WIFI_WS_PORT        81

/** Lower TX power = smaller current spikes on the 3.3 V rail (fewer brown-out
 *  resets when the robot runs off USB). 8.5 dBm is plenty for a few metres. */
#define WIFI_TX_POWER       WIFI_POWER_8_5dBm
