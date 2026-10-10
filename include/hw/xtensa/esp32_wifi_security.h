/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef ESP32_WIFI_SECURITY_H
#define ESP32_WIFI_SECURITY_H

#include "qemu/osdep.h"
#include "hw/xtensa/esp32_wlan.h"

typedef struct Esp32Wpa2Peer {
    char ssid[33];
    char password[64];
    uint8_t bssid[6];
    uint8_t station[6];
    uint8_t pmk[32];
    uint8_t anonce[32];
    uint8_t snonce[32];
    uint8_t ptk[64];
    uint8_t gtk[16];
    uint64_t replay;
    uint64_t tx_pn;
    uint64_t rx_pn;
    bool configured;
    bool associated;
    bool m3_sent;
    bool keys_installed;
} Esp32Wpa2Peer;

void esp32_wpa2_configure(Esp32Wpa2Peer *peer, const char *ssid,
                          const char *password, const uint8_t bssid[6]);
bool esp32_wpa2_assoc_request(Esp32Wpa2Peer *peer,
                              const mac80211_frame *frame);
mac80211_frame *esp32_wpa2_assoc_response(Esp32Wpa2Peer *peer);
mac80211_frame *esp32_wpa2_start(Esp32Wpa2Peer *peer);
mac80211_frame *esp32_wpa2_rx_eapol(Esp32Wpa2Peer *peer,
                                    const mac80211_frame *frame);
bool esp32_wpa2_decrypt_data(Esp32Wpa2Peer *peer, mac80211_frame *frame);
bool esp32_wpa2_encrypt_data(Esp32Wpa2Peer *peer, mac80211_frame *frame);

#endif
