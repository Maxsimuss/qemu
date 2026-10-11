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
    uint8_t negotiated_rsn[257];
    uint16_t negotiated_rsn_len;
    uint8_t pmk[32];
    uint8_t anonce[32];
    uint8_t snonce[32];
    uint8_t ptk[64];
    uint8_t gtk[16];
    uint8_t gtk_key_id;
    uint64_t replay;
    /* CCMP packet numbers are independent for each temporal key. */
    uint64_t tx_pn;
    uint64_t rx_pn;
    uint64_t group_tx_pn;
    uint64_t group_rx_pn;
    bool configured;
    bool associated;
    bool m3_sent;
    bool keys_installed;
} Esp32Wpa2Peer;

typedef struct Esp32WifiKey {
    uint8_t bytes[32];
    uint8_t length;
    uint8_t index;
    uint8_t interface_id;
    uint8_t cipher;
} Esp32WifiKey;

/* The ESP32 MAC key-table encoding observed in the supplied IDF ELF uses
 * cipher value 3 for CCMP (the public esp_wifi cipher enum uses value 4). */
#define ESP32_WIFI_KEY_CIPHER_CCMP 3

/* The MAC packs these fields into metadata word 1 of each 40-byte key
 * record. Physical group slots are allocator-selected and do not equal the
 * 802.11 GTK KeyID. */
#define ESP32_WIFI_KEY_META_INTERFACE_SHIFT 24
#define ESP32_WIFI_KEY_META_INTERFACE_MASK  0x3
#define ESP32_WIFI_KEY_META_ROLE_SHIFT      21
#define ESP32_WIFI_KEY_META_ROLE_MASK       0x7
#define ESP32_WIFI_KEY_META_CIPHER_SHIFT    18
#define ESP32_WIFI_KEY_META_CIPHER_MASK     0x7
#define ESP32_WIFI_KEY_META_GROUP_ID_SHIFT  30
#define ESP32_WIFI_KEY_META_GROUP_ID_MASK   0x3
/* Observed group records set bit 23 and pairwise records set bit 21 in the
 * three-bit role/permission field. Treat these as role markers instead of
 * requiring the entire field to equal one captured encoding. */
#define ESP32_WIFI_KEY_META_GROUP_ROLE      (1U << 2)
#define ESP32_WIFI_KEY_META_PAIRWISE_ROLE   (1U << 0)

/* Resolve keys from the guest-programmed hardware table. Pairwise lookup
 * searches hardware slots 4..31 and matches record class, interface, cipher,
 * and peer address. Group records occupy slots 0..3 and are selected by
 * their encoded GTK KeyID, not by physical slot number. */
bool esp32_wifi_key_lookup(const uint8_t table[32][40], uint32_t valid,
                           uint8_t interface_id, uint8_t cipher,
                           const uint8_t peer[6], int group_key_id,
                           Esp32WifiKey *key);

void esp32_wpa2_configure(Esp32Wpa2Peer *peer, const char *ssid,
                          const char *password, const uint8_t bssid[6]);
bool esp32_wpa2_assoc_request(Esp32Wpa2Peer *peer,
                              const mac80211_frame *frame);
uint16_t esp32_wpa2_assoc_request_status(Esp32Wpa2Peer *peer,
                                         const mac80211_frame *frame);
mac80211_frame *esp32_wpa2_assoc_response(Esp32Wpa2Peer *peer);
mac80211_frame *esp32_wpa2_assoc_reject_response(const uint8_t bssid[6],
                                                 const uint8_t station[6],
                                                 uint16_t status);
mac80211_frame *esp32_wpa2_start(Esp32Wpa2Peer *peer);
mac80211_frame *esp32_wpa2_rx_eapol(Esp32Wpa2Peer *peer,
                                    const mac80211_frame *frame);
const uint8_t *esp32_wpa2_expected_data_key(const Esp32Wpa2Peer *peer,
                                           const uint8_t receiver[6]);
bool esp32_wpa2_decrypt_data(Esp32Wpa2Peer *peer, mac80211_frame *frame,
                             const uint8_t key[16]);
bool esp32_wpa2_encrypt_data(Esp32Wpa2Peer *peer, mac80211_frame *frame,
                             const uint8_t key[16]);
bool esp32_ccmp_decrypt(mac80211_frame *frame, const uint8_t key[16],
                        uint64_t *last_pn);
bool esp32_ccmp_encrypt(mac80211_frame *frame, const uint8_t key[16],
                        uint8_t key_id, uint64_t *next_pn);

#endif
