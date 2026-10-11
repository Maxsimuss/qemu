/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Small WPA2-PSK peer implementation for the original ESP32 WiFi MAC. */
#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "crypto/cipher.h"
#include "crypto/hmac.h"
#include "crypto/pbkdf.h"
#include "crypto/random.h"
#include "qapi/error.h"
#include "hw/xtensa/esp32_wifi_security.h"

#define WPA2_EAPOL_VERSION 2
#define WPA2_EAPOL_KEY 3
#define WPA2_KEY_DESC_RSN 2
#define WPA2_KEY_INFO_PAIRWISE 0x0008
#define WPA2_KEY_INFO_INSTALL 0x0040
#define WPA2_KEY_INFO_ACK 0x0080
#define WPA2_KEY_INFO_MIC 0x0100
#define WPA2_KEY_INFO_SECURE 0x0200
#define WPA2_KEY_INFO_ENCRYPTED 0x1000

static const uint8_t wpa2_ap_rsn_ie[] = {
    48, 20, 1,0, 0,0x0f,0xac,4, 1,0, 0,0x0f,0xac,4,
    1,0, 0,0x0f,0xac,2, 0,0
};

bool esp32_wifi_key_lookup(const uint8_t table[32][40], uint32_t valid,
                           uint8_t interface_id, uint8_t cipher,
                           const uint8_t peer[6], int group_key_id,
                           Esp32WifiKey *key)
{
    unsigned first, last;
    unsigned matches = 0;
    Esp32WifiKey selected = { 0 };

    if (!table || !key || interface_id > 2 || !peer ||
        cipher != ESP32_WIFI_KEY_CIPHER_CCMP) {
        return false;
    }

    if (group_key_id >= 0) {
        if (group_key_id > 3) {
            return false;
        }
        first = 0;
        last = 3;
    } else {
        /* The driver's allocator search range is not a hardware key-table
         * limit. Inspect all pairwise slots and use record metadata to
         * distinguish pairwise entries from other valid entries. */
        first = 4;
        last = 31;
    }

    for (unsigned i = first; i <= last; i++) {
        const uint8_t *record = table[i];
        uint32_t word0, word1;
        uint8_t address[6], record_if, record_cipher, record_role;

        if (!(valid & (1U << i))) {
            continue;
        }
        word0 = ldl_le_p(record);
        word1 = ldl_le_p(record + 4);
        record_if = (word1 >> ESP32_WIFI_KEY_META_INTERFACE_SHIFT) &
                    ESP32_WIFI_KEY_META_INTERFACE_MASK;
        record_role = (word1 >> ESP32_WIFI_KEY_META_ROLE_SHIFT) &
                      ESP32_WIFI_KEY_META_ROLE_MASK;
        record_cipher = (word1 >> ESP32_WIFI_KEY_META_CIPHER_SHIFT) &
                        ESP32_WIFI_KEY_META_CIPHER_MASK;
        address[0] = word0;
        address[1] = word0 >> 8;
        address[2] = word0 >> 16;
        address[3] = word0 >> 24;
        /* The supplied MAC writes addr4 in word1[7:0] and addr5 in
         * word1[15:8]; this is not the byte order used by the public key
         * structure passed to ic_set_key(). */
        address[4] = word1;
        address[5] = word1 >> 8;
        if (record_if != interface_id || record_cipher != cipher ||
            memcmp(address, peer, sizeof(address))) {
            continue;
        }
        if (group_key_id >= 0) {
            uint8_t record_group_id =
                (word1 >> ESP32_WIFI_KEY_META_GROUP_ID_SHIFT) &
                ESP32_WIFI_KEY_META_GROUP_ID_MASK;

            if (!(record_role & ESP32_WIFI_KEY_META_GROUP_ROLE) ||
                record_group_id != group_key_id) {
                continue;
            }
        } else if (!(record_role & ESP32_WIFI_KEY_META_PAIRWISE_ROLE)) {
            continue;
        }

        /* There is one active record per GTK ID/interface and one pairwise
         * record per peer. Refuse ambiguous tables rather than selecting a
         * physical slot by accident. */
        if (++matches > 1) {
            return false;
        }
        memset(&selected, 0, sizeof(selected));
        memcpy(selected.bytes, record + 8, 16);
        selected.length = 16;
        selected.index = i;
        selected.interface_id = record_if;
        selected.cipher = record_cipher;
    }
    if (!matches) {
        return false;
    }
    *key = selected;
    return true;
}

static uint16_t get_be16(const uint8_t *p)
{
    return (p[0] << 8) | p[1];
}

static uint16_t get_le16(const uint8_t *p)
{
    return p[0] | (p[1] << 8);
}

static void put_be16(uint8_t *p, uint16_t v)
{
    p[0] = v >> 8;
    p[1] = v;
}

static void put_be64(uint8_t *p, uint64_t v)
{
    for (int i = 7; i >= 0; i--) {
        p[i] = v;
        v >>= 8;
    }
}

static uint64_t get_be64(const uint8_t *p)
{
    uint64_t v = 0;
    for (unsigned i = 0; i < 8; i++) {
        v = (v << 8) | p[i];
    }
    return v;
}

static bool aes_block(const uint8_t key[16], const uint8_t in[16],
                      uint8_t out[16], bool decrypt)
{
    Error *err = NULL;
    QCryptoCipher *c = qcrypto_cipher_new(QCRYPTO_CIPHER_ALGO_AES_128,
                                          QCRYPTO_CIPHER_MODE_ECB,
                                          key, 16, &err);
    int ret;

    if (!c) {
        error_free(err);
        return false;
    }
    ret = decrypt ? qcrypto_cipher_decrypt(c, in, out, 16, &err) :
                    qcrypto_cipher_encrypt(c, in, out, 16, &err);
    qcrypto_cipher_free(c);
    error_free(err);
    return ret == 0;
}

static bool hmac_sha1(const uint8_t *key, size_t keylen,
                      const uint8_t *data, size_t len, uint8_t out[20])
{
    Error *err = NULL;
    QCryptoHmac *h = qcrypto_hmac_new(QCRYPTO_HASH_ALGO_SHA1, key, keylen,
                                      &err);
    uint8_t *outp = out;
    size_t outlen = 20;
    bool ok = h && qcrypto_hmac_bytes(h, (const char *)data, len,
                                      &outp, &outlen, &err) == 0 &&
              outlen == 20;

    qcrypto_hmac_free(h);
    error_free(err);
    return ok;
}

static bool derive_ptk(const Esp32Wpa2Peer *p, const uint8_t snonce[32],
                      uint8_t ptk[64])
{
    static const uint8_t label[] = "Pairwise key expansion";
    uint8_t context[76], input[sizeof(label) + sizeof(context) + 1];
    uint8_t digest[20];
    size_t off = 0;

    if (memcmp(p->bssid, p->station, 6) < 0) {
        memcpy(context, p->bssid, 6);
        memcpy(context + 6, p->station, 6);
    } else {
        memcpy(context, p->station, 6);
        memcpy(context + 6, p->bssid, 6);
    }
    if (memcmp(p->anonce, snonce, 32) < 0) {
        memcpy(context + 12, p->anonce, 32);
        memcpy(context + 44, snonce, 32);
    } else {
        memcpy(context + 12, snonce, 32);
        memcpy(context + 44, p->anonce, 32);
    }
    /* sizeof(label) includes the single NUL separator required by the WPA
     * PRF. Keep it exactly once before the ordered address/nonce context. */
    memcpy(input, label, sizeof(label));
    memcpy(input + sizeof(label), context, sizeof(context));
    for (uint8_t i = 0; off < 64; i++) {
        input[sizeof(label) + sizeof(context)] = i;
        if (!hmac_sha1(p->pmk, sizeof(p->pmk), input, sizeof(input), digest)) {
            return false;
        }
        size_t n = MIN(sizeof(digest), 64 - off);
        memcpy(ptk + off, digest, n);
        off += n;
    }
    return true;
}

static bool aes_wrap(const uint8_t kek[16], const uint8_t *plain, size_t len,
                     uint8_t *wrapped)
{
    uint8_t a[8], block[16], enc[16];
    size_t n = len / 8;

    if (!len || len % 8 || n > 32) {
        return false;
    }
    memset(a, 0xa6, sizeof(a));
    memcpy(wrapped + 8, plain, len);
    for (unsigned j = 0; j < 6; j++) {
        for (size_t i = 1; i <= n; i++) {
            memcpy(block, a, 8);
            memcpy(block + 8, wrapped + 8 * i, 8);
            if (!aes_block(kek, block, enc, false)) {
                return false;
            }
            uint64_t t = n * j + i;
            memcpy(a, enc, 8);
            for (int k = 7; k >= 0; k--) {
                a[k] ^= t;
                t >>= 8;
            }
            memcpy(wrapped + 8 * i, enc + 8, 8);
        }
    }
    memcpy(wrapped, a, 8);
    return true;
}

static bool wpa2_m3_key_data(const Esp32Wpa2Peer *p, uint8_t *out,
                             size_t out_size, size_t *out_len)
{
    uint8_t kde[24] = {0xdd,22,0,0x0f,0xac,1,0,0};
    size_t len = sizeof(wpa2_ap_rsn_ie) + sizeof(kde);
    size_t padded_len = (len + 7) & ~(size_t)7;

    if (!p || !out || !out_len || !p->associated ||
        !p->negotiated_rsn_len || padded_len > out_size ||
        p->gtk_key_id > 3) {
        return false;
    }
    kde[6] = p->gtk_key_id;
    memcpy(kde + 8, p->gtk, 16);
    memcpy(out, wpa2_ap_rsn_ie, sizeof(wpa2_ap_rsn_ie));
    memcpy(out + sizeof(wpa2_ap_rsn_ie), kde, sizeof(kde));
    if (padded_len != len) {
        out[len] = 0xdd;
        memset(out + len + 1, 0, padded_len - len - 1);
    }
    *out_len = padded_len;
    return true;
}

static bool wpa2_m2_rsn_matches(const Esp32Wpa2Peer *p,
                                const uint8_t *key_data, size_t key_data_len)
{
    size_t off = 0;
    bool found_rsn = false;

    if (!p || !key_data || !p->negotiated_rsn_len) {
        return false;
    }
    while (off < key_data_len) {
        size_t ie_len;

        if (key_data_len - off < 2) {
            return false;
        }
        ie_len = (size_t)key_data[off + 1] + 2;
        if (ie_len > key_data_len - off) {
            return false;
        }
        if (key_data[off] == 48) {
            if (found_rsn || ie_len != p->negotiated_rsn_len ||
                memcmp(key_data + off, p->negotiated_rsn, ie_len)) {
                return false;
            }
            found_rsn = true;
        }
        off += ie_len;
    }
    return found_rsn;
}

void esp32_wpa2_configure(Esp32Wpa2Peer *p, const char *ssid,
                          const char *password, const uint8_t bssid[6])
{
    Error *err = NULL;
    memset(p, 0, sizeof(*p));
    if (!ssid || !password || strlen(ssid) > 32 || strlen(password) < 8 ||
        strlen(password) > 63) {
        return;
    }
    g_strlcpy(p->ssid, ssid, sizeof(p->ssid));
    g_strlcpy(p->password, password, sizeof(p->password));
    memcpy(p->bssid, bssid, 6);
    /* GTK KeyID is independent of the physical slot the firmware allocator
     * chooses for the corresponding key-table record. */
    p->gtk_key_id = 0;
    p->configured = qcrypto_pbkdf2(QCRYPTO_HASH_ALGO_SHA1,
        (const uint8_t *)p->password, strlen(p->password),
        (const uint8_t *)p->ssid, strlen(p->ssid), 4096,
        p->pmk, sizeof(p->pmk), &err) == 0;
    error_free(err);
}

static uint16_t wpa2_assoc_parse_rsn(const uint8_t *ie, size_t ie_len)
{
    static const uint8_t ccmp[] = { 0, 0x0f, 0xac, 4 };
    static const uint8_t psk[] = { 0, 0x0f, 0xac, 2 };
    size_t end = ie_len, off = 2;
    uint16_t count;
    bool has_ccmp = false, has_psk = false;

    if (ie_len < 2 || ie[0] != 48 || (size_t)ie[1] + 2 != ie_len) {
        return 40; /* invalid information element */
    }
    if (ie[1] < 18) {
        return 40;
    }
    if (get_le16(ie + off) != 1) {
        return 44; /* unsupported RSN version */
    }
    off += 2;
    if (off + 4 > end) {
        return 40;
    }
    if (memcmp(ie + off, ccmp, sizeof(ccmp))) {
        return 41; /* invalid group cipher */
    }
    off += 4;
    if (off + 2 > end) {
        return 40;
    }
    count = get_le16(ie + off);
    off += 2;
    if (!count || count > (end - off) / 4) {
        return 40;
    }
    for (unsigned i = 0; i < count; i++, off += 4) {
        has_ccmp |= !memcmp(ie + off, ccmp, sizeof(ccmp));
    }
    if (!has_ccmp) {
        return 42; /* no compatible pairwise cipher */
    }
    if (off + 2 > end) {
        return 40;
    }
    count = get_le16(ie + off);
    off += 2;
    if (!count || count > (end - off) / 4) {
        return 40;
    }
    for (unsigned i = 0; i < count; i++, off += 4) {
        has_psk |= !memcmp(ie + off, psk, sizeof(psk));
    }
    if (!has_psk) {
        return 43; /* no compatible AKM */
    }

    /* RSN capabilities, PMKID list, and group-management suite are optional.
     * Consume each only when the preceding field is present and validate its
     * complete length before inspecting it. */
    if (off < end) {
        uint16_t capabilities;

        if (end - off < 2) {
            return 40;
        }
        capabilities = get_le16(ie + off);
        off += 2;
        /* This AP does not advertise PMF support, so it cannot satisfy a
         * station that requires protected management frames. */
        if (capabilities & (1U << 6)) {
            return 45;
        }
    }
    if (off < end) {
        if (end - off < 2) {
            return 40;
        }
        count = get_le16(ie + off);
        off += 2;
        if (count > (end - off) / 16) {
            return 40;
        }
        off += (size_t)count * 16;
    }
    if (off < end) {
        if (end - off != 4) {
            return 40;
        }
        off += 4;
    }
    return off == end ? 0 : 40;
}

uint16_t esp32_wpa2_assoc_request_status(Esp32Wpa2Peer *p,
                                         const mac80211_frame *f)
{
    const uint8_t *ie, *rsn = NULL;
    uint8_t station_rates[sizeof(f->data_and_fcs)];
    size_t n, fixed_len, rsn_len = 0, station_rate_count = 0;
    bool has_ssid = false, has_rates = false, has_ext_rates = false;
    uint16_t status = 40;

    if (!p || !f || !p->configured ||
        f->frame_control.type != IEEE80211_TYPE_MGT ||
        (f->frame_control.sub_type != IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_REQ &&
         f->frame_control.sub_type != IEEE80211_TYPE_MGT_SUBTYPE_REASSOCIATION_REQ) ||
        f->frame_length < IEEE80211_HEADER_SIZE + 4 + 4 ||
        f->frame_length - IEEE80211_HEADER_SIZE - 4 > sizeof(f->data_and_fcs)) {
        return 40;
    }

    n = f->frame_length - IEEE80211_HEADER_SIZE - 4; /* exclude FCS */
    fixed_len = f->frame_control.sub_type ==
                IEEE80211_TYPE_MGT_SUBTYPE_REASSOCIATION_REQ ? 10 : 4;
    if (n < fixed_len) {
        return 40;
    }
    /* Association: capability + listen interval. Reassociation adds the
     * current AP address, so neither format may be parsed as TLVs early. */
    if ((get_le16(f->data_and_fcs) & 0x0003) != 0x0001 ||
        !get_le16(f->data_and_fcs + 2)) {
        return 40;
    }
    if (fixed_len == 10 && memcmp(f->data_and_fcs + 4, p->bssid, 6)) {
        return 1; /* unspecified association denial for another current AP */
    }

    ie = f->data_and_fcs + fixed_len;
    n -= fixed_len;
    while (n) {
        size_t ie_len;

        if (n < 2) {
            return 40;
        }
        ie_len = (size_t)ie[1] + 2;
        if (ie_len > n) {
            return 40;
        }
        switch (ie[0]) {
        case 0: /* SSID */
            if (has_ssid || ie[1] != strlen(p->ssid) ||
                memcmp(ie + 2, p->ssid, ie[1])) {
                return 1;
            }
            has_ssid = true;
            break;
        case 1: /* Supported Rates */
            if (has_rates || !ie[1] || ie[1] > 8) {
                return 40;
            }
            if (station_rate_count + ie[1] > sizeof(station_rates)) {
                return 40;
            }
            has_rates = true;
            for (size_t i = 0; i < ie[1]; i++) {
                station_rates[station_rate_count++] = ie[2 + i];
            }
            break;
        case 50: /* Extended Supported Rates */
            if (has_ext_rates || !ie[1]) {
                return 40;
            }
            if (station_rate_count + ie[1] > sizeof(station_rates)) {
                return 40;
            }
            has_ext_rates = true;
            for (size_t i = 0; i < ie[1]; i++) {
                station_rates[station_rate_count++] = ie[2 + i];
            }
            break;
        case 48: /* RSN */
            if (rsn) {
                return 40;
            }
            status = wpa2_assoc_parse_rsn(ie, ie_len);
            if (status) {
                return status;
            }
            rsn = ie;
            rsn_len = ie_len;
            break;
        default:
            /* Other bounded information elements do not alter this peer's
             * WPA2-PSK/CCMP selection. */
            break;
        }
        ie += ie_len;
        n -= ie_len;
    }
    if (!has_ssid || !has_rates || !rsn || !rsn_len ||
        rsn_len > sizeof(p->negotiated_rsn)) {
        return 40;
    }

    /* Every basic rate advertised by this AP must be supported by the STA;
     * optional rates may be ordered differently or omitted. */
    static const uint8_t basic_rates[] = { 0x8b, 0x96, 0x82, 0x84 };
    for (size_t i = 0; i < ARRAY_SIZE(basic_rates); i++) {
        bool found = false;
        for (size_t j = 0; j < station_rate_count; j++) {
            found |= (station_rates[j] & 0x7f) == (basic_rates[i] & 0x7f);
        }
        if (!found) {
            return 18; /* basic rate set is unsupported */
        }
    }
    if (p->associated &&
        (memcmp(p->station, f->transmitter_address, sizeof(p->station)) ||
         p->negotiated_rsn_len != rsn_len ||
         memcmp(p->negotiated_rsn, rsn, rsn_len))) {
        return 40;
    }
    /* Commit negotiated state only after the complete frame is valid. */
    if (!p->associated) {
        memcpy(p->station, f->transmitter_address, sizeof(p->station));
        memcpy(p->negotiated_rsn, rsn, rsn_len);
        p->negotiated_rsn_len = rsn_len;
    }
    return 0;
}

bool esp32_wpa2_assoc_request(Esp32Wpa2Peer *p, const mac80211_frame *f)
{
    return esp32_wpa2_assoc_request_status(p, f) == 0;
}

mac80211_frame *esp32_wpa2_assoc_response(Esp32Wpa2Peer *p)
{
    static const uint8_t rates[] = {
        1,8,0x8b,0x96,0x82,0x84,0x0c,0x18,0x30,0x60,
        50,4,0x6c,0x12,0x24,0x48
    };
    mac80211_frame *f;

    if (!p || !p->configured) {
        return NULL;
    }
    f = g_malloc0(sizeof(*f));
    f->frame_control.type = IEEE80211_TYPE_MGT;
    f->frame_control.sub_type = IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_RESP;
    f->duration_id = 314;
    f->pos = 0;
    /* Match the WPA2 beacon's ESS and privacy capabilities. */
    const uint8_t fixed[] = {0x11,0x00,0,0,1,0xc0};
    memcpy(f->data_and_fcs, fixed, sizeof(fixed));
    memcpy(f->data_and_fcs + sizeof(fixed), rates, sizeof(rates));
    memcpy(f->data_and_fcs + sizeof(fixed) + sizeof(rates),
           wpa2_ap_rsn_ie, sizeof(wpa2_ap_rsn_ie));
    memcpy(f->receiver_address, p->station, 6);
    memcpy(f->transmitter_address, p->bssid, 6);
    memcpy(f->address_3, p->bssid, 6);
    f->pos = sizeof(fixed) + sizeof(rates) + sizeof(wpa2_ap_rsn_ie);
    f->frame_length = IEEE80211_HEADER_SIZE + f->pos;
    p->associated = true;
    return f;
}

mac80211_frame *esp32_wpa2_assoc_reject_response(const uint8_t bssid[6],
                                                 const uint8_t station[6],
                                                 uint16_t status)
{
    mac80211_frame *f;
    uint8_t fixed[6] = { 0x11, 0x00, status, status >> 8, 0, 0 };

    if (!bssid || !station || !status) {
        return NULL;
    }
    f = g_malloc0(sizeof(*f));
    f->frame_control.type = IEEE80211_TYPE_MGT;
    f->frame_control.sub_type = IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_RESP;
    f->duration_id = 314;
    memcpy(f->receiver_address, station, 6);
    memcpy(f->transmitter_address, bssid, 6);
    memcpy(f->address_3, bssid, 6);
    memcpy(f->data_and_fcs, fixed, sizeof(fixed));
    f->pos = sizeof(fixed);
    f->frame_length = IEEE80211_HEADER_SIZE + sizeof(fixed);
    return f;
}

static mac80211_frame *eapol_key_frame(Esp32Wpa2Peer *p, uint16_t info,
                                      const uint8_t nonce[32],
                                      const uint8_t *keydata, size_t keydata_len,
                                      bool advance_replay)
{
    mac80211_frame *f;
    uint8_t *d;
    size_t body_len;
    uint8_t mic[20];

    if (!p || keydata_len > sizeof(((mac80211_frame *)0)->data_and_fcs) - 107 ||
        (keydata_len && !keydata) || 95 + keydata_len > UINT16_MAX) {
        return NULL;
    }
    body_len = 95 + keydata_len;
    f = g_malloc0(sizeof(*f));
    d = f->data_and_fcs;

    f->frame_control.type = IEEE80211_TYPE_DATA;
    f->frame_control.sub_type = IEEE80211_TYPE_DATA_SUBTYPE_DATA;
    f->frame_control.from_ds = 1;
    f->frame_length = IEEE80211_HEADER_SIZE + 8 + 4 + body_len;
    f->pos = f->frame_length - IEEE80211_HEADER_SIZE;
    memcpy(d, (uint8_t[]){0xaa,0xaa,3,0,0,0,0x88,0x8e}, 8);
    d[8] = WPA2_EAPOL_VERSION; d[9] = WPA2_EAPOL_KEY;
    put_be16(d + 10, body_len);
    d[12] = WPA2_KEY_DESC_RSN;
    put_be16(d + 13, info);
    put_be16(d + 15, 16);
    if (advance_replay) {
        p->replay++;
    }
    put_be64(d + 17, p->replay);
    if (nonce) memcpy(d + 25, nonce, 32);
    put_be16(d + 105, keydata_len);
    if (keydata_len) memcpy(d + 107, keydata, keydata_len);
    if (info & WPA2_KEY_INFO_MIC) {
        if (!hmac_sha1(p->ptk, 16, d + 8, 4 + body_len, mic)) {
            g_free(f);
            return NULL;
        }
        memcpy(d + 8 + 4 + 77, mic, 16);
    }
    return f;
}

mac80211_frame *esp32_wpa2_start(Esp32Wpa2Peer *p)
{
    if (!p->configured || !p->associated ||
        qcrypto_random_bytes(p->anonce, sizeof(p->anonce), NULL) < 0 ||
        qcrypto_random_bytes(p->gtk, sizeof(p->gtk), NULL) < 0) {
        return NULL;
    }
    p->m3_sent = false;
    p->keys_installed = false;
    p->tx_pn = 0;
    p->rx_pn = 0;
    p->group_tx_pn = 0;
    p->group_rx_pn = 0;
    return eapol_key_frame(p, WPA2_KEY_INFO_PAIRWISE | WPA2_KEY_INFO_ACK | 2,
                           p->anonce, NULL, 0, true);
}

mac80211_frame *esp32_wpa2_rx_eapol(Esp32Wpa2Peer *p,
                                    const mac80211_frame *f)
{
    const uint8_t *e;
    size_t n, body_len, keydata_len, packet_len;
    uint16_t info, expected_info;
    uint8_t mic[20], packet[512], m3_key_data[256], wrapped[264];
    uint8_t candidate_ptk[64], zero_nonce[32] = {0};
    size_t m3_key_data_len;
    Esp32Wpa2Peer pending;
    mac80211_frame *m3;

    /* The EAPOL protocol version is the outer 802.1X header byte, distinct
     * from the EAPOL-Key descriptor version in key_info. ESP-IDF's WPA2
     * supplicant transmits version 1; hostapd commonly uses version 2, and
     * version 3 is the 802.1X-2010 protocol version. */
    if (!p || !f || !p->configured || !p->associated ||
        f->frame_length < IEEE80211_HEADER_SIZE + 8 + 99 ||
        f->frame_length > IEEE80211_HEADER_SIZE + sizeof(f->data_and_fcs) ||
        f->frame_control.type != IEEE80211_TYPE_DATA ||
        f->frame_control.sub_type != IEEE80211_TYPE_DATA_SUBTYPE_DATA ||
        !f->frame_control.to_ds || f->frame_control.from_ds ||
        memcmp(f->transmitter_address, p->station, 6) ||
        memcmp(f->receiver_address, p->bssid, 6) ||
        memcmp(f->address_3, p->bssid, 6)) {
        return NULL;
    }
    e = f->data_and_fcs + 8;
    n = f->frame_length - IEEE80211_HEADER_SIZE - 8;
    if (n < 99 || n > sizeof(packet) ||
        memcmp(f->data_and_fcs, (uint8_t[]){0xaa,0xaa,3,0,0,0,0x88,0x8e}, 8) ||
        e[0] < 1 || e[0] > 3 || e[1] != WPA2_EAPOL_KEY ||
        e[4] != WPA2_KEY_DESC_RSN) {
        return NULL;
    }
    body_len = get_be16(e + 2);
    keydata_len = get_be16(e + 97);
    if (body_len < 95 || body_len > n - 4 ||
        keydata_len > body_len - 95) {
        return NULL;
    }
    packet_len = 4 + body_len;
    if (packet_len > sizeof(packet)) {
        return NULL;
    }
    info = get_be16(e + 5);
    /* WPA2-PSK with CCMP uses the RSN EAPOL-Key descriptor version 2.
     * This is independent of the outer 802.1X EAPOL protocol version above. */
    if ((info & 0x0007) != 2 ||
        !(info & WPA2_KEY_INFO_PAIRWISE) || !(info & WPA2_KEY_INFO_MIC)) {
        return NULL;
    }
    memcpy(packet, e, packet_len);
    memset(packet + 81, 0, 16);

    if (!p->m3_sent) {
        /* M2: exact replay and flag shape, with a fresh SNonce. Derive into
         * scratch storage so a bad MIC cannot alter the handshake state. */
        expected_info = WPA2_KEY_INFO_PAIRWISE | WPA2_KEY_INFO_MIC | 2;
        if (p->keys_installed || info != expected_info ||
            get_be64(e + 9) != p->replay ||
            !memcmp(e + 17, zero_nonce, sizeof(zero_nonce)) ||
            !derive_ptk(p, e + 17, candidate_ptk)) {
            return NULL;
        }
        if (!hmac_sha1(candidate_ptk, 16, packet, packet_len, mic) ||
            memcmp(mic, e + 81, 16) ||
            !wpa2_m2_rsn_matches(p, e + 99, keydata_len)) {
            return NULL;
        }
        pending = *p;
        memcpy(pending.snonce, e + 17, sizeof(pending.snonce));
        memcpy(pending.ptk, candidate_ptk, sizeof(pending.ptk));
        if (!wpa2_m3_key_data(&pending, m3_key_data,
                              sizeof(m3_key_data), &m3_key_data_len) ||
            !aes_wrap(pending.ptk + 16, m3_key_data, m3_key_data_len,
                      wrapped)) {
            return NULL;
        }
        m3 = eapol_key_frame(&pending, WPA2_KEY_INFO_PAIRWISE |
                             WPA2_KEY_INFO_INSTALL | WPA2_KEY_INFO_ACK |
                             WPA2_KEY_INFO_MIC | WPA2_KEY_INFO_SECURE |
                             WPA2_KEY_INFO_ENCRYPTED | 2,
                             pending.anonce, wrapped,
                             m3_key_data_len + 8, true);
        if (!m3) {
            return NULL;
        }
        memcpy(p->snonce, pending.snonce, sizeof(p->snonce));
        memcpy(p->ptk, pending.ptk, sizeof(p->ptk));
        p->replay = pending.replay;
        p->m3_sent = true;
        return m3;
    }

    if (!p->keys_installed) {
        /* Waiting M4: accept only the exact response to our M3. A repeated
         * M2 is recognized separately and resends that M3 without mutation. */
        if (get_be64(e + 9) == p->replay - 1 &&
            info == (WPA2_KEY_INFO_PAIRWISE | WPA2_KEY_INFO_MIC | 2) &&
            keydata_len <= body_len - 95 &&
            !memcmp(e + 17, p->snonce, sizeof(p->snonce)) &&
            hmac_sha1(p->ptk, 16, packet, packet_len, mic) &&
            !memcmp(mic, e + 81, 16) &&
            wpa2_m2_rsn_matches(p, e + 99, keydata_len)) {
            if (!wpa2_m3_key_data(p, m3_key_data, sizeof(m3_key_data),
                                  &m3_key_data_len) ||
                !aes_wrap(p->ptk + 16, m3_key_data, m3_key_data_len,
                          wrapped)) {
                return NULL;
            }
            return eapol_key_frame(p, WPA2_KEY_INFO_PAIRWISE |
                                   WPA2_KEY_INFO_INSTALL | WPA2_KEY_INFO_ACK |
                                   WPA2_KEY_INFO_MIC | WPA2_KEY_INFO_SECURE |
                                   WPA2_KEY_INFO_ENCRYPTED | 2,
                                   p->anonce, wrapped,
                                   m3_key_data_len + 8, false);
        }
        expected_info = WPA2_KEY_INFO_PAIRWISE | WPA2_KEY_INFO_MIC |
                        WPA2_KEY_INFO_SECURE | 2;
        if (info != expected_info || keydata_len != 0 || body_len != 95 ||
            get_be64(e + 9) != p->replay ||
            !hmac_sha1(p->ptk, 16, packet, packet_len, mic) ||
            memcmp(mic, e + 81, 16)) {
            return NULL;
        }
        p->keys_installed = true;
        return NULL;
    }

    /* A duplicate valid M4 is harmless; all other frames after completion
     * are rejected without changing keys, replay state, or packet numbers. */
    expected_info = WPA2_KEY_INFO_PAIRWISE | WPA2_KEY_INFO_MIC |
                    WPA2_KEY_INFO_SECURE | 2;
    if (info == expected_info && keydata_len == 0 && body_len == 95 &&
        get_be64(e + 9) == p->replay &&
        hmac_sha1(p->ptk, 16, packet, packet_len, mic) &&
        !memcmp(mic, e + 81, 16)) {
        return NULL;
    }
    return NULL;
}

const uint8_t *esp32_wpa2_expected_data_key(const Esp32Wpa2Peer *p,
                                           const uint8_t receiver[6])
{
    /* IEEE 802.11 group-address frames use GTK; unicast data uses the CCMP
     * temporal key (the third 16-byte component of the 4-way PTK). */
    return receiver[0] & 1 ? p->gtk : p->ptk + 32;
}

bool esp32_ccmp_decrypt(mac80211_frame *f, const uint8_t key[16],
                        uint64_t *last_pn)
{
    uint8_t *raw, *body;
    size_t body_len;
    uint8_t nonce[13], b0[16] = {0}, ctr[16] = {0}, stream[16];
    uint8_t aad[32] = {0}, mac[16] = {0}, block[16], tag[16];
    size_t aad_len = 22, cipher_len;
    uint8_t *plain = NULL;
    uint64_t pn;
    Error *err = NULL;
    QCryptoCipher *aes;
    bool ok = false;

    if (!last_pn || !key || !f || f->frame_length < IEEE80211_HEADER_SIZE ||
        f->frame_length > IEEE80211_HEADER_SIZE + sizeof(f->data_and_fcs) ||
        (f->frame_control.type == IEEE80211_TYPE_DATA &&
         (f->frame_control.sub_type & 8))) {
        return false;
    }
    raw = (uint8_t *)f;
    body = f->data_and_fcs;
    body_len = f->frame_length - IEEE80211_HEADER_SIZE;
    if (body_len < 16 || !(raw[1] & 0x40)) {
        return false;
    }
    pn = ((uint64_t)body[7] << 40) | ((uint64_t)body[6] << 32) |
         ((uint64_t)body[5] << 24) | ((uint64_t)body[4] << 16) |
         ((uint64_t)body[1] << 8) | body[0];
    if (pn <= *last_pn || (body[3] & 0x3f) != 0x20) {
        return false;
    }
    cipher_len = body_len - 16;
    plain = g_malloc(cipher_len);
    nonce[0] = 0;
    memcpy(nonce + 1, raw + 10, 6);
    nonce[7] = body[7]; nonce[8] = body[6]; nonce[9] = body[5];
    nonce[10] = body[4]; nonce[11] = body[1]; nonce[12] = body[0];
    b0[0] = 0x59;
    memcpy(b0 + 1, nonce, sizeof(nonce));
    b0[14] = cipher_len >> 8;
    b0[15] = cipher_len;
    /* CCMP AAD omits Duration/ID and masks retry/power/more-data and
     * sequence-number bits. */
    aad[0] = raw[0] & 0x8f;
    aad[1] = raw[1] & 0xc7;
    memcpy(aad + 2, raw + 4, 18);
    aad[20] = raw[22] & 0x0f;
    aad[21] = 0;
    memcpy(mac, b0, sizeof(mac));
    if (!aes_block(key, mac, mac, false)) goto out;
    block[0] = 0; block[1] = aad_len;
    memcpy(block + 2, aad, 14);
    for (unsigned i = 0; i < 16; i++) mac[i] ^= block[i];
    if (!aes_block(key, mac, mac, false)) goto out;
    memset(block, 0, sizeof(block));
    memcpy(block, aad + 14, aad_len - 14);
    for (unsigned i = 0; i < 16; i++) mac[i] ^= block[i];
    if (!aes_block(key, mac, mac, false)) goto out;

    aes = qcrypto_cipher_new(QCRYPTO_CIPHER_ALGO_AES_128,
                             QCRYPTO_CIPHER_MODE_ECB, key, 16, &err);
    if (!aes) goto out;
    for (size_t off = 0; off < cipher_len; off += 16) {
        size_t n = MIN((size_t)16, cipher_len - off);
        ctr[0] = 1;
        memcpy(ctr + 1, nonce, 13);
        put_be16(ctr + 14, off / 16 + 1);
        if (qcrypto_cipher_encrypt(aes, ctr, stream, 16, &err) < 0) {
            qcrypto_cipher_free(aes);
            goto out;
        }
        for (size_t i = 0; i < n; i++) {
            plain[off + i] = body[8 + off + i] ^ stream[i];
        }
        memset(block, 0, sizeof(block));
        memcpy(block, plain + off, n);
        for (unsigned i = 0; i < 16; i++) mac[i] ^= block[i];
        if (!aes_block(key, mac, mac, false)) {
            qcrypto_cipher_free(aes);
            goto out;
        }
    }
    ctr[0] = 1; memcpy(ctr + 1, nonce, 13); memset(ctr + 14, 0, 2);
    if (qcrypto_cipher_encrypt(aes, ctr, stream, 16, &err) < 0) {
        qcrypto_cipher_free(aes);
        goto out;
    }
    qcrypto_cipher_free(aes);
    memcpy(tag, mac, 8);
    for (unsigned i = 0; i < 8; i++) tag[i] ^= stream[i];
    if (memcmp(tag, body + 8 + cipher_len, 8)) {
        goto out;
    }
    memmove(body, plain, cipher_len);
    f->frame_length -= 16;
    f->pos = f->frame_length - IEEE80211_HEADER_SIZE;
    raw[1] &= ~0x40;
    *last_pn = pn;
    ok = true;
out:
    g_free(plain);
    error_free(err);
    return ok;
}

bool esp32_wpa2_decrypt_data(Esp32Wpa2Peer *p, mac80211_frame *f,
                             const uint8_t key[16])
{
    uint64_t *last_pn;

    if (!p->keys_installed || !f) {
        return false;
    }
    last_pn = f->receiver_address[0] & 1 ? &p->group_rx_pn : &p->rx_pn;
    return esp32_ccmp_decrypt(f, key, last_pn);
}

bool esp32_ccmp_encrypt(mac80211_frame *f, const uint8_t key[16],
                        uint8_t key_id, uint64_t *next_pn)
{
    uint8_t *raw, *body;
    size_t plain_len;
    uint8_t nonce[13], b0[16] = {0}, ctr[16] = {0}, stream[16];
    uint8_t aad[32] = {0}, mac[16] = {0}, block[16], tag[16];
    uint64_t pn;
    Error *err = NULL;
    QCryptoCipher *aes;

    if (!next_pn || !key || !f || key_id > 3 ||
        f->frame_length < IEEE80211_HEADER_SIZE ||
        f->frame_length > IEEE80211_HEADER_SIZE + sizeof(f->data_and_fcs) - 16 ||
        (f->frame_control.type == IEEE80211_TYPE_DATA &&
         (f->frame_control.sub_type & 8))) {
        return false;
    }
    raw = (uint8_t *)f;
    body = f->data_and_fcs;
    plain_len = f->frame_length - IEEE80211_HEADER_SIZE;
    if (plain_len > sizeof(f->data_and_fcs) - 16) {
        return false;
    }
    if (*next_pn >= G_GUINT64_CONSTANT(0xffffffffffff)) {
        return false;
    }
    pn = ++*next_pn;
    memmove(body + 8, body, plain_len);
    body[0] = pn; body[1] = pn >> 8; body[2] = 0;
    body[3] = 0x20 | (key_id << 6);
    body[4] = pn >> 16; body[5] = pn >> 24;
    body[6] = pn >> 32; body[7] = pn >> 40;
    f->frame_length += 16;
    f->pos += 16;
    raw[1] |= 0x40;
    nonce[0] = 0;
    memcpy(nonce + 1, raw + 10, 6);
    nonce[7] = body[7]; nonce[8] = body[6]; nonce[9] = body[5];
    nonce[10] = body[4]; nonce[11] = body[1]; nonce[12] = body[0];
    b0[0] = 0x59;
    memcpy(b0 + 1, nonce, sizeof(nonce));
    b0[14] = plain_len >> 8;
    b0[15] = plain_len;
    aad[0] = raw[0] & 0x8f;
    aad[1] = raw[1] & 0xc7;
    memcpy(aad + 2, raw + 4, 18);
    aad[20] = raw[22] & 0x0f;
    aad[21] = 0;
    memcpy(mac, b0, sizeof(mac));
    if (!aes_block(key, mac, mac, false)) return false;
    block[0] = 0; block[1] = 22;
    memcpy(block + 2, aad, 14);
    for (unsigned i = 0; i < 16; i++) mac[i] ^= block[i];
    if (!aes_block(key, mac, mac, false)) return false;
    memset(block, 0, sizeof(block));
    memcpy(block, aad + 14, 8);
    for (unsigned i = 0; i < 16; i++) mac[i] ^= block[i];
    if (!aes_block(key, mac, mac, false)) return false;
    aes = qcrypto_cipher_new(QCRYPTO_CIPHER_ALGO_AES_128,
                             QCRYPTO_CIPHER_MODE_ECB, key, 16, &err);
    if (!aes) goto fail;
    for (size_t off = 0; off < plain_len; off += 16) {
        size_t n = MIN((size_t)16, plain_len - off);
        memset(ctr, 0, sizeof(ctr)); ctr[0] = 1;
        memcpy(ctr + 1, nonce, 13);
        put_be16(ctr + 14, off / 16 + 1);
        if (qcrypto_cipher_encrypt(aes, ctr, stream, 16, &err) < 0) goto fail;
        memset(block, 0, sizeof(block));
        memcpy(block, body + 8 + off, n);
        for (unsigned i = 0; i < 16; i++) mac[i] ^= block[i];
        if (!aes_block(key, mac, mac, false)) goto fail;
        for (size_t i = 0; i < n; i++) body[8 + off + i] ^= stream[i];
    }
    memset(ctr, 0, sizeof(ctr)); ctr[0] = 1; memcpy(ctr + 1, nonce, 13);
    if (qcrypto_cipher_encrypt(aes, ctr, stream, 16, &err) < 0) goto fail;
    qcrypto_cipher_free(aes);
    memcpy(tag, mac, 8);
    for (unsigned i = 0; i < 8; i++) body[8 + plain_len + i] = tag[i] ^ stream[i];
    error_free(err);
    return true;
fail:
    qcrypto_cipher_free(aes);
    error_free(err);
    return false;
}

bool esp32_wpa2_encrypt_data(Esp32Wpa2Peer *p, mac80211_frame *f,
                             const uint8_t key[16])
{
    uint64_t *next_pn;

    if (!p->keys_installed || !f) {
        return false;
    }
    next_pn = f->receiver_address[0] & 1 ? &p->group_tx_pn : &p->tx_pn;
    return esp32_ccmp_encrypt(f, key,
                              f->receiver_address[0] & 1 ? p->gtk_key_id : 0,
                              next_pn);
}
