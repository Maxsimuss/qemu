/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Small WPA2-PSK peer implementation for the original ESP32 WiFi MAC. */
#include "qemu/osdep.h"
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

static bool derive_ptk(Esp32Wpa2Peer *p)
{
    uint8_t context[76], input[101], digest[20];
    size_t off = 0;

    if (memcmp(p->bssid, p->station, 6) < 0) {
        memcpy(context, p->bssid, 6);
        memcpy(context + 6, p->station, 6);
    } else {
        memcpy(context, p->station, 6);
        memcpy(context + 6, p->bssid, 6);
    }
    if (memcmp(p->anonce, p->snonce, 32) < 0) {
        memcpy(context + 12, p->anonce, 32);
        memcpy(context + 44, p->snonce, 32);
    } else {
        memcpy(context + 12, p->snonce, 32);
        memcpy(context + 44, p->anonce, 32);
    }
    memcpy(input, "Pairwise key expansion", 23);
    input[23] = 0;
    memcpy(input + 24, context, sizeof(context));
    for (uint8_t i = 0; off < sizeof(p->ptk); i++) {
        input[100] = i;
        if (!hmac_sha1(p->pmk, sizeof(p->pmk), input, sizeof(input), digest)) {
            return false;
        }
        size_t n = MIN(sizeof(digest), sizeof(p->ptk) - off);
        memcpy(p->ptk + off, digest, n);
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
    p->configured = qcrypto_pbkdf2(QCRYPTO_HASH_ALGO_SHA1,
        (const uint8_t *)p->password, strlen(p->password),
        (const uint8_t *)p->ssid, strlen(p->ssid), 4096,
        p->pmk, sizeof(p->pmk), &err) == 0;
    error_free(err);
}

bool esp32_wpa2_assoc_request(Esp32Wpa2Peer *p, const mac80211_frame *f)
{
    const uint8_t *ie;
    size_t n;

    if (!p->configured || f->frame_length < IEEE80211_HEADER_SIZE + 4) {
        return false;
    }
    ie = f->data_and_fcs + 4;
    n = f->frame_length - IEEE80211_HEADER_SIZE - 4;
    while (n >= 2) {
        size_t l = ie[1];
        if (l + 2 > n) {
            return false;
        }
        if (ie[0] == 48) {
            static const uint8_t ccmp[] = {0,0x0f,0xac,4};
            static const uint8_t psk[] = {0,0x0f,0xac,2};
            size_t end = l + 2, off = 2;
            uint16_t count;
            bool has_ccmp = false, has_psk = false;

            /* Parse the complete variable-length RSN suite lists. */
            bool ok = l >= 18 && ie[2] == 1;
            if (ok) {
                off += 2; /* version */
                ok = off + 4 <= end && !memcmp(ie + off, ccmp, 4);
                off += 4; /* group cipher */
            }
            if (ok && off + 2 <= end) {
                count = get_le16(ie + off); off += 2;
                ok = count && count <= (end - off) / 4;
                for (unsigned i = 0; ok && i < count; i++, off += 4) {
                    has_ccmp |= !memcmp(ie + off, ccmp, 4);
                }
                ok &= has_ccmp;
            } else {
                ok = false;
            }
            if (ok && off + 2 <= end) {
                count = get_le16(ie + off); off += 2;
                ok = count && count <= (end - off) / 4;
                for (unsigned i = 0; ok && i < count; i++, off += 4) {
                    has_psk |= !memcmp(ie + off, psk, 4);
                }
                ok &= has_psk;
            } else {
                ok = false;
            }
            if (ok) {
                memcpy(p->station, f->transmitter_address, 6);
            }
            return ok;
        }
        ie += l + 2;
        n -= l + 2;
    }
    return false;
}

mac80211_frame *esp32_wpa2_assoc_response(Esp32Wpa2Peer *p)
{
    static const uint8_t rsn[] = {
        48, 20, 1,0, 0,0x0f,0xac,4, 1,0, 0,0x0f,0xac,4,
        1,0, 0,0x0f,0xac,2, 0,0
    };
    mac80211_frame *f = g_malloc0(sizeof(*f));
    f->frame_control.type = IEEE80211_TYPE_MGT;
    f->frame_control.sub_type = IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_RESP;
    f->duration_id = 314;
    f->pos = 0;
    const uint8_t fixed[] = {0x31,0x04,0,0,1,0xc0};
    memcpy(f->data_and_fcs, fixed, sizeof(fixed));
    memcpy(f->data_and_fcs + sizeof(fixed), rsn, sizeof(rsn));
    f->pos = sizeof(fixed) + sizeof(rsn);
    f->frame_length = IEEE80211_HEADER_SIZE + f->pos;
    p->associated = true;
    return f;
}

static mac80211_frame *eapol_key_frame(Esp32Wpa2Peer *p, uint16_t info,
                                      const uint8_t nonce[32],
                                      const uint8_t *keydata, size_t keydata_len)
{
    mac80211_frame *f = g_malloc0(sizeof(*f));
    uint8_t *d = f->data_and_fcs;
    size_t body_len = 95 + keydata_len;
    uint8_t mic[20];

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
    put_be64(d + 17, ++p->replay);
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
    return eapol_key_frame(p, WPA2_KEY_INFO_PAIRWISE | WPA2_KEY_INFO_ACK | 2,
                           p->anonce, NULL, 0);
}

mac80211_frame *esp32_wpa2_rx_eapol(Esp32Wpa2Peer *p,
                                    const mac80211_frame *f)
{
    const uint8_t *e = f->data_and_fcs + 8;
    size_t n = f->frame_length - IEEE80211_HEADER_SIZE - 8;
    uint16_t info;
    uint8_t mic[20], packet[512], gtk_kde[24] = {
        0xdd,22,0,0x0f,0xac,1,0,0
    }, wrapped[32];
    size_t packet_len;

    /* The EAPOL protocol version is the outer 802.1X header byte, distinct
     * from the EAPOL-Key descriptor version in key_info. ESP-IDF's WPA2
     * supplicant transmits version 1; hostapd commonly uses version 2, and
     * version 3 is the 802.1X-2010 protocol version. */
    if (!p->associated || n < 99 || n > sizeof(packet) ||
        e[0] < 1 || e[0] > 3 || e[1] != WPA2_EAPOL_KEY ||
        e[4] != WPA2_KEY_DESC_RSN || 4 + get_be16(e + 2) > n) {
        return NULL;
    }
    info = get_be16(e + 5);
    /* WPA2-PSK with CCMP uses the RSN EAPOL-Key descriptor version 2.
     * This is independent of the outer 802.1X EAPOL protocol version above. */
    if ((info & 0x0007) != 2 ||
        !(info & WPA2_KEY_INFO_PAIRWISE) || !(info & WPA2_KEY_INFO_MIC) ||
        (info & WPA2_KEY_INFO_ACK) || get_be64(e + 9) != p->replay) {
        return NULL;
    }
    if ((!p->m3_sent && (info & (WPA2_KEY_INFO_INSTALL |
                                  WPA2_KEY_INFO_SECURE |
                                  WPA2_KEY_INFO_ENCRYPTED))) ||
        (p->m3_sent && (!(info & WPA2_KEY_INFO_SECURE) ||
                        (info & (WPA2_KEY_INFO_INSTALL |
                                 WPA2_KEY_INFO_ENCRYPTED)) ||
                        get_be16(e + 97) != 0))) {
        return NULL;
    }
    if (!p->m3_sent && !memcmp(e + 17, (uint8_t[32]){0}, 32)) {
        return NULL;
    }
    packet_len = 4 + get_be16(e + 2);
    memcpy(packet, e, packet_len);
    memset(packet + 81, 0, 16);
    if (!p->m3_sent) {
        memcpy(p->snonce, e + 17, 32);
        if (!derive_ptk(p)) {
            return NULL;
        }
    }
    if (!hmac_sha1(p->ptk, 16, packet, packet_len, mic) ||
        memcmp(mic, e + 81, 16)) {
        return NULL;
    }
    if (p->m3_sent) {
        p->keys_installed = true;
        return NULL;
    }
    memcpy(gtk_kde + 8, p->gtk, 16);
    if (!aes_wrap(p->ptk + 16, gtk_kde, sizeof(gtk_kde), wrapped)) {
        return NULL;
    }
    p->m3_sent = true;
    return eapol_key_frame(p, WPA2_KEY_INFO_PAIRWISE | WPA2_KEY_INFO_INSTALL |
                           WPA2_KEY_INFO_ACK | WPA2_KEY_INFO_MIC |
                           WPA2_KEY_INFO_SECURE | WPA2_KEY_INFO_ENCRYPTED | 2,
                           p->anonce, wrapped, sizeof(wrapped));
}

bool esp32_wpa2_decrypt_data(Esp32Wpa2Peer *p, mac80211_frame *f)
{
    uint8_t *raw = (uint8_t *)f;
    uint8_t *body = f->data_and_fcs;
    size_t body_len = f->frame_length - IEEE80211_HEADER_SIZE;
    uint8_t nonce[13], b0[16] = {0}, ctr[16] = {0}, stream[16];
    uint8_t aad[32] = {0}, mac[16] = {0}, block[16], tag[16];
    size_t aad_len = 22, cipher_len;
    uint8_t *plain = NULL;
    uint64_t pn;
    Error *err = NULL;
    QCryptoCipher *aes;
    bool ok = false;

    if (!p->keys_installed || body_len < 16 || !(raw[1] & 0x40)) {
        return false;
    }
    pn = ((uint64_t)body[7] << 40) | ((uint64_t)body[6] << 32) |
         ((uint64_t)body[5] << 24) | ((uint64_t)body[4] << 16) |
         ((uint64_t)body[1] << 8) | body[0];
    if (pn <= p->rx_pn || !(body[3] & 0x20) || (body[3] >> 6) != 0) {
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
    if (!aes_block(p->ptk, mac, mac, false)) goto out;
    block[0] = 0; block[1] = aad_len;
    memcpy(block + 2, aad, 14);
    for (unsigned i = 0; i < 16; i++) mac[i] ^= block[i];
    if (!aes_block(p->ptk, mac, mac, false)) goto out;
    memset(block, 0, sizeof(block));
    memcpy(block, aad + 14, aad_len - 14);
    for (unsigned i = 0; i < 16; i++) mac[i] ^= block[i];
    if (!aes_block(p->ptk, mac, mac, false)) goto out;

    aes = qcrypto_cipher_new(QCRYPTO_CIPHER_ALGO_AES_128,
                             QCRYPTO_CIPHER_MODE_ECB, p->ptk, 16, &err);
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
        if (!aes_block(p->ptk, mac, mac, false)) {
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
    if (memcmp(tag, body + 8 + cipher_len, 8)) goto out;
    memmove(body, plain, cipher_len);
    f->frame_length -= 16;
    f->pos = f->frame_length - IEEE80211_HEADER_SIZE;
    raw[1] &= ~0x40;
    p->rx_pn = pn;
    ok = true;
out:
    g_free(plain);
    error_free(err);
    return ok;
}

bool esp32_wpa2_encrypt_data(Esp32Wpa2Peer *p, mac80211_frame *f)
{
    uint8_t *raw = (uint8_t *)f;
    uint8_t *body = f->data_and_fcs;
    size_t plain_len = f->frame_length - IEEE80211_HEADER_SIZE;
    uint8_t nonce[13], b0[16] = {0}, ctr[16] = {0}, stream[16];
    uint8_t aad[32] = {0}, mac[16] = {0}, block[16], tag[16];
    uint64_t pn;
    Error *err = NULL;
    QCryptoCipher *aes;

    if (!p->keys_installed || plain_len > sizeof(f->data_and_fcs) - 16) {
        return false;
    }
    pn = ++p->tx_pn;
    memmove(body + 8, body, plain_len);
    body[0] = pn; body[1] = pn >> 8; body[2] = 0; body[3] = 0x20;
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
    if (!aes_block(p->ptk, mac, mac, false)) return false;
    block[0] = 0; block[1] = 22;
    memcpy(block + 2, aad, 14);
    for (unsigned i = 0; i < 16; i++) mac[i] ^= block[i];
    if (!aes_block(p->ptk, mac, mac, false)) return false;
    memset(block, 0, sizeof(block));
    memcpy(block, aad + 14, 8);
    for (unsigned i = 0; i < 16; i++) mac[i] ^= block[i];
    if (!aes_block(p->ptk, mac, mac, false)) return false;
    aes = qcrypto_cipher_new(QCRYPTO_CIPHER_ALGO_AES_128,
                             QCRYPTO_CIPHER_MODE_ECB, p->ptk, 16, &err);
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
        if (!aes_block(p->ptk, mac, mac, false)) goto fail;
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
