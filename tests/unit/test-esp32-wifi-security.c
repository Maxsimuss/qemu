/* SPDX-License-Identifier: GPL-2.0-or-later */
/* WPA2 peer key derivation and CCMP data protection tests. */
#include "qemu/osdep.h"
#include "crypto/init.h"
#include "crypto/hmac.h"
#include "crypto/pbkdf.h"
#include "hw/xtensa/esp32_wifi_security.h"

static const uint8_t test_rsn_ie[] = {
    48,20,1,0, 0,0x0f,0xac,4, 1,0, 0,0x0f,0xac,4,
    1,0, 0,0x0f,0xac,2, 0,0
};

static void derive_test_ptk(const uint8_t pmk[32], const uint8_t ap[6],
                            const uint8_t sta[6], const uint8_t anonce[32],
                            const uint8_t snonce[32], uint8_t ptk[64])
{
    uint8_t input[101], context[76], digest[20];
    size_t copied = 0;
    Error *err = NULL;
    QCryptoHmac *h;

    if (memcmp(ap, sta, 6) < 0) {
        memcpy(context, ap, 6); memcpy(context + 6, sta, 6);
    } else {
        memcpy(context, sta, 6); memcpy(context + 6, ap, 6);
    }
    if (memcmp(anonce, snonce, 32) < 0) {
        memcpy(context + 12, anonce, 32); memcpy(context + 44, snonce, 32);
    } else {
        memcpy(context + 12, snonce, 32); memcpy(context + 44, anonce, 32);
    }
    memcpy(input, "Pairwise key expansion", 23);
    input[23] = 0; memcpy(input + 24, context, sizeof(context));
    h = qcrypto_hmac_new(QCRYPTO_HASH_ALGO_SHA1, pmk, 32, &err);
    g_assert_null(err);
    for (uint8_t i = 0; copied < 64; i++) {
        uint8_t *out = digest;
        size_t nout = sizeof(digest);
        input[100] = i;
        g_assert_cmpint(qcrypto_hmac_bytes(h, (char *)input, sizeof(input),
                                            &out, &nout, &err), ==, 0);
        g_assert_null(err);
        size_t n = MIN(sizeof(digest), 64 - copied);
        memcpy(ptk + copied, digest, n);
        copied += n;
    }
    qcrypto_hmac_free(h);
}

static void add_test_eapol_mic(uint8_t *eapol, size_t len,
                               const uint8_t kck[16])
{
    uint8_t digest[20], *out = digest;
    size_t outlen = sizeof(digest);
    Error *err = NULL;
    QCryptoHmac *h = qcrypto_hmac_new(QCRYPTO_HASH_ALGO_SHA1, kck, 16, &err);

    g_assert_null(err);
    memset(eapol + 81, 0, 16);
    g_assert_cmpint(qcrypto_hmac_bytes(h, (char *)eapol, len,
                                        &out, &outlen, &err), ==, 0);
    g_assert_null(err);
    memcpy(eapol + 81, digest, 16);
    qcrypto_hmac_free(h);
}

static void set_test_eapol_version(mac80211_frame *f, uint8_t version,
                                   const uint8_t kck[16])
{
    uint8_t *e = f->data_and_fcs + 8;
    size_t eapol_len = 4 + ((size_t)e[2] << 8) + e[3];

    e[0] = version;
    add_test_eapol_mic(e, eapol_len, kck);
}

static mac80211_frame *make_station_key(const uint8_t sta[6],
                                        uint16_t key_info, uint64_t replay,
                                        const uint8_t nonce[32],
                                        const uint8_t *keydata,
                                        size_t keydata_len,
                                        const uint8_t kck[16])
{
    mac80211_frame *f = g_malloc0(sizeof(*f));
    uint8_t *d = f->data_and_fcs;
    uint8_t *e = d + 8;
    size_t eapol_len = 4 + 95 + keydata_len;

    f->frame_control.type = IEEE80211_TYPE_DATA;
    f->frame_control.to_ds = 1;
    memcpy(f->transmitter_address, sta, 6);
    memcpy(d, (uint8_t[]){0xaa,0xaa,3,0,0,0,0x88,0x8e}, 8);
    /* IDF's stock RSN supplicant uses EAPOL protocol version 1. */
    e[0] = 1; e[1] = 3; e[2] = (eapol_len - 4) >> 8;
    e[3] = eapol_len - 4;
    e[4] = 2; e[5] = key_info >> 8; e[6] = key_info;
    e[7] = 0; e[8] = 16;
    for (int i = 7; i >= 0; i--) { e[9 + i] = replay; replay >>= 8; }
    if (nonce) memcpy(e + 17, nonce, 32);
    e[97] = keydata_len >> 8; e[98] = keydata_len;
    if (keydata_len) memcpy(e + 99, keydata, keydata_len);
    if (key_info & 0x0100) add_test_eapol_mic(e, eapol_len, kck);
    f->frame_length = IEEE80211_HEADER_SIZE + 8 + eapol_len;
    f->pos = f->frame_length - IEEE80211_HEADER_SIZE;
    return f;
}

static void test_pmk_wpa_vector(void)
{
    static const uint8_t expected[32] = {
        0xf4,0x2c,0x6f,0xc5,0x2d,0xf0,0xeb,0xef,
        0x9e,0xbb,0x4b,0x90,0xb3,0x8a,0x5f,0x90,
        0x2e,0x83,0xfe,0x1b,0x13,0x5a,0x70,0xe2,
        0x3a,0xed,0x76,0x2e,0x97,0x10,0xa1,0x2e,
    };
    Esp32Wpa2Peer p;
    const uint8_t bssid[6] = {0,1,2,3,4,5};

    esp32_wpa2_configure(&p, "IEEE", "password", bssid);
    g_assert_true(p.configured);
    g_assert_cmpmem(p.pmk, sizeof(p.pmk), expected, sizeof(expected));
}

static void test_ccmp_roundtrip_and_mic_reject(void)
{
    Esp32Wpa2Peer tx = {.keys_installed = true};
    Esp32Wpa2Peer rx = {.keys_installed = true};
    mac80211_frame *f = g_malloc0(sizeof(*f));
    const uint8_t payload[] = {0xaa,0xaa,3,0,0,0,8,0,1,2,3,4,5,6};
    size_t length;

    for (unsigned i = 0; i < sizeof(tx.ptk); i++) {
        tx.ptk[i] = rx.ptk[i] = i;
    }
    f->frame_control.type = IEEE80211_TYPE_DATA;
    f->frame_control.from_ds = 1;
    memcpy(f->receiver_address, (uint8_t[]){0,1,2,3,4,5}, 6);
    memcpy(f->transmitter_address, (uint8_t[]){6,7,8,9,10,11}, 6);
    memcpy(f->address_3, (uint8_t[]){12,13,14,15,16,17}, 6);
    memcpy(f->data_and_fcs, payload, sizeof(payload));
    f->frame_length = IEEE80211_HEADER_SIZE + sizeof(payload);
    length = f->frame_length;

    g_assert_true(esp32_wpa2_encrypt_data(&tx, f));
    g_assert_cmpuint(f->frame_length, ==, length + 16);
    g_assert_true(((uint8_t *)f)[1] & 0x40);
    g_assert_true(esp32_wpa2_decrypt_data(&rx, f));
    g_assert_cmpuint(f->frame_length, ==, length);
    g_assert_cmpmem(f->data_and_fcs, sizeof(payload), payload, sizeof(payload));
    g_assert_false(((uint8_t *)f)[1] & 0x40);

    g_assert_true(esp32_wpa2_encrypt_data(&tx, f));
    f->data_and_fcs[f->frame_length - IEEE80211_HEADER_SIZE - 1] ^= 0x80;
    g_assert_false(esp32_wpa2_decrypt_data(&rx, f));
    g_free(f);
}

static void test_ccmp_ieee_reference_vector(void)
{
    /* P802.11i/D7.0 CCMP MPDU test vector 1, also used by FreeBSD's
     * net80211 CCMP regression test.  This catches matching encrypt/decrypt
     * mistakes in nonce byte order, AAD masking, and MIC construction. */
    static const uint8_t key[16] = {
        0xc9,0x7c,0x1f,0x67,0xce,0x37,0x11,0x85,
        0x51,0x4a,0x8a,0x19,0xf2,0xbd,0xd5,0x2f
    };
    static const uint8_t hdr[24] = {
        0x08,0x48,0xc3,0x2c,0x0f,0xd2,0xe1,0x28,
        0xa5,0x7c,0x50,0x30,0xf1,0x84,0x44,0x08,
        0xab,0xae,0xa5,0xb8,0xfc,0xba,0x80,0x33
    };
    static const uint8_t ccmp[36] = {
        0x0c,0xe7,0x00,0x20,0x76,0x97,0x03,0xb5,
        0xf3,0xd0,0xa2,0xfe,0x9a,0x3d,0xbf,0x23,
        0x42,0xa6,0x43,0xe4,0x32,0x46,0xe8,0x0c,
        0x3c,0x04,0xd0,0x19,0x78,0x45,0xce,0x0b,
        0x16,0xf9,0x76,0x23
    };
    static const uint8_t plain[20] = {
        0xf8,0xba,0x1a,0x55,0xd0,0x2f,0x85,0xae,0x96,0x7b,
        0xb6,0x2f,0xb6,0xcd,0xa8,0xeb,0x7e,0x78,0xa0,0x50
    };
    const uint64_t pn = G_GUINT64_CONSTANT(0xb5039776e70c);
    Esp32Wpa2Peer rx = {.keys_installed = true, .rx_pn = pn - 1};
    Esp32Wpa2Peer tx = {.keys_installed = true, .tx_pn = pn - 1};
    mac80211_frame *f = g_malloc0(sizeof(*f));

    memcpy(rx.ptk, key, sizeof(key));
    memcpy(tx.ptk, key, sizeof(key));
    memcpy(f, hdr, sizeof(hdr));
    memcpy(f->data_and_fcs, ccmp, sizeof(ccmp));
    f->frame_length = IEEE80211_HEADER_SIZE + sizeof(ccmp);
    g_assert_true(esp32_wpa2_decrypt_data(&rx, f));
    g_assert_cmpmem(f->data_and_fcs, sizeof(plain), plain, sizeof(plain));

    /* The already accepted packet number is rejected even when its
     * ciphertext and authentication tag are otherwise valid. */
    memcpy(f, hdr, sizeof(hdr));
    memcpy(f->data_and_fcs, ccmp, sizeof(ccmp));
    f->frame_length = IEEE80211_HEADER_SIZE + sizeof(ccmp);
    g_assert_false(esp32_wpa2_decrypt_data(&rx, f));

    memcpy(f, hdr, sizeof(hdr));
    memcpy(f->data_and_fcs, plain, sizeof(plain));
    f->frame_length = IEEE80211_HEADER_SIZE + sizeof(plain);
    g_assert_true(esp32_wpa2_encrypt_data(&tx, f));
    g_assert_cmpmem(f->data_and_fcs, sizeof(ccmp), ccmp, sizeof(ccmp));
    g_free(f);
}

static void test_wpa2_four_way_handshake(void)
{
    const uint8_t bssid[6] = {0x10,1,0,0xc4,0x0a,0x51};
    const uint8_t sta[6] = {0x24,0x6f,0x28,0x11,0x22,0x33};
    Esp32Wpa2Peer ap;
    mac80211_frame *assoc = g_malloc0(sizeof(*assoc));
    mac80211_frame *response, *m1, *m2, *m3, *m4;
    uint8_t snonce[32], ptk[64];
    uint8_t wrong_pmk[32], wrong_ptk[64];
    uint8_t *e;
    size_t ielen = sizeof(test_rsn_ie);
    unsigned i;
    Error *err = NULL;
    mac80211_frame *wrong_m2, *bad_m2, *bad_m4, *bad_version, *bad_desc;

    esp32_wpa2_configure(&ap, "test-ap", "test-passphrase", bssid);
    g_assert_true(ap.configured);
    assoc->frame_control.type = IEEE80211_TYPE_MGT;
    assoc->frame_control.sub_type = IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_REQ;
    memcpy(assoc->transmitter_address, sta, 6);
    memcpy(assoc->data_and_fcs, (uint8_t[]){0x31,4,3,0}, 4);
    memcpy(assoc->data_and_fcs + 4, test_rsn_ie, ielen);
    assoc->frame_length = IEEE80211_HEADER_SIZE + 4 + ielen;
    g_assert_true(esp32_wpa2_assoc_request(&ap, assoc));
    response = esp32_wpa2_assoc_response(&ap);
    g_assert_nonnull(response);
    g_assert_true(ap.associated);
    g_free(response);

    m1 = esp32_wpa2_start(&ap);
    g_assert_nonnull(m1);
    e = m1->data_and_fcs + 8;
    g_assert_cmpuint(e[1], ==, 3);
    g_assert_cmphex((e[5] << 8) | e[6], ==, 0x008a);
    g_assert_cmpuint(e[97], ==, 0);
    for (i = 0; i < sizeof(snonce); i++) snonce[i] = i + 1;
    derive_test_ptk(ap.pmk, bssid, sta, e + 17, snonce, ptk);
    g_assert_cmpint(qcrypto_pbkdf2(QCRYPTO_HASH_ALGO_SHA1,
        (const uint8_t *)"bad-passphrase", strlen("bad-passphrase"),
        (const uint8_t *)"test-ap", strlen("test-ap"), 4096,
        wrong_pmk, sizeof(wrong_pmk), &err), ==, 0);
    g_assert_null(err);
    derive_test_ptk(wrong_pmk, bssid, sta, e + 17, snonce, wrong_ptk);
    wrong_m2 = make_station_key(sta, 0x010a, 1, snonce, test_rsn_ie,
                                sizeof(test_rsn_ie), wrong_ptk);
    g_assert_null(esp32_wpa2_rx_eapol(&ap, wrong_m2));
    g_assert_false(ap.m3_sent);
    g_free(wrong_m2);

    bad_version = make_station_key(sta, 0x010a, 1, snonce, test_rsn_ie,
                                   sizeof(test_rsn_ie), ptk);
    set_test_eapol_version(bad_version, 0, ptk);
    g_assert_null(esp32_wpa2_rx_eapol(&ap, bad_version));
    set_test_eapol_version(bad_version, 4, ptk);
    g_assert_null(esp32_wpa2_rx_eapol(&ap, bad_version));
    g_assert_false(ap.m3_sent);
    g_free(bad_version);

    /* WPA2/CCMP requires descriptor version 2 even when the outer EAPOL
     * protocol version is 1. Keep the MIC valid so this tests negotiation. */
    bad_desc = make_station_key(sta, 0x0109, 1, snonce, test_rsn_ie,
                                sizeof(test_rsn_ie), ptk);
    g_assert_null(esp32_wpa2_rx_eapol(&ap, bad_desc));
    g_assert_false(ap.m3_sent);
    g_free(bad_desc);

    m2 = make_station_key(sta, 0x010a, 1, snonce, test_rsn_ie,
                          sizeof(test_rsn_ie), ptk);
    bad_m2 = make_station_key(sta, 0x010a, 1, snonce, test_rsn_ie,
                              sizeof(test_rsn_ie), ptk);
    bad_m2->data_and_fcs[8 + 4 + 81] ^= 1;
    g_assert_null(esp32_wpa2_rx_eapol(&ap, bad_m2));
    g_assert_false(ap.m3_sent);
    g_free(bad_m2);
    m3 = esp32_wpa2_rx_eapol(&ap, m2);
    g_assert_nonnull(m3);
    e = m3->data_and_fcs + 8;
    g_assert_cmphex((e[5] << 8) | e[6], ==, 0x13ca);
    g_assert_cmpuint((e[97] << 8) | e[98], ==, 32);
    g_assert_false(ap.keys_installed);

    bad_m4 = make_station_key(sta, 0x010a, 2, NULL, NULL, 0, ptk);
    g_assert_null(esp32_wpa2_rx_eapol(&ap, bad_m4));
    g_assert_false(ap.keys_installed);
    g_free(bad_m4);
    m4 = make_station_key(sta, 0x030a, 2, NULL, NULL, 0, ptk);
    set_test_eapol_version(m4, 2, ptk); /* hostapd-style header */
    g_assert_null(esp32_wpa2_rx_eapol(&ap, m4));
    g_assert_true(ap.keys_installed);
    g_free(assoc); g_free(m1); g_free(m2); g_free(m3); g_free(m4);
}

static void test_rsn_suite_selection(void)
{
    static const uint8_t rsn_multi[] = {
        48,28, 1,0, 0,0x0f,0xac,4, 2,0,
        0,0x0f,0xac,2, 0,0x0f,0xac,4,
        2,0, 0,0x0f,0xac,1, 0,0x0f,0xac,2, 0,0
    };
    const uint8_t bssid[6] = {0,1,2,3,4,5};
    Esp32Wpa2Peer ap;
    mac80211_frame *assoc = g_malloc0(sizeof(*assoc));

    esp32_wpa2_configure(&ap, "test-ap", "test-passphrase", bssid);
    g_assert_true(ap.configured);
    assoc->frame_control.type = IEEE80211_TYPE_MGT;
    assoc->frame_control.sub_type = IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_REQ;
    memcpy(assoc->transmitter_address, (uint8_t[]){2,3,4,5,6,7}, 6);
    memcpy(assoc->data_and_fcs, (uint8_t[]){0x31,4,3,0}, 4);
    memcpy(assoc->data_and_fcs + 4, rsn_multi, sizeof(rsn_multi));
    assoc->frame_length = IEEE80211_HEADER_SIZE + 4 + sizeof(rsn_multi);
    g_assert_true(esp32_wpa2_assoc_request(&ap, assoc));

    /* A group TKIP offer cannot be silently accepted as the configured
     * CCMP-only peer. */
    assoc->data_and_fcs[4 + 4 + 3] = 2;
    g_assert_false(esp32_wpa2_assoc_request(&ap, assoc));
    g_free(assoc);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_assert_cmpint(qcrypto_init(NULL), ==, 0);
    g_test_add_func("/esp32/wifi/security/pmk-vector", test_pmk_wpa_vector);
    g_test_add_func("/esp32/wifi/security/ccmp-roundtrip",
                    test_ccmp_roundtrip_and_mic_reject);
    g_test_add_func("/esp32/wifi/security/ccmp-ieee-vector",
                    test_ccmp_ieee_reference_vector);
    g_test_add_func("/esp32/wifi/security/four-way-handshake",
                    test_wpa2_four_way_handshake);
    g_test_add_func("/esp32/wifi/security/rsn-suite-selection",
                    test_rsn_suite_selection);
    return g_test_run();
}
