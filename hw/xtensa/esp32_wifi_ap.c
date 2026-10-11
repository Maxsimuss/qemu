/* SPDX-License-Identifier: MIT */
/**
 * QEMU WLAN access point emulation
 *
 * Copyright (c) 2008 Clemens Kolbitsch
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Modifications:
 *  2008-February-24  Clemens Kolbitsch :
 *                                  New implementation based on ne2000.c
 *  18/1/22 Martin Johnson : Modified for esp32 wifi emulation
 */

#include "qemu/osdep.h"
#include "net/net.h"
#include "net/eth.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "crypto/random.h"
#include "hw/misc/trace.h"
#include <zlib.h>

#include "hw/xtensa/esp32_wifi.h"
#include "esp32_wlan.h"
#include "esp32_wlan_packet.h"

// 50ms between beacons
#define BEACON_TIME 50000000
#define INTER_FRAME_TIME 5000000
#define WPA_RETRY_INITIAL_NS 250000000
#define WPA_RETRY_MAX_NS 4000000000ULL
#define WPA_RETRY_LIMIT 5
#define DEBUG 0
#define DEBUG_DUMPFRAMES 0

/* One explicit external-AP fixture, configured with -device misc.esp32_wifi,
 *peer-ssid=...,peer-channel=....  It is a network peer, not a claimed model
 *of an environment discovered on the real radio. */
static access_point_info peer_ap = {
    .sigstrength = -40,
    .mac_address = { 0x10, 0x01, 0x00, 0xc4, 0x0a, 0x51 },
};

static bool is_broadcast_mac(const uint8_t mac[6])
{
    static const uint8_t broadcast[6] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    };

    return !memcmp(mac, broadcast, sizeof(broadcast));
}

/* Peer-generated frames carry an FCS on the wire. The CCMP implementation
 * authenticates the MAC header and MSDU, so remove the validated FCS before
 * decrypting and retain the original FCS bytes for RX metadata. */
static bool wlan_strip_valid_fcs(mac80211_frame *frame, uint8_t fcs[4])
{
    uint8_t *raw = (uint8_t *)frame;
    uint16_t length = frame->frame_length;
    uint32_t actual, expected;

    if (length < IEEE80211_HEADER_SIZE + 4) {
        return false;
    }
    actual = ldl_le_p(raw + length - 4);
    expected = crc32(0, raw, length - 4);
    if (actual != expected) {
        return false;
    }
    memcpy(fcs, raw + length - 4, 4);
    frame->frame_length = length - 4;
    frame->pos = frame->frame_length - IEEE80211_HEADER_SIZE;
    return true;
}

/* A probe request is necessarily addressed to the broadcast BSSID.  Once a
 * station has selected this peer, later management requests must name the
 * configured BSSID in both RA and BSSID fields. */
static bool frame_targets_peer_ap(const Esp32WifiState *s,
                                  const mac80211_frame *frame)
{
    if (!peer_ap.ssid || peer_ap.channel != s->rf_channel ||
        frame->frame_control.type != IEEE80211_TYPE_MGT) {
        return false;
    }

    if (frame->frame_control.sub_type == IEEE80211_TYPE_MGT_SUBTYPE_PROBE_REQ) {
        return is_broadcast_mac(frame->receiver_address) &&
               is_broadcast_mac(frame->address_3);
    }

    return !memcmp(frame->receiver_address, peer_ap.mac_address, 6) &&
           !memcmp(frame->address_3, peer_ap.mac_address, 6);
}

/* Identify guest SoftAP data by the BSSID-bearing address for each DS
 * direction.  The guest may emit AP-to-station frames with From-DS set while
 * it handles HTTP replies, so the transmitter is the BSSID in that case. */
static bool frame_uses_guest_ap_bssid(const Esp32WifiState *s,
                                      const mac80211_frame *frame)
{
    if (frame->frame_control.to_ds && !frame->frame_control.from_ds) {
        return !memcmp(frame->receiver_address, s->guest_ap_macaddr, 6);
    }
    if (!frame->frame_control.to_ds && frame->frame_control.from_ds) {
        return !memcmp(frame->transmitter_address, s->guest_ap_macaddr, 6);
    }
    return false;
}

static void guest_softap_client_disconnect(Esp32WifiState *s)
{
    s->guest_ap_associated = false;
    s->guest_dhcp_complete = false;
    s->guest_dhcp_xid_active = false;
    memset(s->guest_dhcp_xid, 0, sizeof(s->guest_dhcp_xid));
    memset(s->guest_dhcp_server_id, 0, sizeof(s->guest_dhcp_server_id));
    memset(s->guest_dhcp_offered_ip, 0, sizeof(s->guest_dhcp_offered_ip));
    memset(s->ipaddr, 0, sizeof(s->ipaddr));
    memset(s->ap_ipaddr, 0, sizeof(s->ap_ipaddr));
}

static void Esp32_WLAN_beacon_timer(void *opaque)
{
    struct mac80211_frame *frame;
    Esp32WifiState *s = (Esp32WifiState *)opaque;

    if (!esp32_wifi_mac_enabled(s)) {
        return;
    }

    // only send a beacon if we are an access point
    if(s->ap_state!=Esp32_WLAN__STATE_STA_ASSOCIATED) {
        if (peer_ap.channel == s->rf_channel && peer_ap.ssid) {
            frame = Esp32_WLAN_create_beacon_frame(&peer_ap);
            memcpy(s->ap_macaddr, peer_ap.mac_address, 6);
            memcpy(frame->receiver_address, BROADCAST, 6);
            memcpy(frame->transmitter_address, s->ap_macaddr, 6);
            memcpy(frame->address_3, s->ap_macaddr, 6);
            Esp32_WLAN_init_ap_frame(s, frame);
            Esp32_WLAN_insert_frame(s, frame);
        }
        s->beacon_ap = 0;
    }
    timer_mod(s->beacon_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + BEACON_TIME);
}

static void Esp32_WLAN_inject_timer(void *opaque)
{
    Esp32WifiState *s = (Esp32WifiState *)opaque;
    struct mac80211_frame *frame;

    if (!esp32_wifi_mac_enabled(s)) {
        s->inject_timer_running = 0;
        return;
    }

    frame = s->inject_queue;
    if (frame) {
        // remove from queue
        s->inject_queue_size--;
        s->inject_queue = frame->next_frame;
        /* The virtual AP emits on-air CCMP frames using its own keys. Validate
         * and remove the wire FCS before CCMP authenticates the frame. RX
         * metadata includes FCS, so retain the original transmitted FCS after
         * decrypting the security header and payload. */
        if (frame->frame_control._flags & 0x10) {
            uint8_t fcs[4];

            trace_esp32_wifi_guest_frame(0, frame->frame_control.type,
                                         frame->frame_control.sub_type,
                                         s->ap_state, s->rf_channel,
                                         frame->frame_length);
            if (!wlan_strip_valid_fcs(frame, fcs)) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "wifi peer RX rejected invalid FCS len=%u\n",
                              frame->frame_length);
                g_free(frame);
                frame = NULL;
            } else if (!esp32_wifi_mac_ccmp(s, frame, false, 0)) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "wifi peer RX rejected CCMP len=%u RA=%02x:%02x:%02x:%02x:%02x:%02x TA=%02x:%02x:%02x:%02x:%02x:%02x PN=%02x:%02x:%02x:%02x:%02x:%02x keyid=%u\n",
                              frame->frame_length,
                              frame->receiver_address[0], frame->receiver_address[1],
                              frame->receiver_address[2], frame->receiver_address[3],
                              frame->receiver_address[4], frame->receiver_address[5],
                              frame->transmitter_address[0], frame->transmitter_address[1],
                              frame->transmitter_address[2], frame->transmitter_address[3],
                              frame->transmitter_address[4], frame->transmitter_address[5],
                              frame->data_and_fcs[0], frame->data_and_fcs[1],
                              frame->data_and_fcs[4], frame->data_and_fcs[5],
                              frame->data_and_fcs[6], frame->data_and_fcs[7],
                              (frame->data_and_fcs[3] >> 6) & 3);
                g_free(frame);
                frame = NULL;
            } else {
                memcpy((uint8_t *)frame + frame->frame_length, fcs,
                       sizeof(fcs));
                frame->frame_length += sizeof(fcs);
            }
        } else {
            trace_esp32_wifi_guest_frame(0, frame->frame_control.type,
                                         frame->frame_control.sub_type,
                                         s->ap_state, s->rf_channel,
                                         frame->frame_length);
        }
        if (frame) {
            Esp32_sendFrame(s, frame, frame->frame_length,
                            frame->signal_strength);
            g_free(frame);
        }
    }
    if (s->inject_queue_size > 0) {
        // there are more packets... schedule
        // the timer for sending them as well
        timer_mod(s->inject_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + INTER_FRAME_TIME);
    } else {
        // we wait until a new packet schedules
        // us again
        s->inject_timer_running = 0;
    }

}

static void macprint(uint8_t *p, const char * name) {
    printf("%s: %02x:%02x:%02x:%02x:%02x:%02x\n",name, p[0],p[1],p[2],p[3],p[4],p[5]);
}

/* The NIC attached to this MAC is QEMU's external virtual station.  With the
 * user-mode backend it has the conventional SLIRP addresses below; frames on
 * the guest SoftAP side use the address actually offered by the guest's DHCP
 * server.  Host-forwarded traffic is translated between those two L3 links,
 * while every packet still traverses the guest's 802.11 DMA path.
 */
static const uint8_t slirp_gateway_ip[4] = { 10, 0, 2, 2 };
static const uint8_t slirp_station_ip[4] = { 10, 0, 2, 15 };

static uint16_t get_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static uint64_t mac_to_u64(const uint8_t mac[6])
{
    return ((uint64_t)mac[0] << 40) | ((uint64_t)mac[1] << 32) |
           ((uint64_t)mac[2] << 24) | ((uint64_t)mac[3] << 16) |
           ((uint64_t)mac[4] << 8) | mac[5];
}

static void wpa_retry_clear(Esp32WifiState *s)
{
    if (s->wpa_retry_timer) {
        timer_del(s->wpa_retry_timer);
    }
    g_clear_pointer(&s->wpa_retry_frame, g_free);
    s->wpa_retry_count = 0;
    s->wpa_retry_interval_ns = 0;
}

static void wpa_remove_queued_key_frames(Esp32WifiState *s)
{
    mac80211_frame *previous = NULL;
    mac80211_frame *frame = s->inject_queue;

    while (frame) {
        mac80211_frame *next = frame->next_frame;
        bool eapol = frame->frame_control.type == IEEE80211_TYPE_DATA &&
            frame->frame_length >= IEEE80211_HEADER_SIZE + 8 &&
            !memcmp(frame->data_and_fcs,
                    (const uint8_t[]){0xaa,0xaa,3,0,0,0,0x88,0x8e}, 8);

        if (eapol) {
            if (previous) {
                previous->next_frame = next;
            } else {
                s->inject_queue = next;
            }
            g_free(frame);
            s->inject_queue_size--;
        } else {
            previous = frame;
        }
        frame = next;
    }
    if (!s->inject_queue && s->inject_timer) {
        timer_del(s->inject_timer);
        s->inject_timer_running = 0;
    }
}

static void wpa_clear_association(Esp32WifiState *s)
{
    wpa_retry_clear(s);
    wpa_remove_queued_key_frames(s);
    s->security.associated = false;
    s->security.m3_sent = false;
    s->security.keys_installed = false;
    s->security.replay = 0;
    s->security.tx_pn = 0;
    s->security.rx_pn = 0;
    s->security.group_tx_pn = 0;
    s->security.group_rx_pn = 0;
    memset(s->security.station, 0, sizeof(s->security.station));
    memset(s->associated_ap_macaddr, 0, sizeof(s->associated_ap_macaddr));
    memset(s->security.anonce, 0, sizeof(s->security.anonce));
    memset(s->security.snonce, 0, sizeof(s->security.snonce));
    memset(s->security.ptk, 0, sizeof(s->security.ptk));
    memset(s->security.gtk, 0, sizeof(s->security.gtk));
}

static void wpa_retry_send_cached(Esp32WifiState *s)
{
    mac80211_frame *frame;

    if (!s->wpa_retry_frame) {
        return;
    }
    frame = g_malloc(sizeof(*frame));
    memcpy(frame, s->wpa_retry_frame, sizeof(*frame));
    frame->next_frame = NULL;
    Esp32_WLAN_init_ap_frame(s, frame);
    Esp32_WLAN_insert_frame(s, frame);
}

static void wpa_retry_timer(void *opaque)
{
    Esp32WifiState *s = opaque;

    if (!s->security.associated || s->security.keys_installed ||
        !s->wpa_retry_frame) {
        wpa_retry_clear(s);
        return;
    }
    if (s->wpa_retry_count >= WPA_RETRY_LIMIT) {
        mac80211_frame *deauth = Esp32_WLAN_create_deauthentication_reason(15);

        memcpy(deauth->receiver_address, s->security.station, 6);
        memcpy(deauth->transmitter_address, s->associated_ap_macaddr, 6);
        memcpy(deauth->address_3, s->associated_ap_macaddr, 6);
        Esp32_WLAN_init_ap_frame(s, deauth);
        Esp32_WLAN_insert_frame(s, deauth);
        wpa_clear_association(s);
        s->ap_state = Esp32_WLAN__STATE_STA_AUTHENTICATED;
        return;
    }

    wpa_retry_send_cached(s);
    s->wpa_retry_count++;
    s->wpa_retry_interval_ns = MIN(s->wpa_retry_interval_ns * 2,
                                   WPA_RETRY_MAX_NS);
    timer_mod(s->wpa_retry_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              s->wpa_retry_interval_ns);
}

static void wpa_queue_key_frame(Esp32WifiState *s, mac80211_frame *frame)
{
    if (!frame) {
        return;
    }
    memcpy(frame->receiver_address, s->security.station, 6);
    memcpy(frame->transmitter_address, s->associated_ap_macaddr, 6);
    memcpy(frame->address_3, s->associated_ap_macaddr, 6);
    Esp32_WLAN_init_ap_frame(s, frame);
    wpa_retry_clear(s);
    s->wpa_retry_frame = g_malloc(sizeof(*frame));
    memcpy(s->wpa_retry_frame, frame, sizeof(*frame));
    s->wpa_retry_frame->next_frame = NULL;
    s->wpa_retry_interval_ns = WPA_RETRY_INITIAL_NS;
    timer_mod(s->wpa_retry_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              s->wpa_retry_interval_ns);
    Esp32_WLAN_insert_frame(s, frame);
}

static void put_be16(uint8_t *p, uint16_t value);

/* The virtual station owns the lease offered by the real guest DHCP server.
 * Answer its ARP resolution on the 802.11 link so guest TCP replies can reach
 * the station before its packets are translated onto SLIRP. */
static bool softap_peer_arp_reply(Esp32WifiState *s, const uint8_t *ethernet,
                                  size_t length)
{
    const uint8_t *arp;
    uint8_t reply[42] = { 0 };
    mac80211_frame *frame;
    bool replied = false;

    if (!s->guest_dhcp_complete || length < sizeof(reply) ||
        get_be16(ethernet + 12) != ETH_P_ARP) {
        return false;
    }
    arp = ethernet + 14;
    if (get_be16(arp) != 1 || get_be16(arp + 2) != ETH_P_IP ||
        arp[4] != 6 || arp[5] != 4 || get_be16(arp + 6) != 1) {
        return false;
    }
    if (memcmp(arp + 24, s->ipaddr, sizeof(s->ipaddr))) {
        trace_esp32_wifi_softap_arp(1, ldl_be_p(arp + 14),
                                    ldl_be_p(arp + 24), false);
        return false;
    }

    put_be16(reply + 12, ETH_P_ARP);
    put_be16(reply + 14, 1);
    put_be16(reply + 16, ETH_P_IP);
    reply[18] = 6;
    reply[19] = 4;
    put_be16(reply + 20, 2);
    memcpy(reply + 22, s->macaddr, sizeof(s->macaddr));
    memcpy(reply + 28, s->ipaddr, sizeof(s->ipaddr));
    memcpy(reply + 32, arp + 8, 6);
    memcpy(reply + 38, arp + 14, 4);

    frame = Esp32_WLAN_create_data_packet(s, reply, sizeof(reply));
    if (frame) {
        /* This is a station-to-AP transmission: To-DS, with the original
         * guest AP as both receiver and Ethernet destination. */
        frame->frame_control.to_ds = 1;
        memcpy(frame->receiver_address, s->guest_ap_macaddr, 6);
        memcpy(frame->transmitter_address, s->macaddr, 6);
        memcpy(frame->address_3, s->guest_ap_macaddr, 6);
        frame->signal_strength = -10;
        Esp32_WLAN_init_ap_frame(s, frame);
        Esp32_WLAN_insert_frame(s, frame);
        replied = true;
    }
    trace_esp32_wifi_softap_arp(1, ldl_be_p(arp + 14),
                                ldl_be_p(arp + 24), replied);
    return replied;
}

static void put_be16(uint8_t *p, uint16_t value)
{
    p[0] = value >> 8;
    p[1] = value;
}

static uint16_t checksum_replace16(uint16_t checksum, uint16_t old_value,
                                   uint16_t new_value)
{
    uint32_t sum = (uint16_t)~checksum;

    sum += (uint16_t)~old_value;
    sum += new_value;
    sum = (sum & 0xffff) + (sum >> 16);
    sum = (sum & 0xffff) + (sum >> 16);
    return (uint16_t)~sum;
}

static void checksum_replace32(uint8_t *checksum, const uint8_t old_addr[4],
                               const uint8_t new_addr[4])
{
    uint16_t value = get_be16(checksum);

    for (unsigned i = 0; i < 4; i += 2) {
        value = checksum_replace16(value, get_be16(old_addr + i),
                                   get_be16(new_addr + i));
    }
    put_be16(checksum, value);
}

/* Rewrite an IPv4 address and incrementally repair IPv4 plus TCP/UDP checksums.
 * This is used only for SLIRP host forwarding to/from the virtual SoftAP peer.
 */
static bool rewrite_ipv4_address(uint8_t *packet, size_t length,
                                 unsigned address_offset,
                                 const uint8_t new_address[4])
{
    uint8_t *ip = packet;
    unsigned ihl;
    uint16_t total_length;
    uint16_t fragment;
    uint8_t *old_address;

    if (length < 20 || (ip[0] >> 4) != 4) {
        return false;
    }
    ihl = (ip[0] & 0x0f) * 4;
    total_length = get_be16(ip + 2);
    if (ihl < 20 || total_length < ihl || total_length > length ||
        (address_offset != 12 && address_offset != 16)) {
        return false;
    }

    old_address = ip + address_offset;
    if (memcmp(old_address, new_address, 4) == 0) {
        return true;
    }
    checksum_replace32(ip + 10, old_address, new_address);

    fragment = get_be16(ip + 6);
    if (!(fragment & 0x1fff)) {
        uint8_t protocol = ip[9];
        uint8_t *transport = ip + ihl;
        size_t transport_length = total_length - ihl;
        unsigned checksum_offset = protocol == IPPROTO_TCP ? 16 :
                                   protocol == IPPROTO_UDP ? 6 : UINT_MAX;

        if (checksum_offset != UINT_MAX && transport_length >= checksum_offset + 2) {
            uint8_t *checksum = transport + checksum_offset;
            /* A zero UDP checksum means no checksum was supplied. */
            if (protocol != IPPROTO_UDP || get_be16(checksum) != 0) {
                checksum_replace32(checksum, old_address, new_address);
            }
        }
    }
    memcpy(old_address, new_address, 4);
    return true;
}

static bool is_dhcp_from_ap(mac80211_frame *frame, dhcp_t **dhcp_out,
                            size_t *options_length)
{
    const uint8_t *ip = frame->data_and_fcs + 8;
    size_t body_length;
    size_t ip_length;
    size_t ip_header_length;
    size_t udp_length;
    size_t dhcp_length;
    const uint8_t *udp;
    dhcp_t *dhcp;

    if (frame->frame_length < IEEE80211_HEADER_SIZE + 8 + 20 + 8 +
                              offsetof(dhcp_t, bp_options)) {
        return false;
    }
    body_length = frame->frame_length - IEEE80211_HEADER_SIZE - 8;
    if (ip[0] >> 4 != 4 || ip[9] != IPPROTO_UDP) {
        return false;
    }
    ip_header_length = (ip[0] & 0x0f) * 4;
    ip_length = get_be16(ip + 2);
    if (ip_header_length < 20 || ip_length < ip_header_length + 8 ||
        ip_length > body_length) {
        return false;
    }
    udp = ip + ip_header_length;
    udp_length = get_be16(udp + 4);
    if (get_be16(udp) != 67 || get_be16(udp + 2) != 68 ||
        udp_length < 8 + offsetof(dhcp_t, bp_options) ||
        udp_length > ip_length - ip_header_length) {
        return false;
    }
    dhcp = (dhcp_t *)(udp + 8);
    dhcp_length = udp_length - 8;
    /* DHCP's magic cookie is the network-order byte sequence 63 82 53 63.
     * The packed struct's host-endian integer comparison is wrong on
     * little-endian hosts. */
    if (dhcp_length < offsetof(dhcp_t, bp_options) ||
        memcmp((const uint8_t *)dhcp + offsetof(dhcp_t, magic_cookie),
               (const uint8_t[]){ 0x63, 0x82, 0x53, 0x63 }, 4)) {
        return false;
    }
    *dhcp_out = dhcp;
    *options_length = dhcp_length - offsetof(dhcp_t, bp_options);
    return true;
}

/* Learn SoftAP identity and channel from the firmware's actual transmitted
 * beacon.  This is also the only channel evidence available until the ROM
 * analog-I2C/RF path is modeled. */
static void learn_guest_beacon(Esp32WifiState *s, const mac80211_frame *frame)
{
    const uint8_t *ies;
    size_t length;

    if (frame->frame_length < IEEE80211_HEADER_SIZE + 12) {
        return;
    }
    ies = frame->data_and_fcs + 12;
    length = frame->frame_length - IEEE80211_HEADER_SIZE - 12;
    while (length >= 2) {
        uint8_t id = ies[0];
        uint8_t ie_length = ies[1];

        if ((size_t)ie_length + 2 > length) {
            return;
        }
        if (id == 0 && ie_length <= 32) {
            memcpy(s->guest_ssid, ies + 2, ie_length);
            s->guest_ssid[ie_length] = '\0';
        } else if (id == 3 && ie_length == 1 && ies[2] >= 1 &&
                   ies[2] <= 14) {
            s->guest_channel = ies[2];
        }
        ies += ie_length + 2;
        length -= ie_length + 2;
    }
}

static uint8_t dhcp_message_type(const dhcp_t *dhcp, size_t options_length)
{
    const uint8_t *option = dhcp->bp_options;

    for (size_t i = 0; i < options_length;) {
        uint8_t tag = option[i++];
        size_t length;

        if (tag == 0) {
            continue;
        }
        if (tag == 255 || i >= options_length) {
            break;
        }
        length = option[i++];
        if (length > options_length - i) {
            break;
        }
        if (tag == 53 && length == 1) {
            return option[i];
        }
        i += length;
    }
    return 0;
}

static bool dhcp_option_copy(const dhcp_t *dhcp, size_t options_length,
                             uint8_t wanted, uint8_t *value,
                             size_t expected_length)
{
    const uint8_t *option = dhcp->bp_options;
    bool found = false;

    for (size_t i = 0; i < options_length;) {
        uint8_t tag = option[i++];
        size_t length;

        if (tag == 0) {
            continue;
        }
        if (tag == 255) {
            return found;
        }
        if (i >= options_length) {
            return false;
        }
        length = option[i++];
        if (length > options_length - i) {
            break;
        }
        if (tag == wanted) {
            if (length != expected_length || found) {
                return false;
            }
            memcpy(value, option + i, length);
            found = true;
        }
        i += length;
    }
    /* DHCP options are terminated by END; don't accept a truncated packet. */
    return false;
}

static bool slirp_to_softap(Esp32WifiState *s, uint8_t *ethernet,
                            size_t ethernet_length)
{
    uint8_t *ip;

    if (ethernet_length < 14) {
        return false;
    }
    if (get_be16(ethernet + 12) != ETH_P_IP) {
        return true;
    }
    if (!s->guest_dhcp_complete) {
        return false;
    }
    ip = ethernet + 14;
    if (!rewrite_ipv4_address(ip, ethernet_length - 14, 16, s->ap_ipaddr) ||
        !rewrite_ipv4_address(ip, ethernet_length - 14, 12, s->ipaddr)) {
        return false;
    }
    return true;
}

/*
 * QEMU user networking resolves its forwarded guest endpoint before it can
 * deliver a host-forwarded TCP packet.  In SoftAP mode that endpoint is the
 * virtual station address (10.0.2.15), while the ESP32 AP itself lives at
 * 192.168.4.1.  Answer only ARP requests for that configured SLIRP endpoint;
 * subsequent IP packets are still translated and injected through the guest
 * 802.11 receive path by slirp_to_softap().
 */
static bool slirp_softap_arp_proxy(Esp32WifiState *s, NetClientState *nc,
                                   const uint8_t *ethernet, size_t length)
{
    uint8_t reply[60] = { 0 };
    const uint8_t *arp;

    if (!s->guest_dhcp_complete || length < 42 ||
        get_be16(ethernet + 12) != ETH_P_ARP) {
        return false;
    }
    arp = ethernet + 14;
    if (get_be16(arp) != 1 || get_be16(arp + 2) != ETH_P_IP ||
        arp[4] != 6 || arp[5] != 4 || get_be16(arp + 6) != 1 ||
        memcmp(arp + 24, slirp_station_ip, sizeof(slirp_station_ip))) {
        return false;
    }

    memcpy(reply, arp + 8, 6);
    memcpy(reply + 6, s->guest_ap_macaddr, sizeof(s->guest_ap_macaddr));
    put_be16(reply + 12, ETH_P_ARP);
    put_be16(reply + 14, 1);
    put_be16(reply + 16, ETH_P_IP);
    reply[18] = 6;
    reply[19] = 4;
    put_be16(reply + 20, 2);
    memcpy(reply + 22, s->guest_ap_macaddr, sizeof(s->guest_ap_macaddr));
    memcpy(reply + 28, slirp_station_ip, sizeof(slirp_station_ip));
    memcpy(reply + 32, arp + 8, 6);
    memcpy(reply + 38, arp + 14, sizeof(slirp_gateway_ip));
    trace_esp32_wifi_slirp_arp_proxy(ldl_be_p(arp + 14),
                                    ldl_be_p(slirp_station_ip));
    qemu_send_packet(nc, reply, sizeof(reply));
    return true;
}

static void softap_to_slirp(Esp32WifiState *s, uint8_t *ethernet,
                            size_t ethernet_length)
{
    uint8_t *ip;

    if (!s->guest_dhcp_complete || ethernet_length < 14 ||
        get_be16(ethernet + 12) != ETH_P_IP) {
        return;
    }
    ip = ethernet + 14;
    rewrite_ipv4_address(ip, ethernet_length - 14, 12, slirp_station_ip);
    rewrite_ipv4_address(ip, ethernet_length - 14, 16, slirp_gateway_ip);
    if ((ip[0] >> 4) == 4 && (ip[0] & 0xf) >= 5) {
        uint8_t *transport = ip + (ip[0] & 0xf) * 4;
        uint16_t source_port = 0, destination_port = 0;
        uint8_t tcp_flags = 0;
        if (ip[9] == IPPROTO_TCP &&
            get_be16(ip + 2) >= (ip[0] & 0xf) * 4 + 20) {
            source_port = get_be16(transport);
            destination_port = get_be16(transport + 2);
            tcp_flags = transport[13];
        }
        trace_esp32_wifi_softap_uplink(ip[9], ldl_be_p(ip + 12),
                                       ldl_be_p(ip + 16), source_port,
                                       destination_port, tcp_flags,
                                       ethernet_length);
    }
}

static void infoprint(struct mac80211_frame *frame) {
    if(DEBUG_DUMPFRAMES) {
        printf("Frame Info type=%d subtype=%d to_ds=%d from_ds=%d duration=%d frame_length=%d\n",frame->frame_control.type,frame->frame_control.sub_type, frame->frame_control.to_ds,frame->frame_control.from_ds, frame->duration_id, frame->frame_length);
        macprint(frame->receiver_address,   "receiver   ");
        macprint(frame->transmitter_address,"transmitter");
        macprint(frame->address_3,          "3rd address");
        uint8_t *b=(uint8_t *)frame;
        for(int i=0;i<frame->frame_length;i++) {
            if((i%16)==0) printf("\n%04x: ",i);
            printf("%02x ",b[i]);
        }
        printf("\n");
    }
}

void Esp32_WLAN_insert_frame(Esp32WifiState *s, struct mac80211_frame *frame)
{
    struct mac80211_frame *i_frame;

    insertCRC(frame);
    if(DEBUG) printf("QEMU: sent frame (qemu AP -> ESP32) type=%d subtype=%d\n",frame->frame_control.type,frame->frame_control.sub_type);
    infoprint(frame);
    s->inject_queue_size++;
    i_frame = s->inject_queue;
    if (!i_frame) {
        s->inject_queue = frame;
    } else {
        while (i_frame->next_frame) {
            i_frame = i_frame->next_frame;
        }
        i_frame->next_frame = frame;
    }

    if (!s->inject_timer_running) {
        // if the injection timer is not
        // running currently, let's schedule
        // one run...
        s->inject_timer_running = 1;
        timer_mod(s->inject_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + INTER_FRAME_TIME);
    }

}

static _Bool Esp32_WLAN_can_receive(NetClientState *ncs)
{
    Esp32WifiState *s = qemu_get_nic_opaque(ncs);

    if (!esp32_wifi_mac_enabled(s)) {
        return false;
    }

    if (!s->guest_ap_associated &&
        s->ap_state != Esp32_WLAN__STATE_STA_DHCP &&
        s->ap_state != Esp32_WLAN__STATE_STA_ASSOCIATED) {
        // we are currently not connected
        // to the access point
        return 0;
    }
    if (s->inject_queue_size > Esp32_WLAN__MAX_INJECT_QUEUE_SIZE) {
        // overload, please give me some time...
        return 0;
    }
    if (s->security.configured &&
        s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED &&
        !s->security.keys_installed) {
        return 0;
    }

    return 1;
}

static ssize_t Esp32_WLAN_receive(NetClientState *ncs,
                                    const uint8_t *buf, size_t size)
{
    Esp32WifiState *s = qemu_get_nic_opaque(ncs);
    struct mac80211_frame *frame;
    uint8_t ethernet[1518];
    uint16_t ethertype = size >= 14 ? get_be16(buf + 12) : 0;
    if (!Esp32_WLAN_can_receive(ncs)) {
        trace_esp32_wifi_backend_receive(s && s->guest_softap,
                                         s ? s->ap_state : 0,
                                         s && s->guest_dhcp_complete,
                                         ethertype, size, false);
        // this should not happen, but in
        // case it does, let's simply drop
        // the packet
        return -1;
    }

    if (!s) {
        return -1;
    }
    if (size < 14 || size > sizeof(ethernet)) {
        trace_esp32_wifi_backend_receive(s->guest_softap, s->ap_state,
                                         s->guest_dhcp_complete, ethertype,
                                         size, false);
        return -1;
    }
    memcpy(ethernet, buf, size);
    if (s->guest_softap && s->guest_ap_associated &&
        s->ap_state != Esp32_WLAN__STATE_STA_ASSOCIATED) {
        /* SLIRP host forwarding targets its virtual NIC address.  Map the
         * endpoint to the guest AP interface after the emulated station has
         * received a DHCP lease from the real guest DHCP server.
         */
        if (slirp_softap_arp_proxy(s, ncs, ethernet, size)) {
            trace_esp32_wifi_backend_receive(s->guest_softap, s->ap_state,
                                             s->guest_dhcp_complete,
                                             ethertype, size, true);
            return size;
        }
        if (!slirp_to_softap(s, ethernet, size)) {
            trace_esp32_wifi_backend_receive(s->guest_softap, s->ap_state,
                                             s->guest_dhcp_complete,
                                             ethertype, size, false);
            return -1;
        }
    }
    trace_esp32_wifi_backend_receive(s->guest_softap, s->ap_state,
                                     s->guest_dhcp_complete, ethertype,
                                     size, true);
    /*
     * A 802.3 packet comes from the qemu network. The
     * access points turns it into a 802.11 frame and
     * forwards it to the wireless device
     */
    frame = Esp32_WLAN_create_data_packet(s, ethernet, size);
    if (frame) {
        /* send message to ESP32 AP */
        if (s->guest_softap && s->guest_ap_associated &&
            s->ap_state != Esp32_WLAN__STATE_STA_ASSOCIATED) {
            /* A QEMU NIC frame is an Ethernet frame arriving from the
             * virtual station.  On air it is a To-DS data frame. */
            frame->frame_control.to_ds = 1;
            frame->frame_control.from_ds = 0;
            memcpy(frame->receiver_address, s->guest_ap_macaddr, 6);
            memcpy(frame->transmitter_address, s->macaddr, 6);
            memcpy(frame->address_3, s->guest_ap_macaddr, 6);
        } else if(s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED) {
            /* A packet arriving from the wired backend is transmitted by
             * the AP to the associated guest station: From-DS, with addr1
             * as the radio receiver, addr2 as the BSSID, and addr3 as the
             * Ethernet source. Keep group destinations group-addressed;
             * the backend NIC's unicast address maps to the associated STA. */
            frame->frame_control.to_ds = 0;
            frame->frame_control.from_ds = 1;
            if ((buf[0] & 1) || memcmp(buf, s->macaddr, 6)) {
                memcpy(frame->receiver_address, buf, 6);
            } else {
                memcpy(frame->receiver_address, s->security.station, 6);
            }
            memcpy(frame->transmitter_address, s->associated_ap_macaddr, 6);
            memcpy(frame->address_3, buf + 6, 6);
        }
        else { // send message to ESP32 station
            frame->frame_control.to_ds = 0;
            frame->frame_control.from_ds = 1;
            memcpy(frame->receiver_address, &buf[0], 6);
            memcpy(frame->transmitter_address, s->associated_ap_macaddr, 6);
            memcpy(frame->address_3, &buf[6], 6); // source address
        }
        Esp32_WLAN_init_ap_frame(s, frame);
        if (s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED &&
            s->security.configured) {
            const uint8_t *expected = esp32_wpa2_expected_data_key(
                &s->security, frame->receiver_address);

            if (!s->security.keys_installed ||
                !esp32_wpa2_encrypt_data(&s->security, frame, expected)) {
                g_free(frame);
                return -1;
            }
        }
        Esp32_WLAN_insert_frame(s, frame);
    }
    return size;
}
static void Esp32_WLAN_net_cleanup(NetClientState *ncs) { }

static NetClientInfo net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = Esp32_WLAN_can_receive,
    .receive = Esp32_WLAN_receive,
    .cleanup = Esp32_WLAN_net_cleanup,
};

void Esp32_WLAN_reset(Esp32WifiState *s)
{
    mac80211_frame *frame;

    wpa_retry_clear(s);

    if (s->beacon_timer) {
        timer_del(s->beacon_timer);
    }
    if (s->inject_timer) {
        timer_del(s->inject_timer);
    }
    while ((frame = s->inject_queue) != NULL) {
        s->inject_queue = frame->next_frame;
        g_free(frame);
    }

    s->ap_state = Esp32_WLAN__STATE_NOT_AUTHENTICATED;
    s->guest_softap = false;
    s->guest_ap_associated = false;
    s->guest_dhcp_complete = false;
    s->guest_dhcp_xid_active = false;
    memset(s->guest_dhcp_xid, 0, sizeof(s->guest_dhcp_xid));
    memset(s->guest_dhcp_server_id, 0, sizeof(s->guest_dhcp_server_id));
    memset(s->guest_dhcp_offered_ip, 0, sizeof(s->guest_dhcp_offered_ip));
    s->guest_ssid[0] = '\0';
    s->guest_channel = 0;
    memset(s->guest_ap_macaddr, 0, sizeof(s->guest_ap_macaddr));
    s->rf_channel = 0;
    memset(s->ipaddr, 0, sizeof(s->ipaddr));
    memset(s->ap_ipaddr, 0, sizeof(s->ap_ipaddr));
    s->beacon_ap = 0;
    memcpy(s->ap_macaddr, (uint8_t[]){0x01, 0x13, 0x46, 0xbf, 0x31, 0x50},
           sizeof(s->ap_macaddr));
    memcpy(s->macaddr, (uint8_t[]){0x10, 0x01, 0x00, 0xc4, 0x0a, 0x24},
           sizeof(s->macaddr));
    peer_ap.ssid = s->peer_ssid ? s->peer_ssid : "test-ap";
    peer_ap.channel = s->peer_channel ? s->peer_channel : 1;
    esp32_wpa2_configure(&s->security, peer_ap.ssid, s->peer_password,
                         peer_ap.mac_address);
    peer_ap.wpa2 = s->security.configured;

    s->inject_timer_running = 0;
    s->inject_sequence_number = 0;
    s->inject_queue = NULL;
    s->inject_queue_size = 0;
    if (s->beacon_timer && esp32_wifi_mac_enabled(s)) {
        timer_mod(s->beacon_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000000);
    }
}

void Esp32_WLAN_clock_changed(Esp32WifiState *s)
{
    if (!s->beacon_timer || !s->inject_timer) {
        return;
    }
    if (esp32_wifi_mac_enabled(s)) {
        timer_mod(s->beacon_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + BEACON_TIME);
        if (s->inject_queue && !s->inject_timer_running) {
            s->inject_timer_running = 1;
            timer_mod(s->inject_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      INTER_FRAME_TIME);
        }
    } else {
        timer_del(s->beacon_timer);
        timer_del(s->inject_timer);
        s->inject_timer_running = 0;
    }
}

void Esp32_WLAN_cleanup(Esp32WifiState *s)
{
    mac80211_frame *frame;

    wpa_retry_clear(s);

    if (s->beacon_timer) {
        timer_del(s->beacon_timer);
        timer_free(s->beacon_timer);
        s->beacon_timer = NULL;
    }
    if (s->inject_timer) {
        timer_del(s->inject_timer);
        timer_free(s->inject_timer);
        s->inject_timer = NULL;
    }
    if (s->wpa_retry_timer) {
        timer_del(s->wpa_retry_timer);
        timer_free(s->wpa_retry_timer);
        s->wpa_retry_timer = NULL;
    }
    while ((frame = s->inject_queue) != NULL) {
        s->inject_queue = frame->next_frame;
        g_free(frame);
    }
    s->inject_queue_size = 0;
    s->inject_timer_running = 0;
    s->guest_softap = false;
    s->guest_ap_associated = false;
    s->guest_dhcp_complete = false;
    s->guest_dhcp_xid_active = false;
    memset(&s->security, 0, sizeof(s->security));
}

void Esp32_WLAN_setup_ap(DeviceState *dev,Esp32WifiState *s) {
    s->beacon_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, Esp32_WLAN_beacon_timer, s);
    s->inject_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, Esp32_WLAN_inject_timer, s);
    s->wpa_retry_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                      wpa_retry_timer, s);
    Esp32_WLAN_reset(s);

    s->nic = qemu_new_nic(&net_info, &s->conf, object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->macaddr);
}

static void send_single_frame(Esp32WifiState *s, struct mac80211_frame *frame, struct mac80211_frame *reply) {
    reply->sequence_control.sequence_number = s->inject_sequence_number++ +0x730;
    reply->signal_strength=-10;

    if(frame) {
        memcpy(reply->receiver_address, frame->transmitter_address, 6);
        memcpy(reply->transmitter_address, s->macaddr, 6);
        memcpy(reply->address_3, frame->transmitter_address, 6);
    }

    Esp32_WLAN_insert_frame(s, reply);
}

/* Convert a guest station's To-DS 802.11 data frame to Ethernet for the
 * configured QEMU network backend.  Only the observed SNAP encapsulation is
 * accepted; malformed or non-data payloads are dropped. */
static bool sta_frame_to_ethernet(const mac80211_frame *frame,
                                 uint8_t *ethernet, size_t *ethernet_length)
{
    size_t payload_length;

    if (!frame->frame_control.to_ds || frame->frame_control.from_ds ||
        frame->frame_length < IEEE80211_HEADER_SIZE + 8) {
        return false;
    }
    if (memcmp(frame->data_and_fcs,
               (const uint8_t[]){ 0xaa, 0xaa, 0x03, 0, 0, 0 }, 6)) {
        return false;
    }
    /* ESP32 TX DMA frames omit the PHY-generated FCS. */
    payload_length = frame->frame_length - IEEE80211_HEADER_SIZE - 8;
    if (payload_length > 1500) {
        return false;
    }
    memcpy(ethernet, frame->address_3, 6);
    memcpy(ethernet + 6, frame->transmitter_address, 6);
    memcpy(ethernet + 12, frame->data_and_fcs + 6, 2);
    memcpy(ethernet + 14, frame->data_and_fcs + 8, payload_length);
    *ethernet_length = payload_length + 14;
    return true;
}
void Esp32_WLAN_handle_frame(Esp32WifiState *s, struct mac80211_frame *frame)
{
    struct mac80211_frame *reply = NULL;
    bool start_wpa_handshake = false;
    static access_point_info dummy_ap={0};
    char ssid[64];
    unsigned long ethernet_frame_size;
    unsigned char ethernet_frame[1518] = {0};
    trace_esp32_wifi_guest_frame(1, frame->frame_control.type,
                                 frame->frame_control.sub_type,
                                 s->ap_state, s->rf_channel,
                                 frame->frame_length);
    if (frame->frame_control.type == IEEE80211_TYPE_DATA) {
        trace_esp32_wifi_softap_tx_frame(frame->frame_control.to_ds,
                                         frame->frame_control.from_ds,
                                         mac_to_u64(frame->receiver_address),
                                         mac_to_u64(frame->transmitter_address),
                                         mac_to_u64(frame->address_3),
                                         frame->frame_length);
    }
    if(DEBUG)
        printf("QEMU: received frame (esp32 -> qemu) type=%d subtype=%d chan=%d to_ds=%d from_ds=%d state=%d\n",frame->frame_control.type, frame->frame_control.sub_type, s->rf_channel, frame->frame_control.to_ds, frame->frame_control.from_ds, s->ap_state);
    infoprint(frame);
    access_point_info *ap_info=0;
    if (frame_targets_peer_ap(s, frame)) {
        ap_info = &peer_ap;
    }

    if(frame->frame_control.type == IEEE80211_TYPE_MGT) {
        switch(frame->frame_control.sub_type) {
            case IEEE80211_TYPE_MGT_SUBTYPE_BEACON:
                /* A guest beacon means its own SoftAP is running. */
                if (!s->guest_softap) {
                    s->guest_softap = true;
                    learn_guest_beacon(s, frame);
                    memcpy(s->guest_ap_macaddr,
                           frame->transmitter_address, 6);
                    mac80211_frame *auth = Esp32_WLAN_create_authentication_request();
                    memcpy(auth->receiver_address, s->guest_ap_macaddr, 6);
                    memcpy(auth->transmitter_address, s->macaddr, 6);
                    memcpy(auth->address_3, s->guest_ap_macaddr, 6);
                    Esp32_WLAN_init_ap_frame(s, auth);
                    Esp32_WLAN_insert_frame(s, auth);
                }
                break;
                if(s->ap_state==Esp32_WLAN__STATE_NOT_AUTHENTICATED || s->ap_state==Esp32_WLAN__STATE_AUTHENTICATED) {
                    strncpy(ssid,(char *)frame->data_and_fcs+14,frame->data_and_fcs[13]);
                    if(DEBUG) printf("QEMU: beacon from %s\n",ssid);
                    dummy_ap.ssid=ssid;
                    s->ap_state=Esp32_WLAN__STATE_STA_NOT_AUTHENTICATED;
                    send_single_frame(s,frame,Esp32_WLAN_create_probe_request(&dummy_ap));
                }
                break;
            case IEEE80211_TYPE_MGT_SUBTYPE_PROBE_RESP:
                ap_info=&dummy_ap;
                strncpy(ssid,(char *)frame->data_and_fcs+14,frame->data_and_fcs[13]);
                if(DEBUG) printf("QEMU: probe resp from %s\n",ssid);
                dummy_ap.ssid=ssid;
                s->ap_state=Esp32_WLAN__STATE_STA_NOT_AUTHENTICATED;
                send_single_frame(s,frame,Esp32_WLAN_create_deauthentication());
                send_single_frame(s,frame,Esp32_WLAN_create_authentication_request());
                break;
            case IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_RESP:
                if (s->guest_softap) {
                    if (frame->data_and_fcs[2] == 0 && frame->data_and_fcs[3] == 0) {
                        s->guest_ap_associated = true;
                        s->guest_dhcp_complete = false;
                        memset(s->ipaddr, 0, sizeof(s->ipaddr));
                        memset(s->ap_ipaddr, 0, sizeof(s->ap_ipaddr));
                        memcpy(s->associated_ap_macaddr,
                               s->guest_ap_macaddr, 6);
                        s->guest_dhcp_complete = false;
                        s->guest_dhcp_xid_active =
                            qcrypto_random_bytes(s->guest_dhcp_xid,
                                                 sizeof(s->guest_dhcp_xid),
                                                 NULL) == 0;
                        if (!s->guest_dhcp_xid_active) {
                            return;
                        }
                        mac80211_frame *discover = Esp32_WLAN_create_dhcp_discover(
                            s->macaddr, s->guest_dhcp_xid);
                        memcpy(discover->receiver_address,
                               s->guest_ap_macaddr, 6);
                        memcpy(discover->transmitter_address, s->macaddr, 6);
                        memcpy(discover->address_3, BROADCAST, 6);
                        Esp32_WLAN_init_ap_frame(s, discover);
                        Esp32_WLAN_insert_frame(s, discover);
                    }
                    break;
                }
                /* This is a guest-to-peer association request.  Association
                 * success moves the station to the real packet path; DHCP is
                 * driven by the guest's own subsequent 802.11 frames. */
                break;
            case IEEE80211_TYPE_MGT_SUBTYPE_DISASSOCIATION:
                DEBUG_PRINT_AP(("QEMU: Received disassociation!\n"));
                if (s->guest_softap &&
                    !memcmp(frame->address_3, s->guest_ap_macaddr, 6)) {
                    guest_softap_client_disconnect(s);
                }
                if (frame_targets_peer_ap(s, frame) && s->security.associated) {
                    wpa_clear_association(s);
                }
                if (frame_targets_peer_ap(s, frame) &&
                    s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED) {
                    s->ap_state = Esp32_WLAN__STATE_AUTHENTICATED;
                }
                break;
            case IEEE80211_TYPE_MGT_SUBTYPE_DEAUTHENTICATION:
                DEBUG_PRINT_AP(("QEMU: Received deauthentication!\n"));
                if (s->guest_softap &&
                    !memcmp(frame->address_3, s->guest_ap_macaddr, 6)) {
                    guest_softap_client_disconnect(s);
                }
                if (frame_targets_peer_ap(s, frame) && s->security.associated) {
                    wpa_clear_association(s);
                }
                if (frame_targets_peer_ap(s, frame) &&
                    (s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED ||
                     s->ap_state == Esp32_WLAN__STATE_AUTHENTICATED)) {
                    s->ap_state = Esp32_WLAN__STATE_NOT_AUTHENTICATED;
                }
                break;
            case IEEE80211_TYPE_MGT_SUBTYPE_AUTHENTICATION:
                if(frame->data_and_fcs[2]==2) { // response
                    DEBUG_PRINT_AP(("QEMU: Received authentication response!\n"));
                    if (s->guest_softap) {
                        access_point_info guest_ap = {
                            .ssid = s->guest_ssid[0] ? s->guest_ssid : "",
                            .channel = s->guest_channel,
                        };
                        mac80211_frame *assoc = Esp32_WLAN_create_association_request(&guest_ap);
                        memcpy(assoc->receiver_address,
                               s->guest_ap_macaddr, 6);
                        memcpy(assoc->transmitter_address, s->macaddr, 6);
                        memcpy(assoc->address_3, s->guest_ap_macaddr, 6);
                        Esp32_WLAN_init_ap_frame(s, assoc);
                        Esp32_WLAN_insert_frame(s, assoc);
                    }
                }
                break;
        }
        if(ap_info) {
            memcpy(s->ap_macaddr, ap_info->mac_address, 6);
            switch(frame->frame_control.sub_type) {
                case IEEE80211_TYPE_MGT_SUBTYPE_PROBE_REQ:
                    DEBUG_PRINT_AP(("QEMU: Received probe request!\n"));
                    reply = Esp32_WLAN_create_probe_response(ap_info);
                    break;
                case IEEE80211_TYPE_MGT_SUBTYPE_AUTHENTICATION:
                    if(frame->data_and_fcs[2]==1) { // request
                        DEBUG_PRINT_AP(("QEMU: Received authentication request!\n"));
                        reply = Esp32_WLAN_create_authentication_response(ap_info);
                        if (s->ap_state == Esp32_WLAN__STATE_NOT_AUTHENTICATED ||
                            s->ap_state == Esp32_WLAN__STATE_STA_NOT_AUTHENTICATED) {
                            s->ap_state = Esp32_WLAN__STATE_STA_AUTHENTICATED;
                        }
                    }
                break;
                case IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_REQ:
                case IEEE80211_TYPE_MGT_SUBTYPE_REASSOCIATION_REQ:
                    DEBUG_PRINT_AP(("QEMU: Received association request!\n"));
                    if (s->security.configured) {
                        bool duplicate = s->security.associated &&
                            !memcmp(s->security.station,
                                    frame->transmitter_address, 6);
                        uint16_t status =
                            (s->ap_state == Esp32_WLAN__STATE_STA_AUTHENTICATED ||
                             duplicate)
                            ? esp32_wpa2_assoc_request_status(&s->security, frame)
                            : 1;

                        if (!status) {
                            reply = esp32_wpa2_assoc_response(&s->security);
                            if (reply &&
                                s->ap_state == Esp32_WLAN__STATE_STA_AUTHENTICATED) {
                                s->ap_state = Esp32_WLAN__STATE_STA_ASSOCIATED;
                                memcpy(s->associated_ap_macaddr, s->ap_macaddr, 6);
                                start_wpa_handshake = true;
                            }
                        } else {
                            reply = esp32_wpa2_assoc_reject_response(
                                s->ap_macaddr, frame->transmitter_address,
                                status);
                        }
                    } else {
                        reply = Esp32_WLAN_create_association_response(ap_info);
                        if (reply &&
                            s->ap_state == Esp32_WLAN__STATE_STA_AUTHENTICATED) {
                            s->ap_state = Esp32_WLAN__STATE_STA_ASSOCIATED;
                            memcpy(s->associated_ap_macaddr, s->ap_macaddr, 6);
                        }
                    }
                    break;
            }
            if (reply) {
                reply->signal_strength=ap_info->sigstrength;
                memcpy(reply->receiver_address, frame->transmitter_address, 6);
                memcpy(reply->transmitter_address, s->ap_macaddr, 6);
                memcpy(reply->address_3, s->ap_macaddr, 6);
                Esp32_WLAN_init_ap_frame(s, reply);
                Esp32_WLAN_insert_frame(s, reply);
                if (start_wpa_handshake) {
                    wpa_queue_key_frame(s, esp32_wpa2_start(&s->security));
                }
            }
        }
    }
    if ((frame->frame_control.type == IEEE80211_TYPE_DATA) &&
        (frame->frame_control.sub_type == IEEE80211_TYPE_DATA_SUBTYPE_DATA)) {
        if (s->security.configured &&
            s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED &&
            frame->frame_control.to_ds && !frame->frame_control.from_ds &&
            !memcmp(frame->receiver_address, peer_ap.mac_address, 6) &&
            frame->frame_length >= IEEE80211_HEADER_SIZE + 16 &&
            !memcmp(frame->data_and_fcs,
                    (uint8_t[]){0xaa,0xaa,3,0,0,0,0x88,0x8e}, 8)) {
            bool m3_was_sent = s->security.m3_sent;
            mac80211_frame *m3 = esp32_wpa2_rx_eapol(&s->security, frame);
            if (m3) {
                if (m3_was_sent && s->wpa_retry_frame) {
                    g_free(m3);
                    wpa_retry_send_cached(s);
                } else {
                    wpa_queue_key_frame(s, m3);
                }
            }
            if (s->security.keys_installed) {
                wpa_retry_clear(s);
                wpa_remove_queued_key_frames(s);
                return;
            }
            return;
        }
        if (s->guest_softap && s->guest_ap_associated &&
            !frame->frame_control.to_ds && frame->frame_control.from_ds &&
            !memcmp(frame->transmitter_address, s->guest_ap_macaddr, 6)) {
            dhcp_t *dhcp;
            size_t options_length;

            if (is_dhcp_from_ap(frame, &dhcp, &options_length)) {
                uint8_t message_type = dhcp_message_type(dhcp, options_length);

                if (message_type == 2) { /* DHCP OFFER */
                    uint8_t server_id[4] = {0};
                    bool has_server_id = dhcp_option_copy(
                        dhcp, options_length, 54, server_id,
                        sizeof(server_id));
                    trace_esp32_wifi_softap_dhcp(message_type, s->ap_state,
                                                 s->guest_dhcp_complete,
                                                 ldl_be_p(dhcp->yiaddr),
                                                 ldl_be_p(s->ap_ipaddr));
                    trace_esp32_wifi_softap_dhcp_offer(
                        ldl_be_p(&dhcp->xid), ldl_be_p(dhcp->yiaddr),
                        has_server_id, ldl_be_p(server_id),
                        lduw_be_p(&dhcp->flags));
                    if (!has_server_id || dhcp->opcode != 2 ||
                        dhcp->htype != 1 ||
                        dhcp->hlen != 6 ||
                        !s->guest_dhcp_xid_active ||
                        memcmp(&dhcp->xid, s->guest_dhcp_xid, 4) ||
                        memcmp(dhcp->chaddr, s->macaddr,
                               sizeof(s->macaddr))) {
                        qemu_log_mask(LOG_GUEST_ERROR,
                                      "wifi DHCP OFFER has invalid server identifier or client identity\n");
                        return;
                    }
                    memcpy(s->ap_ipaddr, frame->data_and_fcs + 8 + 12, 4);
                    memcpy(s->guest_dhcp_offered_ip, dhcp->yiaddr, 4);
                    memcpy(s->guest_dhcp_server_id, server_id, 4);
                    trace_esp32_wifi_softap_dhcp_request(
                        ldl_be_p(&dhcp->xid), ldl_be_p(dhcp->yiaddr),
                        ldl_be_p(server_id));
                    mac80211_frame *request =
                        Esp32_WLAN_create_dhcp_request(dhcp, server_id);
                    memcpy(request->receiver_address, s->guest_ap_macaddr, 6);
                    memcpy(request->transmitter_address, s->macaddr, 6);
                    memcpy(request->address_3, BROADCAST, 6);
                    Esp32_WLAN_init_ap_frame(s, request);
                    Esp32_WLAN_insert_frame(s, request);
                    return;
                }
                if ((message_type == 5 || message_type == 6) &&
                    dhcp->opcode == 2 && dhcp->htype == 1 &&
                    dhcp->hlen == 6 && s->guest_dhcp_xid_active &&
                    !memcmp(&dhcp->xid, s->guest_dhcp_xid, 4) &&
                    !memcmp(dhcp->chaddr, s->macaddr, 6)) {
                    uint8_t server_id[4];
                    if (!dhcp_option_copy(dhcp, options_length, 54,
                                          server_id, sizeof(server_id)) ||
                        memcmp(server_id, s->guest_dhcp_server_id, 4)) {
                        return;
                    }
                    if (message_type == 5 &&
                        memcmp(dhcp->yiaddr, s->guest_dhcp_offered_ip, 4)) {
                        return;
                    }
                    memcpy(s->ap_ipaddr, frame->data_and_fcs + 8 + 12, 4);
                    if (message_type == 6) {
                        s->guest_dhcp_complete = false;
                        s->guest_dhcp_xid_active = false;
                        memset(s->ipaddr, 0, sizeof(s->ipaddr));
                        memset(s->guest_dhcp_server_id, 0,
                               sizeof(s->guest_dhcp_server_id));
                        return;
                    }
                    memcpy(s->ipaddr, dhcp->yiaddr, 4);
                    s->guest_dhcp_complete = true;
                    s->guest_dhcp_xid_active = false;
                    trace_esp32_wifi_softap_dhcp(message_type, s->ap_state,
                                                 s->guest_dhcp_complete,
                                                 ldl_be_p(dhcp->yiaddr),
                                                 ldl_be_p(s->ap_ipaddr));
                    return;
                }
            }
        }
        if (s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED &&
            frame->frame_control.to_ds && !frame->frame_control.from_ds &&
            !memcmp(frame->receiver_address, peer_ap.mac_address, 6)) {
            size_t ethernet_length;
            if (s->security.configured) {
                uint8_t fcs[4];

                if (frame->frame_length < IEEE80211_HEADER_SIZE + 2 ||
                    !s->security.keys_installed ||
                    !(((uint8_t *)frame)[1] & 0x40)) {
                    return;
                }
                /* The external AP receives the complete on-air frame from
                 * the MAC path. Validate and exclude its FCS before CCMP
                 * authenticates ciphertext; it is not CCMP input. */
                if (!wlan_strip_valid_fcs(frame, fcs)) {
                    return;
                }
                const uint8_t *expected = esp32_wpa2_expected_data_key(
                    &s->security, frame->receiver_address);
                if (!esp32_wpa2_decrypt_data(&s->security, frame, expected)) {
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "wifi STA TX CCMP rejected len=%u RA=%02x:%02x:%02x:%02x:%02x:%02x PN=%02x:%02x:%02x:%02x:%02x:%02x\n",
                                  frame->frame_length,
                                  frame->receiver_address[0], frame->receiver_address[1],
                                  frame->receiver_address[2], frame->receiver_address[3],
                                  frame->receiver_address[4], frame->receiver_address[5],
                                  frame->data_and_fcs[0], frame->data_and_fcs[1],
                                  frame->data_and_fcs[4], frame->data_and_fcs[5],
                                  frame->data_and_fcs[6], frame->data_and_fcs[7]);
                    return;
                }
            }
            if (sta_frame_to_ethernet(frame, ethernet_frame,
                                      &ethernet_length)) {
                qemu_send_packet(qemu_get_queue(s->nic), ethernet_frame,
                                 ethernet_length);
            } else {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "wifi STA TX decrypted frame rejected SNAP len=%u bytes=%02x%02x%02x%02x%02x%02x type=%02x%02x\n",
                              frame->frame_length,
                              frame->data_and_fcs[0], frame->data_and_fcs[1],
                              frame->data_and_fcs[2], frame->data_and_fcs[3],
                              frame->data_and_fcs[4], frame->data_and_fcs[5],
                              frame->data_and_fcs[6], frame->data_and_fcs[7]);
            }
        } else if (s->guest_softap && s->guest_ap_associated &&
                   frame_uses_guest_ap_bssid(s, frame)) {
            // ESP32 to QEMU
            /*
            * The access point uses the 802.11 frame
            * and sends a 802.3 frame into the network...
            * This packet is then understandable by
            * qemu-slirp
            *
            * If we ever want the access point to offer
            * some services, it can be added here!!
            */
            /* Guest SoftAP uplink is To-DS: addr3 is Ethernet destination,
             * addr2 is the external station.  The legacy station path uses
             * the same address mapping. */
            ethernet_frame[12] = frame->data_and_fcs[6];
            ethernet_frame[13] = frame->data_and_fcs[7];

            // the originator the packet is the station who sent the frame
            memcpy(&ethernet_frame[6], frame->transmitter_address, 6);

            if (frame->frame_control.to_ds && !frame->frame_control.from_ds) {
                memcpy(&ethernet_frame[0], frame->address_3, 6);
            } else if (!frame->frame_control.to_ds && frame->frame_control.from_ds) {
                memcpy(&ethernet_frame[0], frame->receiver_address, 6);
                memcpy(&ethernet_frame[6], frame->address_3, 6);
            } else {
                return;
            }

            ethernet_frame_size = frame->frame_length - 22;

            // limit data to max length of ethernet frame
            if (ethernet_frame_size > (sizeof(ethernet_frame) - 14)) {
                ethernet_frame_size = (sizeof(ethernet_frame) - 14);
            }

            // set ethernet data
            memcpy(&ethernet_frame[14], &frame->data_and_fcs[8], ethernet_frame_size);

            if (s->guest_softap &&
                softap_peer_arp_reply(s, ethernet_frame,
                                      ethernet_frame_size)) {
                return;
            }

            if (s->guest_softap) {
                softap_to_slirp(s, ethernet_frame,
                                ethernet_frame_size + 14);
            }

            // send frame
            qemu_send_packet(qemu_get_queue(s->nic), ethernet_frame, ethernet_frame_size);
        }
    }
}
