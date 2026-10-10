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

#include "hw/xtensa/esp32_wifi.h"
#include "esp32_wlan.h"
#include "esp32_wlan_packet.h"

/* RF channel setup through the ROM analog-I2C path is not modeled yet. */
int esp32_wifi_channel;

// 50ms between beacons
#define BEACON_TIME 50000000
#define INTER_FRAME_TIME 5000000
#define DEBUG 0
#define DEBUG_DUMPFRAMES 0

/* One explicit external-AP fixture, configured with -device misc.esp32_wifi,
 *peer-ssid=...,peer-channel=....  It is a network peer, not a claimed model
 *of an environment discovered on the real radio. */
static access_point_info peer_ap = {
    .sigstrength = -40,
    .mac_address = { 0x10, 0x01, 0x00, 0xc4, 0x0a, 0x51 },
};

static void Esp32_WLAN_beacon_timer(void *opaque)
{
    struct mac80211_frame *frame;
    Esp32WifiState *s = (Esp32WifiState *)opaque;

    if (!esp32_wifi_mac_enabled(s)) {
        return;
    }

    // only send a beacon if we are an access point
    if(s->ap_state!=Esp32_WLAN__STATE_STA_ASSOCIATED) {
        if (peer_ap.channel == esp32_wifi_channel && peer_ap.ssid) {
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
        Esp32_sendFrame(s, (void *)frame, frame->frame_length,frame->signal_strength);
        free(frame);
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

    if (s->ap_state != Esp32_WLAN__STATE_ASSOCIATED  &&
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
    if (!Esp32_WLAN_can_receive(ncs)) {
        // this should not happen, but in
        // case it does, let's simply drop
        // the packet
        return -1;
    }

    if (!s) {
        return -1;
    }
    if (size < 14 || size > sizeof(ethernet)) {
        return -1;
    }
    memcpy(ethernet, buf, size);
    if (s->guest_softap) {
        /* SLIRP host forwarding targets its virtual NIC address.  Map the
         * endpoint to the guest AP interface after the emulated station has
         * received a DHCP lease from the real guest DHCP server.
         */
        if (!slirp_to_softap(s, ethernet, size)) {
            return -1;
        }
    }
    /*
     * A 802.3 packet comes from the qemu network. The
     * access points turns it into a 802.11 frame and
     * forwards it to the wireless device
     */
    frame = Esp32_WLAN_create_data_packet(s, ethernet, size);
    if (frame) {
        /* send message to ESP32 AP */
        if (s->guest_softap) {
            /* A QEMU NIC frame is an Ethernet frame arriving from the
             * virtual station.  On air it is a To-DS data frame. */
            frame->frame_control.to_ds = 1;
            frame->frame_control.from_ds = 0;
            memcpy(frame->receiver_address, s->ap_macaddr, 6);
            memcpy(frame->transmitter_address, s->macaddr, 6);
            memcpy(frame->address_3, s->ap_macaddr, 6);
        } else if(s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED) {
            /* A packet arriving from the wired backend is transmitted by
             * the AP to the associated guest station: From-DS, with addr1
             * as the Ethernet destination, addr2 as the BSSID, and addr3
             * as the Ethernet source. */
            frame->frame_control.to_ds = 0;
            frame->frame_control.from_ds = 1;
            memcpy(frame->receiver_address, buf, 6);
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
        if (!s->guest_softap && s->security.configured &&
            s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED &&
            !esp32_wpa2_encrypt_data(&s->security, frame)) {
            g_free(frame);
            return -1;
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

    if (s->beacon_timer) {
        timer_del(s->beacon_timer);
    }
    if (s->inject_timer) {
        timer_del(s->inject_timer);
    }
    while ((frame = s->inject_queue) != NULL) {
        s->inject_queue = frame->next_frame;
        free(frame);
    }

    s->ap_state = Esp32_WLAN__STATE_NOT_AUTHENTICATED;
    s->guest_softap = false;
    s->guest_dhcp_complete = false;
    s->guest_ssid[0] = '\0';
    s->guest_channel = 0;
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
    while ((frame = s->inject_queue) != NULL) {
        s->inject_queue = frame->next_frame;
        free(frame);
    }
    s->inject_queue_size = 0;
    s->inject_timer_running = 0;
    s->guest_softap = false;
    s->guest_dhcp_complete = false;
    memset(&s->security, 0, sizeof(s->security));
}

void Esp32_WLAN_setup_ap(DeviceState *dev,Esp32WifiState *s) {
    s->beacon_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, Esp32_WLAN_beacon_timer, s);
    s->inject_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, Esp32_WLAN_inject_timer, s);
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
    static access_point_info dummy_ap={0};
    char ssid[64];
    unsigned long ethernet_frame_size;
    unsigned char ethernet_frame[1518] = {0};
    if(DEBUG)
        printf("QEMU: received frame (esp32 -> qemu) type=%d subtype=%d chan=%d to_ds=%d from_ds=%d state=%d\n",frame->frame_control.type, frame->frame_control.sub_type, esp32_wifi_channel, frame->frame_control.to_ds, frame->frame_control.from_ds, s->ap_state);
    infoprint(frame);
    access_point_info *ap_info=0;
    if (peer_ap.channel == esp32_wifi_channel && peer_ap.ssid) {
        ap_info = &peer_ap;
    }

    if(frame->frame_control.type == IEEE80211_TYPE_MGT) {
        switch(frame->frame_control.sub_type) {
            case IEEE80211_TYPE_MGT_SUBTYPE_BEACON:
                /* A guest beacon means its own SoftAP is running. */
                if (!s->guest_softap) {
                    s->guest_softap = true;
                    learn_guest_beacon(s, frame);
                    memcpy(s->ap_macaddr, frame->transmitter_address, 6);
                    mac80211_frame *auth = Esp32_WLAN_create_authentication_request();
                    memcpy(auth->receiver_address, s->ap_macaddr, 6);
                    memcpy(auth->transmitter_address, s->macaddr, 6);
                    memcpy(auth->address_3, s->ap_macaddr, 6);
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
                        s->ap_state = Esp32_WLAN__STATE_ASSOCIATED;
                        s->guest_dhcp_complete = false;
                        memset(s->ipaddr, 0, sizeof(s->ipaddr));
                        memset(s->ap_ipaddr, 0, sizeof(s->ap_ipaddr));
                        memcpy(s->associated_ap_macaddr, s->ap_macaddr, 6);
                        mac80211_frame *discover = Esp32_WLAN_create_dhcp_discover();
                        memcpy(discover->receiver_address, s->ap_macaddr, 6);
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
                send_single_frame(s,frame,Esp32_WLAN_create_disassociation());
                if (s->ap_state == Esp32_WLAN__STATE_ASSOCIATED || s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED) {
                    s->ap_state = Esp32_WLAN__STATE_AUTHENTICATED;
                }
                break;
            case IEEE80211_TYPE_MGT_SUBTYPE_DEAUTHENTICATION:
                DEBUG_PRINT_AP(("QEMU: Received deauthentication!\n"));
                //reply = Esp32_WLAN_create_authentication_response(ap_info);
                if (s->ap_state == Esp32_WLAN__STATE_AUTHENTICATED) {
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
                        memcpy(assoc->receiver_address, s->ap_macaddr, 6);
                        memcpy(assoc->transmitter_address, s->macaddr, 6);
                        memcpy(assoc->address_3, s->ap_macaddr, 6);
                        Esp32_WLAN_init_ap_frame(s, assoc);
                        Esp32_WLAN_insert_frame(s, assoc);
                    } else {
                        send_single_frame(s,frame,Esp32_WLAN_create_association_request(&dummy_ap));
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
                    DEBUG_PRINT_AP(("QEMU: Received association request!\n"));
                    if (s->security.configured) {
                        if (s->ap_state == Esp32_WLAN__STATE_STA_AUTHENTICATED &&
                            esp32_wpa2_assoc_request(&s->security, frame)) {
                            reply = esp32_wpa2_assoc_response(&s->security);
                        }
                    } else {
                        reply = Esp32_WLAN_create_association_response(ap_info);
                    }
                    if (reply && s->ap_state == Esp32_WLAN__STATE_STA_AUTHENTICATED) {
                        s->ap_state = Esp32_WLAN__STATE_STA_ASSOCIATED;
                        memcpy(s->macaddr, frame->transmitter_address, 6);
                        memcpy(s->associated_ap_macaddr, s->ap_macaddr, 6);
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
                if (frame->frame_control.sub_type ==
                    IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_REQ &&
                    s->security.configured) {
                    mac80211_frame *m1 = esp32_wpa2_start(&s->security);
                    if (m1) {
                        memcpy(m1->receiver_address, s->macaddr, 6);
                        memcpy(m1->transmitter_address, s->ap_macaddr, 6);
                        memcpy(m1->address_3, s->ap_macaddr, 6);
                        Esp32_WLAN_init_ap_frame(s, m1);
                        Esp32_WLAN_insert_frame(s, m1);
                    }
                }
            }
        }
    }
    if ((frame->frame_control.type == IEEE80211_TYPE_DATA) &&
        (frame->frame_control.sub_type == IEEE80211_TYPE_DATA_SUBTYPE_DATA)) {
        if (s->security.configured &&
            s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED &&
            frame->frame_length >= IEEE80211_HEADER_SIZE + 16 &&
            !memcmp(frame->data_and_fcs,
                    (uint8_t[]){0xaa,0xaa,3,0,0,0,0x88,0x8e}, 8)) {
            mac80211_frame *m3 = esp32_wpa2_rx_eapol(&s->security, frame);
            if (m3) {
                memcpy(m3->receiver_address, s->macaddr, 6);
                memcpy(m3->transmitter_address, s->ap_macaddr, 6);
                memcpy(m3->address_3, s->ap_macaddr, 6);
                Esp32_WLAN_init_ap_frame(s, m3);
                Esp32_WLAN_insert_frame(s, m3);
            }
            if (s->security.keys_installed) {
                return;
            }
            return;
        }
        if (s->guest_softap && s->ap_state == Esp32_WLAN__STATE_ASSOCIATED &&
            !frame->frame_control.to_ds && frame->frame_control.from_ds) {
            dhcp_t *dhcp;
            size_t options_length;

            if (is_dhcp_from_ap(frame, &dhcp, &options_length)) {
                uint8_t message_type = dhcp_message_type(dhcp, options_length);

                if (message_type == 2) { /* DHCP OFFER */
                    memcpy(s->ap_ipaddr, frame->data_and_fcs + 8 + 12, 4);
                    memcpy(s->ipaddr, dhcp->yiaddr, 4);
                    mac80211_frame *request =
                        Esp32_WLAN_create_dhcp_request(dhcp->yiaddr);
                    memcpy(request->receiver_address, s->ap_macaddr, 6);
                    memcpy(request->transmitter_address, s->macaddr, 6);
                    memcpy(request->address_3, BROADCAST, 6);
                    Esp32_WLAN_init_ap_frame(s, request);
                    Esp32_WLAN_insert_frame(s, request);
                    return;
                }
                if (message_type == 5) { /* DHCP ACK */
                    memcpy(s->ipaddr, dhcp->yiaddr, 4);
                    s->guest_dhcp_complete = true;
                    return;
                }
                if (message_type == 6) { /* DHCP NAK */
                    s->guest_dhcp_complete = false;
                    memset(s->ipaddr, 0, sizeof(s->ipaddr));
                    return;
                }
            }
        }
        if (s->ap_state == Esp32_WLAN__STATE_STA_ASSOCIATED) {
            size_t ethernet_length;
            if (s->security.configured &&
                (frame->frame_length < IEEE80211_HEADER_SIZE + 2 ||
                 ((uint8_t *)frame)[1] & 0x40) &&
                !esp32_wpa2_decrypt_data(&s->security, frame)) {
                return;
            }
            if (sta_frame_to_ethernet(frame, ethernet_frame,
                                      &ethernet_length)) {
                qemu_send_packet(qemu_get_queue(s->nic), ethernet_frame,
                                 ethernet_length);
            }
        } else if (s->ap_state == Esp32_WLAN__STATE_ASSOCIATED) {
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

            if (s->guest_softap) {
                softap_to_slirp(s, ethernet_frame,
                                ethernet_frame_size + 14);
            }

            // send frame
            qemu_send_packet(qemu_get_queue(s->nic), ethernet_frame, ethernet_frame_size);
        }
    }
}
