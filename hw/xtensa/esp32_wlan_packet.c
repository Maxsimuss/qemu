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
#include <zlib.h>

#include "net/net.h"

#include "esp32_wlan.h"
#include "esp32_wlan_packet.h"

#define DHCP_MIN_OPTIONS_LEN 68

void insertCRC(mac80211_frame *frame) {
    uint32_t crc;

    /* Peer-generated 802.11 frames carry the IEEE 802.11 FCS.  Chip TX
     * frames are handled separately by the guest's MAC offload path. */
    if (!frame || frame->frame_length < IEEE80211_HEADER_SIZE ||
        frame->frame_length > IEEE80211_HEADER_SIZE + 2312) {
        return;
    }
    /* zlib's API applies the conventional initial/final XOR internally for
     * a zero seed, producing the IEEE 802.11 FCS value. */
    crc = crc32(0, (const Bytef *)frame, frame->frame_length);
    stl_le_p((uint8_t *)frame + frame->frame_length, crc);
    frame->frame_length += 4;
}

static void add_byte(mac80211_frame *frame, uint8_t data) {
    frame->data_and_fcs[frame->pos++]=data;
    frame->frame_length=IEEE80211_HEADER_SIZE+frame->pos;
}

static void add_data(mac80211_frame *frame, int len,uint8_t *data) {
    for(int i=0;i<len;i++)
        add_byte(frame,data[i]);
}

static void add_tag(mac80211_frame *frame, int tag, int len, unsigned char bytes[]) {
    add_byte(frame,tag);
    add_byte(frame,len);
    add_data(frame,len,bytes);
}

void Esp32_WLAN_init_ap_frame(Esp32WifiState *s, mac80211_frame *frame) {
    frame->sequence_control.sequence_number = s->inject_sequence_number++;
}

static mac80211_frame *new_frame(unsigned type, unsigned subtype) {
    mac80211_frame *frame = g_new0(mac80211_frame, 1);
    frame->next_frame = NULL;
    frame->frame_control.protocol_version = 0;
    frame->frame_control.type = type;
    frame->frame_control.sub_type = subtype;
    frame->frame_control._flags = 0;
    frame->frame_control.from_ds = 0;
    frame->frame_control.to_ds = 0;
    frame->signal_strength = -40;
    frame->duration_id = 314;
    frame->sequence_control.fragment_number = 0;
    frame->pos=0;
    return frame;
}

static void add_rates(mac80211_frame *frame) {
    add_tag(frame,IEEE80211_BEACON_PARAM_RATES,8,(uint8_t[]){0x8b,0x96,0x82,0x84,0x0c,0x18,0x30,0x60});
    add_tag(frame,IEEE80211_BEACON_PARAM_EXTENDED_RATES,4,(uint8_t[]){0x6c,0x12,0x24,0x48});
}

static void add_ssid(mac80211_frame *frame, const char *ssid) {
    add_tag(frame,IEEE80211_BEACON_PARAM_SSID,strlen(ssid),(uint8_t *)ssid);
}

static void add_rsn_psk_ccmp(mac80211_frame *frame)
{
    static uint8_t rsn[] = {
        1,0, 0,0x0f,0xac,4, 1,0, 0,0x0f,0xac,4,
        1,0, 0,0x0f,0xac,2, 0,0
    };
    add_tag(frame, 48, sizeof(rsn), rsn);
}

mac80211_frame *Esp32_WLAN_create_beacon_frame(access_point_info *ap) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_BEACON);
    frame->signal_strength=ap->sigstrength;
    frame->beacon_info.timestamp=qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1000;
    frame->beacon_info.interval=1000;
    frame->beacon_info.capability=ap->wpa2 ? 0x11 : 1;
    frame->pos=12;
    add_ssid(frame,ap->ssid);
    add_rates(frame);
    add_tag(frame,IEEE80211_BEACON_PARAM_CHANNEL,1,(uint8_t[]){ap->channel});
    add_tag(frame,IEEE80211_BEACON_PARAM_TIM,4,(uint8_t[]){4,1,3,0,0});
    if (ap->wpa2) add_rsn_psk_ccmp(frame);
    return frame;
}

static uint16_t in_cksum(uint16_t *addr, int len) {
    int sum = 0;
    uint16_t answer = 0;
    uint16_t *w = addr;
    int nleft = len;
    while (nleft > 1) {
        sum += *w++;
        nleft -= 2;
    }
    /* mop up an odd byte, if necessary */
    if (nleft == 1) {
        *(uint8_t *)(&answer) = *(uint8_t *) w;
        sum += answer;
    }
    sum = (sum >> 16) + (sum & 0xffff);     /* add hi 16 to low 16 */
    sum += (sum >> 16);             /* add carry */
    answer = ~sum;              /* truncate to 16 bits */
    return (answer);
}

static mac80211_frame *Esp32_WLAN_create_dhcp_frame(
    const dhcp_t *bootp, int cmd_size, const uint8_t dhcp_commands[])
{
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_DATA,IEEE80211_TYPE_DATA_SUBTYPE_DATA);
    frame->frame_control.to_ds=1;
    add_data(frame,8,(uint8_t[]){ 0xaa, 0xaa ,0x03 ,00 ,00 ,00 ,8 ,00});
    dhcp_request_t req = {
        .ipheader = {
            .version_size = 0x45, .ttl = 0xff, .protocol = 0x11,
            .dest_ip = {0xff,0xff,0xff,0xff},
        },
        .udpheader = {.src_port_l = 0x44, .dest_port_l = 0x43},
    };
    memcpy(&req.dhcp, bootp, offsetof(dhcp_t, bp_options));
    int len=sizeof(req)+cmd_size;
    req.ipheader.len_h=len>>8;
    req.ipheader.len_l=len&0xff;
    len=len-sizeof(ip_header_t);
    req.udpheader.len_h=len>>8;
    req.udpheader.len_l=len&0xff;
    req.ipheader.checksum=in_cksum((void *)&req.ipheader,sizeof(ip_header_t));
    add_data(frame,sizeof(req),(uint8_t *)&req);
    add_data(frame,cmd_size,(uint8_t *)dhcp_commands);
    return frame;
}

mac80211_frame *Esp32_WLAN_create_dhcp_request(const dhcp_t *offer,
                                              const uint8_t server_id[4]) {
    uint8_t dhcp_commands[DHCP_MIN_OPTIONS_LEN] = { 0 };
    size_t option = 0;
    dhcp_t request = {0};

    request.opcode = 1;
    request.htype = offer->htype;
    request.hlen = offer->hlen;
    /* These are network-order fields; retain their wire bytes rather than
     * load/store them through the host-endian integer representation. */
    memcpy(&request.xid, &offer->xid, sizeof(request.xid));
    memcpy(&request.secs, &offer->secs, sizeof(request.secs));
    /* The station has no configured IPv4 address yet and cannot receive a
     * unicast ACK at yiaddr.  Its BOOTP broadcast bit is client capability,
     * not a value to copy from the server's OFFER. */
    memcpy(&request.flags, (const uint8_t[]){ 0x80, 0x00 },
           sizeof(request.flags));
    memcpy(request.chaddr, offer->chaddr, offer->hlen);
    memcpy(&request.magic_cookie,
           (const uint8_t[]){0x63,0x82,0x53,0x63}, 4);

    /* Build the station's Selecting-state DHCPREQUEST from the received
     * OFFER so transaction identity and client hardware address match. */
    dhcp_commands[option++] = 53;
    dhcp_commands[option++] = 1;
    dhcp_commands[option++] = 3;
    dhcp_commands[option++] = 57;
    dhcp_commands[option++] = 2;
    dhcp_commands[option++] = 5;
    dhcp_commands[option++] = 0xdc;
    dhcp_commands[option++] = 50;
    dhcp_commands[option++] = 4;
    memcpy(dhcp_commands + option, offer->yiaddr, 4);
    option += 4;
    dhcp_commands[option++] = 54;
    dhcp_commands[option++] = 4;
    memcpy(dhcp_commands + option, server_id, 4);
    option += 4;
    dhcp_commands[option++] = 61;
    dhcp_commands[option++] = 7;
    dhcp_commands[option++] = 1;
    memcpy(dhcp_commands + option, offer->chaddr, 6);
    option += 6;
    dhcp_commands[option++] = 55;
    dhcp_commands[option++] = 4;
    memcpy(dhcp_commands + option, (uint8_t[]){1,3,28,6}, 4);
    option += 4;
    dhcp_commands[option++] = 255;
    /* lwIP's DHCP client pads the options field to its RFC minimum of 68
     * bytes, yielding a 308-byte BOOTP/DHCP payload including the cookie. */
    option = MAX(option, sizeof(dhcp_commands));
    return Esp32_WLAN_create_dhcp_frame(&request, option,
                                        dhcp_commands);
}

mac80211_frame *Esp32_WLAN_create_dhcp_discover(
    const uint8_t client_mac[6], const uint8_t xid[4])
{
    uint8_t dhcp_commands[DHCP_MIN_OPTIONS_LEN] = {
        0x35, 1, 1,
        0x39, 2, 5, 0xdc,
        0x0c, 0x09, 'e', 's', 'p', 'r', 'e', 's', 's', 'i', 'f',
        0x3d, 0x07, 0x01, client_mac[0], client_mac[1], client_mac[2],
        client_mac[3], client_mac[4], client_mac[5],
        0x37, 0x04, 0x01, 0x03, 0x1c, 0x06,
        0xff,
    };
    dhcp_t discover = { .opcode = 1, .htype = 1, .hlen = 6 };

    memcpy(&discover.xid, xid, sizeof(discover.xid));
    memcpy(&discover.flags, (const uint8_t[]){ 0x80, 0x00 },
           sizeof(discover.flags));
    memcpy(discover.chaddr, client_mac, 6);
    memcpy(&discover.magic_cookie,
           (const uint8_t[]){0x63,0x82,0x53,0x63}, 4);
    return Esp32_WLAN_create_dhcp_frame(&discover, sizeof(dhcp_commands),
                                        dhcp_commands);
}

mac80211_frame *Esp32_WLAN_create_association_request(access_point_info *ap) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_REQ);
    add_data(frame,4,(uint8_t []){0x21,4,3,0});
    add_ssid(frame,ap->ssid);
    add_rates(frame);
    return frame;
}

mac80211_frame *Esp32_WLAN_create_ack(void) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_CTL,IEEE80211_TYPE_CTL_SUBTYPE_ACK);
    frame->frame_length=10;
    return frame;
}

mac80211_frame *Esp32_WLAN_create_probe_response(access_point_info *ap) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_PROBE_RESP);
    frame->signal_strength=ap->sigstrength;
    frame->beacon_info.timestamp=qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1000;
    frame->beacon_info.interval=1000;
    frame->beacon_info.capability=ap->wpa2 ? 0x11 : 1;
    frame->pos=12;
    add_ssid(frame,ap->ssid);
    add_rates(frame);
    add_tag(frame,IEEE80211_BEACON_PARAM_CHANNEL,1,(uint8_t[]){ap->channel});
    if (ap->wpa2) add_rsn_psk_ccmp(frame);
    return frame;
}

mac80211_frame *Esp32_WLAN_create_probe_request(access_point_info *ap) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_PROBE_REQ);
    memcpy(frame->receiver_address,BROADCAST,6);
    memcpy(frame->address_3,BROADCAST,6);
    add_ssid(frame,ap->ssid);
    add_tag(frame,IEEE80211_BEACON_PARAM_CHANNEL,1,(uint8_t[]){ap->channel});
    add_rates(frame);
    return frame;
}

mac80211_frame *Esp32_WLAN_create_authentication_response(access_point_info *ap) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_AUTHENTICATION);
    frame->signal_strength=ap->sigstrength;
    /*
     * Fixed params... typical AP params (6 byte)
     *
     * They include
     *  - Authentication Algorithm (here: Open System)
     *  - Authentication SEQ
     *  - Status code (successful 0x0)
    */
    add_data(frame,6,(uint8_t []){0,0,2,0,0,0});
    /* IEEE 802.11 Authentication carries only the six fixed bytes above;
     * the SSID belongs in probe/beacon frames, not this response. */
    return frame;
}

mac80211_frame *Esp32_WLAN_create_authentication_request(void) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_AUTHENTICATION);
    /*
     * Fixed params... typical AP params (6 byte)
     *
     * They include
     *  - Authentication Algorithm (here: Open System)
     *  - Authentication SEQ
     *  - Status code (successful 0x0)
     */
    //frame->duration_id=0x13a;
    add_data(frame,6,(uint8_t []){0,0,1,0,0,0});
    //add_data(frame,11,(uint8_t []){0,0,0,0,0,0,0,0,0,0,0});
    //add_ssid(frame,ap->ssid);
    return frame;
}

mac80211_frame *Esp32_WLAN_create_deauthentication(void) {
    return Esp32_WLAN_create_deauthentication_reason(3);
}

mac80211_frame *Esp32_WLAN_create_deauthentication_reason(uint16_t reason)
{
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_DEAUTHENTICATION);
    add_data(frame, 2, (uint8_t[]){reason, reason >> 8});
    return frame;
}

mac80211_frame *Esp32_WLAN_create_association_response(access_point_info *ap) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_ASSOCIATION_RESP);
    frame->signal_strength=ap->sigstrength;
    /*
     * Fixed params... typical AP params (6 byte)
     *
     * They include
     *  - Capability Information
     *  - Status code (successful 0x0)
     *  - Association ID
    */
    add_data(frame,6,(uint8_t []){33,4,0,0,1,0xc0});
    add_ssid(frame,ap->ssid);
    add_rates(frame);
    return frame;
}

mac80211_frame *Esp32_WLAN_create_disassociation(void) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_MGT,IEEE80211_TYPE_MGT_SUBTYPE_DISASSOCIATION);
    /*
     * Insert reason code:
     *  "Disassociation because sending STA is leaving"
     */
    add_data(frame,2,(uint8_t []){3,0});
    return frame;
}

mac80211_frame *Esp32_WLAN_create_data_packet(Esp32WifiState *s, const uint8_t *buf, int size) {
    mac80211_frame *frame=new_frame(IEEE80211_TYPE_DATA,IEEE80211_TYPE_DATA_SUBTYPE_DATA);

    frame->duration_id = 44;
    /* LLC */
    add_data(frame,6,(uint8_t[]){ 0xaa, 0xaa ,0x03 ,0 ,0 ,0});
    memcpy(frame->data_and_fcs+6, buf+12, size-12);
    frame->frame_length = IEEE80211_HEADER_SIZE + size-6;
    return frame;
}
