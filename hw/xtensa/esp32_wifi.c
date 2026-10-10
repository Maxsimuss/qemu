#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "sysemu/sysemu.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/xtensa/esp32_wifi.h"
#include "hw/misc/esp32_dport.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"
#include "exec/address-spaces.h"
#include "esp32_wlan_packet.h"
#include "hw/qdev-properties.h"
#include "hw/resettable.h"
#include "qemu/bswap.h"

/* MMIO accesses are traced by the shared peripheral trace infrastructure. */
#include "hw/misc/esp32_reg.h"

static uint64_t esp32_wifi_read(void *opaque, hwaddr addr, unsigned int size)
{

    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t r = s->mem[addr/4];

    switch(addr) {
        case A_WIFI_TXRX_INIT_10C:
        case A_WIFI_TXRX_INIT_114:
        case A_WIFI_TXRX_INIT_C1C:
        case A_WIFI_TXRX_INIT_C20:
        case A_WIFI_TXRX_INIT_C24:
        case A_WIFI_TXRX_INIT_C54:
        case A_WIFI_TXRX_INIT_C5C:
        case A_WIFI_TXRX_INIT_C6C:
        case A_WIFI_TXRX_INIT_C74:
        case A_WIFI_TXRX_INIT_C78:
        case A_WIFI_TXRX_INIT_C88:
        case A_WIFI_TXRX_INIT_CAC:
        case A_WIFI_TXRX_INIT_D78:
        case A_WIFI_TXRX_INIT_288:
            qemu_log_mask(LOG_UNIMP, "wifi TXRX INIT read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_ADDR_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_FST_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_ADDR_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_SND_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_ADDR_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_FST_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_ADDR_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_SND_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_FILTER_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_FST_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_FILTER_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_SND_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_FILTER_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_FST_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_BSSID_FILTER_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_SND_1 read %08x\n", (uint32_t) addr);
            break;

        case A_WIFI_MAC_ADDR_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_FST_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_ADDR_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_SND_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_ADDR_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_FST_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_ADDR_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_SND_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_FILTER_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_FST_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_FILTER_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_SND_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_FILTER_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_FST_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAC_FILTER_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_SND_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_RXBUF_INIT_BITMASK:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_RXBUF_INIT_BITMASK read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_DMA_IN_STATUS:
            r=0;
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_IN_STATUS read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_DMA_INLINK:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_INLINK read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_NEXT_RX_DSCR:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_NEXT_RX_DSCR read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_LAST_RX_DSCR:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_LAST_RX_DSCR read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_RX_POLICY_0:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_RX_POLICY_1:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_RX_POLICY_2:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_2 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_RX_POLICY_3:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_3 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK0:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK1:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK1 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK2:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK2 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK3:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK3 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_RXBUF_INIT_HIGH_ADDR_0:
        case A_WIFI_RXBUF_INIT_LOW_ADDR_0:
        case A_WIFI_RXBUF_INIT_HIGH_ADDR_1:
        case A_WIFI_RXBUF_INIT_LOW_ADDR_1:
            qemu_log_mask(LOG_UNIMP, "wifi RXBUF_INIT high and low addresses read (unexpected!) %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_LAST_RXBUF_INIT_09C:
        case A_WIFI_LAST_RXBUF_INIT_148:
        case A_WIFI_LAST_RXBUF_INIT_14C:
        case A_WIFI_LAST_RXBUF_INIT_158:
        case A_WIFI_LAST_RXBUF_INIT_164:
            qemu_log_mask(LOG_UNIMP, "wifi LAST_RXBUF_INIT registers read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_ANTENNA_INIT_284:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_ANTENNA_INIT_284 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_AUTOACK_INIT_400:
        case A_WIFI_AUTOACK_INIT_404:
        case A_WIFI_AUTOACK_INIT_408:
        case A_WIFI_AUTOACK_INIT_40C:
        case A_WIFI_AUTOACK_INIT_410:
        case A_WIFI_AUTOACK_INIT_414:
            qemu_log_mask(LOG_UNIMP, "wifi AUTOACK_INIT registers read (unexpected!) %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_LOW_RATE_418:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_LOW_RATE_418 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_LOW_RATE_41C:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_LOW_RATE_41C read %08x\n", (uint32_t) addr);
            break;
        case 0x800 ... 0x814:
            qemu_log_mask(LOG_UNIMP, "esp32_wifi_read crypto %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAYBE_TIMESTAMP:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_MAYBE_TIMESTAMP read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_PROMISC_CONTROL_PKT:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_CONTROL_PKT read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_DMA_INT_STATUS:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_INT_STATUS read %08x\n", (uint32_t) addr);
            r=s->raw_interrupt;
            break;
        case A_WIFI_DMA_INT_CLR:
            r=s->raw_interrupt;
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_INT_CLR read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_MAYBE_PWR_CTL:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_MAYBE_PWR_CTL read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_TXQ_CLR_STATE_COLL_TIMEOUT:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_CLR_STATE_COLL_TIMEOUT read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_TXQ_STATE_COLL_TIMEOUT:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_STATE_COLL_TIMEOUT read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_TXQ_CLR_STATE_COMPLETE:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_CLR_STATE_COMPLETE read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_TXQ_STATE_COMPLETE:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_STATE_COMPLETE read %08x\n", (uint32_t) addr);
            r=1;
            break;
        case A_WIFI_TX_CONFIG_0:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TX_CONFIG_0 read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_DMA_OUTLINK:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_OUTLINK read %08x\n", (uint32_t) addr);
            break;
        case A_WIFI_DMA_OUT_STATUS:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_OUT_STATUS read %08x\n", (uint32_t) addr);
            r=1;
            break;
        case 0xd30:
        case 0xd38:
        case 0xd40:
            // vm_stop(RUN_STATE_DEBUG)
            qemu_log_mask(LOG_UNIMP, "wifi phy_enable() init registers read %08x\n", (uint32_t) addr);
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "wifi: unimplemented device read %08x\n", (uint32_t) addr + DR_REG_WIFI_BASE);
            break;
    }

    // Stop VM to debug in GDB
    // vm_stop(RUN_STATE_DEBUG);
    return r;
}
static void set_interrupt(Esp32WifiState *s, int e) {
    s->raw_interrupt |= e;
    qemu_set_irq(s->irq, 1);
}

/* ESP32 Wi-Fi DMA accesses internal DRAM (0x3ffae000..0x40000000).  Keep
 * guest-controlled descriptors and buffers out of MMIO, ROM, and unmapped
 * holes even when the generic address space could dispatch such accesses. */
static bool esp32_wifi_dma_range_valid(hwaddr address, size_t length)
{
    const hwaddr dram_start = 0x3ffae000;
    const hwaddr dram_end = 0x40000000;

    return length && address >= dram_start && address < dram_end &&
           length <= dram_end - address;
}

/* 802.11 fields are little-endian on the wire.  The packed bitfields in
 * mac80211_frame are convenient for the existing frame builders, but their
 * byte layout is host-ABI dependent.  Decode wire frames into logical fields
 * before passing them to the WLAN code, and encode those fields explicitly on
 * RX DMA. */
static void wifi_decode_frame_header(mac80211_frame *frame, size_t length)
{
    uint16_t control = lduw_le_p(frame);

    frame->frame_control.protocol_version = control & 0x3;
    frame->frame_control.type = (control >> 2) & 0x3;
    frame->frame_control.sub_type = (control >> 4) & 0xf;
    frame->frame_control.to_ds = (control >> 8) & 1;
    frame->frame_control.from_ds = (control >> 9) & 1;
    frame->frame_control._flags = (control >> 10) & 0x3f;
    frame->duration_id = lduw_le_p((uint8_t *)frame + 2);

    if (length >= IEEE80211_HEADER_SIZE) {
        uint16_t sequence = lduw_le_p((uint8_t *)frame + 22);

        frame->sequence_control.fragment_number = sequence & 0xf;
        frame->sequence_control.sequence_number = sequence >> 4;
    }
}

static void wifi_encode_frame_header(const mac80211_frame *frame,
                                     uint8_t *wire, size_t length)
{
    uint16_t control = frame->frame_control.protocol_version |
                       (frame->frame_control.type << 2) |
                       (frame->frame_control.sub_type << 4) |
                       (frame->frame_control.to_ds << 8) |
                       (frame->frame_control.from_ds << 9) |
                       (frame->frame_control._flags << 10);

    stw_le_p(wire, control);
    stw_le_p(wire + 2, frame->duration_id);
    if (length >= IEEE80211_HEADER_SIZE) {
        uint16_t sequence = frame->sequence_control.fragment_number |
                            (frame->sequence_control.sequence_number << 4);

        stw_le_p(wire + 22, sequence);
    }
}

static uint32_t wifi_desc_control(const dma_list_item *item)
{
    return ldl_le_p(&item->control_le);
}

static unsigned wifi_desc_size(const dma_list_item *item)
{
    return wifi_desc_control(item) & 0xfff;
}

static unsigned wifi_desc_length(const dma_list_item *item)
{
    return (wifi_desc_control(item) >> 12) & 0xfff;
}

static bool wifi_desc_owner(const dma_list_item *item)
{
    return (wifi_desc_control(item) & (1U << 31)) != 0;
}

static bool wifi_desc_eof(const dma_list_item *item)
{
    return (wifi_desc_control(item) & (1U << 30)) != 0;
}

static void wifi_desc_set_rx_result(dma_list_item *item, unsigned length)
{
    uint32_t control = wifi_desc_control(item);

    control = (control & ~((0xfffU << 12) | (1U << 30))) |
              ((length & 0xfffU) << 12) | (1U << 30);
    stl_le_p(&item->control_le, control);
}

static void esp32_wifi_write(void *opaque, hwaddr addr, uint64_t v, unsigned int size) {
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t value = (uint32_t) v;
    (void)size;
    switch (addr) {
        case A_WIFI_TXRX_INIT_10C:
        case A_WIFI_TXRX_INIT_114:
        case A_WIFI_TXRX_INIT_C1C:
        case A_WIFI_TXRX_INIT_C20:
        case A_WIFI_TXRX_INIT_C24:
        case A_WIFI_TXRX_INIT_C54:
        case A_WIFI_TXRX_INIT_C5C:
        case A_WIFI_TXRX_INIT_C6C:
        case A_WIFI_TXRX_INIT_C74:
        case A_WIFI_TXRX_INIT_C78:
        case A_WIFI_TXRX_INIT_C88:
        case A_WIFI_TXRX_INIT_CAC:
        case A_WIFI_TXRX_INIT_D78:

        case A_WIFI_TXRX_INIT_288: // also called in hal_deinit
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXRX_INIT write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_ADDR_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_FST_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_ADDR_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_SND_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_ADDR_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_FST_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_ADDR_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_ADDR_SND_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_FILTER_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_FST_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_FILTER_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_SND_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_FILTER_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_FST_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_BSSID_FILTER_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_BSSID_FILTER_SND_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_ADDR_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_FST_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_ADDR_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_SND_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_ADDR_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_FST_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_ADDR_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_ADDR_SND_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_FILTER_FST_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_FST_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_FILTER_SND_0:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_SND_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_FILTER_FST_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_FST_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAC_FILTER_SND_1:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_MAC_FILTER_SND_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_RXBUF_INIT_BITMASK:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_RXBUF_INIT_BITMASK write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_DMA_IN_STATUS:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_IN_STATUS write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_DMA_INLINK:
            s->dma_inlink_address = value;
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_INLINK write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_NEXT_RX_DSCR:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_NEXT_RX_DSCR write (unexpected!) %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_LAST_RX_DSCR:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_LAST_RX_DSCR write (unexpected!) %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_RX_POLICY_0:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_RX_POLICY_1:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_RX_POLICY_2:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_2 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_RX_POLICY_3:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_RX_POLICY_3 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK0:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK1:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK1 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK2:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK2 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_PROMISC_MISC_BITMASK3:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_MISC_BITMASK3 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_RXBUF_INIT_HIGH_ADDR_0:
        case A_WIFI_RXBUF_INIT_LOW_ADDR_0:
        case A_WIFI_RXBUF_INIT_HIGH_ADDR_1:
        case A_WIFI_RXBUF_INIT_LOW_ADDR_1:
            qemu_log_mask(LOG_UNIMP, "wifi RXBUF_INIT high and low addresses %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_LAST_RXBUF_INIT_09C:
        case A_WIFI_LAST_RXBUF_INIT_148:
        case A_WIFI_LAST_RXBUF_INIT_14C:
        case A_WIFI_LAST_RXBUF_INIT_158:
        case A_WIFI_LAST_RXBUF_INIT_164:
            qemu_log_mask(LOG_UNIMP, "wifi LAST_RXBUF_INIT registers write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_ANTENNA_INIT_284:
            qemu_log_mask(LOG_UNIMP, "wifi WIFI_ANTENNA_INIT_284 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_AUTOACK_INIT_400:
        case A_WIFI_AUTOACK_INIT_404:
        case A_WIFI_AUTOACK_INIT_408:
        case A_WIFI_AUTOACK_INIT_40C:
        case A_WIFI_AUTOACK_INIT_410:
        case A_WIFI_AUTOACK_INIT_414:
            qemu_log_mask(LOG_UNIMP, "wifi AUTOACK_INIT registers write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_LOW_RATE_418:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_LOW_RATE_418 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_LOW_RATE_41C:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_LOW_RATE_41C write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case 0x800 ... 0x814:
            qemu_log_mask(LOG_UNIMP, "esp32_wifi_write crypto %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAYBE_TIMESTAMP:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_MAYBE_TIMESTAMP write (unexpected!) %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_PROMISC_CONTROL_PKT:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_PROMISC_CONTROL_PKT write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_DMA_INT_STATUS:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_INT_STATUS write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_DMA_INT_CLR:
            s->raw_interrupt &= ~value;
            if (s->raw_interrupt == 0)
                qemu_set_irq(s->irq, 0);
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_INT_CLR write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_MAYBE_PWR_CTL:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_MAYBE_PWR_CTL write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_TXQ_CLR_STATE_COLL_TIMEOUT:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_CLR_STATE_COLL_TIMEOUT write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_TXQ_STATE_COLL_TIMEOUT:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_STATE_COLL_TIMEOUT write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_TXQ_CLR_STATE_COMPLETE:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_CLR_STATE_COMPLETE write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_TXQ_STATE_COMPLETE:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TXQ_STATE_COMPLETE write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_TX_CONFIG_0:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_TX_CONFIG_0 write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_DMA_OUTLINK:
            if (value & 0xc0000000) {
                if (!esp32_wifi_mac_enabled(s)) {
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "wifi TX DMA start while MAC clock is disabled or reset is asserted\n");
                    break;
                }
                // do a DMA transfer to the hardware from esp32 memory
                mac80211_frame frame;
                dma_list_item item;
                hwaddr memaddr = (0x3ff00000 | (value & 0xfffff));
                hwaddr frame_address;
                unsigned frame_length;
                unsigned buffer_size;
                MemTxResult result;

                if ((memaddr & 3) ||
                    !esp32_wifi_dma_range_valid(memaddr, sizeof(item))) {
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "wifi TX descriptor address is unaligned: %08x\n",
                                  (uint32_t)memaddr);
                    break;
                }
                result = address_space_read(&address_space_memory, memaddr,
                                            MEMTXATTRS_UNSPECIFIED, &item,
                                            sizeof(item));
                if (result != MEMTX_OK) {
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "wifi TX descriptor read failed at %08x\n",
                                  (uint32_t)memaddr);
                    break;
                }
                frame_address = ldl_le_p(&item.address);
                frame_length = wifi_desc_length(&item);
                buffer_size = wifi_desc_size(&item);
                if (!wifi_desc_owner(&item)) {
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "wifi TX descriptor at %08x is not owned by DMA\n",
                                  (uint32_t)memaddr);
                    break;
                }
                if (frame_length < sizeof(frame.frame_control) ||
                    frame_length > ESP32_WIFI_MAX_FRAME_SIZE ||
                    frame_length > buffer_size ||
                    !esp32_wifi_dma_range_valid(frame_address, frame_length)) {
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "wifi TX descriptor at %08x has invalid length %u (buffer %u)\n",
                                  (uint32_t)memaddr, frame_length,
                                  buffer_size);
                    break;
                }
                result = address_space_read(&address_space_memory,
                                            frame_address,
                                            MEMTXATTRS_UNSPECIFIED, &frame,
                                            frame_length);
                if (result != MEMTX_OK) {
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "wifi TX frame read failed at %08x length %u\n",
                                  (uint32_t)frame_address, frame_length);
                    break;
                }
                wifi_decode_frame_header(&frame, frame_length);
                if ((frame.frame_control.type == IEEE80211_TYPE_CTL &&
                     frame_length < 10) ||
                    (frame.frame_control.type != IEEE80211_TYPE_CTL &&
                     frame_length < IEEE80211_HEADER_SIZE)) {
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "wifi TX frame at %08x is shorter than its 802.11 header (%u bytes)\n",
                                  (uint32_t)frame_address, frame_length);
                    break;
                }
                qemu_log_mask(LOG_UNIMP,
                              "wifi TX: size=%u length=%u eof=%u owner=%u address=%08x next=%08x\n",
                              buffer_size, frame_length, wifi_desc_eof(&item),
                              wifi_desc_owner(&item), (uint32_t)frame_address,
                              ldl_le_p(&item.next));

                // frame from esp32 to ap
                frame.frame_length=frame_length;
                frame.next_frame=0;
                Esp32_WLAN_handle_frame(s, &frame);
                set_interrupt(s, 0x80);
            }
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_OUTLINK write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case A_WIFI_DMA_OUT_STATUS:
            qemu_log_mask(LOG_UNIMP, "wifi A_WIFI_DMA_OUT_STATUS write %08x=%08x\n", (uint32_t) addr, value);
            break;
        case 0xd30:
        case 0xd38:
        case 0xd40:
            qemu_log_mask(LOG_UNIMP, "wifi phy_enable() init registers write %08x=%08x\n", (uint32_t) addr, value);
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "wifi: unimplemented device write %08x = %08x\n", (uint32_t) addr + DR_REG_WIFI_BASE, value);
            break;
    }
    s->mem[addr/4]=value;
}

static int match_mac_address(uint8_t *a1,uint8_t *a2) {
    if(!memcmp(a1,a2,6)) return 1;
    if(!memcmp(a1,BROADCAST,6)) return 1;
    return 0;
}

// frame from QEMU to ESP32
void Esp32_sendFrame(Esp32WifiState *s, mac80211_frame *frame, int length, int signal_strength) {
    uint8_t header[28 + ESP32_WIFI_MAX_FRAME_SIZE];
    size_t dma_length;
    dma_list_item item;
    MemTxResult result;

    if (!esp32_wifi_mac_enabled(s) || s->dma_inlink_address == 0) {
        return;
    }
    if (!frame || length < 10 || length > ESP32_WIFI_MAX_FRAME_SIZE ||
        frame->frame_length != length ||
        ((frame->frame_control.type == IEEE80211_TYPE_CTL && length < 10) ||
         (frame->frame_control.type != IEEE80211_TYPE_CTL &&
          length < IEEE80211_HEADER_SIZE))) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wifi RX frame has invalid length %d (frame length %u)\n",
                      length, frame ? frame->frame_length : 0);
        return;
    }
    wifi_pkt_rx_ctrl_t *pkt=(wifi_pkt_rx_ctrl_t *)header;
    *pkt=(wifi_pkt_rx_ctrl_t){
        .rssi=(signal_strength+(rand()%10)+96),
        .rate=11,
        .sig_len=length,
        .sig_len_copy=length,
        .legacy_length=length,
        .noise_floor=-97,
        /* In SoftAP mode this is taken from the channel IE in the firmware's
         * beacon. The station path still uses the analog-I2C-derived value. */
        .channel=s->guest_softap && s->guest_channel ? s->guest_channel :
                 esp32_wifi_channel,
        .timestamp=qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1000,
    };
    // These 4 bits are set if the mac addresses previously stored at 0x40 and 0x48
    // match the destination or bssid addresses in the frame
    if(match_mac_address(frame->receiver_address,(uint8_t *)s->mem+0x40))
        pkt->damatch0=1;
    if(match_mac_address(frame->receiver_address,(uint8_t *)s->mem+0x48))
        pkt->damatch1=1;
    if(match_mac_address(frame->address_3,(uint8_t *)s->mem+0x40))
        pkt->bssidmatch0=1;
    if(match_mac_address(frame->address_3,(uint8_t *)s->mem+0x48))
        pkt->bssidmatch1=1;
    //printf("...%x %x\n",header[3],frame->receiver_address[0]);

    memcpy(header+28, frame, length);
    wifi_encode_frame_header(frame, header + 28, length);
    dma_length = 28 + length;
    // do a DMA transfer from the hardware to esp32 memory
    if (!esp32_wifi_dma_range_valid(s->dma_inlink_address, sizeof(item))) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wifi RX descriptor address is outside internal DRAM: %08x\n",
                      (uint32_t)s->dma_inlink_address);
        return;
    }
    result = address_space_read(&address_space_memory, s->dma_inlink_address,
                                MEMTXATTRS_UNSPECIFIED, &item,
                                sizeof(item));
    if (result != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wifi RX descriptor read failed at %08x\n",
                      (uint32_t)s->dma_inlink_address);
        return;
    }
    if (!wifi_desc_owner(&item) || wifi_desc_size(&item) < dma_length ||
        dma_length > 0xfff ||
        !esp32_wifi_dma_range_valid(ldl_le_p(&item.address), dma_length)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wifi RX descriptor at %08x cannot hold frame length %zu (buffer %u, owner %u)\n",
                      (uint32_t)s->dma_inlink_address, dma_length,
                      wifi_desc_size(&item), wifi_desc_owner(&item));
        return;
    }
    result = address_space_write(&address_space_memory,
                                 ldl_le_p(&item.address),
                                 MEMTXATTRS_UNSPECIFIED, header, dma_length);
    if (result != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wifi RX frame write failed at %08x length %zu\n",
                      ldl_le_p(&item.address), dma_length);
        return;
    }
    wifi_desc_set_rx_result(&item, dma_length);
    result = address_space_write(&address_space_memory,
                                 s->dma_inlink_address,
                                 MEMTXATTRS_UNSPECIFIED, &item,
                                 sizeof(item.control_le));
    if (result != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wifi RX descriptor completion write failed at %08x\n",
                      (uint32_t)s->dma_inlink_address);
        return;
    }
    s->dma_inlink_address=ldl_le_p(&item.next);
    set_interrupt(s, 0x1000024);
}

static const MemoryRegionOps esp32_wifi_ops = {
    .read =  esp32_wifi_read,
    .write = esp32_wifi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static void esp32_wifi_reset_hold(Object *obj, ResetType type)
{
    Esp32WifiState *s = ESP32_WIFI(obj);

    (void)type;
    memset(s->mem, 0, sizeof(s->mem));
    s->raw_interrupt = 0;
    s->dma_inlink_address = 0;
    s->receive_queue_address = 0;
    s->receive_queue_count = 0;
    qemu_set_irq(s->irq, 0);
    Esp32_WLAN_reset(s);
}

bool esp32_wifi_mac_enabled(const Esp32WifiState *s)
{
    return s->mac_clock_enabled && !s->mac_reset_asserted;
}

static void esp32_wifi_clock_input(void *opaque, int n, int level)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t old = s->wifi_clock_en;

    s->wifi_clock_en = (old & ~(1U << n)) | ((uint32_t)!!level << n);
    s->mac_clock_enabled =
        (s->wifi_clock_en & ESP32_WIFI_MAC_CLOCK_MASK) ==
        ESP32_WIFI_MAC_CLOCK_MASK;
    if (s->mac_clock_enabled !=
        ((old & ESP32_WIFI_MAC_CLOCK_MASK) == ESP32_WIFI_MAC_CLOCK_MASK)) {
        Esp32_WLAN_clock_changed(s);
    }
}

static void esp32_wifi_core_reset_input(void *opaque, int n, int level)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t old = s->core_reset_en;
    bool old_asserted;

    s->core_reset_en = (old & ~(1U << n)) | ((uint32_t)!!level << n);
    if (n != ESP32_WIFI_MAC_RESET_BIT) {
        return;
    }
    old_asserted = s->mac_reset_asserted;
    s->mac_reset_asserted = level != 0;
    if (s->mac_reset_asserted && !old_asserted) {
        memset(s->mem, 0, sizeof(s->mem));
        s->raw_interrupt = 0;
        s->dma_inlink_address = 0;
        s->receive_queue_address = 0;
        s->receive_queue_count = 0;
        qemu_set_irq(s->irq, 0);
        Esp32_WLAN_reset(s);
    }
    if (s->mac_reset_asserted != old_asserted) {
        Esp32_WLAN_clock_changed(s);
    }
}

static void esp32_wifi_realize(DeviceState *dev, Error **errp)
{
    Esp32WifiState *s = ESP32_WIFI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (s->peer_ssid && (!*s->peer_ssid || strlen(s->peer_ssid) > 32)) {
        error_setg(errp, "ESP32 Wi-Fi peer SSID must contain 1 to 32 bytes");
        return;
    }
    if (s->peer_channel < 1 || s->peer_channel > 13) {
        error_setg(errp, "ESP32 Wi-Fi peer channel must be in range 1 to 13");
        return;
    }
    if (s->peer_password &&
        (strlen(s->peer_password) < 8 || strlen(s->peer_password) > 63)) {
        error_setg(errp, "ESP32 Wi-Fi peer password must contain 8 to 63 bytes");
        return;
    }
    s->dma_inlink_address = 0;

    memory_region_init_io(&s->iomem, OBJECT(dev), &esp32_wifi_ops, s,
                          TYPE_ESP32_WIFI, ESP32_WIFI_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(dev, esp32_wifi_clock_input,
                            ESP32_WIFI_CLOCK_GPIO, 32);
    qdev_init_gpio_in_named(dev, esp32_wifi_core_reset_input,
                            ESP32_WIFI_RESET_GPIO, 32);
    memset(s->mem,0,sizeof(s->mem));
    Esp32_WLAN_setup_ap(dev, s);

}

static void esp32_wifi_unrealize(DeviceState *dev)
{
    Esp32WifiState *s = ESP32_WIFI(dev);

    Esp32_WLAN_cleanup(s);
    if (s->nic) {
        qemu_del_nic(s->nic);
        s->nic = NULL;
    }
}

static Property esp32_wifi_properties[] = {
    DEFINE_NIC_PROPERTIES(Esp32WifiState, conf),
    DEFINE_PROP_STRING("peer-ssid", Esp32WifiState, peer_ssid),
    DEFINE_PROP_STRING("peer-password", Esp32WifiState, peer_password),
    DEFINE_PROP_UINT8("peer-channel", Esp32WifiState, peer_channel, 1),
    DEFINE_PROP_END_OF_LIST(),
};
static void esp32_wifi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32_wifi_realize;
    dc->unrealize = esp32_wifi_unrealize;
    rc->phases.hold = esp32_wifi_reset_hold;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->desc = "Esp32 WiFi";
    device_class_set_props(dc, esp32_wifi_properties);
}


static const TypeInfo esp32_wifi_info = {
    .name = TYPE_ESP32_WIFI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32WifiState),
    .class_init    = esp32_wifi_class_init,
};

static void esp32_wifi_register_types(void)
{
    type_register_static(&esp32_wifi_info);
}

type_init(esp32_wifi_register_types)
