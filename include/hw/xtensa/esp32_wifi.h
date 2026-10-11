/* SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include "hw/hw.h"
#include "qemu/timer.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_reg.h"
#include "sysemu/sysemu.h"
#include "net/net.h"
#include "hw/xtensa/esp32_wifi_security.h"

#define TYPE_ESP32_WIFI "misc.esp32_wifi"
#define ESP32_WIFI(obj) OBJECT_CHECK(Esp32WifiState, (obj), TYPE_ESP32_WIFI)

/* The Espressif-supplied IDF 6.1 image accesses key slots at offset 0x1400,
 * Wi-Fi interrupt registers near 0x1c48, and TSF control at 0x2010..0x20a0.
 * Keep the observed register aperture backed even where behavior is not yet
 * modeled. */
#define ESP32_WIFI_MMIO_SIZE 0x2100
#define ESP32_WIFI_MAX_FRAME_SIZE (24 + 2316)

/* The supplied ESP32 libpp `hal_get_tsf_time()` helper accesses WDEV TSF
 * latch at 0x3ff75010 and snapshots two 64-bit microsecond counters at
 * 0x3ff75014/18 and 0x3ff75054/58. These addresses are within the Wi-Fi
 * aperture at offsets 0x2000..0x2058. */
#define ESP32_WIFI_WDEV_TSF_CTRL 0x2010
#define ESP32_WIFI_WDEV_TSF0_LO  0x2014
#define ESP32_WIFI_WDEV_TSF0_HI  0x2018
#define ESP32_WIFI_WDEV_TSF1_LO  0x2054
#define ESP32_WIFI_WDEV_TSF1_HI  0x2058
#define ESP32_WIFI_WDEV_TSF0_LATCH (1U << 0)
#define ESP32_WIFI_WDEV_TSF1_LATCH (1U << 1)

/* Private ESP32 RX-control MMIO, independently observed in the IDF PHY ELF. */
#define ESP32_WIFI_RXCTRL_BASE 0x3ff5c000
#define ESP32_WIFI_RXCTRL_SIZE 0x2000
#define ESP32_WIFI_RXCTRL_AGC_GAIN 0x01c
#define ESP32_WIFI_RXCTRL_AGC_MODE 0x030
#define ESP32_WIFI_RXCTRL_AGC_STATE 0x038
#define ESP32_WIFI_RXCTRL_AGC_ENABLE 0x080
#define ESP32_WIFI_RXCTRL_RFPLL_MODE 0x1008
#define ESP32_WIFI_RXCTRL_COEX_AGC 0x1080
#define ESP32_WIFI_RXCTRL_COEX_RFPLL 0x1040

/* Sparse RF front-end registers touched by the supplied IDF PHY's
 * force_txrxoff()/fe_reg_init() paths. */
#define ESP32_WIFI_FE_BASE 0x3ff46000
#define ESP32_WIFI_FE_SIZE 0x1000
#define ESP32_WIFI_FE2_BASE 0x3ff45000
#define ESP32_WIFI_FE2_SIZE 0x1000
#define ESP32_WIFI_TXDC_PBUS_BASE 0x3ff4e04c
#define ESP32_WIFI_TXDC_PBUS_APB 0x6000e04c
/* Private SENS control used by IDF phy_chip_v7_ana.c's
 * set_chan_freq_sw_start() path. */
#define ESP32_WIFI_RFPLL_FREQ_BASE 0x3ff4e0c4
#define ESP32_WIFI_RFPLL_FREQ_APB 0x6000e0c4
#define ESP32_WIFI_RFPLL_TUNE_GPIO "rfpll-tune"
#define ESP32_WIFI_RFPLL_TUNE_GPIO_COUNT 9
#define ESP32_WIFI_PHY_BT_IFS_ADDR 0x3ff5103c
#define ESP32_WIFI_COEX_APB_BASE 0x600310d0
#define ESP32_WIFI_COEX_WORD_COUNT 5
#define ESP32_WIFI_FE_CTRL_030 0x030
#define ESP32_WIFI_FE_CTRL_034 0x034
#define ESP32_WIFI_FE_CTRL_038 0x038
#define ESP32_WIFI_FE_CTRL_03C 0x03c
#define ESP32_WIFI_FE_CTRL_040 0x040
#define ESP32_WIFI_FE_CTRL_044 0x044
#define ESP32_WIFI_FE_CTRL_060 0x060
#define ESP32_WIFI_FE_IQ_EST 0x07c
#define ESP32_WIFI_FE_TXRX_CONTROL 0x0a0
#define ESP32_WIFI_FE_TXRX_OTHER 0x0b8
#define ESP32_WIFI_FE_CTRL_09C 0x09c
#define ESP32_WIFI_FE_PBUS_CMD 0x094
#define ESP32_WIFI_FE_CTRL_04C 0x04c
#define ESP32_WIFI_FE2_CTRL_114 0x114
#define ESP32_WIFI_FE2_CTRL_034 0x034
#define ESP32_WIFI_FE2_CTRL_038 0x038
#define ESP32_WIFI_FE2_CTRL_0DC 0x0dc
#define ESP32_WIFI_FE2_CTRL_0D8 0x0d8

/* The supplied IDF 6.1 ESP32 libpp ELF's hal_crypto_set_key_entry() writes
 * a record with 40-byte stride at 0x3ff74400 + index * 40: two metadata
 * words followed by key bytes at offset 8. It RMWs the validity bitmap at
 * 0x3ff73814.
 * These are within the MAC aperture at offsets 0x1400 and 0x814. Keep the
 * register backing; this alone does not implement crypto-engine behavior. */
#define ESP32_WIFI_CRYPTO_KEY_TABLE 0x1400
#define ESP32_WIFI_CRYPTO_KEY_VALID 0x0814

/* ESP-IDF ESP32 DPORT definitions: DPORT_WIFI_CLK_WIFI_EN (0x406) clocks
 * the Wi-Fi MAC; DPORT_WIFIMAC_RST (bit 2) is active high in CORE_RST_EN. */
#define ESP32_WIFI_MAC_CLOCK_MASK 0x00000406
#define ESP32_WIFI_MAC_RESET_BIT 2
#define ESP32_WIFI_FE_RESET_BIT 1
#define ESP32_WIFI_CLOCK_GPIO "wifi-clock"
#define ESP32_WIFI_RESET_GPIO "core-reset"

typedef struct dma_list_item {
    uint32_t control_le;
    uint32_t address;
    uint32_t next;
} QEMU_PACKED dma_list_item;

typedef struct Esp32WifiState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    MemoryRegion rxctrl_iomem;
    MemoryRegion fe_iomem;
    MemoryRegion fe2_iomem;
    MemoryRegion txdc_pbus_iomem;
    MemoryRegion rfpll_freq_iomem;
    MemoryRegion phy_bt_iomem;
    MemoryRegion phy_coex_iomem[ESP32_WIFI_COEX_WORD_COUNT];
    int raw_interrupt;
    qemu_irq irq;
    uint32_t mem[ESP32_WIFI_MMIO_SIZE / sizeof(uint32_t)];
    uint32_t wdev_tsf_latch;
    uint64_t wdev_tsf_elapsed_us;
    uint64_t wdev_tsf_latched_us[2];
    int64_t wdev_tsf_epoch_ns;
    bool wdev_tsf_running;
    uint32_t rxctrl_agc_gain;
    uint32_t rxctrl_agc_mode;
    uint32_t rxctrl_agc_state;
    uint32_t rxctrl_agc_enable;
    uint32_t rxctrl_rfpll_mode;
    uint32_t rxctrl_coex_agc;
    uint32_t rxctrl_coex_rfpll;
    uint32_t fe_regs[13];
    uint32_t fe2_regs[5];
    QEMUTimer *fe_pbus_timer;
    bool fe_pbus_busy;
    uint16_t fe_pbus_test_candidate[4];
    uint8_t fe_pbus_test_index[4];
    bool fe_pbus_test_valid[4];
    QEMUTimer *fe_iq_timer;
    bool fe_iq_ready;
    QEMUTimer *txdc_pbus_timer;
    uint32_t txdc_pbus_status;
    uint32_t txdc_pbus_command;
    bool txdc_pbus_pending;
    QEMUTimer *rfpll_freq_timer;
    uint32_t rfpll_freq_value;
    bool rfpll_freq_pending;
    qemu_irq rfpll_tune_out[ESP32_WIFI_RFPLL_TUNE_GPIO_COUNT];
    uint32_t phy_bt_ifs;
    uint32_t phy_coex_regs[ESP32_WIFI_COEX_WORD_COUNT];
    uint64_t crypto_rx_pn[32];
    uint64_t crypto_tx_pn[32];
    uint32_t wifi_clock_en;
    uint32_t core_reset_en;
    bool mac_clock_enabled;
    bool mac_reset_asserted;
    hwaddr dma_inlink_address;
    uint32_t ap_state;
    int inject_queue_size;
    struct mac80211_frame *inject_queue;
    int inject_timer_running;
    unsigned int inject_sequence_number;
    int beacon_ap;

    hwaddr receive_queue_address;
    uint32_t receive_queue_count;
    NICConf conf;
    NICState *nic;
    /* Optional external AP used by STA-mode protocol tests. */
    char *peer_ssid;
    char *peer_password;
    uint8_t peer_channel;
    /* RF channel selected by a completed, supported C4 tune operation. */
    uint8_t rf_channel;
    Esp32Wpa2Peer security;
    // various timers
    QEMUTimer *beacon_timer;
    QEMUTimer *inject_timer;
    QEMUTimer *wpa_retry_timer;
    mac80211_frame *wpa_retry_frame;
    uint8_t wpa_retry_count;
    uint64_t wpa_retry_interval_ns;
    uint8_t ipaddr[4];
    uint8_t macaddr[6];

    uint8_t ap_ipaddr[4];
    uint8_t ap_macaddr[6];
    /* BSSID advertised by the guest while it is operating as a SoftAP.
     * Keep it separate from the configured external AP used in STA mode. */
    uint8_t guest_ap_macaddr[6];

    uint8_t associated_ap_macaddr[6];
    /* The emulated NIC peer has authenticated to the guest's SoftAP. */
    bool guest_softap;
    /* The emulated NIC peer has completed association to the guest SoftAP.
     * This is independent of the guest's own STA association in ap_state. */
    bool guest_ap_associated;
    /* Lease learned by the external virtual station from the guest's DHCP ACK. */
    bool guest_dhcp_complete;
    bool guest_dhcp_xid_active;
    uint8_t guest_dhcp_xid[4];
    uint8_t guest_dhcp_server_id[4];
    uint8_t guest_dhcp_offered_ip[4];
    char guest_ssid[33];
    uint8_t guest_channel;

} Esp32WifiState;


void Esp32_WLAN_handle_frame(Esp32WifiState *s, struct mac80211_frame *frame);
void Esp32_WLAN_setup_ap(DeviceState *dev,Esp32WifiState *s);
void Esp32_WLAN_reset(Esp32WifiState *s);
void Esp32_WLAN_cleanup(Esp32WifiState *s);
bool esp32_wifi_mac_enabled(const Esp32WifiState *s);
bool esp32_wifi_mac_ccmp(Esp32WifiState *s, mac80211_frame *frame,
                         bool encrypt, uint8_t interface_id);
void Esp32_WLAN_clock_changed(Esp32WifiState *s);
void Esp32_sendFrame(Esp32WifiState *s, struct mac80211_frame *frame,int length, int signal_strength);

REG32(WIFI_BSSID_ADDR_FST_0, 0x000);
REG32(WIFI_BSSID_ADDR_SND_0, 0x004);
REG32(WIFI_BSSID_ADDR_FST_1, 0x008);
REG32(WIFI_BSSID_ADDR_SND_1, 0x00c);
REG32(WIFI_BSSID_FILTER_FST_0, 0x020);
REG32(WIFI_BSSID_FILTER_SND_0, 0x024);
REG32(WIFI_BSSID_FILTER_FST_1, 0x028);
REG32(WIFI_BSSID_FILTER_SND_1, 0x02c);

REG32(WIFI_MAC_ADDR_FST_0, 0x040);
REG32(WIFI_MAC_ADDR_SND_0, 0x044);
REG32(WIFI_MAC_ADDR_FST_1, 0x048);
REG32(WIFI_MAC_ADDR_SND_1, 0x04c);
REG32(WIFI_MAC_FILTER_FST_0, 0x060);
REG32(WIFI_MAC_FILTER_SND_0, 0x064);
REG32(WIFI_MAC_FILTER_FST_1, 0x068);
REG32(WIFI_MAC_FILTER_SND_1, 0x06c);

REG32(WIFI_RXBUF_INIT_BITMASK, 0x80); // only set in mac_rxbuf_init
REG32(WIFI_DMA_IN_STATUS, 0x84);
REG32(WIFI_DMA_INLINK, 0x88);
REG32(WIFI_NEXT_RX_DSCR, 0x8c);
REG32(WIFI_LAST_RX_DSCR, 0x90);
REG32(WIFI_LAST_RXBUF_INIT_09C, 0x09C); // only set in hal_init and mac_last_rxbuf_init
REG32(WIFI_RX_POLICY_0, 0xd8);
REG32(WIFI_RX_POLICY_1, 0xdc);
REG32(WIFI_RX_POLICY_2, 0xe0);
REG32(WIFI_RX_POLICY_3, 0xe4);
REG32(WIFI_PROMISC_MISC_BITMASK0, 0xf8);
REG32(WIFI_PROMISC_MISC_BITMASK1, 0xfc);
REG32(WIFI_PROMISC_MISC_BITMASK2, 0x100);
REG32(WIFI_PROMISC_MISC_BITMASK3, 0x104);

REG32(WIFI_RXBUF_INIT_HIGH_ADDR_0, 0x118); // only set in mac_rxbuf_init
REG32(WIFI_RXBUF_INIT_LOW_ADDR_0, 0x11c);  // only set in mac_rxbuf_init
REG32(WIFI_RXBUF_INIT_HIGH_ADDR_1, 0x120); // only set in mac_rxbuf_init
REG32(WIFI_RXBUF_INIT_LOW_ADDR_1, 0x124);  // only set in mac_rxbuf_init

REG32(WIFI_LAST_RXBUF_INIT_148, 0x148); // only set in mac_last_rxbuf_init
REG32(WIFI_LAST_RXBUF_INIT_14C, 0x14C); // only set in mac_last_rxbuf_init
REG32(WIFI_LAST_RXBUF_INIT_158, 0x158); // only set in mac_last_rxbuf_init
REG32(WIFI_LAST_RXBUF_INIT_164, 0x164); // only set in mac_last_rxbuf_init

REG32(WIFI_ANTENNA_INIT_284, 0x284);

REG32(WIFI_AUTOACK_INIT_400, 0x400); // only set in hal_mac_rate_autoack_init
REG32(WIFI_AUTOACK_INIT_404, 0x404); // only set in hal_mac_rate_autoack_init
REG32(WIFI_AUTOACK_INIT_408, 0x408); // only set in hal_mac_rate_autoack_init
REG32(WIFI_AUTOACK_INIT_40C, 0x40c); // only set in hal_mac_rate_autoack_init
REG32(WIFI_AUTOACK_INIT_410, 0x410); // only set in hal_mac_rate_autoack_init
REG32(WIFI_AUTOACK_INIT_414, 0x414); // only set in hal_mac_rate_autoack_init

REG32(WIFI_LOW_RATE_418, 0x418);
REG32(WIFI_LOW_RATE_41C, 0x41C);


// 0x800 - 0x814 are cryptography registers, likely for WPA and WEP

REG32(WIFI_MAYBE_TIMESTAMP, 0xc00);
REG32(WIFI_PROMISC_CONTROL_PKT, 0xc40);
REG32(WIFI_DMA_INT_STATUS, 0xc48);
REG32(WIFI_DMA_INT_CLR, 0xc4c);
REG32(WIFI_MAYBE_PWR_CTL, 0xcb8);
REG32(WIFI_TXQ_CLR_STATE_COLL_TIMEOUT, 0xcbc);
REG32(WIFI_TXQ_STATE_COLL_TIMEOUT, 0xcc0);
REG32(WIFI_TXQ_CLR_STATE_COMPLETE, 0xcc4);
REG32(WIFI_TXQ_STATE_COMPLETE, 0xcc8);
REG32(WIFI_TX_CONFIG_0, 0xd1c);
REG32(WIFI_DMA_OUTLINK, 0xd20);
REG32(WIFI_DMA_OUT_STATUS, 0xd24);


// Wifi registers only used for initialization
REG32(WIFI_TXRX_INIT_10C, 0x10c); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_114, 0x114); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C1C, 0xc1c); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C20, 0xc20); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C24, 0xc24); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C54, 0xc54); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C5C, 0xc5c); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C6C, 0xc6c); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C74, 0xc74); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C78, 0xc78); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_C88, 0xc88); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_CAC, 0xcac); // only set in mac_txrx_init
REG32(WIFI_TXRX_INIT_D78, 0xd78); // only set in mac_txrx_init

REG32(WIFI_TXRX_INIT_288, 0x288); // set in mac_txrx_init and hal_deinit
