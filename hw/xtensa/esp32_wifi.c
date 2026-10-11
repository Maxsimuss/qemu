/* SPDX-License-Identifier: GPL-2.0-or-later */
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
#include "hw/misc/trace.h"

/* MMIO accesses are traced by the shared peripheral trace infrastructure. */
#include "hw/misc/esp32_reg.h"

/*
 * The ESP32 TRM marks 0x3ff5c000..0x3ff5dfff reserved.  Independent reverse
 * engineering identifies 0x3ff5c000..0x3ff5cfff as Wi-Fi RX control.  The
 * supplied IDF PHY ELF exposes the following AGC RMW protocol:
 *
 * disable_wifi_agc(): c01c[23:16] = 0x7f; c038 |= bit 26;
 *                     c030[5:4] = 0; c080.bit0 = 1
 * enable_wifi_agc():  c080.bit0 = 0; c030[5:4] = 1;
 *                     c01c[23:16] = 0x0c; c038 |= bit 26
 * set_channel_rfpll_freq() tests d008[31:29] to select its RFPLL branch.
 *
 * This is a narrow register model, not a generic RAM window.  The private
 * block's silicon reset values and analog AGC transfer function are not
 * published; zero reset values preserve the prior QEMU initial state and
 * remain an explicit compatibility assumption.  Unknown offsets report a
 * guest error instead of silently behaving as writable RAM.
 */

static uint32_t esp32_wifi_rxctrl_reg(Esp32WifiState *s, hwaddr addr)
{
    switch (addr) {
    case ESP32_WIFI_RXCTRL_AGC_GAIN:
        return s->rxctrl_agc_gain;
    case ESP32_WIFI_RXCTRL_AGC_MODE:
        return s->rxctrl_agc_mode;
    case ESP32_WIFI_RXCTRL_AGC_STATE:
        return s->rxctrl_agc_state;
    case ESP32_WIFI_RXCTRL_AGC_ENABLE:
        return s->rxctrl_agc_enable;
    case ESP32_WIFI_RXCTRL_RFPLL_MODE:
        return s->rxctrl_rfpll_mode;
    case ESP32_WIFI_RXCTRL_COEX_AGC:
        return s->rxctrl_coex_agc;
    case ESP32_WIFI_RXCTRL_COEX_RFPLL:
        return s->rxctrl_coex_rfpll;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32-wifi-rxctrl: unknown read at +0x%03x\n",
                      (uint32_t)addr);
        return 0;
    }
}

static uint64_t esp32_wifi_rxctrl_read(void *opaque, hwaddr addr,
                                       unsigned int size)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);

    (void)size;
    return esp32_wifi_rxctrl_reg(s, addr);
}

static void esp32_wifi_rxctrl_write(void *opaque, hwaddr addr, uint64_t value,
                                    unsigned int size)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t v = value;

    (void)size;
    switch (addr) {
    case ESP32_WIFI_RXCTRL_AGC_GAIN:
        s->rxctrl_agc_gain = v;
        break;
    case ESP32_WIFI_RXCTRL_AGC_MODE:
        s->rxctrl_agc_mode = v;
        break;
    case ESP32_WIFI_RXCTRL_AGC_STATE:
        s->rxctrl_agc_state = v;
        break;
    case ESP32_WIFI_RXCTRL_AGC_ENABLE:
        s->rxctrl_agc_enable = v;
        break;
    case ESP32_WIFI_RXCTRL_RFPLL_MODE:
        s->rxctrl_rfpll_mode = v;
        break;
    case ESP32_WIFI_RXCTRL_COEX_AGC:
        s->rxctrl_coex_agc = v;
        break;
    case ESP32_WIFI_RXCTRL_COEX_RFPLL:
        s->rxctrl_coex_rfpll = v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32-wifi-rxctrl: unknown write at +0x%03x=%08x\n",
                      (uint32_t)addr, v);
        break;
    }
}

static const MemoryRegionOps esp32_wifi_rxctrl_ops = {
    .read = esp32_wifi_rxctrl_read,
    .write = esp32_wifi_rxctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .valid.unaligned = false,
};

/* The IDF PHY accesses the sparse FE/FE2 offsets below. The public IDF
 * fe_reg.h defines only the FE block bases and a few unrelated sleep bits.
 * Reset values and analog meaning for these private registers are unknown. */
static int esp32_wifi_fe_reg_index(hwaddr addr, bool fe2)
{
    static const hwaddr fe_offsets[] = {
        ESP32_WIFI_FE_CTRL_030, ESP32_WIFI_FE_CTRL_034,
        ESP32_WIFI_FE_CTRL_038, ESP32_WIFI_FE_CTRL_03C,
        ESP32_WIFI_FE_CTRL_040, ESP32_WIFI_FE_CTRL_044,
        ESP32_WIFI_FE_CTRL_060, ESP32_WIFI_FE_IQ_EST,
        ESP32_WIFI_FE_CTRL_04C, ESP32_WIFI_FE_PBUS_CMD,
        ESP32_WIFI_FE_CTRL_09C, ESP32_WIFI_FE_TXRX_CONTROL,
        ESP32_WIFI_FE_TXRX_OTHER,
    };
    static const hwaddr fe2_offsets[] = {
        ESP32_WIFI_FE2_CTRL_034, ESP32_WIFI_FE2_CTRL_038,
        ESP32_WIFI_FE2_CTRL_0D8, ESP32_WIFI_FE2_CTRL_0DC,
        ESP32_WIFI_FE2_CTRL_114,
    };
    const hwaddr *offsets = fe2 ? fe2_offsets : fe_offsets;
    unsigned count = fe2 ? ARRAY_SIZE(fe2_offsets) : ARRAY_SIZE(fe_offsets);

    for (unsigned i = 0; i < count; i++) {
        if (addr == offsets[i]) {
            return i;
        }
    }
    return -1;
}

/* The supplied IDF's ram_pbus_force_test() pulses FE+0x94 bit 1 and polls
 * FE+0xa0 bit 31 until the PBUS operation is no longer busy. Keep this
 * command engine separate from the private ANA/PBUS TXDC status port at
 * 0x3ff4e04c. The transaction duration is a one-microsecond virtual-time
 * approximation; command payload/result semantics remain unmodeled. */
#define ESP32_WIFI_FE_PBUS_BUSY BIT(31)
#define ESP32_WIFI_FE_PBUS_START BIT(1)
#define ESP32_WIFI_FE_PBUS_DELAY_NS 1000
#define ESP32_WIFI_FE_IQ_ENABLE BIT(26)
#define ESP32_WIFI_FE_IQ_START BIT(1)
#define ESP32_WIFI_FE_IQ_READY BIT(31)
#define ESP32_WIFI_FE_IQ_DELAY_NS 1000
#define ESP32_TXDC_PHY_CLOCKS 0x00008f8f

static void esp32_wifi_fe_pbus_complete(void *opaque)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);

    s->fe_pbus_busy = false;
}

static void esp32_wifi_fe_pbus_write(Esp32WifiState *s, uint32_t value)
{
    int cmd_index = esp32_wifi_fe_reg_index(ESP32_WIFI_FE_PBUS_CMD, false);
    bool start = !(s->fe_regs[cmd_index] & ESP32_WIFI_FE_PBUS_START) &&
                 (value & ESP32_WIFI_FE_PBUS_START);

    s->fe_regs[cmd_index] = value;
    if (start && !s->fe_pbus_busy) {
        /* IDF's ram_pbus_force_test() (g_phyFuns+184) encodes args as
         * (selector << 2) | (candidate << 6) | (index << 15), then pulses
         * bit 1. The TXDC loop programs selectors 2 and 3 with 9-bit values
         * before it samples the separate SENS+0x584c result. */
        /* The helper's selector argument is shifted by two, but its upper
         * bits overlap the candidate field beginning at bit 6. The actual
         * TXDC routine uses selector values 2 and 3, encoded in bits 2:3. */
        unsigned selector = (value >> 2) & 0x3;
        if (selector == 2 || selector == 3) {
            s->fe_pbus_test_candidate[selector] = (value >> 6) & 0x1ff;
            s->fe_pbus_test_index[selector] = (value >> 15) & 0xff;
            s->fe_pbus_test_valid[selector] = true;
        }
        s->fe_pbus_busy = true;
        timer_mod(s->fe_pbus_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  ESP32_WIFI_FE_PBUS_DELAY_NS);
    }
}

static void esp32_wifi_fe_iq_complete(void *opaque)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);

    s->fe_iq_ready = true;
}

static void esp32_wifi_fe_iq_write(Esp32WifiState *s, uint32_t value)
{
    int iq_index = esp32_wifi_fe_reg_index(ESP32_WIFI_FE_IQ_EST, false);
    int enable_index = esp32_wifi_fe_reg_index(ESP32_WIFI_FE_CTRL_060, false);
    uint32_t old = s->fe_regs[iq_index];
    bool start = !(old & ESP32_WIFI_FE_IQ_START) &&
                 (value & ESP32_WIFI_FE_IQ_START);

    s->fe_regs[iq_index] = value & ~ESP32_WIFI_FE_IQ_READY;
    if (!(value & ESP32_WIFI_FE_IQ_START)) {
        timer_del(s->fe_iq_timer);
        s->fe_iq_ready = false;
        return;
    }
    if (!start) {
        return;
    }
    if (!(s->fe_regs[enable_index] & ESP32_WIFI_FE_IQ_ENABLE) ||
        (s->wifi_clock_en & ESP32_TXDC_PHY_CLOCKS) !=
         ESP32_TXDC_PHY_CLOCKS ||
        (s->core_reset_en & (1u << ESP32_WIFI_FE_RESET_BIT))) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32-wifi: IQ estimator start without FE enable/PHY clocks\n");
        return;
    }
    s->fe_iq_ready = false;
    timer_mod(s->fe_iq_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              ESP32_WIFI_FE_IQ_DELAY_NS);
}

static uint64_t esp32_wifi_fe_read(void *opaque, hwaddr addr,
                                   unsigned size, bool fe2)
{
    Esp32WifiState *s = opaque;
    int i = esp32_wifi_fe_reg_index(addr, fe2);

    if (i < 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32-fe%s: unknown read at +0x%03x\n",
                      fe2 ? "2" : "", (unsigned)addr);
        return 0;
    }
    if (!fe2 && addr == ESP32_WIFI_FE_TXRX_CONTROL) {
        return s->fe_regs[i] |
               (s->fe_pbus_busy ? ESP32_WIFI_FE_PBUS_BUSY : 0);
    }
    if (!fe2 && addr == ESP32_WIFI_FE_IQ_EST) {
        return s->fe_regs[i] |
               (s->fe_iq_ready ? ESP32_WIFI_FE_IQ_READY : 0);
    }
    return fe2 ? s->fe2_regs[i] : s->fe_regs[i];
}

static void esp32_wifi_fe_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size, bool fe2)
{
    Esp32WifiState *s = opaque;
    int i = esp32_wifi_fe_reg_index(addr, fe2);

    if (i < 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32-fe%s: unknown write at +0x%03x=%08x\n",
                      fe2 ? "2" : "", (unsigned)addr, (uint32_t)value);
        return;
    }
    if (!fe2 && addr == ESP32_WIFI_FE_PBUS_CMD) {
        esp32_wifi_fe_pbus_write(s, value);
    } else if (!fe2 && addr == ESP32_WIFI_FE_IQ_EST) {
        esp32_wifi_fe_iq_write(s, value);
    } else if (fe2) {
        s->fe2_regs[i] = value;
    } else {
        s->fe_regs[i] = value;
    }
}

static uint64_t esp32_wifi_fe_read_cb(void *opaque, hwaddr addr,
                                     unsigned size)
{
    return esp32_wifi_fe_read(opaque, addr, size, false);
}

static void esp32_wifi_fe_write_cb(void *opaque, hwaddr addr, uint64_t value,
                                   unsigned size)
{
    esp32_wifi_fe_write(opaque, addr, value, size, false);
}

static uint64_t esp32_wifi_fe2_read_cb(void *opaque, hwaddr addr,
                                      unsigned size)
{
    return esp32_wifi_fe_read(opaque, addr, size, true);
}

static void esp32_wifi_fe2_write_cb(void *opaque, hwaddr addr, uint64_t value,
                                    unsigned size)
{
    esp32_wifi_fe_write(opaque, addr, value, size, true);
}

static const MemoryRegionOps esp32_wifi_fe_ops = {
    .read = esp32_wifi_fe_read_cb,
    .write = esp32_wifi_fe_write_cb,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static const MemoryRegionOps esp32_wifi_fe2_ops = {
    .read = esp32_wifi_fe2_read_cb,
    .write = esp32_wifi_fe2_write_cb,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

/* The supplied IDF 6.1 and 6.2 PHY ELFs write 0x00113cf1 then
 * 0x00113cf3 to this private ANA/PBUS port. Bit 1 is the start pulse, bit 24
 * is polled set as completion, and bit 31 is consumed as a signed TXDC
 * measurement. This is not part of the analog-I2C host aperture. */
#define ESP32_TXDC_PBUS_START BIT(1)
#define ESP32_TXDC_PBUS_DONE BIT(24)
#define ESP32_TXDC_PBUS_Q_SIGN BIT(30)
#define ESP32_TXDC_PBUS_SIGN BIT(31)
#define ESP32_TXDC_PBUS_DELAY_NS 1000

static bool esp32_wifi_txdc_path_enabled(const Esp32WifiState *s)
{
    return (s->wifi_clock_en & ESP32_TXDC_PHY_CLOCKS) ==
           ESP32_TXDC_PHY_CLOCKS &&
           !(s->core_reset_en & (1u << ESP32_WIFI_FE_RESET_BIT));
}

static void esp32_wifi_txdc_schedule(Esp32WifiState *s)
{
    if (!s->txdc_pbus_pending) {
        return;
    }
    if (!esp32_wifi_txdc_path_enabled(s)) {
        timer_del(s->txdc_pbus_timer);
        return;
    }
    timer_mod(s->txdc_pbus_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              ESP32_TXDC_PBUS_DELAY_NS);
}

static void esp32_wifi_txdc_pbus_complete(void *opaque)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);
    int32_t residual_i;
    int32_t residual_q;

    if (!s->txdc_pbus_pending || !esp32_wifi_txdc_path_enabled(s)) {
        return;
    }
    s->txdc_pbus_pending = false;
    /* The IDF calibration initializes both 9-bit quadrature candidates at
     * 0x100 and clamps them to [0, 0x1ff]. Model the ideal nominal RF path as
     * zero feedthrough at that midpoint. The measured signed residual is
     * therefore driven by the two values actually serialized through the
     * FE force-test commands, not by the SENS trigger word. The physical
     * silicon's nonzero board/PA offset is not published and remains outside
     * this nominal analytic environment. */
    residual_i = 0x100 - s->fe_pbus_test_candidate[2];
    residual_q = 0x100 - s->fe_pbus_test_candidate[3];
    if (residual_i < 0) {
        s->txdc_pbus_status |= ESP32_TXDC_PBUS_SIGN;
    } else {
        s->txdc_pbus_status &= ~ESP32_TXDC_PBUS_SIGN;
    }
    if (residual_q < 0) {
        s->txdc_pbus_status |= ESP32_TXDC_PBUS_Q_SIGN;
    } else {
        s->txdc_pbus_status &= ~ESP32_TXDC_PBUS_Q_SIGN;
    }
    s->txdc_pbus_status |= ESP32_TXDC_PBUS_DONE;
}

static uint64_t esp32_wifi_txdc_pbus_read(void *opaque, hwaddr addr,
                                          unsigned int size)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);

    (void)addr;
    (void)size;
    return s->txdc_pbus_status;
}

static void esp32_wifi_txdc_pbus_write(void *opaque, hwaddr addr,
                                       uint64_t value, unsigned int size)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t command = value;
    bool start = !(s->txdc_pbus_command & ESP32_TXDC_PBUS_START) &&
                 (command & ESP32_TXDC_PBUS_START);

    (void)addr;
    (void)size;
    s->txdc_pbus_command = command;
    if (!start) {
        return;
    }

    s->txdc_pbus_status = command & ~(ESP32_TXDC_PBUS_DONE |
                                      ESP32_TXDC_PBUS_SIGN);
    timer_del(s->txdc_pbus_timer);
    if (!esp32_wifi_txdc_path_enabled(s) ||
        !s->fe_pbus_test_valid[2] || !s->fe_pbus_test_valid[3]) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32-wifi: TXDC measurement without programmed FE candidates or valid PHY clocks/reset (clk=%08x rst=%08x)\n",
                      s->wifi_clock_en, s->core_reset_en);
        return;
    }
    s->txdc_pbus_pending = true;
    esp32_wifi_txdc_schedule(s);
}

static const MemoryRegionOps esp32_wifi_txdc_pbus_ops = {
    .read = esp32_wifi_txdc_pbus_read,
    .write = esp32_wifi_txdc_pbus_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

/* The supplied IDF PHY's set_chan_freq_sw_start() writes its channel tune
 * code in SENS+0xc4, sets bit 8, waits 2 us, then checks bit 31.
 * If bit 31 is clear it polls while bit 8 remains set; the completed path
 * clears bit 8, waits 40 us, and calls pll_correct_dcap(). This private field
 * contract comes from that exact driver disassembly; the public TRM does not
 * document the analog transfer function. */
#define ESP32_RFPLL_FREQ_START BIT(8)
#define ESP32_RFPLL_FREQ_READY BIT(31)
#define ESP32_RFPLL_FREQ_DELAY_NS 2000
#define ESP32_RFPLL_REQUIRED_CLOCKS 0x00000406
#define ESP32_RFPLL_REQUIRED_RESETS (BIT(0) | BIT(1))

static bool esp32_wifi_rfpll_enabled(const Esp32WifiState *s)
{
    return (s->wifi_clock_en & ESP32_RFPLL_REQUIRED_CLOCKS) ==
               ESP32_RFPLL_REQUIRED_CLOCKS &&
           !(s->core_reset_en & ESP32_RFPLL_REQUIRED_RESETS);
}

static uint8_t esp32_wifi_rfpll_channel_for_code(uint8_t code)
{
    /* The real IDF set_channel(1/2/14) path writes 0x18/0x22/0xa8
     * respectively to C4, measured at the C4 store/readback. Channels 1..13
     * advance by 0x0a per 5 MHz channel step; channel 14 is the 2484 MHz
     * special case. Do not treat the public frequency in MHz as the C4 code. */
    if (code >= 0x18 && code <= 0x90 && (code - 0x18) % 0x0a == 0) {
        return 1 + (code - 0x18) / 0x0a;
    }
    return code == 0xa8 ? 14 : 0;
}

static void esp32_wifi_rfpll_publish(Esp32WifiState *s)
{
    for (unsigned bit = 0; bit < 8; bit++) {
        qemu_set_irq(s->rfpll_tune_out[bit],
                     (s->rfpll_freq_value >> bit) & 1);
    }
    qemu_set_irq(s->rfpll_tune_out[8],
                 (s->rfpll_freq_value & ESP32_RFPLL_FREQ_START) != 0);
}

static void esp32_wifi_rfpll_schedule(Esp32WifiState *s)
{
    if (s->rfpll_freq_pending && esp32_wifi_rfpll_enabled(s)) {
        timer_mod(s->rfpll_freq_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  ESP32_RFPLL_FREQ_DELAY_NS);
    }
}

static void esp32_wifi_rfpll_complete(void *opaque)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);

    if (!s->rfpll_freq_pending || !esp32_wifi_rfpll_enabled(s)) {
        return;
    }
    s->rfpll_freq_pending = false;
    s->rfpll_freq_value &= ~ESP32_RFPLL_FREQ_START;
    s->rf_channel = esp32_wifi_rfpll_channel_for_code(
        s->rfpll_freq_value & 0xff);
    if (s->rf_channel) {
        /* set_chan_freq_sw_start() branches on the sign bit before polling
         * START: bit 31 set is the observed completed/ready path. */
        s->rfpll_freq_value |= ESP32_RFPLL_FREQ_READY;
    } else {
        /* Unsupported tune values do not report completion. */
        s->rfpll_freq_value &= ~ESP32_RFPLL_FREQ_READY;
    }
    esp32_wifi_rfpll_publish(s);
}

static uint64_t esp32_wifi_rfpll_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);

    (void)addr;
    (void)size;
    return s->rfpll_freq_value;
}

static void esp32_wifi_rfpll_write(void *opaque, hwaddr addr, uint64_t value,
                                   unsigned size)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t old = s->rfpll_freq_value;
    uint32_t next = value;
    bool start = !(old & ESP32_RFPLL_FREQ_START) &&
                 (next & ESP32_RFPLL_FREQ_START);

    (void)addr;
    (void)size;
    if (!(next & ESP32_RFPLL_FREQ_START)) {
        timer_del(s->rfpll_freq_timer);
        s->rfpll_freq_pending = false;
    }
    if (start) {
        s->rfpll_freq_pending = true;
        next &= ~ESP32_RFPLL_FREQ_READY;
        timer_del(s->rfpll_freq_timer);
    }
    s->rfpll_freq_value = next;
    esp32_wifi_rfpll_publish(s);
    esp32_wifi_rfpll_schedule(s);
}

static void esp32_wifi_rfpll_reset(Esp32WifiState *s)
{
    timer_del(s->rfpll_freq_timer);
    s->rfpll_freq_value = 0;
    s->rfpll_freq_pending = false;
    esp32_wifi_rfpll_publish(s);
}

static const MemoryRegionOps esp32_wifi_rfpll_ops = {
    .read = esp32_wifi_rfpll_read,
    .write = esp32_wifi_rfpll_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

/* IDF's phy_bt_ifs_set() accesses UART1+0x103c (0x3ff5103c), a vendor PHY
 * coexistence word outside the UART register aperture. The ELF's masked
 * writes preserve bits 11:0 and select one of two upper field encodings;
 * undocumented reset and RF effect remain unknown. */
static uint64_t esp32_wifi_phy_bt_read(void *opaque, hwaddr addr,
                                       unsigned int size)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);

    (void)addr;
    (void)size;
    return s->phy_bt_ifs;
}

static void esp32_wifi_phy_bt_write(void *opaque, hwaddr addr,
                                    uint64_t value, unsigned int size)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);

    (void)addr;
    (void)size;
    s->phy_bt_ifs = value;
}

static const MemoryRegionOps esp32_wifi_phy_bt_ops = {
    .read = esp32_wifi_phy_bt_read,
    .write = esp32_wifi_phy_bt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

/* coex_bt_high_prio() in the supplied PHY image performs RMWs at these
 * APB addresses. Keep each observed word isolated: this range overlaps the
 * Wi-Fi APB window, and unobserved offsets have no inferred behavior. The
 * firmware programs Bluetooth coexistence policy here; QEMU does not model
 * a Bluetooth RF peer, so this records those digital controls without
 * claiming a coexistence/RF effect. Silicon reset values are unpublished. */
static uint64_t esp32_wifi_phy_coex_read(void *opaque, hwaddr addr,
                                         unsigned int size)
{
    uint32_t *reg = opaque;

    (void)addr;
    (void)size;
    return *reg;
}

static void esp32_wifi_phy_coex_write(void *opaque, hwaddr addr,
                                      uint64_t value, unsigned int size)
{
    uint32_t *reg = opaque;

    (void)addr;
    (void)size;
    *reg = value;
}

static const MemoryRegionOps esp32_wifi_phy_coex_ops = {
    .read = esp32_wifi_phy_coex_read,
    .write = esp32_wifi_phy_coex_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

/* The IDF 6.1 libpp `hal_get_tsf_time()` code sets bit 0 or 1 in WDEV+0x10,
 * reads the corresponding low/high pair, then clears the bit. IEEE 802.11
 * TSF units are microseconds. The WDEV timer shares the Wi-Fi MAC clock/reset
 * domain; advance from QEMU virtual time only while that domain is enabled.
 * The two counter pairs are exposed independently because the helper selects
 * them with its argument. */
static uint64_t esp32_wifi_wdev_tsf_now(Esp32WifiState *s)
{
    uint64_t elapsed = s->wdev_tsf_elapsed_us;

    if (s->wdev_tsf_running) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

        if (now > s->wdev_tsf_epoch_ns) {
            elapsed += (now - s->wdev_tsf_epoch_ns) / 1000;
        }
    }
    return elapsed;
}

static void esp32_wifi_wdev_tsf_set_clock(Esp32WifiState *s, bool enabled)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (s->wdev_tsf_running && !enabled) {
        s->wdev_tsf_elapsed_us = esp32_wifi_wdev_tsf_now(s);
        s->wdev_tsf_running = false;
    } else if (!s->wdev_tsf_running && enabled) {
        s->wdev_tsf_epoch_ns = now;
        s->wdev_tsf_running = true;
    }
}

static void esp32_wifi_wdev_tsf_reset(Esp32WifiState *s)
{
    s->wdev_tsf_elapsed_us = 0;
    s->wdev_tsf_latched_us[0] = 0;
    s->wdev_tsf_latched_us[1] = 0;
    s->wdev_tsf_latch = 0;
    s->wdev_tsf_epoch_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->wdev_tsf_running = false;
    s->mem[ESP32_WIFI_WDEV_TSF_CTRL / 4] = 0;
}

static uint32_t esp32_wifi_wdev_tsf_read(Esp32WifiState *s, hwaddr addr)
{
    unsigned counter;
    bool high;
    uint64_t value;

    if (addr == ESP32_WIFI_WDEV_TSF_CTRL) {
        return s->wdev_tsf_latch;
    }
    switch (addr) {
    case ESP32_WIFI_WDEV_TSF0_LO:
        counter = 0;
        high = false;
        break;
    case ESP32_WIFI_WDEV_TSF0_HI:
        counter = 0;
        high = true;
        break;
    case ESP32_WIFI_WDEV_TSF1_LO:
        counter = 1;
        high = false;
        break;
    case ESP32_WIFI_WDEV_TSF1_HI:
        counter = 1;
        high = true;
        break;
    default:
        return s->mem[addr / 4];
    }

    value = (s->wdev_tsf_latch & (1U << counter)) ?
        s->wdev_tsf_latched_us[counter] : esp32_wifi_wdev_tsf_now(s);
    return high ? value >> 32 : value;
}

static void esp32_wifi_wdev_tsf_write(Esp32WifiState *s, hwaddr addr,
                                      uint32_t value)
{
    uint32_t rising;

    if (addr != ESP32_WIFI_WDEV_TSF_CTRL) {
        return;
    }
    rising = (value & ~s->wdev_tsf_latch) & 0x3;
    for (unsigned i = 0; i < 2; i++) {
        if (rising & (1U << i)) {
            s->wdev_tsf_latched_us[i] = esp32_wifi_wdev_tsf_now(s);
        }
    }
    s->wdev_tsf_latch = value;
    s->mem[addr / 4] = value;
}

static uint64_t esp32_wifi_read(void *opaque, hwaddr addr, unsigned int size)
{

    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t r = esp32_wifi_wdev_tsf_read(s, addr);

    switch(addr) {
        case ESP32_WIFI_WDEV_TSF_CTRL:
        case ESP32_WIFI_WDEV_TSF0_LO:
        case ESP32_WIFI_WDEV_TSF0_HI:
        case ESP32_WIFI_WDEV_TSF1_LO:
        case ESP32_WIFI_WDEV_TSF1_HI:
            break;
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

    /* RX descriptors arrive owned by the MAC (OWN=1).  Completion returns
     * the descriptor to software; the driver sets OWN again only when it
     * recycles the RX buffer onto the hardware list. */
    control = (control & ~((0xfffU << 12) | (1U << 30) | (1U << 31))) |
              ((length & 0xfffU) << 12) | (1U << 30);
    stl_le_p(&item->control_le, control);
}

bool esp32_wifi_mac_ccmp(Esp32WifiState *s, mac80211_frame *frame,
                         bool encrypt, uint8_t interface_id)
{
    uint8_t table[32][40];
    uint8_t peer[6];
    uint32_t valid;
    int group_key_id;
    Esp32WifiKey key;
    uint32_t control;

    if (!esp32_wifi_mac_enabled(s) || !frame || interface_id > 2 ||
        frame->frame_length < IEEE80211_HEADER_SIZE) {
        return false;
    }
    control = s->mem[(0x800 + 4 * interface_id) / 4];
    /* hal_crypto_enable() programs this interface's base mode and receive
     * controls independently of the global crypto engine. Bit 31 is not a
     * general interface-enable bit: the guest leaves it clear for the
     * observed CCMP key-install path. */
    if ((control & 0x00010103U) != 0x00010103U) {
        return false;
    }

    for (unsigned i = 0; i < 32; i++) {
        for (unsigned word = 0; word < 10; word++) {
            stl_le_p(&table[i][word * 4],
                     s->mem[(ESP32_WIFI_CRYPTO_KEY_TABLE + i * 40 +
                             word * 4) / 4]);
        }
    }
    valid = s->mem[ESP32_WIFI_CRYPTO_KEY_VALID / 4];
    if (encrypt) {
        memcpy(peer, frame->receiver_address, sizeof(peer));
        /* The supported STA path sends to its AP with a unicast receiver
         * address, even when the LLC destination is broadcast (DHCP, ARP).
         * Do not guess an active GTK for secured multicast TX. */
        if (peer[0] & 1) {
            return false;
        }
        group_key_id = -1;
    } else {
        uint8_t *body = frame->data_and_fcs;

        memcpy(peer, frame->transmitter_address, sizeof(peer));
        if (frame->frame_length < IEEE80211_HEADER_SIZE + 8) {
            return false;
        }
        /* For an AP-to-STA From-DS frame, addr2 (the transmitter) is the
         * unicast AP BSSID even when addr1 (the receiver) is multicast. The
         * receiver address selects GTK versus pairwise TK; KeyID then picks
         * the GTK slot. Keep addr2 as the peer for pairwise lookup. */
        group_key_id = (frame->receiver_address[0] & 1) ?
                       (body[3] >> 6) & 3 : -1;
    }
    if (!esp32_wifi_key_lookup(table, valid, interface_id,
                               ESP32_WIFI_KEY_CIPHER_CCMP, peer,
                               group_key_id, &key)) {
        return false;
    }
    if (encrypt) {
        uint8_t *body = frame->data_and_fcs;
        uint64_t pn;
        size_t plaintext_len;
        uint64_t next_pn;

        /* TX DMA supplies MAC header, CCMP IV, plaintext, reserved MIC and
         * reserved FCS. Preserve its IV/PN and transform only the payload. */
        if (frame->frame_length < IEEE80211_HEADER_SIZE + 20 ||
            frame->frame_length > IEEE80211_HEADER_SIZE +
                                  sizeof(frame->data_and_fcs)) {
            return false;
        }
        pn = (uint64_t)body[0] | ((uint64_t)body[1] << 8) |
             ((uint64_t)body[4] << 16) | ((uint64_t)body[5] << 24) |
             ((uint64_t)body[6] << 32) | ((uint64_t)body[7] << 40);
        if (!(body[3] & 0x20) || ((body[3] >> 6) & 3) != 0 ||
            !pn || pn <= s->crypto_tx_pn[key.index]) {
            return false;
        }
        plaintext_len = frame->frame_length - IEEE80211_HEADER_SIZE - 20;
        memmove(body, body + 8, plaintext_len);
        frame->frame_length = IEEE80211_HEADER_SIZE + plaintext_len;
        frame->pos = plaintext_len;
        next_pn = pn - 1;
        if (!esp32_ccmp_encrypt(frame, key.bytes, 0, &next_pn) ||
            next_pn != pn) {
            return false;
        }
        s->crypto_tx_pn[key.index] = pn;
        insertCRC(frame);
        return frame->frame_length == IEEE80211_HEADER_SIZE + 8 +
                                      plaintext_len + 8 + 4;
    }
    {
        uint8_t ccmp_header[8], mic[8];
        uint8_t *raw = (uint8_t *)frame;
        uint8_t *body = frame->data_and_fcs;
        size_t encrypted_body_len = frame->frame_length -
                                    IEEE80211_HEADER_SIZE;
        size_t plaintext_len;

        if (encrypted_body_len < sizeof(ccmp_header) + sizeof(mic)) {
            return false;
        }
        memcpy(ccmp_header, body, sizeof(ccmp_header));
        memcpy(mic, body + encrypted_body_len - sizeof(mic), sizeof(mic));
        if (!esp32_ccmp_decrypt(frame, key.bytes,
                                &s->crypto_rx_pn[key.index])) {
            return false;
        }

        /* The ESP32 RX DMA contract feeds the firmware's protected-frame
         * crypto decapsulation path. The MAC has already verified/decrypted
         * the CCMP payload, but it retains the on-air CCMP header, KeyID and
         * MIC so net80211 can remove them and apply its replay/state logic.
         * esp32_ccmp_decrypt() is also used by the external Ethernet peer, so
         * restore that hardware-facing layout only at this MAC boundary. */
        plaintext_len = frame->frame_length - IEEE80211_HEADER_SIZE;
        memmove(body + sizeof(ccmp_header), body, plaintext_len);
        memcpy(body, ccmp_header, sizeof(ccmp_header));
        memcpy(body + sizeof(ccmp_header) + plaintext_len, mic, sizeof(mic));
        frame->frame_length = IEEE80211_HEADER_SIZE + sizeof(ccmp_header) +
                              plaintext_len + sizeof(mic);
        frame->pos = sizeof(ccmp_header) + plaintext_len + sizeof(mic);
        raw[1] |= 0x40; /* Protected remains set until firmware decapsulation. */
        return true;
    }
}

static void esp32_wifi_crypto_track_write(Esp32WifiState *s, hwaddr addr,
                                          uint32_t value)
{
    if (addr == ESP32_WIFI_CRYPTO_KEY_VALID) {
        uint32_t changed = s->mem[addr / 4] ^ value;

        /* A validity transition is either a key install or invalidation.
         * PN state belongs to the key slot, so a new use of that slot starts
         * at PN zero even when the same key bytes are installed again. */
        for (unsigned i = 0; i < 32; i++) {
            if (changed & BIT(i)) {
                s->crypto_rx_pn[i] = 0;
                s->crypto_tx_pn[i] = 0;
            }
        }
        return;
    }

    if (addr >= ESP32_WIFI_CRYPTO_KEY_TABLE &&
        addr < ESP32_WIFI_CRYPTO_KEY_TABLE + 32 * 40) {
        unsigned slot = (addr - ESP32_WIFI_CRYPTO_KEY_TABLE) / 40;

        /* Also cover replacement of a still-valid record that did not
         * toggle its valid bit. Identical writes leave an active key's PN
         * untouched. */
        if (s->mem[addr / 4] != value) {
            s->crypto_rx_pn[slot] = 0;
            s->crypto_tx_pn[slot] = 0;
        }
    }
}

static void esp32_wifi_write(void *opaque, hwaddr addr, uint64_t v, unsigned int size) {
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t value = (uint32_t) v;
    (void)size;
    if (addr == ESP32_WIFI_WDEV_TSF_CTRL) {
        esp32_wifi_wdev_tsf_write(s, addr, value);
        return;
    }
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
                mac80211_frame frame = { 0 };
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
                /* On TX, the ESP32 MAC fetch length is the descriptor's
                 * length field. The supplied IDF's ppProcTxSecFrame()
                 * extends that field while leaving size unchanged, so size
                 * is not a TX transfer bound. Keep the guest read within
                 * this fixed frame and internal DRAM instead. */
                if (frame_length < sizeof(frame.frame_control) ||
                    frame_length > ESP32_WIFI_MAX_FRAME_SIZE ||
                    !esp32_wifi_dma_range_valid(frame_address, frame_length)) {
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "wifi TX descriptor at %08x control=%08x data=%08x next=%08x has unsupported length %u (buffer %u)\n",
                                  (uint32_t)memaddr,
                                  wifi_desc_control(&item),
                                  (uint32_t)frame_address,
                                  ldl_le_p(&item.next), frame_length,
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
                if (frame.frame_control.type == IEEE80211_TYPE_DATA &&
                    (((uint8_t *)&frame)[1] & 0x40)) {
                    bool encrypted = false;

                    /* The DMA buffer contains a prefilled CCMP header and
                     * reserved MIC/FCS tail regardless of descriptor bit 29.
                     * Transform it once using the matching programmed key. */
                    encrypted = esp32_wifi_mac_ccmp(s, &frame, true, 0);
                    if (!encrypted) {
                        qemu_log_mask(LOG_GUEST_ERROR,
                                      "wifi TX protected frame has no active matching CCMP key\n");
                    } else {
                        Esp32_WLAN_handle_frame(s, &frame);
                    }
                } else {
                    Esp32_WLAN_handle_frame(s, &frame);
                }
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
    esp32_wifi_crypto_track_write(s, addr, value);
    s->mem[addr/4]=value;
}

static bool wifi_is_broadcast(const uint8_t mac[6])
{
    return !memcmp(mac, BROADCAST, 6);
}

/* The first filter register compares the low four address bytes; the low
 * 16 bits of the second compare the final two bytes.  The driver's setter
 * uses bit 16 of the second register as the slot enable (the upper bits are
 * control, not part of the six-byte mask). */
static bool wifi_address_filter_match(const Esp32WifiState *s,
                                      const uint8_t candidate[6],
                                      unsigned address_offset,
                                      unsigned filter_offset)
{
    uint32_t address_low = s->mem[address_offset / 4];
    uint32_t address_high = s->mem[address_offset / 4 + 1];
    uint32_t filter_low = s->mem[filter_offset / 4];
    uint32_t filter_high = s->mem[filter_offset / 4 + 1];

    if (!(filter_high & (1U << 16))) {
        return false;
    }

    for (unsigned i = 0; i < 6; i++) {
        uint8_t address_byte = i < 4 ? address_low >> (i * 8)
                                     : address_high >> ((i - 4) * 8);
        uint8_t mask_byte = i < 4 ? filter_low >> (i * 8)
                                  : filter_high >> ((i - 4) * 8);

        if ((candidate[i] & mask_byte) != (address_byte & mask_byte)) {
            return false;
        }
    }
    return true;
}

static const uint8_t *wifi_frame_bssid(const mac80211_frame *frame)
{
    if (frame->frame_control.to_ds && frame->frame_control.from_ds) {
        return NULL;
    }
    if (frame->frame_control.to_ds) {
        return frame->receiver_address;
    }
    if (frame->frame_control.from_ds) {
        return frame->transmitter_address;
    }
    return frame->address_3;
}

// frame from QEMU to ESP32
void Esp32_sendFrame(Esp32WifiState *s, mac80211_frame *frame, int length, int signal_strength) {
    uint8_t header[28 + ESP32_WIFI_MAX_FRAME_SIZE];
    size_t dma_length;
    dma_list_item item;
    uint32_t descriptor_address;
    uint32_t next_descriptor;
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
    const uint8_t *bssid;
    *pkt=(wifi_pkt_rx_ctrl_t){
        .rssi=signal_strength,
        .rate=11,
        .sig_len=length,
        .sig_len_copy=length,
        .legacy_length=length,
        .noise_floor=-97,
        .channel=s->rf_channel,
        .timestamp=qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1000,
    };
    /* DA match uses the two programmed local MAC slots. Broadcast is a
     * destination match; multicast is compared against the programmed
     * address and mask instead of being treated as broadcast. */
    pkt->damatch0 = wifi_is_broadcast(frame->receiver_address) ||
        wifi_address_filter_match(s, frame->receiver_address,
                                  0x040, 0x060);
    pkt->damatch1 = wifi_is_broadcast(frame->receiver_address) ||
        wifi_address_filter_match(s, frame->receiver_address,
                                  0x048, 0x068);

    /* BSSID match is independent of the DA and follows the 802.11 DS
     * address mapping. WDS has no single BSSID. */
    bssid = wifi_frame_bssid(frame);
    if (bssid) {
        pkt->bssidmatch0 = wifi_address_filter_match(s, bssid,
                                                    0x000, 0x020);
        pkt->bssidmatch1 = wifi_address_filter_match(s, bssid,
                                                    0x008, 0x028);
        if (s->guest_softap && s->guest_channel &&
            !memcmp(bssid, s->guest_ap_macaddr, sizeof(s->guest_ap_macaddr))) {
            pkt->channel = s->guest_channel;
        }
    }
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
    descriptor_address = s->dma_inlink_address;
    next_descriptor = ldl_le_p(&item.next);
    trace_esp32_wifi_rx_dma_attempt(
        descriptor_address, ldl_le_p(&item.address), next_descriptor,
        wifi_desc_size(&item), dma_length, wifi_desc_owner(&item),
        frame->frame_control.type, frame->frame_control.sub_type);
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
    s->dma_inlink_address=next_descriptor;
    set_interrupt(s, 0x1000024);
    trace_esp32_wifi_rx_dma_complete(
        descriptor_address, next_descriptor, dma_length, 0x1000024,
        frame->frame_control.type, frame->frame_control.sub_type);
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
    esp32_wifi_wdev_tsf_reset(s);
    memset(s->crypto_rx_pn, 0, sizeof(s->crypto_rx_pn));
    memset(s->crypto_tx_pn, 0, sizeof(s->crypto_tx_pn));
    s->rxctrl_agc_gain = 0;
    s->rxctrl_agc_mode = 0;
    s->rxctrl_agc_state = 0;
    s->rxctrl_agc_enable = 0;
    s->rxctrl_rfpll_mode = 0;
    s->rxctrl_coex_agc = 0;
    s->rxctrl_coex_rfpll = 0;
    memset(s->fe_regs, 0, sizeof(s->fe_regs));
    memset(s->fe2_regs, 0, sizeof(s->fe2_regs));
    memset(s->fe_pbus_test_candidate, 0,
           sizeof(s->fe_pbus_test_candidate));
    memset(s->fe_pbus_test_valid, 0, sizeof(s->fe_pbus_test_valid));
    memset(s->fe_pbus_test_index, 0xff, sizeof(s->fe_pbus_test_index));
    timer_del(s->fe_pbus_timer);
    s->fe_pbus_busy = false;
    timer_del(s->fe_iq_timer);
    s->fe_iq_ready = false;
    timer_del(s->txdc_pbus_timer);
    s->txdc_pbus_status = 0;
    s->txdc_pbus_command = 0;
    s->txdc_pbus_pending = false;
    esp32_wifi_rfpll_reset(s);
    s->phy_bt_ifs = 0;
    memset(s->phy_coex_regs, 0, sizeof(s->phy_coex_regs));
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
    esp32_wifi_wdev_tsf_set_clock(s, esp32_wifi_mac_enabled(s));
    if (s->mac_clock_enabled !=
        ((old & ESP32_WIFI_MAC_CLOCK_MASK) == ESP32_WIFI_MAC_CLOCK_MASK)) {
        Esp32_WLAN_clock_changed(s);
    }
    esp32_wifi_rfpll_schedule(s);
    esp32_wifi_txdc_schedule(s);
}

static void esp32_wifi_core_reset_input(void *opaque, int n, int level)
{
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t old = s->core_reset_en;
    bool old_asserted;

    s->core_reset_en = (old & ~(1U << n)) | ((uint32_t)!!level << n);
    if (n == ESP32_WIFI_FE_RESET_BIT || n == 0) {
        if (level && !(old & (1U << n))) {
            esp32_wifi_rfpll_reset(s);
        }
        if (n == ESP32_WIFI_FE_RESET_BIT && level && !(old & (1U << n))) {
            memset(s->fe_regs, 0, sizeof(s->fe_regs));
            memset(s->fe2_regs, 0, sizeof(s->fe2_regs));
            memset(s->fe_pbus_test_candidate, 0,
                   sizeof(s->fe_pbus_test_candidate));
            memset(s->fe_pbus_test_valid, 0,
                   sizeof(s->fe_pbus_test_valid));
            memset(s->fe_pbus_test_index, 0xff,
                   sizeof(s->fe_pbus_test_index));
            timer_del(s->fe_pbus_timer);
            s->fe_pbus_busy = false;
            timer_del(s->fe_iq_timer);
            s->fe_iq_ready = false;
            timer_del(s->txdc_pbus_timer);
            s->txdc_pbus_status = 0;
            s->txdc_pbus_command = 0;
            s->txdc_pbus_pending = false;
        }
        esp32_wifi_rfpll_schedule(s);
        return;
    }
    if (n != ESP32_WIFI_MAC_RESET_BIT) {
        return;
    }
    old_asserted = s->mac_reset_asserted;
    s->mac_reset_asserted = level != 0;
    if (s->mac_reset_asserted && !old_asserted) {
        esp32_wifi_wdev_tsf_reset(s);
        memset(s->mem, 0, sizeof(s->mem));
        s->raw_interrupt = 0;
        s->dma_inlink_address = 0;
        s->receive_queue_address = 0;
        s->receive_queue_count = 0;
        qemu_set_irq(s->irq, 0);
        Esp32_WLAN_reset(s);
    }
    if (s->mac_reset_asserted != old_asserted) {
        esp32_wifi_wdev_tsf_set_clock(s, esp32_wifi_mac_enabled(s));
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
    if (s->peer_channel < 1 || s->peer_channel > 14) {
        error_setg(errp, "ESP32 Wi-Fi peer channel must be in range 1 to 14");
        return;
    }
    if (s->peer_password &&
        (strlen(s->peer_password) < 8 || strlen(s->peer_password) > 63)) {
        error_setg(errp, "ESP32 Wi-Fi peer password must contain 8 to 63 bytes");
        return;
    }
    s->dma_inlink_address = 0;
    s->fe_pbus_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    esp32_wifi_fe_pbus_complete, s);
    s->fe_iq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                  esp32_wifi_fe_iq_complete, s);
    s->txdc_pbus_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                      esp32_wifi_txdc_pbus_complete, s);

    memory_region_init_io(&s->iomem, OBJECT(dev), &esp32_wifi_ops, s,
                          TYPE_ESP32_WIFI, ESP32_WIFI_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    memory_region_init_io(&s->rxctrl_iomem, OBJECT(dev),
                          &esp32_wifi_rxctrl_ops, s,
                          "esp32-wifi-rxctrl", ESP32_WIFI_RXCTRL_SIZE);
    sysbus_init_mmio(sbd, &s->rxctrl_iomem);
    memory_region_init_io(&s->fe_iomem, OBJECT(dev), &esp32_wifi_fe_ops, s,
                          "esp32-fe-txrx-control",
                          ESP32_WIFI_FE_SIZE);
    sysbus_init_mmio(sbd, &s->fe_iomem);
    memory_region_init_io(&s->fe2_iomem, OBJECT(dev), &esp32_wifi_fe2_ops, s,
                          "esp32-fe2-control", ESP32_WIFI_FE2_SIZE);
    sysbus_init_mmio(sbd, &s->fe2_iomem);
    memory_region_init_io(&s->txdc_pbus_iomem, OBJECT(dev),
                          &esp32_wifi_txdc_pbus_ops, s,
                          "esp32-txdc-pbus", 4);
    sysbus_init_mmio(sbd, &s->txdc_pbus_iomem);
    s->rfpll_freq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       esp32_wifi_rfpll_complete, s);
    memory_region_init_io(&s->rfpll_freq_iomem, OBJECT(dev),
                          &esp32_wifi_rfpll_ops, s,
                          "esp32-wifi-rfpll-frequency", 4);
    sysbus_init_mmio(sbd, &s->rfpll_freq_iomem);
    memory_region_init_io(&s->phy_bt_iomem, OBJECT(dev),
                          &esp32_wifi_phy_bt_ops, s,
                          "esp32-phy-bt-ifs", 4);
    sysbus_init_mmio(sbd, &s->phy_bt_iomem);
    for (unsigned i = 0; i < ESP32_WIFI_COEX_WORD_COUNT; i++) {
        char *name = g_strdup_printf("esp32-wifi-coex-%u", i);
        memory_region_init_io(&s->phy_coex_iomem[i], OBJECT(dev),
                              &esp32_wifi_phy_coex_ops,
                              &s->phy_coex_regs[i], name, 4);
        g_free(name);
        sysbus_init_mmio(sbd, &s->phy_coex_iomem[i]);
    }
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(dev, esp32_wifi_clock_input,
                            ESP32_WIFI_CLOCK_GPIO, 32);
    qdev_init_gpio_in_named(dev, esp32_wifi_core_reset_input,
                            ESP32_WIFI_RESET_GPIO, 32);
    qdev_init_gpio_out_named(dev, s->rfpll_tune_out,
                             ESP32_WIFI_RFPLL_TUNE_GPIO,
                             ESP32_WIFI_RFPLL_TUNE_GPIO_COUNT);
    memset(s->mem,0,sizeof(s->mem));
    Esp32_WLAN_setup_ap(dev, s);

}

static void esp32_wifi_unrealize(DeviceState *dev)
{
    Esp32WifiState *s = ESP32_WIFI(dev);

    Esp32_WLAN_cleanup(s);
    timer_del(s->fe_pbus_timer);
    timer_free(s->fe_pbus_timer);
    s->fe_pbus_timer = NULL;
    timer_del(s->fe_iq_timer);
    timer_free(s->fe_iq_timer);
    s->fe_iq_timer = NULL;
    timer_del(s->txdc_pbus_timer);
    timer_free(s->txdc_pbus_timer);
    s->txdc_pbus_timer = NULL;
    timer_del(s->rfpll_freq_timer);
    timer_free(s->rfpll_freq_timer);
    s->rfpll_freq_timer = NULL;
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
