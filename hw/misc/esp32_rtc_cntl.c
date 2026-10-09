/*
 * ESP32 RTC_CNTL (RTC block controller) device
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/misc/esp32_reg.h"
#include "hw/misc/esp32_rtc_cntl.h"
#include "hw/nvram/esp32_efuse.h"

/* TRM v5.8 Register 9.11: documented power bits; 29, 25 and 22..0 reserved. */
#define ESP32_RTC_ANA_CONF_MASK 0xdd800000u
#define ESP32_RTC_ANA_CONF_RESET 0x00800000u

#define ESP32_ANA_I2C_BASE       0x6000e000u
#define ESP32_ANA_CONFIG_ADDR    0x6000e044u
#define ESP32_ANA_I2C_HOSTS      8
#define ESP32_ANA_I2C_BUSY       BIT(25)
#define ESP32_ANA_I2C_WRITE      BIT(24)
#define ESP32_APLL_BLOCK         0x6d
#define ESP32_APLL_CAL_END       BIT(7)
#define ESP32_APLL_CAL_DELAY_NS  10000 /* explicitly unverified approximation */

static bool esp32_apll_bus_enabled(Esp32RtcCntlState *s)
{
    /* ANA_CONFIG resets the analog-I2C host; OPTIONS0 powers its bias bus. */
    return !(s->ana_config_reg & BIT(14)) && !(s->options0_reg & BIT(18));
}

static uint32_t esp32_apll_frequency_from_regs(Esp32RtcCntlState *s)
{
    Esp32EfuseState *efuse = esp32_efuse_find();
    bool rev0 = !efuse || !(efuse->efuse_rd.blk0[3] & BIT(15));
    return esp32_apll_compute_hz(s->xtal_apb_freq,
                                 s->apll_analog[9], s->apll_analog[8],
                                 s->apll_analog[7], s->apll_analog[4] & 0x1f,
                                 rev0);
}

static void esp32_apll_calibration_timer(void *opaque)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);
    s->apll_calibrating = false;
    if (s->apll_cal_valid && esp32_rtc_apll_enabled(s)) {
        s->apll_analog[3] |= ESP32_APLL_CAL_END;
    }
    esp32_rtc_update_clk(s);
}

static uint64_t esp32_ana_i2c_read(void *opaque, hwaddr addr, unsigned size);
static void esp32_ana_i2c_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size);

static void esp32_ana_i2c_complete(Esp32RtcCntlState *s)
{
    uint32_t cmd = s->ana_i2c_last_cmd;
    unsigned host = s->ana_i2c_pending_host;
    unsigned block = (cmd >> 0) & 0xff;
    unsigned reg = (cmd >> 8) & 0xff;
    uint8_t data = (cmd >> 16) & 0xff;

    block = cmd & 0xff;
    reg = (cmd >> 8) & 0xff;
    data = (cmd >> 16) & 0xff;
    if (block != ESP32_APLL_BLOCK || host != 3) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32: unsupported analog-I2C host %u block 0x%02x\n",
                      host, block);
        s->ana_i2c_cmd[host] = 0;
        return;
    }
    if (!esp32_apll_bus_enabled(s)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32: APLL analog-I2C access while bus is reset\n");
        s->ana_i2c_cmd[host] = 0;
        return;
    }
    if (reg >= ARRAY_SIZE(s->apll_analog)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32: invalid APLL analog register 0x%02x\n", reg);
        s->ana_i2c_cmd[host] = 0;
        return;
    }

    if (cmd & ESP32_ANA_I2C_WRITE) {
        uint8_t old = s->apll_analog[reg];
        static const uint8_t write_mask[] = {
            0xff, 0x7f, 0xff, 0x00, 0xdf, 0x7f, 0x1f, 0x3f, 0xff, 0xff,
        };
        s->apll_analog[reg] = (old & ~write_mask[reg]) |
                              (data & write_mask[reg]);
        if (reg == 0) {
            bool started = !(old & BIT(5)) &&
                           (s->apll_analog[reg] & BIT(5));
            bool reset_asserted = !(s->apll_analog[reg] & BIT(4));
            if (reset_asserted) {
                s->apll_analog[3] &= ~ESP32_APLL_CAL_END;
                s->apll_calibrating = false;
                timer_del(s->apll_cal_timer);
            }
            if (started) {
                s->apll_cal_valid = (s->apll_analog[0] & BIT(4)) &&
                                    (s->apll_analog[5] & BIT(6)) &&
                                    esp32_apll_frequency_from_regs(s) != 0;
                s->apll_calibrating = s->apll_cal_valid;
                if (s->apll_calibrating) {
                    if (!s->apll_model_warned) {
                        qemu_log_mask(LOG_UNIMP,
                                      "esp32: APLL CAL_END timing (10 us) and analog CAP/UDF/OVF are unverified model approximations\n");
                        s->apll_model_warned = true;
                    }
                    s->apll_cal_deadline_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                              ESP32_APLL_CAL_DELAY_NS;
                    timer_mod(s->apll_cal_timer, s->apll_cal_deadline_ns);
                }
            }
            if (started || reset_asserted) {
                esp32_rtc_update_clk(s);
            }
        } else if (reg == 4 || reg == 5 || reg == 7 || reg == 8 || reg == 9) {
            if (old != s->apll_analog[reg]) {
                /* Reprogramming a live analog field invalidates lock state. */
                s->apll_analog[3] &= ~ESP32_APLL_CAL_END;
                s->apll_calibrating = false;
                timer_del(s->apll_cal_timer);
                if (reg == 5 && (s->apll_analog[5] & BIT(6))) {
                    /* Releasing RSTB requires a new START pulse. */
                    s->apll_cal_valid = false;
                }
                esp32_rtc_update_clk(s);
            }
        }
    } else {
        data = s->apll_analog[reg] & 0xff;
    }
    s->ana_i2c_cmd[host] = ((uint32_t)data << 16) | (cmd & 0xffff);
}

static void esp32_ana_i2c_timer(void *opaque)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);
    if (!s->ana_i2c_pending) {
        return;
    }
    s->ana_i2c_pending = false;
    esp32_ana_i2c_complete(s);
}

static uint64_t esp32_ana_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);
    if (addr == 0x44) {
        return s->ana_config_reg;
    }
    return esp32_ana_i2c_read(opaque, addr, size);
}

static void esp32_ana_mmio_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);
    if (addr == 0x44) {
        s->ana_config_reg = value & (0x3ffu << 8);
        if (s->ana_config_reg & BIT(14)) {
            if (s->ana_i2c_pending) {
                s->ana_i2c_pending = false;
                timer_del(s->ana_i2c_timer);
                s->ana_i2c_cmd[s->ana_i2c_pending_host] = 0;
            }
        }
        return;
    }
    esp32_ana_i2c_write(opaque, addr, value, size);
}

static uint64_t esp32_ana_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);
    unsigned host = addr / 4;
    if (host >= ESP32_ANA_I2C_HOSTS) {
        return 0;
    }
    return s->ana_i2c_cmd[host];
}

static void esp32_ana_i2c_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);
    unsigned host = addr / 4;
    if (host >= ESP32_ANA_I2C_HOSTS) {
        return;
    }
    if (s->ana_i2c_pending) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32: analog-I2C command issued while host is busy\n");
        return;
    }
    s->ana_i2c_last_cmd = value & 0x01ffffff;
    s->ana_i2c_pending_host = host;
    s->ana_i2c_cmd[host] = s->ana_i2c_last_cmd | ESP32_ANA_I2C_BUSY;
    s->ana_i2c_pending = true;
    timer_mod(s->ana_i2c_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
}

static const MemoryRegionOps esp32_ana_mmio_ops = {
    .read = esp32_ana_mmio_read,
    .write = esp32_ana_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

uint32_t esp32_apll_compute_hz(uint32_t xtal_hz, unsigned sdm0, unsigned sdm1,
                               unsigned sdm2, unsigned odiv, bool rev0)
{
    if (sdm0 > 255 || sdm1 > 255 || sdm2 > 63 || odiv > 31) {
        return 0;
    }
    if (rev0 && (sdm0 || sdm1)) {
        /* ESP32 erratum CLK-3.7: these fields are ignored by revision 0,
         * rather than making an otherwise valid APLL configuration fail. */
        sdm0 = sdm1 = 0;
    }
    if (xtal_hz < 2000000 || xtal_hz > 40000000) {
        return 0;
    }
    uint64_t cfg = ((uint64_t)(4 + sdm2) << 16) |
                   ((uint64_t)sdm1 << 8) | sdm0;
    uint64_t num = (uint64_t)xtal_hz * cfg;
    if (num <= (uint64_t)350000000 << 16 ||
        num >= (uint64_t)500000000 << 16) {
        return 0;
    }
    uint64_t fout = num / ((uint64_t)65536 * 2 * (odiv + 2));
    /* The TRM's overview gives 16..128 MHz, but its coefficient formula
     * permits lower outputs at large ODIV. Official ESP-IDF clk_tree_ll.h
     * consequently supports 5.303031..125 MHz. Do not invent a 16 MHz gate. */
    return (uint32_t)fout;
}

bool esp32_rtc_apll_enabled(const Esp32RtcCntlState *s)
{
    if (s->ana_conf_reg & BIT(23)) {
        return false;
    }
    if (s->ana_conf_reg & BIT(24)) {
        return true;
    }
    /* Both force bits are 0: the PLL follows the system (TRM 7.2.7). Sleep
     * is not modeled, and the active system keeps the PLL enabled. */
    return true;
}

uint32_t esp32_rtc_get_apll_hz(Esp32RtcCntlState *s)
{
    uint64_t numerator, denominator;
    if (!esp32_rtc_get_apll_rate(s, &numerator, &denominator)) {
        return 0;
    }
    return numerator / denominator;
}

bool esp32_rtc_get_apll_rate(Esp32RtcCntlState *s, uint64_t *numerator,
                             uint64_t *denominator)
{
    if (!esp32_rtc_apll_enabled(s) ||
        !(s->apll_analog[3] & ESP32_APLL_CAL_END)) {
        return false;
    }
    unsigned sdm0 = s->apll_analog[9];
    unsigned sdm1 = s->apll_analog[8];
    unsigned sdm2 = s->apll_analog[7];
    unsigned odiv = s->apll_analog[4] & 0x1f;
    Esp32EfuseState *efuse = esp32_efuse_find();
    bool rev0 = !efuse || !(efuse->efuse_rd.blk0[3] & BIT(15));
    if (rev0) {
        sdm0 = sdm1 = 0;
    }
    uint64_t cfg = ((uint64_t)(4 + sdm2) << 16) |
                   ((uint64_t)sdm1 << 8) | sdm0;
    uint64_t num = (uint64_t)s->xtal_apb_freq * cfg;
    uint64_t den = (uint64_t)65536 * 2 * (odiv + 2);
    if (!esp32_apll_compute_hz(s->xtal_apb_freq, sdm0, sdm1,
                              sdm2, odiv, false)) {
        return false;
    }
    uint64_t gcd = num;
    uint64_t rem = den;
    while (rem) {
        uint64_t next = gcd % rem;
        gcd = rem;
        rem = next;
    }
    *numerator = num / gcd;
    *denominator = den / gcd;
    return true;
}

static uint64_t esp32_rtc_cntl_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_RTC_CNTL_OPTIONS0:
        r = s->options0_reg;
        break;
    case A_RTC_CNTL_TIME_UPDATE:
        r = R_RTC_CNTL_TIME_UPDATE_VALID_MASK;
        break;
    case A_RTC_CNTL_TIME0:
        r = s->time_reg & UINT32_MAX;
        break;
    case A_RTC_CNTL_TIME1:
        r = s->time_reg >> 32;
        break;

    case A_RTC_CNTL_RESET_STATE:
        r = FIELD_DP32(r, RTC_CNTL_RESET_STATE, RESET_CAUSE_PROCPU, s->reset_cause[0]);
        r = FIELD_DP32(r, RTC_CNTL_RESET_STATE, RESET_CAUSE_APPCPU, s->reset_cause[1]);
        r = FIELD_DP32(r, RTC_CNTL_RESET_STATE, PROCPU_STAT_VECTOR_SEL, s->stat_vector_sel[0]);
        r = FIELD_DP32(r, RTC_CNTL_RESET_STATE, APPCPU_STAT_VECTOR_SEL, s->stat_vector_sel[1]);
        break;

    case A_RTC_CNTL_ANA_CONF:
        r = s->ana_conf_reg;
        break;

    case A_RTC_CNTL_STORE0:
    case A_RTC_CNTL_STORE1:
    case A_RTC_CNTL_STORE2:
    case A_RTC_CNTL_STORE3:
        r = s->scratch_reg[(addr - A_RTC_CNTL_STORE0) / 4];
        break;

    case A_RTC_CNTL_CLK_CONF:
        r = FIELD_DP32(r, RTC_CNTL_CLK_CONF, SOC_CLK_SEL, s->soc_clk);
        r = FIELD_DP32(r, RTC_CNTL_CLK_CONF, FAST_CLK_RTC_SEL, s->rtc_fastclk);
        r = FIELD_DP32(r, RTC_CNTL_CLK_CONF, ANA_CLK_RTC_SEL, s->rtc_slowclk);
        break;

    case A_RTC_CNTL_SW_CPU_STALL:
        r = s->sw_cpu_stall_reg;
        break;

    case A_RTC_CNTL_STORE4:
    case A_RTC_CNTL_STORE5:
    case A_RTC_CNTL_STORE6:
    case A_RTC_CNTL_STORE7:
        r = s->scratch_reg[(addr - A_RTC_CNTL_STORE4) / 4 + 4];
        break;
    }
    return r;
}

static void esp32_rtc_cntl_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(opaque);
    switch (addr) {
    case A_RTC_CNTL_OPTIONS0: {
        bool i2c_bus_was_enabled = esp32_apll_bus_enabled(s);
        if (value & R_RTC_CNTL_OPTIONS0_SW_SYS_RESET_MASK) {
            s->reset_cause[0] = ESP32_SW_SYS_RESET;
            s->reset_cause[1] = ESP32_SW_SYS_RESET;
            qemu_irq_pulse(s->dig_reset_req);
            value &= ~(R_RTC_CNTL_OPTIONS0_SW_SYS_RESET_MASK);
        }
        if (value & R_RTC_CNTL_OPTIONS0_SW_APPCPU_RESET_MASK) {
            s->reset_cause[1] = ESP32_SW_CPU_RESET;
            qemu_irq_pulse(s->cpu_reset_req[1]);
            value &= ~(R_RTC_CNTL_OPTIONS0_SW_APPCPU_RESET_MASK);
        }
        if (value & R_RTC_CNTL_OPTIONS0_SW_PROCPU_RESET_MASK) {
            s->reset_cause[0] = ESP32_SW_CPU_RESET;
            qemu_irq_pulse(s->cpu_reset_req[0]);
            value &= ~(R_RTC_CNTL_OPTIONS0_SW_PROCPU_RESET_MASK);
        }
        s->options0_reg = value;
        esp32_rtc_update_cpu_stall(s);
        if (i2c_bus_was_enabled && !esp32_apll_bus_enabled(s) &&
            s->ana_i2c_pending) {
            s->ana_i2c_pending = false;
            timer_del(s->ana_i2c_timer);
            s->ana_i2c_cmd[s->ana_i2c_pending_host] = 0;
        }
        break;
        }

    case A_RTC_CNTL_TIME_UPDATE:
        if (value & R_RTC_CNTL_TIME_UPDATE_UPDATE_MASK) {
            s->time_reg = muldiv64(
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->time_base_ns,
                s->rtc_slowclk_freq, NANOSECONDS_PER_SECOND);
        }
        break;

    case A_RTC_CNTL_RESET_STATE:
        s->stat_vector_sel[0] = FIELD_EX32(value, RTC_CNTL_RESET_STATE,
                                           PROCPU_STAT_VECTOR_SEL);
        s->stat_vector_sel[1] = FIELD_EX32(value, RTC_CNTL_RESET_STATE,
                                           APPCPU_STAT_VECTOR_SEL);
        break;

    case A_RTC_CNTL_ANA_CONF:
        s->ana_conf_reg = value & ESP32_RTC_ANA_CONF_MASK;
        if (s->ana_conf_reg & BIT(23)) {
            s->apll_analog[3] &= ~ESP32_APLL_CAL_END;
            s->apll_calibrating = false;
            timer_del(s->apll_cal_timer);
        }
        esp32_rtc_update_clk(s);
        break;

    case A_RTC_CNTL_STORE0:
    case A_RTC_CNTL_STORE1:
    case A_RTC_CNTL_STORE2:
    case A_RTC_CNTL_STORE3:
        s->scratch_reg[(addr - A_RTC_CNTL_STORE0) / 4] = value;
        break;

    case A_RTC_CNTL_CLK_CONF:
        if (FIELD_EX32(value, RTC_CNTL_CLK_CONF, ANA_CLK_RTC_SEL) >
            ESP32_SLOW_CLK_8MD256) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32.rtc_cntl: reserved RTC slow-clock selector\n");
            break;
        }
        s->soc_clk = FIELD_EX32(value, RTC_CNTL_CLK_CONF, SOC_CLK_SEL);
        s->rtc_fastclk = FIELD_EX32(value, RTC_CNTL_CLK_CONF, FAST_CLK_RTC_SEL);
        s->rtc_slowclk = FIELD_EX32(value, RTC_CNTL_CLK_CONF, ANA_CLK_RTC_SEL);
        esp32_rtc_update_clk(s);
        break;

    case A_RTC_CNTL_SW_CPU_STALL:
        s->sw_cpu_stall_reg = value;
        esp32_rtc_update_cpu_stall(s);
        break;

    case A_RTC_CNTL_STORE4:
    case A_RTC_CNTL_STORE5:
    case A_RTC_CNTL_STORE6:
    case A_RTC_CNTL_STORE7:
        s->scratch_reg[(addr - A_RTC_CNTL_STORE4) / 4 + 4] = value;
        break;
    }
}

void esp32_rtc_update_cpu_stall(Esp32RtcCntlState* s)
{
    uint32_t procpu_stall = (FIELD_EX32(s->sw_cpu_stall_reg, RTC_CNTL_SW_CPU_STALL, PROCPU_C1) << 2) |
                            (FIELD_EX32(s->options0_reg, RTC_CNTL_OPTIONS0, SW_STALL_PROCPU_C0));

    uint32_t appcpu_stall = (FIELD_EX32(s->sw_cpu_stall_reg, RTC_CNTL_SW_CPU_STALL, APPCPU_C1) << 2) |
                            (FIELD_EX32(s->options0_reg, RTC_CNTL_OPTIONS0, SW_STALL_APPCPU_C0));

    const uint32_t stall_magic_val = 0x86;

    s->cpu_stall_state[0] = procpu_stall == stall_magic_val;
    s->cpu_stall_state[1] = appcpu_stall == stall_magic_val;

    qemu_set_irq(s->cpu_stall_req[0], s->cpu_stall_state[0]);
    qemu_set_irq(s->cpu_stall_req[1], s->cpu_stall_state[1]);
}

void esp32_rtc_update_clk(Esp32RtcCntlState* s)
{
    const uint32_t slowclk_freq[] = {150000, 32768, 8000000/256};
    const uint32_t fastclk_freq[] = {s->xtal_apb_freq / 4, 8000000};
    if ((unsigned)s->rtc_slowclk >= ARRAY_SIZE(slowclk_freq) ||
        (unsigned)s->rtc_fastclk >= ARRAY_SIZE(fastclk_freq)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32.rtc_cntl: invalid RTC clock selector state\n");
        return;
    }
    s->rtc_slowclk_freq = slowclk_freq[s->rtc_slowclk];
    s->rtc_fastclk_freq = fastclk_freq[s->rtc_fastclk];
    qemu_irq_pulse(s->clk_update);
}

static const MemoryRegionOps esp32_rtc_cntl_ops = {
    .read =  esp32_rtc_cntl_read,
    .write = esp32_rtc_cntl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_rtc_cntl_reset_hold(Object *obj, ResetType type)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(obj);

    s->time_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->time_reg = 0;
    s->ana_conf_reg = ESP32_RTC_ANA_CONF_RESET;
    s->ana_config_reg = 0x3ffu << 8;
    s->options0_reg = 0;
    s->sw_cpu_stall_reg = 0;
    memset(s->scratch_reg, 0, sizeof(s->scratch_reg));
    memset(s->ana_i2c_cmd, 0, sizeof(s->ana_i2c_cmd));
    memset(s->apll_analog, 0, sizeof(s->apll_analog));
    s->ana_i2c_pending = false;
    s->ana_i2c_last_cmd = 0;
    s->ana_i2c_pending_host = 0;
    s->apll_calibrating = false;
    s->apll_cal_valid = false;
    s->apll_cal_deadline_ns = 0;
    s->soc_clk = ESP32_SOC_CLK_XTAL;
    s->rtc_fastclk = ESP32_FAST_CLK_8M;
    s->rtc_slowclk = ESP32_SLOW_CLK_RC;
    timer_del(s->ana_i2c_timer);
    timer_del(s->apll_cal_timer);
}

static void esp32_rtc_cntl_reset_exit(Object *obj, ResetType type)
{
    /* A force-power-down reset must also invalidate downstream sources. */
    esp32_rtc_update_clk(ESP32_RTC_CNTL(obj));
}

static void esp32_rtc_cntl_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_rtc_cntl_init(Object *obj)
{
    Esp32RtcCntlState *s = ESP32_RTC_CNTL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_rtc_cntl_ops, s,
                          TYPE_ESP32_RTC_CNTL, ESP32_RTC_CNTL_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    memory_region_init_io(&s->ana_i2c_iomem, obj, &esp32_ana_mmio_ops, s,
                          "esp32.ana_i2c", 0x48);
    sysbus_init_mmio(sbd, &s->ana_i2c_iomem);
    s->ana_i2c_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    esp32_ana_i2c_timer, s);
    s->apll_cal_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                     esp32_apll_calibration_timer, s);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->dig_reset_req, ESP32_RTC_DIG_RESET_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->cpu_reset_req[0], ESP32_RTC_CPU_RESET_GPIO, ESP32_CPU_COUNT);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->cpu_stall_req[0], ESP32_RTC_CPU_STALL_GPIO, ESP32_CPU_COUNT);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->clk_update, ESP32_RTC_CLK_UPDATE_GPIO, 1);

    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        s->reset_cause[i] = ESP32_POWERON_RESET;
        s->stat_vector_sel[i] = true;
    }

    s->rtc_slowclk = ESP32_SLOW_CLK_RC;
    s->rtc_fastclk = ESP32_FAST_CLK_8M;
    s->soc_clk = ESP32_SOC_CLK_XTAL;
    s->pll_apb_freq = 80000000;
    s->ana_conf_reg = ESP32_RTC_ANA_CONF_RESET;
    s->ana_config_reg = 0x3ffu << 8;
    esp32_rtc_update_clk(s);
}

static Property esp32_rtc_cntl_properties[] = {
    /* One crystal reference feeds RTC, APLL, APB and peripherals. */
    DEFINE_PROP_UINT32("xtal-apb-freq", Esp32RtcCntlState, xtal_apb_freq,
                       40000000),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_rtc_cntl_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_rtc_cntl_reset_hold;
    rc->phases.exit = esp32_rtc_cntl_reset_exit;
    dc->realize = esp32_rtc_cntl_realize;
    dc->vmsd = &vmstate_esp32_rtc_cntl;
    device_class_set_props(dc, esp32_rtc_cntl_properties);
}

static const TypeInfo esp32_rtc_cntl_info = {
    .name = TYPE_ESP32_RTC_CNTL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32RtcCntlState),
    .instance_init = esp32_rtc_cntl_init,
    .class_init = esp32_rtc_cntl_class_init
};

static void esp32_rtc_cntl_register_types(void)
{
    type_register_static(&esp32_rtc_cntl_info);
}

type_init(esp32_rtc_cntl_register_types)
