/*
 * Original ESP32 RTC/APLL migration state.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "migration/vmstate.h"
#include "hw/misc/esp32_rtc_cntl.h"

#define ESP32_ANA_I2C_HOSTS 8
#define ESP32_ANA_I2C_BUSY  BIT(25)
#define ESP32_APLL_CAL_END  BIT(7)
#define ESP32_BBPLL_REGS    13
#define ESP32_RF_ANA_BLOCKS 7
#define ESP32_RF_ANA_REGS   16

static int esp32_rtc_cntl_post_load(void *opaque, int version_id)
{
    Esp32RtcCntlState *s = opaque;
    static const uint32_t apll_masks[] = {
        0xff, 0x7f, 0xff, 0x80, 0x1f, 0x7f, 0, 0x3f, 0xff, 0xff,
    };
    static const uint32_t bbpll_masks[ESP32_BBPLL_REGS] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0, 0,
        0xff, 0xf3, 0xff, 0xff, 0xff,
    };

    if (version_id < 3) {
        memset(s->bbpll_analog, 0, sizeof(s->bbpll_analog));
    }
    if (version_id < 4) {
        memset(s->rf_analog, 0, sizeof(s->rf_analog));
        memset(s->rf_analog_written, 0, sizeof(s->rf_analog_written));
        s->rf_analog_unknown_reset_logged = false;
    }
    if (version_id < 5) {
        s->rfpll_calibrating = false;
        s->rfpll_cal_armed = false;
        s->rfpll_cal_start_low_seen = false;
        s->rfpll_cal_start_high_seen = false;
        s->rfpll_tune_code = 0;
    }

    if (s->ana_i2c_pending != timer_pending(s->ana_i2c_timer) ||
        s->apll_calibrating != timer_pending(s->apll_cal_timer) ||
        s->rfpll_calibrating != timer_pending(s->rfpll_cal_timer)) {
        return -EINVAL;
    }
    if ((s->ana_i2c_pending &&
         (s->ana_i2c_pending_host >= ESP32_ANA_I2C_HOSTS ||
          !(s->ana_i2c_cmd[s->ana_i2c_pending_host] & ESP32_ANA_I2C_BUSY))) ||
        (s->apll_calibrating &&
         (!s->apll_cal_valid || !(s->apll_analog[0] & BIT(4)) ||
          !(s->apll_analog[5] & BIT(6)))) ||
        (s->rfpll_calibrating &&
         (!(s->rf_analog[0][0] & BIT(5)) ||
          !(s->ana_conf_reg & BIT(31)) || (s->options0_reg & BIT(18)))) ||
        (unsigned)s->soc_clk > ESP32_SOC_CLK_APLL ||
        (unsigned)s->rtc_fastclk > ESP32_FAST_CLK_8M ||
        (unsigned)s->rtc_slowclk > ESP32_SLOW_CLK_8MD256 ||
        (s->ana_config_reg & ~(0x3ffu << 8)) ||
        (s->ana_conf_reg & ~0xdd800000u)) {
        return -EINVAL;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(apll_masks); i++) {
        if (s->apll_analog[i] & ~apll_masks[i]) {
            return -EINVAL;
        }
    }
    for (unsigned i = 0; i < ARRAY_SIZE(bbpll_masks); i++) {
        if (s->bbpll_analog[i] & ~bbpll_masks[i]) {
            return -EINVAL;
        }
    }
    static const uint16_t rf_write_masks[ESP32_RF_ANA_BLOCKS] = {
        0x071f, 0x003b, 0x0090, 0x9fff, 0x0003, 0x0075, 0x06fe,
    };
    for (unsigned i = 0; i < ESP32_RF_ANA_BLOCKS; i++) {
        if (s->rf_analog_written[i] & ~rf_write_masks[i]) {
            return -EINVAL;
        }
    }
    if (!esp32_rtc_apll_powered(s) || !s->apll_cal_valid) {
        s->apll_analog[3] &= ~ESP32_APLL_CAL_END;
    }
    if (s->apll_calibrating) {
        s->apll_cal_deadline_ns = timer_expire_time_ns(s->apll_cal_timer);
    }
    if (s->rfpll_calibrating) {
        s->rfpll_cal_deadline_ns = timer_expire_time_ns(s->rfpll_cal_timer);
    }
    esp32_rtc_update_clk(s);
    esp32_rtc_update_cpu_stall(s);
    return 0;
}

const VMStateDescription vmstate_esp32_rtc_cntl = {
    .name = TYPE_ESP32_RTC_CNTL,
    .version_id = 5,
    .minimum_version_id = 1,
    .post_load = esp32_rtc_cntl_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(ana_i2c_timer, Esp32RtcCntlState),
        VMSTATE_TIMER_PTR(apll_cal_timer, Esp32RtcCntlState),
        VMSTATE_TIMER_PTR_V(rfpll_cal_timer, Esp32RtcCntlState, 5),
        VMSTATE_UINT32(ana_config_reg, Esp32RtcCntlState),
        VMSTATE_UINT32(ana_conf_reg, Esp32RtcCntlState),
        VMSTATE_UINT32(xtal_apb_freq, Esp32RtcCntlState),
        VMSTATE_UINT32(pll_apb_freq, Esp32RtcCntlState),
        VMSTATE_UINT32(soc_clk, Esp32RtcCntlState),
        VMSTATE_UINT32(rtc_fastclk, Esp32RtcCntlState),
        VMSTATE_UINT32(rtc_fastclk_freq, Esp32RtcCntlState),
        VMSTATE_UINT32(rtc_slowclk, Esp32RtcCntlState),
        VMSTATE_UINT32(rtc_slowclk_freq, Esp32RtcCntlState),
        VMSTATE_UINT32_ARRAY(ana_i2c_cmd, Esp32RtcCntlState,
                             ESP32_ANA_I2C_HOSTS),
        VMSTATE_UINT32_ARRAY(apll_analog, Esp32RtcCntlState, 10),
        VMSTATE_UINT32_ARRAY_V(bbpll_analog, Esp32RtcCntlState,
                               ESP32_BBPLL_REGS, 3),
        VMSTATE_UINT8_2DARRAY_V(rf_analog, Esp32RtcCntlState,
                                ESP32_RF_ANA_BLOCKS, ESP32_RF_ANA_REGS, 4),
        VMSTATE_UINT16_ARRAY_V(rf_analog_written, Esp32RtcCntlState,
                               ESP32_RF_ANA_BLOCKS, 4),
        VMSTATE_BOOL_V(rf_analog_unknown_reset_logged,
                       Esp32RtcCntlState, 4),
        VMSTATE_BOOL_V(rf_analog_status_unimp_logged,
                       Esp32RtcCntlState, 4),
        VMSTATE_BOOL_V(rfpll_calibrating, Esp32RtcCntlState, 5),
        VMSTATE_BOOL_V(rfpll_cal_armed, Esp32RtcCntlState, 5),
        VMSTATE_BOOL_V(rfpll_cal_start_low_seen, Esp32RtcCntlState, 5),
        VMSTATE_BOOL_V(rfpll_cal_start_high_seen, Esp32RtcCntlState, 5),
        VMSTATE_UINT8_V(rfpll_tune_code, Esp32RtcCntlState, 5),
        VMSTATE_INT64_V(rfpll_cal_deadline_ns, Esp32RtcCntlState, 5),
        VMSTATE_UINT32(ana_i2c_last_cmd, Esp32RtcCntlState),
        VMSTATE_UINT8(ana_i2c_pending_host, Esp32RtcCntlState),
        VMSTATE_BOOL(ana_i2c_pending, Esp32RtcCntlState),
        VMSTATE_BOOL(apll_calibrating, Esp32RtcCntlState),
        VMSTATE_BOOL(apll_cal_valid, Esp32RtcCntlState),
        VMSTATE_BOOL(apll_model_warned, Esp32RtcCntlState),
        VMSTATE_INT64(apll_cal_deadline_ns, Esp32RtcCntlState),
        VMSTATE_UINT32_V(options0_reg, Esp32RtcCntlState, 2),
        VMSTATE_UINT64_V(time_reg, Esp32RtcCntlState, 2),
        VMSTATE_INT64_V(time_base_ns, Esp32RtcCntlState, 2),
        VMSTATE_UINT32_V(sw_cpu_stall_reg, Esp32RtcCntlState, 2),
        VMSTATE_UINT32_ARRAY_V(scratch_reg, Esp32RtcCntlState,
                               ESP32_RTC_CNTL_SCRATCH_REG_COUNT, 2),
        VMSTATE_UINT32_ARRAY_V(reset_cause, Esp32RtcCntlState,
                               ESP32_CPU_COUNT, 2),
        VMSTATE_BOOL_ARRAY_V(stat_vector_sel, Esp32RtcCntlState,
                             ESP32_CPU_COUNT, 2),
        VMSTATE_END_OF_LIST()
    },
};
