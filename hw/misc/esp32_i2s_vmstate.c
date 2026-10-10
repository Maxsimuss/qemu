/* ESP32 I2S VMState definitions, shared with isolated migration tests.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "migration/vmstate.h"
#include "hw/misc/esp32_i2s.h"
#include "hw/irq.h"

bool esp32_i2s_mode_supported(Esp32I2SState *s)
{
    /* Unsupported modes must not silently emit standard-I2S samples. */
    return !(s->regs[0xb4 / 4] & 3) &&
           (!(s->regs[0xac / 4] & BIT(21)) || s->apll_hz) &&
           !(s->regs[0x1c / 4] & 0xffffff) && !(s->regs[0xa4 / 4] & BIT(0)) &&
           !(s->regs[0x98 / 4] & (BIT(0) | BIT(9))) &&
           !(s->regs[0x9c / 4] & BIT(2)) &&
           !(s->regs[0x60 / 4] & (BIT(4) | BIT(5) | BIT(13))) &&
           ((s->regs[0xa0 / 4] & BIT(3)) || (s->regs[0xa0 / 4] & 7) <= 1) &&
           ((s->regs[0xa0 / 4] & BIT(7)) || ((s->regs[0xa0 / 4] >> 4) & 7) <= 1);
}

bool esp32_i2s_state_valid(Esp32I2SState *s)
{
    if (!s->apll_denominator ||
        s->apll_hz != s->apll_numerator / s->apll_denominator ||
        (!s->apll_numerator && s->apll_denominator != 1) ||
        s->apll_hz > 160000000 ||
        s->apll_numerator > UINT64_MAX / (2 * 63) ||
        s->apll_denominator > UINT32_MAX ||
        (__uint128_t)s->apll_numerator >
            (__uint128_t)160000000 * s->apll_denominator) {
        return false;
    }
    if (s->tx.fifo_head >= 64 || s->rx.fifo_head >= 64 ||
        s->tx.fifo_count > 64 || s->rx.fifo_count > 64 ||
        s->tx.bit >= 64 || s->rx.bit > 64 ||
        s->tx.clock_bit >= 64 || s->rx.clock_bit >= 64 ||
        s->tx.descriptor_offset > 4095 || s->rx.descriptor_offset > 4095 ||
        s->tx.descriptor_limit > 4095 || s->rx.descriptor_limit > 4095 ||
        s->tx.paused_ns < 0 || s->rx.paused_ns < 0 || s->mclk_paused_ns < 0) {
        return false;
    }
    for (unsigned i = 0; i < ESP32_I2S_FIFO_WORDS; i++) {
        if ((s->tx.fifo_flags[i] | s->rx.fifo_flags[i]) &
            ~7) {
            return false;
        }
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->input_level); i++) {
        if (s->input_level[i] > 1) {
            return false;
        }
    }
    return true;
}

int esp32_i2s_post_load(void *opaque, int version)
{
    Esp32I2SState *s = opaque;
    if (version < 2) {
        s->apll_numerator = s->apll_hz;
        s->apll_denominator = 1;
    }
    if (!esp32_i2s_state_valid(s)) {
        return -EINVAL;
    }
    if (!esp32_i2s_mode_supported(s)) {
        /* GPIO VMState already preserves unknown drive nodes. Replacing them
         * with ordinary I2S values would fabricate capability on restoration. */
        qemu_set_irq(s->irq, !!(s->regs[0x0c / 4] & s->regs[0x14 / 4]));
        return 0;
    }
    bool parallel = s->regs[0xa8 / 4] & BIT(5);
    unsigned period = (s->regs[0xa8 / 4] & BIT(1)) ? 4 : 2;
    unsigned phase = (s->tx.phase - 1) % period;
    unsigned slot = (s->regs[0xa8 / 4] & BIT(2)) ? phase % 2 :
                    phase / ((s->regs[0xa8 / 4] & BIT(1)) ? 2 : 1);
    uint32_t data = s->tx.frame[slot] >> 8;
    for (unsigned bit = 0; bit < 24; bit++) {
        unsigned signal = (s->controller ? 166 : 140) + bit;
        bool level = parallel ? (data >> bit) & 1 : s->tx.data_level;
        esp32_gpio_set_peripheral_output(s->gpio, signal, level,
            s->tx.started && (parallel ? s->tx.frame_valid : bit == 23), false);
    }
    esp32_gpio_set_peripheral_output(s->gpio, s->controller ? 24 : 23,
        s->tx.clock_level, s->tx.started &&
        !(s->regs[0x08 / 4] & BIT(6)), false);
    esp32_gpio_set_peripheral_output(s->gpio, s->controller ? 26 : 25,
        s->tx.ws_level, s->tx.started &&
        !(s->regs[0x08 / 4] & BIT(6)), false);
    esp32_gpio_set_peripheral_output(s->gpio, s->controller ? 164 : 27,
        s->rx.clock_level, s->rx.started &&
        !(s->regs[0x08 / 4] & BIT(7)), false);
    esp32_gpio_set_peripheral_output(s->gpio, s->controller ? 165 : 28,
        s->rx.ws_level, s->rx.started &&
        !(s->regs[0x08 / 4] & BIT(7)), false);
    uint32_t cfg = s->regs[0xac / 4];
    bool clock = (s->enabled || s->mclk_paused_ns > 0) &&
                 (cfg & BIT(20)) && (cfg & 255) >= 2 &&
                 (((cfg >> 14) & 63) || !((cfg >> 8) & 63)) &&
                 (!(cfg & BIT(21)) || s->apll_hz) &&
                 esp32_gpio_output_is_routed(s->gpio,
                     ESP32_GPIO_MCLK0 + s->controller);
    esp32_gpio_set_peripheral_output(s->gpio,
        ESP32_GPIO_MCLK0 + s->controller, s->mclk_level, clock, false);
    qemu_set_irq(s->irq, !!(s->regs[0x0c / 4] & s->regs[0x14 / 4]));
    return 0;
}

static const VMStateDescription vmstate_channel = {
    .name = "esp32.i2s/channel", .version_id = 3, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, Esp32I2SChannel),
        VMSTATE_UINT32_ARRAY(fifo, Esp32I2SChannel, 64),
        VMSTATE_UINT32_ARRAY(fifo_descriptor, Esp32I2SChannel, 64),
        VMSTATE_UINT8_ARRAY(fifo_flags, Esp32I2SChannel, 64),
        VMSTATE_UINT32(fifo_head, Esp32I2SChannel),
        VMSTATE_UINT32(fifo_count, Esp32I2SChannel),
        VMSTATE_UINT32(descriptor, Esp32I2SChannel),
        VMSTATE_UINT32_ARRAY(descriptor_words, Esp32I2SChannel, 3),
        VMSTATE_UINT32(descriptor_offset, Esp32I2SChannel),
        VMSTATE_UINT32(descriptor_limit, Esp32I2SChannel),
        VMSTATE_UINT32(dma_words, Esp32I2SChannel),
        VMSTATE_UINT32_ARRAY(frame, Esp32I2SChannel, 2),
        VMSTATE_UINT32(shift, Esp32I2SChannel),
        VMSTATE_UINT32(bit, Esp32I2SChannel),
        VMSTATE_UINT32(slot, Esp32I2SChannel),
        VMSTATE_UINT32(phase, Esp32I2SChannel),
        VMSTATE_UINT32(clock_bit, Esp32I2SChannel),
        VMSTATE_UINT64(clock_remainder, Esp32I2SChannel),
        VMSTATE_INT64(deadline, Esp32I2SChannel),
        VMSTATE_UINT64_V(analytic_frame_data, Esp32I2SChannel, 2),
        VMSTATE_UINT64_V(analytic_frame_ws, Esp32I2SChannel, 2),
        VMSTATE_UINT64_V(analytic_frame_remainder, Esp32I2SChannel, 2),
        VMSTATE_UINT64_V(analytic_frame_half_num, Esp32I2SChannel, 2),
        VMSTATE_UINT64_V(analytic_frame_half_den, Esp32I2SChannel, 2),
        VMSTATE_INT64_V(analytic_frame_origin, Esp32I2SChannel, 2),
        VMSTATE_UINT8_V(analytic_frame_count, Esp32I2SChannel, 2),
        VMSTATE_BOOL(link_active, Esp32I2SChannel),
        VMSTATE_BOOL(descriptor_loaded, Esp32I2SChannel),
        VMSTATE_BOOL(clock_level, Esp32I2SChannel),
        VMSTATE_BOOL(ws_level, Esp32I2SChannel),
        VMSTATE_BOOL(data_level, Esp32I2SChannel),
        VMSTATE_BOOL(frame_valid, Esp32I2SChannel),
        VMSTATE_BOOL(started, Esp32I2SChannel),
        VMSTATE_BOOL(synchronized, Esp32I2SChannel),
        VMSTATE_BOOL(previous_ws, Esp32I2SChannel),
        VMSTATE_BOOL(mono_pending, Esp32I2SChannel),
        VMSTATE_BOOL_V(analytic_clock, Esp32I2SChannel, 2),
        VMSTATE_BOOL_V(analytic_frame_pending, Esp32I2SChannel, 2),
        VMSTATE_BOOL_V(analytic_origin_falling, Esp32I2SChannel, 2),
        VMSTATE_BOOL_V(analytic_paired, Esp32I2SChannel, 3),
        VMSTATE_INT64(paused_ns, Esp32I2SChannel),
        VMSTATE_END_OF_LIST()
    },
};

const VMStateDescription vmstate_esp32_i2s = {
    .name = TYPE_ESP32_I2S, .version_id = 3, .minimum_version_id = 1,
    .post_load = esp32_i2s_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Esp32I2SState, 64),
        VMSTATE_STRUCT(tx, Esp32I2SState, 1, vmstate_channel, Esp32I2SChannel),
        VMSTATE_STRUCT(rx, Esp32I2SState, 1, vmstate_channel, Esp32I2SChannel),
        VMSTATE_TIMER_PTR(mclk_timer, Esp32I2SState),
        VMSTATE_UINT64(mclk_remainder, Esp32I2SState),
        VMSTATE_INT64(mclk_deadline, Esp32I2SState),
        VMSTATE_INT64(mclk_paused_ns, Esp32I2SState),
        VMSTATE_BOOL(mclk_level, Esp32I2SState),
        VMSTATE_BOOL(enabled, Esp32I2SState),
        VMSTATE_UINT32(apll_hz, Esp32I2SState),
        VMSTATE_UINT8_ARRAY(input_level, Esp32I2SState, 24),
        VMSTATE_UINT64_V(apll_numerator, Esp32I2SState, 2),
        VMSTATE_UINT64_V(apll_denominator, Esp32I2SState, 2),
        VMSTATE_END_OF_LIST()
    },
};
