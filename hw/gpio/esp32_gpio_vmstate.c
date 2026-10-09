/* ESP32 GPIO migration state shared with isolated migration tests.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/gpio/esp32_gpio.h"

static int gpio_post_load(void *opaque, int version)
{
    Esp32GpioState *s = opaque;
    if (version >= 2) {
        for (unsigned i = 0; i < ARRAY_SIZE(s->clkout_timer); i++) {
            bool enabled = s->clkout_num[i] && s->clkout_den[i];
            if (enabled != timer_pending(s->clkout_timer[i]) ||
                !s->clkout_den[i] ||
                (!s->clkout_num[i] && s->clkout_den[i] != 1) ||
                !s->clkout_half_div[i] ||
                s->clkout_phase[i] >= s->clkout_half_div[i]) {
                return -EINVAL;
            }
            if (enabled) {
                __uint128_t half = (__uint128_t)s->clkout_den[i] *
                                   NANOSECONDS_PER_SECOND;
                uint64_t divisor = 2 * s->clkout_num[i];
                if (s->clkout_num[i] > UINT64_MAX / 2 ||
                    (__uint128_t)s->clkout_num[i] >
                        (__uint128_t)160000000 * s->clkout_den[i] ||
                    s->clkout_den[i] > UINT32_MAX ||
                    s->clkout_half_div[i] != divisor ||
                    s->clkout_half_whole[i] != half / divisor ||
                    s->clkout_half_rem[i] != half % divisor) {
                    return -EINVAL;
                }
            } else if (s->clkout_half_whole[i] ||
                       s->clkout_half_rem[i] ||
                       s->clkout_half_div[i] != 1 ||
                       s->clkout_phase[i]) {
                return -EINVAL;
            }
            s->clkout_context[i].owner = s;
            s->clkout_context[i].index = i;
        }
    } else {
        s->apll_clkout_num = 0;
        s->apll_clkout_den = 1;
        for (unsigned i = 0; i < ARRAY_SIZE(s->clkout_timer); i++) {
            timer_del(s->clkout_timer[i]);
            s->clkout_num[i] = 0;
            s->clkout_den[i] = 1;
            s->clkout_half_whole[i] = 0;
            s->clkout_half_rem[i] = 0;
            s->clkout_half_div[i] = 1;
            s->clkout_phase[i] = 0;
            s->clkout_level[i] = false;
            s->clkout_context[i].owner = s;
            s->clkout_context[i].index = i;
        }
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->external); i++) {
        if (s->external[i] > ESP32_PAD_X) {
            return -EINVAL;
        }
    }
    for (unsigned i = 0; i < ESP32_GPIO_OUTPUTS_V1; i++) {
        if (s->peripheral_value[i] > 1 || s->peripheral_enable[i] > 1 ||
            s->peripheral_open_drain[i] > 1 || s->peripheral_known[i] > 1) {
            return -EINVAL;
        }
    }
    for (unsigned i = 0; i < ESP32_GPIO_PADS; i++) {
        if (s->input_sample[i] > 1) {
            return -EINVAL;
        }
    }
    s->resolving = false;
    s->pending_pads = 0;
    s->pending_all_inputs = false;
    esp32_gpio_rebuild_outputs(s);
    return 0;
}

const VMStateDescription vmstate_esp32_gpio = {
    .name = TYPE_ESP32_GPIO, .version_id = 2, .minimum_version_id = 1,
    .post_load = gpio_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Esp32GpioState, 0x600 / 4),
        VMSTATE_UINT32_ARRAY(mux, Esp32GpioState, 0xa0 / 4),
        VMSTATE_UINT8_ARRAY(peripheral_value, Esp32GpioState,
                            ESP32_GPIO_OUTPUTS_V1),
        VMSTATE_UINT8_ARRAY(peripheral_enable, Esp32GpioState,
                            ESP32_GPIO_OUTPUTS_V1),
        VMSTATE_UINT8_ARRAY(peripheral_open_drain, Esp32GpioState,
                            ESP32_GPIO_OUTPUTS_V1),
        VMSTATE_UINT8_ARRAY(peripheral_known, Esp32GpioState,
                            ESP32_GPIO_OUTPUTS_V1),
        VMSTATE_UINT8_ARRAY(external, Esp32GpioState,
                            ESP32_GPIO_PADS * ESP32_GPIO_EXT_DRIVERS),
        VMSTATE_UINT8_ARRAY(input_sample, Esp32GpioState, ESP32_GPIO_PADS),
        VMSTATE_UINT64_V(apll_clkout_num, Esp32GpioState, 2),
        VMSTATE_UINT64_V(apll_clkout_den, Esp32GpioState, 2),
        VMSTATE_ARRAY_OF_POINTER(clkout_timer, Esp32GpioState, 3, 2,
                                 vmstate_info_timer, QEMUTimer *),
        VMSTATE_UINT64_ARRAY_V(clkout_num, Esp32GpioState, 3, 2),
        VMSTATE_UINT64_ARRAY_V(clkout_den, Esp32GpioState, 3, 2),
        VMSTATE_UINT64_ARRAY_V(clkout_half_whole, Esp32GpioState, 3, 2),
        VMSTATE_UINT64_ARRAY_V(clkout_half_rem, Esp32GpioState, 3, 2),
        VMSTATE_UINT64_ARRAY_V(clkout_half_div, Esp32GpioState, 3, 2),
        VMSTATE_UINT64_ARRAY_V(clkout_phase, Esp32GpioState, 3, 2),
        VMSTATE_BOOL_ARRAY_V(clkout_level, Esp32GpioState, 3, 2),
        VMSTATE_END_OF_LIST()
    },
};
