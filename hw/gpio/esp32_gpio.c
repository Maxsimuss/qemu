/*
 * ESP32 digital pads, IO MUX and GPIO matrix.
 *
 * References: ESP32 TRM v5.8 chapter 6; ESP32 Series Datasheet pin inventory.
 * The resolved pad is distinct from GPIO_OUT and includes electrical release,
 * GPIO/peripheral OE, open-drain operation, pulls and external strong drivers.
 * Unsupported peripheral/analog drives remain unknown, never synthetic values.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"

#define R(s, a) ((s)->regs[(a) / 4])
#define GPIO_OUT 0x04
#define GPIO_OUT1 0x10
#define GPIO_ENABLE 0x20
#define GPIO_ENABLE1 0x2c
#define GPIO_STATUS 0x44
#define GPIO_STATUS1 0x50
#define GPIO_PIN 0x88
#define GPIO_INPUT_SEL 0x130
#define GPIO_OUTPUT_SEL 0x530
#define IOMUX_IE BIT(9)
#define IOMUX_PU BIT(8)
#define IOMUX_PD BIT(7)
#define PAD_VALID UINT64_C(0xff0eefffff)
#define PAD_OUTPUT UINT64_C(0x030eefffff)

static const uint8_t mux_offset[ESP32_GPIO_PADS] = {
    0x44, 0x88, 0x40, 0x84, 0x48, 0x6c, 0x60, 0x64,
    0x68, 0x54, 0x58, 0x5c, 0x34, 0x38, 0x30, 0x3c,
    0x4c, 0x50, 0x70, 0x74, 0x78, 0x7c, 0x80, 0x8c,
    0x90, 0x24, 0x28, 0x2c, 0, 0, 0, 0,
    0x1c, 0x20, 0x14, 0x18, 0x04, 0x08, 0x0c, 0x10,
};

/* QFN48 ESP32-D0WDQ6 terminal inventory, datasheet section 2. GPIO labels
 * alias the resolved digital node. Other signal domains remain explicitly
 * unknown, rather than being substituted with fabricated binary signals. */
typedef struct Esp32Terminal {
    const char *name;
    int gpio;
    const char *domain;
} Esp32Terminal;

static const Esp32Terminal terminals[] = {
    { "VDDA", -1, "power-unmodeled" },
    { "LNA_IN", -1, "rf-unmodeled" },
    { "VDD3P3", -1, "power-unmodeled" },
    { "VDD3P3", -1, "power-unmodeled" },
    { "SENSOR_VP", 36, "digital;analog-unmodeled" },
    { "SENSOR_CAPP", 37, "digital;analog-unmodeled" },
    { "SENSOR_CAPN", 38, "digital;analog-unmodeled" },
    { "SENSOR_VN", 39, "digital;analog-unmodeled" },
    { "CHIP_PU", -1, "reset-terminal-unmodeled" },
    { "VDET_1", 34, "digital;analog-unmodeled" },
    { "VDET_2", 35, "digital;analog-unmodeled" },
    { "32K_XP", 32, "digital;oscillator-unmodeled" },
    { "32K_XN", 33, "digital;oscillator-unmodeled" },
    { "GPIO25", 25, "digital;analog-unmodeled" },
    { "GPIO26", 26, "digital;analog-unmodeled" },
    { "GPIO27", 27, "digital;analog-unmodeled" },
    { "MTMS", 14, "digital;analog-unmodeled" },
    { "MTDI", 12, "digital;analog-unmodeled" },
    { "VDD3P3_RTC", -1, "power-unmodeled" },
    { "MTCK", 13, "digital;analog-unmodeled" },
    { "MTDO", 15, "digital;analog-unmodeled" },
    { "GPIO2", 2, "digital;analog-unmodeled" },
    { "GPIO0", 0, "digital;analog-unmodeled" },
    { "GPIO4", 4, "digital;analog-unmodeled" },
    { "GPIO16", 16, "digital" },
    { "VDD_SDIO", -1, "power-unmodeled" },
    { "GPIO17", 17, "digital" },
    { "SD_DATA_2", 9, "digital" },
    { "SD_DATA_3", 10, "digital" },
    { "SD_CMD", 11, "digital" },
    { "SD_CLK", 6, "digital" },
    { "SD_DATA_0", 7, "digital" },
    { "SD_DATA_1", 8, "digital" },
    { "GPIO5", 5, "digital" },
    { "GPIO18", 18, "digital" },
    { "GPIO23", 23, "digital" },
    { "VDD3P3_CPU", -1, "power-unmodeled" },
    { "GPIO19", 19, "digital" },
    { "GPIO22", 22, "digital" },
    { "U0RXD", 3, "digital" },
    { "U0TXD", 1, "digital" },
    { "GPIO21", 21, "digital" },
    { "VDDA", -1, "power-unmodeled" },
    { "XTAL_N", -1, "oscillator-unmodeled" },
    { "XTAL_P", -1, "oscillator-unmodeled" },
    { "VDDA", -1, "power-unmodeled" },
    { "CAP2", -1, "analog-unmodeled" },
    { "CAP1", -1, "analog-unmodeled" },
    { "GND_EP", -1, "power-unmodeled" },
};

static bool valid_pad(unsigned pad)
{
    return pad < ESP32_GPIO_PADS && (PAD_VALID & (UINT64_C(1) << pad));
}

static uint32_t mux_value(Esp32GpioState *s, unsigned pad)
{
    return s->mux[mux_offset[pad] / 4];
}

static unsigned mux_function(Esp32GpioState *s, unsigned pad)
{
    return (mux_value(s, pad) >> 12) & 7;
}

/* Direct peripheral functions present in TRM table 6.10-1. -1 is not a
 * modeled peripheral output: active drive remains X rather than guessed. */
static int direct_output(Esp32GpioState *s, unsigned pad, unsigned function)
{
    if (function == 2 || (function == 0 &&
        ((pad == 0 || pad == 2 || pad == 4 || pad == 5) || pad >= 16))) {
        return 256;
    }
    if (function == 1 && (pad == 0 || pad == 1 || pad == 3)) {
        unsigned shift = pad == 0 ? 0 : pad == 3 ? 4 : 8;
        unsigned source = (s->mux[0] >> shift) & 15;
        unsigned clock = s->mux[0] & 15;
        unsigned gate = (s->mux[0] >> shift) & 15;
        if (source == 6 && (shift == 0 ||
                            (clock == 6 && gate == 6))) {
            return pad == 0 ? ESP32_GPIO_CLKOUT1 :
                   pad == 3 ? ESP32_GPIO_CLKOUT2 : ESP32_GPIO_CLKOUT3;
        }
        /* TRM 6.33: CLK1 selects 0=I2S0, 15=I2S1; CLK2/3 must
         * select zero to propagate that I2S source. */
        if ((shift == 0 || gate == 0) && (clock == 0 || clock == 15)) {
            return clock == 0 ? ESP32_GPIO_MCLK0 : ESP32_GPIO_MCLK1;
        }
        return -1;
    }
    if (function == 0 && pad == 1) {
        return 14; /* U0TXD */
    }
    if (function == 1 && pad >= 6 && pad <= 11) {
        static const int spi[] = {0, 1, 2, 3, 4, 5};
        return spi[pad - 6];
    }
    if (function == 1 && pad >= 12 && pad <= 15) {
        static const int hspi[] = {9, 10, 8, 11};
        return hspi[pad - 12];
    }
    if (function == 1) {
        switch (pad) {
        case 2: return 13;
        case 4: return 12;
        case 5: return 68;
        case 18: return 63;
        case 19: return 64;
        case 21: return 65;
        case 22: return 66;
        case 23: return 67;
        }
    }
    if (function == 4 && pad == 17) {
        return 198; /* U2TXD */
    }
    return -1;
}

static void gpio_clkout_tick(void *opaque)
{
    Esp32GpioClkoutContext *ctx = opaque;
    Esp32GpioState *s = ctx->owner;
    unsigned i = ctx->index;
    static const unsigned signals[] = {
        ESP32_GPIO_CLKOUT1, ESP32_GPIO_CLKOUT2, ESP32_GPIO_CLKOUT3,
    };
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t delay = s->clkout_half_whole[i];

    esp32_gpio_set_peripheral_output(s, signals[i], !s->clkout_level[i],
                                     true, false);
    s->clkout_phase[i] += s->clkout_half_rem[i];
    if (s->clkout_phase[i] >= s->clkout_half_div[i]) {
        s->clkout_phase[i] -= s->clkout_half_div[i];
        delay++;
    }
    timer_mod(s->clkout_timer[i], now + MAX(delay, 1));
}

void esp32_gpio_set_apll_clkout(Esp32GpioState *s, uint64_t numerator,
                                uint64_t denominator)
{
    static const unsigned signals[] = {
        ESP32_GPIO_CLKOUT1, ESP32_GPIO_CLKOUT2, ESP32_GPIO_CLKOUT3,
    };
    uint64_t divisor;
    uint64_t half_num;
    uint64_t now;

    if (!s) {
        return;
    }
    s->apll_clkout_num = numerator;
    s->apll_clkout_den = denominator;
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    for (unsigned i = 0; i < ARRAY_SIZE(signals); i++) {
        static const unsigned shifts[] = { 0, 4, 8 };
        unsigned source = (s->mux[0] >> shifts[i]) & 15;
        bool apll_selected = source == 6 &&
            (shifts[i] == 0 || ((s->mux[0] & 15) == 6 && source == 6));
        uint64_t output_num = apll_selected ? numerator : 0;
        uint64_t output_den = apll_selected ? denominator : 1;
        if (s->clkout_num[i] == output_num &&
            s->clkout_den[i] == output_den) {
            continue;
        }
        timer_del(s->clkout_timer[i]);
        s->clkout_num[i] = output_num;
        s->clkout_den[i] = output_den;
        s->clkout_phase[i] = 0;
        s->clkout_level[i] = false;
        if (!output_num || !output_den) {
            s->clkout_half_whole[i] = 0;
            s->clkout_half_rem[i] = 0;
            s->clkout_half_div[i] = 1;
            esp32_gpio_set_peripheral_output(s, signals[i], false, true, false);
            continue;
        }
        divisor = 2 * output_num;
        half_num = output_den * NANOSECONDS_PER_SECOND;
        s->clkout_half_whole[i] = half_num / divisor;
        s->clkout_half_rem[i] = half_num % divisor;
        s->clkout_half_div[i] = divisor;
        /* The first edge is scheduled at floor(H); carry the fractional
         * half-period from that first interval into the next one. */
        s->clkout_phase[i] = s->clkout_half_rem[i];
        timer_mod(s->clkout_timer[i], now + MAX(s->clkout_half_whole[i], 1));
    }
}

static Esp32PadLevel merge_drive(Esp32PadLevel a, Esp32PadLevel b)
{
    if (a == ESP32_PAD_Z) {
        return b;
    }
    if (b == ESP32_PAD_Z) {
        return a;
    }
    return a == b ? a : ESP32_PAD_X;
}

/* Trace loss invalidates a pin-based test. Report it as a host failure instead
 * of letting the guest continue with an apparently successful capture. */
static void trace_flush(Esp32GpioState *s)
{
    if (s->trace && (fflush(s->trace) == EOF || ferror(s->trace))) {
        error_report("ESP32 pin trace '%s': %s", s->pin_trace,
                     strerror(errno ? errno : EIO));
        exit(EXIT_FAILURE);
    }
}

static int selected_output(Esp32GpioState *s, unsigned pad)
{
    unsigned function = mux_function(s, pad);
    unsigned output;

    if (function != 2) {
        return direct_output(s, pad, function);
    }
    output = R(s, GPIO_OUTPUT_SEL + pad * 4) & 511;
    /* 257/258 are model-private native-clock nodes, not matrix signals. */
    return output <= 256 ? output : -1;
}

static void trace_pad(Esp32GpioState *s, unsigned pad, Esp32PadLevel level,
                      Esp32PadLevel drive, Esp32PadLevel external, bool force)
{
    static const char value[] = "01zx";
    int64_t now;

    if (!s->trace || !valid_pad(pad)) {
        return;
    }
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (now < s->trace_last_virtual) {
        fprintf(s->trace, "$comment virtual time restored to %" PRId64
                " ns; next timestamps are offset $end\n", now);
        s->trace_epoch += s->trace_last_virtual - now;
    }
    s->trace_last_virtual = now;
    now += s->trace_epoch;
    if (now != s->trace_time) {
        fprintf(s->trace, "#%" PRId64 "\n", now);
        s->trace_time = now;
    }
    if (force || s->resolved[pad] != level) {
        fprintf(s->trace, "%cP%u\n", value[level], pad);
    }
    if (force || s->drive[pad] != drive) {
        fprintf(s->trace, "%cD%u\n", value[drive], pad);
    }
    if (force || s->external_resolved[pad] != external) {
        fprintf(s->trace, "%cE%u\n", value[external], pad);
    }
}

static void update_irq(Esp32GpioState *s)
{
    uint32_t irq = 0, nmi = 0, app_irq = 0, app_nmi = 0;

    for (unsigned pad = 0; pad < ESP32_GPIO_PADS; pad++) {
        if ((R(s, pad < 32 ? GPIO_STATUS : GPIO_STATUS1) >> (pad % 32)) & 1) {
            unsigned ena = (R(s, GPIO_PIN + pad * 4) >> 13) & 31;
            irq |= ena & BIT(2);
            nmi |= ena & BIT(3);
            app_irq |= ena & BIT(0);
            app_nmi |= ena & BIT(1);
        }
    }
    qemu_set_irq(s->irq, !!irq);
    qemu_set_irq(s->nmi, !!nmi);
    qemu_set_irq(s->app_irq, !!app_irq);
    qemu_set_irq(s->app_nmi, !!app_nmi);
}

static unsigned sample_input(Esp32GpioState *s, unsigned pad)
{
    return valid_pad(pad) && (mux_value(s, pad) & IOMUX_IE) &&
           s->resolved[pad] == ESP32_PAD_HIGH;
}

static unsigned matrix_input(Esp32GpioState *s, unsigned signal)
{
    uint32_t cfg = R(s, GPIO_INPUT_SEL + signal * 4);
    unsigned pin = cfg & 63;
    unsigned sample = 0;

    if (cfg & BIT(7)) {
        if (pin == 0x38) {
            sample = 1;
        } else if (valid_pad(pin) && mux_function(s, pin) == 2) {
            sample = sample_input(s, pin);
        }
    } else {
        /* Direct input bypasses the matrix; unspecified inputs default low. */
        if (signal == 14 && mux_function(s, 3) == 0) {
            sample = sample_input(s, 3);
        }
    }
    return sample ^ !!(cfg & BIT(6));
}

static void cache_routes(Esp32GpioState *s)
{
    if (s->routes_valid) {
        return;
    }
    memset(s->output_pads, 0, sizeof(s->output_pads));
    memset(s->input_signals, 0, sizeof(s->input_signals));
    for (unsigned pad = 0; pad < ESP32_GPIO_PADS; pad++) {
        int signal = selected_output(s, pad);
        if (valid_pad(pad) && signal >= 0 && signal < ESP32_GPIO_OUTPUTS) {
            s->output_pads[signal] |= UINT64_C(1) << pad;
        }
    }
    for (unsigned signal = 0; signal < ESP32_GPIO_SIGNALS; signal++) {
        uint32_t cfg = R(s, GPIO_INPUT_SEL + signal * 4);
        unsigned pad = cfg & 63;
        if (!(cfg & BIT(7))) {
            if (signal != 14) {
                continue;
            }
            pad = 3; /* native UART0 RX */
        }
        if (valid_pad(pad)) {
            s->input_signals[pad][signal / 64] |= UINT64_C(1) << (signal % 64);
        }
    }
    s->routes_valid = true;
}

static void gpio_resolve_pads(Esp32GpioState *s, uint64_t pads, bool all_inputs)
{
    unsigned passes = 0;

    s->pending_pads |= pads;
    s->pending_all_inputs |= all_inputs;
    if (s->resolving || (!s->pending_pads && !s->pending_all_inputs)) {
        return;
    }
    s->resolving = true;
    do {
        uint64_t inputs[ESP32_GPIO_SIGNALS / 64] = {0};
        uint64_t pending = s->pending_pads;
        bool refresh_inputs = s->pending_all_inputs;
        s->pending_pads = 0;
        s->pending_all_inputs = false;
        cache_routes(s);
        while (pending) {
            unsigned pad = ctz64(pending);
            pending &= pending - 1;
            uint32_t mux = mux_value(s, pad);
            uint32_t cfg = R(s, GPIO_OUTPUT_SEL + pad * 4);
            unsigned function = mux_function(s, pad);
            int signal = selected_output(s, pad);
            bool register_oe = (R(s, pad < 32 ? GPIO_ENABLE : GPIO_ENABLE1) >>
                                (pad % 32)) & 1;
            bool oe = register_oe;
            bool level = 0, known = true, od = false;
            Esp32PadLevel internal = ESP32_PAD_Z;
            Esp32PadLevel external = ESP32_PAD_Z;
            Esp32PadLevel resolved;
            unsigned sampled, old_sample;

            if (signal == 256) {
                level = (R(s, pad < 32 ? GPIO_OUT : GPIO_OUT1) >> (pad % 32)) & 1;
            } else if (signal >= ESP32_GPIO_CLKOUT1 &&
                       signal <= ESP32_GPIO_CLKOUT3) {
                level = s->clkout_level[signal - ESP32_GPIO_CLKOUT1];
                oe = true;
            } else if (signal >= 0 && signal < ESP32_GPIO_OUTPUTS) {
                known = s->peripheral_known[signal];
                level = s->peripheral_value[signal];
                od = s->peripheral_open_drain[signal];
                if (function != 2 || !(cfg & BIT(10))) {
                    oe = known ? s->peripheral_enable[signal] : true;
                }
            } else {
                known = false;
                if (function != 2) {
                    oe = true;
                }
            }
            if (function == 2) {
                level ^= !!(cfg & BIT(9));
                oe ^= !!(cfg & BIT(11));
            }
            od |= !!(R(s, GPIO_PIN + pad * 4) & BIT(2));
            if (valid_pad(pad) && (PAD_OUTPUT & (UINT64_C(1) << pad)) && oe) {
                internal = !known ? ESP32_PAD_X :
                           od && level ? ESP32_PAD_Z :
                           level ? ESP32_PAD_HIGH : ESP32_PAD_LOW;
            }
            for (unsigned driver = 0; driver < ESP32_GPIO_EXT_DRIVERS; driver++) {
                external = merge_drive(external,
                    s->external[driver * ESP32_GPIO_PADS + pad]);
            }
            resolved = merge_drive(internal, external);
            if (resolved == ESP32_PAD_Z && pad < 34) {
                if ((mux & (IOMUX_PU | IOMUX_PD)) == (IOMUX_PU | IOMUX_PD)) {
                    resolved = ESP32_PAD_X;
                } else if (mux & IOMUX_PU) {
                    resolved = ESP32_PAD_HIGH;
                } else if (mux & IOMUX_PD) {
                    resolved = ESP32_PAD_LOW;
                }
            }
            trace_pad(s, pad, resolved, internal, external, false);
            bool changed = s->resolved[pad] != resolved;
            s->resolved[pad] = resolved;
            s->drive[pad] = internal;
            s->external_resolved[pad] = external;
            if (changed) {
                qemu_set_irq(s->pad_level[pad], resolved);
                GPtrArray *listeners = s->pad_listeners[pad];
                for (unsigned i = 0; listeners && i < listeners->len; i++) {
                    qemu_set_irq(g_ptr_array_index(listeners, i), resolved);
                }
            }
            sampled = sample_input(s, pad);
            old_sample = s->input_sample[pad];
            s->input_sample[pad] = sampled;
            if (sampled != old_sample) {
                for (unsigned word = 0; word < ARRAY_SIZE(inputs); word++) {
                    inputs[word] |= s->input_signals[pad][word];
                }
            }
            if (!s->resetting) {
                unsigned type = (R(s, GPIO_PIN + pad * 4) >> 7) & 7;
                if ((type == 1 && !old_sample && sampled) ||
                    (type == 2 && old_sample && !sampled) ||
                    (type == 3 && old_sample != sampled) ||
                    (type == 4 && !sampled) || (type == 5 && sampled)) {
                    R(s, pad < 32 ? GPIO_STATUS : GPIO_STATUS1) |= BIT(pad % 32);
                }
            }
        }
        for (unsigned word = 0; word < ARRAY_SIZE(inputs); word++) {
            uint64_t signals = refresh_inputs ? UINT64_MAX : inputs[word];
            while (signals) {
                unsigned signal = word * 64 + ctz64(signals);
                unsigned sample = matrix_input(s, signal);
                signals &= signals - 1;
                if (s->signal_sample[signal] != sample) {
                    s->signal_sample[signal] = sample;
                    qemu_set_irq(s->signal_in[signal], sample);
                }
            }
        }
        if (++passes == 256 && (s->pending_pads || s->pending_all_inputs)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32.gpio: combinational pad oscillation\n");
            break;
        }
    } while (s->pending_pads || s->pending_all_inputs);
    s->resolving = false;
    update_irq(s);
    trace_flush(s);
}

/* Register changes can alter any route, pull, input enable or interrupt.
 * A full refresh also publishes constants/inverted inputs without pad edges. */
static void gpio_resolve(Esp32GpioState *s)
{
    s->routes_valid = false;
    gpio_resolve_pads(s, (UINT64_C(1) << ESP32_GPIO_PADS) - 1, true);
}

void esp32_gpio_rebuild_outputs(Esp32GpioState *s)
{
    gpio_resolve(s);
}

void esp32_gpio_set_peripheral_output(Esp32GpioState *s, unsigned signal,
                                     bool level, bool enable, bool open_drain)
{
    if (!s || signal >= ESP32_GPIO_OUTPUTS) {
        return;
    }
    if (signal >= ESP32_GPIO_CLKOUT1) {
        unsigned index = signal - ESP32_GPIO_CLKOUT1;
        if (s->clkout_level[index] != level) {
            s->clkout_level[index] = level;
            cache_routes(s);
            gpio_resolve_pads(s, s->output_pads[signal], false);
        }
        return;
    }
    if (s->peripheral_known[signal] && s->peripheral_value[signal] == level &&
        s->peripheral_enable[signal] == enable &&
        s->peripheral_open_drain[signal] == open_drain) {
        return;
    }
    s->peripheral_known[signal] = 1;
    s->peripheral_value[signal] = level;
    s->peripheral_enable[signal] = enable;
    s->peripheral_open_drain[signal] = open_drain;
    cache_routes(s);
    gpio_resolve_pads(s, s->output_pads[signal], false);
}

void esp32_gpio_set_peripheral_input(Esp32GpioState *s, unsigned signal,
                                    qemu_irq input)
{
    assert(signal < ESP32_GPIO_SIGNALS);
    s->signal_in[signal] = input;
    s->signal_sample[signal] = matrix_input(s, signal);
    qemu_set_irq(input, s->signal_sample[signal]);
}

void esp32_gpio_set_peripheral_unknown(Esp32GpioState *s, unsigned signal)
{
    if (!s || signal >= ESP32_GPIO_OUTPUTS) {
        return;
    }
    if (s->peripheral_known[signal]) {
        s->peripheral_known[signal] = 0;
        cache_routes(s);
        gpio_resolve_pads(s, s->output_pads[signal], false);
    }
}

void esp32_gpio_set_external_drive(Esp32GpioState *s, unsigned pad,
                                  unsigned driver, Esp32PadLevel level)
{
    assert(pad < ESP32_GPIO_PADS && driver < ESP32_GPIO_EXT_DRIVERS);
    assert(level <= ESP32_PAD_X);
    unsigned index = driver * ESP32_GPIO_PADS + pad;
    if (s->external[index] != level) {
        s->external[index] = level;
        gpio_resolve_pads(s, UINT64_C(1) << pad, false);
    }
}

Esp32PadLevel esp32_gpio_get_pad(Esp32GpioState *s, unsigned pad)
{
    assert(pad < ESP32_GPIO_PADS);
    return s->resolved[pad];
}

void esp32_gpio_add_pad_listener(Esp32GpioState *s, unsigned pad, qemu_irq sink)
{
    assert(valid_pad(pad));
    if (!s->pad_listeners[pad]) {
        s->pad_listeners[pad] = g_ptr_array_new();
    }
    g_ptr_array_add(s->pad_listeners[pad], sink);
    qemu_set_irq(sink, s->resolved[pad]);
}

void esp32_gpio_remove_pad_listener(Esp32GpioState *s, unsigned pad, qemu_irq sink)
{
    assert(valid_pad(pad));
    if (s->pad_listeners[pad]) {
        g_ptr_array_remove(s->pad_listeners[pad], sink);
    }
}

void esp32_gpio_add_routing_listener(Esp32GpioState *s, qemu_irq sink)
{
    if (!s->routing_listeners) {
        s->routing_listeners = g_ptr_array_new();
    }
    g_ptr_array_add(s->routing_listeners, sink);
}

void esp32_gpio_remove_routing_listener(Esp32GpioState *s, qemu_irq sink)
{
    if (s->routing_listeners) {
        g_ptr_array_remove(s->routing_listeners, sink);
    }
}

bool esp32_gpio_output_is_routed(Esp32GpioState *s, unsigned signal)
{
    if (!s) {
        return false;
    }
    if (signal >= ESP32_GPIO_OUTPUTS) {
        return false;
    }
    cache_routes(s);
    return s->output_pads[signal] != 0;
}

static void routing_changed(Esp32GpioState *s)
{
    for (unsigned i = 0; s->routing_listeners && i < s->routing_listeners->len; i++) {
        qemu_irq_pulse(g_ptr_array_index(s->routing_listeners, i));
    }
}

static void external_drive(void *opaque, int n, int level)
{
    Esp32GpioState *s = opaque;

    if (level < ESP32_PAD_LOW || level > ESP32_PAD_X) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32.gpio: invalid external pad level %d\n", level);
        return;
    }
    esp32_gpio_set_external_drive(s, n % 40, n / 40, level);
}

static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32GpioState *s = opaque;
    uint32_t result = 0;

    if (addr == A_GPIO_STRAP) {
        return s->strap_mode;
    }
    if (addr == 0x3c || addr == 0x40) {
        unsigned first = addr == 0x40 ? 32 : 0;
        for (unsigned pad = first; pad < MIN(first + 32, 40); pad++) {
            result |= sample_input(s, pad) << (pad - first);
        }
        return result;
    }
    if (addr >= 0x60 && addr <= 0x84) {
        unsigned reg = (addr - 0x60) / 4;
        unsigned core = reg % 5 < 2 ? 1 : reg % 5 < 4 ? 0 : 2;
        bool nmi = reg % 5 == 1 || reg % 5 == 3;
        unsigned first = reg >= 5 ? 32 : 0;
        for (unsigned pad = first; pad < MIN(first + 32, 40); pad++) {
            unsigned enable_bit = core == 2 ? 4 : (core == 0 ? 2 : 0) + nmi;
            if ((R(s, GPIO_PIN + pad * 4) >> (13 + enable_bit)) & 1) {
                result |= BIT(pad - first);
            }
        }
        return result & R(s, first ? GPIO_STATUS1 : GPIO_STATUS);
    }
    if (addr < sizeof(s->regs)) {
        return R(s, addr);
    }
    qemu_log_mask(LOG_UNIMP,
                  "esp32.gpio: unimplemented read 0x%" HWADDR_PRIx "\n", addr);
    return 0;
}

static void esp32_gpio_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    Esp32GpioState *s = opaque;
    uint32_t mask;

    switch (addr) {
    case GPIO_OUT: case GPIO_OUT1:
    case GPIO_ENABLE: case GPIO_ENABLE1:
    case GPIO_STATUS: case GPIO_STATUS1:
        mask = (addr == GPIO_OUT1 || addr == GPIO_ENABLE1 ||
                addr == GPIO_STATUS1) ? 255 : UINT32_MAX;
        R(s, addr) = value & mask;
        break;
    case 0x08: case 0x14: case 0x24: case 0x30: case 0x48: case 0x54:
        mask = (addr == 0x14 || addr == 0x30 || addr == 0x54) ? 255 : UINT32_MAX;
        R(s, addr - 4) |= value & mask;
        break;
    case 0x0c: case 0x18: case 0x28: case 0x34: case 0x4c: case 0x58:
        mask = (addr == 0x18 || addr == 0x34 || addr == 0x58) ? 255 : UINT32_MAX;
        R(s, addr - 8) &= ~(value & mask);
        break;
    case GPIO_PIN ... GPIO_PIN + 39 * 4:
        R(s, addr) = value & 0x3ff84;
        break;
    case GPIO_INPUT_SEL ... GPIO_INPUT_SEL + 255 * 4:
        R(s, addr) = value & 255;
        break;
    case GPIO_OUTPUT_SEL ... GPIO_OUTPUT_SEL + 39 * 4:
        R(s, addr) = value & 4095;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "esp32.gpio: unimplemented write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n",
                      addr, value);
        return;
    }
    gpio_resolve(s);
}

static uint64_t mux_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32GpioState *s = opaque;
    return s->mux[addr / 4];
}

static void mux_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    Esp32GpioState *s = opaque;
    bool valid = addr == 0;

    for (unsigned pad = 0; pad < 40; pad++) {
        valid |= valid_pad(pad) && addr == mux_offset[pad];
    }
    if (!valid) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32.iomux: invalid register 0x%" HWADDR_PRIx "\n", addr);
        return;
    }
    uint32_t mask = addr == 0 ? 0xfff : 0x7fff;
    for (unsigned pad = 34; pad < 40; pad++) {
        if (addr == mux_offset[pad]) {
            mask &= ~(IOMUX_PU | IOMUX_PD | (3 << 10) |
                      BIT(0) | BIT(2) | BIT(3) | (3 << 5));
        }
    }
    s->mux[addr / 4] = value & mask;
    if (addr == 0) {
        esp32_gpio_set_apll_clkout(s, s->apll_clkout_num,
                                   s->apll_clkout_den);
    }
    gpio_resolve(s);
    routing_changed(s);
}

static const MemoryRegionOps gpio_ops = {
    .read = esp32_gpio_read, .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps mux_ops = {
    .read = mux_read, .write = mux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void esp32_gpio_reset_hold(Object *obj, ResetType type)
{
    Esp32GpioState *s = ESP32_GPIO(obj);

    s->resetting = true;
    for (unsigned i = 0; i < ARRAY_SIZE(s->clkout_timer); i++) {
        timer_del(s->clkout_timer[i]);
        s->clkout_num[i] = 0;
        s->clkout_den[i] = 1;
        s->clkout_half_whole[i] = 0;
        s->clkout_half_rem[i] = 0;
        s->clkout_half_div[i] = 1;
        s->clkout_phase[i] = 0;
        s->clkout_level[i] = false;
    }
    s->apll_clkout_num = 0;
    s->apll_clkout_den = 1;
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->mux, 0, sizeof(s->mux));
    for (unsigned pad = 0; pad < 40; pad++) {
        if (!valid_pad(pad)) {
            continue;
        }
        uint32_t mux = pad < 34 ? 2 << 10 : 0;
        if (pad < 24) {
            mux |= IOMUX_IE;
            mux |= (pad == 2 || pad == 4 || pad == 12 || pad == 13) ? IOMUX_PD :
                   (pad <= 15 ? IOMUX_PU : 0);
        }
        s->mux[mux_offset[pad] / 4] = mux;
        R(s, GPIO_OUTPUT_SEL + pad * 4) = 256;
    }
    gpio_resolve(s);
    s->resetting = false;
}

static void esp32_gpio_realize(DeviceState *dev, Error **errp)
{
    Esp32GpioState *s = ESP32_GPIO(dev);

    memset(s->external, ESP32_PAD_Z, sizeof(s->external));
    memset(s->resolved, ESP32_PAD_X, sizeof(s->resolved));
    memset(s->signal_sample, 255, sizeof(s->signal_sample));
    if (s->pin_trace) {
        s->trace = fopen(s->pin_trace, "w");
        if (!s->trace) {
            error_setg_errno(errp, errno, "Cannot open ESP32 pin trace '%s'",
                             s->pin_trace);
            return;
        }
        fprintf(s->trace, "$version QEMU ESP32 resolved digital pads $end\n"
                "$timescale 1 ns $end\n$scope module esp32 $end\n"
                "$comment Digital GPIO inventory only; analog, RF, supply, crystal\n"
                "and unmodeled peripheral outputs are not complete. X denotes unknown\n"
                "or contention; Z denotes electrical release. $end\n");
        for (unsigned pad = 0; pad < 40; pad++) {
            if (valid_pad(pad)) {
                fprintf(s->trace, "$var wire 1 P%u gpio%u $end\n"
                        "$var wire 1 D%u gpio%u_esp_drive $end\n"
                        "$var wire 1 E%u gpio%u_external_drive $end\n",
                        pad, pad, pad, pad, pad, pad);
            }
        }
        for (unsigned pin = 0; pin < ARRAY_SIZE(terminals); pin++) {
            const Esp32Terminal *terminal = &terminals[pin];
            if (terminal->gpio >= 0) {
                fprintf(s->trace, "$var wire 1 P%d pin%02u_%s $end\n",
                        terminal->gpio, pin + 1, terminal->name);
            } else {
                fprintf(s->trace, "$var wire 1 U%u pin%02u_%s_UNMODELED $end\n",
                        pin + 1, pin + 1, terminal->name);
            }
            fprintf(s->trace, "$comment pin%02u domain=%s $end\n",
                    pin + 1, terminal->domain);
        }
        fprintf(s->trace, "$upscope $end\n$enddefinitions $end\n#0\n");
        for (unsigned pin = 0; pin < ARRAY_SIZE(terminals); pin++) {
            if (terminals[pin].gpio < 0) {
                fprintf(s->trace, "xU%u\n", pin + 1);
            }
        }
        s->trace_time = 0;
    }
    esp32_gpio_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
    for (unsigned pad = 0; pad < 40; pad++) {
        trace_pad(s, pad, s->resolved[pad], s->drive[pad],
                  s->external_resolved[pad], true);
    }
    trace_flush(s);
}

static void esp32_gpio_init(Object *obj)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    object_property_set_int(obj, "strap_mode", ESP32_STRAP_MODE_FLASH_BOOT, &error_fatal);
    memory_region_init_io(&s->iomem, obj, &gpio_ops, s, TYPE_ESP32_GPIO, 0x1000);
    memory_region_init_io(&s->io_mux, obj, &mux_ops, s, "esp32.iomux", sizeof(s->mux));
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_mmio(sbd, &s->io_mux);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->nmi);
    sysbus_init_irq(sbd, &s->app_irq);
    sysbus_init_irq(sbd, &s->app_nmi);
    for (unsigned i = 0; i < ARRAY_SIZE(s->clkout_timer); i++) {
        s->clkout_context[i].owner = s;
        s->clkout_context[i].index = i;
        s->clkout_timer[i] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                          gpio_clkout_tick,
                                          &s->clkout_context[i]);
        s->clkout_half_div[i] = 1;
    }
    qdev_init_gpio_in_named(DEVICE(s), external_drive, "pad-drive",
                            ESP32_GPIO_PADS * ESP32_GPIO_EXT_DRIVERS);
    qdev_init_gpio_out_named(DEVICE(s), s->pad_level, "pad-level", 40);
}

static void esp32_gpio_finalize(Object *obj)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    if (s->trace) {
        fclose(s->trace);
    }
    for (unsigned pad = 0; pad < ESP32_GPIO_PADS; pad++) {
        if (s->pad_listeners[pad]) {
            g_ptr_array_unref(s->pad_listeners[pad]);
        }
    }
    if (s->routing_listeners) {
        g_ptr_array_unref(s->routing_listeners);
    }
}

static Property gpio_properties[] = {
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, 0),
    DEFINE_PROP_STRING("pin-trace", Esp32GpioState, pin_trace),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    dc->realize = esp32_gpio_realize;
    dc->vmsd = &vmstate_esp32_gpio;
    rc->phases.hold = esp32_gpio_reset_hold;
    device_class_set_props(dc, gpio_properties);
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState), .instance_init = esp32_gpio_init,
    .instance_finalize = esp32_gpio_finalize, .class_init = esp32_gpio_class_init,
    .class_size = sizeof(Esp32GpioClass),
};

static void esp32_gpio_register_types(void)
{
    type_register_static(&esp32_gpio_info);
}
type_init(esp32_gpio_register_types)
