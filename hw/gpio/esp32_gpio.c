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

static void routing_changed(Esp32GpioState *s);
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

typedef struct Esp32SerialFrameObserver {
    unsigned bclk_pad;
    unsigned ws_pad;
    unsigned data_pad;
    Esp32GpioSerialFrameCB callback;
    void *opaque;
} Esp32SerialFrameObserver;

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
    bool status_changed = false;

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
            if (pad >= ESP32_GPIO_PADS) {
                continue;
            }
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
                } else if (s->board_pull[pad]) {
                    resolved = ESP32_PAD_HIGH;
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
                    qemu_irq sink = g_ptr_array_index(listeners, i);
                    bool analytic = false;
                    if (s->materializing_analytic) {
                        GPtrArray *observers = s->analytic_pad_listeners[pad];
                        for (unsigned j = 0; observers &&
                             j < observers->len; j++) {
                            if (g_ptr_array_index(observers, j) == sink) {
                                analytic = true;
                                break;
                            }
                        }
                    }
                    if (!analytic) {
                        qemu_set_irq(sink, resolved);
                    }
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
                    uint32_t bit = BIT(pad % 32);
                    uint32_t *status = &R(s, pad < 32 ? GPIO_STATUS :
                                             GPIO_STATUS1);
                    status_changed |= !(*status & bit);
                    *status |= bit;
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
    /*
     * Peripheral clocks can resolve a pad millions of times per second.
     * Recompute GPIO interrupt outputs only when this resolution newly sets
     * a latched status bit; MMIO writes refresh them through gpio_resolve().
     */
    if (status_changed) {
        update_irq(s);
    }
    trace_flush(s);
}

/* Register changes can alter any route, pull, input enable or interrupt.
 * A full refresh also publishes constants/inverted inputs without pad edges. */
static void gpio_resolve(Esp32GpioState *s)
{
    s->routes_valid = false;
    gpio_resolve_pads(s, (UINT64_C(1) << ESP32_GPIO_PADS) - 1, true);
    update_irq(s);
}

void esp32_gpio_analytic_clock_state(Esp32GpioState *s, unsigned signal,
                                     int64_t now_ns, bool *level,
                                     int64_t *next_edge_ns,
                                     uint64_t *remainder_before_next)
{
    __uint128_t edge_count = 0;
    __uint128_t next_phase;
    uint64_t numerator, denominator, remainder;
    int64_t origin;

    assert(signal < ESP32_GPIO_OUTPUTS && s->analytic_clock[signal]);
    numerator = s->analytic_clock_num[signal];
    denominator = s->analytic_clock_den[signal];
    remainder = s->analytic_clock_remainder[signal];
    origin = s->analytic_clock_origin[signal];
    *level = (s->analytic_clock_pattern[signal] & 1) != 0;

    /*
     * The rational scheduler places edge k at
     * origin + floor(((k + 1) * numerator + remainder) / denominator).
     */
    if (now_ns >= origin && numerator && denominator) {
        __uint128_t elapsed = now_ns - origin;
        __uint128_t limit = (elapsed + 1) * denominator - 1;
        if (limit >= remainder) {
            edge_count = (limit - remainder) / numerator;
        }
        unsigned index = edge_count % s->analytic_clock_length[signal];
        *level = (s->analytic_clock_pattern[signal] >> index) & 1;
    }
    next_phase = (edge_count + 1) * numerator + remainder;
    __uint128_t offset = next_phase / denominator;
    *next_edge_ns = offset > INT64_MAX - origin ? INT64_MAX :
                    origin + (int64_t)offset;
    *remainder_before_next = (edge_count * numerator + remainder) % denominator;
}

bool esp32_gpio_analytic_clock_active(Esp32GpioState *s, unsigned signal)
{
    return s && signal < ESP32_GPIO_OUTPUTS && s->analytic_clock[signal];
}

void esp32_gpio_set_analytic_clock(Esp32GpioState *s, unsigned signal,
                                   bool active, int64_t origin_ns,
                                   bool initial_level, uint64_t period_num,
                                   uint64_t period_den,
                                   uint64_t remainder_before_first)
{
    uint64_t pattern = (initial_level ? 1 : 0) |
                       (initial_level ? 0 : 2);
    esp32_gpio_set_analytic_pattern(s, signal, active, origin_ns, pattern, 2,
                                    period_num, period_den,
                                    remainder_before_first);
}

void esp32_gpio_set_analytic_pattern(Esp32GpioState *s, unsigned signal,
                                     bool active, int64_t origin_ns,
                                     __uint128_t pattern, unsigned length,
                                     uint64_t step_num, uint64_t step_den,
                                     uint64_t remainder_before_first)
{
    if (!s || signal >= ESP32_GPIO_OUTPUTS) {
        return;
    }
    s->analytic_clock[signal] = active;
    if (!active) {
        s->analytic_signals[signal / 64] &=
            ~(UINT64_C(1) << (signal % 64));
        return;
    }
    assert(length && length <= 128 && step_num && step_den &&
           remainder_before_first < step_den);
    s->analytic_signals[signal / 64] |= UINT64_C(1) << (signal % 64);
    s->analytic_clock_origin[signal] = origin_ns;
    s->analytic_clock_initial_level[signal] = pattern & 1;
    s->analytic_clock_num[signal] = step_num;
    s->analytic_clock_den[signal] = step_den;
    s->analytic_clock_remainder[signal] = remainder_before_first;
    s->analytic_clock_pattern[signal] = pattern;
    s->analytic_clock_length[signal] = length;
    bool was_materializing = s->materializing_analytic;
    s->materializing_analytic = true;
    if (signal >= ESP32_GPIO_CLKOUT1) {
        s->clkout_level[signal - ESP32_GPIO_CLKOUT1] = pattern & 1;
    } else if (signal < ESP32_GPIO_OUTPUTS_V1) {
        s->peripheral_known[signal] = 1;
        s->peripheral_value[signal] = pattern & 1;
        s->peripheral_enable[signal] = 1;
        s->peripheral_open_drain[signal] = 0;
    }
    cache_routes(s);
    gpio_resolve_pads(s, s->output_pads[signal], false);
    s->materializing_analytic = was_materializing;
}

static void gpio_materialize_analytic_clock_pads(Esp32GpioState *s,
                                                 uint64_t pads)
{
    cache_routes(s);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    for (unsigned word = 0; word < ARRAY_SIZE(s->analytic_signals); word++) {
        uint64_t signals = s->analytic_signals[word];
        while (signals) {
            unsigned signal = word * 64 + ctz64(signals);
            signals &= signals - 1;
            bool level;
            int64_t next_edge;
            uint64_t remainder;

            if (!s->analytic_clock[signal] ||
                !(s->output_pads[signal] & pads)) {
                continue;
            }
            esp32_gpio_analytic_clock_state(s, signal, now, &level,
                                             &next_edge, &remainder);
            if (s->peripheral_value[signal] != level) {
                s->peripheral_value[signal] = level;
                bool was_materializing = s->materializing_analytic;
                s->materializing_analytic = true;
                gpio_resolve_pads(s, s->output_pads[signal], false);
                s->materializing_analytic = was_materializing;
            }
            (void)next_edge;
            (void)remainder;
        }
    }
}

static void gpio_materialize_analytic_clocks(Esp32GpioState *s)
{
    gpio_materialize_analytic_clock_pads(s,
                                         (UINT64_C(1) << ESP32_GPIO_PADS) - 1);
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
    gpio_materialize_analytic_clock_pads(s, UINT64_C(1) << pad);
    unsigned index = driver * ESP32_GPIO_PADS + pad;
    if (s->external[index] != level) {
        s->external[index] = level;
        gpio_resolve_pads(s, UINT64_C(1) << pad, false);
        for (unsigned signal = 0; signal < ESP32_GPIO_OUTPUTS; signal++) {
            if (s->analytic_clock[signal] &&
                (s->output_pads[signal] & (UINT64_C(1) << pad))) {
                routing_changed(s);
                break;
            }
        }
    }
}

Esp32PadLevel esp32_gpio_get_pad(Esp32GpioState *s, unsigned pad)
{
    assert(pad < ESP32_GPIO_PADS);
    gpio_materialize_analytic_clock_pads(s, UINT64_C(1) << pad);
    return s->resolved[pad];
}

void esp32_gpio_set_board_pull(Esp32GpioState *s, unsigned pad, bool pull)
{
    assert(pad < ESP32_GPIO_PADS);
    gpio_materialize_analytic_clock_pads(s, UINT64_C(1) << pad);
    if (!!s->board_pull[pad] != pull) {
        s->board_pull[pad] = pull;
        gpio_resolve_pads(s, UINT64_C(1) << pad, false);
    }
}

static void gpio_add_pad_listener(Esp32GpioState *s, unsigned pad,
                                  qemu_irq sink, bool analytic)
{
    assert(valid_pad(pad));
    gpio_materialize_analytic_clock_pads(s, UINT64_C(1) << pad);
    if (!s->pad_listeners[pad]) {
        s->pad_listeners[pad] = g_ptr_array_new();
    }
    if (analytic && !s->analytic_pad_listeners[pad]) {
        s->analytic_pad_listeners[pad] = g_ptr_array_new();
    }
    g_ptr_array_add(s->pad_listeners[pad], sink);
    if (analytic) {
        g_ptr_array_add(s->analytic_pad_listeners[pad], sink);
    }
    qemu_set_irq(sink, s->resolved[pad]);
    routing_changed(s);
}

void esp32_gpio_add_pad_listener(Esp32GpioState *s, unsigned pad, qemu_irq sink)
{
    gpio_add_pad_listener(s, pad, sink, false);
}

void esp32_gpio_add_analytic_pad_listener(Esp32GpioState *s, unsigned pad,
                                          qemu_irq sink)
{
    gpio_add_pad_listener(s, pad, sink, true);
}

void esp32_gpio_remove_pad_listener(Esp32GpioState *s, unsigned pad, qemu_irq sink)
{
    assert(valid_pad(pad));
    if (s->pad_listeners[pad]) {
        g_ptr_array_remove(s->pad_listeners[pad], sink);
        if (s->analytic_pad_listeners[pad]) {
            g_ptr_array_remove(s->analytic_pad_listeners[pad], sink);
        }
        routing_changed(s);
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

bool esp32_gpio_output_needs_edges_except_input(Esp32GpioState *s,
                                                unsigned signal,
                                                unsigned input_signal)
{
    uint64_t pads;

    if (!s || signal >= ESP32_GPIO_OUTPUTS) {
        return false;
    }
    if (s->trace) {
        return true;
    }
    cache_routes(s);
    pads = s->output_pads[signal];
    while (pads) {
        unsigned pad = ctz64(pads);
        pads &= pads - 1;
        GPtrArray *listeners = s->pad_listeners[pad];
        for (unsigned i = 0; listeners && i < listeners->len; i++) {
            bool analytic = false;
            GPtrArray *analytic_listeners = s->analytic_pad_listeners[pad];
            for (unsigned j = 0; analytic_listeners &&
                 j < analytic_listeners->len; j++) {
                if (g_ptr_array_index(analytic_listeners, j) ==
                    g_ptr_array_index(listeners, i)) {
                    analytic = true;
                    break;
                }
            }
            if (!analytic) {
                return true;
            }
        }
        for (unsigned driver = 0; driver < ESP32_GPIO_EXT_DRIVERS; driver++) {
            if (s->external[driver * ESP32_GPIO_PADS + pad] != ESP32_PAD_Z) {
                return true;
            }
        }
        if (R(s, GPIO_PIN + pad * 4) & (7 << 7)) {
            return true;
        }
        for (unsigned word = 0; word < ARRAY_SIZE(s->input_signals[pad]);
             word++) {
            uint64_t inputs = s->input_signals[pad][word];
            if (input_signal / 64 == word) {
                inputs &= ~(UINT64_C(1) << (input_signal % 64));
            }
            if (inputs) {
                return true;
            }
        }
    }
    return false;
}

bool esp32_gpio_output_needs_edges(Esp32GpioState *s, unsigned signal)
{
    return esp32_gpio_output_needs_edges_except_input(s, signal,
                                                       ESP32_GPIO_SIGNALS);
}

bool esp32_gpio_output_feeds_input(Esp32GpioState *s, unsigned output_signal,
                                   unsigned input_signal)
{
    uint64_t pads;
    bool found = false;

    if (!s || output_signal >= ESP32_GPIO_OUTPUTS ||
        input_signal >= ESP32_GPIO_SIGNALS ||
        esp32_gpio_output_needs_edges_except_input(s, output_signal,
                                                   input_signal)) {
        return false;
    }
    cache_routes(s);
    pads = s->output_pads[output_signal];
    while (pads) {
        unsigned pad = ctz64(pads);
        uint32_t input = R(s, GPIO_INPUT_SEL + input_signal * 4);
        uint32_t output = R(s, GPIO_OUTPUT_SEL + pad * 4);
        uint32_t pin = R(s, GPIO_PIN + pad * 4);
        pads &= pads - 1;
        /* The time-indexed input sampler models a direct push-pull
         * peripheral output.  Register OE overrides and open-drain pads
         * instead resolve through pad drivers/pulls, so require the normal
         * peripheral OE path here. */
        if (output & (BIT(10) | BIT(11)) || pin & BIT(2)) {
            continue;
        }
        if ((input & BIT(7)) && !(input & BIT(6)) && (input & 63) == pad &&
            (mux_value(s, pad) & IOMUX_IE) && selected_output(s, pad) ==
                output_signal) {
            found = true;
        }
    }
    return found;
}

bool esp32_gpio_input_needs_edges(Esp32GpioState *s, unsigned signal)
{
    if (!s || signal >= ESP32_GPIO_SIGNALS) {
        return true;
    }
    if (s->trace) {
        return true;
    }
    gpio_materialize_analytic_clocks(s);
    cache_routes(s);
    for (unsigned pad = 0; pad < ESP32_GPIO_PADS; pad++) {
        if (!(s->input_signals[pad][signal / 64] &
              (UINT64_C(1) << (signal % 64)))) {
            continue;
        }
        int output = selected_output(s, pad);
        if (output >= 0 && output < ESP32_GPIO_OUTPUTS &&
            s->analytic_clock[output] &&
            ((R(s, GPIO_OUTPUT_SEL + pad * 4) & (BIT(10) | BIT(11))) ||
             (R(s, GPIO_PIN + pad * 4) & BIT(2)))) {
            return true;
        }
        if (s->drive[pad] != ESP32_PAD_Z ||
            s->external_resolved[pad] != ESP32_PAD_Z) {
            return true;
        }
        if (R(s, GPIO_PIN + pad * 4) & (7 << 7)) {
            return true;
        }
    }
    return false;
}

unsigned esp32_gpio_get_input_level(Esp32GpioState *s, unsigned signal)
{
    if (!s || signal >= ESP32_GPIO_SIGNALS) {
        return 0;
    }
    gpio_materialize_analytic_clocks(s);
    return matrix_input(s, signal);
}

unsigned esp32_gpio_get_input_level_at(Esp32GpioState *s, unsigned signal,
                                       int64_t time_ns)
{
    uint32_t cfg;
    unsigned pad;
    int output;
    bool level;
    int64_t next_edge;
    uint64_t remainder;

    if (!s || signal >= ESP32_GPIO_SIGNALS) {
        return 0;
    }
    cfg = R(s, GPIO_INPUT_SEL + signal * 4);
    if (!(cfg & BIT(7))) {
        return esp32_gpio_get_input_level(s, signal);
    }
    pad = cfg & 63;
    if (pad == 0x38) {
        return 1 ^ !!(cfg & BIT(6));
    }
    if (!valid_pad(pad) || !(mux_value(s, pad) & IOMUX_IE)) {
        return 0 ^ !!(cfg & BIT(6));
    }
    output = selected_output(s, pad);
    if (output < 0 || output >= ESP32_GPIO_OUTPUTS ||
        !s->analytic_clock[output]) {
        return esp32_gpio_get_input_level(s, signal);
    }
    /* This API can only predict a future level when the peripheral clock is
     * the pad's physical, push-pull source.  For external drivers, contention,
     * OE overrides, open-drain pads, or other unsupported routing, use the
     * ordinary pad resolver rather than returning a logical clock value as
     * though it were the physical level.  Analytic I2S eligibility uses the
     * same predicate and therefore never relies on this fallback for timing. */
    if (!esp32_gpio_output_feeds_input(s, output, signal)) {
        return esp32_gpio_get_input_level(s, signal);
    }
    esp32_gpio_analytic_clock_state(s, output, time_ns, &level, &next_edge,
                                     &remainder);
    uint32_t out_cfg = R(s, GPIO_OUTPUT_SEL + pad * 4);
    bool oe = s->peripheral_enable[output];
    if (mux_function(s, pad) == 2 && (out_cfg & BIT(11))) {
        oe = !oe;
    }
    if (!oe) {
        level = !!(mux_value(s, pad) & IOMUX_PU) || s->board_pull[pad];
    } else {
        if (mux_function(s, pad) == 2) {
            level ^= !!(out_cfg & BIT(9));
        }
    }
    return level ^ !!(cfg & BIT(6));
}

void esp32_gpio_add_serial_frame_observer(Esp32GpioState *s,
                                          unsigned bclk_pad,
                                          unsigned ws_pad,
                                          unsigned data_pad,
                                          Esp32GpioSerialFrameCB callback,
                                          void *opaque)
{
    Esp32SerialFrameObserver *observer;

    assert(valid_pad(bclk_pad) && valid_pad(ws_pad) && valid_pad(data_pad));
    if (!s->serial_frame_observers) {
        s->serial_frame_observers = g_ptr_array_new_with_free_func(g_free);
    }
    observer = g_new0(Esp32SerialFrameObserver, 1);
    observer->bclk_pad = bclk_pad;
    observer->ws_pad = ws_pad;
    observer->data_pad = data_pad;
    observer->callback = callback;
    observer->opaque = opaque;
    g_ptr_array_add(s->serial_frame_observers, observer);
    routing_changed(s);
}

void esp32_gpio_remove_serial_frame_observer(Esp32GpioState *s,
                                             Esp32GpioSerialFrameCB callback,
                                             void *opaque)
{
    for (unsigned i = 0; s && s->serial_frame_observers &&
                         i < s->serial_frame_observers->len; i++) {
        Esp32SerialFrameObserver *observer =
            g_ptr_array_index(s->serial_frame_observers, i);
        if (observer->callback == callback && observer->opaque == opaque) {
            g_ptr_array_remove_index(s->serial_frame_observers, i);
            return;
        }
    }
}

static bool gpio_output_inverted(Esp32GpioState *s, unsigned pad)
{
    return mux_function(s, pad) == 2 &&
           (R(s, GPIO_OUTPUT_SEL + pad * 4) & BIT(9));
}

void esp32_gpio_publish_serial_frame(Esp32GpioState *s,
                                     unsigned bclk_signal,
                                     unsigned ws_signal,
                                     unsigned data_signal,
                                     uint64_t data_bits,
                                     uint64_t ws_bits,
                                     unsigned bit_count,
                                     int64_t origin_ns,
                                     uint64_t half_period_num,
                                     uint64_t half_period_den,
                                     uint64_t remainder_before_first_rise)
{
    uint64_t mask;

    if (!s || !s->serial_frame_observers || !bit_count || bit_count > 64 ||
        !half_period_num || !half_period_den) {
        return;
    }
    cache_routes(s);
    mask = bit_count == 64 ? UINT64_MAX : (UINT64_C(1) << bit_count) - 1;
    for (unsigned i = 0; i < s->serial_frame_observers->len; i++) {
        Esp32SerialFrameObserver *observer =
            g_ptr_array_index(s->serial_frame_observers, i);
        unsigned bp = observer->bclk_pad, wp = observer->ws_pad;
        unsigned dp = observer->data_pad;
        uint64_t frame_data = data_bits;
        uint64_t frame_ws = ws_bits;
        if (!(s->output_pads[bclk_signal] & (UINT64_C(1) << bp)) ||
            !(s->output_pads[ws_signal] & (UINT64_C(1) << wp)) ||
            !(s->output_pads[data_signal] & (UINT64_C(1) << dp)) ||
            s->drive[bp] >= ESP32_PAD_Z || s->drive[wp] >= ESP32_PAD_Z ||
            s->drive[dp] >= ESP32_PAD_Z ||
            s->external_resolved[bp] != ESP32_PAD_Z ||
            s->external_resolved[wp] != ESP32_PAD_Z ||
            s->external_resolved[dp] != ESP32_PAD_Z) {
            continue;
        }
        bool bclk_inverted = gpio_output_inverted(s, bp);
        if (gpio_output_inverted(s, wp)) {
            frame_ws ^= mask;
        }
        if (gpio_output_inverted(s, dp)) {
            frame_data ^= mask;
        }
        observer->callback(observer->opaque, frame_data, frame_ws, bit_count,
                           origin_ns, half_period_num, half_period_den,
                           remainder_before_first_rise, bclk_inverted);
    }
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
        gpio_materialize_analytic_clocks(s);
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

    gpio_materialize_analytic_clocks(s);
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

    gpio_materialize_analytic_clocks(s);
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
    memset(s->analytic_clock, 0, sizeof(s->analytic_clock));
    memset(s->analytic_signals, 0, sizeof(s->analytic_signals));
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
        if (s->analytic_pad_listeners[pad]) {
            g_ptr_array_unref(s->analytic_pad_listeners[pad]);
        }
    }
    if (s->routing_listeners) {
        g_ptr_array_unref(s->routing_listeners);
    }
    if (s->serial_frame_observers) {
        g_ptr_array_unref(s->serial_frame_observers);
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
