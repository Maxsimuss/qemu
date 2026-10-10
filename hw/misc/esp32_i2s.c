/*
 * ESP32 I2S register, FIFO, DMA and digital interface model.
 * References: ESP32 TRM v5.8 chapters 2, 6 and 22; Espressif ESP32 register
 * definitions. Interfaces connect through resolved physical pads, not SDK APIs.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "exec/address-spaces.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/misc/esp32_i2s.h"

#define REG(s, a) ((s)->regs[(a) / 4])
#define CONF 0x08
#define INT_RAW 0x0c
#define INT_ENA 0x14
#define FIFO_CONF 0x20
#define RXEOF_NUM 0x24
#define SINGLE_DATA 0x28
#define CHAN_CONF 0x2c
#define OUT_LINK 0x30
#define IN_LINK 0x34
#define LC_CONF 0x60
#define CONF1 0xa0
#define CONF2 0xa8
#define CLKM_CONF 0xac
#define SAMPLE_CONF 0xb0
#define PDM_CONF 0xb4
#define INT_TX_EMPTY BIT(5)
#define INT_TX_FULL BIT(4)
#define INT_RX_EMPTY BIT(3)
#define INT_RX_FULL BIT(2)
#define INT_TX_PUT BIT(1)
#define INT_RX_TAKE BIT(0)
#define INT_TX_HUNG BIT(7)
#define INT_RX_HUNG BIT(6)
#define INT_IN_DONE BIT(8)
#define INT_IN_EOF BIT(9)
#define INT_OUT_DONE BIT(11)
#define INT_OUT_EOF BIT(12)
#define INT_IN_ERROR BIT(13)
#define INT_OUT_ERROR BIT(14)
#define INT_IN_EMPTY BIT(15)
#define INT_OUT_TOTAL BIT(16)
#define FIFO_LAST BIT(0)
#define FIFO_EOF BIT(1)
#define FIFO_TOTAL BIT(2)
#define DESC_OWNER BIT(31)
#define DESC_EOF BIT(30)
#define LINK_ADDRESS 0xfffff
#define DMA_RAM_BASE 0x3ffae000
#define DMA_RAM_END 0x40000000

/* SDK headers document a few fields omitted by the TRM (CLK_EN, MEM_TRANS_EN,
 * CVSD). The generated masks retain these; unimplemented operating modes emit
 * LOG_UNIMP and do not produce a synthetic successful transfer. */
typedef struct I2SRegisterInfo {
    uint32_t reset;
    uint32_t read_mask;
    uint32_t write_mask;
} I2SRegisterInfo;
#include "esp32_i2s_regs.inc"

static void i2s_update(Esp32I2SState *s);
static void dma_pump(Esp32I2SState *s);
static unsigned fifo_mode(Esp32I2SState *s, bool tx);
static bool tx_analytic_eligible(Esp32I2SState *s);
static bool rx_analytic_paired(Esp32I2SState *s);
static void tx_analytic_to_edges(Esp32I2SState *s, int64_t now);
static bool rx_analytic_eligible(Esp32I2SState *s);
static void rx_analytic_to_edges(Esp32I2SState *s, int64_t now);

static void capability_gap(Esp32I2SState *s, unsigned flag, const char *detail)
{
    if (!(s->warned_capabilities & BIT(flag))) {
        warn_report("ESP32 I2S%u capability gap: %s", s->controller, detail);
        s->warned_capabilities |= BIT(flag);
    }
}

static void mode_diagnostics(Esp32I2SState *s)
{
    if (REG(s, PDM_CONF) & 3) {
        capability_gap(s, 0, "PDM bitstream/conversion is not implemented");
    }
    if ((REG(s, CLKM_CONF) & BIT(21)) && !s->apll_hz) {
        capability_gap(s, 1, "APLL programming/calibration clock source is absent");
    }
    if (REG(s, 0x1c) & 0xffffff) {
        capability_gap(s, 2, "nonzero TIMING delays/double-sync are not modeled");
    }
    if (REG(s, 0xa4) & BIT(0)) {
        capability_gap(s, 3, "FIFO power-down retention is not modeled");
    }
    if ((REG(s, 0x98) & (BIT(0) | BIT(9))) || (REG(s, 0x9c) & BIT(2))) {
        capability_gap(s, 4, "CVSD/PLC conversion is not implemented");
    }
    if (REG(s, LC_CONF) & (BIT(4) | BIT(5) | BIT(13))) {
        capability_gap(s, 5, "DMA loop-test/memory-copy mode is not implemented");
    }
    if ((!(REG(s, CONF1) & BIT(3)) && (REG(s, CONF1) & 7) > 1) ||
        (!(REG(s, CONF1) & BIT(7)) && ((REG(s, CONF1) >> 4) & 7) > 1)) {
        capability_gap(s, 6, "selected PCM processing configuration is unsupported");
    }
}

static void interrupt(Esp32I2SState *s, uint32_t bits)
{
    REG(s, INT_RAW) |= bits;
    qemu_set_irq(s->irq, !!(REG(s, INT_RAW) & REG(s, INT_ENA)));
}

static unsigned signal_bck(Esp32I2SState *s, bool tx)
{
    return tx ? (s->controller ? 24 : 23) : (s->controller ? 164 : 27);
}

static unsigned signal_ws(Esp32I2SState *s, bool tx)
{
    return tx ? (s->controller ? 26 : 25) : (s->controller ? 165 : 28);
}

static unsigned signal_data(Esp32I2SState *s, bool tx)
{
    return (s->controller ? 166 : 140) + (tx ? 23 : 15);
}

static void drive(Esp32I2SState *s, unsigned signal, bool level, bool enable)
{
    if (s->tx.analytic_clock &&
        (signal == signal_bck(s, true) || signal == signal_ws(s, true) ||
         signal == signal_data(s, true))) {
        return;
    }
    if (s->rx.analytic_clock &&
        (signal == signal_bck(s, false) || signal == signal_ws(s, false))) {
        return;
    }
    esp32_gpio_set_peripheral_output(s->gpio, signal, level, enable, false);
}

static void fifo_interrupts(Esp32I2SState *s)
{
    uint32_t bits = 0;
    if (s->tx.fifo_count == 0) {
        bits |= INT_TX_EMPTY;
    }
    if (s->tx.fifo_count == ESP32_I2S_FIFO_WORDS) {
        bits |= INT_TX_FULL;
    }
    if (s->rx.fifo_count == 0) {
        bits |= INT_RX_EMPTY;
    }
    if (s->rx.fifo_count == ESP32_I2S_FIFO_WORDS) {
        bits |= INT_RX_FULL;
    }
    if (s->tx.fifo_count < ((REG(s, FIFO_CONF) >> 6) & 63)) {
        bits |= INT_TX_PUT;
    }
    if (s->rx.fifo_count > (REG(s, FIFO_CONF) & 63)) {
        bits |= INT_RX_TAKE;
    }
    interrupt(s, bits);
}

static bool dma_address(uint32_t address, uint32_t size)
{
    return !(address & 3) && address >= DMA_RAM_BASE &&
           size <= DMA_RAM_END - address && address < DMA_RAM_END;
}

static bool dma_read(uint32_t address, void *data, unsigned size)
{
    return dma_address(address, size) &&
           address_space_read(&address_space_memory, address,
                              MEMTXATTRS_UNSPECIFIED, data, size) == MEMTX_OK;
}

static bool dma_write(uint32_t address, const void *data, unsigned size)
{
    return dma_address(address, size) &&
           address_space_write(&address_space_memory, address,
                               MEMTXATTRS_UNSPECIFIED, data, size) == MEMTX_OK;
}

static void dma_error(Esp32I2SState *s, bool tx)
{
    Esp32I2SChannel *c = tx ? &s->tx : &s->rx;
    c->link_active = false;
    c->descriptor_loaded = false;
    REG(s, tx ? OUT_LINK : IN_LINK) |= BIT(31);
    interrupt(s, tx ? INT_OUT_ERROR : INT_IN_ERROR);
}

static bool load_descriptor(Esp32I2SState *s, bool tx)
{
    Esp32I2SChannel *c = tx ? &s->tx : &s->rx;
    uint32_t raw[3];
    unsigned size, length;

    if (!c->descriptor || !dma_read(c->descriptor, raw, sizeof(raw))) {
        dma_error(s, tx);
        return false;
    }
    for (unsigned i = 0; i < 3; i++) {
        c->descriptor_words[i] = le32_to_cpu(raw[i]);
    }
    size = c->descriptor_words[0] & 4095;
    length = (c->descriptor_words[0] >> 12) & 4095;
    if (size < 4 || (size & 3) || (tx && (!length || (length & 3) || length > size)) ||
        !dma_address(c->descriptor_words[1], size) ||
        ((REG(s, LC_CONF) & BIT(12)) && !(c->descriptor_words[0] & DESC_OWNER))) {
        dma_error(s, tx);
        return false;
    }
    c->descriptor_offset = 0;
    c->descriptor_limit = tx ? length : size;
    c->descriptor_loaded = true;
    REG(s, tx ? 0x54 : 0x48) = c->descriptor;
    REG(s, tx ? 0x58 : 0x4c) = c->descriptor_words[2];
    uint32_t next_buffer = 0;
    if (c->descriptor_words[2] &&
        dma_read(c->descriptor_words[2] + 4, &next_buffer, 4)) {
        next_buffer = le32_to_cpu(next_buffer);
    }
    REG(s, tx ? 0x5c : 0x50) = next_buffer;
    return true;
}

static void fifo_push(Esp32I2SChannel *c, uint32_t word,
                      uint32_t descriptor, uint8_t flags)
{
    unsigned tail = (c->fifo_head + c->fifo_count) % ESP32_I2S_FIFO_WORDS;
    assert(c->fifo_count < ESP32_I2S_FIFO_WORDS);
    c->fifo[tail] = word;
    c->fifo_descriptor[tail] = descriptor;
    c->fifo_flags[tail] = flags;
    c->fifo_count++;
}

static bool fifo_pop(Esp32I2SState *s, bool tx, uint32_t *value)
{
    Esp32I2SChannel *c = tx ? &s->tx : &s->rx;
    unsigned head = c->fifo_head;
    uint8_t flags;

    if (!c->fifo_count) {
        return false;
    }
    *value = c->fifo[head];
    flags = c->fifo_flags[head];
    if (tx && (flags & FIFO_LAST)) {
        if (REG(s, LC_CONF) & BIT(6)) {
            uint32_t raw;
            if (dma_read(c->fifo_descriptor[head], &raw, 4)) {
                raw = cpu_to_le32(le32_to_cpu(raw) & ~DESC_OWNER);
                if (!dma_write(c->fifo_descriptor[head], &raw, 4)) {
                    dma_error(s, true);
                }
            } else {
                dma_error(s, true);
            }
        }
        interrupt(s, INT_OUT_DONE);
    }
    if (tx && (flags & FIFO_EOF) && (REG(s, LC_CONF) & BIT(8))) {
        REG(s, 0x38) = c->fifo_descriptor[head];
        interrupt(s, INT_OUT_EOF);
    }
    if (tx && (flags & FIFO_TOTAL)) {
        interrupt(s, INT_OUT_TOTAL);
    }
    c->fifo_head = (head + 1) % ESP32_I2S_FIFO_WORDS;
    c->fifo_count--;
    fifo_interrupts(s);
    if (!s->pumping_dma) {
        dma_pump(s);
    }
    return true;
}

static void tx_dma(Esp32I2SState *s)
{
    Esp32I2SChannel *c = &s->tx;
    if ((REG(s, CONF) & BIT(2)) || (REG(s, LC_CONF) & (BIT(1) | BIT(3)))) {
        return;
    }
    unsigned entries = fifo_mode(s, true) == 1 ? 2 : 1;
    while (c->link_active && c->fifo_count + entries <= ESP32_I2S_FIFO_WORDS) {
        uint32_t raw, word;
        uint8_t flags = 0;
        if (!c->descriptor_loaded && !load_descriptor(s, true)) {
            break;
        }
        if (!dma_read(c->descriptor_words[1] + c->descriptor_offset, &raw, 4)) {
            dma_error(s, true);
            break;
        }
        word = le32_to_cpu(raw);
        c->descriptor_offset += 4;
        if (c->descriptor_offset == c->descriptor_limit) {
            flags = FIFO_LAST;
            if (c->descriptor_words[0] & DESC_EOF) {
                flags |= FIFO_EOF;
                REG(s, 0x40) = c->descriptor;
                if (!(REG(s, LC_CONF) & BIT(8))) {
                    REG(s, 0x38) = c->descriptor;
                    interrupt(s, INT_OUT_EOF);
                }
            }
            if (!c->descriptor_words[2]) {
                flags |= FIFO_TOTAL;
                c->link_active = false;
                REG(s, OUT_LINK) |= BIT(31);
            }
        }
        if (entries == 2) {
            fifo_push(c, word & 0xffff0000, c->descriptor, 0);
            fifo_push(c, word << 16, c->descriptor, flags);
        } else {
            fifo_push(c, word, c->descriptor, flags);
        }
        if (flags & FIFO_LAST) {
            c->descriptor = c->descriptor_words[2];
            c->descriptor_loaded = false;
        }
    }
}

static void rx_dma(Esp32I2SState *s)
{
    Esp32I2SChannel *c = &s->rx;
    if ((REG(s, CONF) & BIT(3)) || (REG(s, LC_CONF) & (BIT(0) | BIT(3)))) {
        return;
    }
    while (c->link_active && c->fifo_count) {
        uint32_t raw, word;
        bool eof;
        if (!c->descriptor_loaded && !load_descriptor(s, false)) {
            break;
        }
        fifo_pop(s, false, &word);
        raw = cpu_to_le32(word);
        if (!dma_write(c->descriptor_words[1] + c->descriptor_offset, &raw, 4)) {
            dma_error(s, false);
            break;
        }
        c->descriptor_offset += 4;
        c->dma_words++;
        eof = REG(s, RXEOF_NUM) && c->dma_words == REG(s, RXEOF_NUM);
        if (eof || c->descriptor_offset == c->descriptor_limit) {
            uint32_t header = c->descriptor_words[0] &
                              ~(DESC_OWNER | DESC_EOF | 0xfff000);
            header |= c->descriptor_offset << 12;
            header |= eof ? DESC_EOF : 0;
            raw = cpu_to_le32(header);
            if (!dma_write(c->descriptor, &raw, 4)) {
                dma_error(s, false);
                break;
            }
            interrupt(s, INT_IN_DONE);
            if (eof) {
                REG(s, 0x3c) = c->descriptor;
                interrupt(s, INT_IN_EOF);
                c->dma_words = 0;
            }
            c->descriptor = c->descriptor_words[2];
            c->descriptor_loaded = false;
            if (!c->descriptor) {
                c->link_active = false;
                REG(s, IN_LINK) |= BIT(31);
                interrupt(s, INT_IN_EMPTY);
            }
        }
    }
}

static void dma_pump(Esp32I2SState *s)
{
    if (s->pumping_dma || !s->enabled || !(REG(s, FIFO_CONF) & BIT(12)) ||
        !esp32_i2s_mode_supported(s)) {
        return;
    }
    s->pumping_dma = true;
    tx_dma(s);
    rx_dma(s);
    s->pumping_dma = false;
    fifo_interrupts(s);
}

static unsigned sample_bits(Esp32I2SState *s, bool tx)
{
    return (REG(s, SAMPLE_CONF) >> (tx ? 12 : 18)) & 63;
}

static unsigned fifo_mode(Esp32I2SState *s, bool tx)
{
    return (REG(s, FIFO_CONF) >> (tx ? 13 : 16)) & 7;
}

/* ITU-T G.711 table 1a: 13 significant uniform PCM bits, represented in
 * the high bits of a signed 16-bit sample, and alternating-bit inversion. */
static uint32_t pcm_convert(Esp32I2SState *s, bool tx, uint32_t sample)
{
    unsigned shift = tx ? 0 : 4;
    unsigned configuration = (REG(s, CONF1) >> shift) & 15;
    if (configuration & 8) {
        return sample;
    }
    if (configuration & 1) {
        int pcm = (int16_t)(sample >> 16);
        unsigned magnitude = pcm < 0 ? -pcm - 1 : pcm;
        unsigned segment = 0;
        while (segment < 7 && magnitude > ((256u << segment) - 1)) {
            segment++;
        }
        unsigned quantization = (magnitude >> (segment < 2 ? 4 : segment + 3)) & 15;
        uint8_t encoded = ((segment << 4) | quantization) ^ (pcm < 0 ? 0x55 : 0xd5);
        return (uint32_t)encoded << 24;
    }
    uint8_t encoded = (sample >> 24) ^ 0x55;
    unsigned segment = (encoded >> 4) & 7;
    int magnitude = ((encoded & 15) << 4) + (segment ? 0x108 : 8);
    if (segment > 1) {
        magnitude <<= segment - 1;
    }
    int16_t pcm = (encoded & 0x80) ? magnitude : -magnitude;
    return (uint32_t)(uint16_t)pcm << 16;
}

static bool tx_load_frame(Esp32I2SState *s)
{
    unsigned mode = fifo_mode(s, true);
    unsigned chan = REG(s, CHAN_CONF) & 7;
    bool msb_right = REG(s, CONF) & BIT(16);
    uint32_t a, b;

    dma_pump(s);
    if (!fifo_pop(s, true, &a)) {
        if (REG(s, CONF1) & BIT(8)) {
            return false;
        }
        /* TRM 22.4.6: replay the last frame on FIFO underrun. */
        return s->tx.frame_valid;
    }
    if (mode == 0) {
        b = a << 16;
        a &= 0xffff0000;
    } else if (mode == 1) {
        /* Figure 22.4-4 expands CPU/DMA writes into padded mono words. */
        b = a;
    } else if (mode == 2) {
        if (!fifo_pop(s, true, &b)) {
            b = 0;
        }
    } else if (mode == 3) {
        b = a;
    } else {
        qemu_log_mask(LOG_UNIMP,
                      "esp32.i2s%u: unsupported TX FIFO mode %u\n",
                      s->controller, mode);
        return false;
    }
    s->tx.frame[msb_right ? 1 : 0] = a;
    s->tx.frame[msb_right ? 0 : 1] = b;
    if (chan == 1 || (REG(s, CONF) & BIT(14))) {
        s->tx.frame[!msb_right] = s->tx.frame[msb_right];
    } else if (chan == 2) {
        s->tx.frame[msb_right] = s->tx.frame[!msb_right];
    } else if (chan == 3) {
        s->tx.frame[msb_right ? 1 : 0] = REG(s, SINGLE_DATA);
    } else if (chan == 4) {
        s->tx.frame[msb_right ? 0 : 1] = REG(s, SINGLE_DATA);
    } else if (chan > 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32.i2s%u: reserved TX channel mode %u\n",
                      s->controller, chan);
        return false;
    }
    s->tx.frame[0] = pcm_convert(s, true, s->tx.frame[0]);
    s->tx.frame[1] = pcm_convert(s, true, s->tx.frame[1]);
    return true;
}

static void rx_store_frame(Esp32I2SState *s)
{
    if (REG(s, CONF) & BIT(3)) {
        return;
    }
    unsigned mode = fifo_mode(s, false);
    unsigned chan = (REG(s, CHAN_CONF) >> 3) & 3;
    bool msb_right = REG(s, CONF) & BIT(17);
    uint32_t a = pcm_convert(s, false, s->rx.frame[msb_right ? 1 : 0]);
    uint32_t b = pcm_convert(s, false, s->rx.frame[msb_right ? 0 : 1]);

    if (chan == 1 || (REG(s, CONF) & BIT(15))) {
        b = a;
    } else if (chan == 2) {
        a = b;
    }
    unsigned words = mode >= 2 && mode != 3 ? 2 : 1;
    if (s->rx.fifo_count + words > ESP32_I2S_FIFO_WORDS) {
        interrupt(s, INT_RX_FULL);
        return;
    }
    if (mode == 0) {
        fifo_push(&s->rx, (a & 0xffff0000) | (b >> 16), 0, 0);
    } else if (mode == 1) {
        if (s->rx.mono_pending) {
            fifo_push(&s->rx, s->rx.shift | (a >> 16), 0, 0);
        } else {
            s->rx.shift = a & 0xffff0000;
        }
        s->rx.mono_pending = !s->rx.mono_pending;
    } else if (mode == 2) {
        fifo_push(&s->rx, a, 0, 0);
        fifo_push(&s->rx, b, 0, 0);
    } else if (mode == 3) {
        fifo_push(&s->rx, a, 0, 0);
    } else {
        qemu_log_mask(LOG_UNIMP,
                      "esp32.i2s%u: unsupported RX FIFO mode %u\n",
                      s->controller, mode);
        return;
    }
    fifo_interrupts(s);
    dma_pump(s);
}

/* Rational nanosecond intervals retain the fractional source-clock period;
 * rounding each edge independently would accumulate clock drift. */
static bool clock_period(Esp32I2SState *s, unsigned divider,
                         uint64_t *numerator, uint64_t *denominator)
{
    uint32_t cfg = REG(s, CLKM_CONF);
    uint64_t n = cfg & 255;
    uint64_t a = (cfg >> 14) & 63;
    uint64_t b = (cfg >> 8) & 63;
    uint64_t source_num = (cfg & BIT(21)) ? s->apll_numerator : 160000000;
    uint64_t source_den = (cfg & BIT(21)) ? s->apll_denominator : 1;
    uint64_t divisor;
    __uint128_t period;

    if (!s->enabled || !(cfg & BIT(20)) || !source_num || !source_den || n < 2 ||
        (!a && b) || divider == 0) {
        return false;
    }
    if (!a) {
        a = 1;
    }
    divisor = 2 * a * source_num;
    period = (__uint128_t)(n * a + b) * divider *
             UINT64_C(1000000000) * source_den;
    if (period > UINT64_MAX) {
        return false;
    }
    *numerator = period;
    *denominator = divisor;
    return true;
}

static bool clock_step(Esp32I2SState *s, unsigned divider,
                       uint64_t *remainder, int64_t *deadline)
{
    uint64_t numerator, denominator;
    __uint128_t accumulated;

    if (!clock_period(s, divider, &numerator, &denominator)) {
        return false;
    }
    accumulated = (__uint128_t)numerator + *remainder;
    *deadline += accumulated / denominator;
    *remainder = accumulated % denominator;
    return true;
}

static bool channel_running(Esp32I2SState *s, bool tx)
{
    uint32_t conf = REG(s, CONF);
    return s->enabled && (conf & BIT(tx ? 4 : 5)) &&
           !(conf & BIT(tx ? 0 : 1)) &&
           (REG(s, CLKM_CONF) & BIT(20));
}

static bool master(Esp32I2SState *s, bool tx)
{
    return !(REG(s, CONF) & BIT(tx ? 6 : 7));
}

static void tx_falling(Esp32I2SState *s)
{
    Esp32I2SChannel *c = &s->tx;
    unsigned bits = sample_bits(s, true);
    unsigned right_first = !!(REG(s, CONF) & BIT(8));
    unsigned shifted = !!(REG(s, CONF) & BIT(10));
    unsigned slot = c->bit / bits;
    unsigned bit = c->bit % bits;

    if (c->bit == 0) {
        c->frame_valid = tx_load_frame(s);
        if (!c->frame_valid) {
            if (REG(s, CONF1) & BIT(8)) {
                return;
            }
            memset(c->frame, 0, sizeof(c->frame));
        }
    }
    unsigned channel = slot ^ right_first;
    unsigned ws = ((c->bit + shifted) / bits) & 1;
    c->ws_level = (REG(s, CONF) & BIT(12)) ?
                  ((c->bit + shifted) % (2 * bits) == 0) :
                  (ws ^ right_first);
    c->data_level = (c->frame[channel] >> (31 - bit)) & 1;
    drive(s, signal_ws(s, true), c->ws_level, master(s, true));
    drive(s, signal_data(s, true), c->data_level, true);
    c->bit = (c->bit + 1) % (2 * bits);
}

static void rx_rising(Esp32I2SState *s)
{
    Esp32I2SChannel *c = &s->rx;
    unsigned bits = sample_bits(s, false);
    unsigned right_first = !!(REG(s, CONF) & BIT(9));
    unsigned data = s->input_level[4];
    unsigned current_ws = (REG(s, CONF) & BIT(18)) ?
                          (master(s, true) ? s->tx.ws_level : s->input_level[1]) :
                          (master(s, false) ? c->ws_level : s->input_level[3]);
    unsigned ws = (REG(s, CONF) & BIT(11)) ? c->previous_ws : current_ws;
    bool short_sync = REG(s, CONF) & BIT(13);
    bool changed = ws != c->slot;
    c->previous_ws = current_ws;

    if (short_sync) {
        if (ws && !c->slot) {
            c->bit = 0;
            c->synchronized = true;
            memset(c->frame, 0, sizeof(c->frame));
        }
        c->slot = ws;
        if (c->synchronized && c->bit < 2 * bits) {
            unsigned slot = (c->bit / bits) ^ right_first;
            c->frame[slot] |= data << (31 - (c->bit % bits));
            if (++c->bit == 2 * bits) {
                rx_store_frame(s);
            }
        }
        return;
    }
    if (!c->synchronized) {
        if (ws != right_first) {
            return;
        }
        c->synchronized = true;
        c->slot = ws;
        c->bit = 0;
        c->frame[ws] = 0;
    } else if (changed) {
        c->slot = ws;
        c->bit = 0;
        c->frame[ws] = 0;
    }
    if (c->bit < bits) {
        c->frame[ws] |= data << (31 - c->bit);
        if (++c->bit == bits) {
            c->phase |= BIT(ws);
            if (c->phase == 3) {
                rx_store_frame(s);
                c->phase = 0;
            }
        }
    }
}

static void lcd_tx(Esp32I2SState *s)
{
    Esp32I2SChannel *c = &s->tx;
    unsigned conf = REG(s, CONF2);
    unsigned phase = c->phase++ % ((conf & BIT(1)) ? 4 : 2);
    bool duplicate_pair = conf & BIT(2);

    if (phase == 0 && !tx_load_frame(s)) {
        return;
    }
    c->frame_valid = true;
    unsigned slot = duplicate_pair ? phase % 2 : phase / ((conf & BIT(1)) ? 2 : 1);
    uint32_t data = c->frame[slot] >> 8;
    for (unsigned bit = 0; bit < 24; bit++) {
        drive(s, (s->controller ? 166 : 140) + bit, (data >> bit) & 1, true);
    }
}

static void camera_rx(Esp32I2SState *s)
{
    Esp32I2SChannel *c = &s->rx;
    uint32_t sample = 0;
    if (!s->input_level[21] || !s->input_level[22] || !s->input_level[23]) {
        return;
    }
    for (unsigned bit = 0; bit < 16; bit++) {
        sample |= s->input_level[bit == 15 ? 4 : bit + 5] << bit;
    }
    c->frame[0] = c->frame[1] = sample << 16;
    rx_store_frame(s);
}

static unsigned tx_frame_values(Esp32I2SState *s, uint64_t *data,
                                uint64_t *ws, __uint128_t *data_levels,
                                __uint128_t *ws_levels, bool origin_falling)
{
    unsigned bits = sample_bits(s, true);
    unsigned count = 2 * bits;
    unsigned first = !!(REG(s, CONF) & BIT(8));
    unsigned shifted = !!(REG(s, CONF) & BIT(10));

    *data = *ws = 0;
    *data_levels = *ws_levels = 0;
    for (unsigned i = 0; i < count; i++) {
        unsigned slot = i / bits;
        unsigned bit = i % bits;
        unsigned channel = slot ^ first;
        bool d = (s->tx.frame[channel] >> (31 - bit)) & 1;
        bool w = (REG(s, CONF) & BIT(12)) ?
                 ((i + shifted) % (2 * bits) == 0) :
                 ((((i + shifted) / bits) & 1) ^ first);
        *data |= (uint64_t)d << i;
        *ws |= (uint64_t)w << i;
        if (origin_falling) {
            *data_levels |= (__uint128_t)d << (2 * i);
            *data_levels |= (__uint128_t)d << (2 * i + 1);
            *ws_levels |= (__uint128_t)w << (2 * i);
            *ws_levels |= (__uint128_t)w << (2 * i + 1);
        } else {
            *data_levels |= (__uint128_t)d << (2 * i);
            *ws_levels |= (__uint128_t)w << (2 * i);
            bool nd = i + 1 < count ?
                (s->tx.frame[((i + 1) / bits) ^ first] >>
                 (31 - ((i + 1) % bits))) & 1 : d;
            unsigned ni = i + 1;
            bool nw = ni < count ?
                ((REG(s, CONF) & BIT(12)) ?
                 ((ni + shifted) % (2 * bits) == 0) :
                 ((((ni + shifted) / bits) & 1) ^ first)) : w;
            *data_levels |= (__uint128_t)nd << (2 * i + 1);
            *ws_levels |= (__uint128_t)nw << (2 * i + 1);
        }
    }
    return count;
}

static void tx_analytic_patterns(Esp32I2SState *s, int64_t origin,
                                 uint64_t half_num, uint64_t half_den,
                                 uint64_t remainder, unsigned count,
                                 bool origin_falling)
{
    __uint128_t data_levels = 0, ws_levels = 0;
    uint64_t unused_data, unused_ws;
    tx_frame_values(s, &unused_data, &unused_ws, &data_levels, &ws_levels,
                    origin_falling);
    esp32_gpio_set_analytic_pattern(s->gpio, signal_bck(s, true), true,
        origin, origin_falling ? 2 : 1, 2, half_num, half_den, remainder);
    esp32_gpio_set_analytic_pattern(s->gpio, signal_ws(s, true), true,
        origin, ws_levels, count * 2, half_num, half_den, remainder);
    esp32_gpio_set_analytic_pattern(s->gpio, signal_data(s, true), true,
        origin, data_levels, count * 2, half_num, half_den, remainder);
}

static bool tx_analytic_frame_start(Esp32I2SState *s, int64_t now,
                                    unsigned divider)
{
    Esp32I2SChannel *c = &s->tx;
    uint64_t half_num, half_den;
    __uint128_t unused_data_levels, unused_ws_levels;
    unsigned count;

    if (!clock_period(s, divider, &half_num, &half_den)) {
        return false;
    }
    c->analytic_clock = true;
    c->analytic_frame_origin = now;
    c->analytic_origin_falling = false;
    c->analytic_frame_remainder = c->clock_remainder;
    c->analytic_frame_half_num = half_num;
    c->analytic_frame_half_den = half_den;
    count = tx_frame_values(s, &c->analytic_frame_data,
                            &c->analytic_frame_ws, &unused_data_levels,
                            &unused_ws_levels, false);
    c->analytic_frame_count = count;
    c->analytic_frame_pending = true;
    c->clock_level = true;
    tx_analytic_patterns(s, now, half_num, half_den, c->clock_remainder,
                         count, false);
    /*
     * Retire one frame at its final falling edge so FIFO/DMA side effects
     * retain their hardware time instead of happening a frame early.
     */
    for (unsigned i = 0; i < 2 * count - 1; i++) {
        if (!clock_step(s, divider, &c->clock_remainder, &c->deadline)) {
            return false;
        }
    }
    return true;
}

static bool tx_analytic_frame_end(Esp32I2SState *s, int64_t now,
                                  unsigned divider)
{
    Esp32I2SChannel *c = &s->tx;
    unsigned bits = sample_bits(s, true);
    uint64_t half_num, half_den;
    __uint128_t unused_data_levels, unused_ws_levels;

    if (c->analytic_frame_pending) {
        esp32_gpio_publish_serial_frame(s->gpio, signal_bck(s, true),
            signal_ws(s, true), signal_data(s, true), c->analytic_frame_data,
            c->analytic_frame_ws, c->analytic_frame_count,
            c->analytic_frame_origin, c->analytic_frame_half_num,
            c->analytic_frame_half_den, c->analytic_frame_remainder);
    }
    c->analytic_frame_pending = false;
    c->clock_level = false;
    c->bit = 0;
    tx_falling(s);
    if ((REG(s, CONF1) & BIT(8)) && !c->frame_valid && !c->fifo_count) {
        c->analytic_clock = false;
        esp32_gpio_set_analytic_pattern(s->gpio, signal_bck(s, true),
                                         false, 0, 0, 0, 0, 0, 0);
        esp32_gpio_set_analytic_pattern(s->gpio, signal_ws(s, true),
                                         false, 0, 0, 0, 0, 0, 0);
        esp32_gpio_set_analytic_pattern(s->gpio, signal_data(s, true),
                                         false, 0, 0, 0, 0, 0, 0);
        drive(s, signal_bck(s, true), false, true);
        drive(s, signal_ws(s, true), c->ws_level, true);
        drive(s, signal_data(s, true), c->data_level, true);
        timer_del(c->timer);
        return false;
    }
    if (!clock_period(s, divider, &half_num, &half_den)) {
        return false;
    }
    uint64_t next_phase = c->clock_remainder + half_num;
    int64_t first_rising = now + next_phase / half_den;
    c->analytic_frame_origin = first_rising;
    c->analytic_origin_falling = true;
    c->analytic_frame_remainder = next_phase % half_den;
    c->analytic_frame_half_num = half_num;
    c->analytic_frame_half_den = half_den;
    c->analytic_frame_count = tx_frame_values(s, &c->analytic_frame_data,
        &c->analytic_frame_ws, &unused_data_levels, &unused_ws_levels, true);
    c->analytic_frame_pending = true;
    tx_analytic_patterns(s, now, half_num, half_den, c->clock_remainder,
                         c->analytic_frame_count, true);
    for (unsigned i = 0; i < 4 * bits; i++) {
        if (!clock_step(s, divider, &c->clock_remainder, &c->deadline)) {
            return false;
        }
    }
    return true;
}

static void tx_clock(void *opaque)
{
    Esp32I2SState *s = opaque;
    Esp32I2SChannel *c = &s->tx;
    unsigned divider = REG(s, SAMPLE_CONF) & 63;

    if (!channel_running(s, true) || !master(s, true) ||
        !esp32_i2s_mode_supported(s)) {
        return;
    }
    bool frame_analytic = tx_analytic_eligible(s);
    if (frame_analytic) {
        bool ok;
        if (c->analytic_clock && c->analytic_frame_pending) {
            ok = tx_analytic_frame_end(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                                       divider);
        } else {
            ok = tx_analytic_frame_start(s,
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), divider);
        }
        if (ok) {
            timer_mod_ns(c->timer, c->deadline);
        } else {
            timer_del(c->timer);
        }
        return;
    }
    if (c->analytic_clock) {
        tx_analytic_to_edges(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        return;
    }
    c->clock_level = !c->clock_level;
    if (REG(s, CONF2) & BIT(5)) {
        if (!c->clock_level) {
            c->ws_level = !c->ws_level;
            if (!c->ws_level) {
                lcd_tx(s);
            }
            drive(s, signal_ws(s, true), c->ws_level, true);
        }
    } else if (!c->clock_level) {
        tx_falling(s);
    }
    drive(s, signal_bck(s, true), c->clock_level, true);
    if (!c->clock_level && (REG(s, CONF1) & BIT(8)) &&
        !c->frame_valid && !c->fifo_count) {
        return;
    }
    if (c->clock_level && (REG(s, CONF) & BIT(18)) &&
        channel_running(s, false) && !master(s, false)) {
        rx_rising(s);
    }
    if (clock_step(s, divider, &c->clock_remainder, &c->deadline)) {
        timer_mod_ns(c->timer, c->deadline);
    }
}

static void rx_clock(void *opaque)
{
    Esp32I2SState *s = opaque;
    Esp32I2SChannel *c = &s->rx;
    unsigned divider = (REG(s, SAMPLE_CONF) >> 6) & 63;
    unsigned bits = sample_bits(s, false);

    if (!channel_running(s, false) || !master(s, false) ||
        !esp32_i2s_mode_supported(s)) {
        return;
    }
    if (rx_analytic_eligible(s)) {
        unsigned half_edges = 4 * bits;
        uint64_t half_num, half_den;
        bool first_frame = !c->analytic_clock;
        bool finishing_frame = c->analytic_frame_pending;
        bool first_cycle = !c->analytic_origin_falling;
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        __uint128_t ws_levels = 0;
        bool level = c->clock_level;
        bool ws = c->ws_level;
        unsigned clock_bit = c->clock_bit;

        if (!clock_period(s, divider, &half_num, &half_den)) {
            return;
        }
        if (first_frame) {
            c->analytic_paired = rx_analytic_paired(s);
        }
        c->analytic_clock = true;
        if (finishing_frame) {
            unsigned transitions = c->analytic_paired && first_cycle ?
                                   half_edges - 2 :
                                   (first_cycle ? half_edges - 1 : half_edges);

            /*
             * Fold the serial samples at their last-sample timestamp.  This
             * preserves FIFO, descriptor and IRQ timing without per-bit GPIO
             * events.  Eligibility guarantees the DIN level is stable.
             */
            for (unsigned edge = 0; edge < transitions; edge++) {
                c->clock_level = !c->clock_level;
                if (c->clock_level) {
                    if (c->analytic_paired) {
                        __uint128_t elapsed = (__uint128_t)(edge + 1) *
                            half_num + c->analytic_frame_remainder;
                        int64_t sample_time = c->analytic_frame_origin +
                            (elapsed + half_den - 1) / half_den;
                        s->input_level[4] = esp32_gpio_get_input_level_at(
                            s->gpio, (s->controller ? 166 : 140) + 15,
                            sample_time);
                        s->tx.ws_level = esp32_gpio_get_input_level_at(
                            s->gpio, signal_ws(s, false), sample_time);
                    } else {
                        s->input_level[4] = esp32_gpio_get_input_level(s->gpio,
                            (s->controller ? 166 : 140) + 15);
                    }
                    rx_rising(s);
                    c->clock_bit = (c->clock_bit + 1) % (2 * bits);
                } else {
                    unsigned shifted = !!(REG(s, CONF) & BIT(11));
                    unsigned right_first = !!(REG(s, CONF) & BIT(9));
                    c->ws_level = (REG(s, CONF) & BIT(13)) ?
                        ((c->clock_bit + shifted) % (2 * bits) == 0) :
                        (((c->clock_bit + shifted) / bits) & 1) ^ right_first;
                }
            }
            c->analytic_origin_falling = true;
            c->analytic_frame_pending = false;
            first_cycle = false;
            level = c->clock_level;
            ws = c->ws_level;
            clock_bit = c->clock_bit;
        }

        c->analytic_frame_origin = now;
        c->analytic_frame_remainder = c->clock_remainder;
        c->analytic_frame_half_num = half_num;
        c->analytic_frame_half_den = half_den;

        /*
         * Describe the next physical frame from the current resolved phase.
         * On initial entry the timer is at the first rising edge; later it is
         * at the preceding frame's final rising edge.
         */
        if (first_frame) {
            level = true;
            clock_bit = (clock_bit + 1) % (2 * bits);
            if (c->analytic_paired) {
                c->clock_level = true;
                c->clock_bit = clock_bit;
                s->input_level[4] = esp32_gpio_get_input_level_at(s->gpio,
                    (s->controller ? 166 : 140) + 15, now);
                s->tx.ws_level = esp32_gpio_get_input_level_at(s->gpio,
                    signal_ws(s, false), now);
                rx_rising(s);
            }
        }
        ws_levels |= (__uint128_t)ws;
        for (unsigned edge = 1; edge < half_edges; edge++) {
            level = !level;
            if (level) {
                clock_bit = (clock_bit + 1) % (2 * bits);
            } else {
                unsigned shifted = !!(REG(s, CONF) & BIT(11));
                unsigned right_first = !!(REG(s, CONF) & BIT(9));
                ws = (REG(s, CONF) & BIT(13)) ?
                    ((clock_bit + shifted) % (2 * bits) == 0) :
                    (((clock_bit + shifted) / bits) & 1) ^ right_first;
            }
            ws_levels |= (__uint128_t)ws << edge;
        }
        esp32_gpio_set_analytic_pattern(s->gpio, signal_bck(s, false), true,
            now, 1, 2, half_num, half_den, c->clock_remainder);
        esp32_gpio_set_analytic_pattern(s->gpio, signal_ws(s, false), true,
            now, ws_levels, half_edges, half_num, half_den,
            c->clock_remainder);
        c->analytic_frame_pending = true;
        unsigned steps = finishing_frame ? half_edges : half_edges - 2;
        for (unsigned edge = 0; edge < steps; edge++) {
            if (!clock_step(s, divider, &c->clock_remainder, &c->deadline)) {
                c->analytic_frame_pending = false;
                return;
            }
        }
        timer_mod_ns(c->timer, c->deadline);
        return;
    }
    if (c->analytic_clock) {
        rx_analytic_to_edges(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        return;
    }
    c->clock_level = !c->clock_level;
    if (c->clock_level) {
        rx_rising(s);
        c->clock_bit = (c->clock_bit + 1) % (2 * bits);
    } else {
        unsigned shifted = !!(REG(s, CONF) & BIT(11));
        unsigned right_first = !!(REG(s, CONF) & BIT(9));
        c->ws_level = (REG(s, CONF) & BIT(13)) ?
                      ((c->clock_bit + shifted) % (2 * bits) == 0) :
                      (((c->clock_bit + shifted) / bits) & 1) ^ right_first;
        drive(s, signal_ws(s, false), c->ws_level, true);
    }
    drive(s, signal_bck(s, false), c->clock_level, true);
    if (clock_step(s, divider, &c->clock_remainder, &c->deadline)) {
        timer_mod_ns(c->timer, c->deadline);
    }
}

static void mclk_clock(void *opaque)
{
    Esp32I2SState *s = opaque;
    if (clock_step(s, 1, &s->mclk_remainder, &s->mclk_deadline)) {
        s->mclk_level = !s->mclk_level;
        drive(s, ESP32_GPIO_MCLK0 + s->controller, s->mclk_level, true);
        timer_mod_ns(s->mclk_timer, s->mclk_deadline);
    }
}

static void input_changed(void *opaque, int n, int level)
{
    Esp32I2SState *s = opaque;
    if (n == 24) {
        if (s->tx.analytic_clock) {
            tx_analytic_to_edges(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
        if (s->rx.analytic_clock) {
            rx_analytic_to_edges(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
        if (level) {
            i2s_update(s);
        }
        return;
    }
    unsigned old = s->input_level[n];
    if (n == 4 && s->rx.analytic_clock) {
        /*
         * Preserve the prior stable DIN level through this timestamp before
         * the changed level becomes visible to the edge-scheduled receiver.
         */
        rx_analytic_to_edges(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    s->input_level[n] = !!level;
    if (n == 0 && (REG(s, 0x1c) & BIT(24))) {
        old = !old;
        level = !level;
    }
    if (!esp32_i2s_mode_supported(s)) {
        mode_diagnostics(s);
        return;
    }
    if ((REG(s, CONF2) & (BIT(5) | BIT(0))) == (BIT(5) | BIT(0))) {
        if (n == 3 && !old && level && channel_running(s, false) && !master(s, false)) {
            camera_rx(s);
        }
        return;
    }
    if (n == 0 && old && !level && channel_running(s, true) && !master(s, true)) {
        unsigned ws = s->input_level[1];
        if ((REG(s, CONF) & BIT(12)) && ws && !s->tx.previous_ws) {
            unsigned bits = sample_bits(s, true);
            s->tx.bit = (REG(s, CONF) & BIT(10)) ? 2 * bits - 1 : 0;
        } else if (!(REG(s, CONF) & BIT(12)) && ws != s->tx.previous_ws) {
            unsigned bits = sample_bits(s, true);
            unsigned shift = !!(REG(s, CONF) & BIT(10));
            unsigned right_first = !!(REG(s, CONF) & BIT(8));
            s->tx.bit = ((ws ^ right_first) * bits + 2 * bits - shift) % (2 * bits);
        }
        s->tx.previous_ws = ws;
        tx_falling(s);
    }
    if (((n == 2 && !(REG(s, CONF) & BIT(18))) ||
         (n == 0 && (REG(s, CONF) & BIT(18)))) &&
        !old && level && channel_running(s, false) && !master(s, false)) {
        rx_rising(s);
    }
}

static bool tx_analytic_eligible(Esp32I2SState *s)
{
    bool paired_rx = (REG(s, CONF) & BIT(18)) &&
        channel_running(s, false) && master(s, false) &&
        sample_bits(s, true) == sample_bits(s, false) &&
        (REG(s, SAMPLE_CONF) & 63) == ((REG(s, SAMPLE_CONF) >> 6) & 63) &&
        esp32_gpio_output_feeds_input(s->gpio, signal_bck(s, true),
                                      signal_bck(s, false)) &&
        esp32_gpio_output_feeds_input(s->gpio, signal_ws(s, true),
                                      signal_ws(s, false));

    return channel_running(s, true) && master(s, true) &&
           esp32_i2s_mode_supported(s) && !(REG(s, CONF2) & BIT(5)) &&
           !(paired_rx ?
             esp32_gpio_output_needs_edges_except_input(s->gpio,
                 signal_bck(s, true), signal_bck(s, false)) :
             esp32_gpio_output_needs_edges(s->gpio, signal_bck(s, true))) &&
           !(paired_rx ?
             esp32_gpio_output_needs_edges_except_input(s->gpio,
                 signal_ws(s, true), signal_ws(s, false)) :
             esp32_gpio_output_needs_edges(s->gpio, signal_ws(s, true))) &&
           !esp32_gpio_output_needs_edges(s->gpio, signal_data(s, true));
}

static bool rx_analytic_paired(Esp32I2SState *s)
{
    unsigned din = (s->controller ? 166 : 140) + 15;

    return (REG(s, CONF) & BIT(18)) &&
           channel_running(s, true) && channel_running(s, false) &&
           master(s, true) && master(s, false) &&
           sample_bits(s, true) == sample_bits(s, false) &&
           (REG(s, SAMPLE_CONF) & 63) == ((REG(s, SAMPLE_CONF) >> 6) & 63) &&
           esp32_gpio_output_feeds_input(s->gpio, signal_bck(s, true),
                                         signal_bck(s, false)) &&
           esp32_gpio_output_feeds_input(s->gpio, signal_ws(s, true),
                                         signal_ws(s, false)) &&
           !esp32_gpio_input_needs_edges(s->gpio, din);
}

static bool rx_analytic_eligible(Esp32I2SState *s)
{
    unsigned din = (s->controller ? 166 : 140) + 15;
    bool paired = rx_analytic_paired(s);

    return channel_running(s, false) && master(s, false) &&
           esp32_i2s_mode_supported(s) &&
           !esp32_gpio_output_needs_edges(s->gpio, signal_bck(s, false)) &&
           !esp32_gpio_output_needs_edges(s->gpio, signal_ws(s, false)) &&
           !esp32_gpio_input_needs_edges(s->gpio, din) &&
           (!(REG(s, CONF) & BIT(18)) || !channel_running(s, true) || paired);
}

static void rx_analytic_to_edges(Esp32I2SState *s, int64_t now)
{
    Esp32I2SChannel *c = &s->rx;
    unsigned signals[] = { signal_bck(s, false), signal_ws(s, false) };
    bool *levels[] = { &c->clock_level, &c->ws_level };
    int64_t next_edge = now;
    uint64_t next_remainder = c->clock_remainder;

    if (c->analytic_clock && c->analytic_frame_pending &&
        c->analytic_frame_half_num && c->analytic_frame_half_den &&
        now >= c->analytic_frame_origin) {
        __uint128_t elapsed = now - c->analytic_frame_origin;
        __uint128_t limit = (elapsed + 1) * c->analytic_frame_half_den - 1;
        __uint128_t transitions = 0;

        if (limit >= c->analytic_frame_remainder) {
            transitions = (limit - c->analytic_frame_remainder) /
                          c->analytic_frame_half_num;
        }
        unsigned bits = sample_bits(s, false);
        bool first_cycle = !c->analytic_origin_falling;
        __uint128_t edge = first_cycle ? 0 : 1;
        for (; edge <= transitions; edge++) {
            c->clock_level = !c->clock_level;
            if (c->clock_level) {
                if (c->analytic_paired) {
                    __uint128_t sample_elapsed =
                        edge * c->analytic_frame_half_num +
                        c->analytic_frame_remainder;
                    int64_t sample_time = c->analytic_frame_origin +
                        (sample_elapsed + c->analytic_frame_half_den - 1) /
                            c->analytic_frame_half_den;
                    s->input_level[4] = esp32_gpio_get_input_level_at(
                        s->gpio, (s->controller ? 166 : 140) + 15,
                        sample_time);
                    s->tx.ws_level = esp32_gpio_get_input_level_at(
                        s->gpio, signal_ws(s, false), sample_time);
                }
                rx_rising(s);
                c->clock_bit = (c->clock_bit + 1) % (2 * bits);
            } else {
                unsigned shifted = !!(REG(s, CONF) & BIT(11));
                unsigned right_first = !!(REG(s, CONF) & BIT(9));
                c->ws_level = (REG(s, CONF) & BIT(13)) ?
                    ((c->clock_bit + shifted) % (2 * bits) == 0) :
                    (((c->clock_bit + shifted) / bits) & 1) ^ right_first;
            }
        }
        c->analytic_origin_falling = true;
    }
    c->analytic_clock = false;
    c->analytic_frame_pending = false;
    c->analytic_paired = false;
    for (unsigned i = 0; i < ARRAY_SIZE(signals); i++) {
        bool level;
        int64_t edge;
        uint64_t remainder;
        if (!esp32_gpio_analytic_clock_active(s->gpio, signals[i])) {
            continue;
        }
        esp32_gpio_analytic_clock_state(s->gpio, signals[i], now, &level,
                                         &edge, &remainder);
        *levels[i] = level;
        if (i == 0) {
            next_edge = edge;
            next_remainder = remainder;
        }
        esp32_gpio_set_analytic_pattern(s->gpio, signals[i], false, 0,
                                         0, 0, 0, 0, 0);
        drive(s, signals[i], level, true);
    }
    c->deadline = next_edge;
    c->clock_remainder = next_remainder;
    if (channel_running(s, false) && master(s, false)) {
        timer_mod_ns(c->timer, next_edge);
    } else {
        timer_del(c->timer);
    }
}

static void tx_analytic_to_edges(Esp32I2SState *s, int64_t now)
{
    Esp32I2SChannel *c = &s->tx;
    unsigned signals[] = { signal_bck(s, true), signal_ws(s, true),
                           signal_data(s, true) };
    bool *levels[] = { &c->clock_level, &c->ws_level, &c->data_level };
    int64_t next_edge = now;
    uint64_t next_remainder = c->clock_remainder;

    c->analytic_clock = false;
    if (c->analytic_frame_half_num && c->analytic_frame_half_den &&
        now >= c->analytic_frame_origin) {
        __uint128_t elapsed = now - c->analytic_frame_origin;
        __uint128_t limit = (elapsed + 1) * c->analytic_frame_half_den - 1;
        if (limit >= c->analytic_frame_remainder) {
            __uint128_t transitions =
                (limit - c->analytic_frame_remainder) /
                c->analytic_frame_half_num;
            __uint128_t falls = c->analytic_origin_falling ?
                transitions / 2 : (transitions + 1) / 2;
            unsigned bits = sample_bits(s, true);
            if (bits) {
                c->bit = (1 + falls) % (2 * bits);
            }
        }
    }
    for (unsigned i = 0; i < ARRAY_SIZE(signals); i++) {
        bool level;
        int64_t edge;
        uint64_t remainder;
        if (!esp32_gpio_analytic_clock_active(s->gpio, signals[i])) {
            continue;
        }
        esp32_gpio_analytic_clock_state(s->gpio, signals[i], now, &level,
                                         &edge, &remainder);
        *levels[i] = level;
        if (i == 0) {
            next_edge = edge;
            next_remainder = remainder;
        }
        esp32_gpio_set_analytic_pattern(s->gpio, signals[i], false, 0,
                                         0, 0, 0, 0, 0);
        drive(s, signals[i], level,
              i == 2 ? channel_running(s, true) :
                       channel_running(s, true) && master(s, true));
    }
    c->deadline = next_edge;
    c->clock_remainder = next_remainder;
    if (channel_running(s, true) && master(s, true)) {
        timer_mod_ns(c->timer, next_edge);
    } else {
        timer_del(c->timer);
    }
}

static void i2s_update(Esp32I2SState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (s->tx.analytic_clock && !tx_analytic_eligible(s)) {
        tx_analytic_to_edges(s, now);
    }
    if (s->rx.analytic_clock && !rx_analytic_eligible(s)) {
        rx_analytic_to_edges(s, now);
    }
    if (!esp32_i2s_mode_supported(s)) {
        mode_diagnostics(s);
        qemu_log_mask(LOG_UNIMP,
                      "esp32.i2s%u: selected conversion, nonzero TIMING delays, "
                      "FIFO power-down or memory-copy mode is not implemented\n",
                      s->controller);
    }
    bool parallel = channel_running(s, true) && esp32_i2s_mode_supported(s) &&
                    (REG(s, CONF2) & BIT(5));
    for (unsigned bit = 0; bit < 23; bit++) {
        if (!parallel) {
            drive(s, (s->controller ? 166 : 140) + bit, false, false);
        }
    }
    for (unsigned direction = 0; direction < 2; direction++) {
        bool tx = direction == 0;
        Esp32I2SChannel *c = tx ? &s->tx : &s->rx;
        unsigned divider = (REG(s, SAMPLE_CONF) >> (tx ? 0 : 6)) & 63;
        bool was_started = c->started;
        bool running = channel_running(s, tx) && esp32_i2s_mode_supported(s);
        if (!tx && running && (REG(s, CONF2) & BIT(5)) &&
            (!(REG(s, CONF2) & BIT(0)) || master(s, false))) {
            running = false;
            capability_gap(s, 7, "LCD master RX requires an unmodeled ADC interface");
            qemu_log_mask(LOG_UNIMP,
                          "esp32.i2s%u: LCD master RX requires the unmodeled ADC "
                          "interface\n", s->controller);
        }
        unsigned bits = sample_bits(s, tx);
        if (running && (!bits || bits > 32 || divider < 2)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32.i2s%u: invalid %s width/divider\n",
                          s->controller, tx ? "TX" : "RX");
            running = false;
        }
        if (bits && bits <= 32) {
            c->bit %= 2 * bits;
            c->clock_bit %= 2 * bits;
        }
        if (running && !c->started) {
            c->bit = 0;
            c->clock_bit = 0;
            c->phase = 0;
            c->synchronized = false;
            c->clock_level = false;
            c->analytic_clock = false;
            c->analytic_frame_pending = false;
            c->analytic_origin_falling = false;
            c->analytic_paired = false;
            c->ws_level = !!(REG(s, CONF) & BIT(tx ? 8 : 9));
            c->previous_ws = c->ws_level;
            c->started = true;
            if (tx && master(s, true)) {
                if (REG(s, CONF2) & BIT(5)) {
                    lcd_tx(s);
                } else {
                    if ((REG(s, CONF) & (BIT(12) | BIT(10))) == (BIT(12) | BIT(10))) {
                        c->bit = 2 * bits - 1;
                    }
                    tx_falling(s);
                }
            }
        } else if (!running) {
            c->started = false;
            c->analytic_clock = false;
            c->analytic_frame_pending = false;
            c->analytic_origin_falling = false;
            c->analytic_paired = false;
            timer_del(c->timer);
        }
        if (tx && running && master(s, true) && was_started && c->started &&
            !c->frame_valid && c->bit == 0 && c->fifo_count &&
            !(REG(s, CONF2) & BIT(5))) {
            tx_falling(s);
        }
        drive(s, signal_ws(s, tx), c->ws_level, running && master(s, tx));
        if (tx && !parallel) {
            drive(s, signal_data(s, true), c->data_level, running);
        }
        drive(s, signal_bck(s, tx), c->clock_level, running && master(s, tx));
        if (running && master(s, tx) && !timer_pending(c->timer)) {
            if (tx && (REG(s, CONF1) & BIT(8)) && !c->frame_valid && !c->fifo_count) {
                continue;
            }
            c->deadline = now;
            if (clock_step(s, divider, &c->clock_remainder, &c->deadline)) {
                timer_mod_ns(c->timer, c->deadline);
            }
        } else if (!master(s, tx)) {
            timer_del(c->timer);
        }
    }
    unsigned mclk_signal = ESP32_GPIO_MCLK0 + s->controller;
    uint64_t period_num, period_den;
    bool clock = esp32_gpio_output_is_routed(s->gpio, mclk_signal) &&
                 clock_period(s, 1, &period_num, &period_den);
    bool lazy_clock = clock &&
                      !esp32_gpio_output_needs_edges(s->gpio, mclk_signal);

    if (lazy_clock) {
        bool initial_level = s->mclk_level;
        int64_t next_edge;
        uint64_t before_next;

        if (esp32_gpio_analytic_clock_active(s->gpio, mclk_signal)) {
            esp32_gpio_analytic_clock_state(s->gpio, mclk_signal, now,
                                             &s->mclk_level, &next_edge,
                                             &before_next);
        } else {
            uint64_t remainder_before_first;
            int64_t origin;
            if (timer_pending(s->mclk_timer)) {
                uint64_t residue = period_num % period_den;
                remainder_before_first =
                    (s->mclk_remainder + period_den - residue) % period_den;
                int64_t interval = (period_num + remainder_before_first) /
                                   period_den;
                origin = s->mclk_deadline - interval;
            } else {
                remainder_before_first = s->mclk_remainder;
                origin = now;
            }
            esp32_gpio_set_analytic_clock(s->gpio, mclk_signal, true,
                                          origin, initial_level, period_num,
                                          period_den,
                                          remainder_before_first);
            esp32_gpio_analytic_clock_state(s->gpio, mclk_signal, now,
                                             &s->mclk_level, &next_edge,
                                             &before_next);
        }
        if (lazy_clock) {
            s->mclk_deadline = next_edge;
            s->mclk_remainder = before_next;
            drive(s, mclk_signal, s->mclk_level, true);
            timer_del(s->mclk_timer);
        }
    }
    if (clock && !lazy_clock) {
        if (esp32_gpio_analytic_clock_active(s->gpio, mclk_signal)) {
            int64_t next_edge;
            uint64_t before_next;
            esp32_gpio_analytic_clock_state(s->gpio, mclk_signal, now,
                                             &s->mclk_level, &next_edge,
                                             &before_next);
            s->mclk_deadline = next_edge;
            s->mclk_remainder = before_next;
            esp32_gpio_set_analytic_clock(s->gpio, mclk_signal, false, 0,
                                          false, 0, 0, 0);
            drive(s, mclk_signal, s->mclk_level, true);
            timer_mod_ns(s->mclk_timer, next_edge);
        } else {
            drive(s, mclk_signal, s->mclk_level, true);
            if (!timer_pending(s->mclk_timer)) {
                uint64_t remainder = s->mclk_remainder;
                int64_t deadline = now;
                if (clock_step(s, 1, &remainder, &deadline)) {
                    s->mclk_remainder = remainder;
                    s->mclk_deadline = deadline;
                    timer_mod_ns(s->mclk_timer, deadline);
                }
            }
        }
    } else if (!clock) {
        if (esp32_gpio_analytic_clock_active(s->gpio, mclk_signal)) {
            int64_t next_edge;
            uint64_t before_next;
            esp32_gpio_analytic_clock_state(s->gpio, mclk_signal, now,
                                             &s->mclk_level, &next_edge,
                                             &before_next);
            esp32_gpio_set_analytic_clock(s->gpio, mclk_signal, false, 0,
                                          false, 0, 0, 0);
        }
        drive(s, mclk_signal, s->mclk_level, false);
        timer_del(s->mclk_timer);
    }
    if ((REG(s, CLKM_CONF) & BIT(21)) && !s->apll_hz) {
        qemu_log_mask(LOG_UNIMP,
                      "esp32.i2s%u: APLL analog clock source unavailable\n",
                      s->controller);
    }
    dma_pump(s);
    if (!esp32_i2s_mode_supported(s)) {
        /* An unmodeled active transmitter is unknown, not electrical release
         * or a fabricated standard-I2S waveform. No DMA success is produced. */
        if (channel_running(s, true)) {
            if (REG(s, CONF2) & BIT(5)) {
                for (unsigned bit = 0; bit < 24; bit++) {
                    esp32_gpio_set_peripheral_unknown(s->gpio,
                        (s->controller ? 166 : 140) + bit);
                }
            } else {
                esp32_gpio_set_peripheral_unknown(s->gpio, signal_data(s, true));
            }
            if (master(s, true)) {
                esp32_gpio_set_peripheral_unknown(s->gpio, signal_bck(s, true));
                esp32_gpio_set_peripheral_unknown(s->gpio, signal_ws(s, true));
            }
        }
        if (channel_running(s, false) && master(s, false)) {
            esp32_gpio_set_peripheral_unknown(s->gpio, signal_bck(s, false));
            esp32_gpio_set_peripheral_unknown(s->gpio, signal_ws(s, false));
        }
    }
    qemu_set_irq(s->irq, !!(REG(s, INT_RAW) & REG(s, INT_ENA)));
}

void esp32_i2s_connect_gpio(Esp32I2SState *s, Esp32GpioState *gpio,
                          unsigned controller)
{
    s->gpio = gpio;
    s->controller = controller;
    esp32_gpio_add_routing_listener(gpio, s->inputs[24]);
    esp32_gpio_set_peripheral_input(gpio, signal_bck(s, true), s->inputs[0]);
    esp32_gpio_set_peripheral_input(gpio, signal_ws(s, true), s->inputs[1]);
    esp32_gpio_set_peripheral_input(gpio, signal_bck(s, false), s->inputs[2]);
    esp32_gpio_set_peripheral_input(gpio, signal_ws(s, false), s->inputs[3]);
    for (unsigned bit = 0; bit < 16; bit++) {
        unsigned signal = (controller ? 166 : 140) + bit;
        esp32_gpio_set_peripheral_input(gpio, signal, s->inputs[bit == 15 ? 4 : bit + 5]);
    }
    for (unsigned sync = 0; sync < 3; sync++) {
        esp32_gpio_set_peripheral_input(gpio, (controller ? 193 : 190) + sync,
                                        s->inputs[21 + sync]);
    }
    i2s_update(s);
}

void esp32_i2s_set_enabled(Esp32I2SState *s, bool enabled)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (s->enabled == enabled) {
        return;
    }
    if (!enabled) {
        if (s->tx.analytic_clock) {
            tx_analytic_to_edges(s, now);
        }
        if (s->rx.analytic_clock) {
            rx_analytic_to_edges(s, now);
        }
    }
    if (s->rx.analytic_clock) {
        rx_analytic_to_edges(s, now);
    }
    s->enabled = enabled;
    for (unsigned i = 0; i < 2; i++) {
        Esp32I2SChannel *c = i ? &s->rx : &s->tx;
        if (!enabled) {
            c->paused_ns = timer_pending(c->timer) ? MAX(1, c->deadline - now) : 0;
            timer_del(c->timer);
        } else if (c->paused_ns) {
            c->deadline = now + c->paused_ns;
            timer_mod_ns(c->timer, c->deadline);
            c->paused_ns = 0;
        }
    }
    if (!enabled) {
        s->mclk_paused_ns = timer_pending(s->mclk_timer) ?
                            MAX(1, s->mclk_deadline - now) : 0;
        timer_del(s->mclk_timer);
    } else {
        if (s->mclk_paused_ns) {
            s->mclk_deadline = now + s->mclk_paused_ns;
            timer_mod_ns(s->mclk_timer, s->mclk_deadline);
            s->mclk_paused_ns = 0;
        }
        i2s_update(s);
    }
}

void esp32_i2s_set_apll(Esp32I2SState *s, uint32_t hz)
{
    esp32_i2s_set_apll_rate(s, hz, 1);
}

void esp32_i2s_set_apll_rate(Esp32I2SState *s, uint64_t numerator,
                             uint64_t denominator)
{
    if (s->tx.analytic_clock) {
        tx_analytic_to_edges(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    if (s->rx.analytic_clock) {
        rx_analytic_to_edges(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    s->apll_numerator = numerator;
    s->apll_denominator = denominator ? denominator : 1;
    s->apll_hz = numerator / s->apll_denominator;
    i2s_update(s);
}

static uint64_t i2s_read(void *opaque, hwaddr address, unsigned size)
{
    Esp32I2SState *s = opaque;
    uint32_t word = 0;
    if (address >= 0x100) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32.i2s%u: invalid read 0x%" HWADDR_PRIx "\n",
                      s->controller, address);
        return 0;
    }
    switch (address) {
    case 0x04:
        if (!fifo_pop(s, false, &word)) {
            interrupt(s, INT_RX_EMPTY);
        }
        return word;
    case 0x10:
        return REG(s, INT_RAW) & REG(s, INT_ENA);
    case OUT_LINK:
        return REG(s, OUT_LINK) | (s->tx.link_active ? 0 : BIT(31));
    case IN_LINK:
        return REG(s, IN_LINK) | (s->rx.link_active ? 0 : BIT(31));
    case 0xbc:
        capability_gap(s, 8, "FIFO reset-back polarity conflicts in TRM/SDK");
        qemu_log_mask(LOG_UNIMP,
                      "esp32.i2s%u: FIFO reset-back status polarity conflicts "
                      "between TRM and SDK; SDK reset bits are retained\n",
                      s->controller);
        return (!s->tx.started || (!s->tx.frame_valid && !s->tx.fifo_count)) | 6;
    case 0x6c: case 0x70:
        capability_gap(s, 9, "DMA FSM status encoding is undocumented");
        qemu_log_mask(LOG_UNIMP,
                      "esp32.i2s%u: DMA FSM status encoding is undocumented\n",
                      s->controller);
        return 0;
    default:
        return REG(s, address) & i2s_reg_info[address / 4].read_mask;
    }
}

static void fifo_reset(Esp32I2SChannel *c)
{
    c->fifo_head = c->fifo_count = 0;
    memset(c->fifo, 0, sizeof(c->fifo));
    memset(c->fifo_flags, 0, sizeof(c->fifo_flags));
}

static void link_write(Esp32I2SState *s, bool tx, uint32_t value)
{
    Esp32I2SChannel *c = tx ? &s->tx : &s->rx;
    unsigned address = tx ? OUT_LINK : IN_LINK;
    REG(s, address) = value & LINK_ADDRESS;
    if (value & BIT(28)) {
        c->link_active = false;
    }
    if (value & (BIT(29) | BIT(30))) {
        if ((value & BIT(29)) || !c->descriptor) {
            c->descriptor = 0x3ff00000 | (value & LINK_ADDRESS);
        }
        c->descriptor_loaded = false;
        c->link_active = true;
        c->dma_words = 0;
    }
    dma_pump(s);
    i2s_update(s);
}

static void i2s_write(void *opaque, hwaddr address, uint64_t value, unsigned size)
{
    Esp32I2SState *s = opaque;
    uint32_t mask, old;
    if (address >= 0x100) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32.i2s%u: invalid write 0x%" HWADDR_PRIx "\n",
                      s->controller, address);
        return;
    }
    mask = i2s_reg_info[address / 4].write_mask;
    old = REG(s, address);
    value &= mask;
    if (!mask) {
        return;
    }
    if (address != 0x00 && address != 0x04 && address != 0x18 &&
        address != 0x14) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        if (s->tx.analytic_clock) {
            tx_analytic_to_edges(s, now);
        }
        if (s->rx.analytic_clock) {
            rx_analytic_to_edges(s, now);
        }
    }
    switch (address) {
    case 0x00:
        if (s->tx.fifo_count + (fifo_mode(s, true) == 1 ? 2 : 1) > ESP32_I2S_FIFO_WORDS) {
            interrupt(s, INT_TX_FULL);
        } else if (!(REG(s, CONF) & BIT(2))) {
            if (fifo_mode(s, true) == 1) {
                fifo_push(&s->tx, value & 0xffff0000, 0, 0);
                fifo_push(&s->tx, value << 16, 0, 0);
            } else {
                fifo_push(&s->tx, value, 0, 0);
            }
            fifo_interrupts(s);
            i2s_update(s);
        }
        return;
    case 0x18:
        REG(s, INT_RAW) &= ~value;
        qemu_set_irq(s->irq, !!(REG(s, INT_RAW) & REG(s, INT_ENA)));
        return;
    case OUT_LINK: case IN_LINK:
        link_write(s, address == OUT_LINK, value);
        return;
    case CONF:
        if (value & BIT(2)) {
            fifo_reset(&s->tx);
        }
        if (value & BIT(3)) {
            fifo_reset(&s->rx);
        }
        if (value & BIT(0)) {
            s->tx.bit = 0;
            s->tx.frame_valid = false;
            s->tx.started = false;
        }
        if (value & BIT(1)) {
            s->rx.bit = 0;
            s->rx.started = false;
            s->rx.synchronized = false;
            s->rx.mono_pending = false;
            memset(s->rx.frame, 0, sizeof(s->rx.frame));
        }
        break;
    case LC_CONF:
        if (value & (BIT(1) | BIT(3))) {
            s->tx.link_active = s->tx.descriptor_loaded = false;
            s->tx.descriptor = 0;
        }
        if (value & (BIT(0) | BIT(3))) {
            s->rx.link_active = s->rx.descriptor_loaded = false;
            s->rx.descriptor = 0;
        }
        break;
    case 0x80 ... 0x9c:
        if (value != old) {
            qemu_log_mask(LOG_UNIMP,
                          "esp32.i2s%u: CVSD/PLC processing not modeled\n",
                          s->controller);
        }
        break;
    case 0x44: case 0x64: case 0x68: case 0x74:
        if (value != old) {
            capability_gap(s, 10, "DMA cmdFIFO/test/hung behavior is not modeled");
        }
        qemu_log_mask(LOG_UNIMP,
                      "esp32.i2s%u: DMA test/hung configuration is not "
                      "implemented\n", s->controller);
        break;
    case 0xa4:
        if (value != old) {
            qemu_log_mask(LOG_UNIMP,
                          "esp32.i2s%u: FIFO/PLC power-down retention is not "
                          "modeled\n", s->controller);
        }
        break;
    }
    REG(s, address) = (old & ~mask) | value;
    i2s_update(s);
}

static const MemoryRegionOps i2s_ops = {
    .read = i2s_read, .write = i2s_write, .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void i2s_reset_hold(Object *obj, ResetType type)
{
    Esp32I2SState *s = ESP32_I2S(obj);
    for (unsigned i = 0; i < 64; i++) {
        s->regs[i] = i2s_reg_info[i].reset;
    }
    QEMUTimer *tx_timer = s->tx.timer;
    QEMUTimer *rx_timer = s->rx.timer;
    timer_del(tx_timer);
    timer_del(rx_timer);
    timer_del(s->mclk_timer);
    memset(&s->tx, 0, sizeof(s->tx));
    memset(&s->rx, 0, sizeof(s->rx));
    s->tx.timer = tx_timer;
    s->rx.timer = rx_timer;
    s->mclk_level = false;
    s->mclk_remainder = 0;
    s->mclk_deadline = 0;
    s->mclk_paused_ns = 0;
    s->pumping_dma = false;
    s->warned_capabilities = 0;
    i2s_update(s);
    REG(s, INT_RAW) = 0;
    qemu_irq_lower(s->irq);
}

static void i2s_init(Object *obj)
{
    Esp32I2SState *s = ESP32_I2S(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    memory_region_init_io(&s->iomem, obj, &i2s_ops, s, TYPE_ESP32_I2S, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->tx.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tx_clock, s);
    s->rx.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, rx_clock, s);
    s->mclk_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mclk_clock, s);
    s->apll_denominator = 1;
    s->inputs = qemu_allocate_irqs(input_changed, s, 25);
    s->enabled = true;
}

static void i2s_finalize(Object *obj)
{
    Esp32I2SState *s = ESP32_I2S(obj);
    timer_free(s->tx.timer);
    timer_free(s->rx.timer);
    timer_free(s->mclk_timer);
    qemu_free_irqs(s->inputs, 25);
}

static void i2s_unrealize(DeviceState *dev)
{
    Esp32I2SState *s = ESP32_I2S(dev);

    timer_del(s->tx.timer);
    timer_del(s->rx.timer);
    timer_del(s->mclk_timer);
    if (s->gpio) {
        esp32_gpio_remove_routing_listener(s->gpio, s->inputs[24]);
        esp32_gpio_set_peripheral_input(s->gpio, signal_bck(s, true), NULL);
        esp32_gpio_set_peripheral_input(s->gpio, signal_ws(s, true), NULL);
        esp32_gpio_set_peripheral_input(s->gpio, signal_bck(s, false), NULL);
        esp32_gpio_set_peripheral_input(s->gpio, signal_ws(s, false), NULL);
        drive(s, signal_bck(s, true), false, false);
        drive(s, signal_ws(s, true), false, false);
        drive(s, signal_bck(s, false), false, false);
        drive(s, signal_ws(s, false), false, false);
        for (unsigned bit = 0; bit < 24; bit++) {
            unsigned signal = (s->controller ? 166 : 140) + bit;
            if (bit < 16) {
                esp32_gpio_set_peripheral_input(s->gpio, signal, NULL);
            }
            drive(s, signal, false, false);
        }
        for (unsigned sync = 0; sync < 3; sync++) {
            esp32_gpio_set_peripheral_input(s->gpio,
                (s->controller ? 193 : 190) + sync, NULL);
        }
        drive(s, ESP32_GPIO_MCLK0 + s->controller, false, false);
        s->gpio = NULL;
    }
}

static void i2s_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    dc->vmsd = &vmstate_esp32_i2s;
    dc->unrealize = i2s_unrealize;
    rc->phases.hold = i2s_reset_hold;
}

static const TypeInfo i2s_info = {
    .name = TYPE_ESP32_I2S, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32I2SState), .instance_init = i2s_init,
    .instance_finalize = i2s_finalize, .class_init = i2s_class_init,
};

static void i2s_register_types(void)
{
    type_register_static(&i2s_info);
}
type_init(i2s_register_types)
