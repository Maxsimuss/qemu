/*
 * Virtual I2S DAC / audio recorder fixture for ESP32 board tests.
 *
 * The device is an electrical peer. It never touches the ESP32 I2C or I2S
 * controllers: it listens to the *resolved* GPIO pad nodes (after GPIO matrix,
 * IO MUX, pulls and external drivers) and drives SDA only as open drain through
 * an external-driver slot of the GPIO model.
 *
 * Control (I2C, 7-bit address 0x4c by default). The first byte after the
 * address phase is the register pointer; following write bytes go to the
 * pointer, which auto-increments. A read returns the register at the pointer
 * and auto-increments.
 *
 *   0x00 channels   1 or 2             (default 2)
 *   0x01 bit depth  16, 24 or 32       (default 16)
 *   0x02 rate id    0=8k 1=16k 2=32k 3=48k  (default 3)
 *   0x03 record     0 idle, 1 record
 *   0x04 status (read only)  bit0 sticky clock/framing error
 *                            bit1 configuration or other error
 *                            bit2 recording
 *   0x05..0x08 frames written in the current/last recording, little endian,
 *              latched when register 0x05 is read (read only)
 *
 * Data (Philips I2S, slave, MSB first, data sampled on BCLK rising edge, WS
 * low = left, WS changes one BCLK before the slot MSB). Slot width equals the
 * bit depth, so BCLK = 2 * depth * Fs. The frame rate is measured from WS
 * falling-edge timestamps against the programmed rate id (1% tolerance) and
 * every frame must contain exactly 2 * depth BCLK rising edges. The MCLK
 * multiple (256 * Fs, 384 * Fs for 24 bit) is a configuration contract, not
 * a measured signal: MCLK edges are perfectly periodic by divider
 * construction, so counting tens of millions of them adds simulation cost
 * without verification content beyond what WS rate plus BCLK-per-frame
 * already prove. MCLK input, if present, is ignored.
 *
 * Mono (channels = 1) stores the left slot; the right slot is still checked.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "sysemu/sysemu.h"

#define TYPE_ESP32_I2S_DAC "esp32-i2s-dac"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32I2SDac, ESP32_I2S_DAC)

#define REG_CHANNELS 0x00
#define REG_DEPTH    0x01
#define REG_RATE     0x02
#define REG_RECORD   0x03
#define REG_STATUS   0x04
#define REG_FRAMES   0x05
#define REG_LAST     0x08

#define ST_CLOCK_ERROR BIT(0)
#define ST_CONFIG_ERROR BIT(1)
#define ST_RECORDING BIT(2)

#define WAV_HEADER_SIZE 44
#define STAGE_SIZE 65536
#define FLUSH_MS 250
/* A BCLK gap longer than this many nominal bit periods restarts the stream. */
#define GAP_BIT_PERIODS 16

enum I2cPhase {
    I2C_IDLE, I2C_ADDRESS, I2C_WRITE, I2C_READ,
    I2C_ACK_SETUP, I2C_ACK_CLOCK, I2C_ACK_END,
    I2C_READ_ACK_SETUP, I2C_READ_ACK, I2C_READ_ACK_END, I2C_IGNORE,
};

enum PadIndex {
    PIN_SDA, PIN_SCL, PIN_BCLK, PIN_WS, PIN_DATA, PIN_COUNT,
};

static const uint32_t rate_table[] = {8000, 16000, 32000, 48000};

struct Esp32I2SDac {
    DeviceState parent;

    /* Properties */
    Esp32GpioState *gpio;
    char *wav;
    uint32_t pad[PIN_COUNT];
    uint32_t driver_slot;
    uint32_t address;
    uint64_t max_bytes;

    /* Registers */
    uint8_t channels, depth, rate_id;
    bool recording;
    bool clock_error, config_error;
    uint32_t frames;
    uint32_t frames_latched;

    /* I2C target */
    uint8_t phase, next, bits, shift, pointer;
    bool ack, expect_pointer, sda_low, scl_low;

    /* Resolved pad levels (Esp32PadLevel) */
    uint8_t level[PIN_COUNT];
    qemu_irq sink[PIN_COUNT];

    /* Stream measurement and decoding */
    uint64_t bclk_rises;
    int64_t last_bclk_ns;
    bool bclk_seen;
    uint32_t stream_edges;
    bool synced, provisional, ws_prev, left_valid, frame_bad, data_bad;
    bool snap_valid, interval_ok, pending_valid;
    bool right_slot;
    unsigned slot_bits;
    uint32_t slot_shift;
    uint32_t left_value, pending_left, pending_right;
    int64_t snap_ns;
    uint64_t snap_bclk;

    /* WAV output */
    int fd;
    bool header_failed;
    uint64_t data_bytes;
    uint8_t *stage;
    uint32_t stage_len;
    QEMUTimer *flush_timer;
    Notifier exit_notifier;
};

static inline bool level_binary(uint8_t level)
{
    return level == ESP32_PAD_LOW || level == ESP32_PAD_HIGH;
}

/* ---------------------------------------------------------------- status */

static uint8_t dac_status(Esp32I2SDac *s)
{
    return (s->clock_error ? ST_CLOCK_ERROR : 0) |
           (s->config_error ? ST_CONFIG_ERROR : 0) |
           (s->recording ? ST_RECORDING : 0);
}

static void flag_clock_error(Esp32I2SDac *s, const char *why)
{
    if (!s->clock_error) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32-i2s-dac: clock error: %s\n", why);
    }
    s->clock_error = true;
}

static void flag_config_error(Esp32I2SDac *s, const char *why)
{
    if (!s->config_error) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32-i2s-dac: error: %s\n", why);
    }
    s->config_error = true;
}

/* ------------------------------------------------------------- WAV file */

static void wav_header(Esp32I2SDac *s, uint8_t h[WAV_HEADER_SIZE],
                       bool final)
{
    uint32_t rate = rate_table[s->rate_id];
    uint32_t bytes = s->depth / 8;
    uint32_t align = bytes * s->channels;
    uint64_t data = MIN(s->data_bytes, UINT32_MAX - 64);
    uint64_t riff = 36 + data + ((final && (data & 1)) ? 1 : 0);

    memcpy(h, "RIFF", 4);
    stl_le_p(h + 4, riff);
    memcpy(h + 8, "WAVEfmt ", 8);
    stl_le_p(h + 16, 16);
    stw_le_p(h + 20, 1); /* integer PCM */
    stw_le_p(h + 22, s->channels);
    stl_le_p(h + 24, rate);
    stl_le_p(h + 28, rate * align);
    stw_le_p(h + 32, align);
    stw_le_p(h + 34, s->depth);
    memcpy(h + 36, "data", 4);
    stl_le_p(h + 40, data);
}

static bool wav_write_header(Esp32I2SDac *s, bool final)
{
    uint8_t h[WAV_HEADER_SIZE];

    wav_header(s, h, final);
    if (lseek(s->fd, 0, SEEK_SET) < 0 ||
        qemu_write_full(s->fd, h, sizeof(h)) != sizeof(h) ||
        lseek(s->fd, 0, SEEK_END) < 0) {
        return false;
    }
    return true;
}

static void wav_fail(Esp32I2SDac *s, const char *what)
{
    if (!s->header_failed) {
        error_report("esp32-i2s-dac: %s '%s': %s", what, s->wav,
                     strerror(errno));
    }
    s->header_failed = true;
    flag_config_error(s, "WAV file write failed");
}

/* Write staged samples and refresh the header so a killed QEMU leaves a
 * readable file. */
static void wav_flush(Esp32I2SDac *s)
{
    if (s->fd < 0) {
        return;
    }
    if (s->stage_len &&
        qemu_write_full(s->fd, s->stage, s->stage_len) != s->stage_len) {
        wav_fail(s, "cannot write");
    }
    s->stage_len = 0;
    if (!wav_write_header(s, false)) {
        wav_fail(s, "cannot update header of");
    }
    timer_del(s->flush_timer);
}

static void wav_close(Esp32I2SDac *s)
{
    if (s->fd < 0) {
        return;
    }
    wav_flush(s);
    if (s->data_bytes & 1) {
        static const uint8_t pad;
        if (qemu_write_full(s->fd, &pad, 1) != 1) {
            wav_fail(s, "cannot pad");
        }
    }
    if (!wav_write_header(s, true)) {
        wav_fail(s, "cannot finalize header of");
    }
    if (close(s->fd) < 0) {
        wav_fail(s, "cannot close");
    }
    s->fd = -1;
}

static void wav_open(Esp32I2SDac *s)
{
    s->data_bytes = 0;
    s->stage_len = 0;
    s->header_failed = false;
    if (!s->wav) {
        return;
    }
    s->fd = qemu_open_old(s->wav, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY,
                          0644);
    if (s->fd < 0) {
        wav_fail(s, "cannot open");
        return;
    }
    if (!wav_write_header(s, false)) {
        wav_fail(s, "cannot write header of");
        close(s->fd);
        s->fd = -1;
    }
}

static void flush_timer_cb(void *opaque)
{
    wav_flush(opaque);
}

static void record_stop(Esp32I2SDac *s)
{
    if (s->recording) {
        wav_close(s);
    }
    s->recording = false;
}

static void exit_notify(Notifier *n, void *data)
{
    Esp32I2SDac *s = container_of(n, Esp32I2SDac, exit_notifier);

    record_stop(s);
}

static void store_sample(Esp32I2SDac *s, uint32_t value, unsigned bytes)
{
    for (unsigned i = 0; i < bytes; i++) {
        s->stage[s->stage_len++] = value >> (8 * i);
    }
}

/* Returns false when the file size limit stopped the recording. */
static bool store_frame(Esp32I2SDac *s, uint32_t left, uint32_t right)
{
    unsigned bytes = s->depth / 8;
    unsigned size = bytes * s->channels;

    if (s->data_bytes + size + WAV_HEADER_SIZE > s->max_bytes) {
        flag_config_error(s, "recording size limit reached");
        record_stop(s);
        return false;
    }
    s->frames++;
    s->data_bytes += size;
    if (s->fd < 0) {
        return true; /* no file configured: only count */
    }
    store_sample(s, left, bytes);
    if (s->channels == 2) {
        store_sample(s, right, bytes);
    }
    if (s->stage_len + 8 > STAGE_SIZE) {
        wav_flush(s);
    } else if (!timer_pending(s->flush_timer)) {
        timer_mod(s->flush_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + FLUSH_MS);
    }
    return true;
}

/* -------------------------------------------------------------- decoding */

static void lose_sync(Esp32I2SDac *s, bool flag, const char *why)
{
    if (flag && s->recording) {
        flag_clock_error(s, why);
    }
    s->synced = false;
    s->provisional = false;
    s->snap_valid = false;
    s->interval_ok = false;
    s->pending_valid = false;
    s->left_valid = false;
    s->frame_bad = false;
    s->data_bad = false;
    s->slot_bits = 0;
    s->slot_shift = 0;
}

/* WS fell: close a measured interval and open the next one. */
static void ws_fell(Esp32I2SDac *s, int64_t now)
{
    s->interval_ok = false;
    if (s->snap_valid) {
        uint64_t rate = rate_table[s->rate_id];
        uint64_t dt = now - s->snap_ns;
        uint64_t dbclk = s->bclk_rises - s->snap_bclk;
        /* |dt - 1/rate| <= 1% of 1/rate, in integer arithmetic */
        uint64_t scaled = dt * rate;
        bool rate_ok = (scaled > 1000000000ull ? scaled - 1000000000ull :
                        1000000000ull - scaled) <= 10000000ull;
        bool bclk_ok = dbclk == 2u * s->depth;

        if (!rate_ok) {
            flag_clock_error(s, "frame rate differs from programmed rate");
        } else if (!bclk_ok) {
            flag_clock_error(s, "BCLK count per frame is not 2 * bit depth");
        }
        s->interval_ok = rate_ok && bclk_ok;
    }
    s->snap_valid = true;
    s->snap_ns = now;
    s->snap_bclk = s->bclk_rises;
}

static void frame_complete(Esp32I2SDac *s, uint32_t right)
{
    bool good = s->left_valid && !s->frame_bad && !s->data_bad;
    uint32_t left = s->left_value;

    s->left_valid = false;
    s->frame_bad = false;
    s->data_bad = false;
    if (!good) {
        s->pending_valid = false;
        return;
    }
    if (s->provisional) {
        /* First frame of a stream: its own interval cannot be measured, keep
         * it until the next frame has a verified interval. */
        s->provisional = false;
        s->pending_valid = true;
        s->pending_left = left;
        s->pending_right = right;
        return;
    }
    if (!s->interval_ok) {
        s->pending_valid = false;
        return;
    }
    if (s->pending_valid) {
        s->pending_valid = false;
        if (!store_frame(s, s->pending_left, s->pending_right)) {
            return;
        }
    }
    store_frame(s, left, right);
}

static void slot_complete(Esp32I2SDac *s)
{
    /* The slot that just ended belongs to the previous WS polarity. */
    if (s->slot_bits != s->depth) {
        s->frame_bad = true;
        flag_clock_error(s, "slot length differs from bit depth");
    }
    if (!s->right_slot) {
        s->left_value = s->slot_shift;
        s->left_valid = true;
    } else {
        frame_complete(s, s->slot_shift);
    }
}

static void bclk_rise(Esp32I2SDac *s, int64_t now)
{
    uint64_t nominal = 1000000000ull /
                       ((uint64_t)rate_table[s->rate_id] * 2 * s->depth);
    bool ws_high, bit, change;

    if (s->bclk_seen && (uint64_t)(now - s->last_bclk_ns) >
                        GAP_BIT_PERIODS * nominal) {
        /* Clock stopped (for instance a TX underrun). Not an error between
         * frames; inside a frame the frame is lost. */
        lose_sync(s, s->synced && s->slot_bits, "BCLK stopped inside a frame");
        s->stream_edges = 0;
    }
    s->bclk_seen = true;
    s->last_bclk_ns = now;
    s->stream_edges++;
    if (!s->recording) {
        return;
    }
    if (!level_binary(s->level[PIN_WS])) {
        lose_sync(s, s->synced, "WS floating or contended");
        return;
    }
    ws_high = s->level[PIN_WS] == ESP32_PAD_HIGH;
    change = ws_high != s->ws_prev;
    s->ws_prev = ws_high;
    bit = s->level[PIN_DATA] == ESP32_PAD_HIGH;

    if (!s->synced) {
        if (s->stream_edges == 1 && !ws_high) {
            /* A transmitter that starts with WS low puts the left MSB on its
             * first BCLK rising edge. */
            s->synced = true;
            s->provisional = true;
            s->right_slot = false;
            s->slot_bits = 0;
            s->slot_shift = 0;
        } else if (change && !ws_high) {
            /* Frame boundary: this bit is the previous right slot's LSB. */
            s->synced = true;
            s->provisional = false;
            s->right_slot = false;
            s->slot_bits = 0;
            s->slot_shift = 0;
            return;
        } else {
            return;
        }
        change = false;
    }
    if (!level_binary(s->level[PIN_DATA])) {
        s->data_bad = true;
        flag_config_error(s, "DATA floating or contended while recording");
    }

    if (change) {
        /* This bit is the last bit of the slot that is ending. */
        if (s->slot_bits < 32) {
            s->slot_shift = (s->slot_shift << 1) | bit;
            s->slot_bits++;
        }
        slot_complete(s);
        s->right_slot = ws_high;
        s->slot_bits = 0;
        s->slot_shift = 0;
        return;
    }
    if (s->slot_bits < s->depth) {
        s->slot_shift = (s->slot_shift << 1) | bit;
        s->slot_bits++;
    } else {
        s->frame_bad = true;
        flag_clock_error(s, "slot longer than bit depth");
    }
}

static void pad_event(void *opaque, int n, int value)
{
    Esp32I2SDac *s = opaque;
    uint8_t previous = s->level[n];
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->level[n] = value;
    switch (n) {
    case PIN_BCLK:
        if (!level_binary(value)) {
            s->stream_edges = 0;
            s->bclk_seen = false;
            if (s->recording) {
                lose_sync(s, s->synced, "BCLK floating or contended");
            }
        } else if (previous == ESP32_PAD_LOW && value == ESP32_PAD_HIGH) {
            s->bclk_rises++;
            bclk_rise(s, now);
        }
        break;
    case PIN_WS:
        if (!s->recording) {
            break;
        }
        if (!level_binary(value)) {
            lose_sync(s, s->synced, "WS floating or contended");
        } else if (previous == ESP32_PAD_HIGH && value == ESP32_PAD_LOW) {
            ws_fell(s, now);
        }
        break;
    default:
        break;
    }
}

/* ----------------------------------------------------------- registers */

static void record_start(Esp32I2SDac *s)
{
    s->clock_error = false;
    s->config_error = false;
    s->frames = 0;
    wav_open(s);
    if (s->wav && s->fd < 0) {
        return; /* error flagged, stay idle */
    }
    lose_sync(s, false, NULL);
    s->ws_prev = s->level[PIN_WS] == ESP32_PAD_HIGH;
    s->recording = true;
}

static void reg_write(Esp32I2SDac *s, uint8_t reg, uint8_t value)
{
    switch (reg) {
    case REG_CHANNELS:
    case REG_DEPTH:
    case REG_RATE: {
        uint8_t *field = reg == REG_CHANNELS ? &s->channels :
                         reg == REG_DEPTH ? &s->depth : &s->rate_id;
        bool legal = reg == REG_CHANNELS ? (value == 1 || value == 2) :
                     reg == REG_DEPTH ? (value == 16 || value == 24 ||
                                         value == 32) :
                     value < ARRAY_SIZE(rate_table);
        if (!legal) {
            flag_config_error(s, "unsupported register value");
        } else if (s->recording && *field != value) {
            flag_config_error(s, "format change while recording");
        } else {
            *field = value;
        }
        break;
    }
    case REG_RECORD:
        if (value == 1) {
            if (!s->recording) {
                record_start(s);
            }
        } else if (value == 0) {
            record_stop(s);
        } else {
            flag_config_error(s, "invalid record value");
        }
        break;
    default:
        flag_config_error(s, "write to read-only or unknown register");
        break;
    }
}

static uint8_t reg_read(Esp32I2SDac *s, uint8_t reg)
{
    switch (reg) {
    case REG_CHANNELS: return s->channels;
    case REG_DEPTH: return s->depth;
    case REG_RATE: return s->rate_id;
    case REG_RECORD: return s->recording;
    case REG_STATUS: return dac_status(s);
    case REG_FRAMES:
        s->frames_latched = s->frames;
        /* fall through */
    case REG_FRAMES + 1:
    case REG_FRAMES + 2:
    case REG_LAST:
        return s->frames_latched >> (8 * (reg - REG_FRAMES));
    default:
        flag_config_error(s, "read of unknown register");
        return 0;
    }
}

/* ------------------------------------------------------------ I2C target */

static void sda_drive(Esp32I2SDac *s, bool low)
{
    s->sda_low = low;
    esp32_gpio_set_external_drive(s->gpio, s->pad[PIN_SDA], s->driver_slot,
                                  low ? ESP32_PAD_LOW : ESP32_PAD_Z);
}

static void i2c_load(Esp32I2SDac *s)
{
    s->shift = reg_read(s, s->pointer++);
    s->bits = 0;
    s->phase = I2C_READ;
    sda_drive(s, !(s->shift & 0x80));
}

static void i2c_rise(Esp32I2SDac *s)
{
    bool sda = s->level[PIN_SDA] == ESP32_PAD_HIGH;

    switch (s->phase) {
    case I2C_ADDRESS:
    case I2C_WRITE:
        s->shift = (s->shift << 1) | sda;
        if (++s->bits == 8) {
            if (s->phase == I2C_ADDRESS) {
                s->ack = (s->shift >> 1) == s->address;
                s->next = s->shift & 1 ? I2C_READ : I2C_WRITE;
                s->expect_pointer = !(s->shift & 1);
            } else {
                s->ack = true;
                if (s->expect_pointer) {
                    s->pointer = s->shift;
                    s->expect_pointer = false;
                } else {
                    reg_write(s, s->pointer++, s->shift);
                }
                s->next = I2C_WRITE;
            }
            if (!s->ack) {
                s->next = I2C_IGNORE;
            }
            s->phase = I2C_ACK_SETUP;
        }
        break;
    case I2C_ACK_CLOCK:
        s->phase = I2C_ACK_END;
        break;
    case I2C_READ:
        if (++s->bits == 8) {
            s->phase = I2C_READ_ACK_SETUP;
        }
        break;
    case I2C_READ_ACK:
        s->phase = sda ? I2C_IGNORE : I2C_READ_ACK_END;
        break;
    default:
        break;
    }
}

static void i2c_fall(Esp32I2SDac *s)
{
    switch (s->phase) {
    case I2C_ACK_SETUP:
        s->phase = I2C_ACK_CLOCK;
        sda_drive(s, s->ack);
        break;
    case I2C_ACK_END:
        s->phase = s->next;
        s->bits = s->shift = 0;
        sda_drive(s, false);
        if (s->phase == I2C_READ) {
            i2c_load(s);
        }
        break;
    case I2C_READ:
        sda_drive(s, !(s->shift & (0x80 >> s->bits)));
        break;
    case I2C_READ_ACK_SETUP:
        s->phase = I2C_READ_ACK;
        sda_drive(s, false);
        break;
    case I2C_READ_ACK_END:
        i2c_load(s);
        break;
    default:
        break;
    }
}

static void i2c_event(void *opaque, int n, int value)
{
    Esp32I2SDac *s = opaque;
    bool level, previous;

    /* A floating or contended bus cannot be interpreted as a valid bit. */
    if (!level_binary(value)) {
        s->level[n] = value;
        s->phase = I2C_IGNORE;
        return;
    }
    level = value == ESP32_PAD_HIGH;
    previous = s->level[n] == ESP32_PAD_HIGH;
    s->level[n] = value;
    if (n == PIN_SCL) {
        if (level != previous) {
            if (level) {
                i2c_rise(s);
            } else {
                i2c_fall(s);
            }
        }
    } else if (level != previous && s->level[PIN_SCL] == ESP32_PAD_HIGH) {
        /* SDA edge while SCL is high: START (fall) or STOP (rise) */
        s->phase = level ? I2C_IDLE : I2C_ADDRESS;
        s->bits = s->shift = 0;
        sda_drive(s, false);
    }
}

/* ---------------------------------------------------------- QOM plumbing */

static void dac_reset(DeviceState *dev)
{
    Esp32I2SDac *s = ESP32_I2S_DAC(dev);

    record_stop(s);
    s->channels = 2;
    s->depth = 16;
    s->rate_id = 3;
    s->clock_error = s->config_error = false;
    s->frames = s->frames_latched = 0;
    s->phase = I2C_IDLE;
    s->bits = s->shift = s->pointer = s->next = 0;
    s->ack = s->expect_pointer = false;
    s->stream_edges = 0;
    s->bclk_seen = false;
    lose_sync(s, false, NULL);
    if (s->gpio) {
        sda_drive(s, false);
    }
}

static void dac_realize(DeviceState *dev, Error **errp)
{
    Esp32I2SDac *s = ESP32_I2S_DAC(dev);

    if (!s->gpio) {
        error_setg(errp, "esp32-i2s-dac needs the 'gpio' link");
        return;
    }
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        if (s->pad[i] >= ESP32_GPIO_PADS) {
            error_setg(errp, "esp32-i2s-dac: pad %u is out of range",
                       s->pad[i]);
            return;
        }
        for (unsigned j = 0; j < i; j++) {
            if (s->pad[i] == s->pad[j]) {
                error_setg(errp, "esp32-i2s-dac: pad %u used for two signals",
                           s->pad[i]);
                return;
            }
        }
    }
    if (s->driver_slot >= ESP32_GPIO_EXT_DRIVERS || s->address > 0x7f) {
        error_setg(errp, "esp32-i2s-dac needs a valid driver-slot and a "
                   "7-bit address");
        return;
    }
    if (s->max_bytes < WAV_HEADER_SIZE || s->max_bytes > UINT32_MAX - 64) {
        error_setg(errp, "esp32-i2s-dac max-bytes must be 44..4294967231");
        return;
    }
    s->fd = -1;
    s->stage = g_malloc(STAGE_SIZE);
    s->flush_timer = timer_new_ms(QEMU_CLOCK_REALTIME, flush_timer_cb, s);
    s->exit_notifier.notify = exit_notify;
    qemu_add_exit_notifier(&s->exit_notifier);
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        s->level[i] = ESP32_PAD_Z;
        s->sink[i] = qemu_allocate_irq(i == PIN_SDA || i == PIN_SCL ?
                                       i2c_event : pad_event, s, i);
    }
    dac_reset(dev);
    /* Registering listeners replays the present level of each pad. */
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        esp32_gpio_add_pad_listener(s->gpio, s->pad[i], s->sink[i]);
    }
    /* Leave a valid, empty WAV behind when nothing is recorded. */
    if (s->wav) {
        wav_open(s);
        wav_close(s);
    }
}

static void dac_unrealize(DeviceState *dev)
{
    Esp32I2SDac *s = ESP32_I2S_DAC(dev);

    record_stop(s);
    qemu_remove_exit_notifier(&s->exit_notifier);
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        esp32_gpio_remove_pad_listener(s->gpio, s->pad[i], s->sink[i]);
        qemu_free_irq(s->sink[i]);
    }
    sda_drive(s, false);
    timer_free(s->flush_timer);
    g_free(s->stage);
}

/* Host file state cannot be migrated or snapshotted. */
static const VMStateDescription dac_vmstate = {
    .name = TYPE_ESP32_I2S_DAC,
    .version_id = 1,
    .minimum_version_id = 1,
    .unmigratable = 1,
};

static const Property dac_properties[] = {
    DEFINE_PROP_LINK("gpio", Esp32I2SDac, gpio, TYPE_ESP32_GPIO,
                     Esp32GpioState *),
    DEFINE_PROP_STRING("wav", Esp32I2SDac, wav),
    DEFINE_PROP_UINT32("sda", Esp32I2SDac, pad[PIN_SDA], 21),
    DEFINE_PROP_UINT32("scl", Esp32I2SDac, pad[PIN_SCL], 22),
    DEFINE_PROP_UINT32("bclk", Esp32I2SDac, pad[PIN_BCLK], 18),
    DEFINE_PROP_UINT32("ws", Esp32I2SDac, pad[PIN_WS], 19),
    DEFINE_PROP_UINT32("data", Esp32I2SDac, pad[PIN_DATA], 23),
    DEFINE_PROP_UINT32("driver-slot", Esp32I2SDac, driver_slot, 2),
    DEFINE_PROP_UINT32("address", Esp32I2SDac, address, 0x4c),
    DEFINE_PROP_UINT64("max-bytes", Esp32I2SDac, max_bytes, 1ull << 30),
    DEFINE_PROP_END_OF_LIST(),
};

static void dac_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = dac_realize;
    dc->unrealize = dac_unrealize;
    device_class_set_legacy_reset(dc, dac_reset);
    dc->vmsd = &dac_vmstate;
    dc->desc = "I2C-controlled I2S slave DAC that records a WAV file from "
               "resolved ESP32 pads";
    device_class_set_props(dc, dac_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo dac_type = {
    .name = TYPE_ESP32_I2S_DAC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(Esp32I2SDac),
    .class_init = dac_class_init,
};

static void dac_register(void)
{
    type_register_static(&dac_type);
}

type_init(dac_register)
