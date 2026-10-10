/*
 * Tests for the esp32-i2s-dac virtual DAC / WAV recorder.
 *
 * The DAC only observes resolved GPIO pads. Most cases drive the pads with
 * the GPIO model's external "pad-drive" inputs (driver 0), so every clock edge
 * has an exact virtual time. One case uses the real ESP32 I2S0 transmitter.
 * The I2C control bus is bit-banged on pads 21/22 with the IO MUX pull-ups
 * enabled, as an open-drain bus with pull-ups.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bswap.h"

#define GPIO 0x3ff44000
#define MUX 0x3ff49000
#define DPORT 0x3ff00000
#define I2S0 0x3ff4f000
#define DESC 0x3ffb0000
#define DATA_BUF 0x3ffb0100
#define RX_DESC (DESC + 0x20)
#define RX_DATA (DATA_BUF + 0x2000)

#define PAD_SDA 21
#define PAD_SCL 22
#define PAD_BCLK 18
#define PAD_WS 19
#define PAD_DATA 23

#define DAC_ADDR 0x4c
#define REG_CHANNELS 0
#define REG_DEPTH 1
#define REG_RATE 2
#define REG_RECORD 3
#define REG_STATUS 4
#define REG_FRAMES 5

#define ST_CLOCK 1
#define ST_CONFIG 2
#define ST_RECORDING 4

/* ------------------------------------------------------------- pad access */

static void ext(QTestState *q, unsigned pad, unsigned level)
{
    qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", pad, level);
}

static void tick(QTestState *q)
{
    qtest_clock_step(q, 1000);
}

/* ------------------------------------------------------------ I2C bit-bang */

static void sda(QTestState *q, bool high)
{
    ext(q, PAD_SDA, high ? 2 : 0);
}

static void scl(QTestState *q, bool high)
{
    ext(q, PAD_SCL, high ? 2 : 0);
}

static void i2c_start(QTestState *q)
{
    sda(q, true);
    scl(q, true);
    tick(q);
    sda(q, false);
    tick(q);
    scl(q, false);
    tick(q);
}

static void i2c_restart(QTestState *q)
{
    sda(q, true);
    tick(q);
    i2c_start(q);
}

static void i2c_stop(QTestState *q)
{
    sda(q, false);
    tick(q);
    scl(q, true);
    tick(q);
    sda(q, true);
    tick(q);
}

static bool sda_in(QTestState *q)
{
    return (qtest_readl(q, GPIO + 0x3c) >> PAD_SDA) & 1;
}

/* Returns true when the target acknowledges. */
static bool i2c_put(QTestState *q, uint8_t byte)
{
    bool ack;

    for (int bit = 7; bit >= 0; bit--) {
        sda(q, (byte >> bit) & 1);
        tick(q);
        scl(q, true);
        tick(q);
        scl(q, false);
        tick(q);
    }
    sda(q, true);
    tick(q);
    scl(q, true);
    tick(q);
    ack = !sda_in(q);
    scl(q, false);
    tick(q);
    return ack;
}

static uint8_t i2c_get(QTestState *q, bool ack)
{
    uint8_t value = 0;

    sda(q, true);
    for (int bit = 0; bit < 8; bit++) {
        tick(q);
        scl(q, true);
        tick(q);
        value = (value << 1) | sda_in(q);
        scl(q, false);
        tick(q);
    }
    sda(q, !ack);
    tick(q);
    scl(q, true);
    tick(q);
    scl(q, false);
    tick(q);
    sda(q, true);
    tick(q);
    return value;
}

/* Pointer-first write: reg then values. Returns true when all bytes ACK. */
static bool dac_write(QTestState *q, uint8_t reg, const uint8_t *values,
                      unsigned count)
{
    bool ok;

    i2c_start(q);
    ok = i2c_put(q, DAC_ADDR << 1);
    ok = ok && i2c_put(q, reg);
    for (unsigned i = 0; ok && i < count; i++) {
        ok = i2c_put(q, values[i]);
    }
    i2c_stop(q);
    return ok;
}

static void dac_set(QTestState *q, uint8_t reg, uint8_t value)
{
    g_assert_true(dac_write(q, reg, &value, 1));
}

static void dac_read(QTestState *q, uint8_t reg, uint8_t *out, unsigned count)
{
    i2c_start(q);
    g_assert_true(i2c_put(q, DAC_ADDR << 1));
    g_assert_true(i2c_put(q, reg));
    i2c_restart(q);
    g_assert_true(i2c_put(q, (DAC_ADDR << 1) | 1));
    for (unsigned i = 0; i < count; i++) {
        out[i] = i2c_get(q, i + 1 < count);
    }
    i2c_stop(q);
}

static uint8_t dac_get(QTestState *q, uint8_t reg)
{
    uint8_t value;

    dac_read(q, reg, &value, 1);
    return value;
}

static uint32_t dac_frames(QTestState *q)
{
    uint8_t b[4];

    dac_read(q, REG_FRAMES, b, 4);
    return ldl_le_p(b);
}

/* ------------------------------------------------------------------ setup */

typedef struct Dut {
    QTestState *q;
    char *wav;
} Dut;

static Dut dut_start(const char *dac_args, const char *machine_args)
{
    Dut d;
    int fd = g_file_open_tmp("esp32-dac-XXXXXX.wav", &d.wav, NULL);
    g_autofree char *args;

    g_assert_cmpint(fd, >=, 0);
    close(fd);
    args = g_strdup_printf("-machine esp32 -display none -serial none "
                           "-nic none %s -device esp32-i2s-dac,"
                           "gpio=/machine/soc/gpio,wav=%s%s%s",
                           machine_args ? machine_args : "", d.wav,
                           dac_args ? "," : "", dac_args ? dac_args : "");
    d.q = qtest_init(args);
    /* I2C pads: GPIO function, input enable, pull-up. */
    qtest_writel(d.q, MUX + 0x7c, (1 << 9) | (1 << 8));
    qtest_writel(d.q, MUX + 0x80, (1 << 9) | (1 << 8));
    return d;
}

static void dut_end(Dut *d)
{
    qtest_quit(d->q);
    unlink(d->wav);
    g_free(d->wav);
}

static void configure(QTestState *q, uint8_t channels, uint8_t depth,
                      uint8_t rate_id)
{
    const uint8_t regs[] = {channels, depth, rate_id};

    g_assert_true(dac_write(q, REG_CHANNELS, regs, 3));
}

/* ------------------------------------------------------------ WAV helpers */

typedef struct Wav {
    uint8_t *data;
    gsize size;
    uint32_t channels, rate, depth, data_bytes;
    const uint8_t *pcm;
} Wav;

static Wav wav_load(const char *path)
{
    Wav w = {};
    gsize length;
    gchar *contents;
    uint32_t riff;

    g_assert_true(g_file_get_contents(path, &contents, &length, NULL));
    w.data = (uint8_t *)contents;
    w.size = length;
    g_assert_cmpuint(length, >=, 44);
    g_assert_cmpint(memcmp(w.data, "RIFF", 4), ==, 0);
    g_assert_cmpint(memcmp(w.data + 8, "WAVEfmt ", 8), ==, 0);
    g_assert_cmpint(memcmp(w.data + 36, "data", 4), ==, 0);
    g_assert_cmpuint(ldl_le_p(w.data + 16), ==, 16);
    g_assert_cmpuint(lduw_le_p(w.data + 20), ==, 1);
    w.channels = lduw_le_p(w.data + 22);
    w.rate = ldl_le_p(w.data + 24);
    w.depth = lduw_le_p(w.data + 34);
    g_assert_cmpuint(ldl_le_p(w.data + 28), ==,
                     w.rate * w.channels * w.depth / 8);
    g_assert_cmpuint(lduw_le_p(w.data + 32), ==, w.channels * w.depth / 8);
    w.data_bytes = ldl_le_p(w.data + 40);
    riff = ldl_le_p(w.data + 4);
    g_assert_cmpuint(riff, ==, 36 + w.data_bytes + (w.data_bytes & 1));
    g_assert_cmpuint(w.size, ==, 8 + (uint64_t)riff);
    w.pcm = w.data + 44;
    return w;
}

static uint32_t wav_sample(const Wav *w, unsigned frame, unsigned channel)
{
    unsigned bytes = w->depth / 8;
    const uint8_t *p = w->pcm + (frame * w->channels + channel) * bytes;
    uint32_t value = 0;

    for (unsigned i = 0; i < bytes; i++) {
        value |= (uint32_t)p[i] << (8 * i);
    }
    return value;
}

static void wav_free(Wav *w)
{
    g_free(w->data);
}

static uint32_t mask_depth(uint32_t value, unsigned depth)
{
    return depth == 32 ? value : value & ((1u << depth) - 1);
}

/* Test pattern; includes sign extremes. */
static uint32_t pattern(unsigned i, unsigned channel, unsigned depth)
{
    static const uint32_t special[] = {0x80000000, 0x7fffffff, 0xffffffff, 1};
    uint32_t value;

    if (i < 4) {
        value = special[(i + channel) & 3];
    } else {
        value = (0x13579bdfu * (i + 1)) ^ (channel ? 0xa5c3e17fu : 0x1b2d4f69u);
    }
    return mask_depth(value, depth);
}

/* ------------------------------------------------------ pad-driven stream */

typedef struct Synth {
    QTestState *q;
    unsigned depth;
    int64_t bclk_half; /* ns */
    unsigned data_mode; /* 0 normal, 1 DATA released */
    int64_t now;
} Synth;

static void advance(Synth *s, int64_t ns)
{
    int64_t end;

    s->now = qtest_clock_step(s->q, 0);
    end = s->now + ns;
    qtest_clock_step(s->q, end - s->now);
    s->now = end;
}

static Synth synth_init(QTestState *q, unsigned depth, int64_t bclk_half)
{
    Synth s = {.q = q, .depth = depth, .bclk_half = bclk_half};

    ext(q, PAD_BCLK, 0);
    ext(q, PAD_WS, 0);
    ext(q, PAD_DATA, 0);
    return s;
}

/* One Philips I2S frame: WS and data change on BCLK falling edge. */
static void synth_frame(Synth *s, uint32_t left, uint32_t right)
{
    for (unsigned b = 0; b < 2 * s->depth; b++) {
        uint32_t word = b < s->depth ? left : right;
        unsigned bit = (word >> (s->depth - 1 - (b % s->depth))) & 1;

        ext(s->q, PAD_WS, ((b + 1) / s->depth) & 1);
        ext(s->q, PAD_DATA, s->data_mode ? 2 : bit);
        advance(s, s->bclk_half);
        ext(s->q, PAD_BCLK, 1);
        advance(s, s->bclk_half);
        ext(s->q, PAD_BCLK, 0);
    }
}

static void synth_frames(Synth *s, unsigned first, unsigned count)
{
    for (unsigned i = first; i < first + count; i++) {
        synth_frame(s, pattern(i, 0, s->depth), pattern(i, 1, s->depth));
    }
}

/* 16 kHz-class presets: frame time = 62.5 us +-0.5% */
#define BH16 976   /* BCLK half period, 32 BCLK per frame */
#define BH24 648   /* 48 BCLK per frame */
#define BH32 488   /* 64 BCLK per frame */

/* ------------------------------------------------------------------ cases */

static void i2c_registers(void)
{
    Dut d = dut_start(NULL, NULL);
    uint8_t v[4];
    const uint8_t cfg[] = {1, 24, 2};

    dac_read(d.q, 0, v, 4);
    g_assert_cmpuint(v[0], ==, 2);
    g_assert_cmpuint(v[1], ==, 16);
    g_assert_cmpuint(v[2], ==, 3);
    g_assert_cmpuint(v[3], ==, 0);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, 0);
    g_assert_cmpuint(dac_frames(d.q), ==, 0);

    /* Auto-increment write and readback */
    g_assert_true(dac_write(d.q, REG_CHANNELS, cfg, 3));
    dac_read(d.q, 0, v, 3);
    g_assert_cmpuint(v[0], ==, 1);
    g_assert_cmpuint(v[1], ==, 24);
    g_assert_cmpuint(v[2], ==, 2);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, 0);

    /* Illegal values are refused, flagged, and leave the register alone. */
    dac_set(d.q, REG_DEPTH, 17);
    g_assert_cmpuint(dac_get(d.q, REG_DEPTH), ==, 24);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_CONFIG);
    dac_set(d.q, REG_RATE, 4);
    dac_set(d.q, REG_CHANNELS, 3);
    dac_set(d.q, REG_RECORD, 2);
    g_assert_cmpuint(dac_get(d.q, REG_RATE), ==, 2);
    g_assert_cmpuint(dac_get(d.q, REG_CHANNELS), ==, 1);
    g_assert_cmpuint(dac_get(d.q, REG_RECORD), ==, 0);
    /* Status is read only and reading it does not clear it. */
    dac_set(d.q, REG_STATUS, 0);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_CONFIG);

    /* Record start clears errors; format is locked while recording. */
    dac_set(d.q, REG_RECORD, 1);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_RECORDING);
    g_assert_cmpuint(dac_get(d.q, REG_RECORD), ==, 1);
    dac_set(d.q, REG_DEPTH, 16);
    g_assert_cmpuint(dac_get(d.q, REG_DEPTH), ==, 24);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_RECORDING | ST_CONFIG);
    dac_set(d.q, REG_DEPTH, 24); /* same value is harmless */
    dac_set(d.q, REG_RECORD, 0);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_CONFIG);

    /* Other addresses are not acknowledged. */
    i2c_start(d.q);
    g_assert_false(i2c_put(d.q, 0x4d << 1));
    i2c_stop(d.q);
    /* Unknown register read returns zero and flags an error. */
    g_assert_cmpuint(dac_get(d.q, 0x20), ==, 0);
    dut_end(&d);
}

typedef struct Format {
    unsigned channels, depth;
    int64_t bclk_half;
    unsigned rate_id;
} Format;

static const Format formats[] = {
    {2, 16, BH16, 1},
    {1, 16, BH16, 1},
    {2, 24, BH24, 1},
    {1, 24, BH24, 1},
    {2, 32, BH32, 1},
};

static void decode_format(gconstpointer data)
{
    const Format *f = data;
    Dut d = dut_start(NULL, NULL);
    enum { N = 40 };
    Synth s = synth_init(d.q, f->depth, f->bclk_half);
    Wav w;

    configure(d.q, f->channels, f->depth, f->rate_id);
    dac_set(d.q, REG_RECORD, 1);
    synth_frames(&s, 0, N);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_RECORDING);
    dac_set(d.q, REG_RECORD, 0);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, 0);
    g_assert_cmpuint(dac_frames(d.q), ==, N);

    w = wav_load(d.wav);
    g_assert_cmpuint(w.channels, ==, f->channels);
    g_assert_cmpuint(w.depth, ==, f->depth);
    {
        static const uint32_t rates[] = {8000, 16000, 32000, 48000};
        g_assert_cmpuint(w.rate, ==, rates[f->rate_id]);
    }
    g_assert_cmpuint(w.data_bytes, ==, N * f->channels * f->depth / 8);
    for (unsigned i = 0; i < N; i++) {
        g_assert_cmphex(wav_sample(&w, i, 0), ==, pattern(i, 0, f->depth));
        if (f->channels == 2) {
            g_assert_cmphex(wav_sample(&w, i, 1), ==, pattern(i, 1, f->depth));
        }
    }
    wav_free(&w);
    dut_end(&d);
}

/* Run a stream that violates the programmed clocks. No frame may be stored. */
static void bad_clock(gconstpointer data)
{
    enum { Rate, BclkRatio, BclkRatio24 };
    unsigned kind = GPOINTER_TO_UINT(data);
    Dut d = dut_start(NULL, NULL);
    uint8_t depth = 16;
    Synth s = synth_init(d.q, 16, BH16);
    Wav w;

    configure(d.q, 2, depth, kind == Rate ? 3 : 1); /* 48 kHz programmed */
    switch (kind) {
    case BclkRatio:
        /* 24-bit slots sent while 16 bit is programmed */
        s.depth = 24;
        s.bclk_half = BH24;
        break;
    case BclkRatio24:
        /* Slots programmed 16 bit at 2x the BCLK speed: 64 BCLK per frame */
        s.depth = 32;
        s.bclk_half = BH32;
        break;
    default:
        break;
    }
    dac_set(d.q, REG_RECORD, 1);
    synth_frames(&s, 0, 12);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_RECORDING | ST_CLOCK);
    dac_set(d.q, REG_RECORD, 0);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_CLOCK);
    g_assert_cmpuint(dac_frames(d.q), ==, 0);
    w = wav_load(d.wav);
    g_assert_cmpuint(w.data_bytes, ==, 0);
    g_assert_cmpuint(w.size, ==, 44);
    wav_free(&w);
    dut_end(&d);
}

/* A single slow frame is dropped, flagged, and the stream recovers. */
static void bad_frame_recovery(void)
{
    Dut d = dut_start(NULL, NULL);
    Synth s = synth_init(d.q, 16, BH16);
    Wav w;
    unsigned index = 0;

    configure(d.q, 2, 16, 1);
    dac_set(d.q, REG_RECORD, 1);
    synth_frames(&s, 0, 10);
    s.bclk_half = 1170;
    synth_frames(&s, 10, 1);
    s.bclk_half = BH16;
    synth_frames(&s, 11, 9);
    dac_set(d.q, REG_RECORD, 0);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_CLOCK);
    g_assert_cmpuint(dac_frames(d.q), ==, 19);
    w = wav_load(d.wav);
    g_assert_cmpuint(w.data_bytes, ==, 19 * 4);
    for (unsigned i = 0; i < 20; i++) {
        if (i == 10) {
            continue;
        }
        g_assert_cmphex(wav_sample(&w, index, 0), ==, pattern(i, 0, 16));
        g_assert_cmphex(wav_sample(&w, index, 1), ==, pattern(i, 1, 16));
        index++;
    }
    wav_free(&w);
    dut_end(&d);
}

/* A floating data line is never recorded as zero samples. */
static void data_floating(void)
{
    Dut d = dut_start(NULL, NULL);
    Synth s = synth_init(d.q, 16, BH16);
    Wav w;

    configure(d.q, 2, 16, 1);
    dac_set(d.q, REG_RECORD, 1);
    s.data_mode = 1;
    synth_frames(&s, 0, 8);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_RECORDING | ST_CONFIG);
    dac_set(d.q, REG_RECORD, 0);
    g_assert_cmpuint(dac_frames(d.q), ==, 0);
    w = wav_load(d.wav);
    g_assert_cmpuint(w.data_bytes, ==, 0);
    wav_free(&w);
    dut_end(&d);
}

/* Samples are stored only while record is 1. Recording that starts in the
 * middle of a stream starts at a frame boundary. */
static void record_idle(void)
{
    Dut d = dut_start(NULL, NULL);
    Synth s = synth_init(d.q, 16, BH16);
    Wav w;
    uint32_t frames;

    configure(d.q, 2, 16, 1);
    /* Idle: valid, empty WAV exists; stream is ignored. */
    w = wav_load(d.wav);
    g_assert_cmpuint(w.data_bytes, ==, 0);
    g_assert_cmpuint(w.size, ==, 44);
    wav_free(&w);
    synth_frames(&s, 0, 10);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, 0);
    g_assert_cmpuint(dac_frames(d.q), ==, 0);
    w = wav_load(d.wav);
    g_assert_cmpuint(w.size, ==, 44);
    wav_free(&w);

    dac_set(d.q, REG_RECORD, 1);
    synth_frames(&s, 10, 20);
    dac_set(d.q, REG_RECORD, 0);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, 0);
    frames = dac_frames(d.q);
    g_assert_cmpuint(frames, >=, 17);
    g_assert_cmpuint(frames, <=, 20);
    w = wav_load(d.wav);
    g_assert_cmpuint(w.data_bytes, ==, frames * 4);
    /* Frames are the tail of the sequence 10..29, nothing earlier. */
    for (unsigned i = 0; i < frames; i++) {
        unsigned source = 30 - frames + i;
        g_assert_cmphex(wav_sample(&w, i, 0), ==, pattern(source, 0, 16));
        g_assert_cmphex(wav_sample(&w, i, 1), ==, pattern(source, 1, 16));
    }
    wav_free(&w);

    /* After record=0 the stream no longer reaches the file. */
    synth_frames(&s, 30, 10);
    g_assert_cmpuint(dac_frames(d.q), ==, frames);
    w = wav_load(d.wav);
    g_assert_cmpuint(w.data_bytes, ==, frames * 4);
    wav_free(&w);
    dut_end(&d);
}

/* Errors clear on the next record start; a new recording replaces the file. */
static void restart_recording(void)
{
    Dut d = dut_start(NULL, NULL);
    Synth s = synth_init(d.q, 16, BH16);
    Synth bad = synth_init(d.q, 16, BH16);
    bad.bclk_half = 1170; /* slow BCLK: every frame misses its rate */
    Wav w;

    configure(d.q, 2, 16, 1);
    dac_set(d.q, REG_RECORD, 1);
    synth_frames(&bad, 0, 5);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_RECORDING | ST_CLOCK);
    dac_set(d.q, REG_RECORD, 0);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_CLOCK); /* sticky */
    dac_set(d.q, REG_RECORD, 1);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_RECORDING);
    synth_frames(&s, 0, 12);
    dac_set(d.q, REG_RECORD, 0);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, 0);
    g_assert_cmpuint(dac_frames(d.q), >=, 9);
    w = wav_load(d.wav);
    g_assert_cmpuint(w.data_bytes, ==, dac_frames(d.q) * 4);
    wav_free(&w);

    /* Second recording in another format replaces the file. */
    configure(d.q, 1, 24, 1);
    s = synth_init(d.q, 24, BH24);
    dac_set(d.q, REG_RECORD, 1);
    synth_frames(&s, 0, 8);
    dac_set(d.q, REG_RECORD, 0);
    w = wav_load(d.wav);
    g_assert_cmpuint(w.depth, ==, 24);
    g_assert_cmpuint(w.channels, ==, 1);
    g_assert_cmpuint(w.data_bytes, ==, 8 * 3);
    g_assert_cmphex(wav_sample(&w, 7, 0), ==, pattern(7, 0, 24));
    wav_free(&w);
    dut_end(&d);
}

/* The recorder is only the destination: the same 24-bit mono stream with an
 * odd data size ends with a padded chunk and a consistent header. */
static void odd_chunk(void)
{
    Dut d = dut_start(NULL, NULL);
    Synth s = synth_init(d.q, 24, BH24);
    Wav w;

    configure(d.q, 1, 24, 1);
    dac_set(d.q, REG_RECORD, 1);
    synth_frames(&s, 0, 5);
    dac_set(d.q, REG_RECORD, 0);
    w = wav_load(d.wav);
    g_assert_cmpuint(w.data_bytes, ==, 15);
    wav_free(&w);
    dut_end(&d);
}

/* Recording stops by itself at the size limit and reports the error. */
static void size_limit(void)
{
    Dut d = dut_start("max-bytes=84", NULL); /* 44 header + 10 stereo frames */
    Synth s = synth_init(d.q, 16, BH16);
    Wav w;

    configure(d.q, 2, 16, 1);
    dac_set(d.q, REG_RECORD, 1);
    synth_frames(&s, 0, 20);
    g_assert_cmpuint(dac_get(d.q, REG_STATUS), ==, ST_CONFIG);
    g_assert_cmpuint(dac_frames(d.q), ==, 10);
    w = wav_load(d.wav);
    g_assert_cmpuint(w.data_bytes, ==, 40);
    wav_free(&w);
    dut_end(&d);
}

/* ------------------------------------------------- real ESP32 I2S0 master */

static void i2s_write(QTestState *q, unsigned offset, uint32_t value)
{
    qtest_writel(q, I2S0 + offset, value);
}

/*
 * ESP32 I2S0 TX master -> GPIO matrix -> pads 18/19/23 -> DAC. 160 MHz / 39
 * = 4.10 MHz I2S clock, BCLK = clock / 8, 16 bit stereo, so
 * Fs = 16.03 kHz (0.16% from 16 kHz). MCLK is not routed: the DAC measures
 * the frame rate from WS edges and the BCLK count per frame.
 */
static void real_i2s_run(bool trace)
{
    g_autofree char *trace_path = NULL;
    g_autofree char *machine_args = NULL;
    if (trace) {
        int fd = g_file_open_tmp("esp32-i2s-trace-XXXXXX.vcd", &trace_path,
                                 NULL);
        g_assert_cmpint(fd, >=, 0);
        close(fd);
        unlink(trace_path);
        machine_args = g_strdup_printf(
            "-global driver=esp32.gpio,property=pin-trace,value=%s",
            trace_path);
    }
    Dut d = dut_start(NULL, machine_args);
    QTestState *q = d.q;
    enum { N = 24, M = 4 };
    Wav w;

    qtest_writel(q, DPORT + 0xc0, (1 << 4) | (1 << 21));
    qtest_writel(q, MUX + 0x70, (2 << 12) | (1 << 9));
    qtest_writel(q, MUX + 0x74, (2 << 12) | (1 << 9));
    qtest_writel(q, MUX + 0x8c, (2 << 12) | (1 << 9));
    qtest_writel(q, GPIO + 0x530 + PAD_BCLK * 4, 23);
    qtest_writel(q, GPIO + 0x530 + PAD_WS * 4, 25);
    qtest_writel(q, GPIO + 0x530 + PAD_DATA * 4, 163);
    i2s_write(q, 0xac, (1 << 20) | 39);
    i2s_write(q, 0xb0, 8 | (8 << 6) | (16 << 12) | (16 << 18));
    i2s_write(q, 0x20, 32 | (32 << 6) | (1 << 19) | (1 << 20));
    i2s_write(q, 0x08, 0);

    configure(q, 2, 16, 1);
    dac_set(q, REG_RECORD, 1);

    /* Burst 1: DMA. Philips format, stop on underrun. */
    qtest_writel(q, DESC, (1u << 31) | (1 << 30) | (N * 4 << 12) | (N * 4));
    qtest_writel(q, DESC + 4, DATA_BUF);
    qtest_writel(q, DESC + 8, 0);
    for (unsigned i = 0; i < N; i++) {
        qtest_writel(q, DATA_BUF + 4 * i,
                     (pattern(i, 0, 16) << 16) | pattern(i, 1, 16));
    }
    i2s_write(q, 0xa0, 0x189); /* TX_STOP_EN */
    i2s_write(q, 0x60, (1 << 12) | (1 << 8) | (1 << 6));
    i2s_write(q, 0x20, qtest_readl(q, I2S0 + 0x20) | (1 << 12));
    i2s_write(q, 0x30, (DESC & 0xfffff) | (1 << 29));
    i2s_write(q, 0x08, (1 << 4) | (1 << 10));
    qtest_clock_step(q, 70000 * (N + 3));

    /* Burst 2: the clock stalled after burst 1; restart through the FIFO. */
    for (unsigned i = N; i < N + M; i++) {
        i2s_write(q, 0x00, (pattern(i, 0, 16) << 16) | pattern(i, 1, 16));
    }
    qtest_clock_step(q, 70000 * (M + 3));

    g_assert_cmpuint(dac_get(q, REG_STATUS), ==, ST_RECORDING);
    dac_set(q, REG_RECORD, 0);
    g_assert_cmpuint(dac_get(q, REG_STATUS), ==, 0);
    g_assert_cmpuint(dac_frames(q), ==, N + M);
    w = wav_load(d.wav);
    g_assert_cmpuint(w.data_bytes, ==, (N + M) * 4);
    for (unsigned i = 0; i < N + M; i++) {
        g_assert_cmphex(wav_sample(&w, i, 0), ==, pattern(i, 0, 16));
        g_assert_cmphex(wav_sample(&w, i, 1), ==, pattern(i, 1, 16));
    }
    wav_free(&w);
    dut_end(&d);

    if (trace) {
        gchar *vcd;
        gsize length;
        unsigned bclk_transitions = 0;
        g_assert_true(g_file_get_contents(trace_path, &vcd, &length, NULL));
        g_assert_cmpuint(length, >, 1024);
        g_assert_nonnull(strstr(vcd, "$timescale"));
        g_assert_nonnull(strstr(vcd, "#"));
        for (char *line = vcd; line && *line;) {
            char *end = strchr(line, '\n');
            if (end && end - line >= 4 && line[1] == 'P' &&
                line[2] == '1' && line[3] == '8' &&
                (line[0] == '0' || line[0] == '1')) {
                bclk_transitions++;
            }
            line = end ? end + 1 : NULL;
        }
        /*
         * 28 stereo frames carry 1,792 BCLK transitions; allow startup and
         * stop phase details while still proving the complete traced stream.
         */
        g_assert_cmpuint(bclk_transitions, >=, 1700);
        g_free(vcd);
        unlink(trace_path);
    }
}

static void real_i2s(void)
{
    real_i2s_run(false);
}

static void real_i2s_trace(void)
{
    /*
     * With full pad tracing enabled, the same physical DAC switches to the
     * timestamped edge path and must capture the identical DMA sample stream.
     */
    real_i2s_run(true);
}

static void real_i2s_wrong_physical_route(void)
{
    Dut d = dut_start(NULL, NULL);
    QTestState *q = d.q;

    qtest_writel(q, DPORT + 0xc0, (1 << 4) | (1 << 21));
    qtest_writel(q, MUX + 0x44, (2 << 12) | (1 << 9)); /* GPIO5 */
    qtest_writel(q, MUX + 0x74, (2 << 12) | (1 << 9)); /* GPIO19 */
    qtest_writel(q, MUX + 0x8c, (2 << 12) | (1 << 9)); /* GPIO23 */
    qtest_writel(q, GPIO + 0x530 + 5 * 4, 23); /* BCLK is misrouted. */
    qtest_writel(q, GPIO + 0x530 + PAD_WS * 4, 25);
    qtest_writel(q, GPIO + 0x530 + PAD_DATA * 4, 163);
    i2s_write(q, 0xac, (1 << 20) | 39);
    i2s_write(q, 0xb0, 8 | (8 << 6) | (16 << 12) | (16 << 18));
    i2s_write(q, 0x20, 32 | (32 << 6) | (1 << 19) | (1 << 20));
    configure(q, 2, 16, 1);
    dac_set(q, REG_RECORD, 1);
    i2s_write(q, 0x00, 0x12345678);
    i2s_write(q, 0x08, (1 << 4) | (1 << 10));
    qtest_clock_step(q, 2000000);
    g_assert_cmpuint(dac_frames(q), ==, 0);
    dac_set(q, REG_RECORD, 0);
    dut_end(&d);
}

/*
 * One virtual second of 48 kHz, 32-bit stereo TX through the physical GPIO
 * matrix into both TAS observers and the recording DAC.  The DMA burst is
 * intentionally finite; after its EOF the real I2S underrun rule replays the
 * last frame, giving the WAV a simple golden tail.
 */
static void analytic_audio_second(void)
{
    g_autofree char *devices = g_strdup(
        "-device esp32-tas5828m,gpio=/machine/soc/gpio,address=0x61,"
        "driver-slot=2,bclk=18,ws=19,data=23 "
        "-device esp32-tas5830,gpio=/machine/soc/gpio,address=0x62,"
        "driver-slot=3");
    Dut d = dut_start(NULL, devices);
    QTestState *q = d.q;
    const unsigned burst_frames = 511;
    Wav w;
    int64_t start_us, elapsed_us;
    unsigned frames;

    qtest_writel(q, DPORT + 0xc0, (1 << 4) | (1 << 21));
    qtest_writel(q, MUX + 0x70, (2 << 12) | (1 << 9));
    qtest_writel(q, MUX + 0x74, (2 << 12) | (1 << 9));
    qtest_writel(q, MUX + 0x8c, (2 << 12) | (1 << 9));
    qtest_writel(q, GPIO + 0x530 + PAD_BCLK * 4, 23);
    qtest_writel(q, GPIO + 0x530 + PAD_WS * 4, 25);
    qtest_writel(q, GPIO + 0x530 + PAD_DATA * 4, 163);
    /* RX clocks come from the routed TX pads; DIN is independent and low. */
    qtest_writel(q, GPIO + 0x130 + 27 * 4, PAD_BCLK | 128);
    qtest_writel(q, GPIO + 0x130 + 28 * 4, PAD_WS | 128);
    qtest_writel(q, GPIO + 0x130 + 155 * 4, 16 | 128);
    i2s_write(q, 0xac, (1 << 20) | 6 | (2 << 14) | (1 << 8));
    i2s_write(q, 0xb0, 8 | (8 << 6) | (32 << 12) | (32 << 18));
    i2s_write(q, 0x20, (2 << 13) | (2 << 16) | (1 << 19) | (1 << 20));
    i2s_write(q, 0x08, 0);

    configure(q, 2, 32, 3);
    dac_set(q, REG_RECORD, 1);
    qtest_writel(q, DESC, (1u << 31) | (1u << 30) |
                 (4088u << 12) | 4092u);
    qtest_writel(q, DESC + 4, DATA_BUF);
    qtest_writel(q, DESC + 8, 0);
    qtest_writel(q, RX_DESC, (1u << 31) | 512);
    qtest_writel(q, RX_DESC + 4, RX_DATA);
    qtest_writel(q, RX_DESC + 8, RX_DESC);
    qtest_writel(q, RX_DATA, 0xa5a5a5a5);
    qtest_writel(q, RX_DATA + 508, 0xa5a5a5a5);
    for (unsigned i = 0; i < burst_frames; i++) {
        qtest_writel(q, DATA_BUF + 8 * i, pattern(i, 0, 32));
        qtest_writel(q, DATA_BUF + 8 * i + 4, pattern(i, 1, 32));
    }
    i2s_write(q, 0xa0, 0x89); /* Repeat the final sample on underrun. */
    i2s_write(q, 0x60, (1 << 12) | (1 << 8) | (1 << 6));
    i2s_write(q, 0x20, qtest_readl(q, I2S0 + 0x20) | (1 << 12));
    i2s_write(q, 0x24, 128);
    i2s_write(q, 0x30, (DESC & 0xfffff) | (1 << 29));
    i2s_write(q, 0x34, (RX_DESC & 0xfffff) | (1 << 29));
    i2s_write(q, 0x08, (1 << 4) | (1 << 5) | (1 << 10) | (1 << 18));

    start_us = g_get_monotonic_time();
    qtest_clock_step(q, 1000000000);
    elapsed_us = g_get_monotonic_time() - start_us;
    g_assert_cmpuint(dac_get(q, REG_STATUS), ==, ST_RECORDING);
    g_assert_cmphex(qtest_readl(q, I2S0 + 0x0c) & (1 << 9), ==, 1 << 9);
    g_assert_cmphex(qtest_readl(q, RX_DATA), ==, 0);
    g_assert_cmphex(qtest_readl(q, RX_DATA + 508), ==, 0);
    dac_set(q, REG_RECORD, 0);
    g_assert_cmpuint(dac_get(q, REG_STATUS), ==, 0);
    frames = dac_frames(q);
    g_assert_cmpuint(frames, >=, 47000);
    g_assert_cmpuint(frames, <=, 49000);
    w = wav_load(d.wav);
    g_assert_cmpuint(w.depth, ==, 32);
    g_assert_cmpuint(w.rate, ==, 48000);
    for (unsigned i = 0; i < burst_frames; i++) {
        g_assert_cmphex(wav_sample(&w, i, 0), ==, pattern(i, 0, 32));
        g_assert_cmphex(wav_sample(&w, i, 1), ==, pattern(i, 1, 32));
    }
    for (unsigned i = burst_frames; i < frames; i++) {
        g_assert_cmphex(wav_sample(&w, i, 0), ==,
                        pattern(burst_frames - 1, 0, 32));
        g_assert_cmphex(wav_sample(&w, i, 1), ==,
                        pattern(burst_frames - 1, 1, 32));
    }
    g_test_message("ANALYTIC_AUDIO_BENCH sim_ns=1000000000 host_us=%" PRId64
                   " speed_x=%.3f frames=%u",
                   elapsed_us, 1000000.0 / elapsed_us, dac_frames(q));
    wav_free(&w);
    dut_end(&d);
}

/* Programming 48 kHz while the ESP32 produces 16 kHz is rejected. */
static void real_i2s_wrong_rate(void)
{
    Dut d = dut_start(NULL, NULL);
    QTestState *q = d.q;
    Wav w;

    qtest_writel(q, DPORT + 0xc0, (1 << 4) | (1 << 21));
    qtest_writel(q, MUX + 0x70, (2 << 12) | (1 << 9));
    qtest_writel(q, MUX + 0x74, (2 << 12) | (1 << 9));
    qtest_writel(q, MUX + 0x8c, (2 << 12) | (1 << 9));
    qtest_writel(q, GPIO + 0x530 + PAD_BCLK * 4, 23);
    qtest_writel(q, GPIO + 0x530 + PAD_WS * 4, 25);
    qtest_writel(q, GPIO + 0x530 + PAD_DATA * 4, 163);
    i2s_write(q, 0xac, (1 << 20) | 39);
    i2s_write(q, 0xb0, 8 | (8 << 6) | (16 << 12) | (16 << 18));
    i2s_write(q, 0x20, 32 | (32 << 6) | (1 << 19) | (1 << 20));
    configure(q, 2, 16, 3);
    dac_set(q, REG_RECORD, 1);
    i2s_write(q, 0x00, 0x12345678);
    i2s_write(q, 0x08, (1 << 4) | (1 << 10));
    qtest_clock_step(q, 70000 * 12);
    g_assert_cmpuint(dac_get(q, REG_STATUS), ==, ST_RECORDING | ST_CLOCK);
    dac_set(q, REG_RECORD, 0);
    g_assert_cmpuint(dac_frames(q), ==, 0);
    w = wav_load(d.wav);
    g_assert_cmpuint(w.data_bytes, ==, 0);
    wav_free(&w);
    dut_end(&d);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32/i2s-dac/i2c-registers", i2c_registers);
    qtest_add_data_func("/esp32/i2s-dac/decode/stereo16", &formats[0],
                        decode_format);
    qtest_add_data_func("/esp32/i2s-dac/decode/mono16", &formats[1],
                        decode_format);
    qtest_add_data_func("/esp32/i2s-dac/decode/stereo24", &formats[2],
                        decode_format);
    qtest_add_data_func("/esp32/i2s-dac/decode/mono24", &formats[3],
                        decode_format);
    qtest_add_data_func("/esp32/i2s-dac/decode/stereo32", &formats[4],
                        decode_format);
    qtest_add_data_func("/esp32/i2s-dac/bad-clock/rate",
                        GUINT_TO_POINTER(0), bad_clock);
    qtest_add_data_func("/esp32/i2s-dac/bad-clock/slot-24-for-16",
                        GUINT_TO_POINTER(1), bad_clock);
    qtest_add_data_func("/esp32/i2s-dac/bad-clock/slot-32-for-16",
                        GUINT_TO_POINTER(2), bad_clock);
    qtest_add_func("/esp32/i2s-dac/bad-frame-recovery", bad_frame_recovery);
    qtest_add_func("/esp32/i2s-dac/data-floating", data_floating);
    qtest_add_func("/esp32/i2s-dac/record-idle", record_idle);
    qtest_add_func("/esp32/i2s-dac/restart-recording", restart_recording);
    qtest_add_func("/esp32/i2s-dac/odd-chunk", odd_chunk);
    qtest_add_func("/esp32/i2s-dac/size-limit", size_limit);
    qtest_add_func("/esp32/i2s-dac/real-i2s0", real_i2s);
    qtest_add_func("/esp32/i2s-dac/real-i2s0-trace", real_i2s_trace);
    qtest_add_func("/esp32/i2s-dac/real-i2s0-wrong-route",
                   real_i2s_wrong_physical_route);
    qtest_add_func("/esp32/i2s-dac/analytic-audio-second",
                   analytic_audio_second);
    qtest_add_func("/esp32/i2s-dac/real-i2s0-wrong-rate", real_i2s_wrong_rate);
    return g_test_run();
}
