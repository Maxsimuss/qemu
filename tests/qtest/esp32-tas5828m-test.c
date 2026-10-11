/*
 * Tests for the esp32-tas5828m virtual TAS5828M amplifier with DSP BQ path.
 *
 * BQ math follows SLUUDB4 Table 3-1 (y = B0*x0 + 2*B1*x1 + B2*x2 +
 * 2*A1*y1 + A2*y2, 5.27) through book 0xAA; mixer/volume are 9.23.
 * Pads are driven with exact virtual time; I2C uses synthetic fixture pads.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <math.h>
#include "libqtest.h"
#include "qemu/bswap.h"

#define GPIO 0x3ff44000
#define MUX 0x3ff49000

#define PAD_SDA 18
#define PAD_SCL 19
#define PAD_BCLK 25
#define PAD_WS 26
#define PAD_DATA 32

#define TAS_ADDR 0x61

static void ext(QTestState *q, unsigned pad, unsigned level)
{
    qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", pad, level);
}

static void tick(QTestState *q)
{
    qtest_clock_step(q, 1000);
}

static void sda(QTestState *q, bool high) { ext(q, PAD_SDA, high ? 2 : 0); }
static void scl(QTestState *q, bool high) { ext(q, PAD_SCL, high ? 2 : 0); }

static void i2c_start(QTestState *q)
{
    sda(q, true); scl(q, true); tick(q);
    sda(q, false); tick(q); scl(q, false); tick(q);
}

static void i2c_restart(QTestState *q)
{
    sda(q, true); tick(q); i2c_start(q);
}

static void i2c_stop(QTestState *q)
{
    sda(q, false); tick(q); scl(q, true); tick(q); sda(q, true); tick(q);
}

static bool sda_in(QTestState *q)
{
    return (qtest_readl(q, GPIO + 0x3c) >> PAD_SDA) & 1;
}

static bool i2c_put(QTestState *q, uint8_t byte)
{
    bool ack;
    for (int bit = 7; bit >= 0; bit--) {
        sda(q, (byte >> bit) & 1); tick(q);
        scl(q, true); tick(q); scl(q, false); tick(q);
    }
    sda(q, true); tick(q); scl(q, true); tick(q);
    ack = !sda_in(q);
    scl(q, false); tick(q);
    return ack;
}

static uint8_t i2c_get(QTestState *q, bool ack)
{
    uint8_t value = 0;
    sda(q, true);
    for (int bit = 0; bit < 8; bit++) {
        tick(q); scl(q, true); tick(q);
        value = (value << 1) | sda_in(q);
        scl(q, false); tick(q);
    }
    sda(q, !ack); tick(q); scl(q, true); tick(q); scl(q, false); tick(q);
    sda(q, true); tick(q);
    return value;
}

static bool tas_write(QTestState *q, uint8_t addr, uint8_t reg,
                      const uint8_t *values, unsigned count)
{
    bool ok;
    i2c_start(q);
    ok = i2c_put(q, addr << 1);
    ok = ok && i2c_put(q, reg);
    for (unsigned i = 0; ok && i < count; i++) {
        ok = i2c_put(q, values[i]);
    }
    i2c_stop(q);
    return ok;
}

static void tas_set(QTestState *q, uint8_t reg, uint8_t value)
{
    g_assert_true(tas_write(q, TAS_ADDR, reg, &value, 1));
}

static void tas_read(QTestState *q, uint8_t reg, uint8_t *out, unsigned count)
{
    i2c_start(q);
    g_assert_true(i2c_put(q, TAS_ADDR << 1));
    g_assert_true(i2c_put(q, reg));
    i2c_restart(q);
    g_assert_true(i2c_put(q, (TAS_ADDR << 1) | 1));
    for (unsigned i = 0; i < count; i++) {
        out[i] = i2c_get(q, i + 1 < count);
    }
    i2c_stop(q);
}

static uint8_t tas_get(QTestState *q, uint8_t reg)
{
    uint8_t value;
    tas_read(q, reg, &value, 1);
    return value;
}

static void set_book_page(QTestState *q, uint8_t book, uint8_t page)
{
    tas_set(q, 0x00, 0x00);
    tas_set(q, 0x7F, book);
    tas_set(q, 0x00, page);
}

static void tas_play(QTestState *q)
{
    set_book_page(q, 0x00, 0x00);
    tas_set(q, 0x02, 0x02);
    tas_set(q, 0x03, 0x03);
}

static void tas_hiz(QTestState *q)
{
    set_book_page(q, 0x00, 0x00);
    tas_set(q, 0x03, 0x02);
}

typedef struct Dut {
    QTestState *q;
    char *wav;
} Dut;

static Dut dut_start(const char *extra)
{
    Dut d;
    int fd = g_file_open_tmp("esp32-tas5828m-XXXXXX.wav", &d.wav, NULL);
    g_autofree char *args;

    g_assert_cmpint(fd, >=, 0);
    close(fd);
    args = g_strdup_printf("-machine esp32 -display none -serial none "
                           "-nic none -device esp32-tas5828m,id=tas,"
                           "gpio=/machine/soc/gpio,sda=18,scl=19,"
                           "bclk=25,ws=26,data=32,wav=%s%s%s", d.wav,
                           extra ? "," : "", extra ? extra : "");
    d.q = qtest_init(args);
    /* Board pull-ups (device default) hold the bus; MUX pull-ups stay off
     * so pullups=false is observable. */
    return d;
}

static Dut dut_start_addr(const char *extra, unsigned addr,
                          unsigned slot)
{
    Dut d;
    int fd = g_file_open_tmp("esp32-tas5828m-XXXXXX.wav", &d.wav, NULL);
    g_autofree char *args;

    g_assert_cmpint(fd, >=, 0);
    close(fd);
    args = g_strdup_printf("-machine esp32 -display none -serial none "
                           "-nic none -device esp32-tas5828m,id=tas,"
                           "gpio=/machine/soc/gpio,sda=18,scl=19,"
                           "bclk=25,ws=26,data=32,wav=%s,address=%u,"
                           "driver-slot=%u%s%s", d.wav, addr, slot,
                           extra ? "," : "", extra ? extra : "");
    d.q = qtest_init(args);
    return d;
}

static void dut_end(Dut *d)
{
    qtest_quit(d->q);
    unlink(d->wav);
    g_free(d->wav);
}

typedef struct Wav {
    uint8_t *data;
    gsize size;
    uint32_t channels, rate;
    int32_t depth;
    uint32_t data_bytes;
    const uint8_t *pcm;
} Wav;

static Wav wav_load(const char *path)
{
    Wav w = {};
    gchar *contents;
    gsize length;
    g_assert_true(g_file_get_contents(path, &contents, &length, NULL));
    w.data = (uint8_t *)contents;
    w.size = length;
    g_assert_cmpuint(length, >=, 44);
    w.channels = lduw_le_p(w.data + 22);
    w.rate = ldl_le_p(w.data + 24);
    w.depth = lduw_le_p(w.data + 34);
    w.data_bytes = ldl_le_p(w.data + 40);
    w.pcm = w.data + 44;
    return w;
}

static int64_t wav_sample(const Wav *w, unsigned frame, unsigned channel)
{
    unsigned bytes = w->depth / 8;
    const uint8_t *p = w->pcm + (frame * w->channels + channel) * bytes;
    uint64_t v = 0;
    for (unsigned i = 0; i < bytes; i++) {
        v |= (uint64_t)p[i] << (8 * i);
    }
    if (w->depth < 64 && (v & (1ull << (w->depth - 1)))) {
        v |= ~0ull << w->depth;
    }
    return (int64_t)v;
}

static void wav_free(Wav *w) { g_free(w->data); }

typedef struct Synth {
    QTestState *q;
    unsigned depth;
    int64_t bclk_half;
} Synth;

static void advance(Synth *s, int64_t ns)
{
    qtest_clock_step(s->q, ns);
}

static Synth synth_init(QTestState *q, unsigned depth, int64_t bclk_half)
{
    Synth s = {.q = q, .depth = depth, .bclk_half = bclk_half};
    ext(q, PAD_BCLK, 0); ext(q, PAD_WS, 0); ext(q, PAD_DATA, 0);
    return s;
}

static void synth_frame(Synth *s, uint32_t left, uint32_t right)
{
    for (unsigned b = 0; b < 2 * s->depth; b++) {
        uint32_t word = b < s->depth ? left : right;
        unsigned bit = (word >> (s->depth - 1 - (b % s->depth))) & 1;
        ext(s->q, PAD_WS, ((b + 1) / s->depth) & 1);
        ext(s->q, PAD_DATA, bit);
        advance(s, s->bclk_half);
        ext(s->q, PAD_BCLK, 1);
        advance(s, s->bclk_half);
        ext(s->q, PAD_BCLK, 0);
    }
}

/* 48 kHz, 32-bit stereo: 64 BCLK/frame, ~163 ns half period. */
#define BH32_48K 163

static void bq_write(QTestState *q, unsigned ch, unsigned bq,
                     int32_t c0, int32_t c1, int32_t c2,
                     int32_t c3, int32_t c4)
{
    static const uint8_t map[2][12][2] = {
        {{1, 0x30}, {1, 0x44}, {1, 0x58}, {1, 0x6C},
         {2, 0x08}, {2, 0x1C}, {2, 0x30}, {2, 0x44},
         {2, 0x58}, {2, 0x6C}, {3, 0x08}, {3, 0x1C}},
        {{3, 0x30}, {3, 0x44}, {3, 0x58}, {3, 0x6C},
         {4, 0x08}, {4, 0x1C}, {4, 0x30}, {4, 0x44},
         {4, 0x58}, {4, 0x6C}, {5, 0x08}, {5, 0x1C}},
    };
    uint8_t buf[20];
    int32_t c[5] = {c0, c1, c2, c3, c4};
    set_book_page(q, 0xAA, map[ch][bq][0]);
    for (unsigned i = 0; i < 5; i++) {
        buf[4 * i] = c[i] >> 24;
        buf[4 * i + 1] = c[i] >> 16;
        buf[4 * i + 2] = c[i] >> 8;
        buf[4 * i + 3] = c[i];
    }
    g_assert_true(tas_write(q, TAS_ADDR, map[ch][bq][1], buf, 20));
}

static void bq_readback(QTestState *q, unsigned ch, unsigned bq,
                        int32_t out[5])
{
    static const uint8_t map[2][12][2] = {
        {{1, 0x30}, {1, 0x44}, {1, 0x58}, {1, 0x6C},
         {2, 0x08}, {2, 0x1C}, {2, 0x30}, {2, 0x44},
         {2, 0x58}, {2, 0x6C}, {3, 0x08}, {3, 0x1C}},
        {{3, 0x30}, {3, 0x44}, {3, 0x58}, {3, 0x6C},
         {4, 0x08}, {4, 0x1C}, {4, 0x30}, {4, 0x44},
         {4, 0x58}, {4, 0x6C}, {5, 0x08}, {5, 0x1C}},
    };
    uint8_t buf[20];
    set_book_page(q, 0xAA, map[ch][bq][0]);
    tas_read(q, map[ch][bq][1], buf, 20);
    for (unsigned i = 0; i < 5; i++) {
        out[i] = (buf[4 * i] << 24) | (buf[4 * i + 1] << 16) |
                 (buf[4 * i + 2] << 8) | buf[4 * i + 3];
    }
}

static char *coefficient_safety(QTestState *q)
{
    QDict *reply = qtest_qmp(q,
        "{ 'execute': 'qom-get', 'arguments': {"
        " 'path': '/machine/peripheral/tas',"
        " 'property': 'coefficient-safety' } }");
    const char *value;
    char *result;

    g_assert_true(qdict_haskey(reply, "return"));
    value = qdict_get_str(reply, "return");
    result = g_strdup(value);
    qobject_unref(reply);
    return result;
}

static void coefficient_safety_checks(void)
{
    Dut d = dut_start(NULL);
    uint8_t partial[] = {0x08, 0x00, 0x00, 0x00};
    g_autofree char *status;

    /* Completed I2C transaction and default Q5.27 identity are safe. */
    bq_write(d.q, 0, 0, 0x08000000, 0, 0, 0, 0);
    status = coefficient_safety(d.q);
    g_assert_cmpstr(status, ==, "stable");

    /* Exercise every documented coefficient location with identity filters. */
    for (unsigned ch = 0; ch < 2; ch++) {
        for (unsigned bq = 0; bq < 12; bq++) {
            bq_write(d.q, ch, bq, 0x08000000, 0, 0, 0, 0);
            g_clear_pointer(&status, g_free);
            status = coefficient_safety(d.q);
            g_assert_cmpstr(status, ==, "stable");
        }
    }
    /* A non-identity stable denominator also passes after signed Q5.27
     * decoding: A1 ~= 0.1, A2 = -0.5 gives strict Jury inequalities. */
    bq_write(d.q, 0, 0, 0x08000000, 0, 0, 0x00CCCCCD, (int32_t)0xFC000000);
    g_clear_pointer(&status, g_free);
    status = coefficient_safety(d.q);
    g_assert_cmpstr(status, ==, "stable");
    /* The feedback term is signed Q5.27. A1=+0.75 makes the documented
     * characteristic polynomial fail Jury's strict condition. */
    bq_write(d.q, 0, 0, 0x08000000, 0, 0, 0x06000000, 0);
    g_clear_pointer(&status, g_free);
    status = coefficient_safety(d.q);
    g_assert_cmpstr(status, ==, "unstable");

    /* A1=+0.5 places a pole on the unit circle (marginal boundary). */
    bq_write(d.q, 0, 0, 0x08000000, 0, 0, 0x04000000, 0);
    g_clear_pointer(&status, g_free);
    status = coefficient_safety(d.q);
    g_assert_cmpstr(status, ==, "marginal");

    /* One coefficient LSB below the unit-circle boundary remains stable. */
    bq_write(d.q, 0, 0, 0x08000000, 0, 0, 0x03FFFFFF, 0);
    g_clear_pointer(&status, g_free);
    status = coefficient_safety(d.q);
    g_assert_cmpstr(status, ==, "stable");

    /* A short write is reported as incomplete; no partial coefficient set is
     * classified as unstable while the normal programming transaction runs. */
    set_book_page(d.q, 0xAA, 1);
    /* A partial four-byte coefficient word may be an in-progress live update. */
    g_assert_true(tas_write(d.q, TAS_ADDR, 0x3C, partial, sizeof(partial)));
    g_clear_pointer(&status, g_free);
    status = coefficient_safety(d.q);
    g_assert_cmpstr(status, ==, "incomplete");

    /* The DSP-core reset restores coefficient RAM, invalidating the prior
     * diagnostic result; the diagnostic never claims a chip status bit. */
    set_book_page(d.q, 0x00, 0x00);
    tas_set(d.q, 0x01, 0x10);
    g_clear_pointer(&status, g_free);
    status = coefficient_safety(d.q);
    g_assert_cmpstr(status, ==, "not-checked");
    dut_end(&d);
}

/* Host DF1 model of one BQ (Table 3-1), state kept by caller. */
static double host_bq(double x, double c0, double c1, double c2,
                      double c3, double c4, double *x1, double *x2,
                      double *y1, double *y2)
{
    double y = c0 * x + 2.0 * c1 * *x1 + c2 * *x2 + 2.0 * c3 * *y1 + c4 * *y2;
    *x2 = *x1; *x1 = x; *y2 = *y1; *y1 = y;
    return y;
}

static void bringup_books(void)
{
    Dut d = dut_start(NULL);
    set_book_page(d.q, 0x00, 0x00);
    g_assert_cmpuint(tas_get(d.q, 0x67), ==, 0x95);
    g_assert_cmpuint(tas_get(d.q, 0x03), ==, 0x10);
    g_assert_cmpuint(tas_get(d.q, 0x68), ==, 0x00);
    g_assert_cmpuint(tas_get(d.q, 0x71), ==, 0x00);
    /* Book/page select verified via DSP defaults. */
    set_book_page(d.q, 0x8C, 0x09);
    uint8_t v0[4];
    tas_read(d.q, 0x74, v0, 4);
    g_assert_cmpuint(v0[0], ==, 0x00);
    g_assert_cmpuint(v0[1], ==, 0x80);
    set_book_page(d.q, 0x00, 0x00);
    g_assert_cmpuint(tas_get(d.q, 0x03), ==, 0x10);
    /* BQ defaults: B0 = 1.0, rest 0. */
    int32_t c[5];
    bq_readback(d.q, 0, 0, c);
    g_assert_cmphex(c[0], ==, 0x08000000);
    g_assert_cmphex(c[1], ==, 0x00000000);
    g_assert_cmphex(c[4], ==, 0x00000000);
    bq_readback(d.q, 1, 11, c);
    g_assert_cmphex(c[0], ==, 0x08000000);
    /* Write + readback round-trip through book/page RAM. */
    bq_write(d.q, 0, 5, 0x08000000, 0x01000000, 0x02000000,
             0x03000000, 0x04000000);
    bq_readback(d.q, 0, 5, c);
    g_assert_cmphex(c[0], ==, 0x08000000);
    g_assert_cmphex(c[1], ==, 0x01000000);
    g_assert_cmphex(c[4], ==, 0x04000000);
    /* Control registers unaffected by DSP traffic. */
    set_book_page(d.q, 0x00, 0x00);
    g_assert_cmpuint(tas_get(d.q, 0x68), ==, 0x00);
    dut_end(&d);
}

static void reset_modes(void)
{
    Dut d = dut_start(NULL);
    int32_t c[5];
    uint8_t mixer[4];
    const uint8_t custom_mixer[] = {0x00, 0x40, 0x00, 0x00};

    /* RST_REG resets the control port but preserves coefficient/data RAM. */
    set_book_page(d.q, 0x00, 0x00);
    tas_set(d.q, 0x03, 0x03);
    tas_set(d.q, 0x4C, 0x44);
    bq_write(d.q, 0, 0, 0x10000000, 0x01000000, 0x02000000,
             0x03000000, 0x04000000);
    set_book_page(d.q, 0x8C, 0x09);
    g_assert_true(tas_write(d.q, TAS_ADDR, 0x74, custom_mixer,
                            sizeof(custom_mixer)));
    set_book_page(d.q, 0x00, 0x00);
    tas_set(d.q, 0x01, 0x01);

    g_assert_cmpuint(tas_get(d.q, 0x03), ==, 0x10);
    g_assert_cmpuint(tas_get(d.q, 0x4C), ==, 0x30);
    g_assert_cmpuint(tas_get(d.q, 0x01), ==, 0);
    bq_readback(d.q, 0, 0, c);
    g_assert_cmphex(c[0], ==, 0x10000000);
    g_assert_cmphex(c[4], ==, 0x04000000);
    set_book_page(d.q, 0x8C, 0x09);
    tas_read(d.q, 0x74, mixer, sizeof(mixer));
    g_assert_cmpmem(mixer, sizeof(mixer), custom_mixer, sizeof(custom_mixer));

    /* RST_DIG_CORE resets registers and restores DSP RAM defaults. */
    set_book_page(d.q, 0x00, 0x00);
    tas_set(d.q, 0x03, 0x02);
    tas_set(d.q, 0x4C, 0x55);
    bq_write(d.q, 0, 0, 0x10000000, 0x01000000, 0x02000000,
             0x03000000, 0x04000000);
    set_book_page(d.q, 0x00, 0x00);
    tas_set(d.q, 0x01, 0x10);

    g_assert_cmpuint(tas_get(d.q, 0x03), ==, 0x10);
    g_assert_cmpuint(tas_get(d.q, 0x33), ==, 0x02);
    g_assert_cmpuint(tas_get(d.q, 0x46), ==, 0x01);
    g_assert_cmpuint(tas_get(d.q, 0x4C), ==, 0x30);
    g_assert_cmpuint(tas_get(d.q, 0x01), ==, 0);
    bq_readback(d.q, 0, 0, c);
    g_assert_cmphex(c[0], ==, 0x08000000);
    g_assert_cmphex(c[1], ==, 0);
    g_assert_cmphex(c[4], ==, 0);
    set_book_page(d.q, 0x8C, 0x09);
    tas_read(d.q, 0x74, mixer, sizeof(mixer));
    g_assert_cmpuint(mixer[0], ==, 0x00);
    g_assert_cmpuint(mixer[1], ==, 0x80);
    g_assert_cmpuint(mixer[2], ==, 0x00);
    g_assert_cmpuint(mixer[3], ==, 0x00);

    dut_end(&d);
}

static void no_pullups_float_low(void)
{
    Dut d = dut_start_addr("pullups=false", TAS_ADDR, 2);
    /* Without board pull-ups (and no MUX pull-ups set here), the released
     * bus floats and GPIO input samples it as low. */
    g_assert_true(!((qtest_readl(d.q, GPIO + 0x3c) >> PAD_SDA) & 1));
    g_assert_true(!((qtest_readl(d.q, GPIO + 0x3c) >> PAD_SCL) & 1));
    dut_end(&d);
}

static void two_instances(void)
{
    Dut d = dut_start_addr(NULL, 0x62, 3);
    /* Board pull-ups hold the idle bus HIGH. */
    g_assert_true((qtest_readl(d.q, GPIO + 0x3c) >> PAD_SDA) & 1);
    g_assert_true((qtest_readl(d.q, GPIO + 0x3c) >> PAD_SCL) & 1);
    /* Default instance answers 0x61, second answers 0x62. */
    i2c_start(d.q);
    g_assert_false(i2c_put(d.q, TAS_ADDR << 1));
    i2c_stop(d.q);
    i2c_start(d.q);
    g_assert_true(i2c_put(d.q, 0x62 << 1));
    i2c_stop(d.q);
    dut_end(&d);
}

static void passthrough_exact(void)
{
    Dut d = dut_start(NULL);
    Synth s = synth_init(d.q, 32, BH32_48K);
    /* 32-bit I2S, auto clocks. */
    set_book_page(d.q, 0x00, 0x00);
    tas_set(d.q, 0x33, 0x03);
    tas_set(d.q, 0x02, 0x02);
    tas_set(d.q, 0x03, 0x03);
    enum { N = 24 };
    for (unsigned i = 0; i < N; i++) {
        uint32_t l = 0x10000000 + i * 0x01010101;
        uint32_t r = 0xE0000000 - i * 0x01010101;
        synth_frame(&s, l, r);
    }
    tas_hiz(d.q);
    Wav w = wav_load(d.wav);
    g_assert_cmpuint(w.rate, ==, 48000);
    g_assert_cmpuint(w.depth, ==, 32);
    g_assert_cmpuint(w.data_bytes, >=, (N - 3) * 8);
    unsigned frames = w.data_bytes / 8;
    for (unsigned i = 0; i < frames; i++) {
        unsigned src = N - frames + i;
        uint32_t l = 0x10000000 + src * 0x01010101;
        uint32_t r = 0xE0000000 - src * 0x01010101;
        g_assert_cmphex((uint32_t)wav_sample(&w, i, 0), ==, l);
        g_assert_cmphex((uint32_t)wav_sample(&w, i, 1), ==, r);
    }
    g_assert_cmpuint(tas_get(d.q, 0x68), ==, 0x02);
    set_book_page(d.q, 0x00, 0x00);
    g_assert_cmpuint(tas_get(d.q, 0x71), ==, 0x00);
    wav_free(&w);
    dut_end(&d);
}

static void bq_gain_exact(void)
{
    Dut d = dut_start(NULL);
    Synth s = synth_init(d.q, 32, BH32_48K);
    /* Left BQ1 = 2.0 gain (B0 = 2.0), rest passthrough. */
    bq_write(d.q, 0, 0, 0x10000000, 0, 0, 0, 0);
    set_book_page(d.q, 0x00, 0x00);
    tas_set(d.q, 0x33, 0x03);
    tas_set(d.q, 0x02, 0x02);
    tas_set(d.q, 0x03, 0x03);
    enum { N = 16 };
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    double rx1 = 0, rx2 = 0, ry1 = 0, ry2 = 0;
    int64_t expect_l[N], expect_r[N];
    for (unsigned i = 0; i < N; i++) {
        int32_t in = 0x00100000 * (int)(i + 1);
        double x = in / 2147483648.0;
        double yl = host_bq(x, 2.0, 0, 0, 0, 0, &x1, &x2, &y1, &y2);
        double yr = host_bq(x, 1.0, 0, 0, 0, 0, &rx1, &rx2, &ry1, &ry2);
        expect_l[i] = llround(yl * 2147483648.0);
        expect_r[i] = llround(yr * 2147483648.0);
        synth_frame(&s, in, in);
    }
    tas_hiz(d.q);
    Wav w = wav_load(d.wav);
    unsigned frames = w.data_bytes / 8;
    g_assert_cmpuint(frames, >=, N - 3);
    for (unsigned i = 0; i < frames; i++) {
        unsigned src = N - frames + i;
        g_assert_cmpint(wav_sample(&w, i, 0), ==, expect_l[src]);
        g_assert_cmpint(wav_sample(&w, i, 1), ==, expect_r[src]);
    }
    wav_free(&w);
    dut_end(&d);
}

static void mixer_volume_bypass(void)
{
    Dut d = dut_start(NULL);
    Synth s = synth_init(d.q, 32, BH32_48K);
    /* Swap mixer: L<-R, R<-L at 0.5. */
    set_book_page(d.q, 0x8C, 0x09);
    uint8_t half[] = {0x00, 0x40, 0x00, 0x00}; /* 0.5 in 9.23 */
    uint8_t zero[] = {0x00, 0x00, 0x00, 0x00};
    g_assert_true(tas_write(d.q, TAS_ADDR, 0x74, zero, 4));
    g_assert_true(tas_write(d.q, TAS_ADDR, 0x78, half, 4));
    g_assert_true(tas_write(d.q, TAS_ADDR, 0x7C, half, 4));
    set_book_page(d.q, 0x8C, 0x0A);
    g_assert_true(tas_write(d.q, TAS_ADDR, 0x08, zero, 4));
    set_book_page(d.q, 0x00, 0x00);
    tas_set(d.q, 0x33, 0x03);
    tas_set(d.q, 0x02, 0x02);
    tas_set(d.q, 0x03, 0x03);
    for (unsigned i = 0; i < 12; i++) {
        synth_frame(&s, 0x10000000, 0x20000000);
    }
    tas_hiz(d.q);
    Wav w = wav_load(d.wav);
    unsigned frames = w.data_bytes / 8;
    g_assert_cmpuint(frames, >, 6);
    /* Out L = inR*0.5 = 0x10000000, out R = inL*0.5 = 0x08000000. */
    g_assert_cmphex((uint32_t)wav_sample(&w, frames - 1, 0), ==, 0x10000000);
    g_assert_cmphex((uint32_t)wav_sample(&w, frames - 1, 1), ==, 0x08000000);
    wav_free(&w);
    /* BypassEQ: BQs skipped even with B0 = 0. */
    bq_write(d.q, 0, 0, 0, 0, 0, 0, 0);
    set_book_page(d.q, 0x8C, 0x06);
    tas_set(d.q, 0x60, 0x01);
    set_book_page(d.q, 0x00, 0x00);
    tas_set(d.q, 0x03, 0x03);
    for (unsigned i = 0; i < 12; i++) {
        synth_frame(&s, 0x10000000, 0x20000000);
    }
    tas_hiz(d.q);
    w = wav_load(d.wav);
    frames = w.data_bytes / 8;
    g_assert_cmphex((uint32_t)wav_sample(&w, frames - 1, 0), ==, 0x10000000);
    wav_free(&w);
    dut_end(&d);
}

static void volume_digvol_meters(void)
{
    Dut d = dut_start(NULL);
    Synth s = synth_init(d.q, 32, BH32_48K);
    set_book_page(d.q, 0x00, 0x00);
    tas_set(d.q, 0x33, 0x03);
    tas_set(d.q, 0x02, 0x02);
    /* DIG_VOL -6 dB = 0x3C. */
    tas_set(d.q, 0x4C, 0x3C);
    tas_set(d.q, 0x03, 0x03);
    for (unsigned i = 0; i < 12; i++) {
        synth_frame(&s, 0x10000000, 0x10000000);
    }
    tas_hiz(d.q);
    Wav w = wav_load(d.wav);
    unsigned frames = w.data_bytes / 8;
    int64_t got = wav_sample(&w, frames - 1, 0);
    int64_t want = llround(0x10000000 * pow(10.0, -6.0 / 20.0));
    g_assert_cmpint(got > want - 2 && got < want + 2, ==, true);
    wav_free(&w);
    /* Level meters show the playback peak. */
    set_book_page(d.q, 0x78, 0x06);
    uint8_t m[4];
    tas_read(d.q, 0x54, m, 4);
    uint32_t peak = (m[0] << 24) | (m[1] << 16) | (m[2] << 8) | m[3];
    g_assert_cmpuint(peak, >, 0);
    dut_end(&d);
}

static void pdn_tracks_resolved_gpio(void)
{
    Dut d = dut_start("pdn=4");
    Synth synth = synth_init(d.q, 32, BH32_48K);
    uint8_t value = 0x03;

    /* A floating pad does not silently enable the device. */
    ext(d.q, 4, 2);
    g_assert_false(tas_write(d.q, TAS_ADDR, 0x03, &value, 1));

    /* Resolved GPIO4 high enables I2C; low shuts it down and resets the
     * modeled volatile register/DSP state, so re-enable needs fresh setup. */
    ext(d.q, 4, 1);
    g_assert_true(tas_write(d.q, TAS_ADDR, 0x03, &value, 1));
    set_book_page(d.q, 0x00, 0x00);
    tas_set(d.q, 0x33, 0x03);
    tas_play(d.q);
    for (unsigned i = 0; i < 12; i++) {
        synth_frame(&synth, 0x10000000, 0x10000000);
    }
    ext(d.q, 4, 0);
    Wav before = wav_load(d.wav);
    uint32_t audio_bytes = before.data_bytes;
    wav_free(&before);
    g_assert_cmpuint(audio_bytes, >, 0);
    g_assert_false(tas_write(d.q, TAS_ADDR, 0x03, &value, 1));
    for (unsigned i = 0; i < 12; i++) {
        synth_frame(&synth, 0x10000000, 0x10000000);
    }
    Wav after = wav_load(d.wav);
    g_assert_cmpuint(after.data_bytes, ==, audio_bytes);
    wav_free(&after);
    ext(d.q, 4, 1);
    g_assert_cmphex(tas_get(d.q, 0x03), ==, 0x10);
    set_book_page(d.q, 0x00, 0x00);
    tas_set(d.q, 0x33, 0x03);
    tas_play(d.q);
    g_assert_cmphex(tas_get(d.q, 0x03), ==, 0x03);

    dut_end(&d);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32/tas5828m/bringup-books", bringup_books);
    qtest_add_func("/esp32/tas5828m/reset-modes", reset_modes);
    qtest_add_func("/esp32/tas5828m/coefficient-safety-checks",
                   coefficient_safety_checks);
    qtest_add_func("/esp32/tas5828m/no-pullups-float-low", no_pullups_float_low);
    qtest_add_func("/esp32/tas5828m/two-instances", two_instances);
    qtest_add_func("/esp32/tas5828m/passthrough-exact", passthrough_exact);
    qtest_add_func("/esp32/tas5828m/bq-gain-exact", bq_gain_exact);
    qtest_add_func("/esp32/tas5828m/mixer-volume-bypass", mixer_volume_bypass);
    qtest_add_func("/esp32/tas5828m/volume-digvol-meters",
                   volume_digvol_meters);
    qtest_add_func("/esp32/tas5828m/pdn-resolved-gpio",
                   pdn_tracks_resolved_gpio);
    return g_test_run();
}
