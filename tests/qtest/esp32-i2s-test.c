/* Independent ESP32 digital I2S conformance cases.
 * Register/format expectations: ESP32 TRM v5.8 chapters 2, 6 and 22.
 * All serial samples use resolved pads; DMA descriptors reside in guest DRAM.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define GPIO 0x3ff44000
#define MUX 0x3ff49000
#define DPORT 0x3ff00000
#define DESC 0x3ffb0000
#define DATA 0x3ffb0100
#define CLK 18
#define WS 19
#define SD 23
static const uint64_t base[] = {0x3ff4f000, 0x3ff6d000};

typedef struct TestBus {
    QTestState *q;
    unsigned port;
    uint64_t regs;
} TestBus;

static void write_reg(TestBus *b, unsigned offset, uint32_t value)
{
    qtest_writel(b->q, b->regs + offset, value);
}

static uint32_t read_reg(TestBus *b, unsigned offset)
{
    return qtest_readl(b->q, b->regs + offset);
}

static bool pin(TestBus *b, unsigned pad)
{
    return (qtest_readl(b->q, GPIO + 0x3c) >> pad) & 1;
}

static TestBus setup(unsigned port, bool tx, const char *extra)
{
    TestBus b = {.port = port, .regs = base[port]};
    g_autofree char *args = g_strdup_printf("-machine esp32 -display none -serial none -nic none %s", extra ? extra : "");
    b.q = qtest_init(args);
    qtest_writel(b.q, DPORT + 0xc0, (1 << 4) | (1 << 21));
    qtest_writel(b.q, MUX + 0x70, (2 << 12) | (1 << 9));
    qtest_writel(b.q, MUX + 0x74, (2 << 12) | (1 << 9));
    qtest_writel(b.q, MUX + 0x8c, (2 << 12) | (1 << 9));
    if (tx) {
        qtest_writel(b.q, GPIO + 0x530 + CLK * 4, port ? 24 : 23);
        qtest_writel(b.q, GPIO + 0x530 + WS * 4, port ? 26 : 25);
        qtest_writel(b.q, GPIO + 0x530 + SD * 4, port ? 189 : 163);
    } else {
        qtest_writel(b.q, GPIO + 0x130 + (port ? 164 : 27) * 4, CLK | 128);
        qtest_writel(b.q, GPIO + 0x130 + (port ? 165 : 28) * 4, WS | 128);
        qtest_writel(b.q, GPIO + 0x130 + (port ? 181 : 155) * 4, SD | 128);
    }
    write_reg(&b, 0xac, (1 << 20) | 10); /* 160 MHz / 10 = 16 MHz MCLK */
    write_reg(&b, 0xb0, 8 | (8 << 6) | (16 << 12) | (16 << 18));
    write_reg(&b, 0x20, 32 | (32 << 6) | (1 << 19) | (1 << 20));
    write_reg(&b, 0x08, 0);
    return b;
}

static void check_frame(TestBus *b, uint32_t expected, bool shift)
{
    uint32_t received = 0;
    for (unsigned bit = 0; bit < 32; bit++) {
        g_assert_false(pin(b, CLK));
        g_assert_cmpint(pin(b, WS), ==, ((bit + shift) / 16) & 1);
        qtest_clock_step(b->q, 250); /* BCLK sample rising edge */
        g_assert_true(pin(b, CLK));
        received = (received << 1) | pin(b, SD);
        qtest_clock_step(b->q, 250); /* next setup falling edge */
    }
    g_assert_cmphex(received, ==, expected);
}

static void tx_format(gconstpointer data)
{
    unsigned port = GPOINTER_TO_UINT(data) & 1;
    bool shift = GPOINTER_TO_UINT(data) & 2;
    TestBus b = setup(port, true, NULL);
    write_reg(&b, 0x00, 0xa55ac33c);
    write_reg(&b, 0x08, (1 << 4) | (shift ? 1 << 10 : 0));
    check_frame(&b, 0xa55ac33c, shift);
    /* Hardware repeats the last frame when TX_STOP_EN is disabled. */
    write_reg(&b, 0xa0, 0x89);
    check_frame(&b, 0xa55ac33c, shift);
    write_reg(&b, 0x08, 0);
    qtest_quit(b.q);
}

static void external(TestBus *b, unsigned pad, bool value)
{
    qtest_set_irq_in(b->q, "/machine/soc/gpio", "pad-drive", pad, value);
}

static void inject_frame(TestBus *b, uint32_t word)
{
    for (unsigned bit = 0; bit < 32; bit++) {
        external(b, CLK, false);
        external(b, WS, bit >= 16);
        external(b, SD, (word >> (31 - bit)) & 1);
        qtest_clock_step(b->q, 250);
        external(b, CLK, true);
        qtest_clock_step(b->q, 250);
    }
}

static void rx_slave(gconstpointer data)
{
    TestBus b = setup(GPOINTER_TO_UINT(data), false, NULL);
    write_reg(&b, 0x08, (1 << 5) | (1 << 7));
    inject_frame(&b, 0x81234fed);
    inject_frame(&b, 0x76549abc);
    g_assert_cmphex(read_reg(&b, 0x04), ==, 0x81234fed);
    g_assert_cmphex(read_reg(&b, 0x04), ==, 0x76549abc);
    write_reg(&b, 0x18, UINT32_MAX);
    read_reg(&b, 0x04);
    g_assert_cmphex(read_reg(&b, 0x0c) & (1 << 3), ==, 1 << 3);
    qtest_quit(b.q);
}

static void dma_tx(gconstpointer data)
{
    TestBus b = setup(GPOINTER_TO_UINT(data), true, NULL);
    g_autofree char *path = g_strdup_printf("/machine/soc/i2s%u", b.port);
    qtest_irq_intercept_out_named(b.q, path, "sysbus-irq");
    qtest_writel(b.q, DESC, (1u << 31) | (1 << 30) | (4 << 12) | 4);
    qtest_writel(b.q, DESC + 4, DATA);
    qtest_writel(b.q, DESC + 8, 0);
    qtest_writel(b.q, DATA, 0x12345678);
    write_reg(&b, 0x60, (1 << 12) | (1 << 8) | (1 << 6));
    write_reg(&b, 0x20, read_reg(&b, 0x20) | (1 << 12));
    write_reg(&b, 0x30, (DESC & 0xfffff) | (1 << 29));
    g_assert_false(qtest_get_irq(b.q, 0));
    write_reg(&b, 0x08, 1 << 4);
    check_frame(&b, 0x12345678, false);
    g_assert_cmphex(qtest_readl(b.q, DESC) & (1u << 31), ==, 0);
    g_assert_cmphex(read_reg(&b, 0x38), ==, DESC);
    g_assert_cmphex(read_reg(&b, 0x0c) & ((1 << 11) | (1 << 12) | (1 << 16)), ==,
                    (1 << 11) | (1 << 12) | (1 << 16));
    write_reg(&b, 0x14, 1 << 12);
    g_assert_true(qtest_get_irq(b.q, 0));
    write_reg(&b, 0x18, 1 << 12);
    g_assert_false(qtest_get_irq(b.q, 0));
    write_reg(&b, 0x08, 0);
    write_reg(&b, 0x30, (DESC & 0xfffff) | (1 << 29));
    g_assert_cmphex(read_reg(&b, 0x0c) & (1 << 14), ==, 1 << 14);
    qtest_writel(b.q, DPORT + 0xc4, 1 << (b.port ? 21 : 4));
    g_assert_cmphex(read_reg(&b, 0x08), ==, 0x30300);
    g_assert_cmphex(read_reg(&b, 0x0c), ==, 0);
    g_assert_false(qtest_get_irq(b.q, 0));
    qtest_quit(b.q);
}

static void dma_rx(gconstpointer data)
{
    TestBus b = setup(GPOINTER_TO_UINT(data), false, NULL);
    qtest_writel(b.q, DESC, (1u << 31) | 8);
    qtest_writel(b.q, DESC + 4, DATA);
    qtest_writel(b.q, DESC + 8, 0);
    write_reg(&b, 0x60, 1 << 12);
    write_reg(&b, 0x24, 2);
    write_reg(&b, 0x20, read_reg(&b, 0x20) | (1 << 12));
    write_reg(&b, 0x34, (DESC & 0xfffff) | (1 << 29));
    write_reg(&b, 0x08, (1 << 5) | (1 << 7));
    inject_frame(&b, 0xabcd0123);
    inject_frame(&b, 0xfaceb00c);
    g_assert_cmphex(qtest_readl(b.q, DATA), ==, 0xabcd0123);
    g_assert_cmphex(qtest_readl(b.q, DATA + 4), ==, 0xfaceb00c);
    g_assert_cmphex(qtest_readl(b.q, DESC), ==, (1 << 30) | (8 << 12) | 8);
    g_assert_cmphex(read_reg(&b, 0x3c), ==, DESC);
    g_assert_cmphex(read_reg(&b, 0x0c) & (1 << 9), ==, 1 << 9);
    qtest_quit(b.q);
}

static void tx_width(gconstpointer data)
{
    unsigned packed = GPOINTER_TO_UINT(data);
    unsigned width = packed >> 1;
    TestBus b = setup(packed & 1, true, NULL);
    write_reg(&b, 0xb0, 8 | (8 << 6) | (width << 12) | (width << 18));
    write_reg(&b, 0x20, read_reg(&b, 0x20) | (2 << 13));
    write_reg(&b, 0x00, 0x12345678);
    write_reg(&b, 0x00, 0x9abcdef0);
    write_reg(&b, 0x08, 1 << 4);
    uint32_t expected[] = {0x12345678, 0x9abcdef0};
    for (unsigned slot = 0; slot < 2; slot++) {
        uint32_t sample = 0;
        for (unsigned bit = 0; bit < width; bit++) {
            g_assert_cmpint(pin(&b, WS), ==, slot);
            qtest_clock_step(b.q, 250);
            sample = (sample << 1) | pin(&b, SD);
            qtest_clock_step(b.q, 250);
        }
        g_assert_cmphex(sample, ==, expected[slot] >> (32 - width));
    }
    qtest_quit(b.q);
}

static void tx_slave(gconstpointer data)
{
    unsigned port = GPOINTER_TO_UINT(data);
    TestBus b = setup(port, true, NULL);
    qtest_writel(b.q, GPIO + 0x130 + (port ? 24 : 23) * 4, CLK | 128);
    qtest_writel(b.q, GPIO + 0x130 + (port ? 26 : 25) * 4, WS | 128);
    external(&b, CLK, true);
    external(&b, WS, false);
    write_reg(&b, 0x00, 0x76548123);
    write_reg(&b, 0x08, (1 << 4) | (1 << 6));
    uint32_t received = 0;
    for (unsigned bit = 0; bit < 32; bit++) {
        external(&b, WS, bit >= 16);
        external(&b, CLK, false);
        qtest_clock_step(b.q, 250);
        external(&b, CLK, true);
        received = (received << 1) | pin(&b, SD);
        qtest_clock_step(b.q, 250);
    }
    g_assert_cmphex(received, ==, 0x76548123);
    qtest_quit(b.q);
}

static void cross_controller(void)
{
    TestBus tx = setup(0, true, NULL);
    TestBus rx = {.q = tx.q, .port = 1, .regs = base[1]};
    qtest_writel(tx.q, GPIO + 0x130 + 164 * 4, CLK | 128);
    qtest_writel(tx.q, GPIO + 0x130 + 165 * 4, WS | 128);
    qtest_writel(tx.q, GPIO + 0x130 + 181 * 4, SD | 128);
    write_reg(&rx, 0xac, (1 << 20) | 10);
    write_reg(&rx, 0xb0, 8 | (8 << 6) | (16 << 12) | (16 << 18));
    write_reg(&rx, 0x20, 32 | (32 << 6) | (1 << 19) | (1 << 20));
    write_reg(&rx, 0x08, (1 << 5) | (1 << 7));
    write_reg(&tx, 0x00, 0x3210fedc);
    write_reg(&tx, 0x08, 1 << 4);
    check_frame(&tx, 0x3210fedc, false);
    g_assert_cmphex(read_reg(&rx, 0x04), ==, 0x3210fedc);
    qtest_quit(tx.q);
}

static void duplex_run(unsigned port, const char *extra, unsigned pad_mode)
{
    TestBus b = setup(port, true, extra);
    unsigned rx_bck = port ? 164 : 27;
    unsigned rx_ws = port ? 165 : 28;
    unsigned din = port ? 181 : 155;
    unsigned dout = port ? 189 : 163;

    /* Route master clocks through physical pads; keep DIN separate from DOUT. */
    qtest_writel(b.q, GPIO + 0x130 + rx_bck * 4, CLK | 128);
    qtest_writel(b.q, GPIO + 0x130 + rx_ws * 4, WS | 128);
    qtest_writel(b.q, GPIO + 0x130 + din * 4, SD | 128);
    qtest_writel(b.q, MUX + 0x90, (2 << 12) | (1 << 9));
    qtest_writel(b.q, GPIO + 0x530 + SD * 4, 256);
    qtest_writel(b.q, GPIO + 0x530 + 24 * 4, dout);
    if (pad_mode == 1) {
        /* Open-drain clock pads release their high phase without a pull-up. */
        qtest_writel(b.q, GPIO + 0x88 + CLK * 4, 1 << 2);
        qtest_writel(b.q, GPIO + 0x88 + WS * 4, 1 << 2);
    } else if (pad_mode == 2) {
        /* Register OE is disabled: the logical clock cannot drive these pads. */
        qtest_writel(b.q, GPIO + 0x530 + CLK * 4,
                     (port ? 24 : 23) | (1 << 10));
        qtest_writel(b.q, GPIO + 0x530 + WS * 4,
                     (port ? 26 : 25) | (1 << 10));
    }
    write_reg(&b, 0xb0, 8 | (8 << 6) | (32 << 12) | (32 << 18));

    qtest_writel(b.q, DESC, (1u << 31) | 8);
    qtest_writel(b.q, DESC + 4, DATA);
    qtest_writel(b.q, DESC + 8, 0);
    write_reg(&b, 0x24, 2);
    write_reg(&b, 0x34, (DESC & 0xfffff) | (1 << 29));
    write_reg(&b, 0x20, read_reg(&b, 0x20) | (2 << 16));
    write_reg(&b, 0x20, read_reg(&b, 0x20) | (1 << 12));
    write_reg(&b, 0x00, 0x0123abcd);
    write_reg(&b, 0x08, (1 << 4) | (1 << 5) | (1 << 18));
    /* The 64th rising sample edge arrives at 31,750 ns. */
    if (pad_mode) {
        qtest_clock_step(b.q, 250);
        g_assert_false(pin(&b, CLK));
    }
    qtest_clock_step(b.q, pad_mode ? 31499 : 31749);
    g_assert_cmphex(qtest_readl(b.q, DESC) & (1u << 31), ==, 1u << 31);
    qtest_clock_step(b.q, 1);
    g_assert_cmphex(qtest_readl(b.q, DATA), ==, 0);
    g_assert_cmphex(qtest_readl(b.q, DATA + 4), ==, 0);
    g_assert_cmphex(qtest_readl(b.q, DESC) & (1u << 31), ==, 0);
    g_assert_cmphex(read_reg(&b, 0x0c) & (1 << 9), ==, 1 << 9);
    /* DIN is held low on GPIO23 while DOUT uses GPIO24. */
    g_assert_false(pin(&b, SD));
    qtest_quit(b.q);
}

static void duplex(gconstpointer data)
{
    duplex_run(GPOINTER_TO_UINT(data), NULL, 0);
}

static void duplex_unresolved_pad(gconstpointer data)
{
    unsigned port = GPOINTER_TO_UINT(data) & 1;
    unsigned pad_mode = GPOINTER_TO_UINT(data) >> 1;

    duplex_run(port, NULL, pad_mode);
}

static void duplex_trace(gconstpointer data)
{
    unsigned port = GPOINTER_TO_UINT(data);
    g_autofree char *name = NULL;
    g_autofree char *text = NULL;
    int fd = g_file_open_tmp("esp32-duplex-XXXXXX.vcd", &name, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    g_autofree char *args = g_strdup_printf(
        "-global driver=esp32.gpio,property=pin-trace,value=%s", name);

    duplex_run(port, args, 0);
    g_assert_true(g_file_get_contents(name, &text, NULL, NULL));
    g_auto(GStrv) lines = g_strsplit(text, "\n", -1);
    int64_t now = 0, previous = -1;
    unsigned edges = 0;
    for (unsigned i = 0; lines[i]; i++) {
        if (lines[i][0] == '#') {
            now = g_ascii_strtoll(lines[i] + 1, NULL, 10);
        } else if (strlen(lines[i]) == 4 && !strcmp(lines[i] + 1, "P18")) {
            g_assert_cmpint(now, >=, previous);
            previous = now;
            edges++;
        }
    }
    g_assert_cmpuint(edges, >, 100);
    unlink(name);
}

static void clock_gate_and_stop(gconstpointer data)
{
    unsigned port = GPOINTER_TO_UINT(data);
    unsigned gate = 1 << (port ? 21 : 4);
    TestBus b = setup(port, true, NULL);
    write_reg(&b, 0x00, 0xa55ac33c);
    write_reg(&b, 0xa0, 0x189);
    write_reg(&b, 0x08, 1 << 4);
    qtest_clock_step(b.q, 600); /* Pause after analytic edges have begun. */
    qtest_writel(b.q, DPORT + 0xc0, 0);
    qtest_clock_step(b.q, 10000);
    g_assert_false(pin(&b, CLK));
    qtest_writel(b.q, DPORT + 0xc0, gate);
    qtest_clock_step(b.q, 149);
    g_assert_false(pin(&b, CLK));
    qtest_clock_step(b.q, 1);
    g_assert_true(pin(&b, CLK));
    qtest_clock_step(b.q, 15750);
    g_assert_false(pin(&b, CLK));
    qtest_clock_step(b.q, 10000);
    g_assert_false(pin(&b, CLK));
    write_reg(&b, 0x00, 0xa55ac33c);
    qtest_clock_step(b.q, 250);
    g_assert_true(pin(&b, CLK));
    qtest_quit(b.q);
}

static const unsigned mux_offsets[] = {
    0x44, 0x88, 0x40, 0x84, 0x48, 0x6c, 0x60, 0x64,
    0x68, 0x54, 0x58, 0x5c, 0x34, 0x38, 0x30, 0x3c,
    0x4c, 0x50, 0x70, 0x74, 0x78, 0x7c, 0x80, 0x8c,
    0x90, 0x24, 0x28,
};

static void camera(gconstpointer data)
{
    unsigned port = GPOINTER_TO_UINT(data);
    TestBus b = setup(port, false, NULL);
    for (unsigned bit = 0; bit < 16; bit++) {
        unsigned pad = bit + 4;
        qtest_writel(b.q, MUX + mux_offsets[pad], (2 << 12) | (1 << 9));
        qtest_writel(b.q, GPIO + 0x130 + ((port ? 166 : 140) + bit) * 4, pad | 128);
    }
    qtest_writel(b.q, MUX + 0x7c, (2 << 12) | (1 << 9));
    qtest_writel(b.q, MUX + 0x80, (2 << 12) | (1 << 9));
    qtest_writel(b.q, GPIO + 0x130 + (port ? 165 : 28) * 4, 22 | 128);
    qtest_writel(b.q, GPIO + 0x130 + (port ? 193 : 190) * 4, 0x38 | 128);
    qtest_writel(b.q, GPIO + 0x130 + (port ? 194 : 191) * 4, 21 | 128);
    qtest_writel(b.q, GPIO + 0x130 + (port ? 195 : 192) * 4, 0x38 | 128);
    write_reg(&b, 0x20, read_reg(&b, 0x20) | (1 << 16));
    write_reg(&b, 0x2c, 1 << 3);
    write_reg(&b, 0xa8, (1 << 5) | 1);
    write_reg(&b, 0x08, (1 << 5) | (1 << 7));
    const uint16_t sample[] = {0x1111, 0x1234, 0xcafe};
    for (unsigned i = 0; i < G_N_ELEMENTS(sample); i++) {
        external(&b, 22, false);
        external(&b, 21, i != 0); /* VSYNC-low inhibits the first sample */
        for (unsigned bit = 0; bit < 16; bit++) {
            external(&b, bit + 4, (sample[i] >> bit) & 1);
        }
        qtest_clock_step(b.q, 1000);
        external(&b, 22, true);
        qtest_clock_step(b.q, 1000);
    }
    g_assert_cmphex(read_reg(&b, 0x04), ==, 0x1234cafe);
    qtest_quit(b.q);
}

static void lcd(gconstpointer data)
{
    unsigned packed = GPOINTER_TO_UINT(data);
    unsigned port = packed & 1;
    bool pairs = packed & 2;
    TestBus b = setup(port, true, NULL);
    const unsigned pads[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
                             12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25};
    for (unsigned bit = 0; bit < 24; bit++) {
        qtest_writel(b.q, MUX + mux_offsets[pads[bit]], (2 << 12) | (1 << 9));
        qtest_writel(b.q, GPIO + 0x530 + pads[bit] * 4, (port ? 166 : 140) + bit);
    }
    qtest_writel(b.q, MUX + mux_offsets[26], (2 << 12) | (1 << 9));
    qtest_writel(b.q, GPIO + 0x530 + 26 * 4, (port ? 26 : 25) | (1 << 9));
    write_reg(&b, 0xb0, 8 | (8 << 6) | (24 << 12) | (24 << 18));
    write_reg(&b, 0x20, read_reg(&b, 0x20) | (2 << 13));
    write_reg(&b, 0x00, 0x12345600);
    write_reg(&b, 0x00, 0x9abcde00);
    write_reg(&b, 0xa8, (1 << 5) | (1 << 1) | (pairs ? 1 << 2 : 0));
    write_reg(&b, 0x08, 1 << 4);
    const uint32_t samples[] = {0x123456, 0x9abcde};
    for (unsigned i = 0; i < 4; i++) {
        unsigned slot = pairs ? i % 2 : i / 2;
        uint32_t actual = 0;
        for (unsigned bit = 0; bit < 24; bit++) {
            actual |= pin(&b, pads[bit]) << bit;
        }
        g_assert_cmphex(actual, ==, samples[slot]);
        qtest_clock_step(b.q, 1000); /* LCD WS is BCLK / 2 */
    }
    qtest_quit(b.q);
}

static void pcm(gconstpointer data)
{
    TestBus b = setup(GPOINTER_TO_UINT(data), true, NULL);
    write_reg(&b, 0xb0, 8 | (8 << 6) | (8 << 12) | (8 << 18));
    write_reg(&b, 0x00, 0x0000ffff); /* +0 and -1 -> A-law d5 and 55 */
    write_reg(&b, 0xa0, 0x81); /* TX compress, RX bypass */
    write_reg(&b, 0x08, 1 << 4);
    unsigned word = 0;
    for (unsigned bit = 0; bit < 16; bit++) {
        qtest_clock_step(b.q, 250);
        word = (word << 1) | pin(&b, SD);
        qtest_clock_step(b.q, 250);
    }
    g_assert_cmphex(word, ==, 0xd555);
    qtest_quit(b.q);
}

static void short_sync(gconstpointer data)
{
    unsigned port = GPOINTER_TO_UINT(data);
    TestBus b = setup(port, true, NULL);
    qtest_writel(b.q, GPIO + 0x130 + (port ? 181 : 155) * 4, SD | 128);
    write_reg(&b, 0x00, 0x1234abcd);
    write_reg(&b, 0x08, (1 << 4) | (1 << 5) | (1 << 7) | (1 << 10) |
                        (1 << 11) | (1 << 12) | (1 << 13) | (1 << 18));
    g_assert_true(pin(&b, WS)); /* one BCLK sync precedes first PCM data */
    qtest_clock_step(b.q, 500);
    uint32_t decoded = 0;
    for (unsigned bit = 0; bit < 32; bit++) {
        g_assert_cmpint(pin(&b, WS), ==, bit == 31);
        qtest_clock_step(b.q, 250);
        decoded = (decoded << 1) | pin(&b, SD);
        qtest_clock_step(b.q, 250);
    }
    g_assert_cmphex(decoded, ==, 0x1234abcd);
    g_assert_cmphex(read_reg(&b, 0x04), ==, 0x1234abcd);
    qtest_quit(b.q);
}

static void mono_packing(gconstpointer data)
{
    unsigned port = GPOINTER_TO_UINT(data);
    TestBus b = setup(port, true, NULL);
    qtest_writel(b.q, GPIO + 0x130 + (port ? 181 : 155) * 4, SD | 128);
    write_reg(&b, 0x20, read_reg(&b, 0x20) | (1 << 13) | (1 << 16));
    write_reg(&b, 0x2c, 1 | (1 << 3));
    write_reg(&b, 0x00, 0x1234abcd);
    write_reg(&b, 0x08, (1 << 4) | (1 << 5) | (1 << 7) | (1 << 18));
    check_frame(&b, 0x12341234, false);
    check_frame(&b, 0xabcdabcd, false);
    g_assert_cmphex(read_reg(&b, 0x04), ==, 0x1234abcd);
    qtest_quit(b.q);
}

static void pcm_decompress(gconstpointer data)
{
    TestBus b = setup(GPOINTER_TO_UINT(data), false, NULL);
    write_reg(&b, 0xb0, 8 | (8 << 6) | (8 << 12) | (8 << 18));
    write_reg(&b, 0xa0, 9); /* RX decompress, TX bypass */
    write_reg(&b, 0x08, (1 << 5) | (1 << 7));
    for (unsigned bit = 0; bit < 16; bit++) {
        external(&b, CLK, false);
        external(&b, WS, bit >= 8);
        external(&b, SD, (0xd555 >> (15 - bit)) & 1);
        qtest_clock_step(b.q, 250);
        external(&b, CLK, true);
        qtest_clock_step(b.q, 250);
    }
    g_assert_cmphex(read_reg(&b, 0x04), ==, 0x0008fff8); /* G.711 d5=+8,55=-8 */
    qtest_quit(b.q);
}

static void mclk_trace(gconstpointer data)
{
    unsigned port = GPOINTER_TO_UINT(data);
    g_autofree char *name = NULL;
    g_autofree char *text = NULL;
    int fd = g_file_open_tmp("esp32-mclk-XXXXXX.vcd", &name, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    g_autofree char *args = g_strdup_printf("-global driver=esp32.gpio,property=pin-trace,value=%s", name);
    TestBus b = setup(port, true, args);
    qtest_writel(b.q, MUX, port ? 15 : 0);
    qtest_writel(b.q, MUX + 0x44, 1 << 12); /* GPIO0 CLK_OUT1 direct IOMUX */
    qtest_clock_step(b.q, 1000);
    qtest_quit(b.q);
    g_assert_true(g_file_get_contents(name, &text, NULL, NULL));
    g_auto(GStrv) lines = g_strsplit(text, "\n", -1);
    int64_t now = 0, last = -1;
    unsigned count = 0;
    char previous = 'x';
    for (unsigned i = 0; lines[i]; i++) {
        if (lines[i][0] == '#') {
            now = g_ascii_strtoll(lines[i] + 1, NULL, 10);
        } else if (strlen(lines[i]) == 3 && !strcmp(lines[i] + 1, "P0")) {
            char value = lines[i][0];
            if (value == '1' && previous == '0') {
                if (last >= 0) {
                    g_assert_cmpint(now - last, >=, 62);
                    g_assert_cmpint(now - last, <=, 63);
                }
                last = now;
                count++;
            }
            previous = value;
        }
    }
    g_assert_cmpint(count, ==, 16);
    unlink(name);
}

static void mclk_lazy_read(gconstpointer data)
{
    unsigned port = GPOINTER_TO_UINT(data);
    TestBus b = setup(port, true, NULL);

    qtest_writel(b.q, MUX, port ? 15 : 0);
    qtest_writel(b.q, MUX + 0x44, (1 << 12) | (1 << 9));
    g_assert_false(pin(&b, 0));
    qtest_clock_step(b.q, 30);
    g_assert_false(pin(&b, 0));
    qtest_clock_step(b.q, 1);
    g_assert_true(pin(&b, 0));
    qtest_clock_step(b.q, 31);
    g_assert_false(pin(&b, 0));
    qtest_clock_step(b.q, 31);
    g_assert_true(pin(&b, 0));
    qtest_clock_step(b.q, 31);
    g_assert_true(pin(&b, 0));
    qtest_clock_step(b.q, 1);
    g_assert_false(pin(&b, 0));
    qtest_quit(b.q);
}

static void dma_chain(gconstpointer data)
{
    TestBus b = setup(GPOINTER_TO_UINT(data), true, NULL);
    const uint32_t desc2 = DESC + 16;
    qtest_writel(b.q, DESC, (1u << 31) | (1 << 30) | (4 << 12) | 4);
    qtest_writel(b.q, DESC + 4, DATA);
    qtest_writel(b.q, DESC + 8, desc2);
    qtest_writel(b.q, desc2, (1u << 31) | (1 << 30) | (4 << 12) | 4);
    qtest_writel(b.q, desc2 + 4, DATA + 4);
    qtest_writel(b.q, desc2 + 8, 0);
    qtest_writel(b.q, DATA, 0xa55a1234);
    qtest_writel(b.q, DATA + 4, 0x43215aa5);
    write_reg(&b, 0x60, (1 << 12) | (1 << 6)); /* EOF at AHB -> FIFO */
    write_reg(&b, 0x20, read_reg(&b, 0x20) | (1 << 12));
    write_reg(&b, 0x30, (DESC & 0xfffff) | (1 << 29));
    g_assert_cmphex(read_reg(&b, 0x30) & (1 << 29), ==, 0);
    g_assert_cmphex(read_reg(&b, 0x38), ==, desc2);
    g_assert_cmphex(read_reg(&b, 0x40), ==, desc2);
    g_assert_cmphex(read_reg(&b, 0x54), ==, desc2);
    g_assert_cmphex(read_reg(&b, 0x0c) & (1 << 12), ==, 1 << 12);
    g_assert_cmphex(read_reg(&b, 0x0c) & ((1 << 11) | (1 << 16)), ==, 0);
    g_assert_cmphex(qtest_readl(b.q, DESC) & (1u << 31), ==, 1u << 31);
    write_reg(&b, 0x18, UINT32_MAX);
    write_reg(&b, 0x08, 1 << 4);
    g_assert_cmphex(qtest_readl(b.q, DESC) & (1u << 31), ==, 0);
    check_frame(&b, 0xa55a1234, false);
    check_frame(&b, 0x43215aa5, false);
    g_assert_cmphex(qtest_readl(b.q, desc2) & (1u << 31), ==, 0);
    g_assert_cmphex(read_reg(&b, 0x0c) & (1 << 16), ==, 1 << 16);
    qtest_quit(b.q);
}

static void dma_receive_chain(gconstpointer data)
{
    TestBus b = setup(GPOINTER_TO_UINT(data), false, NULL);
    for (unsigned i = 0; i < 3; i++) {
        qtest_writel(b.q, DESC + 16 * i, (1u << 31) | (i == 2 ? 8 : 4));
        qtest_writel(b.q, DESC + 16 * i + 4, DATA + 8 * i);
        qtest_writel(b.q, DESC + 16 * i + 8, i == 2 ? 0 : DESC + 16 * (i + 1));
    }
    write_reg(&b, 0x60, 1 << 12);
    write_reg(&b, 0x24, 3);
    write_reg(&b, 0x20, read_reg(&b, 0x20) | (1 << 12));
    write_reg(&b, 0x34, (DESC & 0xfffff) | (1 << 29));
    write_reg(&b, 0x08, (1 << 5) | (1 << 7));
    const uint32_t words[] = {0x1234abcd, 0x7654cafe, 0x01239876};
    for (unsigned i = 0; i < G_N_ELEMENTS(words); i++) {
        inject_frame(&b, words[i]);
        g_assert_cmphex(qtest_readl(b.q, DATA + 8 * i), ==, words[i]);
        uint32_t header = (4 << 12) | (i == 2 ? (1 << 30) | 8 : 4);
        g_assert_cmphex(qtest_readl(b.q, DESC + 16 * i), ==, header);
    }
    g_assert_cmphex(read_reg(&b, 0x3c), ==, DESC + 32);
    g_assert_cmphex(read_reg(&b, 0x0c) & ((1 << 8) | (1 << 9) | (1 << 15)), ==,
                    (1 << 8) | (1 << 9) | (1 << 15));
    qtest_quit(b.q);
}

static void dma_errors(gconstpointer data)
{
    TestBus b = setup(GPOINTER_TO_UINT(data), true, NULL);
    const uint32_t headers[] = {
        (1u << 31) | (4 << 12) | 6, /* non-word buffer size */
        (1u << 31) | (2 << 12) | 4, /* non-word transfer */
        (1u << 31) | (8 << 12) | 4, /* transfer exceeds size */
        (4 << 12) | 4,              /* owner CPU with checking enabled */
    };
    write_reg(&b, 0x60, (1 << 12) | (1 << 8));
    write_reg(&b, 0x20, read_reg(&b, 0x20) | (1 << 12));
    qtest_writel(b.q, DESC + 4, DATA);
    qtest_writel(b.q, DESC + 8, 0);
    for (unsigned i = 0; i < G_N_ELEMENTS(headers); i++) {
        qtest_writel(b.q, DESC, headers[i]);
        write_reg(&b, 0x18, UINT32_MAX);
        write_reg(&b, 0x30, (DESC & 0xfffff) | (1 << 29));
        g_assert_cmphex(read_reg(&b, 0x0c) & (1 << 14), ==, 1 << 14);
        g_assert_cmphex(read_reg(&b, 0x0c) & ((1 << 11) | (1 << 12) | (1 << 16)), ==, 0);
    }
    write_reg(&b, 0x18, UINT32_MAX);
    write_reg(&b, 0x30, ((DESC + 1) & 0xfffff) | (1 << 29));
    g_assert_cmphex(read_reg(&b, 0x0c) & (1 << 14), ==, 1 << 14);
    qtest_writel(b.q, DESC, (1u << 31) | (8 << 12) | 8);
    qtest_writel(b.q, DESC + 4, 0x3ffffffc); /* buffer crosses DMA RAM end */
    write_reg(&b, 0x18, UINT32_MAX);
    write_reg(&b, 0x30, (DESC & 0xfffff) | (1 << 29));
    g_assert_cmphex(read_reg(&b, 0x0c) & (1 << 14), ==, 1 << 14);
    qtest_quit(b.q);
}

static void fifo_bounds(gconstpointer data)
{
    TestBus b = setup(GPOINTER_TO_UINT(data), true, NULL);
    for (unsigned i = 1; i <= 64; i++) {
        write_reg(&b, 0, (i << 16) | i);
    }
    g_assert_cmphex(read_reg(&b, 0x0c) & (1 << 4), ==, 1 << 4);
    write_reg(&b, 0, 0xdeadbeef);
    write_reg(&b, 0x08, 1 << 4);
    for (unsigned i = 1; i <= 64; i++) {
        check_frame(&b, (i << 16) | i, false);
    }
    check_frame(&b, 0x00400040, false); /* full write cannot corrupt contents */
    write_reg(&b, 0x08, 1 << 2); /* FIFO reset held */
    write_reg(&b, 0, 0xdeadbeef);
    write_reg(&b, 0x08, 0);
    write_reg(&b, 0, 0x01234567);
    write_reg(&b, 0x08, 1 << 4);
    check_frame(&b, 0x01234567, false);
    qtest_quit(b.q);
}

static void fractional_clock(gconstpointer data)
{
    TestBus b = setup(GPOINTER_TO_UINT(data), true, NULL);
    write_reg(&b, 0xac, (1 << 20) | (3 << 14) | (1 << 8) | 10);
    write_reg(&b, 0, 0xa55ac33c);
    write_reg(&b, 0x08, 1 << 4);
    /* 160MHz / (10+1/3) / 8 => half period 775/3 ns.
     * Verify 96 edges against cumulative rational deadlines, not rounding
     * every interval to the same integer and accumulating drift. */
    int64_t previous = 0;
    for (unsigned edge = 1; edge <= 96; edge++) {
        int64_t deadline = (int64_t)edge * 775 / 3;
        qtest_clock_step(b.q, deadline - previous - 1);
        g_assert_cmpint(pin(&b, CLK), ==, !(edge & 1));
        qtest_clock_step(b.q, 1);
        g_assert_cmpint(pin(&b, CLK), ==, edge & 1);
        previous = deadline;
    }
    qtest_quit(b.q);
}

static void receive_master(gconstpointer data)
{
    unsigned port = GPOINTER_TO_UINT(data);
    TestBus b = setup(port, false, NULL);
    qtest_writel(b.q, GPIO + 0x530 + CLK * 4, port ? 164 : 27);
    qtest_writel(b.q, GPIO + 0x530 + WS * 4, port ? 165 : 28);
    write_reg(&b, 0x08, 1 << 5);
    const uint32_t samples[] = {0xa55ac33c, 0x1234cafe};
    for (unsigned i = 0; i < G_N_ELEMENTS(samples); i++) {
        for (unsigned bit = 0; bit < 32; bit++) {
            g_assert_cmpint(pin(&b, WS), ==, bit >= 16);
            external(&b, SD, (samples[i] >> (31 - bit)) & 1);
            qtest_clock_step(b.q, 250);
            g_assert_true(pin(&b, CLK));
            qtest_clock_step(b.q, 250);
            g_assert_false(pin(&b, CLK));
        }
        g_assert_cmphex(read_reg(&b, 0x04), ==, samples[i]);
    }
    qtest_quit(b.q);
}

/*
 * In analytic mode RX retires the frame at its final sample edge, keeping
 * DMA ownership and EOF timing aligned with the bit-clock waveform.
 */
static void rx_analytic_dma_boundary(gconstpointer data)
{
    TestBus b = setup(GPOINTER_TO_UINT(data), false, NULL);

    qtest_writel(b.q, DESC, (1u << 31) | (1u << 30) | (4 << 12) | 4);
    qtest_writel(b.q, DESC + 4, DATA);
    qtest_writel(b.q, DESC + 8, 0);
    write_reg(&b, 0x24, 1);
    write_reg(&b, 0x20, read_reg(&b, 0x20) | (1 << 12));
    write_reg(&b, 0x34, (DESC & 0xfffff) | (1 << 29));
    write_reg(&b, 0x08, 1 << 5);

    /*
     * 32-bit stereo at 16 MHz / 8 gives 250 ns per half-cycle. The first
     * rising edge is at 250 ns; the frame's final sample edge is 15,750 ns.
     */
    qtest_clock_step(b.q, 15500);
    g_assert_cmphex(qtest_readl(b.q, DESC), ==,
                    (1u << 31) | (1u << 30) | (4 << 12) | 4);
    g_assert_cmphex(read_reg(&b, 0x0c) & (1 << 9), ==, 0);
    qtest_clock_step(b.q, 250);
    g_assert_cmphex(qtest_readl(b.q, DESC), ==,
                    (1u << 30) | (4 << 12) | 4);
    g_assert_cmphex(read_reg(&b, 0x0c) & (1 << 9), ==, 1 << 9);
    g_assert_cmphex(qtest_readl(b.q, DATA), ==, 0);
    g_assert_cmphex(qtest_readl(b.q, DATA + 4), ==, 0);
    qtest_quit(b.q);
}

static void channel_selection(gconstpointer data)
{
    TestBus b = setup(GPOINTER_TO_UINT(data), true, NULL);
    for (unsigned msb_right = 0; msb_right <= 1; msb_right++) {
        for (unsigned mode = 0; mode <= 4; mode++) {
            for (unsigned right_first = 0; right_first <= 1; right_first++) {
                write_reg(&b, 0x08, 1 << 2);
                write_reg(&b, 0x08, 0);
                write_reg(&b, 0x2c, mode);
                write_reg(&b, 0x28, 0xabcd0000);
                write_reg(&b, 0, 0x12345678);
                write_reg(&b, 0x08, (1 << 4) | (msb_right << 16) | (right_first << 8));
                uint16_t left = msb_right ? 0x5678 : 0x1234;
                uint16_t right = msb_right ? 0x1234 : 0x5678;
                switch (mode) {
                case 1:
                    left = right = 0x1234;
                    break;
                case 2:
                    left = right = 0x5678;
                    break;
                case 3:
                    if (msb_right) {
                        right = 0xabcd;
                    } else {
                        left = 0xabcd;
                    }
                    break;
                case 4:
                    if (msb_right) {
                        left = 0xabcd;
                    } else {
                        right = 0xabcd;
                    }
                    break;
                }
                uint32_t actual = 0;
                for (unsigned bit = 0; bit < 32; bit++) {
                    g_assert_cmpint(pin(&b, WS), ==, (bit >= 16) ^ right_first);
                    qtest_clock_step(b.q, 250);
                    actual = (actual << 1) | pin(&b, SD);
                    qtest_clock_step(b.q, 250);
                }
                uint32_t expected = right_first ? ((uint32_t)right << 16) | left :
                                                 ((uint32_t)left << 16) | right;
                g_assert_cmphex(actual, ==, expected);
            }
        }
    }
    qtest_quit(b.q);
}

static void unsupported_mode(gconstpointer data)
{
    g_autofree char *name = NULL;
    g_autofree char *text = NULL;
    int fd = g_file_open_tmp("esp32-unmodeled-XXXXXX.vcd", &name, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    g_autofree char *args = g_strdup_printf("-global driver=esp32.gpio,property=pin-trace,value=%s", name);
    TestBus b = setup(GPOINTER_TO_UINT(data), true, args);
    qtest_writel(b.q, DESC, (1u << 31) | (1 << 30) | (4 << 12) | 4);
    qtest_writel(b.q, DESC + 4, DATA);
    qtest_writel(b.q, DESC + 8, 0);
    qtest_writel(b.q, DATA, 0x12345678);
    write_reg(&b, 0xb4, read_reg(&b, 0xb4) | 1); /* unimplemented PDM TX */
    write_reg(&b, 0x20, read_reg(&b, 0x20) | (1 << 12));
    write_reg(&b, 0x30, (DESC & 0xfffff) | (1 << 29));
    write_reg(&b, 0x08, 1 << 4);
    qtest_clock_step(b.q, 10000);
    g_assert_cmphex(read_reg(&b, 0x0c) & ((1 << 11) | (1 << 12) | (1 << 16)), ==, 0);
    g_assert_cmphex(qtest_readl(b.q, DESC) & (1u << 31), ==, 1u << 31);
    write_reg(&b, 0xb4, read_reg(&b, 0xb4) & ~1u);
    check_frame(&b, 0x12345678, false);
    qtest_quit(b.q);
    g_assert_true(g_file_get_contents(name, &text, NULL, NULL));
    g_assert_nonnull(strstr(text, "xP18\nxD18"));
    g_assert_nonnull(strstr(text, "xP19\nxD19"));
    g_assert_nonnull(strstr(text, "xP23\nxD23"));
    unlink(name);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (unsigned port = 0; port < 2; port++) {
        g_autofree char *msb = g_strdup_printf("/esp32/i2s%u/tx-msb", port);
        g_autofree char *philips = g_strdup_printf("/esp32/i2s%u/tx-philips", port);
        g_autofree char *rx = g_strdup_printf("/esp32/i2s%u/rx-slave", port);
        g_autofree char *txdma = g_strdup_printf("/esp32/i2s%u/tx-dma-irq-reset", port);
        g_autofree char *rxdma = g_strdup_printf("/esp32/i2s%u/rx-dma", port);
        g_test_add_data_func(msb, GUINT_TO_POINTER(port), tx_format);
        g_test_add_data_func(philips, GUINT_TO_POINTER(port | 2), tx_format);
        g_test_add_data_func(rx, GUINT_TO_POINTER(port), rx_slave);
        g_test_add_data_func(txdma, GUINT_TO_POINTER(port), dma_tx);
        g_test_add_data_func(rxdma, GUINT_TO_POINTER(port), dma_rx);
        g_autofree char *slave = g_strdup_printf("/esp32/i2s%u/tx-slave", port);
        g_autofree char *duplex_name = g_strdup_printf("/esp32/i2s%u/full-duplex", port);
        g_autofree char *duplex_trace_name = g_strdup_printf(
            "/esp32/i2s%u/full-duplex-trace", port);
        g_autofree char *duplex_od_name = g_strdup_printf(
            "/esp32/i2s%u/full-duplex-open-drain", port);
        g_autofree char *duplex_oe_name = g_strdup_printf(
            "/esp32/i2s%u/full-duplex-register-oe", port);
        g_autofree char *gate = g_strdup_printf("/esp32/i2s%u/clock-gate-stop-resume", port);
        g_test_add_data_func(slave, GUINT_TO_POINTER(port), tx_slave);
        g_test_add_data_func(duplex_name, GUINT_TO_POINTER(port), duplex);
        g_test_add_data_func(duplex_trace_name, GUINT_TO_POINTER(port),
                             duplex_trace);
        g_test_add_data_func(duplex_od_name, GUINT_TO_POINTER((1 << 1) | port),
                             duplex_unresolved_pad);
        g_test_add_data_func(duplex_oe_name, GUINT_TO_POINTER((2 << 1) | port),
                             duplex_unresolved_pad);
        g_test_add_data_func(gate, GUINT_TO_POINTER(port), clock_gate_and_stop);
        g_autofree char *camera_name = g_strdup_printf("/esp32/i2s%u/camera", port);
        g_autofree char *lcd1 = g_strdup_printf("/esp32/i2s%u/lcd-form1", port);
        g_autofree char *lcd2 = g_strdup_printf("/esp32/i2s%u/lcd-form2", port);
        g_autofree char *pcm_name = g_strdup_printf("/esp32/i2s%u/pcm-alaw", port);
        g_test_add_data_func(camera_name, GUINT_TO_POINTER(port), camera);
        g_test_add_data_func(lcd1, GUINT_TO_POINTER(port), lcd);
        g_test_add_data_func(lcd2, GUINT_TO_POINTER(port | 2), lcd);
        g_test_add_data_func(pcm_name, GUINT_TO_POINTER(port), pcm);
        g_autofree char *sync_name = g_strdup_printf("/esp32/i2s%u/pcm-short-sync", port);
        g_test_add_data_func(sync_name, GUINT_TO_POINTER(port), short_sync);
        g_autofree char *mono_name = g_strdup_printf("/esp32/i2s%u/mono-fifo-packing", port);
        g_autofree char *decompress = g_strdup_printf("/esp32/i2s%u/pcm-decompress", port);
        g_autofree char *mclk = g_strdup_printf("/esp32/i2s%u/mclk-physical-vcd", port);
        g_autofree char *mclk_lazy = g_strdup_printf(
            "/esp32/i2s%u/mclk-lazy-level", port);
        g_test_add_data_func(mono_name, GUINT_TO_POINTER(port), mono_packing);
        g_test_add_data_func(decompress, GUINT_TO_POINTER(port), pcm_decompress);
        g_test_add_data_func(mclk, GUINT_TO_POINTER(port), mclk_trace);
        g_test_add_data_func(mclk_lazy, GUINT_TO_POINTER(port), mclk_lazy_read);
        g_autofree char *chain = g_strdup_printf("/esp32/i2s%u/dma-chain-eof", port);
        g_autofree char *rxchain = g_strdup_printf("/esp32/i2s%u/rx-dma-chain", port);
        g_autofree char *errors = g_strdup_printf("/esp32/i2s%u/dma-errors", port);
        g_autofree char *bounds = g_strdup_printf("/esp32/i2s%u/fifo-bounds-reset", port);
        g_autofree char *fractional = g_strdup_printf("/esp32/i2s%u/fractional-clock", port);
        g_test_add_data_func(chain, GUINT_TO_POINTER(port), dma_chain);
        g_test_add_data_func(rxchain, GUINT_TO_POINTER(port), dma_receive_chain);
        g_test_add_data_func(errors, GUINT_TO_POINTER(port), dma_errors);
        g_test_add_data_func(bounds, GUINT_TO_POINTER(port), fifo_bounds);
        g_test_add_data_func(fractional, GUINT_TO_POINTER(port), fractional_clock);
        g_autofree char *master = g_strdup_printf("/esp32/i2s%u/rx-master", port);
        g_autofree char *rxboundary = g_strdup_printf(
            "/esp32/i2s%u/rx-analytic-dma-boundary", port);
        g_autofree char *channels = g_strdup_printf("/esp32/i2s%u/channel-selection", port);
        g_test_add_data_func(master, GUINT_TO_POINTER(port), receive_master);
        g_test_add_data_func(rxboundary, GUINT_TO_POINTER(port),
                             rx_analytic_dma_boundary);
        g_test_add_data_func(channels, GUINT_TO_POINTER(port), channel_selection);
        g_autofree char *unmodeled = g_strdup_printf("/esp32/i2s%u/unmodeled-is-unknown", port);
        g_test_add_data_func(unmodeled, GUINT_TO_POINTER(port), unsupported_mode);
        const unsigned widths[] = {8, 16, 24, 32};
        for (unsigned i = 0; i < G_N_ELEMENTS(widths); i++) {
            g_autofree char *name = g_strdup_printf("/esp32/i2s%u/width-%u", port, widths[i]);
            g_test_add_data_func(name, GUINT_TO_POINTER((widths[i] << 1) | port), tx_width);
        }
    }
    g_test_add_func("/esp32/cross-controller-pads", cross_controller);
    return g_test_run();
}
