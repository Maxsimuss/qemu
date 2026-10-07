/* ESP32 I2C MMIO and physical bus tests.
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Register addresses/timing: ESP32 TRM v5.8 chapter 21.
 * External peer below changes only resolved GPIO-pad drives, never I2C regs.
 */
#include "qemu/osdep.h"
#include "libqtest.h"
#include <glib/gstdio.h>

#define GPIO UINT64_C(0x3ff44000)
#define MUX UINT64_C(0x3ff49000)
#define DPORT UINT64_C(0x3ff00000)
#define PAD_PATH "/machine/soc/gpio"
#define SDA 21
#define SCL 22
#define HIGH 1
#define LOW 0
#define RELEASE 2
static const uint64_t controller[] = {0x3ff53000, 0x3ff67000};

typedef struct Bus {
    QTestState *q;
    unsigned port;
    uint64_t base;
} Bus;

static void external(Bus *b, unsigned pad, int level)
{
    qtest_set_irq_in(b->q, PAD_PATH, "pad-drive", pad, level);
}

static bool pin(Bus *b, unsigned pad)
{
    return (qtest_readl(b->q, GPIO + 0x3c) >> pad) & 1;
}

static void reg(Bus *b, unsigned offset, uint32_t value)
{
    qtest_writel(b->q, b->base + offset, value);
}

static uint32_t rd(Bus *b, unsigned offset)
{
    return qtest_readl(b->q, b->base + offset);
}

static Bus bus_fixture(unsigned port, bool as_master, const char *fixture)
{
    Bus b = {.port = port, .base = controller[port]};
    unsigned clock = port ? 95 : 29, data = clock + 1;
    b.q = qtest_initf("-machine esp32 -display none -serial none -nic none %s", fixture);
    qtest_writel(b.q, DPORT + 0xc0, (1 << 7) | (1 << 18));
    qtest_writel(b.q, DPORT + 0xc4, 0);
    qtest_writel(b.q, MUX + 0x7c, (2 << 12) | (1 << 9) | (1 << 8));
    qtest_writel(b.q, MUX + 0x80, (2 << 12) | (1 << 9) | (1 << 8));
    qtest_writel(b.q, GPIO + 0x530 + SDA * 4, data);
    qtest_writel(b.q, GPIO + 0x530 + SCL * 4, clock);
    qtest_writel(b.q, GPIO + 0x130 + data * 4, SDA | (1 << 7));
    qtest_writel(b.q, GPIO + 0x130 + clock * 4, SCL | (1 << 7));
    reg(&b, 4, as_master ? 0x13 : 3);
    reg(&b, 0xc, 50000);
    reg(&b, 0, 19);
    reg(&b, 0x38, 12);
    reg(&b, 0x30, 2);
    reg(&b, 0x34, 2);
    reg(&b, 0x40, 10);
    reg(&b, 0x44, 10);
    reg(&b, 0x48, 10);
    reg(&b, 0x4c, 10);
    reg(&b, 0x50, 0);
    reg(&b, 0x54, 0);
    external(&b, SDA, RELEASE);
    external(&b, SCL, RELEASE);
    qtest_clock_step(b.q, 2000);
    g_assert_true(pin(&b, SDA));
    g_assert_true(pin(&b, SCL));
    return b;
}

static Bus bus_new(unsigned port, bool as_master)
{
    return bus_fixture(port, as_master, "");
}

static void wait_complete(Bus *b)
{
    for (unsigned i = 0; i < 10000 && !(rd(b, 0x20) & (1 << 7)); i++) {
        qtest_clock_step(b->q, 100);
    }
    g_assert_true(rd(b, 0x20) & (1 << 7));
    g_assert_false(rd(b, 0x20) & ((1 << 5) | (1 << 8) | (1 << 10)));
}

static void bitbang_start(Bus *b)
{
    external(b, SDA, RELEASE);
    external(b, SCL, RELEASE);
    qtest_clock_step(b->q, 1000);
    external(b, SDA, LOW);
    qtest_clock_step(b->q, 1000);
    external(b, SCL, LOW);
}

static void bitbang_stop(Bus *b)
{
    external(b, SCL, LOW);
    external(b, SDA, LOW);
    qtest_clock_step(b->q, 1000);
    external(b, SCL, RELEASE);
    qtest_clock_step(b->q, 1000);
    external(b, SDA, RELEASE);
    qtest_clock_step(b->q, 1000);
}

static bool bitbang_write(Bus *b, uint8_t byte)
{
    for (unsigned i = 0; i < 8; i++) {
        external(b, SDA, byte & (0x80 >> i) ? RELEASE : LOW);
        qtest_clock_step(b->q, 500);
        external(b, SCL, RELEASE);
        qtest_clock_step(b->q, 1000);
        external(b, SCL, LOW);
        qtest_clock_step(b->q, 500);
    }
    external(b, SDA, RELEASE);
    qtest_clock_step(b->q, 500);
    external(b, SCL, RELEASE);
    qtest_clock_step(b->q, 1000);
    bool ack = !pin(b, SDA);
    external(b, SCL, LOW);
    qtest_clock_step(b->q, 500);
    return ack;
}

static uint8_t bitbang_read(Bus *b, bool nack)
{
    uint8_t byte = 0;
    external(b, SDA, RELEASE);
    for (unsigned i = 0; i < 8; i++) {
        qtest_clock_step(b->q, 500);
        external(b, SCL, RELEASE);
        qtest_clock_step(b->q, 1000);
        byte = (byte << 1) | pin(b, SDA);
        external(b, SCL, LOW);
        qtest_clock_step(b->q, 500);
    }
    external(b, SDA, nack ? RELEASE : LOW);
    qtest_clock_step(b->q, 500);
    external(b, SCL, RELEASE);
    qtest_clock_step(b->q, 1000);
    external(b, SCL, LOW);
    qtest_clock_step(b->q, 500);
    external(b, SDA, RELEASE);
    return byte;
}

static void slave_fifo(void)
{
    for (unsigned p = 0; p < 2; p++) {
        Bus b = bus_new(p, false);
        reg(&b, 0x10, 0x50);
        reg(&b, 0x18, (3 << 5) | 1);
        reg(&b, 0x28, 0x1fff);
        g_assert_true(rd(&b, 0x2c) & (1 << 1));
        bitbang_start(&b);
        g_assert_true(bitbang_write(&b, 0xa0));
        g_assert_true(bitbang_write(&b, 0x12));
        g_assert_true(bitbang_write(&b, 0x34));
        g_assert_cmpuint((rd(&b, 8) >> 8) & 63, ==, 2);
        g_assert_true(rd(&b, 0x20) & 1);
        g_assert_cmphex(rd(&b, 0x1c), ==, 0x12);
        g_assert_false(rd(&b, 0x20) & 1);
        g_assert_cmphex(rd(&b, 0x1c), ==, 0x34);
        bitbang_stop(&b);
        g_assert_true(rd(&b, 0x20) & (1 << 7));
        reg(&b, 0x24, 0x1fff);
        g_assert_cmphex(rd(&b, 0x20), ==, 1 << 1);
        /* Wrong address must be NACK, leaving the RX FIFO untouched. */
        bitbang_start(&b);
        g_assert_false(bitbang_write(&b, 0xa2));
        bitbang_stop(&b);
        g_assert_cmpuint((rd(&b, 8) >> 8) & 63, ==, 0);
        reg(&b, 0x1c, 0xc3);
        reg(&b, 0x1c, 0x5a);
        bitbang_start(&b);
        g_assert_true(bitbang_write(&b, 0xa1));
        g_assert_cmphex(bitbang_read(&b, false), ==, 0xc3);
        g_assert_cmphex(bitbang_read(&b, true), ==, 0x5a);
        bitbang_stop(&b);
        g_assert_true(rd(&b, 8) & 1); /* last ACK sampled from master is NACK */
        g_assert_false(rd(&b, 0x20) & (1 << 10));
        qtest_quit(b.q);
    }
}

static void slave_nonfifo_10bit(void)
{
    Bus b = bus_new(1, false);
    /* Address 0x2ab encoded by original ESP32 HW: 11110 A9 A8, A7..A0. */
    reg(&b, 0x10, 0x80000000 | (0xab << 7) | 0x7a);
    reg(&b, 0x18, (1 << 10) | (1 << 11) | (2 << 14) | (2 << 20));
    bitbang_start(&b);
    g_assert_true(bitbang_write(&b, 0xf4));
    g_assert_true(bitbang_write(&b, 0xab));
    g_assert_true(bitbang_write(&b, 4));
    g_assert_true(bitbang_write(&b, 0xbe));
    g_assert_true(bitbang_write(&b, 0xef));
    g_assert_cmphex(rd(&b, 0x110), ==, 0xbe);
    g_assert_cmphex(rd(&b, 0x114), ==, 0xef);
    g_assert_true(rd(&b, 0x20) & (1 << 11));
    /* A 10-bit repeated START read uses just the first address byte. */
    bitbang_start(&b);
    g_assert_true(bitbang_write(&b, 0xf5));
    g_assert_cmphex(bitbang_read(&b, false), ==, 0xbe);
    g_assert_cmphex(bitbang_read(&b, true), ==, 0xef);
    bitbang_stop(&b);
    qtest_quit(b.q);
}

static void master_commands(Bus *b, uint8_t byte, unsigned end)
{
    reg(b, 0x1c, byte);
    reg(b, 0x58, 0);
    reg(b, 0x5c, (1 << 11) | (1 << 8) | 1);
    reg(b, 0x60, end << 11);
    reg(b, 0x24, 0x1fff);
    reg(b, 4, 0x33);
}

static void master_ack_nack(void)
{
    for (unsigned p = 0; p < 2; p++) {
        for (unsigned ack = 0; ack < 2; ack++) {
            Bus b = bus_new(p, true);
            unsigned rising = 0;
            uint8_t byte = 0;
            bool before = pin(&b, SCL);
            master_commands(&b, 0xa0, 3);
            for (unsigned tick = 0; tick < 10000; tick++) {
                qtest_clock_step(b.q, 50);
                bool after = pin(&b, SCL);
                if (after && !before) {
                    if (rising < 8) {
                        byte = (byte << 1) | pin(&b, SDA);
                    }
                    rising++;
                } else if (!after && before) {
                    if (rising == 8 && ack) {
                        external(&b, SDA, LOW);
                    } else if (rising >= 9) {
                        external(&b, SDA, RELEASE);
                    }
                }
                before = after;
                if (rd(&b, 0x20) & ((1 << 7) | (1 << 10))) {
                    break;
                }
            }
            g_assert_cmphex(byte, ==, 0xa0);
            if (ack) {
                g_assert_true(rd(&b, 0x20) & (1 << 7));
                g_assert_false(rd(&b, 0x20) & (1 << 10));
                g_assert_true(rd(&b, 0x60) & 0x80000000);
                g_assert_true(pin(&b, SDA));
                g_assert_true(pin(&b, SCL));
            } else {
                g_assert_true(rd(&b, 0x20) & (1 << 10));
                g_assert_false(rd(&b, 0x60) & 0x80000000);
                qtest_clock_step(b.q, 10000);
                g_assert_true(pin(&b, SDA));
                g_assert_true(pin(&b, SCL));
                g_assert_false(rd(&b, 8) & (1 << 4));
            }
            qtest_quit(b.q);
        }
    }
}

static void stretch_timeout_arbitration(void)
{
    Bus b = bus_new(0, true);
    master_commands(&b, 0xff, 3);
    /* Wait for START, then stretch the first data bit's low phase. */
    for (unsigned i = 0; i < 200; i++) {
        qtest_clock_step(b.q, 50);
        if (!pin(&b, SCL)) {
            break;
        }
    }
    external(&b, SCL, LOW);
    qtest_clock_step(b.q, 10000);
    g_assert_false(pin(&b, SCL));
    g_assert_false(rd(&b, 0x20) & ((1 << 7) | (1 << 8) | (1 << 10)));
    external(&b, SDA, LOW); /* competing master sends 0 where this master sends 1 */
    external(&b, SCL, RELEASE);
    qtest_clock_step(b.q, 1000);
    g_assert_true(rd(&b, 0x20) & (1 << 5));
    g_assert_true(rd(&b, 8) & (1 << 3));
    qtest_quit(b.q);

    b = bus_new(1, true);
    reg(&b, 0xc, 80);
    external(&b, SCL, LOW);
    master_commands(&b, 0xff, 3);
    qtest_clock_step(b.q, 10000);
    g_assert_true(rd(&b, 0x20) & (1 << 8));
    g_assert_true(rd(&b, 8) & (1 << 2));
    g_assert_false(rd(&b, 0x60) & 0x80000000);
    qtest_quit(b.q);
}

static void peer_repeated_start(void)
{
    for (unsigned port = 0; port < 2; port++) {
        Bus b = bus_fixture(port, true,
            "-device esp32-i2c-peer,gpio=/machine/soc/gpio,stretch-ns=5000");
        reg(&b, 0x1c, 0xa0);
        reg(&b, 0x1c, 0x20);
        reg(&b, 0x1c, 0xbe);
        reg(&b, 0x1c, 0xef);
        reg(&b, 0x58, 0);
        reg(&b, 0x5c, (1 << 11) | (1 << 8) | 4);
        reg(&b, 0x60, 3 << 11);
        reg(&b, 4, 0x33);
        wait_complete(&b);
        /* Pointer write, repeated START, two read commands with ACK then NACK. */
        reg(&b, 0x24, 0x1fff);
        reg(&b, 0x1c, 0xa0);
        reg(&b, 0x1c, 0x20);
        reg(&b, 0x1c, 0xa1);
        reg(&b, 0x58, 0);
        reg(&b, 0x5c, (1 << 11) | (1 << 8) | 2);
        reg(&b, 0x60, 0);
        reg(&b, 0x64, (1 << 11) | (1 << 8) | 1);
        reg(&b, 0x68, (2 << 11) | 1);
        reg(&b, 0x6c, (2 << 11) | (1 << 10) | 1);
        reg(&b, 0x70, 3 << 11);
        reg(&b, 4, 0x33);
        wait_complete(&b);
        g_assert_cmphex(rd(&b, 0x1c), ==, 0xbe);
        g_assert_cmphex(rd(&b, 0x1c), ==, 0xef);
        for (unsigned cmd = 0; cmd < 7; cmd++) {
            g_assert_true(rd(&b, 0x58 + cmd * 4) & 0x80000000);
        }
        qtest_quit(b.q);
    }
}

static void master_10bit_peer(void)
{
    for (unsigned port = 0; port < 2; port++) {
        Bus b = bus_fixture(port, true,
            "-device esp32-i2c-peer,gpio=/machine/soc/gpio,address=683,addr-10bit=on");
        /* Standard10-bit address0x2ab: prefix11110 A9 A8 R/W, then A7..A0. */
        const uint8_t write[] = {0xf4, 0xab, 0x20, 0x96, 0xbe};
        for (unsigned i = 0; i < G_N_ELEMENTS(write); i++) {
            reg(&b, 0x1c, write[i]);
        }
        reg(&b, 0x58, 0);
        reg(&b, 0x5c, (1 << 11) | (1 << 8) | G_N_ELEMENTS(write));
        reg(&b, 0x60, 3 << 11);
        reg(&b, 4, 0x33);
        wait_complete(&b);
        reg(&b, 0x24, 0x1fff);
        const uint8_t read[] = {0xf4, 0xab, 0x20, 0xf5};
        for (unsigned i = 0; i < G_N_ELEMENTS(read); i++) {
            reg(&b, 0x1c, read[i]);
        }
        reg(&b, 0x58, 0);
        reg(&b, 0x5c, (1 << 11) | (1 << 8) | 3);
        reg(&b, 0x60, 0);
        reg(&b, 0x64, (1 << 11) | (1 << 8) | 1);
        reg(&b, 0x68, (2 << 11) | 1);
        reg(&b, 0x6c, (2 << 11) | (1 << 10) | 1);
        reg(&b, 0x70, 3 << 11);
        reg(&b, 4, 0x33);
        wait_complete(&b);
        g_assert_cmphex(rd(&b, 0x1c), ==, 0x96);
        g_assert_cmphex(rd(&b, 0x1c), ==, 0xbe);
        qtest_quit(b.q);
    }
}

static void end_resume_gate_reset(void)
{
    Bus b = bus_fixture(0, true,
        "-device esp32-i2c-peer,gpio=/machine/soc/gpio");
    reg(&b, 0x1c, 0xa0);
    reg(&b, 0x1c, 0x20);
    reg(&b, 0x58, 0);
    reg(&b, 0x5c, (1 << 11) | (1 << 8) | 2);
    reg(&b, 0x60, 4 << 11);
    reg(&b, 4, 0x33);
    for (unsigned i = 0; i < 10000 && !(rd(&b, 0x20) & (1 << 3)); i++) {
        qtest_clock_step(b.q, 100);
    }
    g_assert_true(rd(&b, 0x20) & (1 << 3));
    g_assert_false(pin(&b, SCL));
    g_assert_true(rd(&b, 8) & (1 << 4));
    reg(&b, 0x24, 0x1fff);
    reg(&b, 0x1c, 0xfe);
    reg(&b, 0x1c, 0xcd);
    reg(&b, 0x58, (1 << 11) | (1 << 8) | 2);
    reg(&b, 0x5c, 3 << 11);
    reg(&b, 4, 0x33);
    qtest_clock_step(b.q, 100);
    bool scl = pin(&b, SCL), sda = pin(&b, SDA);
    qtest_writel(b.q, DPORT + 0xc0, 1 << 18);
    qtest_clock_step(b.q, 100000);
    g_assert_cmpint(pin(&b, SCL), ==, scl);
    g_assert_cmpint(pin(&b, SDA), ==, sda);
    g_assert_false(rd(&b, 0x20) & ((1 << 7) | (1 << 8)));
    qtest_writel(b.q, DPORT + 0xc0, (1 << 7) | (1 << 18));
    wait_complete(&b);
    /* Mid-transaction reset cancels timers and releases the peripheral drive. */
    reg(&b, 0x24, 0x1fff);
    master_commands(&b, 0xa0, 3);
    qtest_clock_step(b.q, 200);
    qtest_writel(b.q, DPORT + 0xc4, 1 << 7);
    qtest_writel(b.q, DPORT + 0xc4, 0);
    g_assert_cmphex(rd(&b, 4), ==, 3);
    g_assert_cmphex(rd(&b, 0x20), ==, 0);
    qtest_clock_step(b.q, 100000);
    g_assert_cmphex(rd(&b, 0x20), ==, 0);
    g_assert_true(pin(&b, SCL));
    g_assert_true(pin(&b, SDA));
    qtest_quit(b.q);
}

static void fifo_overflow_and_interrupt_mask(void)
{
    Bus b = bus_new(0, false);
    reg(&b, 0x10, 0x50);
    reg(&b, 0x18, 15);
    reg(&b, 0x28, 0);
    bitbang_start(&b);
    g_assert_true(bitbang_write(&b, 0xa0));
    for (unsigned i = 0; i < 33; i++) {
        g_assert_true(bitbang_write(&b, i));
    }
    g_assert_true(rd(&b, 0x20) & (1 << 2));
    g_assert_cmphex(rd(&b, 0x2c), ==, 0);
    reg(&b, 0x28, 1 << 2);
    g_assert_cmphex(rd(&b, 0x2c), ==, 1 << 2);
    reg(&b, 0x24, 1 << 2);
    g_assert_cmphex(rd(&b, 0x2c), ==, 0);
    g_assert_cmpuint((rd(&b, 8) >> 8) & 63, ==, 32);
    for (unsigned i = 0; i < 32; i++) {
        g_assert_cmphex(rd(&b, 0x1c), ==, i);
    }
    bitbang_stop(&b);
    qtest_quit(b.q);
}

static void slave_hold_and_filter(void)
{
    Bus b = bus_new(1, false);
    reg(&b, 0x10, 0x50);
    reg(&b, 0x54, 15); /* stable-level filter threshold = 7 APB cycles */
    external(&b, SDA, LOW);
    qtest_clock_step(b.q, 100);
    external(&b, SDA, RELEASE);
    qtest_clock_step(b.q, 500);
    g_assert_false(rd(&b, 0x20) & (1 << 9));
    external(&b, SDA, LOW);
    qtest_clock_step(b.q, 200);
    g_assert_true(rd(&b, 0x20) & (1 << 9));
    external(&b, SDA, RELEASE);
    qtest_clock_step(b.q, 500);
    reg(&b, 0x24, 0x1fff);
    reg(&b, 0x54, 0);
    reg(&b, 0x50, 15);
    bitbang_start(&b);
    qtest_clock_step(b.q, 500);
    /* A sub-threshold SCL pulse must not shift an address bit. */
    external(&b, SCL, RELEASE);
    qtest_clock_step(b.q, 100);
    external(&b, SCL, LOW);
    qtest_clock_step(b.q, 500);
    g_assert_true(bitbang_write(&b, 0xa0));
    g_assert_true(bitbang_write(&b, 0x5a));
    g_assert_cmphex(rd(&b, 0x1c), ==, 0x5a);
    bitbang_stop(&b);
    qtest_quit(b.q);

    b = bus_new(0, false);
    reg(&b, 0x10, 0x50);
    reg(&b, 0x30, 8); /* Slave SDA hold applies too: 200 ns at 40 MHz APB. */
    reg(&b, 0x1c, 0x55);
    bitbang_start(&b);
    for (unsigned i = 0; i < 8; i++) {
        external(&b, SDA, 0xa1 & (0x80 >> i) ? RELEASE : LOW);
        qtest_clock_step(b.q, 500);
        external(&b, SCL, RELEASE);
        qtest_clock_step(b.q, 1000);
        external(&b, SCL, LOW);
        if (i != 7) {
            qtest_clock_step(b.q, 500);
        }
    }
    external(&b, SDA, RELEASE);
    qtest_clock_step(b.q, 199);
    g_assert_true(pin(&b, SDA));
    qtest_clock_step(b.q, 1);
    g_assert_false(pin(&b, SDA));
    qtest_quit(b.q);
}

static void interrupt_level_routing(void)
{
    for (unsigned port = 0; port < 2; port++) {
        Bus b = bus_new(port, false);
        /* TRM interrupt matrix source49/50, not merely the INT_ST register. */
        qtest_irq_intercept_in(b.q, "/machine/soc/intmatrix");
        unsigned source = 49 + port;
        reg(&b, 0x18, 5 << 5);
        reg(&b, 0x28, 1 << 1);
        g_assert_true(qtest_get_irq(b.q, source));
        reg(&b, 0x24, 1 << 1);
        g_assert_true(qtest_get_irq(b.q, source));
        for (unsigned i = 0; i < 5; i++) {
            reg(&b, 0x1c, i);
        }
        g_assert_false(qtest_get_irq(b.q, source));
        reg(&b, 0x28, 0);
        reg(&b, 0x18, (5 << 5) | (1 << 13));
        g_assert_true(rd(&b, 0x20) & (1 << 1));
        g_assert_false(qtest_get_irq(b.q, source));
        reg(&b, 0x28, 1 << 1);
        g_assert_true(qtest_get_irq(b.q, source));
        reg(&b, 0x18, 0);
        g_assert_false(qtest_get_irq(b.q, source));
        qtest_quit(b.q);
    }
}

static void slave_bit_order_low_sample(void)
{
    Bus b = bus_new(0, false);
    reg(&b, 0x10, 0x50);
    reg(&b, 4, 0xc3);
    bitbang_start(&b);
    g_assert_true(bitbang_write(&b, 0xa0));
    g_assert_true(bitbang_write(&b, 0x96));
    g_assert_cmphex(rd(&b, 0x1c), ==, 0x69); /* RX LSB-first storage */
    bitbang_stop(&b);
    reg(&b, 0x1c, 0x96);
    bitbang_start(&b);
    g_assert_true(bitbang_write(&b, 0xa1));
    g_assert_cmphex(bitbang_read(&b, true), ==, 0x69); /* TX LSB-first wire */
    bitbang_stop(&b);
    qtest_quit(b.q);

    b = bus_new(1, false);
    reg(&b, 0x10, 0x50);
    reg(&b, 4, 7); /* sample SDA in SCL low phase */
    reg(&b, 0x34, 24); /* 600ns: after peer changes SDA at fall+500ns */
    bitbang_start(&b);
    g_assert_true(bitbang_write(&b, 0xa0));
    g_assert_true(bitbang_write(&b, 0x69));
    g_assert_cmphex(rd(&b, 0x1c), ==, 0x69);
    bitbang_stop(&b);
    qtest_quit(b.q);
}

static void master_sample_and_read_arbitration(void)
{
    for (unsigned port = 0; port < 2; port++) {
        for (unsigned low_sample = 0; low_sample < 2; low_sample++) {
            for (unsigned conflict = 0; conflict < 2; conflict++) {
                Bus b = bus_new(port, true);
                /* Receive wire byte 0x96 in LSB-first mode. Sampling on the
                 * low phase is delayed until after the peer changes data. */
                reg(&b, 0x34, low_sample ? 12 : 2);
                reg(&b, 0x58, 0);
                reg(&b, 0x5c, (2 << 11) | (1 << 10) | 1);
                reg(&b, 0x60, 3 << 11);
                reg(&b, 4, 0xb3 | (low_sample << 2));
                unsigned rises = 0;
                bool before = pin(&b, SCL);
                for (unsigned tick = 0; tick < 1000; tick++) {
                    qtest_clock_step(b.q, 25);
                    bool after = pin(&b, SCL);
                    if (!after && before) {
                        if (rises < 8) {
                            external(&b, SDA, 0x96 & (0x80 >> rises) ? RELEASE : LOW);
                        } else {
                            /* Competing receiver ACKs while this master
                             * emits NACK, which must lose arbitration. */
                            external(&b, SDA, conflict ? LOW : RELEASE);
                        }
                    } else if (after && !before) {
                        rises++;
                    }
                    before = after;
                    if (rd(&b, 0x20) & ((1 << 5) | (1 << 7))) {
                        break;
                    }
                }
                if (conflict) {
                    g_assert_true(rd(&b, 0x20) & (1 << 5));
                    g_assert_true(rd(&b, 8) & (1 << 3));
                    g_assert_false(rd(&b, 0x5c) & 0x80000000);
                    g_assert_false(rd(&b, 0x20) & (1 << 7));
                } else {
                    g_assert_true(rd(&b, 0x20) & (1 << 7));
                    g_assert_false(rd(&b, 0x20) & ((1 << 5) | (1 << 8)));
                    g_assert_cmphex(rd(&b, 0x1c), ==, 0x69);
                    g_assert_true(pin(&b, SDA));
                    g_assert_true(pin(&b, SCL));
                }
                qtest_quit(b.q);
            }
        }
    }
}

static void simultaneous_controllers(void)
{
    Bus first = bus_fixture(0, true,
        "-device esp32-i2c-peer,gpio=/machine/soc/gpio "
        "-device esp32-i2c-peer,gpio=/machine/soc/gpio,sda=18,scl=19,address=81");
    Bus second = {.q = first.q, .port = 1, .base = controller[1]};
    qtest_writel(first.q, MUX + 0x70, (2 << 12) | (1 << 9) | (1 << 8));
    qtest_writel(first.q, MUX + 0x74, (2 << 12) | (1 << 9) | (1 << 8));
    qtest_writel(first.q, GPIO + 0x530 + 18 * 4, 96);
    qtest_writel(first.q, GPIO + 0x530 + 19 * 4, 95);
    qtest_writel(first.q, GPIO + 0x130 + 96 * 4, 18 | (1 << 7));
    qtest_writel(first.q, GPIO + 0x130 + 95 * 4, 19 | (1 << 7));
    static const unsigned timing[] = {0, 0xc, 0x30, 0x34, 0x38, 0x40,
                                      0x44, 0x48, 0x4c, 0x50, 0x54};
    for (unsigned i = 0; i < ARRAY_SIZE(timing); i++) {
        reg(&second, timing[i], rd(&first, timing[i]));
    }
    /* Different bytes and different SCL periods establish independent engines. */
    reg(&second, 0, 39);
    reg(&second, 0x38, 32);
    master_commands(&first, 0xa0, 3);
    master_commands(&second, 0xa2, 3);
    wait_complete(&first);
    g_assert_false(rd(&second, 0x20) & (1 << 7));
    wait_complete(&second);
    g_assert_true(rd(&first, 0x60) & 0x80000000);
    g_assert_true(rd(&second, 0x60) & 0x80000000);
    qtest_quit(first.q);
}

static void pin_trace_case(unsigned filter, uint32_t apb_hz)
{
    char *path = g_strdup_printf("%s/esp32-i2c-%u.vcd", g_get_tmp_dir(), getpid());
    char *args = g_strdup_printf(
        "-global driver=esp32.gpio,property=pin-trace,value=%s "
        "-device esp32-i2c-peer,gpio=/machine/soc/gpio", path);
    Bus b = bus_fixture(0, true, args);
    reg(&b, 0x50, filter);
    if (apb_hz != 40000000) {
        /* RTC SOC clock: PLL=1 supplies 80MHz APB; CK8M=2 supplies 8MHz. */
        uint32_t clk = qtest_readl(b.q, 0x3ff48070);
        clk = (clk & ~(3u << 27)) | ((apb_hz == 80000000 ? 1u : 2u) << 27);
        qtest_writel(b.q, 0x3ff48070, clk);
    }
    master_commands(&b, 0xa0, 3);
    wait_complete(&b);
    qtest_quit(b.q);
    char *text;
    g_assert_true(g_file_get_contents(path, &text, NULL, NULL));
    g_assert_nonnull(strstr(text, "$timescale 1 ns $end"));
    g_assert_nonnull(strstr(text, "$var wire 1 P21 gpio21 $end"));
    g_assert_nonnull(strstr(text, "$var wire 1 D21 gpio21_esp_drive $end"));
    g_assert_nonnull(strstr(text, "$var wire 1 E21 gpio21_external_drive $end"));
    int64_t time = 0, last_fall = 0, last_rise = 0;
    bool scl = false, sda = false;
    unsigned clocks = 0, starts = 0, stops = 0;
    uint8_t wire_byte = 0;
    char drive = 'x', peer = 'z';
    char **lines = g_strsplit(text, "\n", -1);
    for (unsigned i = 0; lines[i]; i++) {
        char *line = lines[i];
        if (line[0] == '#') {
            int64_t next = g_ascii_strtoll(line + 1, NULL, 10);
            g_assert_cmpint(next, >=, time);
            time = next;
        } else if (line[0] && strchr("01zx", line[0]) && !strcmp(line + 1, "D21")) {
            drive = line[0];
            /* An open-drain ESP32 output never actively drives SDA high. */
            g_assert_cmpint(drive, !=, '1');
        } else if (line[0] && strchr("01zx", line[0]) && !strcmp(line + 1, "E21")) {
            peer = line[0];
        } else if (line[0] && strchr("01", line[0]) && !strcmp(line + 1, "P21")) {
            bool next = line[0] == '1';
            if (starts && scl && next != sda) {
                if (next) {
                    stops++;
                } else {
                    starts++;
                }
            } else if (!starts && scl && !next) {
                starts++;
            }
            sda = next;
        } else if (line[0] && strchr("01", line[0]) && !strcmp(line + 1, "P22")) {
            bool next = line[0] == '1';
            if (starts && !stops && next != scl) {
                if (next) {
                    if (clocks < 8) {
                        wire_byte = (wire_byte << 1) | sda;
                    } else if (clocks == 8) {
                        g_assert_false(sda);
                        g_assert_cmpint(drive, ==, 'z');
                        g_assert_cmpint(peer, ==, '0');
                    }
                    if (clocks < 9) {
                        /* (LOW_PERIOD+1) at reset APB=40 MHz. */
                        g_assert_cmpint(time - last_fall, ==, UINT64_C(20) * 1000000000 / apb_hz);
                    }
                    last_rise = time;
                    clocks++;
                } else {
                    if (clocks >= 1 && clocks <= 9) {
                        /* TRM table21.3-1: exact APB high cycles. Preserve
                         * the fractional tick at 80MHz (alternating ns). */
                        uint64_t extra = !filter ? 7 : (filter & 7) < 3 ?
                                         8 : 6 + (filter & 7);
                        uint64_t ns = (12 + extra) * UINT64_C(1000000000);
                        int64_t expected = clocks * ns / apb_hz -
                                           (clocks - 1) * ns / apb_hz;
                        g_assert_cmpint(time - last_rise, ==, expected);
                    }
                    last_fall = time;
                }
            }
            scl = next;
        }
    }
    g_assert_cmphex(wire_byte, ==, 0xa0);
    g_assert_cmpuint(starts, ==, 1);
    g_assert_cmpuint(stops, ==, 1);
    g_assert_cmpuint(clocks, ==, 10); /* 9 data/ACK clocks + STOP setup rise */
    g_strfreev(lines);
    g_free(text);
    g_unlink(path);
    g_free(path);
    g_free(args);
}

static void pin_trace_timing(void)
{
    pin_trace_case(0, 40000000);
    for (unsigned filter = 8; filter <= 15; filter++) {
        pin_trace_case(filter, 40000000);
    }
    pin_trace_case(0, 80000000);
    pin_trace_case(15, 80000000);
    pin_trace_case(0, 8000000);
}

static void reset_registers(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none -nic none");
    for (unsigned p = 0; p < 2; p++) {
        uint64_t a = controller[p];
        g_assert_cmphex(qtest_readl(q, a + 4), ==, 3);
        g_assert_cmphex(qtest_readl(q, a + 0x18), ==, (0x15 << 14) | (0x15 << 20));
        g_assert_cmphex(qtest_readl(q, a + 0x50), ==, 8);
        g_assert_cmphex(qtest_readl(q, a + 0xf8), ==, 0x16042000);
        /* TRM register reset values and defined writable field widths. */
        static const struct { unsigned offset; uint32_t mask, reset; } rw[] = {
            {0x00, 0x3fff, 0}, {0x04, 0x1f7, 3}, {0x0c, 0xfffff, 0},
            {0x10, 0x80007fff, 0}, {0x18, 0x3ffffff, (0x15 << 14) | (0x15 << 20)},
            {0x28, 0x1fff, 0}, {0x30, 0x3ff, 0}, {0x34, 0x3ff, 0},
            {0x38, 0x3fff, 0}, {0x40, 0x3ff, 8}, {0x44, 0x3ff, 8},
            {0x48, 0x3fff, 0}, {0x4c, 0x3ff, 0}, {0x50, 0xf, 8},
            {0x54, 0xf, 8}, {0xf8, UINT32_MAX, 0x16042000},
        };
        for (unsigned i = 0; i < G_N_ELEMENTS(rw); i++) {
            g_assert_cmphex(qtest_readl(q, a + rw[i].offset), ==, rw[i].reset);
            qtest_writel(q, a + rw[i].offset, UINT32_MAX);
            g_assert_cmphex(qtest_readl(q, a + rw[i].offset), ==, rw[i].mask);
            qtest_writel(q, a + rw[i].offset, rw[i].reset);
        }
        for (unsigned i = 0; i < 16; i++) {
            g_assert_cmphex(qtest_readl(q, a + 0x58 + i * 4), ==, 0);
            qtest_writel(q, a + 0x58 + i * 4, UINT32_MAX);
            g_assert_cmphex(qtest_readl(q, a + 0x58 + i * 4), ==, 0x80003fff);
        }
        qtest_writel(q, a + 0, UINT32_MAX);
        g_assert_cmphex(qtest_readl(q, a), ==, 0x3fff);
        qtest_writel(q, a + 0x98, UINT32_MAX);
        g_assert_cmphex(qtest_readl(q, a + 0x98), ==, 0);
        qtest_writel(q, a + 0x100, 0xaabbccdd);
        g_assert_cmphex(qtest_readl(q, a + 0x100), ==, 0xdd);
    }
    qtest_quit(q);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32/i2c/reset-registers", reset_registers);
    qtest_add_func("/esp32/i2c/slave-fifo", slave_fifo);
    qtest_add_func("/esp32/i2c/slave-nonfifo-10bit", slave_nonfifo_10bit);
    qtest_add_func("/esp32/i2c/master-ack-nack", master_ack_nack);
    qtest_add_func("/esp32/i2c/stretch-timeout-arbitration", stretch_timeout_arbitration);
    qtest_add_func("/esp32/i2c/peer-repeated-start", peer_repeated_start);
    qtest_add_func("/esp32/i2c/end-resume-gate-reset", end_resume_gate_reset);
    qtest_add_func("/esp32/i2c/fifo-overflow-interrupt-mask", fifo_overflow_and_interrupt_mask);
    qtest_add_func("/esp32/i2c/pin-trace-timing", pin_trace_timing);
    qtest_add_func("/esp32/i2c/simultaneous-controllers", simultaneous_controllers);
    qtest_add_func("/esp32/i2c/slave-hold-filter", slave_hold_and_filter);
    qtest_add_func("/esp32/i2c/interrupt-level-routing", interrupt_level_routing);
    qtest_add_func("/esp32/i2c/slave-bit-order-low-sample", slave_bit_order_low_sample);
    qtest_add_func("/esp32/i2c/master-sampling-read-arbitration", master_sample_and_read_arbitration);
    qtest_add_func("/esp32/i2c/master-10bit-peer", master_10bit_peer);
    return g_test_run();
}
