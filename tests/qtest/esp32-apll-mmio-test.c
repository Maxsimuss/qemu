/*
 * Independent original-ESP32 analog-I2C/APLL acceptance checks.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Command encoding is taken from the actual mask ROM's
 * rom_chip_i2c_readReg (0x40004110) and writeReg (0x40004168), independently
 * matched against Espressif's esp32_rev300_rom.elf. Register fields/START
 * pulse are from ESP-IDF regi2c_apll.h and clk_tree_ll.h; clock arithmetic
 * and force-power precedence are from ESP32 TRM 7.2.7. No particular bus
 * latency, calibration duration or analog CAL_CAP/UDF/OVF law is assumed.
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define ANA_APB 0x6000e000
#define ANA_DPORT 0x3ff4e000
#define ANA_CONFIG 0x44
#define ANA_APLL_DISABLED (1u << 14)
#define ANA_BBPLL_DISABLED (1u << 17)
#define COMMAND_WRITE (1u << 24)
#define COMMAND_BUSY (1u << 25)
#define APLL_BLOCK 0x6d
#define APLL_HOST 3
#define BBPLL_BLOCK 0x66
#define BBPLL_HOST 4
#define RTC_OPTIONS0 0x3ff48000
#define RTC_ANA_CONF 0x3ff48030
#define FORCE_PU (1u << 24)
#define FORCE_PD (1u << 23)
#define CAL_END (1u << 7)
#define EFUSE_RDATA3 0x3ff5a00c
#define GPIO 0x3ff44000
#define MUX 0x3ff49000
#define I2S 0x3ff4f000
#define WIFI_CLK_EN 0x3ff000cc
#define WIFI_CLK_COMMON 0x000003c9
#define WIFI_CLK_WIFI 0x00000406
#define WIFI_CLK_RNG (1u << 15)
#define WIFI_CLK_BT (0x61u << 11)
#define WIFI_PHY_REQUIRED 0x00008f8f

static QTestState *start(const char *trace, bool rev1, char **efuse_name)
{
    uint8_t efuse[124] = { 0 }; /* seven BLK0 words and three eight-word blocks */
    int fd = g_file_open_tmp("esp32-apll-efuse-XXXXXX", efuse_name, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    if (rev1) {
        efuse[3 * 4 + 1] = 0x80; /* BLK0_RDATA3 CHIP_VER_REV1 bit15 */
    }
    g_assert_true(g_file_set_contents(*efuse_name, (char *)efuse,
                                     sizeof(efuse), NULL));
    g_autofree char *trace_arg = trace ? g_strdup_printf(
        " -global driver=esp32.gpio,property=pin-trace,value=%s", trace) :
        g_strdup("");
    g_autofree char *args = g_strdup_printf(
        "-machine esp32 -display none -serial none -nic none "
        "-drive file=%s,if=none,id=apll-efuse,format=raw "
        "-global driver=nvram.esp32.efuse,property=drive,value=apll-efuse%s",
        *efuse_name, trace_arg);
    QTestState *q = qtest_init(args);
    g_assert_cmpint(!!(qtest_readl(q, EFUSE_RDATA3) & (1 << 15)), ==, rev1);
    qtest_writel(q, RTC_OPTIONS0,
                 (qtest_readl(q, RTC_OPTIONS0) & ~(1 << 18)) | (1 << 19));
    qtest_writel(q, ANA_APB + ANA_CONFIG,
                 0x3ff00 & ~ANA_APLL_DISABLED & ~ANA_BBPLL_DISABLED);
    qtest_writel(q, RTC_ANA_CONF, FORCE_PU);
    return q;
}

static uint32_t complete(QTestState *q, uint64_t address)
{
    /* This is a test watchdog, not a claim about silicon transaction time. */
    for (unsigned poll = 0; poll < 1000; poll++) {
        uint32_t status = qtest_readl(q, address);
        if (!(status & COMMAND_BUSY)) {
            return status;
        }
        qtest_clock_step(q, 1000);
    }
    g_error("analog-I2C command did not complete within test watchdog");
}

static void write_reg(QTestState *q, uint64_t alias, unsigned host,
                      unsigned block, unsigned reg, unsigned value)
{
    uint64_t address = alias + host * 4;
    qtest_writel(q, address, COMMAND_WRITE | value << 16 | reg << 8 | block);
    complete(q, address);
}

static unsigned read_reg(QTestState *q, uint64_t alias, unsigned host,
                         unsigned block, unsigned reg)
{
    uint64_t address = alias + host * 4;
    qtest_writel(q, address, reg << 8 | block);
    return (complete(q, address) >> 16) & 255;
}

static void apll_write(QTestState *q, unsigned reg, unsigned value)
{
    write_reg(q, ANA_APB, APLL_HOST, APLL_BLOCK, reg, value);
}

static unsigned apll_read(QTestState *q, unsigned reg)
{
    return read_reg(q, ANA_APB, APLL_HOST, APLL_BLOCK, reg);
}

static void calibrate(QTestState *q, bool rev1, unsigned sdm1)
{
    apll_write(q, 7, 5);
    apll_write(q, 9, 0);
    apll_write(q, 8, sdm1);
    apll_write(q, 5, 0x09);
    apll_write(q, 5, rev1 ? 0x49 : 0x69);
    apll_write(q, 4, 4);
    apll_write(q, 0, 0x0f);
    apll_write(q, 0, 0x3f);
    apll_write(q, 0, 0x1f);
    for (unsigned poll = 0; poll < 1000; poll++) {
        if (apll_read(q, 3) & CAL_END) {
            return;
        }
        qtest_clock_step(q, 1000);
    }
    g_error("SDK calibration START pulse did not complete within test watchdog");
}

static void finish(QTestState *q, char *efuse_name)
{
    qtest_quit(q);
    unlink(efuse_name);
    g_free(efuse_name);
}

static void test_alias_and_host(void)
{
    char *efuse_name;
    QTestState *q = start(NULL, true, &efuse_name);
    g_assert_cmphex(qtest_readl(q, ANA_DPORT + ANA_CONFIG), ==,
                    qtest_readl(q, ANA_APB + ANA_CONFIG));
    write_reg(q, ANA_DPORT, APLL_HOST, APLL_BLOCK, 9, 0xa5);
    g_assert_cmphex(apll_read(q, 9), ==, 0xa5);
    apll_write(q, 8, 0x5a);
    g_assert_cmphex(read_reg(q, ANA_DPORT, APLL_HOST, APLL_BLOCK, 8), ==, 0x5a);
    /* BBPLL is outside this model's supported analog subset. Its traffic
     * must be reported as unsupported and must not alias the APLL bank;
     * this test makes no claim about BBPLL command/data silicon behavior. */
    write_reg(q, ANA_APB, BBPLL_HOST, BBPLL_BLOCK, 2, 0x65);
    g_assert_cmphex(apll_read(q, 9), ==, 0xa5);
    g_assert_cmphex(apll_read(q, 8), ==, 0x5a);
    finish(q, efuse_name);
}

static void test_no_trigger(void)
{
    char *efuse_name;
    QTestState *q = start(NULL, true, &efuse_name);
    apll_write(q, 0, 0x0f); /* reset asserted, START clear */
    qtest_clock_step(q, 1000000);
    g_assert_cmphex(apll_read(q, 3) & CAL_END, ==, 0);
    apll_write(q, 0, 0x1f); /* reset released, still no START edge */
    qtest_clock_step(q, 1000000);
    g_assert_cmphex(apll_read(q, 3) & CAL_END, ==, 0);
    /* CAL_END is a hardware output, not software-writable success state. */
    apll_write(q, 3, 0xff);
    g_assert_cmphex(apll_read(q, 3) & CAL_END, ==, 0);
    calibrate(q, true, 0);
    g_assert_cmphex(apll_read(q, 3) & CAL_END, ==, CAL_END);
    apll_write(q, 0, 0x0f); /* a later reset must invalidate previous completion */
    qtest_clock_step(q, 1000000);
    g_assert_cmphex(apll_read(q, 3) & CAL_END, ==, 0);
    finish(q, efuse_name);
}

static void start_clock(QTestState *q)
{
    qtest_writel(q, 0x3ff000c0, 1 << 4);
    qtest_writel(q, MUX + 0x70, (2 << 12) | (1 << 9));
    qtest_writel(q, GPIO + 0x530 + 18 * 4, 23);
    qtest_writel(q, I2S + 0xac, (1 << 21) | (1 << 20) | 15);
    qtest_writel(q, I2S + 0xb0, 8 | (8 << 6) | (16 << 12) | (16 << 18));
    qtest_writel(q, I2S + 0x20, 32 | (32 << 6) | (1 << 19) | (1 << 20));
    qtest_writel(q, I2S, 0xa55ac33c);
    qtest_writel(q, I2S + 8, 1 << 4);
}

static unsigned check_edges(const char *name, int64_t start_ns,
                            int64_t end_ns, uint64_t period_num,
                            uint64_t period_den, unsigned wanted)
{
    g_autofree char *contents = NULL;
    g_assert_true(g_file_get_contents(name, &contents, NULL, NULL));
    g_auto(GStrv) lines = g_strsplit(contents, "\n", -1);
    uint64_t now = 0;
    unsigned edges = 0;
    for (unsigned i = 0; lines[i]; i++) {
        if (lines[i][0] == '#') {
            now = g_ascii_strtoull(lines[i] + 1, NULL, 10);
        } else if (now > start_ns && now <= end_ns &&
                   (!strcmp(lines[i], "0P18") || !strcmp(lines[i], "1P18"))) {
            edges++;
            g_assert_cmpuint(now - start_ns, ==,
                             (uint64_t)edges * period_num / period_den);
        }
    }
    g_assert_cmpuint(edges, ==, wanted);
    return edges;
}

static void test_fractional_trace(gconstpointer opaque)
{
    bool rev1 = GPOINTER_TO_INT(opaque);
    char *efuse_name;
    g_autofree char *trace = NULL;
    int fd = g_file_open_tmp("esp32-apll-mmio-XXXXXX.vcd", &trace, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    QTestState *q = start(trace, rev1, &efuse_name);
    calibrate(q, rev1, 0x80);
    /* Rev1 fout=40MHz*9.5/12=95MHz/3; rev0 ignores SDM1, fout=30MHz.
     * N15, BCK divider8 => half period36000/19ns or2000ns respectively.
     * 64000 edges distinguish the rational rate from integer-Hz truncation. */
    uint64_t num = rev1 ? 36000 : 2000;
    uint64_t den = rev1 ? 19 : 1;
    unsigned edges = rev1 ? 64000 : 32;
    int64_t start_ns = qtest_clock_step(q, 0);
    start_clock(q);
    int64_t end_ns = start_ns + (uint64_t)edges * num / den;
    qtest_clock_step(q, end_ns - start_ns);
    qtest_writel(q, I2S + 8, 0);
    finish(q, efuse_name);
    check_edges(trace, start_ns, end_ns, num, den, edges);
    unlink(trace);
}

static void test_power_dominance(void)
{
    char *efuse_name;
    QTestState *q = start(NULL, true, &efuse_name);
    calibrate(q, true, 0);
    start_clock(q);
    qtest_clock_step(q, 2000);
    g_assert_true(qtest_readl(q, GPIO + 0x3c) & (1 << 18));
    qtest_writel(q, RTC_ANA_CONF, FORCE_PU | FORCE_PD);
    uint32_t held = qtest_readl(q, GPIO + 0x3c) & (1 << 18);
    for (unsigned sample = 0; sample < 20; sample++) {
        qtest_clock_step(q, 2000);
        g_assert_cmphex(qtest_readl(q, GPIO + 0x3c) & (1 << 18), ==, held);
    }
    finish(q, efuse_name);
}

static void test_masks_and_bus_reset(void)
{
    char *efuse_name;
    QTestState *q = start(NULL, true, &efuse_name);
    static const unsigned masks[] = {
        0xff, 0x7f, 0xff, 0, 0xdf, 0x7f, 0x1f, 0x3f, 0xff, 0xff,
    };
    /* Masks follow the named fields in ESP-IDF regi2c_apll.h. Reset values
     * of this analog bank are unverified and are not asserted here. */
    for (unsigned reg = 0; reg < G_N_ELEMENTS(masks); reg++) {
        apll_write(q, reg, 0xff);
        g_assert_cmphex(apll_read(q, reg), ==, masks[reg]);
    }
    apll_write(q, 9, 0x5a);
    uint64_t command = ANA_APB + APLL_HOST * 4;
    qtest_writel(q, command, COMMAND_WRITE | (0xa5 << 16) | (9 << 8) |
                               APLL_BLOCK);
    g_assert_true(qtest_readl(q, command) & COMMAND_BUSY);
    g_assert_true(qtest_readl(q, ANA_DPORT + APLL_HOST * 4) & COMMAND_BUSY);
    qtest_writel(q, ANA_DPORT + ANA_CONFIG, 0x3ff00);
    qtest_clock_step(q, 1000000);
    g_assert_false(qtest_readl(q, command) & COMMAND_BUSY);
    qtest_writel(q, ANA_APB + ANA_CONFIG, 0x3ff00 & ~ANA_APLL_DISABLED);
    g_assert_cmphex(apll_read(q, 9), ==, 0x5a);
    finish(q, efuse_name);
}

static void test_cancelled_calibration(gconstpointer opaque)
{
    unsigned cancellation = GPOINTER_TO_UINT(opaque);
    char *efuse_name;
    QTestState *q = start(NULL, true, &efuse_name);
    calibrate(q, true, 0);
    apll_write(q, 0, 0x0f);
    apll_write(q, 0, 0x3f);
    /* Exercise cancellation during pending calibration. No assertion about
     * a silicon duration is made; only the model's eventual completion is
     * used, and cancellation must prevent a stale completion afterward. */
    if (cancellation == 0) {
        qtest_writel(q, RTC_ANA_CONF, FORCE_PD);
    } else if (cancellation == 1) {
        apll_write(q, 0, 0x0f);
    } else if (cancellation == 2) {
        apll_write(q, 7, 0); /* Invalid VCO: 40 MHz * 4 = 160 MHz. */
    } else {
        qtest_qmp_assert_success(q, "{ 'execute': 'system_reset' }");
        qtest_writel(q, ANA_APB + ANA_CONFIG, 0x3ff00 & ~ANA_APLL_DISABLED);
    }
    qtest_clock_step(q, 1000000);
    g_assert_cmphex(apll_read(q, 3) & CAL_END, ==, 0);
    finish(q, efuse_name);
}

static void test_reprogram_and_recover(void)
{
    char *efuse_name;
    QTestState *q = start(NULL, true, &efuse_name);
    calibrate(q, true, 0);
    start_clock(q);
    qtest_clock_step(q, 2000);
    g_assert_true(qtest_readl(q, GPIO + 0x3c) & (1 << 18));
    apll_write(q, 8, 0x80);
    g_assert_cmphex(apll_read(q, 3) & CAL_END, ==, 0);
    uint32_t held = qtest_readl(q, GPIO + 0x3c) & (1 << 18);
    for (unsigned sample = 0; sample < 20; sample++) {
        qtest_clock_step(q, 1000);
        g_assert_cmphex(qtest_readl(q, GPIO + 0x3c) & (1 << 18), ==, held);
    }
    calibrate(q, true, 0);
    qtest_writel(q, I2S + 8, 0);
    qtest_writel(q, I2S + 8, 1 << 4);
    qtest_clock_step(q, 1999);
    g_assert_false(qtest_readl(q, GPIO + 0x3c) & (1 << 18));
    qtest_clock_step(q, 1);
    g_assert_true(qtest_readl(q, GPIO + 0x3c) & (1 << 18));
    finish(q, efuse_name);
}

static void test_bias_power(void)
{
    char *efuse_name;
    QTestState *q = start(NULL, true, &efuse_name);
    apll_write(q, 9, 0x5a);
    qtest_writel(q, RTC_OPTIONS0, qtest_readl(q, RTC_OPTIONS0) | (1u << 18));
    apll_write(q, 9, 0xa5);
    qtest_writel(q, RTC_OPTIONS0, qtest_readl(q, RTC_OPTIONS0) & ~(1u << 18));
    g_assert_cmphex(apll_read(q, 9), ==, 0x5a);
    finish(q, efuse_name);
}

static void test_wifi_clock_enable_register(void)
{
    char *efuse_name;
    QTestState *q = start(NULL, true, &efuse_name);
    uint32_t value = qtest_readl(q, WIFI_CLK_EN);

    /* ESP-IDF soc/esp32/register/soc/dport_reg.h documents this full-width
     * R/W register's reset value and the masks used by phy_module_enable(). */
    g_assert_cmphex(value, ==, 0xfffce030);
    value |= WIFI_CLK_COMMON | WIFI_CLK_WIFI | WIFI_CLK_RNG | WIFI_CLK_BT;
    qtest_writel(q, WIFI_CLK_EN, value);
    g_assert_cmphex(qtest_readl(q, WIFI_CLK_EN), ==, value);
    g_assert_cmphex(qtest_readl(q, WIFI_CLK_EN) & WIFI_PHY_REQUIRED, ==,
                    WIFI_PHY_REQUIRED);
    /* Writes are ordinary R/W, not sticky forced-enable bits. */
    qtest_writel(q, WIFI_CLK_EN, value & ~WIFI_CLK_BT);
    g_assert_cmphex(qtest_readl(q, WIFI_CLK_EN), ==, value & ~WIFI_CLK_BT);
    finish(q, efuse_name);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32/apll-mmio/aliases-and-hosts", test_alias_and_host);
    qtest_add_func("/esp32/apll-mmio/calibration-no-trigger", test_no_trigger);
    qtest_add_data_func("/esp32/apll-mmio/rational-trace-rev1",
                       GINT_TO_POINTER(true), test_fractional_trace);
    qtest_add_data_func("/esp32/apll-mmio/fraction-ignored-rev0",
                       GINT_TO_POINTER(false), test_fractional_trace);
    qtest_add_func("/esp32/apll-mmio/power-dominance", test_power_dominance);
    qtest_add_func("/esp32/apll-mmio/masks-and-bus-reset", test_masks_and_bus_reset);
    static const char *cancel_names[] = { "power", "analog-reset",
        "invalid-coefficient", "system-reset" };
    for (unsigned i = 0; i < G_N_ELEMENTS(cancel_names); i++) {
        g_autofree char *path = g_strdup_printf(
            "/esp32/apll-mmio/cancel-calibration/%s", cancel_names[i]);
        qtest_add_data_func(path, GUINT_TO_POINTER(i), test_cancelled_calibration);
    }
    qtest_add_func("/esp32/apll-mmio/reprogram-and-recover", test_reprogram_and_recover);
    qtest_add_func("/esp32/apll-mmio/bias-power", test_bias_power);
    qtest_add_func("/esp32/apll-mmio/wifi-clock-enable",
                   test_wifi_clock_enable_register);
    return g_test_run();
}
