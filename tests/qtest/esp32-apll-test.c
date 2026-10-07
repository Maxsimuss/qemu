/* ESP32 APLL regressions: resolved pads and downstream clock consumers are
 * driven by the same guest-visible analog-I2C commands used by the ROM/SDK.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

/* DPORT peripheral clock gate register; bit 4 feeds I2S0, bit 21 feeds I2S1
 * (see esp32_perip_update in hw/xtensa/esp32.c). */
#define DPORT_PERIP_CLK_EN 0x3ff000c0
#define DPORT_CLK_EN_I2S0 (1u << 4)
#define DPORT_CLK_EN_I2S1 (1u << 21)

/* RTC power/clock registers (TRM 9.11, 9.24). */
#define RTC_ANA_CONF 0x3ff48030
#define RTC_PLLA_FORCE_PU (1u << 24)
#define RTC_PLLA_FORCE_PD (1u << 23)

/* GPIO matrix: output-signal select base; IO MUX function/input-enable. */
#define GPIO_OUT_SEL_BASE 0x3ff44530
#define GPIO_IN_REG 0x3ff4403c
#define IOMUX_BASE 0x3ff49000
#define IOMUX_PAD_BCK 0x70
#define IOMUX_PAD_WS 0x74
#define IOMUX_PAD_SD 0x8c
#define IOMUX_FUNC_MATRIX 2
#define IOMUX_FUNC_SHIFT 12
#define IOMUX_IE (1u << 9)

/* I2S0 master-TX output signals (TRM 22.1): BCK 23, WS 25, DATA 163. */
#define I2S0_SIG_TX_BCK 23
#define I2S0_SIG_TX_WS 25
#define I2S0_SIG_TX_DATA 163

/* I2S0 registers (TRM 22.7). */
#define I2S0_BASE 0x3ff4f000
#define I2S_FIFO_WR 0x00
#define I2S_CONF 0x08
#define I2S_TX_START (1u << 4)
#define I2S_FIFO_CONF 0x20
#define I2S_TX_DATA_NUM_SHIFT 6
#define I2S_TX_FIFO_MOD_FORCE_EN (1u << 19)
#define I2S_RX_FIFO_MOD_FORCE_EN (1u << 20)
#define I2S_SAMPLE_CONF 0xb0
#define I2S_TX_BCK_DIV_SHIFT 0
#define I2S_RX_BCK_DIV_SHIFT 6
#define I2S_TX_BITS_MOD_SHIFT 12
#define I2S_RX_BITS_MOD_SHIFT 18
/* CLKM_CONF (TRM 22.31): CLKA_ENA selects APLL over 160 MHz PLL_F160M.
 * CLK_EN (bit 20) exists only in the SDK header, not the TRM. */
#define I2S_CLKM_CONF 0xac
#define I2S_CLKA_ENA (1u << 21)
#define I2S_CLK_EN (1u << 20)
#define I2S_CLKM_DIV_NUM_MASK 0xff

/* Resolved digital pads used by every case. */
#define PAD_BCK 18
#define PAD_WS 19
#define PAD_SD 23

/* APLL fixture: fout = 40 MHz * (4 + 5) / (2 * (4 + 2)) = 30 MHz.
 * With N = 15 and M = 8: fi2s = 2 MHz, BCK = 250 kHz, half period 2000 ns. */
#define ANA_I2C_BASE 0x6000e000
#define ANA_CONFIG 0x6000e044
#define APLL_HOST 3
#define APLL_BLOCK 0x6d
#define APLL_CLKM_DIV_NUM 15
#define APLL_BCK_DIV 8
#define APLL_BCK_HALF_NS 2000

#define TEST_WORD 0xa55ac33c

static QTestState *boot(const char *extra)
{
    g_autofree char *args = g_strdup_printf("-machine esp32 -display none "
                                            "-serial none -nic none%s",
                                            extra ? extra : "");
    return qtest_init(args);
}

static void ana_i2c_write(QTestState *q, unsigned reg, uint8_t data)
{
    uint32_t cmd = (1u << 24) | ((uint32_t)data << 16) |
                   (reg << 8) | APLL_BLOCK;
    qtest_writel(q, ANA_I2C_BASE + 4 * APLL_HOST, cmd);
    for (unsigned i = 0; i < 8; i++) {
        if (!(qtest_readl(q, ANA_I2C_BASE + 4 * APLL_HOST) & (1u << 25))) {
            return;
        }
        qtest_clock_step(q, 1000);
    }
    g_assert_not_reached();
}

static uint8_t ana_i2c_read(QTestState *q, unsigned reg)
{
    uint32_t cmd = (reg << 8) | APLL_BLOCK;
    qtest_writel(q, ANA_I2C_BASE + 4 * APLL_HOST, cmd);
    for (unsigned i = 0; i < 8; i++) {
        uint32_t result = qtest_readl(q, ANA_I2C_BASE + 4 * APLL_HOST);
        if (!(result & (1u << 25))) {
            return result >> 16;
        }
        qtest_clock_step(q, 1000);
    }
    g_assert_not_reached();
    return 0;
}

static void program_apll(QTestState *q, unsigned sdm0, unsigned sdm1,
                         unsigned sdm2, unsigned odiv)
{
    /* ANA_CONFIG defaults to reset asserted for all analog I2C hosts. */
    qtest_writel(q, ANA_CONFIG, (0x3ffu << 8) & ~(1u << 14));
    ana_i2c_write(q, 4, odiv);
    ana_i2c_write(q, 5, 0x69);
    ana_i2c_write(q, 7, sdm2);
    ana_i2c_write(q, 8, sdm1);
    ana_i2c_write(q, 9, sdm0);
    ana_i2c_write(q, 0, 0x0f);
    ana_i2c_write(q, 0, 0x3f);
    ana_i2c_write(q, 0, 0x1f);
    for (unsigned i = 0; i < 8 && !(ana_i2c_read(q, 3) & 0x80); i++) {
        qtest_clock_step(q, 10000);
    }
    g_assert_true(ana_i2c_read(q, 3) & 0x80);
}

static void program_default_apll(QTestState *q)
{
    program_apll(q, 0, 0, 5, 4);
}

static bool pad_level(QTestState *q, unsigned pad)
{
    return (qtest_readl(q, GPIO_IN_REG) >> pad) & 1;
}

static void route_tx_pins(QTestState *q)
{
    qtest_writel(q, DPORT_PERIP_CLK_EN, DPORT_CLK_EN_I2S0 | DPORT_CLK_EN_I2S1);
    qtest_writel(q, IOMUX_BASE + IOMUX_PAD_BCK,
                 (IOMUX_FUNC_MATRIX << IOMUX_FUNC_SHIFT) | IOMUX_IE);
    qtest_writel(q, IOMUX_BASE + IOMUX_PAD_WS,
                 (IOMUX_FUNC_MATRIX << IOMUX_FUNC_SHIFT) | IOMUX_IE);
    qtest_writel(q, IOMUX_BASE + IOMUX_PAD_SD,
                 (IOMUX_FUNC_MATRIX << IOMUX_FUNC_SHIFT) | IOMUX_IE);
    qtest_writel(q, GPIO_OUT_SEL_BASE + PAD_BCK * 4, I2S0_SIG_TX_BCK);
    qtest_writel(q, GPIO_OUT_SEL_BASE + PAD_WS * 4, I2S0_SIG_TX_WS);
    qtest_writel(q, GPIO_OUT_SEL_BASE + PAD_SD * 4, I2S0_SIG_TX_DATA);
}

static void start_controller(QTestState *q, uint64_t base, uint32_t clkm)
{
    qtest_writel(q, base + I2S_CLKM_CONF, clkm);
    qtest_writel(q, base + I2S_SAMPLE_CONF,
                 APLL_BCK_DIV << I2S_TX_BCK_DIV_SHIFT |
                 APLL_BCK_DIV << I2S_RX_BCK_DIV_SHIFT |
                 16 << I2S_TX_BITS_MOD_SHIFT | 16 << I2S_RX_BITS_MOD_SHIFT);
    qtest_writel(q, base + I2S_FIFO_CONF,
                 32 | 32 << I2S_TX_DATA_NUM_SHIFT |
                 I2S_TX_FIFO_MOD_FORCE_EN | I2S_RX_FIFO_MOD_FORCE_EN);
    qtest_writel(q, base + I2S_FIFO_WR, TEST_WORD);
    qtest_writel(q, base + I2S_CONF, I2S_TX_START);
}

static void start_tx(QTestState *q, uint32_t clkm)
{
    start_controller(q, I2S0_BASE, clkm);
}

static void ana_conf_reset_mask(void)
{
    QTestState *q = boot(NULL);
    g_assert_cmphex(qtest_readl(q, RTC_ANA_CONF), ==, RTC_PLLA_FORCE_PD);
    qtest_writel(q, RTC_ANA_CONF, 0xffffffffu);
    g_assert_cmphex(qtest_readl(q, RTC_ANA_CONF), ==, 0xdd800000u);
    qtest_writel(q, RTC_ANA_CONF, 0);
    g_assert_cmphex(qtest_readl(q, RTC_ANA_CONF), ==, 0);
    qtest_quit(q);
}

static void tx_on_apll(void)
{
    QTestState *q = boot(NULL);
    program_default_apll(q);
    qtest_writel(q, RTC_ANA_CONF, RTC_PLLA_FORCE_PU);
    route_tx_pins(q);
    start_tx(q, I2S_CLKA_ENA | I2S_CLK_EN | APLL_CLKM_DIV_NUM);
    g_assert_false(pad_level(q, PAD_BCK));
    qtest_clock_step(q, APLL_BCK_HALF_NS - 1);
    g_assert_false(pad_level(q, PAD_BCK));
    qtest_clock_step(q, 1);
    g_assert_true(pad_level(q, PAD_BCK));
    qtest_clock_step(q, APLL_BCK_HALF_NS);
    g_assert_false(pad_level(q, PAD_BCK));
    qtest_clock_step(q, APLL_BCK_HALF_NS);
    g_assert_true(pad_level(q, PAD_BCK));
    /* Realign the transmitter so the decoded frame starts at bit 0. */
    qtest_writel(q, I2S0_BASE + I2S_CONF, 0);
    qtest_clock_step(q, 2 * APLL_BCK_HALF_NS);
    g_assert_false(pad_level(q, PAD_BCK));
    qtest_writel(q, I2S0_BASE + I2S_FIFO_WR, TEST_WORD);
    qtest_writel(q, I2S0_BASE + I2S_CONF, I2S_TX_START);
    g_assert_false(pad_level(q, PAD_BCK));
    uint32_t received = 0;
    for (unsigned bit = 0; bit < 32; bit++) {
        g_assert_cmpint(pad_level(q, PAD_WS), ==, (bit / 16) & 1);
        qtest_clock_step(q, APLL_BCK_HALF_NS);
        g_assert_true(pad_level(q, PAD_BCK));
        received = (received << 1) | pad_level(q, PAD_SD);
        qtest_clock_step(q, APLL_BCK_HALF_NS);
        g_assert_false(pad_level(q, PAD_BCK));
    }
    g_assert_cmphex(received, ==, TEST_WORD);
    qtest_quit(q);
}

static void tx_without_source(QTestState *q)
{
    route_tx_pins(q);
    start_tx(q, I2S_CLKA_ENA | I2S_CLK_EN | APLL_CLKM_DIV_NUM);
    g_assert_false(pad_level(q, PAD_BCK));
    qtest_clock_step(q, 10 * APLL_BCK_HALF_NS);
    g_assert_false(pad_level(q, PAD_BCK));
    qtest_quit(q);
}

/* Valid coefficients but power-down forced: no BCK edges. */
static void disabled_no_clock(void)
{
    QTestState *q = boot(NULL);
    program_default_apll(q);
    qtest_writel(q, RTC_ANA_CONF, RTC_PLLA_FORCE_PD);
    tx_without_source(q);
}

/* Power enabled but default coefficients are invalid: no BCK edges. */
static void invalid_no_clock(void)
{
    QTestState *q = boot(NULL);
    qtest_writel(q, RTC_ANA_CONF, RTC_PLLA_FORCE_PU);
    tx_without_source(q);
}

typedef struct RateCase {
    const char *globals;
    uint32_t hz;
} RateCase;

static void coefficient_rate(gconstpointer opaque)
{
    const RateCase *test = opaque;
    QTestState *q = boot(test->globals);
    qtest_writel(q, RTC_ANA_CONF, RTC_PLLA_FORCE_PU);
    route_tx_pins(q);
    start_tx(q, I2S_CLKA_ENA | I2S_CLK_EN | APLL_CLKM_DIV_NUM);
    /* N=15, M=8: half-period is 60 seconds / fout in hertz. These
     * independently calculated rates cover revision errata and large ODIV. */
    uint64_t previous = 0;
    for (unsigned edge = 1; edge <= 128; edge++) {
        uint64_t deadline = UINT64_C(60000000000) * edge / test->hz;
        qtest_clock_step(q, deadline - previous - 1);
        g_assert_cmpint(pad_level(q, PAD_BCK), ==, !(edge & 1));
        qtest_clock_step(q, 1);
        g_assert_cmpint(pad_level(q, PAD_BCK), ==, edge & 1);
        previous = deadline;
    }
    qtest_quit(q);
}

static const RateCase rates[] = {
    { APLL_GLOBALS " -global driver=misc.esp32.rtc_cntl,property=apll-rev0,value=on"
      " -global driver=misc.esp32.rtc_cntl,property=apll-sdm0,value=255"
      " -global driver=misc.esp32.rtc_cntl,property=apll-sdm1,value=255", 30000000 },
    { APLL_GLOBALS " -global driver=misc.esp32.rtc_cntl,property=apll-sdm1,value=128",
      31666666 },
    { APLL_GLOBALS " -global driver=misc.esp32.rtc_cntl,property=apll-sdm0,value=1",
      30000050 },
    { " -global driver=misc.esp32.rtc_cntl,property=apll-sdm2,value=5"
      " -global driver=misc.esp32.rtc_cntl,property=apll-odiv,value=31", 5454545 },
    { " -global driver=misc.esp32.rtc_cntl,property=apll-sdm2,value=10"
      " -global driver=misc.esp32.rtc_cntl,property=apll-odiv,value=4"
      " -global driver=misc.esp32.rtc_cntl,property=apll-xtal-hz,value=26000000",
      30333333 },
};

static void source_loss(gconstpointer opaque)
{
    bool reset = GPOINTER_TO_UINT(opaque);
    g_autofree char *name = NULL;
    g_autofree char *contents = NULL;
    int fd = g_file_open_tmp("esp32-apll-XXXXXX.vcd", &name, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    g_autofree char *args = g_strdup_printf(APLL_GLOBALS
        " -global driver=esp32.gpio,property=pin-trace,value=%s", name);
    QTestState *q = boot(args);
    qtest_writel(q, RTC_ANA_CONF, RTC_PLLA_FORCE_PU);
    route_tx_pins(q);
    start_tx(q, I2S_CLKA_ENA | I2S_CLK_EN | APLL_CLKM_DIV_NUM);
    qtest_clock_step(q, APLL_BCK_HALF_NS);
    g_assert_true(pad_level(q, PAD_BCK));
    if (reset) {
        qtest_qmp_assert_success(q, "{ 'execute': 'system_reset' }");
        g_assert_cmphex(qtest_readl(q, RTC_ANA_CONF), ==, RTC_PLLA_FORCE_PD);
        route_tx_pins(q);
        start_tx(q, I2S_CLKA_ENA | I2S_CLK_EN | APLL_CLKM_DIV_NUM);
    } else {
        /* CPU_CLK=APLL/4, APB_CLK=APLL/8 must not prevent independent
         * I2S source invalidation when the APLL is subsequently disabled. */
        qtest_writel(q, 0x3ff48070, 3u << 27);
        qtest_writel(q, RTC_ANA_CONF, RTC_PLLA_FORCE_PD | RTC_PLLA_FORCE_PU);
    }
    qtest_clock_step(q, 10 * APLL_BCK_HALF_NS);
    qtest_quit(q);
    g_assert_true(g_file_get_contents(name, &contents, NULL, NULL));
    /* An unavailable source must not keep emitting the old clock. */
    g_assert_nonnull(strstr(contents, "xP18\nxD18"));
    unlink(name);
}

static void cpu_apb_derivation(void)
{
    QTestState *q = boot(APLL_GLOBALS);
    qtest_writel(q, RTC_ANA_CONF, RTC_PLLA_FORCE_PU);
    qtest_writel(q, 0x3ff48070, 3u << 27);
    for (unsigned divider = 0; divider < 2; divider++) {
        uint32_t cpu_hz = divider ? 15000000 : 7500000;
        qtest_writel(q, 0x3ff0003c, divider);
        for (unsigned core = 0; core < 2; core++) {
            g_autofree char *path = g_strdup_printf("/machine/soc/cpu%u/clk-in",
                                                    core);
            QDict *reply = qtest_qmp(q, "{ 'execute': 'qom-get', 'arguments': {"
                " 'path': %s, 'property': 'qtest-clock-period' } }", path);
            g_assert_true(qdict_haskey(reply, "return"));
            g_assert_cmpuint(qdict_get_int(reply, "return"), ==,
                            (UINT64_C(1000000000) << 32) / cpu_hz);
            qobject_unref(reply);
        }
        /* An actual APB consumer, not just a reported source frequency.
         * FRC counts up with prescaler 1 at CPU/APLL/2. */
        qtest_writel(q, 0x3ff47008, 0);
        qtest_writel(q, 0x3ff47000, 0);
        qtest_writel(q, 0x3ff47008, 1u << 7);
        qtest_clock_step(q, 100000);
        g_assert_cmpuint(qtest_readl(q, 0x3ff47004), ==, divider ? 750 : 375);
        /* TIMG divider=2 must retain fractional-MHz APB precision too. */
        qtest_writel(q, 0x3ff5f000, 0);
        qtest_writel(q, 0x3ff5f018, 0);
        qtest_writel(q, 0x3ff5f01c, 0);
        qtest_writel(q, 0x3ff5f020, 1);
        qtest_writel(q, 0x3ff5f000, (3u << 30) | (2 << 13));
        qtest_clock_step(q, 200000);
        qtest_writel(q, 0x3ff5f00c, 1);
        g_assert_cmpuint(qtest_readl(q, 0x3ff5f004), ==, divider ? 750 : 375);
        qtest_writel(q, 0x3ff5f010, divider ? 1500 : 750);
        qtest_writel(q, 0x3ff5f014, 0);
        qtest_writel(q, 0x3ff5f0a4, 1);
        qtest_writel(q, 0x3ff5f000,
                     (3u << 30) | (2 << 13) | (1 << 11) | (1 << 10));
        qtest_clock_step(q, 199999);
        g_assert_cmphex(qtest_readl(q, 0x3ff5f09c) & 1, ==, 0);
        qtest_clock_step(q, 1);
        g_assert_cmphex(qtest_readl(q, 0x3ff5f09c) & 1, ==, 1);
    }
    qtest_quit(q);
}

static void watchdog_apb_precision(void)
{
    QTestState *q = boot(APLL_GLOBALS);
    qtest_writel(q, RTC_ANA_CONF, RTC_PLLA_FORCE_PU);
    qtest_writel(q, 0x3ff48070, 3u << 27);
    /* APLL30MHz/4/2 = 3.75MHz APB, WDT prescaler2,375ticks=200us.
     * Stage0 requests only an interrupt, so no CPU/whole-chip reset occurs. */
    qtest_writel(q, 0x3ff5f064, 0x50d83aa1);
    qtest_writel(q, 0x3ff5f048, 0);
    qtest_writel(q, 0x3ff5f04c, 2 << 16);
    qtest_writel(q, 0x3ff5f050, 375);
    qtest_writel(q, 0x3ff5f060, 1u << 31);
    qtest_writel(q, 0x3ff5f048, (1u << 31) | (1 << 29) | (1 << 21));
    qtest_clock_step(q, 199999);
    g_assert_cmphex(qtest_readl(q, 0x3ff5f09c) & 4, ==, 0);
    qtest_clock_step(q, 1);
    g_assert_cmphex(qtest_readl(q, 0x3ff5f09c) & 4, ==, 4);
    qtest_quit(q);
}

static void second_i2s_controller(void)
{
    QTestState *q = boot(APLL_GLOBALS);
    qtest_writel(q, RTC_ANA_CONF, RTC_PLLA_FORCE_PU);
    route_tx_pins(q);
    qtest_writel(q, GPIO_OUT_SEL_BASE + PAD_BCK * 4, 24);
    qtest_writel(q, GPIO_OUT_SEL_BASE + PAD_WS * 4, 26);
    qtest_writel(q, GPIO_OUT_SEL_BASE + PAD_SD * 4, 189);
    start_controller(q, 0x3ff6d000,
                     I2S_CLKA_ENA | I2S_CLK_EN | APLL_CLKM_DIV_NUM);
    uint32_t word = 0;
    for (unsigned bit = 0; bit < 32; bit++) {
        qtest_clock_step(q, APLL_BCK_HALF_NS);
        g_assert_true(pad_level(q, PAD_BCK));
        g_assert_cmpint(pad_level(q, PAD_WS), ==, bit / 16);
        word = (word << 1) | pad_level(q, PAD_SD);
        qtest_clock_step(q, APLL_BCK_HALF_NS);
        g_assert_false(pad_level(q, PAD_BCK));
    }
    g_assert_cmphex(word, ==, TEST_WORD);
    qtest_quit(q);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32/apll/ana-conf", ana_conf_reset_mask);
    qtest_add_func("/esp32/apll/tx-on-apll", tx_on_apll);
    qtest_add_func("/esp32/apll/disabled-no-clock", disabled_no_clock);
    qtest_add_func("/esp32/apll/invalid-no-clock", invalid_no_clock);
    static const char *names[] = { "rev0-fraction-ignored", "sdm1-fraction",
        "sdm0-fraction", "below-16mhz", "26mhz-crystal" };
    for (unsigned i = 0; i < G_N_ELEMENTS(rates); i++) {
        g_autofree char *path = g_strdup_printf("/esp32/apll/%s", names[i]);
        g_test_add_data_func(path, &rates[i], coefficient_rate);
    }
    g_test_add_data_func("/esp32/apll/cpu-source-power-down", NULL, source_loss);
    g_test_add_data_func("/esp32/apll/reset-source", GUINT_TO_POINTER(1), source_loss);
    qtest_add_func("/esp32/apll/cpu-apb-derivation", cpu_apb_derivation);
    qtest_add_func("/esp32/apll/i2s1", second_i2s_controller);
    qtest_add_func("/esp32/apll/watchdog-apb", watchdog_apb_precision);
    return g_test_run();
}
