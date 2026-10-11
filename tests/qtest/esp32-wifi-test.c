/* Original ESP32 Wi-Fi MMIO mapping checks.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"
#include "hw/xtensa/esp32_wifi.h"

#define WIFI_DPORT 0x3ff73000
#define WIFI_APB 0x60033000
#define WIFI_RXCTRL 0x3ff5c000
#define WIFI_RXCTRL_APB 0x6001c000
#define WIFI_FE_TXRX 0x3ff460a0
#define WIFI_FE 0x3ff46000
#define WIFI_FE2 0x3ff45000
#define WIFI_FE_APB 0x60006000
#define WIFI_FE2_APB 0x60005000
#define WIFI_BSSID0 0x000
#define WIFI_MAC0 0x040
#define WIFI_DMA_INT_STATUS 0xc48
#define WIFI_DMA_INT_CLR 0xc4c
#define WIFI_DMA_OUTLINK 0xd20
#define WIFI_CRYPTO_KEY_TABLE ESP32_WIFI_CRYPTO_KEY_TABLE
#define WIFI_CRYPTO_KEY_VALID ESP32_WIFI_CRYPTO_KEY_VALID
#define WIFI_MMIO_LAST_WORD 0x1ffc
#define WIFI_DMA_INLINK 0x088
#define DPORT_WIFI_CLK_EN 0x3ff000cc
#define DPORT_CORE_RST_EN 0x3ff000d0
#define DPORT_WIFI_CLK_RESET 0xfffce030
#define DPORT_WIFI_MAC_CLOCKS 0x00000406
#define DPORT_WIFIMAC_RST (1u << 2)
#define MATRIX 0x3ff00104
#define INTMATRIX_PATH "/machine/soc/intmatrix"

static void wifi_enable_mac(QTestState *q)
{
    qtest_writel(q, DPORT_WIFI_CLK_EN,
                 DPORT_WIFI_CLK_RESET | DPORT_WIFI_MAC_CLOCKS);
    qtest_writel(q, DPORT_CORE_RST_EN, 0);
}

static void mmio_aliases(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic user,model=misc.esp32_wifi");

    qtest_writel(q, WIFI_DPORT + WIFI_BSSID0, 0x11223344);
    g_assert_cmphex(qtest_readl(q, WIFI_APB + WIFI_BSSID0), ==, 0x11223344);
    qtest_writel(q, WIFI_APB + WIFI_MAC0, 0x55667788);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_MAC0), ==, 0x55667788);
    qtest_writel(q, WIFI_DPORT + 0x205c, 0x5a5a1234);
    g_assert_cmphex(qtest_readl(q, WIFI_APB + 0x205c), ==, 0x5a5a1234);
    qtest_quit(q);
}

static uint64_t read_wdev_tsf(QTestState *q, hwaddr low, hwaddr high)
{
    uint64_t value = qtest_readl(q, WIFI_APB + low);

    value |= (uint64_t)qtest_readl(q, WIFI_APB + high) << 32;
    return value;
}

static void wdev_tsf_latch_clock_alias_reset(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");
    uint64_t first, second, third, pair1;

    /* WDEV TSF runs from virtual microseconds in the MAC clock/reset domain. */
    qtest_clock_step(q, 2000000);
    g_assert_cmpuint(read_wdev_tsf(q, ESP32_WIFI_WDEV_TSF0_LO,
                                   ESP32_WIFI_WDEV_TSF0_HI), ==, 0);
    wifi_enable_mac(q);
    qtest_clock_step(q, 12345000);

    /* The IDF helper latches each 64-bit source with bit 0/1 before reading
     * its low/high words. The APB view aliases the legacy Wi-Fi MMIO window. */
    qtest_writel(q, WIFI_DPORT + ESP32_WIFI_WDEV_TSF_CTRL,
                 ESP32_WIFI_WDEV_TSF0_LATCH);
    first = read_wdev_tsf(q, ESP32_WIFI_WDEV_TSF0_LO,
                          ESP32_WIFI_WDEV_TSF0_HI);
    qtest_clock_step(q, 2500000);
    second = read_wdev_tsf(q, ESP32_WIFI_WDEV_TSF0_LO,
                           ESP32_WIFI_WDEV_TSF0_HI);
    g_assert_cmpuint(first, >, 10000);
    g_assert_cmpuint(second, ==, first);

    /* Counter 1 is independently latched; changing its latch does not change
     * the already-frozen counter 0 snapshot. */
    qtest_writel(q, WIFI_APB + ESP32_WIFI_WDEV_TSF_CTRL,
                 ESP32_WIFI_WDEV_TSF0_LATCH | ESP32_WIFI_WDEV_TSF1_LATCH);
    pair1 = read_wdev_tsf(q, ESP32_WIFI_WDEV_TSF1_LO,
                          ESP32_WIFI_WDEV_TSF1_HI);
    g_assert_cmpuint(pair1, >=, second + 2000);
    g_assert_cmpuint(read_wdev_tsf(q, ESP32_WIFI_WDEV_TSF0_LO,
                                   ESP32_WIFI_WDEV_TSF0_HI), ==, second);
    qtest_writel(q, WIFI_DPORT + ESP32_WIFI_WDEV_TSF_CTRL, 0);
    qtest_clock_step(q, 3000000);
    third = read_wdev_tsf(q, ESP32_WIFI_WDEV_TSF0_LO,
                          ESP32_WIFI_WDEV_TSF0_HI);
    g_assert_cmpuint(third, >=, first + 2500);

    /* Clock gating pauses the counter, and re-enabling resumes accumulated
     * virtual time rather than counting while the MAC clock is absent. */
    qtest_writel(q, DPORT_WIFI_CLK_EN, DPORT_WIFI_CLK_RESET);
    qtest_clock_step(q, 9000000);
    g_assert_cmpuint(read_wdev_tsf(q, ESP32_WIFI_WDEV_TSF0_LO,
                                   ESP32_WIFI_WDEV_TSF0_HI), ==, third);
    wifi_enable_mac(q);
    qtest_clock_step(q, 4000000);
    first = read_wdev_tsf(q, ESP32_WIFI_WDEV_TSF0_LO,
                          ESP32_WIFI_WDEV_TSF0_HI);
    g_assert_cmpuint(first, >=, third + 3500);

    /* WIFIMAC reset clears both timer and latch state. */
    qtest_writel(q, DPORT_CORE_RST_EN, DPORT_WIFIMAC_RST);
    g_assert_cmphex(qtest_readl(q, WIFI_APB + ESP32_WIFI_WDEV_TSF_CTRL), ==, 0);
    qtest_writel(q, DPORT_CORE_RST_EN, 0);
    qtest_clock_step(q, 1000000);
    g_assert_cmpuint(read_wdev_tsf(q, ESP32_WIFI_WDEV_TSF0_LO,
                                   ESP32_WIFI_WDEV_TSF0_HI), >=, 1000);
    qtest_clock_step(q, 2000000);
    g_assert_cmpuint(read_wdev_tsf(q, ESP32_WIFI_WDEV_TSF0_LO,
                                   ESP32_WIFI_WDEV_TSF0_HI), >=, 2000);

    qtest_qmp_assert_success(q, "{ 'execute': 'system_reset' }");
    qtest_qmp_eventwait(q, "RESET");
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + ESP32_WIFI_WDEV_TSF_CTRL), ==, 0);
    first = qtest_readl(q, WIFI_DPORT + ESP32_WIFI_WDEV_TSF0_LO);
    first |= (uint64_t)qtest_readl(q, WIFI_DPORT + ESP32_WIFI_WDEV_TSF0_HI) << 32;
    g_assert_cmpuint(first, ==, 0);
    qtest_quit(q);
}

static void rxctrl_agc_rmw_aliases_and_reset(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");
    uint32_t value;

    /* The supplied PHY ELF's disable_wifi_agc() sequence, preserving
     * unrelated bits exactly as its RMW operations do. */
    value = qtest_readl(q, WIFI_RXCTRL + 0x01c);
    qtest_writel(q, WIFI_RXCTRL + 0x01c,
                 (value & 0xff00ffff) | 0x007f0000);
    qtest_writel(q, WIFI_RXCTRL + 0x038,
                 qtest_readl(q, WIFI_RXCTRL + 0x038) | 0x04000000);
    qtest_writel(q, WIFI_RXCTRL + 0x030,
                 qtest_readl(q, WIFI_RXCTRL + 0x030) & ~0x30);
    qtest_writel(q, WIFI_RXCTRL + 0x080,
                 qtest_readl(q, WIFI_RXCTRL + 0x080) | 1);
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL_APB + 0x01c), ==,
                    0x007f0000);
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL_APB + 0x038), ==,
                    0x04000000);
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL_APB + 0x030), ==, 0);
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL_APB + 0x080), ==, 1);

    /* enable_wifi_agc() restores automatic control, retains the common
     * RX-control bit, and updates the same register contents through DPORT. */
    qtest_writel(q, WIFI_RXCTRL_APB + 0x080,
                 qtest_readl(q, WIFI_RXCTRL_APB + 0x080) & ~1u);
    qtest_writel(q, WIFI_RXCTRL_APB + 0x030,
                 (qtest_readl(q, WIFI_RXCTRL_APB + 0x030) & ~0x30u) | 0x10);
    qtest_writel(q, WIFI_RXCTRL_APB + 0x01c,
                 (qtest_readl(q, WIFI_RXCTRL_APB + 0x01c) & 0xff00ffff) |
                 0x000c0000);
    qtest_writel(q, WIFI_RXCTRL_APB + 0x038,
                 qtest_readl(q, WIFI_RXCTRL_APB + 0x038) | 0x04000000);
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL + 0x01c), ==, 0x000c0000);
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL + 0x030), ==, 0x10);
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL + 0x038), ==, 0x04000000);
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL + 0x080), ==, 0);

    /* The driver's channel-selection path reads d008[31:29] before choosing
     * the RFPLL implementation. Its reset value is undocumented; zero keeps
     * the previous unmapped-read behavior until guest writes are observed. */
    qtest_writel(q, WIFI_RXCTRL + ESP32_WIFI_RXCTRL_RFPLL_MODE, 0x60000000);
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL_APB +
                                ESP32_WIFI_RXCTRL_RFPLL_MODE), ==,
                    0x60000000);

    qtest_qmp_assert_success(q, "{ 'execute': 'system_reset' }");
    qtest_qmp_eventwait(q, "RESET");
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL + 0x01c), ==, 0);
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL + 0x030), ==, 0);
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL + 0x038), ==, 0);
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL + 0x080), ==, 0);
    g_assert_cmphex(qtest_readl(q, WIFI_RXCTRL +
                                ESP32_WIFI_RXCTRL_RFPLL_MODE), ==, 0);
    qtest_quit(q);
}

static void fe_txrx_force_mode_rmw_and_reset(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic user,model=misc.esp32_wifi");
    uint32_t value;

    /* Exact three RMW masks from the supplied IDF PHY's force_txrxoff().
     * The register reset/analog semantics are not public; the test checks
     * that documented software accesses preserve the other bits. */
    qtest_writel(q, WIFI_FE_TXRX, 0xa5a55a5a);
    value = qtest_readl(q, WIFI_FE_TXRX);
    value = (value & 0xffffcfff) | 0x2000;
    qtest_writel(q, WIFI_FE_TXRX, value);
    value = qtest_readl(q, WIFI_FE_TXRX);
    value = (value & 0xfffff3ff) | 0x0800;
    qtest_writel(q, WIFI_FE_TXRX, value);
    value = qtest_readl(q, WIFI_FE_TXRX);
    value = (value & 0xfffffcff) | 0x0200;
    qtest_writel(q, WIFI_FE_TXRX, value);
    g_assert_cmphex(qtest_readl(q, WIFI_FE_TXRX), ==, 0xa5a56a5a);

    /* The same supplied ELF's fe_reg_init() performs these RMW operations
     * on FE/FE2. These checks preserve unrelated bits and cover both APB
     * aliases; they make no claim about private analog semantics. */
    qtest_writel(q, WIFI_FE + 0x0b8, 0x13572468);
    qtest_writel(q, WIFI_FE + 0x0b8,
                 qtest_readl(q, WIFI_FE_APB + 0x0b8) | 0x20000000);
    g_assert_cmphex(qtest_readl(q, WIFI_FE + 0x0b8), ==, 0x33572468);
    qtest_writel(q, WIFI_FE + 0x09c,
                 qtest_readl(q, WIFI_FE_APB + 0x09c) | 0x4000);
    g_assert_cmphex(qtest_readl(q, WIFI_FE + 0x09c), ==, 0x4000);
    qtest_writel(q, WIFI_FE + 0x04c, 0x12345678);
    qtest_writel(q, WIFI_FE + 0x04c,
                 (qtest_readl(q, WIFI_FE_APB + 0x04c) & 0xffff00ff) |
                 0xc800);
    g_assert_cmphex(qtest_readl(q, WIFI_FE + 0x04c), ==, 0x1234c878);
    qtest_writel(q, WIFI_FE2 + 0x114,
                 qtest_readl(q, WIFI_FE2_APB + 0x114) | 0x200);
    qtest_writel(q, WIFI_FE2 + 0x0dc,
                 qtest_readl(q, WIFI_FE2_APB + 0x0dc) | 0xc0000000);
    qtest_writel(q, WIFI_FE2 + 0x0d8,
                 qtest_readl(q, WIFI_FE2_APB + 0x0d8) | 0x02000000);
    g_assert_cmphex(qtest_readl(q, WIFI_FE2 + 0x114), ==, 0x200);
    g_assert_cmphex(qtest_readl(q, WIFI_FE2 + 0x0dc), ==, 0xc0000000);
    g_assert_cmphex(qtest_readl(q, WIFI_FE2 + 0x0d8), ==, 0x02000000);

    /* The supplied ELF's ram_set_pbus_mem() bit-serializes calibration
     * values through this sparse FE/FE2 register set. */
    qtest_writel(q, WIFI_FE + 0x30, 0x12345678);
    qtest_writel(q, WIFI_FE + 0x34, 0x87654321);
    qtest_writel(q, WIFI_FE + 0x38, 0x00010000);
    qtest_writel(q, WIFI_FE + 0x3c, 0x00020000);
    qtest_writel(q, WIFI_FE + 0x40, 0x00030000);
    qtest_writel(q, WIFI_FE + 0x44, 0x00040000);
    qtest_writel(q, WIFI_FE2 + 0x34, 0x00050000);
    qtest_writel(q, WIFI_FE2 + 0x38, 0x00060000);
    g_assert_cmphex(qtest_readl(q, WIFI_FE_APB + 0x30), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(q, WIFI_FE_APB + 0x34), ==, 0x87654321);
    g_assert_cmphex(qtest_readl(q, WIFI_FE_APB + 0x38), ==, 0x00010000);
    g_assert_cmphex(qtest_readl(q, WIFI_FE_APB + 0x3c), ==, 0x00020000);
    g_assert_cmphex(qtest_readl(q, WIFI_FE_APB + 0x40), ==, 0x00030000);
    g_assert_cmphex(qtest_readl(q, WIFI_FE_APB + 0x44), ==, 0x00040000);
    g_assert_cmphex(qtest_readl(q, WIFI_FE2_APB + 0x34), ==, 0x00050000);
    g_assert_cmphex(qtest_readl(q, WIFI_FE2_APB + 0x38), ==, 0x00060000);

    /* FE is an independent DPORT reset bit (IDF's modem power-domain reset
     * mask deliberately excludes it). Assert it clears only the sparse FE
     * register state; it is not coupled to the Wi-Fi MAC reset. */
    qtest_writel(q, DPORT_CORE_RST_EN, 1u << 1);
    g_assert_cmphex(qtest_readl(q, WIFI_FE + 0x0b8), ==, 0);
    g_assert_cmphex(qtest_readl(q, WIFI_FE2 + 0x114), ==, 0);
    g_assert_cmphex(qtest_readl(q, WIFI_FE + 0x30), ==, 0);
    g_assert_cmphex(qtest_readl(q, WIFI_FE2 + 0x34), ==, 0);
    qtest_writel(q, DPORT_CORE_RST_EN, 0);

    qtest_qmp_assert_success(q, "{ 'execute': 'system_reset' }");
    g_assert_cmphex(qtest_readl(q, WIFI_FE_TXRX), ==, 0);
    g_assert_cmphex(qtest_readl(q, WIFI_FE + 0x0b8), ==, 0);
    g_assert_cmphex(qtest_readl(q, WIFI_FE2 + 0x114), ==, 0);
    qtest_quit(q);
}

static void mmio_present_without_network_backend(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");

    qtest_writel(q, WIFI_DPORT + WIFI_BSSID0, 0xa1b2c3d4);
    g_assert_cmphex(qtest_readl(q, WIFI_APB + WIFI_BSSID0), ==, 0xa1b2c3d4);
    qtest_writel(q, WIFI_APB + WIFI_MAC0, 0x10203040);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_MAC0), ==, 0x10203040);
    qtest_quit(q);
}

static void mmio_requires_aligned_words(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");
    const uint32_t address = WIFI_DPORT + WIFI_BSSID0;

    qtest_writel(q, address, 0x12345678);
    g_assert_cmphex(qtest_readb(q, address), ==, 0);
    g_assert_cmphex(qtest_readl(q, address + 1), ==, 0);
    qtest_writeb(q, address, 0xa5);
    qtest_writel(q, address + 1, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(q, address), ==, 0x12345678);
    qtest_quit(q);
}

static void phy_bt_ifs_word_and_reset(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");

    qtest_writel(q, ESP32_WIFI_PHY_BT_IFS_ADDR, 0xa5a5a5a5);
    g_assert_cmphex(qtest_readl(q, ESP32_WIFI_PHY_BT_IFS_ADDR), ==,
                    0xa5a5a5a5);
    qtest_qmp_assert_success(q, "{ 'execute': 'system_reset' }");
    qtest_qmp_eventwait(q, "RESET");
    g_assert_cmphex(qtest_readl(q, ESP32_WIFI_PHY_BT_IFS_ADDR), ==, 0);
    qtest_quit(q);
}

static void phy_coex_words_are_sparse_and_reset(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");
    static const uint32_t addr[] = {
        ESP32_WIFI_COEX_APB_BASE,
        ESP32_WIFI_COEX_APB_BASE + 0x230,
        ESP32_WIFI_COEX_APB_BASE + 0x2c60,
        ESP32_WIFI_COEX_APB_BASE + 0x2c68,
        ESP32_WIFI_COEX_APB_BASE + 0x2c70,
    };

    for (unsigned i = 0; i < ARRAY_SIZE(addr); i++) {
        qtest_writel(q, addr[i], 0xa5000000 | i);
        g_assert_cmphex(qtest_readl(q, addr[i]), ==, 0xa5000000 | i);
    }
    /* These singleton registers overlap the Wi-Fi APB mapping. Ensure the
     * unmodeled intervening address still reaches the MAC window. */
    qtest_writel(q, 0x60033000 + WIFI_BSSID0, 0x12345678);
    g_assert_cmphex(qtest_readl(q, 0x60033000 + WIFI_BSSID0), ==,
                    0x12345678);
    qtest_qmp_assert_success(q, "{ 'execute': 'system_reset' }");
    qtest_qmp_eventwait(q, "RESET");
    for (unsigned i = 0; i < ARRAY_SIZE(addr); i++) {
        g_assert_cmphex(qtest_readl(q, addr[i]), ==, 0);
    }
    qtest_quit(q);
}

static void fe_pbus_command_busy_and_reset(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");
    uint32_t command = WIFI_FE + ESP32_WIFI_FE_PBUS_CMD;
    uint32_t status = WIFI_FE + ESP32_WIFI_FE_TXRX_CONTROL;

    /* The IDF helper starts a PBUS transaction on command bit 1's rising
     * edge and polls status bit 31 until the transaction completes. */
    qtest_writel(q, command, 0x00113cf1);
    qtest_writel(q, command, 0x00113cf3);
    g_assert_true(qtest_readl(q, status) & BIT(31));
    qtest_clock_step(q, 1000);
    g_assert_false(qtest_readl(q, status) & BIT(31));

    qtest_writel(q, command, 0x00113cf1);
    qtest_writel(q, command, 0x00113cf3);
    g_assert_true(qtest_readl(q, status) & BIT(31));
    qtest_writel(q, DPORT_CORE_RST_EN, 1u << ESP32_WIFI_FE_RESET_BIT);
    g_assert_false(qtest_readl(q, status) & BIT(31));
    qtest_clock_step(q, 1000);
    g_assert_false(qtest_readl(q, status) & BIT(31));
    qtest_quit(q);
}

static void txdc_program_candidate(QTestState *q, unsigned selector,
                                   unsigned index, unsigned candidate);

static void txdc_pbus_completion_requires_phy_clocks(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");
    uint32_t command = ESP32_WIFI_TXDC_PBUS_BASE;

    qtest_writel(q, command, 0x00113cf1);
    qtest_writel(q, command, 0x00113cf3);
    qtest_clock_step(q, 2000);
    g_assert_false(qtest_readl(q, ESP32_WIFI_TXDC_PBUS_APB) & BIT(24));

    /* PHY clock outputs come from the documented DPORT clock register.
     * The command completes only when all observed Wi-Fi PHY clock enables
     * are active and FE reset is deasserted. */
    qtest_writel(q, DPORT_WIFI_CLK_EN, 0x00008f8f);
    txdc_program_candidate(q, 2, 1, 0x100);
    txdc_program_candidate(q, 3, 1, 0x100);
    qtest_writel(q, command, 0x00113cf1);
    qtest_writel(q, command, 0x00113cf3);
    g_assert_false(qtest_readl(q, command) & BIT(24));
    qtest_clock_step(q, 1000);
    g_assert_true(qtest_readl(q, ESP32_WIFI_TXDC_PBUS_APB) & BIT(24));
    g_assert_false(qtest_readl(q, command) & BIT(31));

    qtest_writel(q, DPORT_CORE_RST_EN, 1u << ESP32_WIFI_FE_RESET_BIT);
    g_assert_cmphex(qtest_readl(q, command), ==, 0);
    qtest_writel(q, DPORT_CORE_RST_EN, 0);
    qtest_quit(q);
}

static void txdc_program_candidate(QTestState *q, unsigned selector,
                                   unsigned index, unsigned candidate)
{
    uint32_t command = WIFI_FE + ESP32_WIFI_FE_PBUS_CMD;
    uint32_t value = (selector << 2) | ((candidate & 0x1ff) << 6) |
                     (index << 15);

    /* ram_pbus_force_test(a2, a3, a4) encodes the selector, command index,
     * and nine-bit value, then raises bit 1 and waits for FE+0xa0 busy to
     * clear. */
    qtest_writel(q, command, value);
    qtest_writel(q, command, value | BIT(1));
    g_assert_true(qtest_readl(q, WIFI_FE + ESP32_WIFI_FE_TXRX_CONTROL) &
                  BIT(31));
    qtest_clock_step(q, 1000);
    g_assert_false(qtest_readl(q, WIFI_FE + ESP32_WIFI_FE_TXRX_CONTROL) &
                   BIT(31));
    qtest_writel(q, command, value);
}

static void txdc_pbus_measures_programmed_candidates(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");
    uint32_t measurement = ESP32_WIFI_TXDC_PBUS_BASE;

    qtest_writel(q, DPORT_WIFI_CLK_EN, 0x00008f8f);
    txdc_program_candidate(q, 2, 1, 255);
    txdc_program_candidate(q, 3, 1, 255);
    qtest_writel(q, measurement, 0x00113cf1);
    qtest_writel(q, measurement, 0x00113cf3);
    qtest_writel(q, DPORT_WIFI_CLK_EN, 0);
    qtest_clock_step(q, 1000);
    g_assert_false(qtest_readl(q, ESP32_WIFI_TXDC_PBUS_APB) & BIT(24));
    qtest_writel(q, DPORT_WIFI_CLK_EN, 0x00008f8f);
    qtest_clock_step(q, 1000);
    g_assert_true(qtest_readl(q, ESP32_WIFI_TXDC_PBUS_APB) & BIT(24));
    /* Both codewords below the nominal midpoint leave a positive residual. */
    g_assert_false(qtest_readl(q, measurement) & BIT(31));

    txdc_program_candidate(q, 2, 2, 257);
    txdc_program_candidate(q, 3, 2, 257);
    qtest_writel(q, measurement, 0x00113cf1);
    qtest_writel(q, measurement, 0x00113cf3);
    qtest_clock_step(q, 1000);
    g_assert_true(qtest_readl(q, ESP32_WIFI_TXDC_PBUS_APB) & BIT(24));
    /* Above the midpoint the same physical measurement path reports a
     * negative residual; this is the direction bit consumed by the ELF. */
    g_assert_cmphex(qtest_readl(q, measurement) & (BIT(31) | BIT(30)), ==,
                    BIT(31) | BIT(30));

    txdc_program_candidate(q, 2, 1, 255);
    txdc_program_candidate(q, 3, 1, 257);
    qtest_writel(q, measurement, 0x00113cf1);
    qtest_writel(q, measurement, 0x00113cf3);
    qtest_clock_step(q, 1000);
    g_assert_cmphex(qtest_readl(q, measurement) & (BIT(31) | BIT(30)), ==,
                    BIT(30));

    txdc_program_candidate(q, 2, 1, 257);
    txdc_program_candidate(q, 3, 1, 255);
    qtest_writel(q, measurement, 0x00113cf1);
    qtest_writel(q, measurement, 0x00113cf3);
    qtest_clock_step(q, 1000);
    g_assert_cmphex(qtest_readl(q, measurement) & (BIT(31) | BIT(30)), ==,
                    BIT(31));

    /* The two candidate channels are measured independently; the selector
     * index is retained as a command field but does not gate completion. */
    txdc_program_candidate(q, 2, 3, 255);
    txdc_program_candidate(q, 3, 2, 257);
    qtest_writel(q, measurement, 0x00113cf1);
    qtest_writel(q, measurement, 0x00113cf3);
    qtest_clock_step(q, 1000);
    g_assert_true(qtest_readl(q, measurement) & BIT(24));
    g_assert_false(qtest_readl(q, measurement) & BIT(31));

    /* FE reset clears the programmed measurement inputs, so it cannot
     * manufacture a completed residual from stale candidate state. */
    qtest_writel(q, DPORT_CORE_RST_EN, 1u << ESP32_WIFI_FE_RESET_BIT);
    qtest_writel(q, DPORT_CORE_RST_EN, 0);
    qtest_writel(q, measurement, 0x00113cf1);
    qtest_writel(q, measurement, 0x00113cf3);
    qtest_clock_step(q, 2000);
    g_assert_false(qtest_readl(q, measurement) & BIT(24));
    qtest_quit(q);
}

static void rfpll_frequency_command_clock_and_reset(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");
    uint32_t tune = ESP32_WIFI_RFPLL_FREQ_BASE;

    /* The IDF set_channel(1) path stores 0x18 in C4 and waits on the Wi-Fi
     * clock domain for completion. */
    qtest_writel(q, tune, 0x18 | BIT(8));
    qtest_clock_step(q, 5000);
    g_assert_cmphex(qtest_readl(q, tune), ==, 0x18 | BIT(8));

    qtest_writel(q, DPORT_WIFI_CLK_EN,
                 DPORT_WIFI_CLK_RESET | DPORT_WIFI_MAC_CLOCKS);
    qtest_clock_step(q, 1999);
    g_assert_cmphex(qtest_readl(q, ESP32_WIFI_RFPLL_FREQ_APB), ==,
                    0x18 | BIT(8));
    qtest_clock_step(q, 1);
    g_assert_cmphex(qtest_readl(q, tune), ==, 0x80000018);

    /* An unsupported code must not inherit the prior command's ready bit. */
    qtest_writel(q, tune, 0x01 | BIT(8));
    qtest_clock_step(q, 2000);
    g_assert_cmphex(qtest_readl(q, tune), ==, 0x00000001);

    /* Reset during a command cancels the virtual timer and clears status. */
    qtest_writel(q, tune, 0x22 | BIT(8));
    qtest_qmp_assert_success(q, "{ 'execute': 'system_reset' }");
    qtest_qmp_eventwait(q, "RESET");
    qtest_clock_step(q, 5000);
    g_assert_cmphex(qtest_readl(q, tune), ==, 0);
    qtest_quit(q);
}

static void rfpll_tune_selects_only_matching_peer_channel(void)
{
    QTestState *q = qtest_init("-machine esp32,wifi-peer-ssid=test-ap,wifi-peer-channel=1 "
                               "-display none -serial none "
                               "-nic user,model=misc.esp32_wifi");
    uint32_t descriptor[3] = { 0x800007ff, 0x3ffb0200, 0 };
    uint8_t received[96];
    uint32_t control;

    wifi_enable_mac(q);
    qtest_writel(q, ESP32_WIFI_RFPLL_FREQ_BASE, 0x18 | BIT(8));
    qtest_clock_step(q, 2000);
    g_assert_cmphex(qtest_readl(q, ESP32_WIFI_RFPLL_FREQ_BASE), ==,
                    0x80000018);
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor,
                   sizeof(descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_INLINK, 0x3ffb0000);
    qtest_clock_step(q, 100000000);
    control = qtest_readl(q, 0x3ffb0000);
    g_assert_cmpuint((control >> 12) & 0xfff, >, 28);
    qtest_memread(q, 0x3ffb0200, received, sizeof(received));
    g_assert_cmphex(received[28], ==, 0x80); /* channel-1 peer beacon */

    /* Channel 2 is a valid tune but cannot make the channel-1 peer visible. */
    qtest_qmp_assert_success(q, "{ 'execute': 'system_reset' }");
    qtest_qmp_eventwait(q, "RESET");
    wifi_enable_mac(q);
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor,
                   sizeof(descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_INLINK, 0x3ffb0000);
    qtest_writel(q, ESP32_WIFI_RFPLL_FREQ_BASE, 0x22 | BIT(8));
    qtest_clock_step(q, 100000000);
    g_assert_cmphex(qtest_readl(q, ESP32_WIFI_RFPLL_FREQ_BASE), ==,
                    0x80000022);
    g_assert_cmphex(qtest_readl(q, 0x3ffb0000), ==, descriptor[0]);
    qtest_quit(q);
}

static void rfpll_tune_supports_channel_fourteen(void)
{
    QTestState *q = qtest_init("-machine esp32,wifi-peer-ssid=test-ap,wifi-peer-channel=14 "
                               "-display none -serial none "
                               "-nic user,model=misc.esp32_wifi");
    uint32_t descriptor[3] = { 0x800007ff, 0x3ffb0200, 0 };
    uint8_t received[96];

    wifi_enable_mac(q);
    qtest_writel(q, ESP32_WIFI_RFPLL_FREQ_BASE, 0xa8 | BIT(8));
    qtest_clock_step(q, 2000);
    g_assert_cmphex(qtest_readl(q, ESP32_WIFI_RFPLL_FREQ_BASE), ==,
                    0x800000a8);
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor,
                   sizeof(descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_INLINK, 0x3ffb0000);
    qtest_clock_step(q, 100000000);
    g_assert_cmpuint((qtest_readl(q, 0x3ffb0000) >> 12) & 0xfff, >, 28);
    qtest_memread(q, 0x3ffb0200, received, sizeof(received));
    g_assert_cmphex(received[28], ==, 0x80);
    qtest_quit(q);
}

static void fe_iq_estimator_requires_enable_and_clocks(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");
    uint32_t iq = WIFI_FE + ESP32_WIFI_FE_IQ_EST;

    qtest_writel(q, WIFI_FE + ESP32_WIFI_FE_CTRL_060, BIT(26));
    qtest_writel(q, iq, BIT(0));
    qtest_writel(q, iq, BIT(0) | BIT(1));
    qtest_clock_step(q, 2000);
    g_assert_false(qtest_readl(q, iq) & BIT(31));

    qtest_writel(q, DPORT_WIFI_CLK_EN, 0x00008f8f);
    qtest_writel(q, iq, BIT(0));
    qtest_writel(q, iq, BIT(0) | BIT(1));
    g_assert_false(qtest_readl(q, iq) & BIT(31));
    qtest_clock_step(q, 1000);
    g_assert_true(qtest_readl(q, iq) & BIT(31));

    qtest_writel(q, DPORT_CORE_RST_EN, 1u << ESP32_WIFI_FE_RESET_BIT);
    g_assert_false(qtest_readl(q, iq) & BIT(31));
    qtest_qmp_assert_success(q, "{ 'execute': 'system_reset' }");
    qtest_qmp_eventwait(q, "RESET");
    g_assert_false(qtest_readl(q, iq) & BIT(31));
    qtest_quit(q);
}

static void extended_mac_window_aliases(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");
    /* IDF's hal_crypto_set_key_entry accesses the hardware key table at
     * 0x3ff74400 (Wi-Fi MAC offset 0x1400).  Verify that both documented
     * views reach the same backing register bank. */
    qtest_writel(q, WIFI_DPORT + WIFI_CRYPTO_KEY_TABLE, 0x12345678);
    g_assert_cmphex(qtest_readl(q, WIFI_APB + WIFI_CRYPTO_KEY_TABLE), ==,
                    0x12345678);
    qtest_writel(q, WIFI_APB + WIFI_CRYPTO_KEY_TABLE + 4, 0x90abcdef);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_CRYPTO_KEY_TABLE + 4),
                    ==, 0x90abcdef);
    /* Supplied IDF 6.1's libpp helper uses a 40-byte entry stride: two
     * metadata words followed by key bytes at offset 8, and a corresponding
     * valid bit. Check adjacent entries stay distinct. This tests register
     * backing, not the private crypto engine's use or CCMP encryption. */
    qtest_writel(q, WIFI_DPORT + WIFI_CRYPTO_KEY_TABLE + 8,
                 0x01020304);
    qtest_writel(q, WIFI_DPORT + WIFI_CRYPTO_KEY_TABLE + 36,
                 0x05060708);
    qtest_writel(q, WIFI_DPORT + WIFI_CRYPTO_KEY_TABLE + 40,
                 0x11121314);
    g_assert_cmphex(qtest_readl(q, WIFI_APB + WIFI_CRYPTO_KEY_TABLE + 8), ==,
                    0x01020304);
    g_assert_cmphex(qtest_readl(q, WIFI_APB + WIFI_CRYPTO_KEY_TABLE + 36), ==,
                    0x05060708);
    g_assert_cmphex(qtest_readl(q, WIFI_APB + WIFI_CRYPTO_KEY_TABLE + 40), ==,
                    0x11121314);
    qtest_writel(q, WIFI_DPORT + WIFI_CRYPTO_KEY_VALID, 1 << 0);
    qtest_writel(q, WIFI_DPORT + WIFI_CRYPTO_KEY_VALID,
                 qtest_readl(q, WIFI_DPORT + WIFI_CRYPTO_KEY_VALID) | (1 << 7));
    g_assert_cmphex(qtest_readl(q, WIFI_APB + WIFI_CRYPTO_KEY_VALID), ==,
                    (1 << 0) | (1 << 7));
    /* Clear-key code in the same ELF clears one validity bit by RMW. */
    qtest_writel(q, WIFI_DPORT + WIFI_CRYPTO_KEY_VALID,
                 qtest_readl(q, WIFI_DPORT + WIFI_CRYPTO_KEY_VALID) & ~(1 << 0));
    g_assert_cmphex(qtest_readl(q, WIFI_APB + WIFI_CRYPTO_KEY_VALID), ==,
                    1 << 7);
    /* The binary hal_crypto_enable() helper also updates 0x800/804/808 and
     * 0x810. Preserve these register values through both Wi-Fi aliases. */
    qtest_writel(q, WIFI_DPORT + 0x800, 0x01234567);
    qtest_writel(q, WIFI_APB + 0x804, 0x89abcdef);
    qtest_writel(q, WIFI_DPORT + 0x808, 0x76543210);
    qtest_writel(q, WIFI_APB + 0x810, 0xa5a55a5a);
    g_assert_cmphex(qtest_readl(q, WIFI_APB + 0x800), ==, 0x01234567);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + 0x804), ==, 0x89abcdef);
    g_assert_cmphex(qtest_readl(q, WIFI_APB + 0x808), ==, 0x76543210);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + 0x810), ==, 0xa5a55a5a);
    qtest_writel(q, WIFI_DPORT + WIFI_CRYPTO_KEY_TABLE + 7 * 40,
                 0xfeedc0de);
    g_assert_cmphex(qtest_readl(q, WIFI_APB + WIFI_CRYPTO_KEY_TABLE + 7 * 40),
                    ==, 0xfeedc0de);
    /* Index 31 exercises the top bitmap bit and final word in the 32-entry
     * region. The 40-byte stride keeps this key payload inside the aperture. */
    qtest_writel(q, WIFI_APB + WIFI_CRYPTO_KEY_TABLE + 31 * 40 + 8,
                 0x89abcdef);
    qtest_writel(q, WIFI_APB + WIFI_CRYPTO_KEY_TABLE + 31 * 40 + 36,
                 0x76543210);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_CRYPTO_KEY_TABLE +
                                31 * 40 + 8), ==, 0x89abcdef);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_CRYPTO_KEY_TABLE +
                                31 * 40 + 36), ==, 0x76543210);
    qtest_writel(q, WIFI_APB + WIFI_CRYPTO_KEY_VALID,
                 qtest_readl(q, WIFI_APB + WIFI_CRYPTO_KEY_VALID) |
                 (1U << 31));
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_CRYPTO_KEY_VALID), ==,
                    (1U << 31) | (1U << 7));
    qtest_writel(q, WIFI_DPORT + WIFI_MMIO_LAST_WORD, 0xa5a55a5a);
    g_assert_cmphex(qtest_readl(q, WIFI_APB + WIFI_MMIO_LAST_WORD), ==,
                    0xa5a55a5a);
    qtest_quit(q);
}

static void extended_mac_window_reset(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");
    uint32_t tx_descriptor[3] = {
        0x80000000 | (48 << 12) | 48, 0x3ffb0100, 0,
    };
    uint32_t rx_descriptor[3] = { 0x800007ff, 0x3ffb0200, 0 };
    uint8_t beacon[48] = {
        0x80, 0x00, 0, 0,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0x02, 0x12, 0x34, 0x56, 0x78, 0x9a,
        0x02, 0x12, 0x34, 0x56, 0x78, 0x9a,
        0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0x64, 0, 1, 0,
        0, 7, 'M', 'O', 'D', 'U', 'A', 'M', 'P', 3, 1, 1,
    };

    qtest_irq_intercept_out_named(q, INTMATRIX_PATH, "cpu-irq");
    qtest_writel(q, MATRIX, 1);
    wifi_enable_mac(q);
    qtest_writel(q, WIFI_DPORT + WIFI_CRYPTO_KEY_TABLE + 31 * 40 + 36,
                 0xdeadbeef);
    qtest_writel(q, WIFI_DPORT + WIFI_CRYPTO_KEY_VALID, 1U << 31);
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)rx_descriptor,
                   sizeof(rx_descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_INLINK, 0x3ffb0000);
    qtest_memwrite(q, 0x3ffb0100, beacon, sizeof(beacon));
    qtest_memwrite(q, 0x3ffb1000, (uint8_t *)tx_descriptor,
                   sizeof(tx_descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b1000);
    g_assert_true(qtest_get_irq(q, 1));

    qtest_qmp_assert_success(q, "{ 'execute': 'system_reset' }");
    qtest_qmp_eventwait(q, "RESET");
    g_assert_cmphex(qtest_readl(q, WIFI_APB + WIFI_CRYPTO_KEY_TABLE), ==, 0);
    g_assert_cmphex(qtest_readl(q, WIFI_APB + WIFI_CRYPTO_KEY_TABLE +
                                31 * 40 + 36), ==, 0);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_CRYPTO_KEY_VALID), ==, 0);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0);
    g_assert_false(qtest_get_irq(q, 1));
    qtest_clock_step(q, 5000000);
    g_assert_cmphex(qtest_readl(q, 0x3ffb0000), ==, 0x800007ff);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS) &
                    0x01000024, ==, 0);
    qtest_quit(q);
}

static void tx_dma_interrupt(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic user,model=misc.esp32_wifi");
    uint32_t descriptor[3] = { 0x80018020, 0x3ffb0100, 0 };
    uint8_t frame[24] = { [0] = 8 }; /* IEEE 802.11 data frame, subtype 0 */

    qtest_irq_intercept_out_named(q, INTMATRIX_PATH, "cpu-irq");
    qtest_writel(q, MATRIX, 1); /* Wi-Fi MAC source 0 -> CPU interrupt 1 */
    wifi_enable_mac(q);
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor, sizeof(descriptor));
    qtest_memwrite(q, 0x3ffb0100, frame, sizeof(frame));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);

    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0x80);
    g_assert_true(qtest_get_irq(q, 1));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_INT_CLR, 0x80);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0);
    g_assert_false(qtest_get_irq(q, 1));
    qtest_quit(q);
}

static void mac_clock_reset_gates_dma(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic none");
    uint32_t descriptor[3] = { 0x80018020, 0x3ffb0100, 0 };
    uint8_t frame[24] = { [0] = 8 };

    qtest_irq_intercept_out_named(q, INTMATRIX_PATH, "cpu-irq");
    qtest_writel(q, MATRIX, 1);
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor, sizeof(descriptor));
    qtest_memwrite(q, 0x3ffb0100, frame, sizeof(frame));

    /* The reset default lacks the Wi-Fi MAC clock; descriptor writes cannot
     * start DMA or raise its completion IRQ. */
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0);
    g_assert_false(qtest_get_irq(q, 1));

    qtest_writel(q, DPORT_WIFI_CLK_EN,
                 DPORT_WIFI_CLK_RESET | DPORT_WIFI_MAC_CLOCKS);
    qtest_writel(q, DPORT_CORE_RST_EN, DPORT_WIFIMAC_RST);
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0);
    g_assert_false(qtest_get_irq(q, 1));

    qtest_writel(q, DPORT_CORE_RST_EN, 0);
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0x80);
    g_assert_true(qtest_get_irq(q, 1));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_INT_CLR, 0x80);
    g_assert_false(qtest_get_irq(q, 1));

    /* Asserting WIFIMAC reset clears the in-flight MAC state and IRQ; release
     * permits a new transfer while its clock remains enabled. */
    qtest_writel(q, DPORT_CORE_RST_EN, DPORT_WIFIMAC_RST);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0);
    g_assert_false(qtest_get_irq(q, 1));
    qtest_writel(q, DPORT_CORE_RST_EN, 0);
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0x80);
    g_assert_true(qtest_get_irq(q, 1));

    /* System reset restores the documented DPORT clock default. The MAC
     * remains gated until the guest enables its own clock again. */
    qtest_qmp_assert_success(q, "{ 'execute': 'system_reset' }");
    qtest_qmp_eventwait(q, "RESET");
    g_assert_cmphex(qtest_readl(q, DPORT_WIFI_CLK_EN), ==,
                    DPORT_WIFI_CLK_RESET);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0);
    g_assert_false(qtest_get_irq(q, 1));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0);
    wifi_enable_mac(q);
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0x80);
    qtest_quit(q);
}

static void tx_dma_descriptor_bounds(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic user,model=misc.esp32_wifi");
    uint8_t data_frame[24] = { [0] = 0x08 };
    uint8_t short_data_frame[2] = { [0] = 0x08 };
    uint8_t ack_frame[10] = { [0] = 0xd4 };
    uint32_t descriptor[3];

    wifi_enable_mac(q);

    qtest_memwrite(q, 0x3ffb0100, data_frame, sizeof(data_frame));
    qtest_memwrite(q, 0x3ffb0200, short_data_frame,
                   sizeof(short_data_frame));
    qtest_memwrite(q, 0x3ffb0300, ack_frame, sizeof(ack_frame));

    /* Zero length and a truncated data header must not read a frame or report
     * the successful TX interrupt used by completed descriptors. */
    descriptor[0] = 0x80000020;
    descriptor[1] = 0x3ffb0100;
    descriptor[2] = 0;
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor, sizeof(descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0);

    descriptor[0] = 0x80002020; /* data header length 2, capacity 32 */
    descriptor[1] = 0x3ffb0200;
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor, sizeof(descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0);

    /* The 12-bit descriptor length cannot enlarge the fixed model frame. */
    descriptor[0] = 0x80ffffff;
    descriptor[1] = 0x3ffb0100;
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor, sizeof(descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0);

    /* TX consumes the descriptor length. The supplied IDF's ppProcTxSecFrame
     * extends this field while preserving size; keep the guest read bounded
     * by internal RAM and the fixed frame, but do not treat RX capacity as
     * the TX byte count. */
    descriptor[0] = 0x80018017; /* length 24, size 23 */
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor, sizeof(descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==,
                    0x80);
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_INT_CLR, 0x80);

    /* A frame buffer outside internal DMA RAM is rejected before a read. */
    descriptor[0] = 0x80018020; /* length 24, size 32 */
    descriptor[1] = 0x60000000;
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor, sizeof(descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0);

    /* The shortest supported control frame is a 10-byte ACK.  A poisoned
     * next link is not followed by this single-descriptor transfer. */
    descriptor[0] = 0x8000a00a;
    descriptor[1] = 0x3ffb0300;
    descriptor[2] = UINT32_MAX;
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor, sizeof(descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==,
                    0x80);
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_INT_CLR, 0x80);

    descriptor[0] = 0x80018020;
    descriptor[1] = 0x3ffb0100;
    descriptor[2] = 0;
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor, sizeof(descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==,
                    0x80);
    qtest_quit(q);
}

static void guest_softap_beacon_enters_rx_dma(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic user,model=misc.esp32_wifi");
    uint32_t rx_descriptor[3] = { 0x800007ff, 0x3ffb0200, 0 };
    uint8_t beacon[48] = {
        0x80, 0x00, 0, 0,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0x02, 0x12, 0x34, 0x56, 0x78, 0x9a,
        0x02, 0x12, 0x34, 0x56, 0x78, 0x9a,
        0, 0,
        /* timestamp, beacon interval, capability */
        0, 0, 0, 0, 0, 0, 0, 0, 0x64, 0, 1, 0,
        /* SSID and DS parameter channel IEs */
        0, 7, 'M', 'O', 'D', 'U', 'A', 'M', 'P',
        3, 1, 1,
    };
    uint32_t tx_descriptor[3] = {
        0x80000000 | (sizeof(beacon) << 12) | sizeof(beacon),
        0x3ffb0100, 0,
    };
    uint32_t rx_control;
    uint8_t received[64];

    wifi_enable_mac(q);

    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)rx_descriptor,
                   sizeof(rx_descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_INLINK, 0x3ffb0000);
    qtest_memwrite(q, 0x3ffb0100, beacon, sizeof(beacon));
    qtest_memwrite(q, 0x3ffb1000, (uint8_t *)tx_descriptor,
                   sizeof(tx_descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b1000);

    /* The model queues an auth response to the guest beacon. Turning off the
     * MAC clock pauses its timer; the queued response is delivered through
     * the same RX DMA ring after the guest restores the clock. */
    qtest_writel(q, DPORT_WIFI_CLK_EN, DPORT_WIFI_CLK_RESET);
    qtest_clock_step(q, 5000000);
    g_assert_cmphex(qtest_readl(q, 0x3ffb0000), ==, 0x800007ff);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS) &
                    0x01000024, ==, 0);
    qtest_writel(q, DPORT_WIFI_CLK_EN,
                 DPORT_WIFI_CLK_RESET | DPORT_WIFI_MAC_CLOCKS);
    qtest_clock_step(q, 5000000);
    rx_control = qtest_readl(q, 0x3ffb0000);
    g_assert_cmphex(rx_control & 0x80000000, ==, 0);
    g_assert_cmpuint((rx_control >> 12) & 0xfff, >, 28);
    qtest_memread(q, 0x3ffb0200, received, sizeof(received));
    g_assert_cmphex(received[28], ==, 0xb0);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS) &
                    0x01000024, ==, 0x01000024);
    qtest_quit(q);
}

static void softap_arm_rx(QTestState *q)
{
    uint32_t descriptor[3] = { 0x800007ff, 0x3ffb0200, 0 };

    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor, sizeof(descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_INLINK, 0x3ffb0000);
}

static size_t softap_receive_frame(QTestState *q, uint8_t *frame,
                                   size_t capacity)
{
    uint32_t control;
    uint8_t rx[2048];
    size_t length;

    softap_arm_rx(q);
    qtest_clock_step(q, 5000000);
    control = qtest_readl(q, 0x3ffb0000);
    g_assert_cmphex(control & 0x80000000, ==, 0);
    length = (control >> 12) & 0xfff;
    g_assert_cmpuint(length, >, 28);
    g_assert_cmpuint(length - 28, <=, capacity);
    g_assert_cmpuint(length, <=, sizeof(rx));
    qtest_memread(q, 0x3ffb0200, rx, length);
    memcpy(frame, rx + 28, length - 28);
    return length - 28;
}

static void softap_transmit_frame(QTestState *q, const uint8_t *frame,
                                  size_t length)
{
    uint32_t descriptor[3] = {
        0x80000000 | (length << 12) | length, 0x3ffb1100, 0,
    };

    g_assert_cmpuint(length, <=, 1500);
    qtest_memwrite(q, 0x3ffb1100, frame, length);
    qtest_memwrite(q, 0x3ffb1000, (uint8_t *)descriptor, sizeof(descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b1000);
}

static void put_be16_test(uint8_t *p, uint16_t value)
{
    p[0] = value >> 8;
    p[1] = value;
}

static bool ipv4_checksum_valid(const uint8_t *header, size_t length)
{
    uint32_t sum = 0;

    for (size_t i = 0; i + 1 < length; i += 2) {
        sum += ((uint16_t)header[i] << 8) | header[i + 1];
    }
    while (sum >> 16) {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    return sum == 0xffff;
}

static uint32_t ieee_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xffffffff;

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1));
        }
    }
    return crc ^ 0xffffffff;
}

static size_t make_dhcp_reply(uint8_t *frame, uint8_t message_type,
                              const uint8_t xid[4])
{
    uint8_t *ip = frame + 32;
    uint8_t *udp = ip + 20;
    uint8_t *dhcp = udp + 8;
    const size_t dhcp_len = 250;
    const size_t ip_len = 20 + 8 + dhcp_len;
    const size_t frame_len = 24 + 8 + ip_len;

    memset(frame, 0, frame_len);
    frame[0] = 0x08; /* data */
    frame[1] = 0x02; /* From DS */
    memcpy(frame + 4, (uint8_t[]){0x24,0x6f,0x28,0x11,0x22,0x33}, 6);
    memcpy(frame + 10, (uint8_t[]){0x02,0x12,0x34,0x56,0x78,0x9a}, 6);
    memcpy(frame + 16, (uint8_t[]){192,168,4,2,0,0}, 6);
    memcpy(frame + 24, (uint8_t[]){0xaa,0xaa,3,0,0,0,8,0}, 8);
    ip[0] = 0x45;
    put_be16_test(ip + 2, ip_len);
    ip[8] = 64;
    ip[9] = 17;
    memcpy(ip + 12, (uint8_t[]){192,168,4,1}, 4);
    memset(ip + 16, 0xff, 4);
    put_be16_test(udp, 67);
    put_be16_test(udp + 2, 68);
    put_be16_test(udp + 4, 8 + dhcp_len);
    dhcp[0] = 2;
    dhcp[1] = 1;
    dhcp[2] = 6;
    memcpy(dhcp + 4, xid, 4);
    memcpy(dhcp + 16, (uint8_t[]){192,168,4,2}, 4); /* yiaddr */
    memcpy(dhcp + 20, (uint8_t[]){192,168,4,1}, 4); /* siaddr */
    memcpy(dhcp + 28, (uint8_t[]){0x10,0x01,0x00,0xc4,0x0a,0x24}, 6);
    memcpy(dhcp + 236, (uint8_t[]){0x63,0x82,0x53,0x63}, 4);
    if (message_type == 2) {
        memcpy(dhcp + 240, (uint8_t[]){53,1,2,54,4,192,168,4,1,255}, 10);
    } else {
        memcpy(dhcp + 240,
               (uint8_t[]){53,1,message_type,54,4,192,168,4,1,255}, 10);
    }
    return frame_len;
}

static size_t make_softap_arp_request(uint8_t *frame,
                                      const uint8_t target_ip[4])
{
    static const uint8_t ap[6] = {0x02,0x12,0x34,0x56,0x78,0x9a};
    uint8_t *arp = frame + 32;

    memset(frame, 0, 64);
    frame[0] = 0x08; /* data */
    frame[1] = 0x02; /* From DS */
    memset(frame + 4, 0xff, 6);
    memcpy(frame + 10, ap, 6);
    memset(frame + 16, 0xff, 6);
    memcpy(frame + 24, (uint8_t[]){0xaa,0xaa,3,0,0,0,8,6}, 8);
    put_be16_test(arp, 1);
    put_be16_test(arp + 2, 0x0800);
    arp[4] = 6;
    arp[5] = 4;
    put_be16_test(arp + 6, 1);
    memcpy(arp + 8, ap, 6);
    memcpy(arp + 14, (uint8_t[]){192,168,4,1}, 4);
    memcpy(arp + 24, target_ip, 4);
    return 64; /* 802.11 header + LLC/SNAP + ARP + FCS */
}

static void guest_softap_auth_assoc_dhcp_dma(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none "
                               "-nic user,model=misc.esp32_wifi");
    static const uint8_t ap[6] = {0x02,0x12,0x34,0x56,0x78,0x9a};
    static const uint8_t station[6] = {0x24,0x6f,0x28,0x11,0x22,0x33};
    static const uint8_t assoc_fixed[] = {0x21,0x04};
    static const uint8_t snap_ipv4[] = {0xaa,0xaa,3,0,0,0,8,0};
    static const uint8_t discover_option[] = {53,1,1};
    static const uint8_t request_option[] = {53,1,3};
    uint8_t beacon[48] = {
        0x80,0x00,0,0, 0xff,0xff,0xff,0xff,0xff,0xff,
        0x02,0x12,0x34,0x56,0x78,0x9a,
        0x02,0x12,0x34,0x56,0x78,0x9a, 0,0,
        0,0,0,0,0,0,0,0, 0x64,0,1,0,
        0,7,'M','O','D','U','A','M','P', 3,1,1,
    };
    uint8_t tx[512] = {0}, rx[2048];
    uint8_t xid[4];
    size_t tx_len, rx_len;
    const size_t dhcp_options = 24 + 8 + 20 + 8 + 240;

    wifi_enable_mac(q);

    /* Beacon -> open-system authentication request via the RX DMA ring. */
    tx_len = sizeof(beacon);
    softap_transmit_frame(q, beacon, tx_len);
    rx_len = softap_receive_frame(q, rx, sizeof(rx));
    g_assert_cmpuint(rx_len, >, 28);
    g_assert_cmphex(rx[0], ==, 0xb0);
    g_assert_cmphex(rx[26], ==, 1);

    /* Guest authentication response causes the station association request. */
    tx[0] = 0xb0;
    memcpy(tx + 4, ap, 6);
    memcpy(tx + 10, station, 6);
    memcpy(tx + 16, ap, 6);
    memcpy(tx + 24, (uint8_t[]){0,0,2,0,0,0}, 6);
    softap_transmit_frame(q, tx, 30);
    rx_len = softap_receive_frame(q, rx, sizeof(rx));
    g_assert_cmpuint(rx_len, >, 30);
    g_assert_cmphex(rx[0], ==, 0x00);
    g_assert_cmpmem(rx + 24, 2, assoc_fixed, sizeof(assoc_fixed));

    /* Successful association starts an actual guest DHCP DISCOVER. */
    memset(tx, 0, sizeof(tx));
    tx[0] = 0x10;
    memcpy(tx + 4, station, 6);
    memcpy(tx + 10, ap, 6);
    memcpy(tx + 16, ap, 6);
    memcpy(tx + 24, (uint8_t[]){0x31,0x04,0,0,1,0xc0}, 6);
    softap_transmit_frame(q, tx, 30);
    rx_len = softap_receive_frame(q, rx, sizeof(rx));
    g_assert_cmpuint(rx_len, >, dhcp_options + sizeof(discover_option));
    g_assert_cmphex(rx[0], ==, 0x08);
    g_assert_true(rx[1] & 1); /* To DS */
    g_assert_cmpmem(rx + 24, 8, snap_ipv4, sizeof(snap_ipv4));
    g_assert_cmpmem(rx + dhcp_options, sizeof(discover_option),
                    discover_option, sizeof(discover_option));
    memcpy(xid, rx + 24 + 8 + 20 + 8 + 4, sizeof(xid));
    g_assert_cmpmem(rx + 24 + 8 + 20 + 8 + 10, 2,
                    ((uint8_t[]){0x80, 0x00}), 2);

    /* Offer/ACK frames are parsed from From-DS traffic; the OFFER must
     * produce the guest's DHCP REQUEST through the same RX DMA path. */
    tx_len = make_dhcp_reply(tx, 2, xid);
    softap_transmit_frame(q, tx, tx_len);
    rx_len = softap_receive_frame(q, rx, sizeof(rx));
    g_assert_cmpuint(rx_len, >, dhcp_options + sizeof(request_option));
    g_assert_cmphex(rx[0], ==, 0x08);
    /* The Selecting DHCPREQUEST must retain the OFFER transaction identity
     * and client address, and name both the requested lease and selected
     * server. The IPv4 and UDP lengths must cover these exact options. */
    const uint8_t *request_ip = rx + 24 + 8;
    const uint8_t *request_udp = request_ip + 20;
    const uint8_t *request_dhcp = request_udp + 8;
    const uint8_t *request_opts = request_dhcp + 240;
    g_assert_cmpmem(request_dhcp + 4, 4,
                    xid, 4);
    g_assert_cmpmem(request_dhcp + 28, 6,
                    ((uint8_t[]){0x10,0x01,0x00,0xc4,0x0a,0x24}), 6);
    g_assert_cmpmem(request_dhcp + 10, 2,
                    ((uint8_t[]){0x80,0x00}), 2);
    g_assert_cmpmem(rx + 24 + 8 + 20 + 8 + 240,
                    sizeof(request_option), request_option,
                    sizeof(request_option));
    g_assert_cmpmem(request_opts + 7,
                    sizeof((uint8_t[]){50,4,192,168,4,2}),
                    ((uint8_t[]){50,4,192,168,4,2}), 6);
    g_assert_cmpmem(request_opts + 13,
                    sizeof((uint8_t[]){54,4,192,168,4,1}),
                    ((uint8_t[]){54,4,192,168,4,1}), 6);
    g_assert_cmphex(request_ip[0], ==, 0x45);
    g_assert_cmpuint((request_ip[2] << 8) | request_ip[3], ==,
                     rx_len - 24 - 8 - 4); /* trailing 802.11 FCS */
    g_assert_cmpuint((request_udp[4] << 8) | request_udp[5], ==,
                     rx_len - 24 - 8 - 20 - 4); /* trailing 802.11 FCS */
    g_assert_true(ipv4_checksum_valid(request_ip, 20));
    g_assert_cmpuint((request_udp[4] << 8) | request_udp[5], >=,
                     8 + 240 + 68);
    uint32_t fcs = ieee_crc32(rx, rx_len - 4);
    uint8_t expected_fcs[4] = {
        fcs, fcs >> 8, fcs >> 16, fcs >> 24,
    };
    g_assert_cmpmem(rx + rx_len - sizeof(expected_fcs),
                    sizeof(expected_fcs), expected_fcs,
                    sizeof(expected_fcs));

    /* A truncated OFFER without option 54 must not trigger SELECTING state. */
    tx_len = make_dhcp_reply(tx, 2, xid);
    tx[32 + 20 + 8 + 240 + 3] = 255; /* END replaces server-id option */
    softap_arm_rx(q);
    softap_transmit_frame(q, tx, tx_len);
    qtest_clock_step(q, 5000000);
    g_assert_true(qtest_readl(q, 0x3ffb0000) & 0x80000000);

    tx_len = make_dhcp_reply(tx, 5, xid);
    softap_transmit_frame(q, tx, tx_len);

    /* The station answers only for the IP it learned from the real guest
     * DHCP ACK.  An unknown target must not receive a synthetic reply. */
    tx_len = make_softap_arp_request(tx,
                                     (uint8_t[]){192,168,4,99});
    softap_arm_rx(q);
    softap_transmit_frame(q, tx, tx_len);
    qtest_clock_step(q, 5000000);
    g_assert_cmphex(qtest_readl(q, 0x3ffb0000), ==, 0x800007ff);

    /* The lease holder answers as a To-DS station frame through guest RX DMA. */
    tx_len = make_softap_arp_request(tx,
                                     (uint8_t[]){192,168,4,2});
    softap_transmit_frame(q, tx, tx_len);
    rx_len = softap_receive_frame(q, rx, sizeof(rx));
    g_assert_cmpuint(rx_len, >=, 60);
    g_assert_cmphex(rx[0], ==, 0x08);
    g_assert_cmphex(rx[1] & 0x03, ==, 0x01); /* To DS */
    g_assert_cmpmem(rx + 4, 6, ap, sizeof(ap));
    g_assert_cmpmem(rx + 10, 6,
                    ((uint8_t[]){0x10,0x01,0x00,0xc4,0x0a,0x24}), 6);
    g_assert_cmpmem(rx + 16, 6, ap, sizeof(ap));
    g_assert_cmpmem(rx + 24, 8,
                    ((uint8_t[]){0xaa,0xaa,3,0,0,0,8,6}), 8);
    g_assert_cmphex(rx[38], ==, 0);
    g_assert_cmphex(rx[39], ==, 2); /* ARP reply */
    g_assert_cmpmem(rx + 40, 6,
                    ((uint8_t[]){0x10,0x01,0x00,0xc4,0x0a,0x24}), 6);
    g_assert_cmpmem(rx + 46, 4, ((uint8_t[]){192,168,4,2}), 4);
    g_assert_cmpmem(rx + 50, 6, ap, sizeof(ap));
    g_assert_cmpmem(rx + 56, 4, ((uint8_t[]){192,168,4,1}), 4);
    qtest_quit(q);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32/wifi/mmio-aliases", mmio_aliases);
    qtest_add_func("/esp32/wifi/wdev-tsf-latch-clock-alias-reset",
                   wdev_tsf_latch_clock_alias_reset);
    qtest_add_func("/esp32/wifi/rxctrl-agc-rmw-alias-reset",
                   rxctrl_agc_rmw_aliases_and_reset);
    qtest_add_func("/esp32/wifi/fe-txrx-force-mode-rmw-reset",
                   fe_txrx_force_mode_rmw_and_reset);
    qtest_add_func("/esp32/wifi/fe-pbus-command-busy-reset",
                   fe_pbus_command_busy_and_reset);
    qtest_add_func("/esp32/wifi/txdc-pbus-clock-completion-reset",
                   txdc_pbus_completion_requires_phy_clocks);
    qtest_add_func("/esp32/wifi/txdc-pbus-programmed-candidates",
                   txdc_pbus_measures_programmed_candidates);
    qtest_add_func("/esp32/wifi/rfpll-frequency-command-clock-reset",
                   rfpll_frequency_command_clock_and_reset);
    qtest_add_func("/esp32/wifi/rfpll-tune-selects-peer-channel",
                   rfpll_tune_selects_only_matching_peer_channel);
    qtest_add_func("/esp32/wifi/rfpll-tune-channel-fourteen",
                   rfpll_tune_supports_channel_fourteen);
    qtest_add_func("/esp32/wifi/fe-iq-estimator-enable-clock-reset",
                   fe_iq_estimator_requires_enable_and_clocks);
    qtest_add_func("/esp32/wifi/extended-mac-window-aliases",
                   extended_mac_window_aliases);
    qtest_add_func("/esp32/wifi/extended-mac-window-reset",
                   extended_mac_window_reset);
    qtest_add_func("/esp32/wifi/mmio-without-network-backend",
                   mmio_present_without_network_backend);
    qtest_add_func("/esp32/wifi/mmio-aligned-word-access",
                   mmio_requires_aligned_words);
    qtest_add_func("/esp32/wifi/phy-bt-ifs-word-reset",
                   phy_bt_ifs_word_and_reset);
    qtest_add_func("/esp32/wifi/phy-coex-sparse-words-reset",
                   phy_coex_words_are_sparse_and_reset);
    qtest_add_func("/esp32/wifi/tx-dma-interrupt-clear", tx_dma_interrupt);
    qtest_add_func("/esp32/wifi/mac-clock-reset-gates-dma",
                   mac_clock_reset_gates_dma);
    qtest_add_func("/esp32/wifi/tx-dma-descriptor-bounds",
                   tx_dma_descriptor_bounds);
    qtest_add_func("/esp32/wifi/guest-softap-beacon-rx-dma",
                   guest_softap_beacon_enters_rx_dma);
    qtest_add_func("/esp32/wifi/guest-softap-auth-assoc-dhcp-dma",
                   guest_softap_auth_assoc_dhcp_dma);
    return g_test_run();
}
