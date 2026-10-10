/* Original ESP32 Wi-Fi MMIO mapping checks.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"
#include "hw/xtensa/esp32_wifi.h"

#define WIFI_DPORT 0x3ff73000
#define WIFI_APB 0x60033000
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

    /* Nor may the transfer exceed the capacity recorded in the descriptor. */
    descriptor[0] = 0x80019018; /* length 25, size 24 */
    qtest_memwrite(q, 0x3ffb0000, (uint8_t *)descriptor, sizeof(descriptor));
    qtest_writel(q, WIFI_DPORT + WIFI_DMA_OUTLINK, 0xc00b0000);
    g_assert_cmphex(qtest_readl(q, WIFI_DPORT + WIFI_DMA_INT_STATUS), ==, 0);

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

static size_t make_dhcp_reply(uint8_t *frame, uint8_t message_type)
{
    uint8_t *ip = frame + 32;
    uint8_t *udp = ip + 20;
    uint8_t *dhcp = udp + 8;
    const size_t dhcp_len = 246;
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
    memcpy(dhcp + 16, (uint8_t[]){192,168,4,2}, 4); /* yiaddr */
    memcpy(dhcp + 20, (uint8_t[]){192,168,4,1}, 4); /* siaddr */
    memcpy(dhcp + 236, (uint8_t[]){0x63,0x82,0x53,0x63}, 4);
    memcpy(dhcp + 240, (uint8_t[]){53,1,message_type,255,0,0}, 6);
    return frame_len;
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

    /* Offer/ACK frames are parsed from From-DS traffic; the OFFER must
     * produce the guest's DHCP REQUEST through the same RX DMA path. */
    tx_len = make_dhcp_reply(tx, 2);
    softap_transmit_frame(q, tx, tx_len);
    rx_len = softap_receive_frame(q, rx, sizeof(rx));
    g_assert_cmpuint(rx_len, >, dhcp_options + sizeof(request_option));
    g_assert_cmphex(rx[0], ==, 0x08);
    g_assert_cmpmem(rx + 24 + 8 + 20 + 8 + 240,
                    sizeof(request_option), request_option,
                    sizeof(request_option));

    tx_len = make_dhcp_reply(tx, 5);
    softap_transmit_frame(q, tx, tx_len);
    qtest_quit(q);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32/wifi/mmio-aliases", mmio_aliases);
    qtest_add_func("/esp32/wifi/extended-mac-window-aliases",
                   extended_mac_window_aliases);
    qtest_add_func("/esp32/wifi/extended-mac-window-reset",
                   extended_mac_window_reset);
    qtest_add_func("/esp32/wifi/mmio-without-network-backend",
                   mmio_present_without_network_backend);
    qtest_add_func("/esp32/wifi/mmio-aligned-word-access",
                   mmio_requires_aligned_words);
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
