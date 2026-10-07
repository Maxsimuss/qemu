/* ESP32 resolved pad routing, electrical resolution and trace tests.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"
#define GPIO 0x3ff44000
#define MUX 0x3ff49000

static void pad_resolution(void)
{
    g_autofree char *name = NULL;
    g_autofree char *text = NULL;
    int fd = g_file_open_tmp("esp32-pad-XXXXXX.vcd", &name, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    g_autofree char *args = g_strdup_printf("-machine esp32 -serial none -display none -nic none -global driver=esp32.gpio,property=pin-trace,value=%s", name);
    QTestState *q = qtest_init(args);
    qtest_writel(q, MUX + 0x70, (2 << 12) | (1 << 9) | (1 << 8));
    qtest_writel(q, GPIO + 0x530 + 18 * 4, 256 | (1 << 10));
    g_assert_cmphex(qtest_readl(q, GPIO + 0x3c) & (1 << 18), ==, 1 << 18);
    qtest_clock_step(q, 10);
    qtest_writel(q, GPIO + 0x24, 1 << 18);
    g_assert_cmphex(qtest_readl(q, GPIO + 0x3c) & (1 << 18), ==, 0);
    qtest_clock_step(q, 10);
    qtest_writel(q, GPIO + 0x08, 1 << 18);
    g_assert_cmphex(qtest_readl(q, GPIO + 0x3c) & (1 << 18), ==, 1 << 18);
    qtest_clock_step(q, 10);
    qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", 18, 0);
    qtest_clock_step(q, 10);
    qtest_writel(q, GPIO + 0x88 + 18 * 4, 1 << 2); /* open drain releases high */
    g_assert_cmphex(qtest_readl(q, GPIO + 0x3c) & (1 << 18), ==, 0);
    qtest_clock_step(q, 10);
    qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", 18, 2);
    g_assert_cmphex(qtest_readl(q, GPIO + 0x3c) & (1 << 18), ==, 1 << 18);
    qtest_clock_step(q, 10);
    qtest_writel(q, GPIO + 0x530 + 18 * 4, 223); /* unknown peripheral remains X */
    qtest_clock_step(q, 10);
    qtest_writel(q, MUX + 0x70, 2 << 12); /* input disabled */
    g_assert_cmphex(qtest_readl(q, GPIO + 0x3c) & (1 << 18), ==, 0);
    qtest_writel(q, MUX + 0x18, UINT32_MAX); /* GPIO35 input only */
    g_assert_cmphex(qtest_readl(q, MUX + 0x18) & ((3 << 10) | (3 << 7) | (3 << 5) | (3 << 2) | 1), ==, 0);
    /* Native MCLK nodes are not accessible through matrix OUTPUT_SEL. */
    qtest_writel(q, GPIO + 0x530 + 18 * 4, 257 | (1 << 10));
    qtest_quit(q);
    g_assert_true(g_file_get_contents(name, &text, NULL, NULL));
    g_assert_nonnull(strstr(text, "$timescale 1 ns"));
    g_assert_nonnull(strstr(text, "#10\n0P18\n0D18"));
    g_assert_nonnull(strstr(text, "#20\n1P18\n1D18"));
    g_assert_nonnull(strstr(text, "#30\nxP18\n0E18"));
    g_assert_nonnull(strstr(text, "#40\n0P18\nzD18"));
    g_assert_nonnull(strstr(text, "#50\n1P18\nzE18"));
    g_assert_nonnull(strstr(text, "#60\nxP18\nxD18"));
    for (unsigned pad = 0; pad < 40; pad++) {
        if ((UINT64_C(0xff0eefffff) >> pad) & 1) {
            g_autofree char *label = g_strdup_printf("gpio%u_external_drive $end", pad);
            g_assert_nonnull(strstr(text, label));
        }
    }
    for (unsigned terminal = 1; terminal <= 49; terminal++) {
        g_autofree char *label = g_strdup_printf("pin%02u_", terminal);
        g_assert_nonnull(strstr(text, label));
    }
    g_assert_nonnull(strstr(text, "P37 pin06_SENSOR_CAPP"));
    g_assert_nonnull(strstr(text, "P38 pin07_SENSOR_CAPN"));
    g_assert_nonnull(strstr(text, "U2 pin02_LNA_IN_UNMODELED"));
    g_assert_nonnull(strstr(text, "xU2\n"));
    unlink(name);
}

static void trace_failure(void)
{
    if (access("/dev/full", W_OK)) {
        g_test_skip("Host lacks /dev/full");
        return;
    }
    const char *argv[] = {g_getenv("QTEST_QEMU_BINARY"), "-machine", "esp32",
        "-serial", "none", "-display", "none", "-nic", "none", "-global",
        "driver=esp32.gpio,property=pin-trace,value=/dev/full", NULL};
    g_autofree char *error = NULL;
    int status;
    g_assert_true(g_spawn_sync(NULL, (char **)argv, NULL, 0, NULL, NULL, NULL,
                              &error, &status, NULL));
    g_assert_true(WIFEXITED(status));
    g_assert_cmpint(WEXITSTATUS(status), !=, 0);
    g_assert_nonnull(strstr(error, "ESP32 pin trace '/dev/full'"));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/esp32/pad-resolution-and-vcd", pad_resolution);
    g_test_add_func("/esp32/trace-host-write-failure", trace_failure);
    return g_test_run();
}
