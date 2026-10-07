/* ESP32 resolved pad routing, electrical resolution and trace tests.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"
#define GPIO 0x3ff44000
#define MUX 0x3ff49000
#define DPORT 0x3ff00000
#define I2S 0x3ff4f000

static bool digital(QTestState *q, unsigned pad)
{
    return (qtest_readl(q, GPIO + 0x3c) >> pad) & 1;
}

static QTestState *serial_setup(void)
{
    QTestState *q = qtest_init("-machine esp32 -serial none -display none -nic none");
    qtest_writel(q, DPORT + 0xc0, (1 << 4) | (1 << 21));
    qtest_writel(q, MUX + 0x70, (2 << 12) | (1 << 9));
    qtest_writel(q, MUX + 0x74, (2 << 12) | (1 << 9));
    qtest_writel(q, MUX + 0x8c, (2 << 12) | (1 << 9));
    qtest_writel(q, I2S + 0xac, (1 << 20) | 10);
    qtest_writel(q, I2S + 0xb0, 8 | (8 << 6) | (16 << 12) | (16 << 18));
    qtest_writel(q, I2S + 0x20, 32 | (32 << 6) | (1 << 19) | (1 << 20));
    return q;
}

static void live_output_routing(void)
{
    QTestState *q = serial_setup();
    qtest_writel(q, MUX + 0x28, (2 << 12) | (1 << 9));
    qtest_writel(q, GPIO + 0x530 + 18 * 4, 23);
    qtest_writel(q, GPIO + 0x530 + 26 * 4, 23);
    qtest_writel(q, I2S, 0xa55ac33c);
    qtest_writel(q, I2S + 8, 1 << 4);
    qtest_clock_step(q, 250);
    g_assert_true(digital(q, 18));
    g_assert_true(digital(q, 26));
    qtest_writel(q, GPIO + 0x24, (1 << 18) | (1 << 26));
    qtest_writel(q, GPIO + 0x530 + 18 * 4, 256 | (1 << 10));
    qtest_writel(q, GPIO + 0x530 + 26 * 4, 23 | (1 << 9));
    g_assert_false(digital(q, 18));
    g_assert_false(digital(q, 26));
    qtest_clock_step(q, 250);
    g_assert_false(digital(q, 18));
    g_assert_true(digital(q, 26));
    qtest_writel(q, MUX + 0x28, 1 << 9); /* native DAC is unknown */
    qtest_clock_step(q, 250);
    qtest_writel(q, MUX + 0x28, (2 << 12) | (1 << 9));
    g_assert_false(digital(q, 26));
    qtest_clock_step(q, 250);
    g_assert_true(digital(q, 26));
    qtest_writel(q, GPIO + 0x530 + 26 * 4, 256 | (1 << 10));
    qtest_clock_step(q, 250); /* no pad carries BCLK, its latch still advances */
    qtest_writel(q, GPIO + 0x530 + 18 * 4, 23);
    g_assert_true(digital(q, 18));
    qtest_clock_step(q, 250);
    g_assert_false(digital(q, 18));
    qtest_qmp_assert_success(q, "{'execute':'system_reset'}");
    qtest_clock_step(q, 0);
    g_assert_cmphex(qtest_readl(q, GPIO + 0x530 + 18 * 4), ==, 256);
    g_assert_false(digital(q, 18));
    qtest_quit(q);
}

static void receive_constant_frame(QTestState *q, uint32_t expected)
{
    for (unsigned bit = 0; bit < 32; bit++) {
        qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", 19, bit >= 16);
        qtest_clock_step(q, 250);
        qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", 18, 1);
        qtest_clock_step(q, 250);
        qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", 18, 0);
    }
    g_assert_cmphex(qtest_readl(q, I2S + 4), ==, expected);
}

static void live_input_routing(void)
{
    QTestState *q = serial_setup();
    qtest_writel(q, MUX + 0x24, (2 << 12) | (1 << 9));
    qtest_writel(q, GPIO + 0x130 + 27 * 4, 18 | (1 << 7));
    qtest_writel(q, GPIO + 0x130 + 28 * 4, 19 | (1 << 7));
    qtest_writel(q, GPIO + 0x130 + 155 * 4, 23 | (1 << 7));
    qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", 18, 0);
    qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", 19, 0);
    qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", 23, 0);
    qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", 25, 1);
    qtest_writel(q, I2S + 8, (1 << 5) | (1 << 7));
    receive_constant_frame(q, 0);
    /* Matrix and input-enable edits must publish a new sample even when
     * neither physical data pad has an edge. */
    qtest_writel(q, GPIO + 0x130 + 155 * 4, 25 | (1 << 7));
    receive_constant_frame(q, UINT32_MAX);
    qtest_writel(q, MUX + 0x24, 2 << 12);
    receive_constant_frame(q, 0);
    qtest_writel(q, GPIO + 0x130 + 155 * 4, 0x38 | (1 << 7));
    receive_constant_frame(q, UINT32_MAX);
    qtest_writel(q, GPIO + 0x130 + 155 * 4, 0x38 | (1 << 7) | (1 << 6));
    receive_constant_frame(q, 0);
    qtest_quit(q);
}

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
    g_test_add_func("/esp32/live-output-routing", live_output_routing);
    g_test_add_func("/esp32/live-input-routing", live_input_routing);
    return g_test_run();
}
