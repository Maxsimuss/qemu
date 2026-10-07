/* ESP32 interrupt routing regressions: independent cores and wired-OR levels.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"
#define MATRIX 0x3ff00104
#define APP_MATRIX 0x3ff00218
#define GPIO 0x3ff44000
#define MUX 0x3ff49000
#define PATH "/machine/soc/intmatrix"

static QTestState *setup(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none -nic none");
    qtest_irq_intercept_out_named(q, PATH, "cpu-irq");
    return q;
}

static void shared_levels(void)
{
    QTestState *q = setup();
    qtest_writel(q, MATRIX + 32 * 4, 1);
    qtest_writel(q, MATRIX + 33 * 4, 1);
    qtest_writel(q, APP_MATRIX + 32 * 4, 2);
    qtest_set_irq_in(q, PATH, NULL, 32, 1);
    g_assert_true(qtest_get_irq(q, 1));
    g_assert_true(qtest_get_irq(q, 34));
    qtest_set_irq_in(q, PATH, NULL, 33, 1);
    qtest_set_irq_in(q, PATH, NULL, 32, 0);
    g_assert_true(qtest_get_irq(q, 1));
    g_assert_false(qtest_get_irq(q, 34));
    qtest_set_irq_in(q, PATH, NULL, 32, 1);
    qtest_writel(q, MATRIX + 32 * 4, 3); /* remap an asserted source */
    g_assert_true(qtest_get_irq(q, 1));
    g_assert_true(qtest_get_irq(q, 3));
    qtest_set_irq_in(q, PATH, NULL, 33, 0);
    g_assert_false(qtest_get_irq(q, 1));
    g_assert_true(qtest_get_irq(q, 3));
    qtest_set_irq_in(q, PATH, "cpu-source", 69 + 32, 0);
    g_assert_false(qtest_get_irq(q, 34));
    g_assert_true(qtest_get_irq(q, 3));
    qtest_qmp_assert_success(q, "{'execute':'system_reset'}");
    qtest_clock_step(q, 0);
    g_assert_false(qtest_get_irq(q, 3));
    g_assert_cmphex(qtest_readl(q, MATRIX + 32 * 4), ==, 6);
    qtest_quit(q);
}

static void gpio_per_core(void)
{
    QTestState *q = setup();
    qtest_writel(q, MATRIX + 22 * 4, 4);
    qtest_writel(q, APP_MATRIX + 22 * 4, 5);
    qtest_writel(q, MUX + 0x70, (2 << 12) | (1 << 9));
    qtest_writel(q, GPIO + 0x88 + 18 * 4, (1 << 7) | (1 << 15));
    qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", 18, 1);
    g_assert_true(qtest_get_irq(q, 4));
    g_assert_false(qtest_get_irq(q, 37));
    qtest_writel(q, GPIO + 0x88 + 18 * 4, (1 << 7) | (1 << 13) | (1 << 15));
    g_assert_true(qtest_get_irq(q, 4));
    g_assert_true(qtest_get_irq(q, 37));
    qtest_writel(q, GPIO + 0x4c, 1 << 18);
    g_assert_false(qtest_get_irq(q, 4));
    g_assert_false(qtest_get_irq(q, 37));
    qtest_writel(q, GPIO + 0x88 + 18 * 4, (1 << 7) | (1 << 13));
    qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", 18, 0);
    qtest_set_irq_in(q, "/machine/soc/gpio", "pad-drive", 18, 1);
    g_assert_false(qtest_get_irq(q, 4));
    g_assert_true(qtest_get_irq(q, 37));
    qtest_qmp_assert_success(q, "{'execute':'system_reset'}");
    qtest_clock_step(q, 0);
    g_assert_false(qtest_get_irq(q, 37));
    qtest_quit(q);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/esp32/interrupt/shared-level-remap-reset", shared_levels);
    g_test_add_func("/esp32/interrupt/gpio-per-core", gpio_per_core);
    return g_test_run();
}
