/* Original ESP32 one-shot Timer Group alarm regressions.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"

#define TIMER 0x3ff5f000
#define RTC 0x3ff48000
#define ENABLE (1u << 31)
#define INCREASE (1u << 30)
#define ALARM (1u << 10)
#define LEVEL_INT (1u << 11)
#define CONFIG (ENABLE | INCREASE | (40 << 13) | LEVEL_INT)

static QTestState *start(void)
{
    QTestState *q = qtest_init("-machine esp32 -display none -serial none -nic none "
        "-global driver=misc.esp32.rtc_cntl,property=xtal-apb-freq,value=40000000");
    qtest_writel(q, RTC + 0x70, 0);
    qtest_writel(q, TIMER + 0x98, 1);
    return q;
}

static void one_shot(void)
{
    QTestState *q = start();
    qtest_writel(q, TIMER + 0x10, 1000);
    qtest_writel(q, TIMER, CONFIG | ALARM);
    qtest_clock_step(q, 999000);
    g_assert_cmphex(qtest_readl(q, TIMER + 0x9c), ==, 0);
    qtest_clock_step(q, 1000);
    g_assert_cmphex(qtest_readl(q, TIMER + 0x9c), ==, 1);
    g_assert_cmphex(qtest_readl(q, TIMER) & ALARM, ==, 0);
    qtest_writel(q, TIMER + 0xa4, 1);
    qtest_clock_step(q, 2000000);
    g_assert_cmphex(qtest_readl(q, TIMER + 0x9c), ==, 0);
    qtest_quit(q);
}

static void overdue(bool increase)
{
    QTestState *q = start();
    uint32_t config = increase ? CONFIG : CONFIG & ~INCREASE;
    qtest_writel(q, TIMER + 0x18, increase ? 0 : 10000);
    qtest_writel(q, TIMER + 0x20, 1);
    qtest_writel(q, TIMER, config);
    qtest_clock_step(q, 2000000);
    qtest_writel(q, TIMER + 0x10, increase ? 1000 : 9000);
    qtest_writel(q, TIMER, config | ALARM);
    qtest_clock_step(q, 0);
    g_assert_cmphex(qtest_readl(q, TIMER + 0x9c), ==, 1);
    g_assert_cmphex(qtest_readl(q, TIMER) & ALARM, ==, 0);

    qtest_writel(q, TIMER + 0xa4, 1);
    qtest_writel(q, TIMER + 0x10, increase ? 3000 : 7000);
    qtest_writel(q, TIMER, config | ALARM);
    qtest_clock_step(q, 999000);
    g_assert_cmphex(qtest_readl(q, TIMER + 0x9c), ==, 0);
    qtest_clock_step(q, 1000);
    g_assert_cmphex(qtest_readl(q, TIMER + 0x9c), ==, 1);
    qtest_quit(q);
}

static void overdue_up(void)
{
    overdue(true);
}

static void overdue_down(void)
{
    overdue(false);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32/timer/one-shot", one_shot);
    qtest_add_func("/esp32/timer/overdue-up-and-rearm", overdue_up);
    qtest_add_func("/esp32/timer/overdue-down-and-rearm", overdue_down);
    return g_test_run();
}
