/* Original ESP32 REF_TICK divider and UART clock-consumer regressions.
 * TRM 7.2.4.2 and 7.3 establish the four SYSCON dividers. TRM 26 CONF0
 * selects APB/REF_TICK; CONF1 timeout uses eight divider cycles at APB.
 * The inherited character backend aggregates RX FIFO pacing; these tests
 * verify its clock input and virtual timers, not UART physical bit framing.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "libqtest.h"
#include <sys/socket.h>
#include <sys/un.h>

#define SYSCON 0x3ff66000
#define RTC 0x3ff48000
#define UART 0x3ff40000
#define TIMEOUT (1u << 8)
#define APB_SELECT (1u << 27)

static QTestState *start(unsigned xtal, int *serial_fd, char **socket_path)
{
    uint8_t efuse[124] = { 0 };
    g_autofree char *efuse_name = NULL;
    int fd = g_file_open_tmp("esp32-ref-efuse-XXXXXX", &efuse_name, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    efuse[13] = 0x80;
    g_assert_true(g_file_set_contents(efuse_name, (char *)efuse,
                                     sizeof(efuse), NULL));
    fd = g_file_open_tmp("esp32-ref-uart-XXXXXX", socket_path, NULL);
    g_assert_cmpint(fd, >=, 0);
    close(fd);
    unlink(*socket_path);
    g_autofree char *args = g_strdup_printf(
        "-machine esp32 -display none -nic none "
        "-global driver=misc.esp32.rtc_cntl,property=xtal-apb-freq,value=%u "
        "-drive file=%s,if=none,id=ref-efuse,format=raw "
        "-global driver=nvram.esp32.efuse,property=drive,value=ref-efuse "
        "-chardev socket,id=ref-uart,path=%s,server=on,wait=off "
        "-serial chardev:ref-uart", xtal, efuse_name, *socket_path);
    QTestState *q = qtest_init(args);
    unlink(efuse_name);
    *serial_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    g_assert_cmpint(*serial_fd, >=, 0);
    struct sockaddr_un address = { .sun_family = AF_UNIX };
    g_assert_cmpuint(strlen(*socket_path), <, sizeof(address.sun_path));
    strcpy(address.sun_path, *socket_path);
    g_assert_cmpint(connect(*serial_fd, (struct sockaddr *)&address,
                            sizeof(address)), ==, 0);
    return q;
}

static void finish(QTestState *q, int fd, char *socket_path)
{
    close(fd);
    qtest_quit(q);
    unlink(socket_path);
    g_free(socket_path);
}

static void wait_fifo(QTestState *q, unsigned count)
{
    int64_t deadline = g_get_monotonic_time() + 2000000;
    while ((qtest_readl(q, UART + 0x1c) & 255) != count) {
        g_assert_cmpint(g_get_monotonic_time(), <, deadline);
        g_usleep(100);
    }
}

static void analog_write(QTestState *q, unsigned reg, unsigned value)
{
    qtest_writel(q, 0x6000e00c, (1 << 24) | (value << 16) | (reg << 8) | 0x6d);
    for (unsigned poll = 0; poll < 1000; poll++) {
        if (!(qtest_readl(q, 0x6000e00c) & (1 << 25))) {
            return;
        }
        qtest_clock_step(q, 1000);
    }
    g_assert_not_reached();
}

static void program_apll(QTestState *q, unsigned sdm0, unsigned sdm1,
                         unsigned sdm2)
{
    qtest_writel(q, RTC, (qtest_readl(q, RTC) & ~(1 << 18)) | (1 << 19));
    qtest_writel(q, 0x6000e044, 0x3ff00 & ~(1 << 14));
    qtest_writel(q, RTC + 0x30, 1 << 24);
    analog_write(q, 4, 4);
    analog_write(q, 5, 0x49);
    analog_write(q, 7, sdm2);
    analog_write(q, 8, sdm1);
    analog_write(q, 9, sdm0);
    analog_write(q, 0, 0x0f);
    analog_write(q, 0, 0x3f);
    analog_write(q, 0, 0x1f);
    for (unsigned poll = 0; poll < 1000; poll++) {
        qtest_writel(q, 0x6000e00c, (3 << 8) | 0x6d);
        qtest_clock_step(q, 1000);
        if (qtest_readl(q, 0x6000e00c) & (0x80 << 16)) {
            return;
        }
    }
    g_assert_not_reached();
}

static void test_registers(void)
{
    int fd;
    char *socket_path;
    QTestState *q = start(40000000, &fd, &socket_path);
    static const unsigned offsets[] = { 4, 8, 12, 0x3c };
    static const unsigned defaults[] = { 39, 79, 11, 99 };
    for (unsigned i = 0; i < G_N_ELEMENTS(offsets); i++) {
        g_assert_cmphex(qtest_readl(q, SYSCON + offsets[i]), ==, defaults[i]);
        qtest_writel(q, SYSCON + offsets[i], UINT32_MAX);
        g_assert_cmphex(qtest_readl(q, SYSCON + offsets[i]), ==, 255);
        g_assert_cmphex(qtest_readl(q, 0x60026000 + offsets[i]), ==, 255);
        qtest_writel(q, 0x60026000 + offsets[i], 0xa5);
        g_assert_cmphex(qtest_readl(q, SYSCON + offsets[i]), ==, 0xa5);
    }
    qtest_qmp_assert_success(q, "{ 'execute': 'system_reset' }");
    for (unsigned i = 0; i < G_N_ELEMENTS(offsets); i++) {
        g_assert_cmphex(qtest_readl(q, SYSCON + offsets[i]), ==, defaults[i]);
    }
    finish(q, fd, socket_path);
}

typedef struct RateCase {
    unsigned xtal, source, tick_offset, tick_num;
    unsigned sdm0, sdm1, sdm2;
    bool ref;
    uint64_t source_num, source_den;
} RateCase;

static const RateCase rates[] = {
    { 40000000, 0, 4, 39, 0, 0, 0, true, 1000000, 1 },
    { 26000000, 0, 4, 25, 0, 0, 0, true, 1000000, 1 },
    { 40000000, 1, 8, 79, 0, 0, 0, true, 1000000, 1 },
    { 40000000, 2, 12, 11, 0, 0, 0, true, 2000000, 3 },
    { 40000000, 3, 0x3c, 3, 0, 128, 5, true, 95000000, 96 },
    { 40000000, 3, 0x3c, 3, 1, 0, 5, true, 46080078125, 49152 },
    { 26000000, 3, 0x3c, 3, 0, 0, 10, true, 91000000, 96 },
    { 40000000, 3, 0x3c, 99, 0, 0, 5, false, 3750000, 1 },
};

static void test_uart_pacing(gconstpointer opaque)
{
    const RateCase *rate = opaque;
    int fd;
    char *socket_path;
    QTestState *q = start(rate->xtal, &fd, &socket_path);
    if (rate->source == 3) {
        program_apll(q, rate->sdm0, rate->sdm1, rate->sdm2);
    }
    qtest_writel(q, SYSCON + rate->tick_offset, rate->tick_num);
    qtest_writel(q, RTC + 0x70, rate->source << 27);
    qtest_writel(q, UART + 0x20, rate->ref ? 0 : APB_SELECT);
    qtest_writel(q, UART + 0x14, 1); /* CLKDIV integer1, fraction0. */
    qtest_writel(q, UART + 0x24, 0); /* Disable timeout while testing pacing. */
    uint8_t data[128] = { 0xa5 };
    g_assert_cmpint(send(fd, data, sizeof(data), 0), ==, sizeof(data));
    wait_fifo(q, 128);
    qtest_readl(q, UART); /* Make room for a queued extra host byte. */
    g_assert_cmpint(send(fd, data, 1, 0), ==, 1);
    uint64_t delay = (__uint128_t)1280 * UINT64_C(1000000000) *
                     rate->source_den / rate->source_num;
    qtest_clock_step(q, delay - 1);
    g_assert_cmpuint(qtest_readl(q, UART + 0x1c) & 255, ==, 127);
    qtest_clock_step(q, 1);
    wait_fifo(q, 128);
    finish(q, fd, socket_path);
}

static void test_timeout_clock_change(void)
{
    int fd;
    char *socket_path;
    QTestState *q = start(40000000, &fd, &socket_path);
    /* REF_TICK timeout follows the TRM's REF/APB compensation, so its
     * deadline is eight CLKDIV cycles at APB for either source selection. */
    qtest_writel(q, UART + 0x20, 0);
    qtest_writel(q, UART + 0x14, 100);
    qtest_writel(q, UART + 0x24, (1u << 31) | (1 << 24));
    uint8_t data = 0xa5;
    g_assert_cmpint(send(fd, &data, 1, 0), ==, 1);
    wait_fifo(q, 1);
    qtest_clock_step(q, 19999);
    g_assert_cmphex(qtest_readl(q, UART + 4) & TIMEOUT, ==, 0);
    qtest_clock_step(q, 1);
    g_assert_cmphex(qtest_readl(q, UART + 4) & TIMEOUT, ==, TIMEOUT);
    qtest_readl(q, UART);
    qtest_writel(q, UART + 0x10, TIMEOUT);
    qtest_writel(q, RTC + 0x70, 1 << 27); /* PLL APB80MHz. */
    g_assert_cmpint(send(fd, &data, 1, 0), ==, 1);
    wait_fifo(q, 1);
    qtest_clock_step(q, 9999);
    g_assert_cmphex(qtest_readl(q, UART + 4) & TIMEOUT, ==, 0);
    qtest_clock_step(q, 1);
    g_assert_cmphex(qtest_readl(q, UART + 4) & TIMEOUT, ==, TIMEOUT);
    qtest_readl(q, UART);
    qtest_writel(q, UART + 0x10, TIMEOUT);
    qtest_writel(q, RTC + 0x70, 0); /* Start another 20 us threshold at XTAL. */
    g_assert_cmpint(send(fd, &data, 1, 0), ==, 1);
    wait_fifo(q, 1);
    qtest_clock_step(q, 5000); /* 200 of 800 divider cycles completed. */
    qtest_writel(q, RTC + 0x30, 1 << 24); /* APB rate unchanged. */
    qtest_writel(q, RTC + 0x70, 1 << 27); /* Remaining 600 cycles at 80 MHz. */
    qtest_clock_step(q, 7499);
    g_assert_cmphex(qtest_readl(q, UART + 4) & TIMEOUT, ==, 0);
    qtest_clock_step(q, 1);
    g_assert_cmphex(qtest_readl(q, UART + 4) & TIMEOUT, ==, TIMEOUT);
    finish(q, fd, socket_path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/esp32/ref-tick/registers-reset", test_registers);
    static const char *names[] = { "xtal40", "xtal26", "pll80", "rc8m",
        "apll-sdm1", "apll-sdm0", "apll-xtal26", "apb-selected" };
    for (unsigned i = 0; i < G_N_ELEMENTS(rates); i++) {
        g_autofree char *path = g_strdup_printf("/esp32/ref-tick/uart-pacing/%s",
                                                names[i]);
        qtest_add_data_func(path, &rates[i], test_uart_pacing);
    }
    qtest_add_func("/esp32/ref-tick/uart-timeout-clock-change",
                   test_timeout_clock_change);
    return g_test_run();
}
