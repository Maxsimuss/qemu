/* Isolated ESP32 I2S migration serialization/validation tests.
 * Xtensa CPU migration is unsupported; this tests the real device VMState
 * stream without claiming whole-machine or physical-output restoration.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/misc/esp32_i2s.h"
#include "hw/irq.h"
#include "io/channel-file.h"
#include "migration/qemu-file-types.h"
#include "../migration/qemu-file.h"
#include "sysemu/cpu-timers.h"

ICountMode use_icount;
static int64_t virtual_time;
static unsigned callbacks;

int64_t cpus_get_virtual_clock(void) { return virtual_time; }
void cpus_set_virtual_clock(int64_t time) { virtual_time = time; }
int64_t icount_get_raw(void) { return virtual_time; }
void icount_start_warp_timer(void) { g_assert_not_reached(); }
bool icount_configure(QemuOpts *opts, Error **errp) { g_assert_not_reached(); }
void icount_account_warp_timer(void) { g_assert_not_reached(); }
void icount_notify_exit(void) { g_assert_not_reached(); }

static uint8_t output_level[ESP32_GPIO_OUTPUTS];
static uint8_t output_enable[ESP32_GPIO_OUTPUTS];
static bool interrupt_level;

/* Observe the model's real post-load signal re-drive at its GPIO interface.
 * Resolved physical pads are independently checked by the device qtests. */
void esp32_gpio_set_peripheral_output(Esp32GpioState *gpio, unsigned signal,
                                     bool level, bool enable, bool open_drain)
{
    g_assert_cmpuint(signal, <, ESP32_GPIO_OUTPUTS);
    g_assert_false(open_drain);
    output_level[signal] = level;
    output_enable[signal] = enable;
}

bool esp32_gpio_output_is_routed(Esp32GpioState *gpio, unsigned signal)
{
    return signal == ESP32_GPIO_MCLK0;
}

void qemu_set_irq(qemu_irq irq, int level)
{
    interrupt_level = level;
}

static void timer_callback(void *opaque)
{
    callbacks++;
}

static void initialize(Esp32I2SState *s, bool fill)
{
    memset(s, 0, sizeof(*s));
    s->tx.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_callback, NULL);
    s->rx.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_callback, NULL);
    s->mclk_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_callback, NULL);
    if (!fill) {
        return;
    }
    for (unsigned i = 0; i < G_N_ELEMENTS(s->regs); i++) {
        s->regs[i] = 0xa5a50000 + i;
    }
    for (unsigned i = 0; i < G_N_ELEMENTS(s->input_level); i++) {
        s->input_level[i] = i & 1;
    }
    for (unsigned direction = 0; direction < 2; direction++) {
        Esp32I2SChannel *c = direction ? &s->rx : &s->tx;
        for (unsigned i = 0; i < ESP32_I2S_FIFO_WORDS; i++) {
            c->fifo[i] = 0x12345678 + i;
            c->fifo_descriptor[i] = 0x3ffb0000 + 16 * i;
            c->fifo_flags[i] = i & 7;
        }
        c->fifo_head = direction ? 31 : 63;
        c->fifo_count = direction ? 17 : 64;
        c->descriptor = 0x3ffb1000;
        c->descriptor_words[0] = 0x80020020;
        c->descriptor_words[1] = 0x3ffb2000;
        c->descriptor_words[2] = 0x3ffb1100;
        c->descriptor_offset = 8;
        c->descriptor_limit = 32;
        c->dma_words = 12;
        c->frame[0] = 0x12345678;
        c->frame[1] = 0x9abcdef0;
        c->shift = 0x12340000;
        c->bit = 12;
        c->slot = 1;
        c->phase = 3;
        c->clock_bit = 22;
        c->clock_remainder = 5;
        c->deadline = 10000;
        c->link_active = c->descriptor_loaded = true;
        c->ws_level = c->data_level = c->frame_valid = true;
        c->started = c->synchronized = c->mono_pending = true;
        c->paused_ns = 15;
    }
    s->regs[0x08 / 4] = (1 << 4) | (1 << 5);
    s->regs[0x1c / 4] = s->regs[0x98 / 4] = s->regs[0x9c / 4] = 0;
    s->regs[0x60 / 4] = 0x100;
    s->regs[0xa0 / 4] = 0x89;
    s->regs[0xa4 / 4] = 0x0a;
    s->regs[0xb4 / 4] = 0x01550020;
    s->regs[0x0c / 4] = s->regs[0x14 / 4] = 1 << 12;
    s->regs[0xa8 / 4] = (1 << 5) | (1 << 1); /* LCD parallel repeat */
    s->regs[0xac / 4] = (1 << 20) | 10;
    s->mclk_remainder = 7;
    s->mclk_deadline = 1000;
    s->mclk_paused_ns = 25;
    s->mclk_level = true;
    s->enabled = true;
    s->apll_hz = 8000000;
    timer_mod_ns(s->tx.timer, 12345);
    timer_mod_ns(s->rx.timer, 23456);
    timer_mod_ns(s->mclk_timer, 34567);
}

static void destroy(Esp32I2SState *s)
{
    timer_free(s->tx.timer);
    timer_free(s->rx.timer);
    timer_free(s->mclk_timer);
}

static int roundtrip(Esp32I2SState *source, Esp32I2SState *dest)
{
    g_autofree char *name = NULL;
    int fd = g_file_open_tmp("esp32-i2s-state-XXXXXX", &name, NULL);
    g_assert_cmpint(fd, >=, 0);
    QIOChannel *ioc = QIO_CHANNEL(qio_channel_file_new_fd(dup(fd)));
    QEMUFile *f = qemu_file_new_output(ioc);
    object_unref(OBJECT(ioc));
    g_assert_cmpint(vmstate_save_state(f, &vmstate_esp32_i2s, source, NULL), ==, 0);
    g_assert_cmpint(qemu_fclose(f), ==, 0);
    g_assert_cmpint(lseek(fd, 0, SEEK_SET), ==, 0);
    ioc = QIO_CHANNEL(qio_channel_file_new_fd(fd));
    f = qemu_file_new_input(ioc);
    object_unref(OBJECT(ioc));
    int result = vmstate_load_state(f, &vmstate_esp32_i2s, dest, 1);
    qemu_fclose(f);
    unlink(name);
    return result;
}

static void compare_channel(Esp32I2SChannel *a, Esp32I2SChannel *b)
{
    Esp32I2SChannel copy_a = *a, copy_b = *b;
    g_assert_cmpint(a->timer->expire_time, ==, b->timer->expire_time);
    copy_a.timer = copy_b.timer = NULL;
    g_assert_cmpmem(&copy_a, sizeof(copy_a), &copy_b, sizeof(copy_b));
}

static void test_restore(void)
{
    Esp32I2SState source, dest;
    initialize(&source, true);
    initialize(&dest, false);
    g_assert_cmpint(roundtrip(&source, &dest), ==, 0);
    g_assert_cmpmem(source.regs, sizeof(source.regs), dest.regs, sizeof(dest.regs));
    g_assert_cmpmem(source.input_level, sizeof(source.input_level),
                    dest.input_level, sizeof(dest.input_level));
    compare_channel(&source.tx, &dest.tx);
    compare_channel(&source.rx, &dest.rx);
    g_assert_cmpuint(dest.mclk_remainder, ==, 7);
    g_assert_cmpint(dest.mclk_deadline, ==, 1000);
    g_assert_cmpint(dest.mclk_paused_ns, ==, 25);
    g_assert_true(dest.mclk_level);
    g_assert_true(dest.enabled);
    g_assert_cmpuint(dest.apll_hz, ==, 8000000);
    for (unsigned bit = 0; bit < 24; bit++) {
        g_assert_cmpint(output_enable[140 + bit], ==, 1);
        g_assert_cmpint(output_level[140 + bit], ==, (0x9abcde >> bit) & 1);
    }
    g_assert_cmpint(output_enable[23], ==, 1); /* master TX BCK */
    g_assert_cmpint(output_enable[25], ==, 1); /* master TX WS */
    g_assert_cmpint(output_enable[27], ==, 1); /* master RX BCK */
    g_assert_cmpint(output_enable[28], ==, 1); /* master RX WS */
    g_assert_cmpint(output_level[ESP32_GPIO_MCLK0], ==, 1);
    g_assert_cmpint(output_enable[ESP32_GPIO_MCLK0], ==, 1);
    g_assert_true(interrupt_level);
    destroy(&source);
    callbacks = 0;
    virtual_time = 12344;
    qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL);
    g_assert_cmpuint(callbacks, ==, 0);
    virtual_time = 12345;
    qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL);
    g_assert_cmpuint(callbacks, ==, 1);
    destroy(&dest);
}

static void test_reject_invalid(void)
{
    Esp32I2SState source, dest;
    initialize(&source, true);
    initialize(&dest, false);
    source.tx.fifo_count = 65;
    g_assert_cmpint(roundtrip(&source, &dest), ==, -EINVAL);
    source.tx.fifo_count = 64;
    source.rx.bit = 64; /* completed 32-bit PCM frame, waiting for next sync */
    g_assert_true(esp32_i2s_state_valid(&source));
    source.input_level[4] = 2;
    g_assert_false(esp32_i2s_state_valid(&source));
    source.input_level[4] = 0;
    source.tx.fifo_flags[3] = 8;
    g_assert_false(esp32_i2s_state_valid(&source));
    destroy(&source);
    destroy(&dest);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    module_call_init(MODULE_INIT_QOM);
    init_clocks(NULL);
    qemu_clock_enable(QEMU_CLOCK_VIRTUAL, true);
    g_test_add_func("/esp32/i2s/vmstate/restore-stream-and-timers", test_restore);
    g_test_add_func("/esp32/i2s/vmstate/reject-invalid", test_reject_invalid);
    return g_test_run();
}
