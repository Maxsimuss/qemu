/* Original ESP32 raw APLL CLK_OUT migration stream and deadline regression.
 * Xtensa whole-machine migration remains unsupported; this exercises the
 * actual GPIO device VMState descriptor and QEMUTimer serializer in isolation.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/misc/esp32_reg.h"
#include "hw/gpio/esp32_gpio.h"
#include "io/channel-file.h"
#include "migration/qemu-file-types.h"
#include "../migration/qemu-file.h"
#include "sysemu/cpu-timers.h"

ICountMode use_icount;
static int64_t virtual_time;
static unsigned callbacks;
static unsigned output_updates;

int64_t cpus_get_virtual_clock(void) { return virtual_time; }
void cpus_set_virtual_clock(int64_t time) { virtual_time = time; }
int64_t icount_get_raw(void) { return virtual_time; }
void icount_start_warp_timer(void) { g_assert_not_reached(); }
unsigned icount_process_idle_timers(bool (*pending_work)(void))
{
    g_assert_not_reached();
}
bool icount_configure(QemuOpts *opts, Error **errp) { g_assert_not_reached(); }
void icount_account_warp_timer(void) { g_assert_not_reached(); }
void icount_notify_exit(void) { g_assert_not_reached(); }

/* Observe post-load reconstruction. Physical pad levels/periods are checked
 * independently by MMIO qtests; these stubs do not fabricate pad events. */
void esp32_gpio_rebuild_outputs(Esp32GpioState *s)
{
    output_updates++;
    s->routes_valid = false;
}

static void timer_callback(void *opaque)
{
    callbacks++;
}

static void initialize(Esp32GpioState *s, bool fill)
{
    memset(s, 0, sizeof(*s));
    memset(s->external, ESP32_PAD_Z, sizeof(s->external));
    s->apll_clkout_den = 1;
    for (unsigned i = 0; i < 3; i++) {
        s->clkout_timer[i] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                         timer_callback, NULL);
        s->clkout_den[i] = 1;
        s->clkout_half_div[i] = 1;
    }
    if (!fill) {
        return;
    }
    /* SDM0=1, SDM1=0, SDM2=5, ODIV=4, crystal=40MHz:
     * fout=46,080,078,125/1536 Hz. A valid fractional clock numerator can
     * be much larger than its frequency in hertz. */
    s->apll_clkout_num = UINT64_C(46080078125);
    s->apll_clkout_den = 1536;
    s->mux[0] = 0x666;
    static const unsigned offsets[] = { 0x44, 0x84, 0x88 };
    for (unsigned i = 0; i < 3; i++) {
        s->mux[offsets[i] / 4] = (1 << 12) | (1 << 9);
        s->clkout_num[i] = s->apll_clkout_num;
        s->clkout_den[i] = s->apll_clkout_den;
        s->clkout_half_div[i] = 2 * s->clkout_num[i];
        uint64_t half = s->clkout_den[i] * UINT64_C(1000000000);
        s->clkout_half_whole[i] = half / s->clkout_half_div[i];
        s->clkout_half_rem[i] = half % s->clkout_half_div[i];
        s->clkout_phase[i] = s->clkout_half_rem[i];
        s->clkout_level[i] = i & 1;
        timer_mod_ns(s->clkout_timer[i], 10000 + 100 * i);
    }
    s->regs[4 / 4] = 0x12345678;
    s->peripheral_known[23] = true;
    s->peripheral_value[23] = true;
    s->peripheral_enable[23] = true;
    s->input_sample[18] = true;
}

static void destroy(Esp32GpioState *s)
{
    for (unsigned i = 0; i < 3; i++) {
        timer_free(s->clkout_timer[i]);
    }
}

static int roundtrip_version(Esp32GpioState *source, Esp32GpioState *dest,
                             unsigned version)
{
    g_autofree char *name = NULL;
    int fd = g_file_open_tmp("esp32-gpio-state-XXXXXX", &name, NULL);
    g_assert_cmpint(fd, >=, 0);
    QIOChannel *ioc = QIO_CHANNEL(qio_channel_file_new_fd(dup(fd)));
    QEMUFile *f = qemu_file_new_output(ioc);
    object_unref(OBJECT(ioc));
    g_assert_cmpint(vmstate_save_state_v(f, &vmstate_esp32_gpio, source,
                                       NULL, version, NULL), ==, 0);
    g_assert_cmpint(qemu_fclose(f), ==, 0);
    g_assert_cmpint(lseek(fd, 0, SEEK_SET), ==, 0);
    ioc = QIO_CHANNEL(qio_channel_file_new_fd(fd));
    f = qemu_file_new_input(ioc);
    object_unref(OBJECT(ioc));
    int result = vmstate_load_state(f, &vmstate_esp32_gpio, dest, version);
    qemu_fclose(f);
    unlink(name);
    return result;
}

static int roundtrip(Esp32GpioState *source, Esp32GpioState *dest)
{
    return roundtrip_version(source, dest, vmstate_esp32_gpio.version_id);
}

static void test_restore_active(void)
{
    Esp32GpioState source, dest;
    initialize(&source, true);
    initialize(&dest, false);
    output_updates = 0;
    g_assert_cmpint(roundtrip(&source, &dest), ==, 0);
    g_assert_cmpuint(output_updates, ==, 1);
    g_assert_cmpuint(dest.apll_clkout_num, ==, UINT64_C(46080078125));
    g_assert_cmpuint(dest.apll_clkout_den, ==, 1536);
    g_assert_cmpmem(dest.regs, sizeof(dest.regs), source.regs, sizeof(source.regs));
    g_assert_cmpmem(dest.mux, sizeof(dest.mux), source.mux, sizeof(source.mux));
    g_assert_cmpmem(dest.clkout_num, sizeof(dest.clkout_num),
                    source.clkout_num, sizeof(source.clkout_num));
    g_assert_cmpmem(dest.clkout_den, sizeof(dest.clkout_den),
                    source.clkout_den, sizeof(source.clkout_den));
    g_assert_cmpmem(dest.clkout_phase, sizeof(dest.clkout_phase),
                    source.clkout_phase, sizeof(source.clkout_phase));
    g_assert_cmpmem(dest.clkout_level, sizeof(dest.clkout_level),
                    source.clkout_level, sizeof(source.clkout_level));
    g_assert_true(dest.peripheral_known[23]);
    g_assert_true(dest.input_sample[18]);
    for (unsigned i = 0; i < 3; i++) {
        g_assert_cmpint(dest.clkout_timer[i]->expire_time, ==, 10000 + 100 * i);
        g_assert_true(dest.clkout_context[i].owner == &dest);
        g_assert_cmpuint(dest.clkout_context[i].index, ==, i);
    }
    destroy(&source);
    callbacks = 0;
    virtual_time = 9999;
    qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL);
    g_assert_cmpuint(callbacks, ==, 0);
    for (unsigned i = 0; i < 3; i++) {
        virtual_time = 10000 + 100 * i;
        qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL);
        g_assert_cmpuint(callbacks, ==, i + 1);
    }
    destroy(&dest);
    virtual_time = 0;
}

static void test_restore_disabled(void)
{
    Esp32GpioState source, dest;
    initialize(&source, false);
    initialize(&dest, true); /* Previously active clocks must be canceled. */
    g_assert_cmpint(roundtrip(&source, &dest), ==, 0);
    for (unsigned i = 0; i < 3; i++) {
        g_assert_false(timer_pending(dest.clkout_timer[i]));
        g_assert_cmpuint(dest.clkout_num[i], ==, 0);
        g_assert_cmpuint(dest.clkout_den[i], ==, 1);
        g_assert_cmpuint(dest.clkout_half_div[i], ==, 1);
    }
    destroy(&source);
    destroy(&dest);
}

static void test_restore_version1(void)
{
    Esp32GpioState source, dest;
    initialize(&source, true);
    initialize(&dest, true);
    g_assert_cmpint(roundtrip_version(&source, &dest, 1), ==, 0);
    g_assert_cmpmem(dest.regs, sizeof(dest.regs), source.regs, sizeof(source.regs));
    g_assert_true(dest.peripheral_known[23]);
    g_assert_true(dest.peripheral_value[23]);
    g_assert_true(dest.input_sample[18]);
    for (unsigned i = 0; i < 3; i++) {
        g_assert_false(timer_pending(dest.clkout_timer[i]));
        g_assert_cmpuint(dest.clkout_num[i], ==, 0);
        g_assert_cmpuint(dest.clkout_den[i], ==, 1);
    }
    destroy(&source);
    destroy(&dest);
}

static void test_reject_invalid(void)
{
    Esp32GpioState source, dest;
    initialize(&source, true);
    initialize(&dest, false);
    source.clkout_den[0] = 0;
    g_assert_cmpint(roundtrip(&source, &dest), ==, -EINVAL);
    source.clkout_den[0] = 1536;
    source.clkout_half_rem[0]++;
    g_assert_cmpint(roundtrip(&source, &dest), ==, -EINVAL);
    source.clkout_half_rem[0]--;
    source.clkout_phase[0] = source.clkout_half_div[0];
    g_assert_cmpint(roundtrip(&source, &dest), ==, -EINVAL);
    destroy(&source);
    destroy(&dest);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    module_call_init(MODULE_INIT_QOM);
    init_clocks(NULL);
    qemu_clock_enable(QEMU_CLOCK_VIRTUAL, true);
    g_test_add_func("/esp32/gpio/vmstate/restore-active-rational-clkout", test_restore_active);
    g_test_add_func("/esp32/gpio/vmstate/restore-disabled", test_restore_disabled);
    g_test_add_func("/esp32/gpio/vmstate/restore-version1", test_restore_version1);
    g_test_add_func("/esp32/gpio/vmstate/reject-invalid", test_reject_invalid);
    return g_test_run();
}
