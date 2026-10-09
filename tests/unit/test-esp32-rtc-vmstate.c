/* Original ESP32 RTC/APLL device migration stream and deadline regression.
 * Xtensa whole-machine migration remains unsupported; this exercises the
 * actual RTC device VMState descriptor and QEMUTimer serializer in isolation.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/misc/esp32_reg.h"
#include "hw/misc/esp32_rtc_cntl.h"
#include "io/channel-file.h"
#include "migration/qemu-file-types.h"
#include "../migration/qemu-file.h"
#include "sysemu/cpu-timers.h"

ICountMode use_icount;
static int64_t virtual_time;
static unsigned callbacks;
static unsigned clock_updates;
static unsigned stall_updates;

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

/* Verify post-load propagation requests, independently of machine routing.
 * Downstream physical-pad clocks are verified by the MMIO qtests. */
void esp32_rtc_update_clk(Esp32RtcCntlState *s)
{
    clock_updates++;
}
void esp32_rtc_update_cpu_stall(Esp32RtcCntlState *s)
{
    stall_updates++;
}

static void timer_callback(void *opaque)
{
    callbacks++;
}

static void initialize(Esp32RtcCntlState *s, bool fill)
{
    memset(s, 0, sizeof(*s));
    s->ana_i2c_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_callback, NULL);
    s->apll_cal_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_callback, NULL);
    if (!fill) {
        return;
    }
    s->xtal_apb_freq = 26000000;
    s->pll_apb_freq = 80000000;
    s->soc_clk = ESP32_SOC_CLK_APLL;
    s->rtc_fastclk = ESP32_FAST_CLK_8M;
    s->rtc_fastclk_freq = 8000000;
    s->rtc_slowclk = ESP32_SLOW_CLK_32KXTAL;
    s->rtc_slowclk_freq = 32768;
    s->time_base_ns = 123;
    s->time_reg = UINT64_C(0x123456789a);
    s->options0_reg = 1u << 19;
    s->sw_cpu_stall_reg = 0x21000000;
    for (unsigned i = 0; i < G_N_ELEMENTS(s->scratch_reg); i++) {
        s->scratch_reg[i] = 0x12345678 + i;
    }
    s->reset_cause[0] = ESP32_SW_SYS_RESET;
    s->reset_cause[1] = ESP32_TGWDT_CPU_RESET;
    s->stat_vector_sel[1] = true;
    s->ana_conf_reg = 1u << 24;
    s->ana_config_reg = (0x3ffu << 8) & ~(1u << 14);
    s->apll_analog[0] = 0x3f;
    s->apll_analog[4] = 4;
    s->apll_analog[5] = 0x49;
    s->apll_analog[7] = 10;
    s->apll_analog[8] = 0x80;
    s->apll_analog[9] = 0x34;
    s->ana_i2c_pending = true;
    s->ana_i2c_pending_host = 3;
    s->ana_i2c_last_cmd = (9 << 8) | 0x6d;
    s->ana_i2c_cmd[3] = s->ana_i2c_last_cmd | (1u << 25);
    s->apll_calibrating = true;
    s->apll_cal_valid = true;
    s->apll_model_warned = true;
    s->apll_cal_deadline_ns = 20000;
    timer_mod_ns(s->ana_i2c_timer, 10000);
    timer_mod_ns(s->apll_cal_timer, 20000);
}

static void destroy(Esp32RtcCntlState *s)
{
    timer_free(s->ana_i2c_timer);
    timer_free(s->apll_cal_timer);
}

static int roundtrip_version(Esp32RtcCntlState *source, Esp32RtcCntlState *dest,
                             unsigned version)
{
    g_autofree char *name = NULL;
    int fd = g_file_open_tmp("esp32-rtc-state-XXXXXX", &name, NULL);
    g_assert_cmpint(fd, >=, 0);
    QIOChannel *ioc = QIO_CHANNEL(qio_channel_file_new_fd(dup(fd)));
    QEMUFile *f = qemu_file_new_output(ioc);
    object_unref(OBJECT(ioc));
    g_assert_cmpint(vmstate_save_state_v(f, &vmstate_esp32_rtc_cntl,
                                     source, NULL, version, NULL), ==, 0);
    g_assert_cmpint(qemu_fclose(f), ==, 0);
    g_assert_cmpint(lseek(fd, 0, SEEK_SET), ==, 0);
    ioc = QIO_CHANNEL(qio_channel_file_new_fd(fd));
    f = qemu_file_new_input(ioc);
    object_unref(OBJECT(ioc));
    int result = vmstate_load_state(f, &vmstate_esp32_rtc_cntl, dest, version);
    qemu_fclose(f);
    unlink(name);
    return result;
}

static int roundtrip(Esp32RtcCntlState *source, Esp32RtcCntlState *dest)
{
    return roundtrip_version(source, dest, vmstate_esp32_rtc_cntl.version_id);
}

static void test_restore_version1(void)
{
    Esp32RtcCntlState source, dest;
    initialize(&source, true);
    initialize(&dest, false);
    g_assert_cmpint(roundtrip_version(&source, &dest, 1), ==, 0);
    g_assert_cmpuint(dest.xtal_apb_freq, ==, 26000000);
    g_assert_cmpmem(dest.apll_analog, sizeof(dest.apll_analog),
                    source.apll_analog, sizeof(source.apll_analog));
    g_assert_cmpint(dest.ana_i2c_timer->expire_time, ==, 10000);
    g_assert_cmpint(dest.apll_cal_timer->expire_time, ==, 20000);
    destroy(&source);
    destroy(&dest);
}

static void test_restore_pending(void)
{
    Esp32RtcCntlState source, dest;
    initialize(&source, true);
    initialize(&dest, false);
    clock_updates = stall_updates = 0;
    g_assert_cmpint(roundtrip(&source, &dest), ==, 0);
    g_assert_cmpuint(clock_updates, ==, 1);
    g_assert_cmpuint(stall_updates, ==, 1);
    g_assert_cmpuint(dest.options0_reg, ==, source.options0_reg);
    g_assert_cmpuint(dest.xtal_apb_freq, ==, 26000000);
    g_assert_cmpint(dest.soc_clk, ==, ESP32_SOC_CLK_APLL);
    g_assert_cmpint(dest.rtc_slowclk, ==, ESP32_SLOW_CLK_32KXTAL);
    g_assert_cmpint(dest.rtc_fastclk, ==, ESP32_FAST_CLK_8M);
    g_assert_cmpint(dest.time_base_ns, ==, 123);
    g_assert_cmpuint(dest.time_reg, ==, source.time_reg);
    g_assert_cmpuint(dest.sw_cpu_stall_reg, ==, source.sw_cpu_stall_reg);
    g_assert_cmpmem(dest.scratch_reg, sizeof(dest.scratch_reg),
                    source.scratch_reg, sizeof(source.scratch_reg));
    g_assert_cmpmem(dest.reset_cause, sizeof(dest.reset_cause),
                    source.reset_cause, sizeof(source.reset_cause));
    g_assert_cmpmem(dest.stat_vector_sel, sizeof(dest.stat_vector_sel),
                    source.stat_vector_sel, sizeof(source.stat_vector_sel));
    g_assert_cmpmem(dest.apll_analog, sizeof(dest.apll_analog),
                    source.apll_analog, sizeof(source.apll_analog));
    g_assert_cmpmem(dest.ana_i2c_cmd, sizeof(dest.ana_i2c_cmd),
                    source.ana_i2c_cmd, sizeof(source.ana_i2c_cmd));
    g_assert_true(dest.ana_i2c_pending);
    g_assert_cmpuint(dest.ana_i2c_pending_host, ==, 3);
    g_assert_true(dest.apll_calibrating);
    g_assert_true(dest.apll_cal_valid);
    g_assert_cmpint(dest.apll_cal_deadline_ns, ==, 20000);
    destroy(&source);
    callbacks = 0;
    virtual_time = 9999;
    qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL);
    g_assert_cmpuint(callbacks, ==, 0);
    virtual_time = 10000;
    qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL);
    g_assert_cmpuint(callbacks, ==, 1);
    virtual_time = 19999;
    qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL);
    g_assert_cmpuint(callbacks, ==, 1);
    virtual_time = 20000;
    qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL);
    g_assert_cmpuint(callbacks, ==, 2);
    destroy(&dest);
    virtual_time = 0;
}

static void test_reject_invalid_pending(void)
{
    Esp32RtcCntlState source, dest;
    initialize(&source, true);
    initialize(&dest, false);
    source.ana_i2c_pending_host = 8;
    g_assert_cmpint(roundtrip(&source, &dest), ==, -EINVAL);
    source.ana_i2c_pending_host = 3;
    source.apll_calibrating = false; /* Timer/state consistency is essential. */
    g_assert_cmpint(roundtrip(&source, &dest), ==, -EINVAL);
    source.apll_calibrating = true;
    source.rtc_slowclk = 3; /* Reserved mux cannot index a clock table. */
    g_assert_cmpint(roundtrip(&source, &dest), ==, -EINVAL);
    source.rtc_slowclk = (Esp32SlowClkSel)UINT32_MAX;
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
    g_test_add_func("/esp32/rtc/vmstate/restore-pending-deadlines",
                   test_restore_pending);
    g_test_add_func("/esp32/rtc/vmstate/reject-invalid-pending",
                   test_reject_invalid_pending);
    g_test_add_func("/esp32/rtc/vmstate/restore-version1",
                   test_restore_version1);
    return g_test_run();
}
