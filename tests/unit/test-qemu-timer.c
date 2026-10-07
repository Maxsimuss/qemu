/*
 * Virtual-timer callback scheduling regression.
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * QEMU issue 2703 / upstream 02ae315467cee589d02dfb89e13a2a6a8de09fc5:
 * timer_mod() must not warp an idle virtual clock while callbacks are still
 * running. A callback can schedule several events and then wake a CPU.
 */
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "sysemu/cpu-timers.h"

ICountMode use_icount = ICOUNT_PRECISE;
static int64_t virtual_time;
static unsigned warp_calls;

int64_t cpus_get_virtual_clock(void)
{
    return virtual_time;
}

void cpus_set_virtual_clock(int64_t new_time)
{
    virtual_time = new_time;
}

int64_t icount_get_raw(void)
{
    return virtual_time;
}

void icount_start_warp_timer(void)
{
    /* Emulate idle sleep=off behavior. Calling this from timer rearm is the
     * bug; choosing the next deadline belongs to the main loop instead. */
    int64_t deadline = qemu_clock_deadline_ns_all(QEMU_CLOCK_VIRTUAL,
                                                  QEMU_TIMER_ATTR_ALL);
    warp_calls++;
    if (deadline > 0) {
        virtual_time += deadline;
    }
}

bool icount_configure(QemuOpts *opts, Error **errp)
{
    g_assert_not_reached();
}

void icount_account_warp_timer(void)
{
    g_assert_not_reached();
}

void icount_notify_exit(void)
{
    g_assert_not_reached();
}

typedef struct Fixture {
    QEMUTimer *trigger, *timeout, *sample;
    unsigned callbacks;
    int64_t sample_time, timeout_time;
} Fixture;

static void timeout_cb(void *opaque)
{
    Fixture *f = opaque;
    f->timeout_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    f->callbacks++;
}

static void sample_cb(void *opaque)
{
    Fixture *f = opaque;
    f->sample_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    f->callbacks++;
}

static void trigger_cb(void *opaque)
{
    Fixture *f = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(f->timeout, now + 10000);
    g_assert_cmpint(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), ==, now);
    timer_mod(f->sample, now + 100);
    g_assert_cmpint(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), ==, now);
    f->callbacks++;
}

static void test_callback_rearm(void)
{
    Fixture f = { 0 };
    virtual_time = 0;
    warp_calls = 0;
    f.trigger = timer_new_ns(QEMU_CLOCK_VIRTUAL, trigger_cb, &f);
    f.timeout = timer_new_ns(QEMU_CLOCK_VIRTUAL, timeout_cb, &f);
    f.sample = timer_new_ns(QEMU_CLOCK_VIRTUAL, sample_cb, &f);
    timer_mod(f.trigger, 1000);
    /* Establish the callback dispatch instant independently of initial arm. */
    virtual_time = 1000;
    warp_calls = 0;
    g_assert_true(qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL));
    g_assert_cmpuint(f.callbacks, ==, 1);
    g_assert_cmpint(qemu_clock_deadline_ns_all(QEMU_CLOCK_VIRTUAL,
                                             QEMU_TIMER_ATTR_ALL), ==, 100);
    virtual_time = 1100;
    g_assert_true(qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL));
    g_assert_cmpuint(f.callbacks, ==, 2);
    g_assert_cmpint(f.sample_time, ==, 1100);
    virtual_time = 11000;
    g_assert_true(qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL));
    g_assert_cmpuint(f.callbacks, ==, 3);
    g_assert_cmpint(f.timeout_time, ==, 11000);
    g_assert_cmpuint(warp_calls, ==, 0);
    timer_free(f.trigger);
    timer_free(f.sample);
    timer_free(f.timeout);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    init_clocks(NULL);
    qemu_clock_enable(QEMU_CLOCK_VIRTUAL, true);
    g_test_add_func("/timer/virtual/callback-rearm-no-warp", test_callback_rearm);
    return g_test_run();
}
