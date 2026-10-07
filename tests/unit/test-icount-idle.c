/*
 * Real timer/deadline regressions for bounded icount idle dispatch.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/seqlock.h"
#include "qemu/timer.h"
#include "sysemu/cpu-timers.h"
#include "sysemu/cpu-timers-internal.h"
#include "sysemu/cpus.h"
#include "sysemu/qtest.h"
#include "sysemu/replay.h"
#include "sysemu/runstate.h"

ICountMode use_icount = ICOUNT_PRECISE;
TimersState timers_state;
static bool running, sleeping, halted, interrupt_pending, work_pending;
static bool deferred_work;

static bool pending_main_loop_work(void)
{
    return deferred_work;
}

int64_t cpus_get_virtual_clock(void)
{
    return timers_state.qemu_icount_bias;
}

void cpus_set_virtual_clock(int64_t value)
{
    timers_state.qemu_icount_bias = value;
}

int64_t icount_get_raw(void)
{
    return 0;
}

bool icount_sleep_enabled(void)
{
    return sleeping;
}

bool runstate_is_running(void)
{
    return running;
}

bool all_cpu_threads_idle(void)
{
    /* cpu_thread_is_idle() also checks cpu_has_work(), not just halted. */
    return halted && !interrupt_pending && !work_pending;
}

bool icount_configure(QemuOpts *opts, Error **errp)
{
    g_assert_not_reached();
}

void icount_start_warp_timer(void)
{
    /* timer_mod() must never enter another warp inside a timer callback. */
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
    QEMUTimer *a, *b, *c;
    int64_t times[512];
    unsigned ids[512], count;
    bool slow;
} Fixture;

static void reset(void)
{
    timers_state.qemu_icount_bias = 0;
    use_icount = ICOUNT_PRECISE;
    running = halted = true;
    sleeping = interrupt_pending = work_pending = qtest_allowed = false;
    replay_mode = REPLAY_MODE_NONE;
    deferred_work = false;
}

static void record(Fixture *f, unsigned id)
{
    g_assert_cmpuint(f->count, <, G_N_ELEMENTS(f->times));
    f->times[f->count] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    f->ids[f->count++] = id;
}

static void periodic_a(void *opaque)
{
    Fixture *f = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    g_assert_true(icount_idle_timers_running());
    record(f, 1);
    if (now == 300) {
        /* The CPU remains halted until its thread runs; pending IRQ must
         * prevent the main-loop from jumping to the following deadline. */
        interrupt_pending = true;
    } else {
        timer_mod(f->a, now + 100);
    }
}

static void periodic_b(void *opaque)
{
    Fixture *f = opaque;
    record(f, 2);
    timer_mod(f->b, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100);
}

static void test_interleaved_irq(void)
{
    Fixture f = { 0 };
    static const int64_t times[] = { 100, 150, 200, 250, 300 };
    static const unsigned ids[] = { 1, 2, 1, 2, 1 };
    reset();
    f.a = timer_new_ns(QEMU_CLOCK_VIRTUAL, periodic_a, &f);
    f.b = timer_new_ns(QEMU_CLOCK_VIRTUAL, periodic_b, &f);
    timer_mod(f.a, 100);
    timer_mod(f.b, 150);
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 5);
    g_assert_cmpmem(f.times, 5 * sizeof(*f.times), times, sizeof(times));
    g_assert_cmpmem(f.ids, 5 * sizeof(*f.ids), ids, sizeof(ids));
    g_assert_true(halted);
    g_assert_true(interrupt_pending);
    g_assert_false(icount_idle_timers_running());
    g_assert_cmpint(cpus_get_virtual_clock(), ==, 300);
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    g_assert_cmpuint(f.count, ==, 5);
    timer_free(f.a);
    timer_free(f.b);
}

static void wake_b(void *opaque)
{
    Fixture *f = opaque;
    record(f, 2);
    work_pending = true;
}

static void same_time_c(void *opaque)
{
    record(opaque, 3);
}

static void rearm_a(void *opaque)
{
    Fixture *f = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    record(f, 1);
    timer_del(f->b);
    timer_mod(f->b, now + 50);
    timer_mod(f->c, now);
    g_assert_cmpint(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), ==, now);
}

static void test_cancel_rearm(void)
{
    Fixture f = { 0 };
    reset();
    f.a = timer_new_ns(QEMU_CLOCK_VIRTUAL, rearm_a, &f);
    f.b = timer_new_ns(QEMU_CLOCK_VIRTUAL, wake_b, &f);
    f.c = timer_new_ns(QEMU_CLOCK_VIRTUAL, same_time_c, &f);
    timer_mod(f.a, 100);
    timer_mod(f.b, 200);
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 2);
    g_assert_cmpuint(f.count, ==, 3);
    g_assert_cmpuint(f.ids[0], ==, 1);
    g_assert_cmpuint(f.ids[1], ==, 3);
    g_assert_cmpuint(f.ids[2], ==, 2);
    g_assert_cmpint(f.times[0], ==, 100);
    g_assert_cmpint(f.times[1], ==, 100);
    g_assert_cmpint(f.times[2], ==, 150);
    timer_free(f.a);
    timer_free(f.b);
    timer_free(f.c);
}

static void test_gates(void)
{
    Fixture f = { 0 };
    reset();
    f.a = timer_new_ns(QEMU_CLOCK_VIRTUAL, same_time_c, &f);
    timer_mod(f.a, 100);
    use_icount = ICOUNT_DISABLED;
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    use_icount = ICOUNT_ADAPTATIVE;
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    use_icount = ICOUNT_PRECISE;
    sleeping = true;
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    sleeping = false;
    replay_mode = REPLAY_MODE_RECORD;
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    replay_mode = REPLAY_MODE_PLAY;
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    replay_mode = REPLAY_MODE_NONE;
    qtest_allowed = true;
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    qtest_allowed = false;
    running = false;
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    running = true;
    halted = false;
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    halted = true;
    interrupt_pending = true;
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    interrupt_pending = false;
    work_pending = true;
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    g_assert_cmpint(cpus_get_virtual_clock(), ==, 0);
    g_assert_cmpuint(f.count, ==, 0);
    timer_free(f.a);
}

static void test_zero_and_other_list(void)
{
    Fixture f = { 0 };
    QEMUTimerList *list;
    QEMUTimerListGroup group = { 0 };
    reset();
    f.a = timer_new_ns(QEMU_CLOCK_VIRTUAL, same_time_c, &f);
    timer_mod(f.a, 0);
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 1);
    g_assert_cmpint(cpus_get_virtual_clock(), ==, 0);
    g_assert_cmpuint(f.count, ==, 1);
    timer_free(f.a);

    list = timerlist_new(QEMU_CLOCK_VIRTUAL, NULL, NULL);
    group.tl[QEMU_CLOCK_VIRTUAL] = list;
    f.a = timer_new_full(&group, QEMU_CLOCK_VIRTUAL, SCALE_NS, 0,
                        same_time_c, &f);
    timer_mod(f.a, 100);
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    g_assert_cmpint(cpus_get_virtual_clock(), ==, 100);
    g_assert_cmpuint(f.count, ==, 1);
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    g_assert_true(timerlist_run_timers(list));
    g_assert_cmpuint(f.count, ==, 2);
    timer_free(f.a);
    timerlist_free(list);
}

static void endless(void *opaque)
{
    Fixture *f = opaque;
    record(f, 1);
    if (f->slow) {
        g_usleep(2000);
    }
    timer_mod(f->a, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1);
}

static void test_fairness(void)
{
    Fixture f = { 0 };
    unsigned processed;
    reset();
    f.a = timer_new_ns(QEMU_CLOCK_VIRTUAL, endless, &f);
    timer_mod(f.a, 1);
    processed = icount_process_idle_timers(pending_main_loop_work);
    g_assert_cmpuint(processed, >, 0);
    g_assert_cmpuint(processed, <=, 256);
    g_assert_cmpuint(f.count, ==, processed);
    g_assert_cmpint(cpus_get_virtual_clock(), ==, processed);
    g_assert_true(timer_pending(f.a));
    timer_free(f.a);

    reset();
    f.count = 0;
    f.slow = true;
    f.a = timer_new_ns(QEMU_CLOCK_VIRTUAL, endless, &f);
    timer_mod(f.a, 1);
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 1);
    g_assert_cmpuint(f.count, ==, 1);
    g_assert_true(timer_pending(f.a));
    timer_free(f.a);
}

static void defer_a(void *opaque)
{
    Fixture *f = opaque;
    record(f, 1);
    deferred_work = true;
}

static void test_deferred_work(void)
{
    Fixture f = { 0 };
    reset();
    f.a = timer_new_ns(QEMU_CLOCK_VIRTUAL, defer_a, &f);
    f.b = timer_new_ns(QEMU_CLOCK_VIRTUAL, same_time_c, &f);
    timer_mod(f.a, 100);
    timer_mod(f.b, 101);
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 1);
    g_assert_cmpint(cpus_get_virtual_clock(), ==, 100);
    g_assert_cmpuint(f.count, ==, 1);
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 0);
    /* Normal event-loop dispatch completes the bottom half before advancing. */
    deferred_work = false;
    g_assert_cmpuint(icount_process_idle_timers(pending_main_loop_work), ==, 1);
    g_assert_cmpint(f.times[1], ==, 101);
    timer_free(f.a);
    timer_free(f.b);
}

int main(int argc, char **argv)
{
    int result;
    g_test_init(&argc, &argv, NULL);
    seqlock_init(&timers_state.vm_clock_seqlock);
    qemu_spin_init(&timers_state.vm_clock_lock);
    init_clocks(NULL);
    qemu_clock_enable(QEMU_CLOCK_VIRTUAL, true);
    g_test_add_func("/icount/idle/interleaved-irq", test_interleaved_irq);
    g_test_add_func("/icount/idle/cancel-rearm", test_cancel_rearm);
    g_test_add_func("/icount/idle/mode-and-cpu-gates", test_gates);
    g_test_add_func("/icount/idle/zero-other-list", test_zero_and_other_list);
    g_test_add_func("/icount/idle/fairness", test_fairness);
    g_test_add_func("/icount/idle/deferred-work", test_deferred_work);
    result = g_test_run();
    qemu_spin_destroy(&timers_state.vm_clock_lock);
    return result;
}
