/*
 * Bounded idle timer dispatch for deterministic, unslept instruction counting.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/seqlock.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "sysemu/cpu-timers.h"
#include "sysemu/cpu-timers-internal.h"
#include "sysemu/cpus.h"
#include "sysemu/qtest.h"
#include "sysemu/replay.h"
#include "sysemu/runstate.h"

/* Other threads must still send their normal timer notifications. */
static __thread bool idle_dispatch;

bool icount_idle_timers_running(void)
{
    return idle_dispatch;
}

unsigned icount_process_idle_timers(bool (*pending_work)(void))
{
    const unsigned max_events = 256;
    const int64_t max_host_ns = SCALE_MS;
    unsigned processed = 0;
    int64_t start;

    /* Sleeping and replay retain their existing clock/host synchronization.
     * qtest advances its clock explicitly. Only the main-loop calls this,
     * after the previous timer callbacks have all returned and with BQL held.
     */
    if (icount_enabled() != ICOUNT_PRECISE || icount_sleep_enabled() ||
        replay_mode != REPLAY_MODE_NONE || qtest_enabled()) {
        return 0;
    }

    start = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    idle_dispatch = true;
    while (processed < max_events && runstate_is_running() &&
           all_cpu_threads_idle() && !pending_work()) {
        int64_t deadline = qemu_clock_deadline_ns_all(QEMU_CLOCK_VIRTUAL,
                                                  ~QEMU_TIMER_ATTR_EXTERNAL);

        if (deadline < 0) {
            break;
        }

        if (deadline > 0) {
            seqlock_write_lock(&timers_state.vm_clock_seqlock,
                               &timers_state.vm_clock_lock);
            qatomic_set_i64(&timers_state.qemu_icount_bias,
                           timers_state.qemu_icount_bias + deadline);
            seqlock_write_unlock(&timers_state.vm_clock_seqlock,
                                 &timers_state.vm_clock_lock);
        }

        qemu_clock_run_all_timers();

        /* Dispatch every individual deadline at its original virtual time.
         * CPU interrupt/work state is checked again before advancing further.
         * There is no need to notify the main-loop: it is dispatching now.
         */
        /* run_all_timers() deliberately excludes virtual timers under
         * icount. The normal vCPU dispatch is unnecessary while every CPU
         * is idle, so dispatch that list here under the same BQL instead.
         * A due timer in another AioContext yields to its normal dispatch;
         * in particular a zero deadline must not cause a busy loop.
         */
        if (!qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL)) {
            break;
        }
        processed++;
        if (qemu_clock_get_ns(QEMU_CLOCK_REALTIME) - start >= max_host_ns) {
            break;
        }
    }
    idle_dispatch = false;
    if (processed) {
        /* Poll host events before the next bounded batch, without posting
         * artificial CPU work just to dispatch an idle peripheral timer.
         */
        qemu_notify_event();
    }
    return processed;
}
