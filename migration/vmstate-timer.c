/* QEMU timer migration fields.
 * Extracted from savevm.c to permit isolated device-VMState unit tests.
 * Copyright (c) 2009-2017 Red Hat Inc.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"
#include "qemu-file.h"

/* QEMUFile timer support.
 * Not in qemu-file.c to not add qemu-timer.c as dependency to qemu-file.c
 */

void timer_put(QEMUFile *f, QEMUTimer *ts)
{
    uint64_t expire_time;

    expire_time = timer_expire_time_ns(ts);
    qemu_put_be64(f, expire_time);
}

void timer_get(QEMUFile *f, QEMUTimer *ts)
{
    uint64_t expire_time;

    expire_time = qemu_get_be64(f);
    if (expire_time != -1) {
        timer_mod_ns(ts, expire_time);
    } else {
        timer_del(ts);
    }
}


/* VMState timer support.
 * Not in vmstate.c to not add qemu-timer.c as dependency to vmstate.c
 */

static int get_timer(QEMUFile *f, void *pv, size_t size,
                     const VMStateField *field)
{
    QEMUTimer *v = pv;
    timer_get(f, v);
    return 0;
}

static int put_timer(QEMUFile *f, void *pv, size_t size,
                     const VMStateField *field, JSONWriter *vmdesc)
{
    QEMUTimer *v = pv;
    timer_put(f, v);

    return 0;
}

const VMStateInfo vmstate_info_timer = {
    .name = "timer",
    .get  = get_timer,
    .put  = put_timer,
};


