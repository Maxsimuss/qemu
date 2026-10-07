/* Original ESP32 I2C real VMState stream/timer tests.
 * These tests cover the device's production serialization table and structural
 * validation. Whole-machine Xtensa migration and post-load physical pad
 * transitions are outside the isolated stream test.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/i2c/esp32_i2c.h"
#include "io/channel-file.h"
#include "migration/qemu-file-types.h"
#include "migration/qemu-file.h"
#include "migration/vmstate.h"
#include "sysemu/cpu-timers.h"

ICountMode use_icount;
static int64_t virtual_time;
static unsigned callbacks;

int64_t cpus_get_virtual_clock(void) { return virtual_time; }
void cpus_set_virtual_clock(int64_t ns) { virtual_time = ns; }
int64_t icount_get_raw(void) { return virtual_time; }
void icount_start_warp_timer(void) { g_assert_not_reached(); }
unsigned icount_process_idle_timers(bool (*pending_work)(void))
{
    g_assert_not_reached();
}

bool icount_configure(QemuOpts *opts, Error **errp) { g_assert_not_reached(); }
void icount_account_warp_timer(void) { g_assert_not_reached(); }
void icount_notify_exit(void) { g_assert_not_reached(); }

int esp32_i2c_post_load(void *opaque, int version)
{
    return esp32_i2c_state_valid(opaque) ? 0 : -EINVAL;
}

static void timer_callback(void *opaque)
{
    callbacks++;
}

static void initialize(Esp32I2CState *s, bool fill)
{
    memset(s, 0, sizeof(*s));
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_callback, NULL);
    s->timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_callback, NULL);
    s->sample_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_callback, NULL);
    s->scl_filter_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_callback, NULL);
    s->sda_filter_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_callback, NULL);
    if (!fill) {
        return;
    }
    for (unsigned i = 0; i < G_N_ELEMENTS(s->reg); i++) {
        s->reg[i] = 0x12340000 + i;
    }
    for (unsigned i = 0; i < 32; i++) {
        s->ram[i] = i ^ 0xa5;
        s->tx[i] = i ^ 0x69;
        s->rx[i] = i ^ 0x96;
    }
    s->apb_freq = 80000000;
    s->ns_remainder = 60000000;
    s->period_fraction = 40000000;
    s->low_duration = 250;
    s->high_duration = 237;
    for (unsigned i = 0; i < G_N_ELEMENTS(s->paused_cycles); i++) {
        s->paused_cycles[i] = 12 + i;
    }
    s->paused_high_cycles = 8;
    s->paused_low_cycles = 12;
    s->enabled = true;
    s->tx_head = 31;
    s->tx_count = 32;
    s->rx_head = 17;
    s->rx_count = 12;
    s->tx_ptr = 25;
    s->rx_ptr = 29;
    s->tx_start = 2;
    s->tx_end = 24;
    s->rx_start = 3;
    s->rx_end = 28;
    s->tx_bytes = 13;
    s->rx_bytes = 27;
    s->int_raw = 0x133;
    s->int_ena = 0x103;
    s->scl_raw = s->sda_raw = true;
    s->scl = false;
    s->sda = true;
    s->scl_drive = s->sda_drive = true;
    s->bus_busy = s->active = s->owns_bus = true;
    s->ack_rec = s->arb_lost = s->timed_out = s->byte_trans = true;
    s->phase = 4;
    s->resume_phase = 7;
    s->cmd_index = 2;
    s->opcode = 1;
    s->bit = 3;
    s->shift = 0x96;
    s->remaining = 255;
    s->address_phase = s->error_stop = true;
    s->high_started = 100;
    s->low_started = 200;
    s->high_requested = 150;
    s->scl_raw_started = 220;
    s->main_state = 4;
    s->scl_state = 2;
    s->slave_phase = 6;
    s->slave_next = 3;
    s->slave_bits = 7;
    s->slave_shift = 0x69;
    s->slave_addressed = s->slave_rw = s->slave_ack = true;
    s->ten_selected = s->expect_offset = true;
    timer_mod(s->timer, 10000);
    timer_mod(s->timeout_timer, 20000);
    timer_mod(s->sample_timer, 6000);
    timer_mod(s->scl_filter_timer, 2000);
    timer_mod(s->sda_filter_timer, 3000);
}

static void destroy(Esp32I2CState *s)
{
    timer_free(s->timer);
    timer_free(s->timeout_timer);
    timer_free(s->sample_timer);
    timer_free(s->scl_filter_timer);
    timer_free(s->sda_filter_timer);
}

static int roundtrip(Esp32I2CState *source, Esp32I2CState *dest)
{
    g_autofree char *name = NULL;
    int fd = g_file_open_tmp("esp32-i2c-state-XXXXXX", &name, NULL);
    g_assert_cmpint(fd, >=, 0);
    QIOChannel *ioc = QIO_CHANNEL(qio_channel_file_new_fd(dup(fd)));
    QEMUFile *file = qemu_file_new_output(ioc);
    object_unref(OBJECT(ioc));
    g_assert_cmpint(vmstate_save_state(file, &vmstate_esp32_i2c, source, NULL), ==, 0);
    g_assert_cmpint(qemu_fclose(file), ==, 0);
    g_assert_cmpint(lseek(fd, 0, SEEK_SET), ==, 0);
    ioc = QIO_CHANNEL(qio_channel_file_new_fd(fd));
    file = qemu_file_new_input(ioc);
    object_unref(OBJECT(ioc));
    int result = vmstate_load_state(file, &vmstate_esp32_i2c, dest,
                                   vmstate_esp32_i2c.version_id);
    qemu_fclose(file);
    unlink(name);
    return result;
}

static void compare(Esp32I2CState *a, Esp32I2CState *b)
{
    /* All serializable scalar/array fields follow apb_freq. The QOM/MMIO/GPIO
     * wiring and host timer pointers preceding it belong to the destination. */
    size_t offset = offsetof(Esp32I2CState, apb_freq);
    g_assert_cmpmem((uint8_t *)a + offset, sizeof(*a) - offset,
                    (uint8_t *)b + offset, sizeof(*b) - offset);
    g_assert_cmpint(timer_expire_time_ns(a->timer), ==,
                    timer_expire_time_ns(b->timer));
    g_assert_cmpint(timer_expire_time_ns(a->timeout_timer), ==,
                    timer_expire_time_ns(b->timeout_timer));
    g_assert_cmpint(timer_expire_time_ns(a->sample_timer), ==,
                    timer_expire_time_ns(b->sample_timer));
    g_assert_cmpint(timer_expire_time_ns(a->scl_filter_timer), ==,
                    timer_expire_time_ns(b->scl_filter_timer));
    g_assert_cmpint(timer_expire_time_ns(a->sda_filter_timer), ==,
                    timer_expire_time_ns(b->sda_filter_timer));
}

static void test_restore(void)
{
    Esp32I2CState source, dest;
    initialize(&source, true);
    initialize(&dest, false);
    g_assert_cmpint(roundtrip(&source, &dest), ==, 0);
    compare(&source, &dest);
    destroy(&source);
    callbacks = 0;
    const int64_t deadlines[] = {2000, 3000, 6000, 10000, 20000};
    for (unsigned i = 0; i < G_N_ELEMENTS(deadlines); i++) {
        virtual_time = deadlines[i] - 1;
        qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL);
        g_assert_cmpuint(callbacks, ==, i);
        virtual_time = deadlines[i];
        qemu_clock_run_timers(QEMU_CLOCK_VIRTUAL);
        g_assert_cmpuint(callbacks, ==, i + 1);
    }
    destroy(&dest);
}

static void test_paused_restore(void)
{
    Esp32I2CState source, dest;
    initialize(&source, true);
    initialize(&dest, false);
    source.enabled = false;
    source.apb_freq = 0;
    source.ns_remainder = source.period_fraction = 0;
    timer_del(source.timer);
    timer_del(source.timeout_timer);
    timer_del(source.sample_timer);
    timer_del(source.scl_filter_timer);
    timer_del(source.sda_filter_timer);
    g_assert_cmpint(roundtrip(&source, &dest), ==, 0);
    compare(&source, &dest);
    destroy(&source);
    destroy(&dest);
}

static void test_reject_invalid(void)
{
    Esp32I2CState source, dest;
    initialize(&source, true);
    initialize(&dest, false);
    source.rx_count = 33;
    g_assert_cmpint(roundtrip(&source, &dest), ==, -EINVAL);
    source.rx_count = 32;
    source.tx_head = 32;
    g_assert_false(esp32_i2c_state_valid(&source));
    source.tx_head = 31;
    source.cmd_index = 16;
    g_assert_false(esp32_i2c_state_valid(&source));
    source.active = false; /* halted at exhausted command list is valid */
    g_assert_true(esp32_i2c_state_valid(&source));
    source.phase = ESP32_I2C_PHASE_MAX + 1;
    g_assert_false(esp32_i2c_state_valid(&source));
    destroy(&source);
    destroy(&dest);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    module_call_init(MODULE_INIT_QOM);
    init_clocks(NULL);
    qemu_clock_enable(QEMU_CLOCK_VIRTUAL, true);
    g_test_add_func("/esp32/i2c/vmstate/restore-stream-and-five-timers", test_restore);
    g_test_add_func("/esp32/i2c/vmstate/restore-paused-clock", test_paused_restore);
    g_test_add_func("/esp32/i2c/vmstate/reject-invalid", test_reject_invalid);
    return g_test_run();
}
