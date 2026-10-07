/*
 * ESP32 external I2C controller, register/serial-pin model.
 *
 * Register map and timing: ESP32 TRM v5.8, chapter 21, and Espressif's
 * original ESP32 register definitions. No guest API or I2CBus transaction
 * callback is used to manufacture transfers: receivers sample resolved pads.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/bitops.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/irq.h"
#include "migration/vmstate.h"

#define REG(s, a) ((s)->reg[(a) / 4])
#define CTL_MASTER BIT(4)
#define CTL_START BIT(5)
#define CTL_TX_LSB BIT(6)
#define CTL_RX_LSB BIT(7)
#define FIFO_NON BIT(10)
#define FIFO_ADDR BIT(11)
#define IRQ_RX_FULL BIT(0)
#define IRQ_TX_EMPTY BIT(1)
#define IRQ_RX_OVF BIT(2)
#define IRQ_END BIT(3)
#define IRQ_SLAVE_BYTE BIT(4)
#define IRQ_ARB BIT(5)
#define IRQ_MASTER_BYTE BIT(6)
#define IRQ_COMPLETE BIT(7)
#define IRQ_TIMEOUT BIT(8)
#define IRQ_START BIT(9)
#define IRQ_ACK_ERR BIT(10)
#define IRQ_NON_RX BIT(11)
#define IRQ_NON_TX BIT(12)
#define IRQ_MASK 0x1fff

enum {
    P_IDLE, P_START_WAIT, P_START_SETUP, P_START_HOLD, P_BIT_HOLD,
    P_BIT_LOW, P_WAIT_HIGH, P_BIT_SAMPLE, P_BIT_HIGH, P_STOP_LOW,
    P_STOP_SETUP, P_STOP_HOLD, P_STOP_WAIT_SDA, P_HALTED,
};
enum {
    S_IDLE, S_ADDRESS, S_ADDRESS10, S_RX, S_TX, S_ACK_SETUP,
    S_ACK_CLOCK, S_ACK_END, S_TX_ACK_SETUP, S_TX_ACK, S_TX_ACK_END,
    S_IGNORE,
};

G_STATIC_ASSERT(P_HALTED == ESP32_I2C_PHASE_MAX);
G_STATIC_ASSERT(S_IGNORE == ESP32_I2C_SLAVE_PHASE_MAX);

static void esp32_i2c_run(void *opaque);
static void esp32_i2c_command(Esp32I2CState *s);
static void esp32_i2c_input(Esp32I2CState *s, bool scl, bool level);

static bool master(Esp32I2CState *s)
{
    return REG(s, A_I2C_CTR) & CTL_MASTER;
}

static int64_t cycles_ns(Esp32I2CState *s, uint64_t cycles)
{
    /* Timers are nanosecond based; nearest subsequent APB sampling instant. */
    return s->apb_freq ? DIV_ROUND_UP(cycles * NANOSECONDS_PER_SECOND,
                                    s->apb_freq) : INT64_MAX;
}

static uint64_t period_ns(Esp32I2CState *s, uint64_t cycles)
{
    /* Keep the fractional APB tick across clock periods. Rounding each half
     * period separately would accumulate drift at 80 MHz (12.5 ns/tick). */
    uint64_t numerator = cycles * NANOSECONDS_PER_SECOND + s->period_fraction;
    uint64_t ns = numerator / s->apb_freq;
    s->period_fraction = numerator % s->apb_freq;
    return MAX(ns, 1);
}

static void schedule(Esp32I2CState *s, uint64_t cycles)
{
    uint64_t numerator;
    int64_t ns;
    if (!s->enabled || !s->apb_freq) {
        return;
    }
    numerator = cycles * NANOSECONDS_PER_SECOND + s->ns_remainder;
    ns = numerator / s->apb_freq;
    s->ns_remainder = numerator % s->apb_freq;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + MAX(ns, 1));
}

static void update_irq(Esp32I2CState *s)
{
    uint32_t conf = REG(s, A_I2C_FIFO_CONF);
    if (!(conf & FIFO_NON)) {
        if (s->tx_count < extract32(conf, 5, 5)) {
            s->int_raw |= IRQ_TX_EMPTY;
        } else {
            s->int_raw &= ~IRQ_TX_EMPTY;
        }
        if (s->rx_count > extract32(conf, 0, 5)) {
            s->int_raw |= IRQ_RX_FULL;
        } else {
            s->int_raw &= ~IRQ_RX_FULL;
        }
    } else {
        s->int_raw &= ~(IRQ_TX_EMPTY | IRQ_RX_FULL);
    }
    qemu_set_irq(s->irq, !!(s->int_raw & s->int_ena));
}

static void drive(Esp32I2CState *s, bool clock, bool level)
{
    unsigned signal = clock ? s->scl_signal : s->sda_signal;
    bool od = REG(s, A_I2C_CTR) & BIT(clock ? 1 : 0);
    if (clock) {
        s->scl_drive = level;
    } else {
        s->sda_drive = level;
    }
    if (s->gpio) {
        esp32_gpio_set_peripheral_output(s->gpio, signal, level,
                                        !clock || master(s), od);
    }
}

static void arm_timeout(Esp32I2CState *s)
{
    if (s->enabled && s->apb_freq &&
        (s->bus_busy || (s->active && s->phase != P_STOP_HOLD))) {
        timer_mod(s->timeout_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  cycles_ns(s, REG(s, A_I2C_TIMEOUT) + 1));
    } else {
        timer_del(s->timeout_timer);
    }
}

static void stop_engine(Esp32I2CState *s, uint32_t interrupt)
{
    timer_del(s->timer);
    timer_del(s->sample_timer);
    s->active = false;
    s->phase = P_HALTED;
    REG(s, A_I2C_CTR) &= ~CTL_START;
    s->int_raw |= interrupt;
    update_irq(s);
}

static void timeout_cb(void *opaque)
{
    Esp32I2CState *s = opaque;
    s->timed_out = true;
    s->int_raw |= IRQ_TIMEOUT;
    /* A timeout does not invent a STOP or release a stuck hardware state. */
    if (s->active) {
        stop_engine(s, IRQ_TIMEOUT);
    } else {
        update_irq(s);
    }
}

static bool tx_pop(Esp32I2CState *s, uint8_t *data)
{
    if (REG(s, A_I2C_FIFO_CONF) & FIFO_NON) {
        *data = s->ram[s->tx_ptr];
    } else {
        if (!s->tx_count) {
            warn_report_once("esp32-i2c: capability gap: TX underflow pin/FSM "
                             "behavior is unverified; model halts the transfer");
            qemu_log_mask(LOG_GUEST_ERROR, "esp32-i2c: TX FIFO underflow\n");
            return false;
        }
        *data = s->tx[s->tx_head];
        s->tx_head = (s->tx_head + 1) & 31;
        s->tx_count--;
    }
    s->tx_ptr = (s->tx_ptr + 1) & 31;
    s->tx_bytes++;
    if ((REG(s, A_I2C_FIFO_CONF) & FIFO_NON) &&
        s->tx_bytes >= MAX(extract32(REG(s, A_I2C_FIFO_CONF), 20, 6), 1)) {
        warn_report_once("esp32-i2c: capability gap: non-FIFO TX threshold "
                         "boundary is unverified (TRM N versus more-than-N)");
        s->tx_end = (s->tx_ptr - 1) & 31;
        s->tx_bytes = 0;
        s->int_raw |= IRQ_NON_TX;
    }
    update_irq(s);
    return true;
}

static void rx_push(Esp32I2CState *s, uint8_t data)
{
    if (REG(s, A_I2C_FIFO_CONF) & FIFO_NON) {
        s->ram[s->rx_ptr] = data;
    } else if (s->rx_count == 32) {
        warn_report_once("esp32-i2c: capability gap: RX overflow retention "
                         "is unverified; model drops the incoming byte");
        s->int_raw |= IRQ_RX_OVF;
    } else {
        s->rx[(s->rx_head + s->rx_count) & 31] = data;
        s->rx_count++;
    }
    s->rx_ptr = (s->rx_ptr + 1) & 31;
    s->rx_bytes++;
    if ((REG(s, A_I2C_FIFO_CONF) & FIFO_NON) &&
        s->rx_bytes >= MAX(extract32(REG(s, A_I2C_FIFO_CONF), 14, 6), 1)) {
        warn_report_once("esp32-i2c: capability gap: non-FIFO RX threshold "
                         "boundary is unverified (TRM N versus more-than-N)");
        s->rx_end = (s->rx_ptr - 1) & 31;
        s->rx_bytes = 0;
        s->int_raw |= IRQ_NON_RX;
    }
    update_irq(s);
}

static void done(Esp32I2CState *s)
{
    REG(s, A_I2C_CMD + 4 * s->cmd_index) |= BIT(31);
}

static uint64_t low_cycles(Esp32I2CState *s)
{
    return REG(s, A_I2C_LOW_PERIOD) + 1;
}

static uint64_t high_cycles(Esp32I2CState *s)
{
    uint32_t filter = REG(s, A_I2C_SCL_FILTER);
    uint64_t extra = !(filter & 8) ? 7 : (filter & 7) < 3 ? 8 : 6 + (filter & 7);
    return REG(s, A_I2C_HIGH_PERIOD) + extra;
}

static bool master_arbitrate(Esp32I2CState *s)
{
    uint32_t command = REG(s, A_I2C_CMD + 4 * s->cmd_index);
    bool transmit = (s->opcode == I2C_OPCODE_WRITE && s->bit < 8) ||
                    (s->opcode == I2C_OPCODE_READ && s->bit == 8 &&
                     (command & BIT(10)));
    if (transmit && s->sda_drive && !s->sda && s->scl) {
        s->arb_lost = true;
        s->owns_bus = false;
        stop_engine(s, IRQ_ARB);
        drive(s, true, true);
        drive(s, false, true);
        return false;
    }
    return true;
}

static void master_sample(Esp32I2CState *s)
{
    if (!master_arbitrate(s)) {
        return;
    }
    if (s->bit == 8) {
        if (s->opcode == I2C_OPCODE_WRITE) {
            s->ack_rec = s->sda;
        }
    } else if (s->opcode == I2C_OPCODE_READ) {
        unsigned position = (REG(s, A_I2C_CTR) & CTL_RX_LSB) ? s->bit : 7 - s->bit;
        s->shift = deposit32(s->shift, position, 1, s->sda);
    }
}

static void slave_sample(Esp32I2CState *s);
static void slave_fall(Esp32I2CState *s);

static void sample_cb(void *opaque)
{
    Esp32I2CState *s = opaque;
    if (!s->enabled) {
        return;
    }
    if (REG(s, A_I2C_CTR) & BIT(2)) {
        warn_report_once("esp32-i2c: capability gap: exact low-phase SDA "
                         "sample ordering is unverified");
    }
    if (master(s)) {
        if (s->active) {
            master_sample(s);
        }
    } else {
        slave_sample(s);
    }
}

static void prepare_bit(Esp32I2CState *s)
{
    uint32_t command = REG(s, A_I2C_CMD + 4 * s->cmd_index);
    bool level = true;
    if (s->bit == 8) {
        s->main_state = s->opcode == I2C_OPCODE_READ ? 5 : 6;
        if (s->opcode == I2C_OPCODE_READ) {
            level = !!(command & BIT(10));
        }
    } else if (s->opcode == I2C_OPCODE_WRITE) {
        unsigned position = (REG(s, A_I2C_CTR) & CTL_TX_LSB) ? s->bit : 7 - s->bit;
        level = (s->shift >> position) & 1;
        s->main_state = s->address_phase ? 1 : 4;
    } else {
        s->main_state = 3;
    }
    drive(s, false, level);
    if (REG(s, A_I2C_CTR) & BIT(2)) {
        timer_mod(s->sample_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  cycles_ns(s, REG(s, A_I2C_SDA_SAMPLE)));
    }
}

static void begin_bit(Esp32I2CState *s)
{
    s->phase = P_BIT_HOLD;
    s->scl_state = 2;
    s->low_started = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->low_duration = period_ns(s, low_cycles(s));
    drive(s, true, false);
    schedule(s, REG(s, A_I2C_SDA_HOLD));
}

static void high_ready(Esp32I2CState *s)
{
    uint8_t target = s->resume_phase;
    /* Table 21.3-1 already includes filter latency in the high period. The
     * physical rising edge, including any external stretching, is the anchor. */
    s->high_started = MAX(s->high_requested, s->scl_raw_started);
    if (target == P_BIT_SAMPLE) {
        s->high_duration = period_ns(s, high_cycles(s));
        s->scl_state = 5;
        s->phase = P_BIT_SAMPLE;
        schedule(s, (REG(s, A_I2C_CTR) & BIT(2)) ? 0 : REG(s, A_I2C_SDA_SAMPLE));
    } else if (target == P_START_SETUP) {
        s->phase = P_START_SETUP;
        schedule(s, REG(s, A_I2C_RSTART_SETUP));
    } else {
        s->phase = P_STOP_SETUP;
        schedule(s, REG(s, A_I2C_STOP_SETUP));
    }
}

static void raise_clock(Esp32I2CState *s, uint8_t target)
{
    s->high_requested = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->resume_phase = target;
    s->phase = P_WAIT_HIGH;
    s->scl_state = 4;
    drive(s, true, true);
    if (s->phase == P_WAIT_HIGH && s->scl) {
        high_ready(s);
    }
    arm_timeout(s);
}

static void begin_byte(Esp32I2CState *s)
{
    s->bit = 0;
    s->shift = 0;
    s->byte_trans = false;
    if (s->opcode == I2C_OPCODE_WRITE && !tx_pop(s, &s->shift)) {
        stop_engine(s, 0);
        return;
    }
    begin_bit(s);
}

static void esp32_i2c_command(Esp32I2CState *s)
{
    uint32_t command;
    if (!s->active || s->cmd_index >= 16) {
        stop_engine(s, 0);
        return;
    }
    command = REG(s, A_I2C_CMD + 4 * s->cmd_index);
    REG(s, A_I2C_CMD + 4 * s->cmd_index) &= ~BIT(31);
    s->opcode = extract32(command, 11, 3);
    s->remaining = command & 0xff;
    switch (s->opcode) {
    case I2C_OPCODE_RSTART:
        s->phase = P_START_WAIT;
        s->scl_state = 1;
        drive(s, false, true);
        if (s->scl && s->sda) {
            s->high_requested = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->resume_phase = P_START_SETUP;
            high_ready(s);
        } else {
            raise_clock(s, P_START_SETUP);
        }
        arm_timeout(s);
        break;
    case I2C_OPCODE_WRITE:
    case I2C_OPCODE_READ:
        if (!s->remaining) {
            qemu_log_mask(LOG_GUEST_ERROR, "esp32-i2c: zero-length command\n");
            stop_engine(s, 0);
        } else {
            begin_byte(s);
        }
        break;
    case I2C_OPCODE_STOP:
        s->phase = P_STOP_LOW;
        s->scl_state = 6;
        drive(s, true, false);
        drive(s, false, false);
        schedule(s, low_cycles(s));
        break;
    case I2C_OPCODE_END:
        done(s);
        s->active = false;
        s->phase = P_IDLE;
        REG(s, A_I2C_CTR) &= ~CTL_START;
        drive(s, true, false);
        s->int_raw |= IRQ_END;
        update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "esp32-i2c: reserved opcode %u\n", s->opcode);
        stop_engine(s, 0);
        break;
    }
}

static void esp32_i2c_run(void *opaque)
{
    Esp32I2CState *s = opaque;
    uint32_t command = REG(s, A_I2C_CMD + 4 * s->cmd_index);
    int64_t remaining_ns;
    if (!s->enabled || !s->apb_freq) {
        return;
    }
    if (!master(s)) {
        slave_fall(s);
        return;
    }
    if (!s->active) {
        return;
    }
    switch (s->phase) {
    case P_START_SETUP:
        if (!s->scl || !s->sda) {
            warn_report_once("esp32-i2c: capability gap: failed-START "
                             "retry/reset transitions are unverified");
            s->phase = P_START_WAIT;
            return;
        }
        s->owns_bus = true;
        s->address_phase = true;
        s->phase = P_START_HOLD;
        drive(s, false, false);
        s->int_raw |= IRQ_START;
        update_irq(s);
        schedule(s, REG(s, A_I2C_START_HOLD));
        break;
    case P_START_HOLD:
        drive(s, true, false);
        done(s);
        s->cmd_index++;
        esp32_i2c_command(s);
        break;
    case P_BIT_HOLD:
        s->phase = P_BIT_LOW;
        s->scl_state = 3;
        prepare_bit(s);
        remaining_ns = (int64_t)s->low_duration -
                       (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->low_started);
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + MAX(remaining_ns, 1));
        break;
    case P_BIT_LOW:
        raise_clock(s, P_BIT_SAMPLE);
        break;
    case P_BIT_SAMPLE:
        if (!(REG(s, A_I2C_CTR) & BIT(2))) {
            master_sample(s);
        } else {
            master_arbitrate(s);
        }
        if (!s->active) {
            return;
        }
        s->phase = P_BIT_HIGH;
        remaining_ns = (int64_t)s->high_duration -
                       (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->high_started);
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + MAX(remaining_ns, 1));
        break;
    case P_BIT_HIGH:
        /* Wired-AND clock synchronization: an earlier external falling edge
         * terminates the common high period too, through the input callback. */
        s->phase = P_BIT_HOLD;
        drive(s, true, false);
        if (s->bit++ < 8) {
            begin_bit(s);
            break;
        }
        s->byte_trans = true;
        s->int_raw |= IRQ_MASTER_BYTE;
        if (s->opcode == I2C_OPCODE_READ) {
            rx_push(s, s->shift);
        } else if ((command & BIT(8)) && s->ack_rec != !!(command & BIT(9))) {
            /* ACK checking aborts data, but the transaction must terminate on
             * the bus. Public ESP32 target validation exercises successive
             * missing-address probes and NACK transfers without forced resets:
             * esp-idf/components/esp_driver_i2c/test_apps/i2c_test_apps/main/
             * test_i2c_common.cpp (probe device / check nack return value).
             * Keep failed/unexecuted command completion bits clear. */
            warn_report_once("esp32-i2c: capability gap: ACK-error command "
                             "DONE bits are unverified; failed/unexecuted "
                             "commands remain incomplete in the model");
            s->error_stop = true;
            s->int_raw |= IRQ_ACK_ERR;
            s->phase = P_STOP_LOW;
            drive(s, false, false);
            schedule(s, low_cycles(s));
            update_irq(s);
            break;
        }
        s->address_phase = false;
        update_irq(s);
        if (--s->remaining) {
            begin_byte(s);
        } else {
            done(s);
            s->cmd_index++;
            esp32_i2c_command(s);
        }
        break;
    case P_STOP_LOW:
        raise_clock(s, P_STOP_SETUP);
        break;
    case P_STOP_SETUP:
        s->phase = P_STOP_WAIT_SDA;
        drive(s, false, true);
        if (!s->sda_raw) {
            warn_report_once("esp32-i2c: capability gap: failed-STOP "
                             "retry/reset transitions are unverified");
        }
        if (s->sda) {
            s->phase = P_STOP_HOLD;
            schedule(s, REG(s, A_I2C_STOP_HOLD));
        }
        break;
    case P_STOP_HOLD:
        if (!s->error_stop) {
            done(s);
        }
        s->error_stop = false;
        s->active = s->owns_bus = false;
        s->phase = P_IDLE;
        s->scl_state = s->main_state = 0;
        s->tx_end = (s->tx_ptr - 1) & 31;
        s->rx_end = (s->rx_ptr - 1) & 31;
        REG(s, A_I2C_CTR) &= ~CTL_START;
        timer_del(s->timeout_timer);
        s->int_raw |= IRQ_COMPLETE;
        update_irq(s);
        break;
    default:
        break;
    }
}

static void slave_load(Esp32I2CState *s)
{
    s->slave_bits = 0;
    if (!tx_pop(s, &s->slave_shift)) {
        /* Underflow is not an invented successfully transmitted byte. */
        s->slave_phase = S_IGNORE;
        drive(s, false, true);
        return;
    }
    s->slave_phase = S_TX;
    s->main_state = 4;
    drive(s, false, (s->slave_shift >> ((REG(s, A_I2C_CTR) & CTL_TX_LSB) ? 0 : 7)) & 1);
}

static void slave_sample(Esp32I2CState *s)
{
    uint32_t address = REG(s, A_I2C_SLAVE_ADDR);
    switch (s->slave_phase) {
    case S_ADDRESS:
    case S_ADDRESS10:
    case S_RX:
        s->slave_shift = (s->slave_shift << 1) | s->sda;
        if (++s->slave_bits < 8) {
            return;
        }
        s->slave_bits = 0;
        s->byte_trans = true;
        s->slave_ack = true;
        if (s->slave_phase == S_ADDRESS || s->slave_phase == S_ADDRESS10) {
            warn_report_once("esp32-i2c: capability gap: slave byte-interrupt "
                             "inclusion of address bytes is unverified");
        }
        if (s->slave_phase == S_ADDRESS) {
            uint8_t prefix = (address & 0x7f) << 1;
            s->slave_rw = s->slave_shift & 1;
            s->slave_ack = (s->slave_shift & 0xfe) == prefix;
            if (address & BIT(31)) {
                s->slave_ack &= !s->slave_rw || s->ten_selected;
                s->slave_next = s->slave_rw ? S_TX : S_ADDRESS10;
                s->slave_addressed = s->slave_ack && s->slave_rw;
            } else {
                s->slave_addressed = s->slave_ack;
                s->slave_next = s->slave_rw ? S_TX : S_RX;
            }
            s->expect_offset = REG(s, A_I2C_FIFO_CONF) & FIFO_ADDR;
        } else if (s->slave_phase == S_ADDRESS10) {
            s->slave_ack = s->slave_shift == ((address >> 7) & 0xff);
            s->ten_selected = s->slave_addressed = s->slave_ack;
            s->slave_next = S_RX;
        } else {
            uint8_t byte = s->slave_shift;
            if (s->expect_offset) {
                s->rx_ptr = s->tx_ptr = byte & 31;
                s->rx_start = s->tx_start = byte & 31;
                s->expect_offset = false;
            } else {
                if (REG(s, A_I2C_CTR) & CTL_RX_LSB) {
                    byte = revbit8(byte);
                }
                rx_push(s, byte);
            }
            s->int_raw |= IRQ_SLAVE_BYTE;
            s->slave_next = S_RX;
        }
        if (!s->slave_ack) {
            s->slave_next = S_IGNORE;
        }
        s->slave_phase = S_ACK_SETUP;
        s->main_state = s->slave_addressed ? 5 : 2;
        update_irq(s);
        break;
    case S_ACK_CLOCK:
        s->slave_phase = S_ACK_END;
        break;
    case S_TX:
        if (++s->slave_bits == 8) {
            s->slave_phase = S_TX_ACK_SETUP;
            s->byte_trans = true;
            s->int_raw |= IRQ_SLAVE_BYTE;
            update_irq(s);
        }
        break;
    case S_TX_ACK:
        s->ack_rec = s->sda;
        if (s->sda) {
            s->slave_phase = S_IGNORE;
        } else {
            s->slave_phase = S_TX_ACK_END;
        }
        update_irq(s);
        break;
    default:
        break;
    }
}

static void slave_fall(Esp32I2CState *s)
{
    switch (s->slave_phase) {
    case S_ACK_SETUP:
        s->slave_phase = S_ACK_CLOCK;
        drive(s, false, !s->slave_ack);
        break;
    case S_ACK_END:
        s->slave_phase = s->slave_next;
        s->slave_shift = s->slave_bits = 0;
        drive(s, false, true);
        if (s->slave_phase == S_TX) {
            slave_load(s);
        } else {
            s->main_state = s->slave_phase == S_RX ? 3 : 1;
        }
        break;
    case S_TX:
        drive(s, false, (s->slave_shift >> ((REG(s, A_I2C_CTR) & CTL_TX_LSB) ?
                                          s->slave_bits : 7 - s->slave_bits)) & 1);
        break;
    case S_TX_ACK_SETUP:
        s->slave_phase = S_TX_ACK;
        s->main_state = 6;
        drive(s, false, true);
        break;
    case S_TX_ACK_END:
        slave_load(s);
        break;
    default:
        break;
    }
}

static void esp32_i2c_input(Esp32I2CState *s, bool clock, bool level)
{
    bool old = clock ? s->scl : s->sda;
    if (old == level) {
        return;
    }
    if (clock) {
        s->scl = level;
        arm_timeout(s);
        if (master(s)) {
            if (s->active && s->phase == P_WAIT_HIGH && level) {
                high_ready(s);
            } else if (s->active && !level &&
                       (s->phase == P_BIT_HIGH || s->phase == P_BIT_SAMPLE)) {
                timer_del(s->timer);
                if (s->phase == P_BIT_SAMPLE) {
                    master_sample(s);
                }
                s->phase = P_BIT_HIGH;
                esp32_i2c_run(s);
            }
        } else if (s->enabled) {
            if (level != !!(REG(s, A_I2C_CTR) & BIT(2))) {
                timer_mod(s->sample_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          cycles_ns(s, REG(s, A_I2C_SDA_SAMPLE)));
            }
            if (!level) {
                s->low_started = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                schedule(s, REG(s, A_I2C_SDA_HOLD));
            }
        }
    } else {
        s->sda = level;
        if (s->scl && !level) {
            s->bus_busy = true;
            s->byte_trans = false;
            arm_timeout(s);
            if (!master(s)) {
                s->slave_phase = S_ADDRESS;
                s->slave_bits = s->slave_shift = 0;
                s->slave_addressed = false;
                s->main_state = 1;
                s->int_raw |= IRQ_START;
                drive(s, false, true);
                update_irq(s);
            }
        } else if (s->scl && level) {
            s->bus_busy = false;
            timer_del(s->timeout_timer);
            if (!master(s)) {
                timer_del(s->sample_timer);
                s->slave_phase = S_IDLE;
                s->slave_addressed = s->ten_selected = false;
                s->main_state = 0;
                s->rx_end = (s->rx_ptr - 1) & 31;
                s->tx_end = (s->tx_ptr - 1) & 31;
                s->int_raw |= IRQ_COMPLETE;
                drive(s, false, true);
                update_irq(s);
            } else if (s->active && s->phase == P_STOP_WAIT_SDA) {
                s->phase = P_STOP_HOLD;
                schedule(s, REG(s, A_I2C_STOP_HOLD));
            }
        }
    }
    if (master(s) && s->active && s->phase == P_START_WAIT && s->scl && s->sda) {
        s->resume_phase = P_START_SETUP;
        high_ready(s);
    }
}

static void scl_filter_cb(void *opaque)
{
    Esp32I2CState *s = opaque;
    esp32_i2c_input(s, true, s->scl_raw);
}

static void sda_filter_cb(void *opaque)
{
    Esp32I2CState *s = opaque;
    esp32_i2c_input(s, false, s->sda_raw);
}

static void raw_input(Esp32I2CState *s, bool clock, bool level)
{
    uint32_t filter = REG(s, clock ? A_I2C_SCL_FILTER : A_I2C_SDA_FILTER);
    QEMUTimer *timer = clock ? s->scl_filter_timer : s->sda_filter_timer;
    bool old = clock ? s->scl_raw : s->sda_raw;
    if (clock) {
        if (old != level) {
            s->scl_raw_started = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        s->scl_raw = level;
    } else {
        s->sda_raw = level;
    }
    if (old == level || !s->enabled || !s->apb_freq) {
        return;
    }
    timer_del(timer);
    if ((filter & 8) && (filter & 7)) {
        timer_mod(timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + cycles_ns(s, filter & 7));
    } else {
        esp32_i2c_input(s, clock, level);
    }
}

static void scl_input(void *opaque, int n, int level)
{
    raw_input(opaque, true, !!level);
}

static void sda_input(void *opaque, int n, int level)
{
    raw_input(opaque, false, !!level);
}

void esp32_i2c_connect_gpio(Esp32I2CState *s, Esp32GpioState *gpio,
                           unsigned controller)
{
    s->gpio = gpio;
    s->scl_signal = controller ? 95 : 29;
    s->sda_signal = controller ? 96 : 30;
    esp32_gpio_set_peripheral_input(gpio, s->scl_signal,
                                   qdev_get_gpio_in_named(DEVICE(s), "scl-in", 0));
    esp32_gpio_set_peripheral_input(gpio, s->sda_signal,
                                   qdev_get_gpio_in_named(DEVICE(s), "sda-in", 0));
    drive(s, true, s->scl_drive);
    drive(s, false, s->sda_drive);
}

/* Gating freezes APB-clocked state; it does not tri-state pad flip-flops. */
static void pause_clocks(Esp32I2CState *s)
{
    QEMUTimer *timers[] = {s->timer, s->timeout_timer, s->sample_timer,
                          s->scl_filter_timer, s->sda_filter_timer};
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    for (unsigned i = 0; i < ARRAY_SIZE(timers); i++) {
        s->paused_cycles[i] = 0;
        if (timer_pending(timers[i])) {
            uint64_t ns = MAX(timer_expire_time_ns(timers[i]) - now, 0);
            s->paused_cycles[i] = DIV_ROUND_UP(ns * s->apb_freq,
                                              NANOSECONDS_PER_SECOND) + 1;
            timer_del(timers[i]);
        }
    }
    s->paused_high_cycles = MAX(now - s->high_started, 0) * s->apb_freq /
                            NANOSECONDS_PER_SECOND;
    s->paused_low_cycles = MAX(now - s->low_started, 0) * s->apb_freq /
                           NANOSECONDS_PER_SECOND;
}

static void trans_start(Esp32I2CState *s)
{
    if ((REG(s, A_I2C_CTR) & CTL_START) && master(s) && !s->active &&
        s->enabled && s->apb_freq) {
        s->active = true;
        s->cmd_index = 0;
        s->arb_lost = s->timed_out = s->error_stop = false;
        if (!s->owns_bus) {
            s->tx_ptr = s->rx_ptr = 0;
            s->tx_start = s->rx_start = 0;
            s->tx_bytes = s->rx_bytes = 0;
        }
        esp32_i2c_command(s);
    }
}

static void resume_clocks(Esp32I2CState *s)
{
    QEMUTimer *timers[] = {s->timer, s->timeout_timer, s->sample_timer,
                          s->scl_filter_timer, s->sda_filter_timer};
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->high_started = now - cycles_ns(s, s->paused_high_cycles);
    s->high_requested = s->high_started;
    s->scl_raw_started = s->high_started;
    s->low_started = now - cycles_ns(s, s->paused_low_cycles);
    s->low_duration = cycles_ns(s, low_cycles(s));
    s->high_duration = cycles_ns(s, high_cycles(s));
    for (unsigned i = 0; i < ARRAY_SIZE(timers); i++) {
        if (s->paused_cycles[i]) {
            timer_mod(timers[i], now + cycles_ns(s, s->paused_cycles[i] - 1));
            s->paused_cycles[i] = 0;
        }
    }
    /* Input synchronizers see the current pads when the APB clock returns. */
    if (s->scl != s->scl_raw && !timer_pending(s->scl_filter_timer)) {
        timer_mod(s->scl_filter_timer, now + cycles_ns(s, 1));
    }
    if (s->sda != s->sda_raw && !timer_pending(s->sda_filter_timer)) {
        timer_mod(s->sda_filter_timer, now + cycles_ns(s, 1));
    }
    trans_start(s);
}

void esp32_i2c_set_enabled(Esp32I2CState *s, bool enabled)
{
    if (s->enabled == enabled) {
        return;
    }
    if (s->enabled && s->apb_freq) {
        pause_clocks(s);
    }
    s->enabled = enabled;
    if (enabled && s->apb_freq) {
        resume_clocks(s);
    }
}

void esp32_i2c_set_apb_freq(Esp32I2CState *s, uint32_t hz)
{
    if (s->apb_freq == hz) {
        return;
    }
    if (s->enabled && s->apb_freq) {
        pause_clocks(s);
    }
    s->apb_freq = hz;
    s->ns_remainder = s->period_fraction = 0;
    if (s->enabled && hz) {
        resume_clocks(s);
    }
}

static uint64_t esp32_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    Esp32I2CState *s = opaque;
    if (addr >= 0x100 && addr < 0x180) {
        return s->ram[(addr - 0x100) / 4];
    }
    switch (addr) {
    case A_I2C_STATUS:
        if (s->active || s->bus_busy) {
            warn_report_once("esp32-i2c: capability gap: exact status-FSM "
                             "transition boundaries are unverified");
        }
        return s->ack_rec | (s->slave_rw << 1) | (s->timed_out << 2) |
               (s->arb_lost << 3) | (s->bus_busy << 4) |
               (s->slave_addressed << 5) | (s->byte_trans << 6) |
               (s->rx_count << 8) | (s->tx_count << 18) |
               (s->main_state << 24) | (s->scl_state << 28);
    case A_I2C_FIFO_ST:
        return s->rx_start | (s->rx_end << 5) | (s->tx_start << 10) | (s->tx_end << 15);
    case A_I2C_FIFO_DATA: {
        uint8_t value;
        if (!s->rx_count) {
            warn_report_once("esp32-i2c: capability gap: empty RX FIFO read "
                             "value is unverified; model returns zero");
            qemu_log_mask(LOG_GUEST_ERROR, "esp32-i2c: reading empty RX FIFO\n");
            return 0;
        }
        value = s->rx[s->rx_head];
        s->rx_head = (s->rx_head + 1) & 31;
        s->rx_count--;
        update_irq(s);
        return value;
    }
    case A_I2C_INT_RAW: return s->int_raw;
    case A_I2C_INT_ENA: return s->int_ena;
    case A_I2C_INT_ST: return s->int_raw & s->int_ena;
    case A_I2C_INT_CLR: return 0;
    default: return addr < 0x100 ? REG(s, addr) : 0;
    }
}

static uint32_t reg_mask(hwaddr addr)
{
    switch (addr) {
    case A_I2C_LOW_PERIOD:
    case A_I2C_HIGH_PERIOD:
    case A_I2C_STOP_HOLD: return 0x3fff;
    case A_I2C_CTR: return 0x1f7;
    case A_I2C_TIMEOUT: return 0xfffff;
    case A_I2C_SLAVE_ADDR: return 0x80007fff;
    case A_I2C_FIFO_CONF: return 0x3ffffff;
    case A_I2C_SDA_HOLD:
    case A_I2C_SDA_SAMPLE:
    case A_I2C_START_HOLD:
    case A_I2C_RSTART_SETUP:
    case A_I2C_STOP_SETUP: return 0x3ff;
    case A_I2C_SCL_FILTER:
    case A_I2C_SDA_FILTER: return 0xf;
    case A_I2C_CMD ... A_I2C_CMD + 15 * 4: return 0x80003fff;
    case 0xf8: return UINT32_MAX;
    default: return 0;
    }
}

static void esp32_i2c_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    Esp32I2CState *s = opaque;
    if (addr >= 0x100 && addr < 0x180) {
        s->ram[(addr - 0x100) / 4] = value;
        return;
    }
    switch (addr) {
    case A_I2C_CTR:
        if ((value & BIT(8)) && !(REG(s, addr) & BIT(8))) {
            warn_report_once("esp32-i2c: capability gap: CTR clock-force bit8 "
                             "is reserved in TRM but present in official "
                             "headers; its effect is unverified");
        }
        if ((s->active || s->bus_busy) &&
            ((REG(s, addr) ^ value) & (reg_mask(addr) & ~CTL_START))) {
            warn_report_once("esp32-i2c: capability gap: active control "
                             "reconfiguration behavior is unverified");
        }
        REG(s, addr) = value & reg_mask(addr);
        drive(s, true, s->scl_drive);
        drive(s, false, s->sda_drive);
        trans_start(s);
        break;
    case A_I2C_FIFO_CONF:
        if (((value ^ REG(s, addr)) & (FIFO_NON | FIFO_ADDR)) &&
            (s->active || s->bus_busy || s->tx_count || s->rx_count)) {
            warn_report_once("esp32-i2c: capability gap: live FIFO/non-FIFO "
                             "backing-store/mode-change behavior is unverified");
        }
        REG(s, addr) = value & reg_mask(addr);
        if (value & BIT(12)) {
            s->rx_head = s->rx_count = s->rx_ptr = s->rx_start = s->rx_end = 0;
            s->rx_bytes = 0;
        }
        if (value & BIT(13)) {
            s->tx_head = s->tx_count = s->tx_ptr = s->tx_start = s->tx_end = 0;
            s->tx_bytes = 0;
        }
        update_irq(s);
        break;
    case A_I2C_FIFO_DATA:
        if (s->tx_count == 32) {
            warn_report_once("esp32-i2c: capability gap: full TX FIFO write "
                             "retention is unverified; model discards the write");
            qemu_log_mask(LOG_GUEST_ERROR, "esp32-i2c: writing full TX FIFO\n");
        } else if (!(REG(s, A_I2C_FIFO_CONF) & BIT(13))) {
            s->tx[(s->tx_head + s->tx_count) & 31] = value;
            s->tx_count++;
        }
        update_irq(s);
        break;
    case A_I2C_INT_ENA:
        s->int_ena = value & IRQ_MASK;
        update_irq(s);
        break;
    case A_I2C_INT_CLR:
        s->int_raw &= ~(value & IRQ_MASK);
        update_irq(s);
        break;
    default:
        if (s->active && addr != 0xf8 && addr < A_I2C_CMD &&
            reg_mask(addr) && REG(s, addr) != (value & reg_mask(addr))) {
            warn_report_once("esp32-i2c: capability gap: active timing/address "
                             "reconfiguration behavior is unverified");
        }
        if (addr < 0x100) {
            REG(s, addr) = value & reg_mask(addr);
        }
        break;
    }
}

static void esp32_i2c_reset_hold(Object *obj, ResetType type)
{
    Esp32I2CState *s = Esp32_I2C(obj);
    timer_del(s->timer);
    timer_del(s->timeout_timer);
    timer_del(s->scl_filter_timer);
    timer_del(s->sda_filter_timer);
    timer_del(s->sample_timer);
    memset(s->reg, 0, sizeof(s->reg));
    memset(s->ram, 0, sizeof(s->ram));
    memset(s->tx, 0, sizeof(s->tx));
    memset(s->rx, 0, sizeof(s->rx));
    REG(s, A_I2C_CTR) = 3;
    /* TRM v5.8 registers21.15/16, matching original ESP32 register headers. */
    REG(s, A_I2C_START_HOLD) = REG(s, A_I2C_RSTART_SETUP) = 8;
    REG(s, A_I2C_FIFO_CONF) = (0x15 << 14) | (0x15 << 20);
    REG(s, A_I2C_SCL_FILTER) = REG(s, A_I2C_SDA_FILTER) = 8;
    REG(s, 0xf8) = 0x16042000;
    s->int_raw = s->int_ena = 0;
    s->tx_head = s->rx_head = s->tx_count = s->rx_count = 0;
    s->tx_ptr = s->rx_ptr = s->tx_start = s->rx_start = s->tx_end = s->rx_end = 0;
    s->tx_bytes = s->rx_bytes = 0;
    s->active = s->owns_bus = s->bus_busy = false;
    s->phase = P_IDLE;
    s->slave_phase = S_IDLE;
    s->slave_addressed = s->ten_selected = s->slave_rw = false;
    s->ack_rec = s->arb_lost = s->timed_out = s->byte_trans = false;
    s->main_state = s->scl_state = 0;
    s->ns_remainder = s->period_fraction = 0;
    s->low_duration = s->high_duration = 0;
    memset(s->paused_cycles, 0, sizeof(s->paused_cycles));
    s->paused_high_cycles = s->paused_low_cycles = 0;
    s->resume_phase = s->cmd_index = s->opcode = s->bit = s->shift = 0;
    s->remaining = 0;
    s->address_phase = s->error_stop = s->slave_ack = s->expect_offset = false;
    s->slave_next = s->slave_bits = s->slave_shift = 0;
    s->high_started = s->low_started = s->high_requested = s->scl_raw_started = 0;
    drive(s, true, true);
    drive(s, false, true);
    update_irq(s);
}

static const MemoryRegionOps esp32_i2c_ops = {
    .read = esp32_i2c_read, .write = esp32_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void esp32_i2c_init(Object *obj)
{
    Esp32I2CState *s = Esp32_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    s->apb_freq = 80000000;
    s->enabled = true;
    s->scl_raw = s->sda_raw = s->scl = s->sda = true;
    s->scl_drive = s->sda_drive = true;
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, esp32_i2c_run, s);
    s->timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timeout_cb, s);
    s->scl_filter_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, scl_filter_cb, s);
    s->sda_filter_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sda_filter_cb, s);
    s->sample_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sample_cb, s);
    memory_region_init_io(&s->iomem, obj, &esp32_i2c_ops, s, TYPE_ESP32_I2C, ESP32_I2C_MEM_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(DEVICE(s), scl_input, "scl-in", 1);
    qdev_init_gpio_in_named(DEVICE(s), sda_input, "sda-in", 1);
    s->bus = i2c_init_bus(DEVICE(s), "i2c");
}

static void esp32_i2c_finalize(Object *obj)
{
    Esp32I2CState *s = Esp32_I2C(obj);
    timer_free(s->timer);
    timer_free(s->timeout_timer);
    timer_free(s->scl_filter_timer);
    timer_free(s->sda_filter_timer);
    timer_free(s->sample_timer);
}

int esp32_i2c_post_load(void *opaque, int version)
{
    Esp32I2CState *s = opaque;
    if (!esp32_i2c_state_valid(s)) {
        return -EINVAL;
    }
    if (s->active || s->bus_busy) {
        warn_report_once("esp32-i2c: capability gap: post-load physical-pad "
                         "transition ordering is unverified");
    }
    drive(s, true, s->scl_drive);
    drive(s, false, s->sda_drive);
    update_irq(s);
    return 0;
}


static void esp32_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32_i2c_reset_hold;
    dc->vmsd = &vmstate_esp32_i2c;
}

static const TypeInfo esp32_i2c_type_info = {
    .name = TYPE_ESP32_I2C, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32I2CState), .instance_init = esp32_i2c_init,
    .instance_finalize = esp32_i2c_finalize, .class_init = esp32_i2c_class_init,
};
static void esp32_i2c_register_types(void)
{
    type_register_static(&esp32_i2c_type_info);
}
type_init(esp32_i2c_register_types)
