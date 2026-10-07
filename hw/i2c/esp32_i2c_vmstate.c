/* ESP32 I2C VMState definitions and structural validation.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "migration/vmstate.h"
#include "hw/i2c/esp32_i2c.h"

bool esp32_i2c_state_valid(Esp32I2CState *s)
{
    if (s->tx_count > 32 || s->rx_count > 32 ||
        s->tx_head >= 32 || s->rx_head >= 32 ||
        s->tx_ptr >= 32 || s->rx_ptr >= 32 ||
        s->tx_start >= 32 || s->tx_end >= 32 ||
        s->rx_start >= 32 || s->rx_end >= 32 ||
        s->cmd_index > 16 || (s->active && s->cmd_index >= 16) ||
        s->phase > ESP32_I2C_PHASE_MAX ||
        s->resume_phase > ESP32_I2C_PHASE_MAX ||
        s->slave_phase > ESP32_I2C_SLAVE_PHASE_MAX ||
        s->slave_next > ESP32_I2C_SLAVE_PHASE_MAX ||
        s->opcode > 7 || s->bit > 9 || s->remaining > 255 ||
        s->slave_bits > 8 || s->main_state > 6 || s->scl_state > 6) {
        return false;
    }
    if (s->apb_freq && (s->ns_remainder >= s->apb_freq ||
                        s->period_fraction >= s->apb_freq)) {
        return false;
    }
    return true;
}

const VMStateDescription vmstate_esp32_i2c = {
    .name = TYPE_ESP32_I2C, .version_id = 1, .minimum_version_id = 1,
    .post_load = esp32_i2c_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(reg, Esp32I2CState, 64),
        VMSTATE_UINT8_ARRAY(ram, Esp32I2CState, 32),
        VMSTATE_UINT8_ARRAY(tx, Esp32I2CState, 32),
        VMSTATE_UINT8_ARRAY(rx, Esp32I2CState, 32),
#define U8(f) VMSTATE_UINT8(f, Esp32I2CState)
#define U16(f) VMSTATE_UINT16(f, Esp32I2CState)
#define U32(f) VMSTATE_UINT32(f, Esp32I2CState)
#define B(f) VMSTATE_BOOL(f, Esp32I2CState)
        U8(tx_head), U8(tx_count), U8(rx_head), U8(rx_count),
        U8(tx_ptr), U8(rx_ptr), U8(tx_start), U8(tx_end), U8(rx_start), U8(rx_end),
        U16(tx_bytes), U16(rx_bytes), U32(int_raw), U32(int_ena), U32(apb_freq),
        VMSTATE_UINT64(ns_remainder, Esp32I2CState), B(enabled),
        VMSTATE_UINT64(period_fraction, Esp32I2CState),
        VMSTATE_UINT64(low_duration, Esp32I2CState),
        VMSTATE_UINT64(high_duration, Esp32I2CState),
        VMSTATE_UINT64_ARRAY(paused_cycles, Esp32I2CState, 5),
        VMSTATE_UINT64(paused_high_cycles, Esp32I2CState),
        VMSTATE_UINT64(paused_low_cycles, Esp32I2CState),
        B(scl_raw), B(sda_raw), B(scl), B(sda), B(scl_drive), B(sda_drive),
        B(bus_busy), B(active), B(owns_bus), B(ack_rec), B(arb_lost), B(timed_out), B(byte_trans),
        U8(phase), U8(resume_phase), U8(cmd_index), U8(opcode), U8(bit), U8(shift), U16(remaining),
        B(address_phase), B(error_stop), VMSTATE_INT64(high_started, Esp32I2CState),
        VMSTATE_INT64(low_started, Esp32I2CState),
        VMSTATE_INT64(high_requested, Esp32I2CState),
        VMSTATE_INT64(scl_raw_started, Esp32I2CState), U8(main_state), U8(scl_state),
        U8(slave_phase), U8(slave_next), U8(slave_bits), U8(slave_shift),
        B(slave_addressed), B(slave_rw), B(slave_ack), B(ten_selected), B(expect_offset),
        VMSTATE_TIMER_PTR(timer, Esp32I2CState), VMSTATE_TIMER_PTR(timeout_timer, Esp32I2CState),
        VMSTATE_TIMER_PTR(scl_filter_timer, Esp32I2CState),
        VMSTATE_TIMER_PTR(sda_filter_timer, Esp32I2CState), VMSTATE_TIMER_PTR(sample_timer, Esp32I2CState),
        VMSTATE_END_OF_LIST()
#undef U8
#undef U16
#undef U32
#undef B
    },
};
