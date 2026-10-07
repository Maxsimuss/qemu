/*
 * ESP32 external I2C controllers.
 * Copyright (c) 2026
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef ESP32_I2C_H
#define ESP32_I2C_H

#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "qemu/timer.h"
#include "hw/registerfields.h"
#include "hw/gpio/esp32_gpio.h"

#define TYPE_ESP32_I2C "esp32.i2c"
#define Esp32_I2C(obj) OBJECT_CHECK(Esp32I2CState, (obj), TYPE_ESP32_I2C)
#define ESP32_I2C_MEM_SIZE 0x180
#define ESP32_I2C_FIFO_LENGTH 32
#define ESP32_I2C_CMD_COUNT 16
#define ESP32_I2C_PHASE_MAX 13
#define ESP32_I2C_SLAVE_PHASE_MAX 11

typedef enum {
    I2C_OPCODE_RSTART, I2C_OPCODE_WRITE, I2C_OPCODE_READ,
    I2C_OPCODE_STOP, I2C_OPCODE_END,
} i2c_opcode_t;

typedef struct Esp32I2CState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus; /* legacy attachment name; not used to bypass physical pins */
    Esp32GpioState *gpio;
    uint32_t scl_signal, sda_signal;
    QEMUTimer *timer, *timeout_timer, *scl_filter_timer, *sda_filter_timer;
    QEMUTimer *sample_timer;
    uint32_t apb_freq;
    uint64_t ns_remainder, period_fraction;
    uint64_t low_duration, high_duration;
    uint64_t paused_cycles[5], paused_high_cycles, paused_low_cycles;
    bool enabled;
    uint32_t reg[64];
    uint8_t ram[32], tx[32], rx[32];
    uint8_t tx_head, tx_count, rx_head, rx_count;
    uint8_t tx_ptr, rx_ptr, tx_start, tx_end, rx_start, rx_end;
    uint16_t tx_bytes, rx_bytes;
    uint32_t int_raw, int_ena;
    bool scl_raw, sda_raw, scl, sda;
    bool scl_drive, sda_drive, bus_busy, active, owns_bus;
    bool ack_rec, arb_lost, timed_out, byte_trans;
    uint8_t phase, resume_phase, cmd_index, opcode, bit, shift;
    uint16_t remaining;
    bool address_phase, error_stop;
    int64_t high_started, low_started, high_requested, scl_raw_started;
    uint8_t main_state, scl_state;
    uint8_t slave_phase, slave_next, slave_bits, slave_shift;
    bool slave_addressed, slave_rw, slave_ack, ten_selected, expect_offset;
} Esp32I2CState;

extern const VMStateDescription vmstate_esp32_i2c;
bool esp32_i2c_state_valid(Esp32I2CState *s);
int esp32_i2c_post_load(void *opaque, int version);

void esp32_i2c_connect_gpio(Esp32I2CState *s, Esp32GpioState *gpio,
                           unsigned controller);
void esp32_i2c_set_apb_freq(Esp32I2CState *s, uint32_t hz);
void esp32_i2c_set_enabled(Esp32I2CState *s, bool enabled);

REG32(I2C_LOW_PERIOD, 0x00)
REG32(I2C_CTR, 0x04)
    FIELD(I2C_CTR, SDA_FORCE_OUT, 0, 1)
    FIELD(I2C_CTR, SCL_FORCE_OUT, 1, 1)
    FIELD(I2C_CTR, SAMPLE_SCL_LEVEL, 2, 1)
    FIELD(I2C_CTR, MS_MODE, 4, 1)
    FIELD(I2C_CTR, TRANS_START, 5, 1)
    FIELD(I2C_CTR, TX_LSB_FIRST, 6, 1)
    FIELD(I2C_CTR, RX_LSB_FIRST, 7, 1)
    FIELD(I2C_CTR, CLK_EN, 8, 1)
REG32(I2C_STATUS, 0x08)
REG32(I2C_TIMEOUT, 0x0c)
REG32(I2C_SLAVE_ADDR, 0x10)
REG32(I2C_FIFO_ST, 0x14)
REG32(I2C_FIFO_CONF, 0x18)
    FIELD(I2C_FIFO_CONF, RX_FULL, 0, 5)
    FIELD(I2C_FIFO_CONF, TX_EMPTY, 5, 5)
    FIELD(I2C_FIFO_CONF, NONFIFO_EN, 10, 1)
    FIELD(I2C_FIFO_CONF, ADDR_CFG_EN, 11, 1)
    FIELD(I2C_FIFO_CONF, RX_FIFO_RST, 12, 1)
    FIELD(I2C_FIFO_CONF, TX_FIFO_RST, 13, 1)
    FIELD(I2C_FIFO_CONF, NONFIFO_RX, 14, 6)
    FIELD(I2C_FIFO_CONF, NONFIFO_TX, 20, 6)
REG32(I2C_FIFO_DATA, 0x1c)
REG32(I2C_INT_RAW, 0x20)
REG32(I2C_INT_CLR, 0x24)
REG32(I2C_INT_ENA, 0x28)
REG32(I2C_INT_ST, 0x2c)
REG32(I2C_SDA_HOLD, 0x30)
REG32(I2C_SDA_SAMPLE, 0x34)
REG32(I2C_HIGH_PERIOD, 0x38)
REG32(I2C_START_HOLD, 0x40)
REG32(I2C_RSTART_SETUP, 0x44)
REG32(I2C_STOP_HOLD, 0x48)
REG32(I2C_STOP_SETUP, 0x4c)
REG32(I2C_SCL_FILTER, 0x50)
REG32(I2C_SDA_FILTER, 0x54)
REG32(I2C_CMD, 0x58)
    FIELD(I2C_CMD, BYTE_NUM, 0, 8)
    FIELD(I2C_CMD, ACK_CHECK_EN, 8, 1)
    FIELD(I2C_CMD, ACK_EXP, 9, 1)
    FIELD(I2C_CMD, ACK_VAL, 10, 1)
    FIELD(I2C_CMD, OPCODE, 11, 3)
    FIELD(I2C_CMD, DONE, 31, 1)
#endif
