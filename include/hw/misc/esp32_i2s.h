/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef HW_MISC_ESP32_I2S_H
#define HW_MISC_ESP32_I2S_H
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"
#include "hw/gpio/esp32_gpio.h"
#define TYPE_ESP32_I2S "esp32.i2s"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32I2SState, ESP32_I2S)
#define ESP32_I2S_FIFO_WORDS 64

typedef struct Esp32I2SChannel {
    QEMUTimer *timer;
    uint32_t fifo[ESP32_I2S_FIFO_WORDS];
    uint32_t fifo_descriptor[ESP32_I2S_FIFO_WORDS];
    uint8_t fifo_flags[ESP32_I2S_FIFO_WORDS];
    uint32_t fifo_head;
    uint32_t fifo_count;
    uint32_t descriptor;
    uint32_t descriptor_words[3];
    uint32_t descriptor_offset;
    uint32_t descriptor_limit;
    uint32_t dma_words;
    uint32_t frame[2];
    uint32_t shift;
    uint32_t bit;
    uint32_t slot;
    uint32_t phase;
    uint32_t clock_bit;
    uint32_t analytic_sample_progress;
    uint64_t clock_remainder;
    uint64_t analytic_frame_data;
    uint64_t analytic_frame_ws;
    uint64_t analytic_frame_remainder;
    uint64_t analytic_frame_half_num;
    uint64_t analytic_frame_half_den;
    int64_t deadline;
    int64_t analytic_frame_origin;
    bool link_active;
    bool descriptor_loaded;
    bool clock_level;
    bool ws_level;
    bool data_level;
    bool frame_valid;
    bool started;
    bool synchronized;
    bool previous_ws;
    bool mono_pending;
    bool analytic_clock;
    bool analytic_frame_pending;
    bool analytic_origin_falling;
    bool analytic_paired;
    uint8_t analytic_frame_count;
    int64_t paused_ns;
} Esp32I2SChannel;

struct Esp32I2SState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    Esp32GpioState *gpio;
    unsigned controller;
    qemu_irq *inputs;
    uint8_t input_level[24];
    uint32_t regs[64];
    Esp32I2SChannel tx;
    Esp32I2SChannel rx;
    QEMUTimer *mclk_timer;
    uint64_t mclk_remainder;
    int64_t mclk_deadline;
    int64_t mclk_paused_ns;
    bool mclk_level;
    bool enabled;
    bool pumping_dma;
    uint32_t apll_hz;
    uint64_t apll_numerator;
    uint64_t apll_denominator;
    uint32_t warned_capabilities;
};
extern const VMStateDescription vmstate_esp32_i2s;
bool esp32_i2s_mode_supported(Esp32I2SState *s);
bool esp32_i2s_state_valid(Esp32I2SState *s);
int esp32_i2s_post_load(void *opaque, int version);

void esp32_i2s_connect_gpio(Esp32I2SState *s, Esp32GpioState *gpio,
                          unsigned controller);
void esp32_i2s_set_enabled(Esp32I2SState *s, bool enabled);
void esp32_i2s_set_apll(Esp32I2SState *s, uint32_t hz);
void esp32_i2s_set_apll_rate(Esp32I2SState *s, uint64_t numerator,
                             uint64_t denominator);
#endif
