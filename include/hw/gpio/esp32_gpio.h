/*
 * ESP32 digital pads, IO MUX and GPIO matrix.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef HW_GPIO_ESP32_GPIO_H
#define HW_GPIO_ESP32_GPIO_H

#include "hw/sysbus.h"
#include "hw/registerfields.h"

#define TYPE_ESP32_GPIO "esp32.gpio"
#define ESP32_GPIO(obj) OBJECT_CHECK(Esp32GpioState, (obj), TYPE_ESP32_GPIO)
#define ESP32_GPIO_GET_CLASS(obj) OBJECT_GET_CLASS(Esp32GpioClass, (obj), TYPE_ESP32_GPIO)
#define ESP32_GPIO_CLASS(klass) OBJECT_CLASS_CHECK(Esp32GpioClass, (klass), TYPE_ESP32_GPIO)
#define ESP32_GPIO_PADS 40
#define ESP32_GPIO_SIGNALS 256
#define ESP32_GPIO_OUTPUTS 259
#define ESP32_GPIO_MCLK0 257
#define ESP32_GPIO_MCLK1 258
#define ESP32_GPIO_EXT_DRIVERS 4
#define ESP32_STRAP_MODE_FLASH_BOOT 0x12
#define ESP32_STRAP_MODE_UART_BOOT 0x0f

REG32(GPIO_STRAP, 0x0038)

typedef enum Esp32PadLevel {
    ESP32_PAD_LOW,
    ESP32_PAD_HIGH,
    ESP32_PAD_Z,
    ESP32_PAD_X,
} Esp32PadLevel;

typedef struct Esp32GpioState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    MemoryRegion io_mux;
    qemu_irq irq;
    qemu_irq nmi;
    qemu_irq app_irq;
    qemu_irq app_nmi;
    qemu_irq pad_level[ESP32_GPIO_PADS];
    GPtrArray *pad_listeners[ESP32_GPIO_PADS];
    GPtrArray *routing_listeners;
    qemu_irq signal_in[ESP32_GPIO_SIGNALS];
    uint32_t strap_mode;
    uint32_t regs[0x600 / 4];
    uint32_t mux[0xa0 / 4];
    uint8_t peripheral_value[ESP32_GPIO_OUTPUTS];
    uint8_t peripheral_enable[ESP32_GPIO_OUTPUTS];
    uint8_t peripheral_open_drain[ESP32_GPIO_OUTPUTS];
    uint8_t peripheral_known[ESP32_GPIO_OUTPUTS];
    uint8_t external[ESP32_GPIO_PADS * ESP32_GPIO_EXT_DRIVERS];
    uint8_t resolved[ESP32_GPIO_PADS];
    uint8_t drive[ESP32_GPIO_PADS];
    uint8_t external_resolved[ESP32_GPIO_PADS];
    uint8_t signal_sample[ESP32_GPIO_SIGNALS];
    uint8_t input_sample[ESP32_GPIO_PADS];
    bool resolving;
    bool dirty;
    bool resetting;
    char *pin_trace;
    FILE *trace;
    int64_t trace_time;
    int64_t trace_last_virtual;
    int64_t trace_epoch;
} Esp32GpioState;

typedef struct Esp32GpioClass {
    SysBusDeviceClass parent_class;
} Esp32GpioClass;

void esp32_gpio_set_peripheral_output(Esp32GpioState *s, unsigned signal,
                                     bool level, bool enable, bool open_drain);
void esp32_gpio_set_peripheral_unknown(Esp32GpioState *s, unsigned signal);
void esp32_gpio_set_peripheral_input(Esp32GpioState *s, unsigned signal,
                                    qemu_irq input);
void esp32_gpio_set_external_drive(Esp32GpioState *s, unsigned pad,
                                  unsigned driver, Esp32PadLevel level);
Esp32PadLevel esp32_gpio_get_pad(Esp32GpioState *s, unsigned pad);
void esp32_gpio_add_pad_listener(Esp32GpioState *s, unsigned pad, qemu_irq sink);
void esp32_gpio_remove_pad_listener(Esp32GpioState *s, unsigned pad, qemu_irq sink);
void esp32_gpio_add_routing_listener(Esp32GpioState *s, qemu_irq sink);
void esp32_gpio_remove_routing_listener(Esp32GpioState *s, qemu_irq sink);
bool esp32_gpio_output_is_routed(Esp32GpioState *s, unsigned signal);

#endif
