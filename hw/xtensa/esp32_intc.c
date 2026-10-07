/*
 * ESP32 Interrupt Matrix
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/xtensa/esp32_intc.h"
#include "migration/vmstate.h"

#define INTMATRIX_UNINT_VALUE   6

#define IRQ_MAP(cpu, input) s->irq_map[cpu][input]

static void esp32_intmatrix_update(Esp32IntMatrixState *s, unsigned cpu)
{
    if (!s->outputs[cpu]) {
        return;
    }
    for (unsigned output = 0; output < s->cpu[cpu]->env.config->nextint; output++) {
        bool level = false;
        for (unsigned source = 0; source < ESP32_INT_MATRIX_INPUTS; source++) {
            if (IRQ_MAP(cpu, source) == s->cpu[cpu]->env.config->extint[output]) {
                level |= s->source_level[cpu][source];
            }
        }
        qemu_set_irq(s->outputs[cpu][output], level);
        qemu_set_irq(s->route_output[cpu * 32 + s->cpu[cpu]->env.config->extint[output]],
                     level);
    }
}

static void esp32_intmatrix_irq_handler(void *opaque, int n, int level)
{
    Esp32IntMatrixState *s = opaque;
    for (unsigned cpu = 0; cpu < ESP32_CPU_COUNT; cpu++) {
        /* Sources deliver levels, not retrigger requests. GPIO pad edges
         * often publish the same deasserted IRQ; rescanning every output
         * then calling every CPU input adds no hardware transition. MMIO
         * remapping and post-load/reset still explicitly drive outputs. */
        if (s->source_level[cpu][n] == !!level) {
            continue;
        }
        s->source_level[cpu][n] = !!level;
        esp32_intmatrix_update(s, cpu);
    }
}

static void esp32_intmatrix_cpu_irq(void *opaque, int n, int level)
{
    Esp32IntMatrixState *s = opaque;
    unsigned cpu = n / ESP32_INT_MATRIX_INPUTS;
    unsigned source = n % ESP32_INT_MATRIX_INPUTS;
    if (s->source_level[cpu][source] == !!level) {
        return;
    }
    s->source_level[cpu][source] = !!level;
    esp32_intmatrix_update(s, cpu);
}

static inline uint8_t* get_map_entry(Esp32IntMatrixState* s, hwaddr addr)
{
    int source_index = addr / sizeof(uint32_t);
    if (source_index >= ESP32_INT_MATRIX_INPUTS * ESP32_CPU_COUNT) {
        error_report("%s: source_index %d out of range", __func__, source_index);
        return NULL;
    }
    int cpu_index = source_index / ESP32_INT_MATRIX_INPUTS;
    source_index = source_index % ESP32_INT_MATRIX_INPUTS;
    return &IRQ_MAP(cpu_index, source_index);
}

static uint64_t esp32_intmatrix_read(void* opaque, hwaddr addr, unsigned int size)
{
    Esp32IntMatrixState *s = ESP32_INTMATRIX(opaque);
    uint8_t* map_entry = get_map_entry(s, addr);
    return (map_entry != NULL) ? *map_entry : 0;
}

static void esp32_intmatrix_write(void* opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    Esp32IntMatrixState *s = ESP32_INTMATRIX(opaque);
    uint8_t* map_entry = get_map_entry(s, addr);
    if (map_entry != NULL) {
        *map_entry = value & 0x1f;
        esp32_intmatrix_update(s, addr / (sizeof(uint32_t) * ESP32_INT_MATRIX_INPUTS));
    }
}

static const MemoryRegionOps esp_intmatrix_ops = {
    .read =  esp32_intmatrix_read,
    .write = esp32_intmatrix_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_intmatrix_reset_hold(Object *obj, ResetType type)
{
    Esp32IntMatrixState *s = ESP32_INTMATRIX(obj);
    memset(s->irq_map, INTMATRIX_UNINT_VALUE, sizeof(s->irq_map));
    memset(s->source_level, 0, sizeof(s->source_level));
    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        if (s->outputs[i] == NULL) {
            continue;
        }
        for (int int_index = 0; int_index < s->cpu[i]->env.config->nextint; ++int_index) {
            qemu_irq_lower(s->outputs[i][int_index]);
            qemu_irq_lower(s->route_output[i * 32 + s->cpu[i]->env.config->extint[int_index]]);
        }
    }

}

static void esp32_intmatrix_realize(DeviceState *dev, Error **errp)
{
    Esp32IntMatrixState *s = ESP32_INTMATRIX(dev);

    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        if (s->cpu[i]) {
            s->outputs[i] = xtensa_get_extints(&s->cpu[i]->env);
        }
    }
    esp32_intmatrix_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
}

static void esp32_intmatrix_init(Object *obj)
{
    Esp32IntMatrixState *s = ESP32_INTMATRIX(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp_intmatrix_ops, s,
                          TYPE_ESP32_INTMATRIX, ESP32_INT_MATRIX_INPUTS * ESP32_CPU_COUNT * sizeof(uint32_t));
    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_in(DEVICE(s), esp32_intmatrix_irq_handler, ESP32_INT_MATRIX_INPUTS);
    qdev_init_gpio_in_named(DEVICE(s), esp32_intmatrix_cpu_irq, "cpu-source",
                            ESP32_CPU_COUNT * ESP32_INT_MATRIX_INPUTS);
    qdev_init_gpio_out_named(DEVICE(s), s->route_output, "cpu-irq",
                             ESP32_CPU_COUNT * 32);
}

static int esp32_intmatrix_post_load(void *opaque, int version)
{
    Esp32IntMatrixState *s = opaque;
    for (unsigned cpu = 0; cpu < ESP32_CPU_COUNT; cpu++) {
        esp32_intmatrix_update(s, cpu);
    }
    return 0;
}

static const VMStateDescription vmstate_esp32_intmatrix = {
    .name = TYPE_ESP32_INTMATRIX, .version_id = 1, .minimum_version_id = 1,
    .post_load = esp32_intmatrix_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_2DARRAY(irq_map, Esp32IntMatrixState, ESP32_CPU_COUNT,
                             ESP32_INT_MATRIX_INPUTS),
        VMSTATE_UINT8_2DARRAY(source_level, Esp32IntMatrixState, ESP32_CPU_COUNT,
                             ESP32_INT_MATRIX_INPUTS),
        VMSTATE_END_OF_LIST()
    },
};

static Property esp32_intmatrix_properties[] = {
    DEFINE_PROP_LINK("cpu0", Esp32IntMatrixState, cpu[0], TYPE_XTENSA_CPU, XtensaCPU *),
    DEFINE_PROP_LINK("cpu1", Esp32IntMatrixState, cpu[1], TYPE_XTENSA_CPU, XtensaCPU *),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_intmatrix_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    dc->vmsd = &vmstate_esp32_intmatrix;

    rc->phases.hold = esp32_intmatrix_reset_hold;
    dc->realize = esp32_intmatrix_realize;
    device_class_set_props(dc, esp32_intmatrix_properties);
}

static const TypeInfo esp32_intmatrix_info = {
    .name = TYPE_ESP32_INTMATRIX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32IntMatrixState),
    .instance_init = esp32_intmatrix_init,
    .class_init = esp32_intmatrix_class_init
};

static void esp32_intmatrix_register_types(void)
{
    type_register_static(&esp32_intmatrix_info);
}

type_init(esp32_intmatrix_register_types)
