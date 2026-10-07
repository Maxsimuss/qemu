/*
 * Electrical I2C memory peer for ESP32 board fixtures.
 * A 256-byte, 7/10-bit addressed memory with one-byte address pointer. This is
 * an explicit test component, not an ESP32 I2C transaction shortcut. It sees
 * resolved physical pads and drives only open-drain external SDA/SCL.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"
#include "hw/irq.h"
#include "qapi/error.h"
#include "migration/vmstate.h"

#define TYPE_ESP32_I2C_PEER "esp32-i2c-peer"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32I2CPeer, ESP32_I2C_PEER)

enum PeerPhase {
    PEER_IDLE, PEER_ADDRESS, PEER_ADDRESS10, PEER_WRITE, PEER_READ,
    PEER_ACK_SETUP, PEER_ACK_CLOCK, PEER_ACK_END,
    PEER_READ_ACK_SETUP, PEER_READ_ACK, PEER_READ_ACK_END, PEER_IGNORE,
};
struct Esp32I2CPeer {
    DeviceState parent;
    Esp32GpioState *gpio;
    uint32_t sda_pad, scl_pad, driver_slot, address;
    uint64_t stretch_ns;
    int32_t nack_after;
    bool nack_address, addr_10bit, ten_selected;
    QEMUTimer *stretch_timer;
    qemu_irq sda_sink, scl_sink;
    uint8_t memory[256];
    uint8_t pointer, phase, next, bits, shift;
    uint32_t written;
    bool sda, scl, ack, expect_pointer;
    bool sda_low, scl_low;
};

static void peer_drive(Esp32I2CPeer *s, bool clock, bool low)
{
    if (clock) {
        s->scl_low = low;
    } else {
        s->sda_low = low;
    }
    esp32_gpio_set_external_drive(s->gpio, clock ? s->scl_pad : s->sda_pad,
                                  s->driver_slot,
                                  low ? ESP32_PAD_LOW : ESP32_PAD_Z);
}

static void stretch_release(void *opaque)
{
    peer_drive(opaque, true, false);
}

static void peer_load(Esp32I2CPeer *s)
{
    s->shift = s->memory[s->pointer++];
    s->bits = 0;
    s->phase = PEER_READ;
    peer_drive(s, false, !(s->shift & 0x80));
}

static void peer_rise(Esp32I2CPeer *s)
{
    switch (s->phase) {
    case PEER_ADDRESS:
    case PEER_WRITE:
        s->shift = (s->shift << 1) | s->sda;
        if (++s->bits == 8) {
            if (s->phase == PEER_ADDRESS) {
                if (s->addr_10bit) {
                    uint8_t prefix = 0xf0 | ((s->address >> 7) & 6);
                    bool read = s->shift & 1;
                    s->ack = (s->shift & 0xfe) == prefix &&
                             (!read || s->ten_selected) && !s->nack_address;
                    s->next = read ? PEER_READ : PEER_ADDRESS10;
                    if (!read) {
                        s->ten_selected = false;
                    }
                } else {
                    s->ack = (s->shift >> 1) == s->address && !s->nack_address;
                    s->next = s->shift & 1 ? PEER_READ : PEER_WRITE;
                }
                s->expect_pointer = !(s->shift & 1);
                s->written = 0;
            } else {
                s->ack = s->nack_after < 0 || s->written < s->nack_after;
                if (s->ack) {
                    if (s->expect_pointer) {
                        s->pointer = s->shift;
                        s->expect_pointer = false;
                    } else {
                        s->memory[s->pointer++] = s->shift;
                    }
                    s->written++;
                }
                s->next = PEER_WRITE;
            }
            if (!s->ack) {
                s->next = PEER_IGNORE;
            }
            s->phase = PEER_ACK_SETUP;
        }
        break;
    case PEER_ADDRESS10:
        s->shift = (s->shift << 1) | s->sda;
        if (++s->bits == 8) {
            s->ack = s->shift == (s->address & 0xff);
            s->ten_selected = s->ack;
            s->next = s->ack ? PEER_WRITE : PEER_IGNORE;
            s->phase = PEER_ACK_SETUP;
        }
        break;
    case PEER_ACK_CLOCK:
        s->phase = PEER_ACK_END;
        break;
    case PEER_READ:
        if (++s->bits == 8) {
            s->phase = PEER_READ_ACK_SETUP;
        }
        break;
    case PEER_READ_ACK:
        s->phase = s->sda ? PEER_IGNORE : PEER_READ_ACK_END;
        break;
    default:
        break;
    }
}

static void peer_fall(Esp32I2CPeer *s)
{
    switch (s->phase) {
    case PEER_ACK_SETUP:
        s->phase = PEER_ACK_CLOCK;
        peer_drive(s, false, s->ack);
        if (s->ack && s->stretch_ns) {
            peer_drive(s, true, true);
            timer_mod(s->stretch_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->stretch_ns);
        }
        break;
    case PEER_ACK_END:
        s->phase = s->next;
        s->bits = s->shift = 0;
        peer_drive(s, false, false);
        if (s->phase == PEER_READ) {
            peer_load(s);
        }
        break;
    case PEER_READ:
        peer_drive(s, false, !(s->shift & (0x80 >> s->bits)));
        break;
    case PEER_READ_ACK_SETUP:
        s->phase = PEER_READ_ACK;
        peer_drive(s, false, false);
        break;
    case PEER_READ_ACK_END:
        peer_load(s);
        break;
    default:
        break;
    }
}

static void peer_pad(void *opaque, int clock, int value)
{
    Esp32I2CPeer *s = opaque;
    bool level;
    /* A floating or contended bus cannot be interpreted as a valid bit. */
    if (value != ESP32_PAD_LOW && value != ESP32_PAD_HIGH) {
        s->phase = PEER_IGNORE;
        return;
    }
    level = value == ESP32_PAD_HIGH;
    if (clock) {
        bool previous = s->scl;
        s->scl = level;
        if (level != previous) {
            if (level) {
                peer_rise(s);
            } else {
                peer_fall(s);
            }
        }
    } else {
        bool previous = s->sda;
        s->sda = level;
        if (previous != level && s->scl) {
            s->phase = level ? PEER_IDLE : PEER_ADDRESS;
            if (level) {
                s->ten_selected = false;
            }
            s->bits = s->shift = 0;
            peer_drive(s, false, false);
        }
    }
}

static void peer_reset(DeviceState *dev)
{
    Esp32I2CPeer *s = ESP32_I2C_PEER(dev);
    timer_del(s->stretch_timer);
    s->phase = PEER_IDLE;
    s->bits = s->shift = s->pointer = s->next = 0;
    s->written = 0;
    s->ack = s->expect_pointer = s->ten_selected = false;
    /* External memory survives controller/system reset. */
    peer_drive(s, false, false);
    peer_drive(s, true, false);
}

static void peer_realize(DeviceState *dev, Error **errp)
{
    Esp32I2CPeer *s = ESP32_I2C_PEER(dev);
    if (!s->gpio || s->sda_pad >= ESP32_GPIO_PADS ||
        s->scl_pad >= ESP32_GPIO_PADS || s->sda_pad == s->scl_pad ||
        s->driver_slot >= ESP32_GPIO_EXT_DRIVERS ||
        s->address > (s->addr_10bit ? 0x3ff : 0x7f)) {
        error_setg(errp, "esp32-i2c-peer needs valid gpio, sda, scl, driver-slot and legal 7/10-bit address");
        return;
    }
    s->stretch_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, stretch_release, s);
    s->sda_sink = qemu_allocate_irq(peer_pad, s, 0);
    s->scl_sink = qemu_allocate_irq(peer_pad, s, 1);
    s->sda = esp32_gpio_get_pad(s->gpio, s->sda_pad) == ESP32_PAD_HIGH;
    s->scl = esp32_gpio_get_pad(s->gpio, s->scl_pad) == ESP32_PAD_HIGH;
    esp32_gpio_add_pad_listener(s->gpio, s->sda_pad, s->sda_sink);
    esp32_gpio_add_pad_listener(s->gpio, s->scl_pad, s->scl_sink);
    peer_reset(dev);
}

static void peer_unrealize(DeviceState *dev)
{
    Esp32I2CPeer *s = ESP32_I2C_PEER(dev);
    esp32_gpio_remove_pad_listener(s->gpio, s->sda_pad, s->sda_sink);
    esp32_gpio_remove_pad_listener(s->gpio, s->scl_pad, s->scl_sink);
    peer_drive(s, false, false);
    peer_drive(s, true, false);
    timer_free(s->stretch_timer);
    qemu_free_irq(s->sda_sink);
    qemu_free_irq(s->scl_sink);
}

static int peer_post_load(void *opaque, int version)
{
    Esp32I2CPeer *s = opaque;
    if (s->phase > PEER_IGNORE || s->bits > 8) {
        return -EINVAL;
    }
    peer_drive(s, false, s->sda_low);
    peer_drive(s, true, s->scl_low);
    return 0;
}

static const VMStateDescription peer_vmstate = {
    .name = TYPE_ESP32_I2C_PEER, .version_id = 1, .minimum_version_id = 1,
    .post_load = peer_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(memory, Esp32I2CPeer, 256),
        VMSTATE_UINT8(pointer, Esp32I2CPeer), VMSTATE_UINT8(phase, Esp32I2CPeer),
        VMSTATE_UINT8(next, Esp32I2CPeer), VMSTATE_UINT8(bits, Esp32I2CPeer),
        VMSTATE_UINT8(shift, Esp32I2CPeer), VMSTATE_UINT32(written, Esp32I2CPeer),
        VMSTATE_BOOL(sda, Esp32I2CPeer), VMSTATE_BOOL(scl, Esp32I2CPeer),
        VMSTATE_BOOL(ack, Esp32I2CPeer), VMSTATE_BOOL(expect_pointer, Esp32I2CPeer),
        VMSTATE_BOOL(sda_low, Esp32I2CPeer), VMSTATE_BOOL(scl_low, Esp32I2CPeer),
        VMSTATE_BOOL(ten_selected, Esp32I2CPeer),
        VMSTATE_TIMER_PTR(stretch_timer, Esp32I2CPeer), VMSTATE_END_OF_LIST()
    },
};
static const Property peer_properties[] = {
    DEFINE_PROP_LINK("gpio", Esp32I2CPeer, gpio, TYPE_ESP32_GPIO, Esp32GpioState *),
    DEFINE_PROP_UINT32("sda", Esp32I2CPeer, sda_pad, 21),
    DEFINE_PROP_UINT32("scl", Esp32I2CPeer, scl_pad, 22),
    DEFINE_PROP_UINT32("driver-slot", Esp32I2CPeer, driver_slot, 1),
    DEFINE_PROP_UINT32("address", Esp32I2CPeer, address, 0x50),
    DEFINE_PROP_UINT64("stretch-ns", Esp32I2CPeer, stretch_ns, 0),
    DEFINE_PROP_INT32("nack-after", Esp32I2CPeer, nack_after, -1),
    DEFINE_PROP_BOOL("nack-address", Esp32I2CPeer, nack_address, false),
    DEFINE_PROP_BOOL("addr-10bit", Esp32I2CPeer, addr_10bit, false),
    DEFINE_PROP_END_OF_LIST(),
};
static void peer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = peer_realize;
    dc->unrealize = peer_unrealize;
    device_class_set_legacy_reset(dc, peer_reset);
    dc->vmsd = &peer_vmstate;
    dc->desc = "Open-drain I2C memory peer on resolved ESP32 physical pads";
    device_class_set_props(dc, peer_properties);
}
static const TypeInfo peer_type = {
    .name = TYPE_ESP32_I2C_PEER, .parent = TYPE_DEVICE,
    .instance_size = sizeof(Esp32I2CPeer), .class_init = peer_class_init,
};
static void peer_register(void)
{
    type_register_static(&peer_type);
}
type_init(peer_register)
