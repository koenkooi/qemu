/*
 * TI AM335x GPIO controller ("ti,omap4-gpio") emulation.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * One 32-bit GPIO bank. The single 0x1000 MMIO window carries both the
 * ti-sysc target-module registers (REVISION 0x000, SYSCONFIG 0x010,
 * SYSSTATUS 0x114) that the Linux ti-sysc bus driver pokes when it
 * resumes the module, and the gpio-omap functional registers (IRQ block
 * 0x024-0x048, data/config block 0x130-0x194) in the "omap4" layout.
 *
 * The model implements input sampling with edge/level interrupt
 * detection, output drive via DATAOUT/SET/CLEARDATAOUT (exported on
 * per-line qemu_irq outputs, e.g. for on-board LEDs), and a combined
 * interrupt to the INTC. It is deliberately register-faithful only where
 * the Linux driver cares: nothing in gpio-omap's probe validates or polls
 * a functional register, and SYSSTATUS.RESETDONE reads 1 so ti-sysc's
 * softreset poll completes immediately.
 *
 * A board can assert a static input level (e.g. an active-low card-detect
 * held low) via the "datain-reset" property without wiring a driver to
 * the input line.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "qemu/osdep.h"
#include "hw/gpio/am335x_gpio.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

/* ti-sysc target-module registers (shared window). */
#define GPIO_REVISION           0x000
#define GPIO_SYSCONFIG          0x010
#define GPIO_SYSSTATUS          0x114

/* gpio-omap "omap4" IRQ block. */
#define GPIO_EOI                0x020
#define GPIO_IRQSTATUS_RAW_0    0x024
#define GPIO_IRQSTATUS_RAW_1    0x028
#define GPIO_IRQSTATUS_0        0x02c
#define GPIO_IRQSTATUS_1        0x030
#define GPIO_IRQSTATUS_SET_0    0x034
#define GPIO_IRQSTATUS_SET_1    0x038
#define GPIO_IRQSTATUS_CLR_0    0x03c
#define GPIO_IRQSTATUS_CLR_1    0x040
#define GPIO_IRQWAKEN_0         0x044
#define GPIO_IRQWAKEN_1         0x048

/* gpio-omap "omap4" data/config block. */
#define GPIO_CTRL               0x130
#define GPIO_OE                 0x134
#define GPIO_DATAIN             0x138
#define GPIO_DATAOUT            0x13c
#define GPIO_LEVELDETECT0       0x140
#define GPIO_LEVELDETECT1       0x144
#define GPIO_RISINGDETECT       0x148
#define GPIO_FALLINGDETECT      0x14c
#define GPIO_DEBOUNCENABLE      0x150
#define GPIO_DEBOUNCINGTIME     0x154
#define GPIO_CLEARDATAOUT       0x190
#define GPIO_SETDATAOUT         0x194

/* GPIO_SYSCONFIG.SOFTRESET (ti,sysc-omap2 layout: bit 1). */
#define GPIO_SYSCONFIG_SOFTRESET (1 << 1)

/* GPIO_SYSSTATUS.RESETDONE. */
#define GPIO_SYSSTATUS_RESETDONE (1 << 0)

/* TRM 25.4.1.1 reset value; cosmetic (the driver only prints it). */
#define AM335X_GPIO_REVISION_VALUE 0x50600801

static void am335x_gpio_update_irq(AM335xGpioState *s)
{
    qemu_set_irq(s->irq, (s->irqstatus & s->irqenable) != 0);
}

/* Drive the per-line outputs from DATAOUT for output-configured pins. */
static void am335x_gpio_update_out(AM335xGpioState *s)
{
    int i;

    for (i = 0; i < AM335X_GPIO_NUM_LINES; i++) {
        /* OE bit set = input; output pins reflect DATAOUT, inputs read 0. */
        int level = (s->oe & (1u << i)) ? 0 : !!(s->dataout & (1u << i));
        qemu_set_irq(s->output[i], level);
    }
}

/* External driver of an input line (qemu_irq gpio-in handler). */
static void am335x_gpio_set_input(void *opaque, int line, int level)
{
    AM335xGpioState *s = AM335X_GPIO(opaque);
    uint32_t mask = 1u << line;
    bool old = !!(s->datain & mask);
    bool new = !!level;

    if (old == new) {
        return;
    }
    if (new) {
        s->datain |= mask;
    } else {
        s->datain &= ~mask;
    }

    /* Interrupt detection for this line. */
    if ((new && (s->risingdetect & mask)) ||
        (!new && (s->fallingdetect & mask)) ||
        (new && (s->leveldetect1 & mask)) ||
        (!new && (s->leveldetect0 & mask))) {
        s->irqstatus |= mask;
        am335x_gpio_update_irq(s);
    }
}

static uint64_t am335x_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    AM335xGpioState *s = AM335X_GPIO(opaque);

    switch (offset) {
    case GPIO_REVISION:
        return AM335X_GPIO_REVISION_VALUE;
    case GPIO_SYSCONFIG:
        return s->sysconfig;
    case GPIO_SYSSTATUS:
        /* Always report the OCP softreset as complete. */
        return GPIO_SYSSTATUS_RESETDONE;
    case GPIO_EOI:
        return 0;
    case GPIO_IRQSTATUS_RAW_0:
        return s->irqstatus;
    case GPIO_IRQSTATUS_RAW_1:
        return s->irqstatus2;
    case GPIO_IRQSTATUS_0:
        return s->irqstatus;
    case GPIO_IRQSTATUS_1:
        return s->irqstatus2;
    case GPIO_IRQSTATUS_SET_0:
    case GPIO_IRQSTATUS_CLR_0:
        return s->irqenable;
    case GPIO_IRQSTATUS_SET_1:
    case GPIO_IRQSTATUS_CLR_1:
        return s->irqenable2;
    case GPIO_IRQWAKEN_0:
        return s->irqwaken;
    case GPIO_IRQWAKEN_1:
        return s->irqwaken2;
    case GPIO_CTRL:
        return s->ctrl;
    case GPIO_OE:
        return s->oe;
    case GPIO_DATAIN:
        return s->datain;
    case GPIO_DATAOUT:
        return s->dataout;
    case GPIO_LEVELDETECT0:
        return s->leveldetect0;
    case GPIO_LEVELDETECT1:
        return s->leveldetect1;
    case GPIO_RISINGDETECT:
        return s->risingdetect;
    case GPIO_FALLINGDETECT:
        return s->fallingdetect;
    case GPIO_DEBOUNCENABLE:
        return s->debouncenable;
    case GPIO_DEBOUNCINGTIME:
        return s->debouncingtime;
    case GPIO_CLEARDATAOUT:
    case GPIO_SETDATAOUT:
        return s->dataout;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_GPIO, offset);
        return 0;
    }
}

static void am335x_gpio_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    AM335xGpioState *s = AM335X_GPIO(opaque);
    uint32_t v = (uint32_t)value;

    switch (offset) {
    case GPIO_REVISION:
    case GPIO_SYSSTATUS:
    case GPIO_IRQSTATUS_RAW_1:
    case GPIO_DATAIN:
        /* read-only */
        break;
    case GPIO_SYSCONFIG:
        /* SOFTRESET is self-clearing; SYSSTATUS.RESETDONE always reads 1,
         * so there is no reset-in-progress state to model. */
        s->sysconfig = v & ~GPIO_SYSCONFIG_SOFTRESET;
        break;
    case GPIO_EOI:
        break;
    case GPIO_IRQSTATUS_RAW_0:
        /* Write-1-to-set (diagnostic force). */
        s->irqstatus |= v;
        am335x_gpio_update_irq(s);
        break;
    case GPIO_IRQSTATUS_0:
        /* Write-1-to-clear. */
        s->irqstatus &= ~v;
        am335x_gpio_update_irq(s);
        break;
    case GPIO_IRQSTATUS_1:
        s->irqstatus2 &= ~v;
        break;
    case GPIO_IRQSTATUS_SET_0:
        s->irqenable |= v;
        am335x_gpio_update_irq(s);
        break;
    case GPIO_IRQSTATUS_SET_1:
        s->irqenable2 |= v;
        break;
    case GPIO_IRQSTATUS_CLR_0:
        s->irqenable &= ~v;
        am335x_gpio_update_irq(s);
        break;
    case GPIO_IRQSTATUS_CLR_1:
        s->irqenable2 &= ~v;
        break;
    case GPIO_IRQWAKEN_0:
        s->irqwaken = v;
        break;
    case GPIO_IRQWAKEN_1:
        s->irqwaken2 = v;
        break;
    case GPIO_CTRL:
        s->ctrl = v;
        break;
    case GPIO_OE:
        s->oe = v;
        am335x_gpio_update_out(s);
        break;
    case GPIO_DATAOUT:
        s->dataout = v;
        am335x_gpio_update_out(s);
        break;
    case GPIO_CLEARDATAOUT:
        s->dataout &= ~v;
        am335x_gpio_update_out(s);
        break;
    case GPIO_SETDATAOUT:
        s->dataout |= v;
        am335x_gpio_update_out(s);
        break;
    case GPIO_LEVELDETECT0:
        s->leveldetect0 = v;
        break;
    case GPIO_LEVELDETECT1:
        s->leveldetect1 = v;
        break;
    case GPIO_RISINGDETECT:
        s->risingdetect = v;
        break;
    case GPIO_FALLINGDETECT:
        s->fallingdetect = v;
        break;
    case GPIO_DEBOUNCENABLE:
        s->debouncenable = v;
        break;
    case GPIO_DEBOUNCINGTIME:
        s->debouncingtime = v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_GPIO, offset);
        break;
    }
}

static const MemoryRegionOps am335x_gpio_ops = {
    .read = am335x_gpio_read,
    .write = am335x_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void am335x_gpio_reset(DeviceState *dev)
{
    AM335xGpioState *s = AM335X_GPIO(dev);

    s->sysconfig = 0;
    s->irqstatus = 0;
    s->irqstatus2 = 0;
    s->irqenable = 0;
    s->irqenable2 = 0;
    s->irqwaken = 0;
    s->irqwaken2 = 0;
    s->ctrl = 0;
    s->oe = 0xFFFFFFFF;         /* all lines inputs */
    s->datain = s->datain_reset;
    s->dataout = 0;
    s->leveldetect0 = 0;
    s->leveldetect1 = 0;
    s->risingdetect = 0;
    s->fallingdetect = 0;
    s->debouncenable = 0;
    s->debouncingtime = 0;

    am335x_gpio_update_out(s);
    am335x_gpio_update_irq(s);
}

static void am335x_gpio_init(Object *obj)
{
    AM335xGpioState *s = AM335X_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &am335x_gpio_ops, s,
                          TYPE_AM335X_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    qdev_init_gpio_in(DEVICE(obj), am335x_gpio_set_input,
                      AM335X_GPIO_NUM_LINES);
    qdev_init_gpio_out(DEVICE(obj), s->output, AM335X_GPIO_NUM_LINES);
}

static const VMStateDescription vmstate_am335x_gpio = {
    .name = TYPE_AM335X_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(sysconfig, AM335xGpioState),
        VMSTATE_UINT32(irqstatus, AM335xGpioState),
        VMSTATE_UINT32(irqstatus2, AM335xGpioState),
        VMSTATE_UINT32(irqenable, AM335xGpioState),
        VMSTATE_UINT32(irqenable2, AM335xGpioState),
        VMSTATE_UINT32(irqwaken, AM335xGpioState),
        VMSTATE_UINT32(irqwaken2, AM335xGpioState),
        VMSTATE_UINT32(ctrl, AM335xGpioState),
        VMSTATE_UINT32(oe, AM335xGpioState),
        VMSTATE_UINT32(datain, AM335xGpioState),
        VMSTATE_UINT32(dataout, AM335xGpioState),
        VMSTATE_UINT32(leveldetect0, AM335xGpioState),
        VMSTATE_UINT32(leveldetect1, AM335xGpioState),
        VMSTATE_UINT32(risingdetect, AM335xGpioState),
        VMSTATE_UINT32(fallingdetect, AM335xGpioState),
        VMSTATE_UINT32(debouncenable, AM335xGpioState),
        VMSTATE_UINT32(debouncingtime, AM335xGpioState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property am335x_gpio_properties[] = {
    DEFINE_PROP_UINT32("datain-reset", AM335xGpioState, datain_reset, 0),
};

static void am335x_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, am335x_gpio_reset);
    dc->vmsd = &vmstate_am335x_gpio;
    device_class_set_props(dc, am335x_gpio_properties);
    dc->desc = "TI AM335x GPIO controller";
}

static const TypeInfo am335x_gpio_info = {
    .name          = TYPE_AM335X_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xGpioState),
    .instance_init = am335x_gpio_init,
    .class_init    = am335x_gpio_class_init,
};

static void am335x_gpio_register_types(void)
{
    type_register_static(&am335x_gpio_info);
}

type_init(am335x_gpio_register_types)
