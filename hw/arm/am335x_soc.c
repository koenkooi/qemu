/*
 * TI AM335x SoC emulation (M1 skeleton)
 *
 * Copyright (C) 2026
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/arm/am335x_soc.h"
#include "hw/sysbus.h"
#include "hw/char/serial-mm.h"
#include "hw/misc/unimp.h"
#include "system/system.h"
#include "exec/address-spaces.h"
#include "target/arm/cpu-qom.h"

/* On-chip memory (internal SRAM) */
#define AM335X_OCMC_BASE        0x40300000
#define AM335X_OCMC_SIZE        (64 * KiB)

/* UART0 (16550-compatible, 4-byte register spacing => regshift 2) */
#define AM335X_UART0_BASE       0x44E09000
#define AM335X_UART0_CLK        48000000

/* INTC input line numbers (TRM spruh73q ch.6) */
#define AM335X_IRQ_UART0        72

/* DMTIMER0..3 MMIO bases and INTC input lines (TRM spruh73q ch.6/20) */
static const struct {
    hwaddr addr;
    unsigned int irq;
} am335x_timer_table[AM335X_NUM_TIMERS] = {
    { 0x44E05000, 66 }, /* DMTIMER0 */
    { 0x44E31000, 67 }, /* DMTIMER1 (1ms, not modelled specially here) */
    { 0x48040000, 68 }, /* DMTIMER2 */
    { 0x48042000, 69 }, /* DMTIMER3 */
};

static void am335x_soc_init(Object *obj)
{
    AM335xState *s = AM335X_SOC(obj);
    int i;

    object_initialize_child(obj, "cpu", &s->cpu,
                            ARM_CPU_TYPE_NAME("cortex-a8"));
    object_initialize_child(obj, "intc", &s->intc, TYPE_AM335X_INTC);
    for (i = 0; i < AM335X_NUM_TIMERS; i++) {
        object_initialize_child(obj, "timer[*]", &s->timer[i],
                                TYPE_AM335X_TIMER);
    }
    object_initialize_child(obj, "prcm", &s->prcm, TYPE_AM335X_PRCM);
}

static void am335x_soc_realize(DeviceState *dev, Error **errp)
{
    AM335xState *s = AM335X_SOC(dev);
    int i;

    if (!qdev_realize(DEVICE(&s->cpu), NULL, errp)) {
        return;
    }

    /* On-chip SRAM (OCMC RAM) */
    memory_region_init_ram(&s->ocmc, OBJECT(dev), "am335x.ocmc",
                           AM335X_OCMC_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(), AM335X_OCMC_BASE,
                                &s->ocmc);

    /* MPU interrupt controller (INTCPS) @ 0x48200000, out 0 = IRQ, 1 = FIQ. */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->intc), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->intc), 0, 0x48200000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->intc), 0,
                       qdev_get_gpio_in(DEVICE(&s->cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->intc), 1,
                       qdev_get_gpio_in(DEVICE(&s->cpu), ARM_CPU_FIQ));
    /* Re-export the 128 INTC input lines on the SoC container. */
    qdev_pass_gpios(DEVICE(&s->intc), dev, NULL);

    /*
     * UART0 (16550-compatible), IRQ line 72 on the INTC.
     * FIXME use a qdev chardev prop instead of serial_hd()
     */
    serial_mm_init(get_system_memory(), AM335X_UART0_BASE, 2,
                   qdev_get_gpio_in(dev, AM335X_IRQ_UART0),
                   AM335X_UART0_CLK / 16, serial_hd(0),
                   DEVICE_LITTLE_ENDIAN);

    /* DMTIMER0..3: real devices, needed for the kernel clockevent/
     * clocksource to make progress past time init. */
    for (i = 0; i < AM335X_NUM_TIMERS; i++) {
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->timer[i]), 0,
                        am335x_timer_table[i].addr);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->timer[i]), 0,
                           qdev_get_gpio_in(dev, am335x_timer_table[i].irq));
    }

    /*
     * Clock Module / PRCM @ 0x44E00000. Needed so ti-sysc can enable
     * module functional clocks (CLKCTRL IDLEST) and DPLLs report locked,
     * which unblocks the DMTIMER probes above.
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->prcm), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->prcm), 0, 0x44E00000);

    /*
     * Placeholders for peripherals that become real devices in later
     * milestones. Mapping them as unimplemented devices means stray guest
     * MMIO is logged instead of aborting the machine.
     */
    create_unimplemented_device("gpio0",           0x44E07000, 0x1000);
    create_unimplemented_device("i2c0",            0x44E0B000, 0x1000);
    create_unimplemented_device("l4_wkup-control", 0x44E10000, 0x20000);
    create_unimplemented_device("wdt1",            0x44E35000, 0x1000);
    create_unimplemented_device("mmc0",            0x48060000, 0x1000);
    create_unimplemented_device("mmc1",            0x481D8000, 0x10000);
    create_unimplemented_device("cpsw",            0x4A100000, 0x8000);
}

static void am335x_soc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = am335x_soc_realize;
    /* Reason: Uses serial_hd() in realize function */
    dc->user_creatable = false;
}

static const TypeInfo am335x_soc_type_info = {
    .name = TYPE_AM335X_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(AM335xState),
    .instance_init = am335x_soc_init,
    .class_init = am335x_soc_class_init,
};

static void am335x_soc_register_types(void)
{
    type_register_static(&am335x_soc_type_info);
}

type_init(am335x_soc_register_types)
