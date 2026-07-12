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

static void am335x_soc_init(Object *obj)
{
    AM335xState *s = AM335X_SOC(obj);

    object_initialize_child(obj, "cpu", &s->cpu,
                            ARM_CPU_TYPE_NAME("cortex-a8"));
}

static void am335x_soc_realize(DeviceState *dev, Error **errp)
{
    AM335xState *s = AM335X_SOC(dev);

    if (!qdev_realize(DEVICE(&s->cpu), NULL, errp)) {
        return;
    }

    /* On-chip SRAM (OCMC RAM) */
    memory_region_init_ram(&s->ocmc, OBJECT(dev), "am335x.ocmc",
                           AM335X_OCMC_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(), AM335X_OCMC_BASE,
                                &s->ocmc);

    /*
     * UART0. No interrupt controller is modelled yet in M1, so pass a NULL
     * qemu_irq; qemu_set_irq(NULL, ...) is a documented no-op.
     *
     * FIXME use a qdev chardev prop instead of serial_hd()
     */
    serial_mm_init(get_system_memory(), AM335X_UART0_BASE, 2,
                   NULL, AM335X_UART0_CLK / 16, serial_hd(0),
                   DEVICE_LITTLE_ENDIAN);

    /*
     * Placeholders for peripherals that become real devices in later
     * milestones. Mapping them as unimplemented devices means stray guest
     * MMIO is logged instead of aborting the machine.
     */
    create_unimplemented_device("l4_wkup-prcm",    0x44E00000, 0x2000);
    create_unimplemented_device("dmtimer0",        0x44E05000, 0x1000);
    create_unimplemented_device("gpio0",           0x44E07000, 0x1000);
    create_unimplemented_device("i2c0",            0x44E0B000, 0x1000);
    create_unimplemented_device("l4_wkup-control", 0x44E10000, 0x20000);
    create_unimplemented_device("dmtimer1",        0x44E31000, 0x1000);
    create_unimplemented_device("wdt1",            0x44E35000, 0x1000);
    create_unimplemented_device("dmtimer2",        0x48040000, 0x1000);
    create_unimplemented_device("mmc0",            0x48060000, 0x1000);
    create_unimplemented_device("intc",            0x48200000, 0x1000);
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
