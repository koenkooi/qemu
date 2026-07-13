/*
 * BeagleBone Black emulation
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
#include "qemu/error-report.h"
#include "hw/boards.h"
#include "hw/arm/am335x_soc.h"
#include "hw/arm/boot.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/misc/led.h"
#include "system/blockdev.h"
#include "exec/address-spaces.h"

/* On-board USR LED descriptions, keyed by GPIO1 line 21..24. These strings
 * are the identifiers a host relay matches to drive the board UI. */
static const char * const bbb_usr_led_desc[4] = {
    "beaglebone-usr0", "beaglebone-usr1",
    "beaglebone-usr2", "beaglebone-usr3",
};

static struct arm_boot_info bbb_binfo = {
    .loader_start = 0x80000000,
    .board_id = -1,
};

static void beaglebone_init(MachineState *machine)
{
    AM335xState *soc;
    int i;

    /* BIOS is not supported by this board */
    if (machine->firmware) {
        error_report("BIOS not supported for this machine");
        exit(1);
    }

    soc = AM335X_SOC(object_new(TYPE_AM335X_SOC));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    object_unref(OBJECT(soc));

    qdev_realize(DEVICE(soc), NULL, &error_fatal);

    /*
     * Attach SD/MMC cards from -sd (first IF_SD drive -> MMC0 -> mmcblk0,
     * second -> MMC1 -> mmcblk1). Which drive backs which controller is a
     * board-level policy, so the wiring lives here rather than in the SoC.
     *
     * MMC0 is the removable microSD, so it gets a TYPE_SD_CARD. MMC1 is the
     * soldered-on eMMC (DT &mmc2: non-removable, 8-bit): give it a TYPE_EMMC
     * so the guest's MMC init path (CMD1 SEND_OP_COND, CMD8 SEND_EXT_CSD)
     * is answered -- an SD card would NAK CMD1 and never enumerate. TYPE_EMMC
     * is user_creatable=false ("soldered on board"), which is why it is wired
     * from board code here rather than via -device.
     */
    for (i = 0; i < AM335X_NUM_MMC; i++) {
        DriveInfo *di = drive_get(IF_SD, 0, i);
        BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
        DeviceState *carddev;

        if (!blk) {
            continue;
        }
        carddev = qdev_new(i == 1 ? TYPE_EMMC : TYPE_SD_CARD);
        qdev_prop_set_drive_err(carddev, "drive", blk, &error_fatal);
        qdev_realize_and_unref(carddev,
                               qdev_get_child_bus(DEVICE(&soc->mmc[i]),
                                                  "sd-bus"),
                               &error_fatal);
    }

    /*
     * On-board LEDs. The four blue USR LEDs (USR0..3) are active-high and
     * driven by GPIO1_21..24 (the am335x-gpio model exports those lines as
     * qemu_irq outputs); the Linux leds-gpio driver blinks them via the
     * heartbeat/mmc/cpu triggers. The green power LED has no GPIO -- it is on
     * whenever the board is powered (active-high resets to on). TYPE_LED
     * emits a led_change_intensity trace event per change, which a host relay
     * forwards to the board UI keyed on these descriptions.
     */
    led_create_simple(OBJECT(machine), GPIO_POLARITY_ACTIVE_HIGH,
                      LED_COLOR_GREEN, "beaglebone-power");
    for (i = 0; i < 4; i++) {
        LEDState *led = led_create_simple(OBJECT(machine),
                                          GPIO_POLARITY_ACTIVE_HIGH,
                                          LED_COLOR_BLUE, bbb_usr_led_desc[i]);
        qdev_connect_gpio_out(DEVICE(&soc->gpio[1]), 21 + i,
                              qdev_get_gpio_in(DEVICE(led), 0));
    }

    memory_region_add_subregion(get_system_memory(), 0x80000000,
                                machine->ram);

    bbb_binfo.ram_size = machine->ram_size;
    arm_load_kernel(&soc->cpu, machine, &bbb_binfo);
}

static void beaglebone_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-a8"),
        NULL
    };

    mc->desc = "TI AM335x BeagleBone Black (Cortex-A8)";
    mc->init = beaglebone_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a8");
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_ram_id = "am335x.ram";
    mc->default_ram_size = 512 * MiB;
    mc->min_cpus = mc->max_cpus = mc->default_cpus = 1;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("beaglebone-black", beaglebone_machine_init)
