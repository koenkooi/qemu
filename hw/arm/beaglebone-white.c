/*
 * BeagleBone (original, "White") emulation
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
 *
 * The original BeagleBone ("White", 2011, github.com/beagleboard/beaglebone)
 * is the same TI AM335x SoC as the BeagleBone Black on beaglebone.c, wired up
 * as a different board. Confirmed from the mainline kernel DT sources
 * (arch/arm/boot/dts/ti/omap/am335x-bone-common.dtsi, the base shared by both
 * boards, layered under am335x-boneblack-common.dtsi for Black) and the
 * U-Boot board-detect code (board/ti/am335x/board.h, board/ti/am335x/board.c):
 *
 *  - No onboard eMMC: am335x-boneblack-common.dtsi is the *only* place that
 *    enables &mmc2 (status = "okay", bus-width 8, non-removable) -- the SoC's
 *    second MMC/SD host controller exists in silicon on White too (it is part
 *    of TYPE_AM335X_SOC regardless of board), but White's DT leaves it
 *    "disabled" and nothing populates it. We mirror that here: only MMC0
 *    (microSD) is ever attached to a card; MMC1 is left with no card, exactly
 *    like the shared SoC's unconnected second controller on real White
 *    hardware.
 *  - No onboard HDMI: the TDA19988 encoder + LCDC pixel timings only appear
 *    via am335x-boneblack-hdmi.dtsi, which only am335x-boneblack.dts includes.
 *    White's DTB (am335x-bone.dtb) leaves &lcdc "disabled" and has no I2C
 *    tda19988 node at all -- White has 2x46 pin expansion headers instead of
 *    onboard HDMI (not modelled here; nothing is connected to them in DT
 *    either).
 *  - Everything else White's DT enables is identical to Black and already
 *    shared via TYPE_AM335X_SOC + the same board-level wiring pattern: the
 *    TPS65217C PMIC and board-ID EEPROM on I2C0, and the USR0-3 LEDs on
 *    GPIO1_21..24 (am335x-bone-common.dtsi leds{} node: gpios = <&gpio1 21..
 *    24 GPIO_ACTIVE_HIGH>, same offsets and polarity as Black) plus the fixed
 *    power LED.
 *  - Board-ID EEPROM "name" field: U-Boot's board/ti/am335x/board.h defines
 *    board_is_bone() as board_ti_is("A335BONE"), distinct from Black's
 *    board_is_bone_lt() == board_ti_is("A335BNLT"). include/configs/
 *    am335x_evm.h's "findfdt" env script confirms the pairing: board_name ==
 *    "A335BONE" selects fdtfile am335x-bone.dtb, matching the DTB used here.
 *    The header's version/serial fields are cosmetic (only U-Boot's SPL board
 *    detection reads them; Linux itself is NAK-tolerant per the same comment
 *    in beaglebone.c) and are not derivable from source, so they are left as
 *    unset/placeholder bytes rather than guessed.
 *  - DT default RAM is 256 MB (am335x-bone-common.dtsi memory@80000000) vs.
 *    Black's 512 MB override in am335x-boneblack-common.dtsi; used as this
 *    machine's default (still overridable with -m, like any QEMU machine).
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
#include "hw/i2c/i2c.h"
#include "hw/i2c/am335x_i2c.h"
#include "hw/misc/tps65217.h"
#include "hw/nvram/eeprom_at24c.h"
#include "system/blockdev.h"
#include "exec/address-spaces.h"

/* On-board USR LED descriptions, keyed by GPIO1 line 21..24 -- identical
 * wiring to beaglebone.c's Black variant (same DT offsets, see file comment
 * above), reused verbatim so the existing host LED relay/viewer just works. */
static const char * const bbw_usr_led_desc[4] = {
    "beaglebone-usr0", "beaglebone-usr1",
    "beaglebone-usr2", "beaglebone-usr3",
};

static struct arm_boot_info bbw_binfo = {
    .loader_start = 0x80000000,
    .board_id = -1,
};

static void beaglebone_white_init(MachineState *machine)
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
     * Attach an SD card from -sd to MMC0 (-> mmcblk0), the removable
     * microSD slot. Unlike beaglebone.c's Black variant, White has no
     * onboard eMMC: MMC1 exists in the shared SoC container but is left
     * unconnected here, matching real hardware (see file comment above). A
     * second -sd drive (index 1) is simply not attached to anything.
     */
    {
        DriveInfo *di = drive_get(IF_SD, 0, 0);
        BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;

        if (blk) {
            DeviceState *carddev = qdev_new(TYPE_SD_CARD);

            qdev_prop_set_drive_err(carddev, "drive", blk, &error_fatal);
            qdev_realize_and_unref(carddev,
                                   qdev_get_child_bus(DEVICE(&soc->mmc[0]),
                                                      "sd-bus"),
                                   &error_fatal);
        }
    }

    /*
     * On-board LEDs -- same GPIO1_21..24 USR0-3 wiring and fixed-on green
     * power LED as beaglebone.c's Black variant; see that file for the
     * trace/host-relay notes.
     */
    led_create_simple(OBJECT(machine), GPIO_POLARITY_ACTIVE_HIGH,
                      LED_COLOR_GREEN, "beaglebone-power");
    for (i = 0; i < 4; i++) {
        LEDState *led = led_create_simple(OBJECT(machine),
                                          GPIO_POLARITY_ACTIVE_HIGH,
                                          LED_COLOR_BLUE, bbw_usr_led_desc[i]);
        qdev_connect_gpio_out(DEVICE(&soc->gpio[1]), 21 + i,
                              qdev_get_gpio_in(DEVICE(led), 0));
    }

    /*
     * On-board I2C0 slaves: board-ID EEPROM + TPS65217C PMIC, same as Black.
     * No TDA19988 HDMI encoder here -- White has no onboard HDMI (see file
     * comment above), so &lcdc stays disabled and I2C0 has no 0x70/0x34
     * slaves.
     *
     * The board-ID EEPROM is an atmel,24c32 (4KB) carrying the BeagleBone
     * SRM header (magic 0xAA5533EE, board "A335BONE" -- verified from
     * U-Boot's board_is_bone(), see file comment above). Only the name field
     * is populated; version/serial are left 0xFF (unwritten), like a real
     * mostly-blank EEPROM outside the fields U-Boot actually checks.
     */
    {
        I2CBus *i2c0 = am335x_i2c_bus(DEVICE(&soc->i2c[0]));
        uint8_t board_id_eeprom[4096];

        memset(board_id_eeprom, 0xFF, sizeof(board_id_eeprom));
        memcpy(board_id_eeprom, "\xAA\x55\x33\xEE" "A335BONE", 4 + 8);
        at24c_eeprom_init_rom(i2c0, 0x50, sizeof(board_id_eeprom),
                              board_id_eeprom, sizeof(board_id_eeprom));

        i2c_slave_create_simple(i2c0, TYPE_TPS65217_PMU, 0x24);
    }

    memory_region_add_subregion(get_system_memory(), 0x80000000,
                                machine->ram);

    bbw_binfo.ram_size = machine->ram_size;
    arm_load_kernel(&soc->cpu, machine, &bbw_binfo);
}

static void beaglebone_white_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-a8"),
        NULL
    };

    mc->desc = "TI AM335x BeagleBone (Cortex-A8)";
    mc->init = beaglebone_white_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a8");
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_ram_id = "am335x.ram";
    mc->default_ram_size = 256 * MiB;
    mc->min_cpus = mc->max_cpus = mc->default_cpus = 1;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("beaglebone", beaglebone_white_machine_init)
