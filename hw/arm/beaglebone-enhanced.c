/*
 * SanCloud BeagleBone Enhanced (BBE) emulation
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
 * The SanCloud BeagleBone Enhanced (github.com/SanCloudLtd/BeagleBoneEnhanced,
 * github.com/beagleboard/BeagleBoneEnhanced) is the same TI AM335x SoC as the
 * BeagleBone Black on beaglebone.c, carrying Black's onboard eMMC and HDMI
 * forward unchanged and adding a Gigabit-capable Ethernet PHY, a USB hub and
 * two I2C sensors. Confirmed from the mainline kernel DT sources
 * (arch/arm/boot/dts/ti/omap/) and U-Boot's board-detect code
 * (board/ti/am335x/board.h, board/ti/am335x/board.c, board/ti/am335x/mux.c):
 *
 *  - Onboard eMMC and HDMI, both present, unchanged from Black: the plain
 *    "am335x-sancloud-bbe.dts" (the DTB this machine targets, as opposed to
 *    the "-lite"/"-extended-wifi" siblings) includes am335x-bone-common.dtsi,
 *    am335x-boneblack-common.dtsi (the &mmc2 8-bit non-removable eMMC) and
 *    am335x-boneblack-hdmi.dtsi (the TDA19988 HDMI encoder) exactly like
 *    am335x-boneblack.dts does. So, unlike White, this machine keeps both the
 *    eMMC on MMC1 and the TDA19988 wired up identically to beaglebone.c.
 *  - Gigabit-capable Ethernet PHY: am335x-sancloud-bbe-common.dtsi overrides
 *    the shared &am33xx_pinmux "cpsw_default"/"cpsw_sleep" pin groups to
 *    MUX_MODE2 (RGMII1) in place of am335x-bone-common.dtsi's MUX_MODE0 (MII)
 *    and sets "&cpsw_port1 { phy-mode = "rgmii-id"; }". U-Boot's
 *    board/ti/am335x/mux.c enable_board_pin_mux() independently confirms this:
 *    board_is_bben() selects rgmii1_pin_mux instead of Black's mii1_pin_mux.
 *    This is the same on-chip CPSW/MDIO silicon as Black (TYPE_AM335X_SOC,
 *    unchanged) talking to a different *external* RGMII PHY. We model that by
 *    setting the CPSW's "gigabit-phy" qdev property, which swaps the shared
 *    hw/net/am335x_cpsw.c model's default 10/100 lan9118_phy for a TI DP83867
 *    Gigabit RGMII PHY that reports a 1000Mbit/full link. NOTE: Sancloud's own
 *    DT does not name the PHY chip (it relies on Linux phylib's MDIO ID-register
 *    autoprobe), so the specific chip is not directly confirmed from Sancloud's
 *    BOM/schematic here; the DP83867 is inferred by close analogy to the
 *    BeagleBone Green Eco (am335x-bonegreen-eco.dts), whose identical RGMII
 *    setup *does* name "ti,dp83867" explicitly on the same CPSW/MDIO silicon,
 *    and which TI's AM335x reference designs commonly pair with -- the closest
 *    concrete evidence available.
 *  - USB hub + sensors, not modelled: am335x-sancloud-bbe-common.dtsi adds a
 *    "usb2512b@2c" USB hub on I2C0, and am335x-sancloud-bbe.dts (the board
 *    file itself, not the shared -common.dtsi) adds "lps331ap@5c" (barometer)
 *    and "mpu6050@68" (accelerometer/gyro), also on I2C0. None of these three
 *    chips has an existing QEMU device model; adding them would mean writing
 *    genuinely new peripherals (the same category of work explicitly deferred
 *    for BeagleBone Green Wireless's SDIO chip), so they are left unpopulated
 *    here -- I2C0 simply NAKs those three addresses, the same as a missing
 *    driver probe on real but unpopulated hardware.
 *  - Board-ID EEPROM: U-Boot's board_is_bben() (board/ti/am335x/board.h) is
 *    "board_is_bone_lt() && !strncmp(board_ti_get_rev(), "SE", 2)" -- i.e.
 *    Enhanced reuses Black's EEPROM "name" field ("A335BNLT") and is told
 *    apart only by a "version" field starting "SE". board.c's
 *    board_fit_config_name_match() and board_late_init() further switch on
 *    board_ti_get_config()[1] (byte 1 of the 32-byte "config" field): 'L'
 *    picks the Lite DTB/env name, 'I' picks Extended-WiFi, and anything else
 *    -- this machine's case, matching the plain "am335x-sancloud-bbe" DTB --
 *    resolves to "BBEN"/am335x-sancloud-bbe. We set version = "SE" + two
 *    0xFF placeholder bytes (only the 2-byte "SE" prefix is ever compared)
 *    and leave the config field entirely 0xFF, following the same
 *    leave-unchecked-fields-unwritten precedent as beaglebone-white.c.
 *    board_is_bben() support was added upstream in commit ad6054f1fe1
 *    ("Add Beaglebone Enhanced support", board/ti/am335x/{board.c,board.h,
 *    mux.c} + include/configs/am335x_evm.h), present in the v2026.01 tag.
 *  - RAM: that same commit's own description lists "1GiB DDR3 RAM" as one of
 *    Enhanced's differences from Black (512MB); the mainline DTB doesn't
 *    override am335x-boneblack-common.dtsi's memory@80000000 node, so this is
 *    a DT/hardware gap upstream, not a modelling choice here. We follow the
 *    real hardware spec (1GiB default, still overridable with -m), same as
 *    how beaglebone-white.c already follows White's real 256MB DT default
 *    rather than Black's.
 *  - LEDs and everything else this machine's DT enables are identical to
 *    Black's and already shared via TYPE_AM335X_SOC plus the same
 *    board-level wiring pattern: the TPS65217C PMIC on I2C0, and the USR0-3
 *    LEDs on GPIO1_21..24 (am335x-bone-common.dtsi leds{} node, common to
 *    every board in the BeagleBone family) plus the fixed power LED.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/boards.h"
#include "hw/arm/am335x_soc.h"
#include "hw/arm/am335x_bootflow.h"
#include "hw/arm/boot.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/misc/led.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/am335x_i2c.h"
#include "hw/misc/tps65217.h"
#include "hw/display/tda19988.h"
#include "hw/nvram/eeprom_at24c.h"
#include "system/blockdev.h"
#include "exec/address-spaces.h"

/* On-board USR LED descriptions, keyed by GPIO1 line 21..24 -- identical
 * wiring to beaglebone.c's Black variant (same DT offsets, see file comment
 * above), reused verbatim so the existing host LED relay/viewer just works. */
static const char * const bbe_usr_led_desc[4] = {
    "beaglebone-usr0", "beaglebone-usr1",
    "beaglebone-usr2", "beaglebone-usr3",
};

static struct arm_boot_info bbe_binfo = {
    .loader_start = 0x80000000,
    .board_id = -1,
};

static void beaglebone_enhanced_init(MachineState *machine)
{
    AM335xState *soc;
    int i;

    soc = AM335X_SOC(object_new(TYPE_AM335X_SOC));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    object_unref(OBJECT(soc));

    /*
     * Enhanced muxes the CPSW to RGMII1 with an external Gigabit PHY (see
     * file comment above), unlike Black/White's 10/100 MII PHY. Select the
     * CPSW model's Gigabit DP83867 variant before the SoC (and its CPSW
     * child) is realized. The child object already exists here -- the SoC's
     * instance_init created it -- so the board sets the property on it
     * directly, the same way it reaches into soc->mmc/soc->gpio/soc->i2c
     * below.
     */
    object_property_set_bool(OBJECT(&soc->cpsw), "gigabit-phy", true,
                             &error_fatal);

    qdev_realize(DEVICE(soc), NULL, &error_fatal);

    /*
     * Attach SD/MMC cards from -sd (first IF_SD drive -> MMC0 -> mmcblk0,
     * second -> MMC1 -> mmcblk1), exactly like beaglebone.c's Black variant:
     * Enhanced keeps Black's onboard eMMC (see file comment above), so MMC1
     * gets a TYPE_EMMC rather than being left unconnected like White's.
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
     * On-board LEDs -- same GPIO1_21..24 USR0-3 wiring and fixed-on green
     * power LED as beaglebone.c's Black variant; see that file for the
     * trace/host-relay notes.
     */
    led_create_simple(OBJECT(machine), GPIO_POLARITY_ACTIVE_HIGH,
                      LED_COLOR_GREEN, "beaglebone-power");
    for (i = 0; i < 4; i++) {
        LEDState *led = led_create_simple(OBJECT(machine),
                                          GPIO_POLARITY_ACTIVE_HIGH,
                                          LED_COLOR_BLUE, bbe_usr_led_desc[i]);
        qdev_connect_gpio_out(DEVICE(&soc->gpio[1]), 21 + i,
                              qdev_get_gpio_in(DEVICE(led), 0));
    }

    /*
     * On-board I2C0 slaves: board-ID EEPROM + TPS65217C PMIC + TDA19988 HDMI
     * encoder, same as Black (see file comment above for what's deliberately
     * left out: the USB hub and the two sensors, none of which have an
     * existing QEMU device model).
     *
     * The board-ID EEPROM is an atmel,24c32 (4KB) carrying the BeagleBone
     * SRM header (magic 0xAA5533EE, board "A335BNLT" like Black, version
     * "SE\xFF\xFF" so U-Boot's board_is_bben() -- which only checks the
     * "SE" prefix -- identifies this as Enhanced rather than Black; see file
     * comment above). Only those fields are populated; everything else
     * (including the 32-byte config field the Lite/Extended-WiFi variants
     * are told apart by) is left 0xFF, like a real mostly-blank EEPROM
     * outside the fields U-Boot actually checks.
     */
    {
        I2CBus *i2c0 = am335x_i2c_bus(DEVICE(&soc->i2c[0]));
        uint8_t board_id_eeprom[4096];

        memset(board_id_eeprom, 0xFF, sizeof(board_id_eeprom));
        memcpy(board_id_eeprom, "\xAA\x55\x33\xEE" "A335BNLT" "SE\xFF\xFF",
              4 + 8 + 4);
        at24c_eeprom_init_rom(i2c0, 0x50, sizeof(board_id_eeprom),
                              board_id_eeprom, sizeof(board_id_eeprom));

        i2c_slave_create_simple(i2c0, TYPE_TPS65217_PMU, 0x24);

        /*
         * TDA19988 HDMI encoder -- unchanged from Black (DT: &i2c0
         * tda19988@70 via am335x-boneblack-hdmi.dtsi, included by this
         * board's DTS exactly like Black's). See beaglebone.c for the paged
         * bank / CEC bank / HPD-GPIO wiring notes.
         */
        {
            I2CSlave *hdmi = i2c_slave_new(TYPE_TDA19988, 0x70);
            I2CSlave *cec = i2c_slave_new(TYPE_TDA19988_CEC, 0x34);

            i2c_slave_realize_and_unref(hdmi, i2c0, &error_fatal);
            object_property_set_link(OBJECT(cec), "hdmi", OBJECT(hdmi),
                                     &error_fatal);
            i2c_slave_realize_and_unref(cec, i2c0, &error_fatal);

            qdev_connect_gpio_out(DEVICE(hdmi), 0,
                                  qdev_get_gpio_in(DEVICE(&soc->gpio[1]), 25));
        }
    }

    memory_region_add_subregion(get_system_memory(), 0x80000000,
                                machine->ram);

    /*
     * Three mutually exclusive boot paths, in priority order -- identical to
     * beaglebone.c's Black variant (the SPL/boot-parameter address layout is
     * fixed by the SoC, so the flow is shared via am335x_bootflow.c):
     *
     *  -kernel: direct Linux boot, handled by arm_load_kernel() below.
     *  -bios <MLO>: boot the genuine SPL -> u-boot -> extlinux chain off the
     *      SD card, with no QEMU-side kernel injection.
     *  neither, with an SD card: what the real board does from a cold start --
     *      the boot ROM finds MLO on the card's FAT partition itself (TRM
     *      SPRUH73Q 26.1.8.5) and proceeds as in the -bios case.
     *
     * The boot-ROM SD/FAT scan targets the removable microSD on MMC0
     * (drive_get(IF_SD, 0, 0)), never the soldered eMMC on MMC1 -- same as
     * the real AM335x ROM's boot order for this board's SYSBOOT straps.
     */
    if (machine->firmware) {
        am335x_boot_load_mlo(machine, &bbe_binfo);
    } else if (!machine->kernel_filename) {
        DriveInfo *di = drive_get(IF_SD, 0, 0);
        BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;

        if (blk) {
            am335x_boot_from_sd(blk, &bbe_binfo);
        }
    }

    bbe_binfo.ram_size = machine->ram_size;
    arm_load_kernel(&soc->cpu, machine, &bbe_binfo);

    if (bbe_binfo.firmware_loaded) {
        /* Runs after arm_load_kernel()'s do_cpu_reset(); sets PC to MLO. */
        am335x_boot_register_firmware_reset(&soc->cpu, &bbe_binfo);
    }
}

static void beaglebone_enhanced_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-a8"),
        NULL
    };

    mc->desc = "SanCloud BeagleBone Enhanced (Cortex-A8)";
    mc->init = beaglebone_enhanced_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a8");
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_ram_id = "am335x.ram";
    mc->default_ram_size = 1 * GiB;
    mc->min_cpus = mc->max_cpus = mc->default_cpus = 1;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("beaglebone-enhanced", beaglebone_enhanced_machine_init)
