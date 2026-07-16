/*
 * Seeed Studio BeagleBone Green Eco emulation
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
 * The Seeed BeagleBone Green Eco (seeed,am335x-bone-green-eco) is the same TI
 * AM335x SoC as the BeagleBone Black on beaglebone.c, wired as a low-power
 * "eco" clone of the BeagleBone Green: it keeps Black's onboard eMMC, drops
 * HDMI (like every Green), and swaps in a different PMIC and a Gigabit RGMII
 * Ethernet PHY. Confirmed from the mainline kernel DT sources
 * (arch/arm/boot/dts/ti/omap/am335x-bonegreen-eco.dts, which includes
 * am335x-bone-common.dtsi + am335x-bonegreen-common.dtsi) and U-Boot's
 * board-detect code (board/ti/am335x/board.h, board/ti/am335x/board.c):
 *
 *  - Onboard eMMC, present, unchanged from Black: am335x-bonegreen-common.dtsi
 *    enables &mmc2 (status = "okay", bus-width 8, non-removable) exactly like
 *    am335x-boneblack-common.dtsi does. So, like Black/Enhanced and unlike
 *    White, MMC1 gets a soldered-on TYPE_EMMC (real hardware: 16 GB eMMC).
 *  - No onboard HDMI: no board in the Green family pulls in
 *    am335x-boneblack-hdmi.dtsi, so there is no tda19988 node on I2C0 and
 *    &lcdc stays disabled -- the Green Eco exposes two Grove connectors (one
 *    I2C, one UART -- am335x-bonegreen-common.dtsi enables &uart2) in place of
 *    Black's micro-HDMI. So, like White, this machine wires up no TDA19988 and
 *    no 0x70/0x34 I2C slaves.
 *  - PMIC is a TI TPS65214, NOT the TPS65217C every other modelled board uses:
 *    am335x-bonegreen-eco.dts does "/delete-node/ pmic@24;" (removing the
 *    inherited am335x-bone-common.dtsi TPS65217 at 0x24) and adds
 *    "tps65214: pmic@30 { compatible = "ti,tps65214"; reg = <0x30>; ... }" --
 *    a different chip at a different I2C address (real hardware: the orderable
 *    TPS6521403). This is a genuinely different regmap-based part driven by
 *    drivers/mfd/tps65219.c (the TPS65219/65215/65214 family), not the
 *    drivers/mfd/tps65217.c driver. It is modelled by a new minimal I2C slave,
 *    hw/misc/tps65214.c (TYPE_TPS65214_PMU): the mfd driver does no chip-ID
 *    read, so a dumb byte-addressed register file is enough to ACK the one
 *    register-unlock write and the regmap-irq mask/status accesses its probe
 *    performs. See that file for the detailed driver-probe analysis. (The
 *    buck1..3/ldo1..2 regulators it exposes are all "regulator-always-on" in
 *    DT and set up by the bootloader on real hardware, so nothing in the guest
 *    path depends on the regulator values, only on the probe succeeding.)
 *  - Gigabit RGMII Ethernet PHY (TI DP83867): am335x-bonegreen-eco.dts sets
 *    "&cpsw_port1 { phy-mode = "rgmii-id"; phy-handle = <&dp83867_0>; }" and
 *    replaces the shared MII ethernet-phy@0 with an explicitly-named
 *    "dp83867_0: ethernet-phy@0 { ... ti,dp83867-rxctrl-strap-quirk; ... }",
 *    overriding am335x-bone-common.dtsi's 100Mbit MII &ethphy0. Unlike the
 *    SanCloud Enhanced (whose DT only implies an RGMII Gigabit PHY without
 *    naming the chip), the Green Eco's DT names "ti,dp83867" outright -- the
 *    strongest concrete evidence in the family for that specific PHY. This is
 *    the same on-chip CPSW/MDIO silicon as Black (TYPE_AM335X_SOC, unchanged)
 *    talking to a different *external* PHY, which we model by setting the
 *    CPSW's "gigabit-phy" qdev property (exactly as beaglebone-enhanced.c
 *    does): it swaps the shared hw/net/am335x_cpsw.c model's default 10/100
 *    lan9118_phy for a TI DP83867 Gigabit RGMII PHY (hw/net/dp83867_phy.c)
 *    that reports a 1000Mbit/full link.
 *  - Board-ID EEPROM: U-Boot's board_is_bbge() (board/ti/am335x/board.h) is
 *    "board_is_bone_lt() && !strncmp(board_ti_get_rev(), "BBGE", 4)" -- i.e.
 *    the Green Eco reuses Black's EEPROM "name" field ("A335BNLT") and is told
 *    apart only by a "version"/rev field of "BBGE" (contrast the regular
 *    BeagleBone Green's "BBG1" via board_is_bbg1(), and Enhanced's "SE" via
 *    board_is_bben()). board.c's board_fit_config_name_match() maps
 *    board_is_bbge() -> the "am335x-bonegreen-eco" FIT config / DTB used here.
 *    So we set magic 0xAA5533EE, name "A335BNLT", version "BBGE"; everything
 *    else is left 0xFF (unwritten), like a real mostly-blank EEPROM outside the
 *    fields U-Boot actually checks (same precedent as beaglebone-white.c /
 *    beaglebone-enhanced.c). board_is_bbge() support was added upstream in
 *    commit 510f2502475b ("board: ti: am33xx: Add support for BeagleBoard
 *    Green Eco", Kory Maincent, 2025-04-25).
 *  - RAM: real hardware is 512 MB DDR3L (Seeed/TI product spec). The mainline
 *    DTB does not override am335x-bone-common.dtsi's memory@80000000 node, so
 *    the built am335x-bonegreen-eco.dtb still declares 256 MB -- an upstream
 *    DT gap, not a modelling choice here. We default to the real 512 MB (still
 *    overridable with -m), following the same follow-the-hardware-spec
 *    precedent beaglebone-enhanced.c uses for its 1 GiB. (In -kernel direct
 *    boot with the stock DTB the guest honours the DTB's 256 MB cap; the M11
 *    SPL -> u-boot path detects/fixes up the real size.)
 *  - LEDs and everything else this machine's DT enables are shared with the
 *    rest of the family via TYPE_AM335X_SOC and the same board-level wiring:
 *    the four USR0-3 LEDs on GPIO1_21..24 (am335x-bone-common.dtsi leds{}
 *    node, common to every BeagleBone board) plus the fixed power LED. The
 *    four USR LEDs are modelled LED_COLOR_BLUE like the rest of the family:
 *    the DT leds{} "beaglebone:green:*" labels are a shared legacy string
 *    carried verbatim by every BeagleBone board (including the physically
 *    *blue*-LED Black), so they are not a reliable per-board colour, and the
 *    Green Eco's user-LED colour is not independently documented (the Seeed
 *    spec lists the LEDs but not their colour); absent a source we follow the
 *    family convention rather than assert a different colour. The host-relay
 *    description strings are kept identical so the existing LED viewer just
 *    works.
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
#include "hw/misc/tps65214.h"
#include "hw/nvram/eeprom_at24c.h"
#include "system/blockdev.h"
#include "exec/address-spaces.h"

/* On-board USR LED descriptions, keyed by GPIO1 line 21..24 -- identical
 * wiring and host-relay descriptions to beaglebone.c's Black variant (same DT
 * offsets, see file comment above), reused verbatim so the existing host LED
 * relay/viewer just works. */
static const char * const bbge_usr_led_desc[4] = {
    "beaglebone-usr0", "beaglebone-usr1",
    "beaglebone-usr2", "beaglebone-usr3",
};

static struct arm_boot_info bbge_binfo = {
    .loader_start = 0x80000000,
    .board_id = -1,
};

static void beaglebone_green_eco_init(MachineState *machine)
{
    AM335xState *soc;
    int i;

    soc = AM335X_SOC(object_new(TYPE_AM335X_SOC));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    object_unref(OBJECT(soc));

    /*
     * The Green Eco muxes the CPSW to RGMII with an external TI DP83867
     * Gigabit PHY -- and, uniquely in this family, names that chip explicitly
     * in DT (see file comment above), unlike Black/White's 10/100 MII PHY.
     * Select the CPSW model's Gigabit DP83867 variant before the SoC (and its
     * CPSW child) is realized, the same way beaglebone-enhanced.c does; the
     * child object already exists here (the SoC's instance_init created it),
     * so the board sets the property on it directly, just as it reaches into
     * soc->mmc/soc->gpio/soc->i2c below.
     */
    object_property_set_bool(OBJECT(&soc->cpsw), "gigabit-phy", true,
                             &error_fatal);

    qdev_realize(DEVICE(soc), NULL, &error_fatal);

    /*
     * Attach SD/MMC cards from -sd (first IF_SD drive -> MMC0 -> mmcblk0,
     * second -> MMC1 -> mmcblk1), exactly like beaglebone.c's Black variant:
     * the Green Eco keeps Black's onboard eMMC (see file comment above), so
     * MMC1 gets a soldered-on TYPE_EMMC (which answers CMD1/CMD8 an SD card
     * would NAK) rather than being left unconnected like White's.
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
     * On-board LEDs -- same GPIO1_21..24 USR0-3 wiring and fixed-on power LED
     * as beaglebone.c's Black variant; see that file for the trace/host-relay
     * notes. The USR LEDs follow the family's blue convention (see file
     * comment above for why the DT "green:" labels are not a colour source).
     */
    led_create_simple(OBJECT(machine), GPIO_POLARITY_ACTIVE_HIGH,
                      LED_COLOR_GREEN, "beaglebone-power");
    for (i = 0; i < 4; i++) {
        LEDState *led = led_create_simple(OBJECT(machine),
                                          GPIO_POLARITY_ACTIVE_HIGH,
                                          LED_COLOR_BLUE, bbge_usr_led_desc[i]);
        qdev_connect_gpio_out(DEVICE(&soc->gpio[1]), 21 + i,
                              qdev_get_gpio_in(DEVICE(led), 0));
    }

    /*
     * On-board I2C0 slaves: board-ID EEPROM + TPS65214 PMIC. No TPS65217C
     * (deleted in this board's DT) and no TDA19988 HDMI encoder (the Green
     * family has no onboard HDMI) -- see file comment above.
     *
     * The board-ID EEPROM is an atmel,24c32 (4KB) carrying the BeagleBone SRM
     * header (magic 0xAA5533EE, board "A335BNLT" like Black, version "BBGE" so
     * U-Boot's board_is_bbge() -- which checks the "BBGE" 4-byte rev prefix --
     * identifies this as a Green Eco; see file comment above). Only those
     * fields are populated; the rest is left 0xFF like a real mostly-blank
     * EEPROM outside the fields U-Boot actually checks.
     *
     * The TPS65214 PMIC sits at 0x30 (not 0x24) and is modelled by
     * TYPE_TPS65214_PMU (hw/misc/tps65214.c). Its NMI-line interrupt is not
     * wired to anything, matching how the TPS65217 boards leave the PMIC IRQ
     * unconnected.
     */
    {
        I2CBus *i2c0 = am335x_i2c_bus(DEVICE(&soc->i2c[0]));
        uint8_t board_id_eeprom[4096];

        memset(board_id_eeprom, 0xFF, sizeof(board_id_eeprom));
        memcpy(board_id_eeprom, "\xAA\x55\x33\xEE" "A335BNLT" "BBGE",
               4 + 8 + 4);
        at24c_eeprom_init_rom(i2c0, 0x50, sizeof(board_id_eeprom),
                              board_id_eeprom, sizeof(board_id_eeprom));

        i2c_slave_create_simple(i2c0, TYPE_TPS65214_PMU, 0x30);
    }

    memory_region_add_subregion(get_system_memory(), 0x80000000,
                                machine->ram);

    /*
     * Three mutually exclusive boot paths, in priority order (same as
     * beaglebone.c's Black variant; see that file for the detailed rationale):
     *
     *  -kernel: direct Linux boot, handled by arm_load_kernel() below.
     *  -bios <MLO>: boot the genuine SPL -> u-boot -> extlinux chain off the
     *      SD card, with no QEMU-side kernel injection.
     *  neither, with an SD card: what the real board does from a cold start --
     *      the boot ROM finds MLO on the card's FAT partition itself (TRM
     *      SPRUH73Q 26.1.8.5) and everything proceeds as in the -bios case.
     */
    if (machine->firmware) {
        am335x_boot_load_mlo(machine, &bbge_binfo);
    } else if (!machine->kernel_filename) {
        DriveInfo *di = drive_get(IF_SD, 0, 0);
        BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;

        if (blk) {
            am335x_boot_from_sd(blk, &bbge_binfo);
        }
    }

    bbge_binfo.ram_size = machine->ram_size;
    arm_load_kernel(&soc->cpu, machine, &bbge_binfo);

    if (bbge_binfo.firmware_loaded) {
        /* Runs after arm_load_kernel()'s do_cpu_reset(); sets PC to MLO. */
        am335x_boot_register_firmware_reset(&soc->cpu, &bbge_binfo);
    }
}

static void beaglebone_green_eco_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-a8"),
        NULL
    };

    mc->desc = "Seeed BeagleBone Green Eco (Cortex-A8)";
    mc->init = beaglebone_green_eco_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a8");
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_ram_id = "am335x.ram";
    mc->default_ram_size = 512 * MiB;
    mc->min_cpus = mc->max_cpus = mc->default_cpus = 1;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("beaglebone-green-eco", beaglebone_green_eco_machine_init)
