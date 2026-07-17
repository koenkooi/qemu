/*
 * Seeed Studio BeagleBone Green Wireless emulation
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
 * The Seeed BeagleBone Green Wireless (ti,am335x-bone-green-wireless) is the
 * same TI AM335x SoC as the BeagleBone Black on beaglebone.c, wired as a
 * Green-family board (Black's eMMC, no HDMI) that drops the wired RJ45 jack
 * and adds a TI WiLink8 (wl1835) combo radio: SDIO WiFi on the third MMC/SD
 * host controller and UART-attached Bluetooth. Confirmed from the mainline
 * kernel DT (arch/arm/boot/dts/ti/omap/am335x-bonegreen-wireless.dts, which
 * includes am335x-bone-common.dtsi + am335x-bonegreen-common.dtsi) and
 * U-Boot's board detection (board/ti/am335x/board.c):
 *
 *  - Onboard eMMC, present, unchanged from Black/Green: am335x-bonegreen-
 *    common.dtsi enables &mmc2 (status = "okay", bus-width 8, non-removable),
 *    so like Black/Enhanced/Green-Eco (and unlike White) MMC1 gets a
 *    soldered-on TYPE_EMMC.
 *  - No onboard HDMI: no Green board pulls in am335x-boneblack-hdmi.dtsi, so
 *    there is no tda19988 on I2C0 and &lcdc stays disabled -- like White and
 *    Green-Eco, this machine wires up no TDA19988 and no 0x70/0x34 I2C slaves.
 *  - PMIC is the standard TI TPS65217C at I2C0 0x24, unchanged from Black:
 *    the wireless DTS keeps am335x-bone-common.dtsi's pmic@24 (it only tweaks
 *    &ldo3_reg via am335x-bonegreen-common.dtsi), unlike the Green-Eco which
 *    deletes pmic@24 and swaps in a TPS65214. So this board wires TYPE_
 *    TPS65217_PMU at 0x24 like beaglebone.c.
 *  - NO wired Ethernet: am335x-bonegreen-wireless.dts sets
 *    "&mac_sw { status = "disabled"; }". The board has no RJ45 at all; the
 *    "network controller" here is the radio, not CPSW. The AM335x CPSW is part
 *    of TYPE_AM335X_SOC and is still instantiated/mapped in the address space
 *    (as on real silicon), but because the guest DTB leaves its node disabled,
 *    no cpsw-switch/davinci_mdio driver ever probes it -- it shows up as absent
 *    rather than broken. So, unlike Enhanced/Green-Eco, this board does NOT
 *    select the CPSW's Gigabit DP83867 PHY variant; the (unused) CPSW keeps its
 *    default 10/100 model. Any -nic given on the command line binds to that
 *    dormant CPSW and is simply never used by the guest.
 *  - SDIO WiFi on the THIRD MMC controller: am335x-bonegreen-wireless.dts
 *    enables &mmc3 (MMCHS2, added to the shared SoC in this series) with a
 *    "wlcore@2 { compatible = "ti,wl1835"; }" child -- a TI WiLink8 (wl18xx)
 *    SDIO WiFi function. We attach TYPE_WL18XX_SDIO (hw/sd/wl18xx_sdio.c) to
 *    MMCHS2's sd-bus: it is on-package and always present (like the eMMC), so
 *    it is wired unconditionally from board code rather than from a -sd drive.
 *    See that file for the "clean probe" scope: enough SDIO/CCCR/CIS + wl18xx
 *    chip-ID behaviour for the Linux wlcore/wl18xx driver to enumerate, bind
 *    and identify the chip; the firmware blob (loaded only at interface-up) is
 *    the documented boundary. The wl1835 SDIO IRQ is out-of-band on GPIO0_27
 *    (DT interrupt-parent = <&gpio0>, interrupts = <27>); it is served by the
 *    existing am335x GPIO controller and need not fire during probe.
 *  - UART-attached Bluetooth: &uart3 carries a "bluetooth { compatible =
 *    "ti,wl1835-st"; enable-gpios = <&gpio1 28>; }" serdev child (BT_EN on
 *    GPIO1_28). The AM335x SoC as modelled here instantiates only UART0 (the
 *    console), so UART3's byte transport is not present and the ti,wl1835-st
 *    HCI handshake is not modelled -- a known, bounded gap (parallel to the
 *    WiFi firmware wall). BT_EN itself needs no board wiring: it is a plain
 *    GPIO1 output the (existing) GPIO controller absorbs; the kernel toggles
 *    it during the HCI attach it cannot complete here.
 *  - GPIO hogs / regulators (ls-buf-en-hog on GPIO1_29, bt-aud-in-hog on
 *    GPIO3_16, and the wlan_en_reg / WL_EN GPIO0_26 regulator) are kernel-side
 *    pinctrl/gpiolib/regulator constructs; the existing am335x GPIO model
 *    handles them with no device-specific code, as gpio-hogs are not something
 *    the controller model needs to know about.
 *  - Board-ID EEPROM: U-Boot detects the Green Wireless inline in
 *    board_late_init() (board/ti/am335x/board.c): board_is_bone_lt() &&
 *    !strncmp(board_ti_get_rev(), "GW1", 3) sets board_name = "BBGW", and the
 *    findfdt env (include/configs/am335x_evm.h) maps BBGW ->
 *    am335x-bonegreen-wireless.dtb. So this board reuses Black's EEPROM name
 *    "A335BNLT" and is told apart only by the version/rev prefix "GW1" (real
 *    boards ship "GW1A"). Unlike bbge/bben there is no board_is_bbgw() macro --
 *    the check lives in board.c and only the runtime findfdt (not the SPL FIT
 *    config match, which falls through to am335x-boneblack) selects the DTB.
 *    We set magic 0xAA5533EE, name "A335BNLT", version "GW1A"; the rest is left
 *    0xFF, like a real mostly-blank EEPROM outside the fields U-Boot checks.
 *    Green Wireless detection was added upstream in commit 2b79fba6915d
 *    ("config: am335x_evm: detect Green Wireless using GW1", Robert Nelson,
 *    2017-03-30).
 *  - RAM: real hardware is 512 MB DDR3 (Seeed product spec). The mainline DTB
 *    does not override am335x-bone-common.dtsi's 256 MB memory@80000000 node
 *    (an upstream DT gap), so we default to the real 512 MB, still overridable
 *    with -m, following the same follow-the-hardware-spec precedent as
 *    beaglebone-enhanced.c / beaglebone-green-eco.c. (The M11 SPL path detects
 *    and fixes up the real size.)
 *  - LEDs and everything else are shared with the family via TYPE_AM335X_SOC
 *    and the same board wiring: the four USR0-3 LEDs on GPIO1_21..24
 *    (am335x-bone-common.dtsi leds{}) plus the fixed power LED. The USR LEDs
 *    follow the family's blue convention (see beaglebone-green-eco.c for why
 *    the DT "green:" labels are not a reliable colour source).
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
#include "hw/sd/wl18xx_sdio.h"
#include "hw/misc/led.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/am335x_i2c.h"
#include "hw/misc/tps65217.h"
#include "hw/nvram/eeprom_at24c.h"
#include "system/blockdev.h"
#include "exec/address-spaces.h"

/* On-board USR LED descriptions, keyed by GPIO1 line 21..24 -- identical
 * wiring and host-relay descriptions to beaglebone.c's Black variant (same DT
 * offsets), reused verbatim so the existing host LED relay/viewer just works. */
static const char * const bbgw_usr_led_desc[4] = {
    "beaglebone-usr0", "beaglebone-usr1",
    "beaglebone-usr2", "beaglebone-usr3",
};

static struct arm_boot_info bbgw_binfo = {
    .loader_start = 0x80000000,
    .board_id = -1,
};

static void beaglebone_green_wireless_init(MachineState *machine)
{
    AM335xState *soc;
    DeviceState *carddev;
    int i;

    soc = AM335X_SOC(object_new(TYPE_AM335X_SOC));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(soc));
    object_unref(OBJECT(soc));

    /*
     * No gigabit-phy selection here: the Green Wireless disables CPSW entirely
     * (&mac_sw disabled, see file comment above), so unlike Enhanced/Green-Eco
     * the (dormant) CPSW keeps its default 10/100 PHY -- the guest never
     * probes it.
     */
    qdev_realize(DEVICE(soc), NULL, &error_fatal);

    /*
     * MMC0 (removable microSD) and MMC1 (soldered eMMC) come from -sd drives,
     * exactly like beaglebone.c's Black variant: the Green Wireless keeps
     * Black's onboard eMMC (see file comment above), so MMC1 gets a TYPE_EMMC.
     * MMC2 is handled separately below.
     */
    for (i = 0; i < 2; i++) {
        DriveInfo *di = drive_get(IF_SD, 0, i);
        BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;

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
     * MMC2 (MMCHS2): the on-package TI WiLink8 (wl1835) SDIO WiFi function.
     * This is the whole point of the Green Wireless and the reason the shared
     * SoC now has a third MMC controller. It is not a user-removable card --
     * it is soldered to the module -- so it is attached unconditionally here
     * rather than from a -sd drive (TYPE_WL18XX_SDIO is user_creatable=false
     * for the same reason). See hw/sd/wl18xx_sdio.c for the clean-probe model.
     */
    carddev = qdev_new(TYPE_WL18XX_SDIO);
    qdev_realize_and_unref(carddev,
                           qdev_get_child_bus(DEVICE(&soc->mmc[2]), "sd-bus"),
                           &error_fatal);

    /*
     * On-board LEDs -- same GPIO1_21..24 USR0-3 wiring and fixed-on power LED
     * as beaglebone.c's Black variant; see that file for the trace/host-relay
     * notes. The USR LEDs follow the family's blue convention.
     */
    led_create_simple(OBJECT(machine), GPIO_POLARITY_ACTIVE_HIGH,
                      LED_COLOR_GREEN, "beaglebone-power");
    for (i = 0; i < 4; i++) {
        LEDState *led = led_create_simple(OBJECT(machine),
                                          GPIO_POLARITY_ACTIVE_HIGH,
                                          LED_COLOR_BLUE, bbgw_usr_led_desc[i]);
        qdev_connect_gpio_out(DEVICE(&soc->gpio[1]), 21 + i,
                              qdev_get_gpio_in(DEVICE(led), 0));
    }

    /*
     * On-board I2C0 slaves: board-ID EEPROM + TPS65217C PMIC. No TDA19988 HDMI
     * encoder (the Green family has no onboard HDMI) -- see file comment above.
     *
     * The board-ID EEPROM is an atmel,24c32 (4KB) carrying the BeagleBone SRM
     * header (magic 0xAA5533EE, board "A335BNLT" like Black, version "GW1A" so
     * U-Boot's board_late_init() -- which checks the "GW1" 3-byte rev prefix --
     * selects am335x-bonegreen-wireless.dtb; see file comment above). Only
     * those fields are populated; the rest is left 0xFF, like a real
     * mostly-blank EEPROM outside the fields U-Boot actually checks.
     */
    {
        I2CBus *i2c0 = am335x_i2c_bus(DEVICE(&soc->i2c[0]));
        uint8_t board_id_eeprom[4096];

        memset(board_id_eeprom, 0xFF, sizeof(board_id_eeprom));
        memcpy(board_id_eeprom, "\xAA\x55\x33\xEE" "A335BNLT" "GW1A",
               4 + 8 + 4);
        at24c_eeprom_init_rom(i2c0, 0x50, sizeof(board_id_eeprom),
                              board_id_eeprom, sizeof(board_id_eeprom));

        i2c_slave_create_simple(i2c0, TYPE_TPS65217_PMU, 0x24);
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
     *      SPRUH73Q 26.1.8.5) and proceeds as in the -bios case.
     *
     * The boot-ROM SD/FAT scan targets the removable microSD on MMC0
     * (drive_get(IF_SD, 0, 0)), never the eMMC on MMC1 or the SDIO WiFi on
     * MMC2 -- same as the real AM335x ROM's boot order for this board.
     */
    if (machine->firmware) {
        am335x_boot_load_mlo(machine, &bbgw_binfo);
    } else if (!machine->kernel_filename) {
        DriveInfo *di = drive_get(IF_SD, 0, 0);
        BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;

        if (blk) {
            am335x_boot_from_sd(blk, &bbgw_binfo);
        }
    }

    bbgw_binfo.ram_size = machine->ram_size;
    arm_load_kernel(&soc->cpu, machine, &bbgw_binfo);

    if (bbgw_binfo.firmware_loaded) {
        /* Runs after arm_load_kernel()'s do_cpu_reset(); sets PC to MLO. */
        am335x_boot_register_firmware_reset(&soc->cpu, &bbgw_binfo);
    }
}

static void beaglebone_green_wireless_machine_init(MachineClass *mc)
{
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-a8"),
        NULL
    };

    mc->desc = "Seeed BeagleBone Green Wireless (Cortex-A8)";
    mc->init = beaglebone_green_wireless_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a8");
    mc->valid_cpu_types = valid_cpu_types;
    mc->default_ram_id = "am335x.ram";
    mc->default_ram_size = 512 * MiB;
    mc->min_cpus = mc->max_cpus = mc->default_cpus = 1;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("beaglebone-green-wireless", beaglebone_green_wireless_machine_init)
