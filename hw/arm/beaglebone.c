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
#include "qemu/datadir.h"
#include "hw/boards.h"
#include "hw/arm/am335x_soc.h"
#include "hw/arm/boot.h"
#include "hw/loader.h"
#include "hw/qdev-properties.h"
#include "hw/sd/sd.h"
#include "hw/misc/led.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/am335x_i2c.h"
#include "hw/misc/tps65217.h"
#include "hw/display/tda19988.h"
#include "hw/nvram/eeprom_at24c.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "exec/address-spaces.h"

/*
 * "From-scratch" SD boot via -bios <MLO>. QEMU stands in for the AM335x
 * boot ROM: it places the real TI SPL (MLO) at its SRAM entry point, seeds
 * the handful of boot parameters the ROM leaves in SRAM, and starts the CPU
 * there. SPL then does genuine EMIF-driven DDR3 init and reads u-boot.img
 * off the SD card's FAT partition itself -- no -kernel/-dtb/-initrd needed.
 *
 * Addresses are the AM335x GP-device values from u-boot's
 * arch/arm/include/asm/arch-am33xx/omap.h and asm/omap_common.h:
 *   NON_SECURE_SRAM_START = 0x402F0400 (== CONFIG_SPL_TEXT_BASE, the ROM's
 *                                       SPL download/run address)
 *   NON_SECURE_SRAM_END   = 0x40310000
 *
 * The ROM passes the address of the boot-parameter struct to SPL in r0.
 * SPL's save_boot_params (reached from the reset vector) stashes that r0 at
 * OMAP_SRAM_SCRATCH_BOOT_PARAMS; save_omap_boot_params()
 * (arch/arm/mach-omap2/boot-common.c) later reads it back, uses
 * boot_device (offset 8) and follows boot_device_descriptor (offset 4)
 * -> +DEVICE_DATA_OFFSET(0x18) -> +BOOT_MODE_OFFSET(0x8) for the boot mode.
 * So we seed the struct/descriptor/device_data in SRAM scratch and hand r0
 * to the CPU at reset -- we must NOT write the scratch pointer slot
 * ourselves, since save_boot_params overwrites it from r0 first. We report
 * BOOT_DEVICE_MMC1 (SD/MMC0) and MMCSD_MODE_FS so SPL loads u-boot.img from
 * the FAT partition. All addresses lie in the reserved SRAM scratch band
 * (0x4030B400..0x4030B800), which SPL's own image/BSS/stack
 * (SP = 0x4030FF00) never touch this early.
 */
#define BBB_SRAM_START          0x402F0400
#define BBB_SRAM_END            0x40310000
#define BBB_BP_STRUCT           0x4030B430  /* omap_boot_parameters (-> r0) */
#define BBB_BP_DESCRIPTOR       0x4030B450  /* boot_device_descriptor */
#define BBB_BP_DEVICE_DATA      0x4030B470  /* device data */
#define BBB_BOOT_DEVICE_MMC1    0x08        /* asm/arch-am33xx/spl.h */
#define BBB_MMCSD_MODE_FS       2           /* include/spl.h */

static struct arm_boot_info bbb_binfo = {
    .loader_start = 0x80000000,
    .board_id = -1,
};

/*
 * Set the CPU's initial PC to the SPL entry. arm_load_kernel()'s firmware
 * path deliberately leaves env->boot_info NULL (do_cpu_reset() then resets
 * the CPU but does not touch the PC, since a -bios image normally sits at
 * the 0x0 reset vector). MLO instead runs from internal SRAM, so we set the
 * PC ourselves in a reset handler registered *after* arm_load_kernel(), so
 * it runs after do_cpu_reset() and wins.
 */
static void bbb_firmware_reset(void *opaque)
{
    ARMCPU *cpu = opaque;

    cpu_set_pc(CPU(cpu), bbb_binfo.entry);
    /* The boot ROM passes the boot-parameter struct pointer to SPL in r0. */
    cpu->env.regs[0] = BBB_BP_STRUCT;
}

/*
 * Load the real TI SPL (MLO) the way the boot ROM would: strip the TI
 * image header, place the SPL body at its GP-header load address in SRAM,
 * and seed the ROM boot parameters. Sets bbb_binfo.firmware_loaded/entry.
 */
static void beaglebone_load_mlo(MachineState *machine)
{
    char *fw_path;
    gchar *contents = NULL;
    gsize len = 0;
    GError *gerr = NULL;
    uint32_t gp_off = 0;
    uint32_t img_size, load_addr;
    size_t body_len;
    uint8_t bootparams[0x4C];

    fw_path = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
    if (!fw_path) {
        error_report("Could not find MLO firmware image '%s'",
                     machine->firmware);
        exit(1);
    }
    if (!g_file_get_contents(fw_path, &contents, &len, &gerr)) {
        error_report("Could not read MLO '%s': %s", fw_path, gerr->message);
        exit(1);
    }
    g_free(fw_path);

    /*
     * The TI 'omapimage' MLO for a GP device starts with a 512-byte
     * configuration header (TOC + CHSETTINGS); the 8-byte GP header (image
     * size, load address) follows it. Detect the CH by its "CHSETTINGS"
     * TOC entry name at offset 0x14; a bare GP image has the header at 0.
     */
    if (len >= 0x210 && memcmp(contents + 0x14, "CHSETTINGS", 10) == 0) {
        gp_off = 0x200;
    }
    if (len < gp_off + 8) {
        error_report("MLO '%s' too short for a GP header", machine->firmware);
        exit(1);
    }
    img_size = ldl_le_p(contents + gp_off);
    load_addr = ldl_le_p(contents + gp_off + 4);
    body_len = len - gp_off - 8;
    if (img_size && img_size < body_len) {
        body_len = img_size;
    }

    if (load_addr < BBB_SRAM_START || load_addr >= BBB_SRAM_END) {
        error_report("MLO load address 0x%08x is outside AM335x internal SRAM",
                     load_addr);
        exit(1);
    }

    rom_add_blob_fixed("am335x.mlo", contents + gp_off + 8, body_len,
                       load_addr);
    g_free(contents);

    /*
     * Seed the ROM boot parameters (see the layout comment above). The blob
     * base is the omap_boot_parameters struct (what r0 points at); byte
     * offsets are just the target SRAM address minus BBB_BP_STRUCT. The
     * scratch pointer slot (0x4030B424) is intentionally not written here --
     * SPL's save_boot_params fills it from r0.
     */
#define BP_OFF(addr) ((addr) - BBB_BP_STRUCT)
    memset(bootparams, 0, sizeof(bootparams));
    /* struct: reserved(+0)=0, boot_device_descriptor(+4), boot_device(+8). */
    stl_le_p(bootparams + BP_OFF(BBB_BP_STRUCT + 4), BBB_BP_DESCRIPTOR);
    bootparams[BP_OFF(BBB_BP_STRUCT + 8)] = BBB_BOOT_DEVICE_MMC1;
    /* descriptor: +DEVICE_DATA_OFFSET(0x18) -> device data. */
    stl_le_p(bootparams + BP_OFF(BBB_BP_DESCRIPTOR + 0x18), BBB_BP_DEVICE_DATA);
    /* device data: +BOOT_MODE_OFFSET(0x8) = MMCSD_MODE_FS. */
    stl_le_p(bootparams + BP_OFF(BBB_BP_DEVICE_DATA + 0x8), BBB_MMCSD_MODE_FS);
#undef BP_OFF
    rom_add_blob_fixed("am335x.bootparams", bootparams, sizeof(bootparams),
                       BBB_BP_STRUCT);

    bbb_binfo.entry = load_addr;
    bbb_binfo.firmware_loaded = true;
}

/* On-board USR LED descriptions, keyed by GPIO1 line 21..24. These strings
 * are the identifiers a host relay matches to drive the board UI. */
static const char * const bbb_usr_led_desc[4] = {
    "beaglebone-usr0", "beaglebone-usr1",
    "beaglebone-usr2", "beaglebone-usr3",
};

static void beaglebone_init(MachineState *machine)
{
    AM335xState *soc;
    int i;

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

    /*
     * On-board I2C0 slaves. Which slaves sit on which bus at which address
     * is board-level policy, so the wiring lives here rather than in the
     * SoC (matching the SD-card attachment above).
     *
     * The board-ID EEPROM is an atmel,24c32 (4KB) at 0x50 carrying the
     * BeagleBone SRM header (magic 0xAA5533EE, board "A335BNLT", rev "00C0").
     * A real, mostly-unwritten EEPROM reads 0xFF outside the header.
     *
     * The TPS65217C PMIC is at 0x24; the Linux mfd driver only needs its
     * CHIPID read (0xE2) to ACK for the regulator/charger children to probe.
     */
    {
        I2CBus *i2c0 = am335x_i2c_bus(DEVICE(&soc->i2c[0]));
        uint8_t board_id_eeprom[4096];

        memset(board_id_eeprom, 0xFF, sizeof(board_id_eeprom));
        memcpy(board_id_eeprom,
               "\xAA\x55\x33\xEE" "A335BNLT" "00C0" "4115BBBK0001",
               4 + 8 + 4 + 12);
        at24c_eeprom_init_rom(i2c0, 0x50, sizeof(board_id_eeprom),
                              board_id_eeprom, sizeof(board_id_eeprom));

        i2c_slave_create_simple(i2c0, TYPE_TPS65217_PMU, 0x24);

        /*
         * TDA19988 HDMI encoder (DT: &i2c0 tda19988@70). It answers two I2C
         * addresses -- the main paged bank at 0x70 and the CEC bank at 0x34 --
         * modelled as two slaves sharing one state via the CEC device's "hdmi"
         * link. Its HPD/EDID interrupt drives GPIO1 line 25
         * (interrupts-extended = <&gpio1 25 IRQ_TYPE_LEVEL_LOW>), which the
         * tda998x driver's EDID-block-ready wait depends on.
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
     * -bios <MLO>: boot the genuine SPL -> u-boot -> extlinux chain off the
     * SD card, with no QEMU-side kernel injection. Additive to (and mutually
     * exclusive at runtime with) the -kernel direct-Linux-boot path, which
     * is unchanged.
     */
    if (machine->firmware) {
        beaglebone_load_mlo(machine);
    }

    bbb_binfo.ram_size = machine->ram_size;
    arm_load_kernel(&soc->cpu, machine, &bbb_binfo);

    if (machine->firmware) {
        /* Runs after arm_load_kernel()'s do_cpu_reset(); sets PC to MLO. */
        qemu_register_reset(bbb_firmware_reset, &soc->cpu);
    }
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
