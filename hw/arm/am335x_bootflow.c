/*
 * Shared AM335x "from-scratch" SD-card boot flow.
 *
 * QEMU stands in for the AM335x boot ROM: it places the real TI SPL (MLO)
 * at its SRAM entry point, seeds the handful of boot parameters the ROM
 * leaves in SRAM, and starts the CPU there. SPL then does genuine
 * EMIF-driven DDR3 init and reads u-boot.img off the SD card's FAT
 * partition itself -- no -kernel/-dtb/-initrd needed. The MLO image comes
 * either from a host file (-bios <MLO>) or, when neither -bios nor -kernel
 * is given, straight off the -sd card image's FAT partition, found the way
 * the ROM's file-system boot mode would find it (am335x_bootrom.c).
 *
 * Extracted out of hw/arm/beaglebone.c (the BeagleBone Black machine, where
 * this flow was first implemented and verified) so every AM335x
 * BeagleBone-family board can share it: the address layout below is fixed
 * by the SoC's boot ROM and SPL, not by which board it's soldered onto.
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

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/datadir.h"
#include "hw/boards.h"
#include "hw/arm/am335x_bootflow.h"
#include "hw/arm/am335x_soc.h"
#include "hw/loader.h"
#include "system/reset.h"

#define BBB_SRAM_START          0x402F0400
#define BBB_SRAM_END            0x40310000
#define BBB_BP_STRUCT           0x4030B430  /* omap_boot_parameters (-> r0) */
#define BBB_BP_DESCRIPTOR       0x4030B450  /* boot_device_descriptor */
#define BBB_BP_DEVICE_DATA      0x4030B470  /* device data */
#define BBB_BOOT_DEVICE_MMC1    0x08        /* asm/arch-am33xx/spl.h */
#define BBB_MMCSD_MODE_FS       2           /* include/spl.h */

/*
 * Load a TI SPL image (MLO) the way the boot ROM would: strip the TI image
 * header, place the SPL body at its GP-header load address in SRAM, and
 * seed the ROM boot parameters. Sets binfo->firmware_loaded/entry.
 * 'name' is only used in error messages.
 */
static void am335x_load_spl(const char *name, const uint8_t *data,
                            size_t len, struct arm_boot_info *binfo)
{
    uint32_t gp_off = 0;
    uint32_t img_size, load_addr;
    size_t body_len;
    uint8_t bootparams[0x4C];

    /*
     * The TI 'omapimage' MLO for a GP device starts with a 512-byte
     * configuration header (TOC + CHSETTINGS); the 8-byte GP header (image
     * size, load address) follows it. Detect the CH by its "CHSETTINGS"
     * TOC entry name at offset 0x14; a bare GP image has the header at 0.
     */
    if (len >= 0x210 && memcmp(data + 0x14, "CHSETTINGS", 10) == 0) {
        gp_off = 0x200;
    }
    if (len < gp_off + 8) {
        error_report("MLO '%s' too short for a GP header", name);
        exit(1);
    }
    img_size = ldl_le_p(data + gp_off);
    load_addr = ldl_le_p(data + gp_off + 4);
    body_len = len - gp_off - 8;
    if (img_size && img_size < body_len) {
        body_len = img_size;
    }

    if (load_addr < BBB_SRAM_START || load_addr >= BBB_SRAM_END) {
        error_report("MLO load address 0x%08x is outside AM335x internal SRAM",
                     load_addr);
        exit(1);
    }

    rom_add_blob_fixed("am335x.mlo", data + gp_off + 8, body_len, load_addr);

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

    binfo->entry = load_addr;
    binfo->firmware_loaded = true;
}

void am335x_boot_load_mlo(MachineState *machine, struct arm_boot_info *binfo)
{
    char *fw_path;
    gchar *contents = NULL;
    gsize len = 0;
    GError *gerr = NULL;

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

    am335x_load_spl(machine->firmware, (const uint8_t *)contents, len, binfo);
    g_free(contents);
}

void am335x_boot_from_sd(BlockBackend *blk, struct arm_boot_info *binfo)
{
    g_autofree uint8_t *mlo = NULL;
    size_t len = 0;

    mlo = am335x_bootrom_read_mlo(blk, &len, &error_fatal);
    am335x_load_spl("MLO (from SD card)", mlo, len, binfo);
}

typedef struct AM335xFirmwareResetState {
    ARMCPU *cpu;
    struct arm_boot_info *binfo;
} AM335xFirmwareResetState;

/*
 * Set the CPU's initial PC to the SPL entry. arm_load_kernel()'s firmware
 * path deliberately leaves env->boot_info NULL (do_cpu_reset() then resets
 * the CPU but does not touch the PC, since a -bios image normally sits at
 * the 0x0 reset vector). MLO instead runs from internal SRAM, so we set the
 * PC ourselves in a reset handler registered *after* arm_load_kernel(), so
 * it runs after do_cpu_reset() and wins.
 */
static void am335x_firmware_reset(void *opaque)
{
    AM335xFirmwareResetState *s = opaque;

    cpu_set_pc(CPU(s->cpu), s->binfo->entry);
    /* The boot ROM passes the boot-parameter struct pointer to SPL in r0. */
    s->cpu->env.regs[0] = BBB_BP_STRUCT;
}

void am335x_boot_register_firmware_reset(ARMCPU *cpu,
                                         struct arm_boot_info *binfo)
{
    AM335xFirmwareResetState *s = g_new0(AM335xFirmwareResetState, 1);

    s->cpu = cpu;
    s->binfo = binfo;
    qemu_register_reset(am335x_firmware_reset, s);
}
