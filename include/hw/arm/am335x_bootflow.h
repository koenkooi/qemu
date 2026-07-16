/*
 * Shared AM335x "from-scratch" SD-card boot flow.
 *
 * Loads an SPL (MLO) image exactly as the boot ROM would -- either from a
 * host file (-bios) or by reading it out of an SD card's FAT partition
 * ourselves (am335x_bootrom.c) -- and arranges for the CPU to start there.
 * Board-generic: every AM335x BeagleBone-family machine that wants a
 * genuine SPL -> u-boot -> extlinux boot chain instead of -kernel injection
 * uses this, since the SRAM/boot-parameter address layout (TRM SPRUH73Q
 * 26.1.8.5, u-boot's arch/arm/include/asm/arch-am33xx/omap.h and
 * asm/omap_common.h) is fixed by the SoC, not the board. See
 * hw/arm/am335x_bootflow.c for the detailed rationale.
 */
#ifndef HW_ARM_AM335X_BOOTFLOW_H
#define HW_ARM_AM335X_BOOTFLOW_H

#include "hw/arm/boot.h"
#include "system/blockdev.h"

/* -bios <MLO>: read the SPL from a host file and load it as the ROM would.
 * Exits the process on error (bad/missing file), matching arm_load_kernel()
 * error-handling conventions elsewhere in these machines. */
void am335x_boot_load_mlo(MachineState *machine, struct arm_boot_info *binfo);

/* Bare "-sd <image>" boot, no -bios/-kernel at all: find the booting file
 * "MLO" on the card's FAT partition ourselves (am335x_bootrom.c) and load
 * it the same way as am335x_boot_load_mlo(). Exits the process on error. */
void am335x_boot_from_sd(BlockBackend *blk, struct arm_boot_info *binfo);

/* Call once arm_load_kernel() has run, only when binfo->firmware_loaded is
 * true: registers the reset hook that sets the CPU's initial PC to the SPL
 * entry point and seeds r0 with the boot-parameter struct pointer, since
 * arm_load_kernel()'s firmware path deliberately leaves both untouched. */
void am335x_boot_register_firmware_reset(ARMCPU *cpu,
                                         struct arm_boot_info *binfo);

#endif
