/*
 * TI AM335x MMC/SD host controller (MMCHS, "ti,omap4-hsmmc").
 *
 * The AM335x MMCHS is an SD Host Standard (SDHCI) core wrapped in a TI
 * "highlander" register block. The SD Host Standard registers begin at
 * offset 0x200 within the module window (TRM spruh73q ch.18: SD_SDMASA at
 * 0x200), while the TI wrapper registers (SD_SYSCONFIG @0x110,
 * SD_SYSSTATUS @0x114, SD_CON @0x12C, ...) live in the low 0x200 bytes.
 *
 * Modelled after hw/sd/cadence_sdhci.c: an outer container MemoryRegion
 * holds the TI wrapper regs at offset 0 and an embedded TYPE_SYSBUS_SDHCI
 * mapped at offset 0x200. The IRQ and the "sd-bus" of the embedded core are
 * re-exported so a board can attach an SD card straight to this device.
 *
 * Copyright (C) 2026 Koen Kooi
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

#ifndef HW_SD_AM335X_HSMMC_H
#define HW_SD_AM335X_HSMMC_H

#include "hw/sysbus.h"
#include "hw/sd/sdhci.h"
#include "qom/object.h"

#define TYPE_AM335X_HSMMC "am335x-hsmmc"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xHsmmcState, AM335X_HSMMC)

/* Whole MMCHS module window (both wrapper and SDHCI live inside this). */
#define AM335X_HSMMC_MMIO_SIZE      0x1000

/* TI wrapper register block: offsets 0x000..0x1FF. */
#define AM335X_HSMMC_WRAP_SIZE      0x200
#define AM335X_HSMMC_NUM_WRAP_REGS  (AM335X_HSMMC_WRAP_SIZE / sizeof(uint32_t))

/* The SD Host Standard register block starts here within the container. */
#define AM335X_HSMMC_SDHCI_OFFSET   0x200

struct AM335xHsmmcState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion container;         /* 0x1000 outer module window */
    MemoryRegion wrapper_iomem;     /* TI wrapper regs @ container offset 0 */

    uint32_t wrap_regs[AM335X_HSMMC_NUM_WRAP_REGS];

    SDHCIState sdhci;               /* embedded TYPE_SYSBUS_SDHCI core */
};

#endif /* HW_SD_AM335X_HSMMC_H */
