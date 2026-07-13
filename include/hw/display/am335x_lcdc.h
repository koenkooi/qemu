/*
 * TI AM335x LCD Controller (LCDC / "ti,am33xx-tilcdc").
 *
 * Reference: TRM spruh73q chapter 13 "LCD Controller", base 0x4830E000,
 * INTC line 36. Register offsets follow both the TRM (Table 13-13) and the
 * Linux tilcdc driver (drivers/gpu/drm/tilcdc/tilcdc_regs.h).
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef HW_DISPLAY_AM335X_LCDC_H
#define HW_DISPLAY_AM335X_LCDC_H

#include "hw/sysbus.h"
#include "exec/memory.h"
#include "ui/console.h"
#include "qom/object.h"

#define TYPE_AM335X_LCDC "am335x-lcdc"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xLcdcState, AM335X_LCDC)

struct AM335xLcdcState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    MemoryRegionSection fbsection;
    QemuConsole *con;
    qemu_irq irq;                /* INTC line 36 */
    QEMUTimer *vblank_timer;     /* periodic END_OF_FRAME0 while scanning */

    /* Register state (TRM Table 13-13). */
    uint32_t ctrl;               /* 0x04 CTRL */
    uint32_t raster_ctrl;        /* 0x28 RASTER_CTRL */
    uint32_t raster_timing[3];   /* 0x2c/0x30/0x34 RASTER_TIMING_0/1/2 */
    uint32_t dma_ctrl;           /* 0x40 LCDDMA_CTRL */
    uint32_t fb0_base;           /* 0x44 LCDDMA_FB0_BASE (scanout base) */
    uint32_t fb0_ceiling;        /* 0x48 LCDDMA_FB0_CEILING (scanout end) */
    uint32_t fb1_base;           /* 0x4c LCDDMA_FB1_BASE (unused) */
    uint32_t fb1_ceiling;        /* 0x50 LCDDMA_FB1_CEILING (unused) */
    uint32_t sysconfig;          /* 0x54 SYSCONFIG (OCP idle/standby) */
    uint32_t irqstatus;          /* 0x58/0x5c raw/masked status */
    uint32_t irqenable;          /* behind 0x60/0x64 SET/CLEAR */
    uint32_t clkc_enable;        /* 0x6c CLKC_ENABLE */

    int invalidate;
    /* Resolution cached from the last framebuffer scan (decoded from the
     * RASTER_TIMING registers), so the surface only resizes on a real
     * mode change. */
    int width;
    int height;
};

#endif /* HW_DISPLAY_AM335X_LCDC_H */
