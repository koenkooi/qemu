/*
 * TI AM335x Watchdog Timer (WDT1) emulation.
 *
 * Reference: TRM spruh73q, chapter 20.4 "Watchdog Timer Registers",
 * base 0x44E35000 (WDT1 / wd_timer1).
 *
 * This is a minimal (M1) stub: it implements just enough of the register
 * set for the Linux ti-sysc / omap_wdt drivers to probe the module,
 * complete an OCP softreset, and (typically) disable the watchdog --
 * without the module ever actually arming or biting. See am335x_wdt.c
 * for the read-side behaviour that makes this work.
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

#ifndef HW_MISC_AM335X_WDT_H
#define HW_MISC_AM335X_WDT_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AM335X_WDT "am335x-wdt"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xWdtState, AM335X_WDT)

struct AM335xWdtState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;

    /*
     * Per-register storage. WIDR (0x00, read-only revision), WDST (0x14,
     * WD_SYSSTATUS) and WWPS (0x3C, write-posted status) have no backing
     * field here: they are synthesized on every read (see am335x_wdt.c).
     */
    uint32_t wdsc;          /* WD_SYSCONFIG (0x10) */
    uint32_t wisr;          /* WISR         (0x24) */
    uint32_t wier;          /* WIER         (0x28) */
    uint32_t wclr;          /* WCLR         (0x2C) */
    uint32_t wcrr;          /* WCRR         (0x30) */
    uint32_t wldr;          /* WLDR         (0x34) */
    uint32_t wtgr;          /* WTGR         (0x38) */
    uint32_t wdly;          /* WDLY         (0x44) */
    uint32_t wspr;          /* WSPR         (0x48) */
    uint32_t wirqstatraw;   /* WIRQSTATRAW  (0x50) */
    uint32_t wirqstat;      /* WIRQSTAT     (0x54) */
    uint32_t wirqwakeen;    /* WIRQWAKEEN   (0x58) */
};

#endif /* HW_MISC_AM335X_WDT_H */
