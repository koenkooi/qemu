/*
 * TI AM335x Watchdog Timer (WDT1) emulation.
 *
 * Reference: TRM spruh73q, chapter 20.4 "Watchdog Timer Registers" /
 * Table 20-111, base 0x44E35000 (WDT1 / wd_timer1).
 *
 * Implements the full start/stop/reload/overflow lifecycle: an armed
 * watchdog that isn't reloaded before WDT_WCRR overflows genuinely resets
 * the guest (qemu_system_reset_request()), matching real silicon. See
 * am335x_wdt.c for the offset table (corrected against the TRM -- an
 * earlier version of this model had WISR..WWPS shifted 8 bytes high) and
 * the countdown/reload logic.
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

/* WSPR (0x48) start/stop key-sequence progress (TRM 20.4.3.8). */
typedef enum {
    AM335X_WDT_WSPR_IDLE,
    AM335X_WDT_WSPR_GOT_DISABLE1,  /* saw XXXX_AAAA, awaiting XXXX_5555 */
    AM335X_WDT_WSPR_GOT_ENABLE1,   /* saw XXXX_BBBB, awaiting XXXX_4444 */
} AM335xWdtSprState;

struct AM335xWdtState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    QEMUTimer *bite_timer;

    /*
     * Per-register storage. WIDR (0x00, read-only revision), WDST (0x14,
     * WD_SYSSTATUS) and WWPS (0x34, write-posted status) have no backing
     * field here: they are synthesized on every read (see am335x_wdt.c).
     * WCRR is likewise synthesized while running (derived from elapsed
     * QEMU_CLOCK_VIRTUAL time, same idiom as am335x_timer.c's TCRR) --
     * wcrr_stopped holds it while the counter isn't advancing.
     */
    uint32_t wdsc;          /* WD_SYSCONFIG (0x10) */
    uint32_t wisr;          /* WISR         (0x18) */
    uint32_t wier;          /* WIER         (0x1C) */
    uint32_t wclr;          /* WCLR         (0x24) */
    uint32_t wcrr_stopped;  /* WCRR         (0x28), latched value while stopped */
    uint32_t wldr;          /* WLDR         (0x2C) */
    uint32_t wtgr;          /* WTGR         (0x30) */
    uint32_t wdly;          /* WDLY         (0x44) */
    uint32_t wspr;          /* WSPR         (0x48), last raw value written */
    uint32_t wirqstatraw;   /* WIRQSTATRAW  (0x54) */
    uint32_t wirqstat;      /* WIRQSTAT     (0x58) */
    uint32_t wirqensetclr;  /* WIRQENSET/WIRQENCLR share one flat store (0x5C/0x60) */

    uint8_t spr_state;      /* AM335xWdtSprState */
    bool running;
    int64_t reload_ns;      /* QEMU_CLOCK_VIRTUAL time of the last (re)load */
};

#endif /* HW_MISC_AM335X_WDT_H */
