/*
 * TI AM335x Clock Module / PRCM (PRM+CM) emulation.
 *
 * Reference: TRM spruh73q, chapter "8 Clock Module" and "8.1.12 CM_PER
 * Registers" / "8.1.13 CM_WKUP Registers" / etc, base 0x44E00000.
 *
 * This model implements just enough of the CLKCTRL / DPLL IDLEST behaviour
 * for the Linux ti-sysc and ti-clkctrl drivers to see modules go
 * "functional" once MODULEMODE is set to ENABLE, and DPLLs report locked,
 * so module functional clocks (e.g. the DMTIMERs) can be enabled and the
 * kernel clockevent can start ticking. All other registers are plain
 * read-back-what-was-written storage.
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

#ifndef HW_MISC_AM335X_PRCM_H
#define HW_MISC_AM335X_PRCM_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AM335X_PRCM "am335x-prcm"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xPrcmState, AM335X_PRCM)

#define AM335X_PRCM_MMIO_SIZE 0x2000

struct AM335xPrcmState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;

    /* Flat backing store for the whole 8KB window, indexed by offset >> 2.
     * Reads are computed from this store (with the CLKCTRL/DPLL IDLEST
     * transforms below); writes just store the raw 32-bit value. */
    uint32_t regs[AM335X_PRCM_MMIO_SIZE / 4];
};

#endif /* HW_MISC_AM335X_PRCM_H */
