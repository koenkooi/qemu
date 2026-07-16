/*
 * TI AM335x EMIF (External Memory Interface / DDR controller) emulation.
 *
 * Reference: TRM spruh73q, chapter 7.3 "EMIF", base 0x4C000000.
 *
 * A minimal register model: the AM335x SPL (MLO) programs the EMIF while
 * bringing up DDR3, but the actual DRAM is backed by QEMU's plain RAM at
 * 0x80000000, so nothing here needs to time or store data. This model is a
 * flat read/write backing store plus two synthesized read-only registers
 * (EMIF_MOD_ID_REV and EMIF_STATUS); see am335x_emif.c.
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

#ifndef HW_MISC_AM335X_EMIF_H
#define HW_MISC_AM335X_EMIF_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AM335X_EMIF "am335x-emif"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xEmifState, AM335X_EMIF)

/*
 * Size of the modelled register window. The DT reg is <0x4c000000
 * 0x1000000> (16MB), but every EMIF register the SPL or Linux touches lives
 * in the first ~0x310 bytes; 4KB backs all of them with margin.
 */
#define AM335X_EMIF_SIZE 0x1000

struct AM335xEmifState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;

    /*
     * Flat backing store for the register window, allocated at realize.
     * Reads and writes hit this array directly except for the synthesized
     * read-only registers handled in am335x_emif.c.
     */
    uint32_t *regs;
};

#endif /* HW_MISC_AM335X_EMIF_H */
