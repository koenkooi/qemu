/*
 * TI AM335x Control Module (System Control Module, SCM) emulation.
 *
 * Reference: TRM spruh73q, chapter 9 "Control Module", base 0x44E10000
 * (device_conf / scm_conf, 128KB window).
 *
 * This is a minimal model whose sole functional behaviour is to report
 * the on-board input crystal frequency via CONTROL_STATUS.SYSBOOT1, so
 * that the Linux clock tree derives the correct sys_clkin rate. The rest
 * of the window (pin-mux CONF registers, syscon fields used by pinctrl,
 * the CPSW PHY, RTC, DDR init, ...) is a plain read/write backing store,
 * which is what those syscon read-modify-write users expect.
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

#ifndef HW_MISC_AM335X_CONTROL_H
#define HW_MISC_AM335X_CONTROL_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AM335X_CONTROL "am335x-control"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xControlState, AM335X_CONTROL)

/* Size of the Control Module register window (TRM ch.9). */
#define AM335X_CONTROL_SIZE 0x20000

struct AM335xControlState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;

    /*
     * Flat backing store for the whole window, allocated at realize.
     * Reads and writes hit this array directly except for the few
     * synthesized/read-only registers handled in am335x_control.c
     * (CONTROL_STATUS, DEVICE_ID).
     */
    uint32_t *regs;
};

#endif /* HW_MISC_AM335X_CONTROL_H */
