/*
 * TI AM335x Control Module (System Control Module, SCM) emulation.
 *
 * Reference: TRM spruh73q, chapter 9 "Control Module", base 0x44E10000
 * (device_conf / scm_conf, 128KB window).
 *
 * This is a minimal model whose functional behaviour is to report the
 * on-board input crystal frequency via CONTROL_STATUS.SYSBOOT1, so that
 * the Linux clock tree derives the correct sys_clkin rate, and to report
 * CPSW slave 0's MAC address via mac_id0_lo/hi (0x630/0x634) -- on real
 * silicon these are read-only, factory-EFUSE-backed registers that
 * drivers/net/ethernet/ti/cpsw-common.c's ti_cm_get_macid() reads as a
 * fallback when the DT has no local-mac-address; see
 * am335x_control_set_mac_id0(). The rest of the window (pin-mux CONF
 * registers, syscon fields used by pinctrl, the CPSW PHY, RTC, DDR init,
 * ...) is a plain read/write backing store, which is what those syscon
 * read-modify-write users expect.
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
     * (CONTROL_STATUS, DEVICE_ID, mac_id0_lo/hi).
     */
    uint32_t *regs;

    /*
     * CPSW slave 0's MAC address, mirrored into mac_id0_lo/hi on every
     * read. Set once by am335x_soc.c after the CPSW device realizes (its
     * NICConf macaddr is only finalized -- qemu_macaddr_default_if_unset()
     * -- at that point), and left untouched across resets, matching a
     * real EFUSE-backed register. Slave 1 (mac_id1_lo/hi, 0x638/0x63C) has
     * no backing NIC in this model and stays flat-zero.
     */
    uint8_t mac_id0[6];
};

void am335x_control_set_mac_id0(AM335xControlState *s, const uint8_t *mac);

#endif /* HW_MISC_AM335X_CONTROL_H */
