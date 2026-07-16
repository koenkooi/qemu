/*
 * TI DP83867 Gigabit RGMII Ethernet PHY emulation
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

#ifndef HW_NET_DP83867_PHY_H
#define HW_NET_DP83867_PHY_H

#include "qom/object.h"
#include "hw/sysbus.h"

#define TYPE_DP83867_PHY "dp83867-phy"
OBJECT_DECLARE_SIMPLE_TYPE(DP83867PhyState, DP83867_PHY)

typedef struct DP83867PhyState {
    SysBusDevice parent_obj;

    /* Clause-22 registers that the phylib/dp83867 probe path reads or writes.
     * The read-only ID/status/extended-status registers are synthesised in
     * dp83867_phy_read() rather than stored. */
    uint16_t bmcr;        /* reg 0  BMCR    */
    uint16_t bmsr;        /* reg 1  BMSR    */
    uint16_t anar;        /* reg 4  ANAR (advertised 10/100 abilities)      */
    uint16_t ctrl1000;    /* reg 9  1000BASE-T control (advertised)         */
    uint16_t phyctrl;     /* reg 0x10 PHYCR (vendor)                        */
    uint16_t physts;      /* reg 0x11 PHYSTS (vendor: resolved link/speed)  */
    uint16_t micr;        /* reg 0x12 MICR (vendor interrupt enable)        */
    uint16_t cfg2;        /* reg 0x14 CFG2 (vendor)                         */
    uint16_t cfg3;        /* reg 0x1e CFG3 (vendor)                         */
    uint16_t mmd_ctrl;    /* reg 13 REGCR: MMD indirect function + devad    */
    uint16_t mmd_addr;    /* reg 14 ADDAR: latched MMD register address     */

    uint16_t ints;
    uint16_t int_mask;
    qemu_irq irq;
    bool link_down;
} DP83867PhyState;

void dp83867_phy_update_link(DP83867PhyState *s, bool link_down);
void dp83867_phy_reset(DP83867PhyState *s);
uint16_t dp83867_phy_read(DP83867PhyState *s, int reg);
void dp83867_phy_write(DP83867PhyState *s, int reg, uint16_t val);

#endif /* HW_NET_DP83867_PHY_H */
