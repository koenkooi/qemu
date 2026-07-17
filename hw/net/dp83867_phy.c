/*
 * TI DP83867 Gigabit RGMII Ethernet PHY emulation
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * A functional MDIO (IEEE 802.3 clause 22) model of the Texas Instruments
 * DP83867 Gigabit RGMII PHY, covering exactly what the in-tree Linux
 * "dp83867" driver (drivers/net/phy/dp83867.c) and phylib touch to match,
 * probe and bring the link up at 1000Mbit/full duplex:
 *
 *  - The PHY identifier registers PHYIDR1/PHYIDR2 (regs 2/3) report
 *    DP83867_PHY_ID (0x2000a231, phy_id_mask 0xfffffff0), so phylib's
 *    ID-register autoprobe binds the dp83867 driver rather than the generic
 *    one.  (The AM335x davinci_mdio bus scan reads these to identify the PHY.
 *    Used by two boards via the shared CPSW "gigabit-phy" property: the
 *    Seeed BeagleBone Green Eco, whose DT names "ti,dp83867" outright -- the
 *    strongest evidence for this chip in the family -- and the SanCloud
 *    BeagleBone Enhanced, whose DT gives its PHY no explicit compatible
 *    string, so the chip there is inferred by analogy to Green Eco's,
 *    not confirmed from Sancloud's own BOM/schematic.)
 *  - BMCR/BMSR (regs 0/1): autonegotiation completes immediately and the
 *    status register reports link up, autoneg-complete, autoneg-able and
 *    "extended status present" (so phylib reads reg 15 for gigabit ability).
 *  - The 1000BASE-T control/status/extended-status registers (regs 9/10/15)
 *    advertise and resolve a 1000BASE-T full-duplex link, so genphy's link
 *    resolution agrees with the vendor PHYSTS override below.
 *  - The vendor PHYSTS register (reg 0x11) reports 1000Mbit/full/link, which
 *    is what dp83867_read_status() uses to set phydev->speed = SPEED_1000.
 *  - The MMD indirect access registers (regs 13/14, REGCR/ADDAR): the
 *    dp83867 config path touches several extended (MMDX) registers
 *    (RGMIICTL, STRAP_STS2, IO_MUX_CFG, ...) via clause-22 indirect access.
 *    None of the values it reads back there is required to be non-zero for a
 *    clean probe -- the STRAP_FLD restore and the io-impedance write are both
 *    conditional and are skipped when the reads return 0 -- so the indirect
 *    data path reads as zero and absorbs writes, matching this project's
 *    "clean, real-driver-satisfying attach, not full protocol realism" bar.
 *  - PHYCR/CFG2/CFG3/MICR (regs 0x10/0x14/0x1e/0x12) are backed so the
 *    driver's read-modify-writes in config_init()/config_intr() round-trip,
 *    and the reset control register (reg 0x1f, SW_RESET/SW_RESTART) triggers
 *    a register reset.
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

#include "qemu/osdep.h"
#include "hw/net/dp83867_phy.h"
#include "hw/net/mii.h"
#include "hw/irq.h"
#include "hw/resettable.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "trace.h"

/* PHY identifier: phylib reads PHYIDR1 into bits [31:16], PHYIDR2 into
 * [15:0]; the dp83867 driver matches (id & 0xfffffff0) == 0x2000a230. */
#define DP83867_PHYID1          0x2000
#define DP83867_PHYID2          0xa231

/* Vendor register numbers (MII_DP83867_* in the kernel driver). */
#define DP83867_REG_PHYCR       0x10
#define DP83867_REG_PHYSTS      0x11
#define DP83867_REG_MICR        0x12
#define DP83867_REG_ISR         0x13
#define DP83867_REG_CFG2        0x14
#define DP83867_REG_CFG3        0x1e
#define DP83867_REG_CTRL        0x1f

/* CTRL (0x1f) self-clearing reset bits. */
#define DP83867_CTRL_SW_RESET   (1 << 15)
#define DP83867_CTRL_SW_RESTART (1 << 14)

/* PHYSTS (0x11) resolved link/speed/duplex bits. */
#define DP83867_PHYSTS_1000     (1 << 15)
#define DP83867_PHYSTS_100      (1 << 14)
#define DP83867_PHYSTS_DUPLEX   (1 << 13)
#define DP83867_PHYSTS_LINK     (1 << 10)

/* REGCR (reg 13) function field [15:14]: 00 = address, non-zero = data. */
#define DP83867_MMD_FN_MASK     0xc000

/* MICR/ISR interrupt bits we drive on a link change (cosmetic: phylib polls
 * this PHY, irq=POLL, so the line is wired to a no-op in the CPSW model). */
#define DP83867_IRQ_AN_COMP     (1 << 11)   /* MII_DP83867_MICR_AUTONEG_COMP */
#define DP83867_IRQ_LINK_CHNG   (1 << 10)   /* MII_DP83867_MICR_LINK_STS_CHNG */

static void dp83867_phy_update_irq(DP83867PhyState *s)
{
    qemu_set_irq(s->irq, !!(s->ints & s->int_mask));
}

uint16_t dp83867_phy_read(DP83867PhyState *s, int reg)
{
    uint16_t val;

    switch (reg) {
    case MII_BMCR:
        val = s->bmcr;
        break;
    case MII_BMSR:
        val = s->bmsr;
        break;
    case MII_PHYID1:
        val = DP83867_PHYID1;
        break;
    case MII_PHYID2:
        val = DP83867_PHYID2;
        break;
    case MII_ANAR:
        val = s->anar;
        break;
    case MII_ANLPAR:
        /* Link partner advertises all 10/100 abilities plus pause. */
        val = MII_ANLPAR_ACK | MII_ANLPAR_PAUSE | MII_ANLPAR_TXFD |
              MII_ANLPAR_TX | MII_ANLPAR_10FD | MII_ANLPAR_10 |
              MII_ANLPAR_CSMACD;
        break;
    case MII_ANER:
        val = MII_ANER_NWAY;
        break;
    case MII_CTRL1000:
        val = s->ctrl1000;
        break;
    case MII_STAT1000:
        /* Link partner is 1000BASE-T full-duplex capable; receivers OK. */
        val = s->link_down ? 0 :
              (MII_STAT1000_FULL | MII_STAT1000_LOK | MII_STAT1000_ROK);
        break;
    case MII_EXTSTAT:
        val = MII_EXTSTAT_1000T_FD | MII_EXTSTAT_1000T_HD;
        break;
    case MII_MDDACR:   /* REGCR */
        val = s->mmd_ctrl;
        break;
    case MII_MDDAADR:  /* ADDAR: address phase reads the latched address; the
                        * data phase reads 0 (see file comment). */
        val = (s->mmd_ctrl & DP83867_MMD_FN_MASK) ? 0 : s->mmd_addr;
        break;
    case DP83867_REG_PHYCR:
        val = s->phyctrl;
        break;
    case DP83867_REG_PHYSTS:
        val = s->physts;
        break;
    case DP83867_REG_MICR:
        val = s->micr;
        break;
    case DP83867_REG_ISR:
        /* Reading the interrupt status register acknowledges it. */
        val = s->ints;
        s->ints = 0;
        dp83867_phy_update_irq(s);
        break;
    case DP83867_REG_CFG2:
        val = s->cfg2;
        break;
    case DP83867_REG_CFG3:
        val = s->cfg3;
        break;
    case DP83867_REG_CTRL:
        val = 0;   /* SW_RESET/SW_RESTART are self-clearing */
        break;
    default:
        /* Unmodelled register: reads as zero, like an unwritten strap. */
        val = 0;
        break;
    }

    trace_dp83867_phy_read(val, reg);
    return val;
}

void dp83867_phy_write(DP83867PhyState *s, int reg, uint16_t val)
{
    trace_dp83867_phy_write(val, reg);

    switch (reg) {
    case MII_BMCR:
        if (val & MII_BMCR_RESET) {
            dp83867_phy_reset(s);
        } else {
            s->bmcr = val & (MII_BMCR_LOOPBACK | MII_BMCR_SPEED100 |
                             MII_BMCR_SPEED1000 | MII_BMCR_AUTOEN |
                             MII_BMCR_PDOWN | MII_BMCR_ISOLATE |
                             MII_BMCR_ANRESTART | MII_BMCR_FD);
            /* Autonegotiation completes immediately in this model. */
            if ((val & MII_BMCR_AUTOEN) && !s->link_down) {
                s->bmsr |= MII_BMSR_AN_COMP;
            }
        }
        break;
    case MII_ANAR:
        s->anar = val;
        break;
    case MII_CTRL1000:
        s->ctrl1000 = val;
        break;
    case MII_MDDACR:   /* REGCR: latch MMD function + devad */
        s->mmd_ctrl = val;
        break;
    case MII_MDDAADR:  /* ADDAR: address phase latches the addr; data absorbed */
        if (!(s->mmd_ctrl & DP83867_MMD_FN_MASK)) {
            s->mmd_addr = val;
        }
        break;
    case DP83867_REG_PHYCR:
        s->phyctrl = val;
        break;
    case DP83867_REG_MICR:
        s->micr = val;
        s->int_mask = val;
        dp83867_phy_update_irq(s);
        break;
    case DP83867_REG_CFG2:
        s->cfg2 = val;
        break;
    case DP83867_REG_CFG3:
        s->cfg3 = val;
        break;
    case DP83867_REG_CTRL:
        /* SW_RESET reloads defaults; SW_RESTART just re-runs autoneg, which
         * is instantaneous here, so both collapse to a register reset. */
        if (val & (DP83867_CTRL_SW_RESET | DP83867_CTRL_SW_RESTART)) {
            dp83867_phy_reset(s);
        }
        break;
    default:
        /* Absorb writes to read-only / unmodelled registers. */
        break;
    }
}

void dp83867_phy_update_link(DP83867PhyState *s, bool link_down)
{
    s->link_down = link_down;

    if (link_down) {
        trace_dp83867_phy_update_link("down");
        s->bmsr &= ~(MII_BMSR_AN_COMP | MII_BMSR_LINK_ST);
        s->physts = 0;
        s->ints |= DP83867_IRQ_LINK_CHNG;
    } else {
        trace_dp83867_phy_update_link("up");
        s->bmsr |= MII_BMSR_AN_COMP | MII_BMSR_LINK_ST;
        /* Resolved link: 1000BASE-T, full duplex, link good. */
        s->physts = DP83867_PHYSTS_1000 | DP83867_PHYSTS_DUPLEX |
                    DP83867_PHYSTS_LINK;
        s->ints |= DP83867_IRQ_LINK_CHNG | DP83867_IRQ_AN_COMP;
    }
    dp83867_phy_update_irq(s);
}

void dp83867_phy_reset(DP83867PhyState *s)
{
    trace_dp83867_phy_reset();

    /* Default to 1000BASE-T full-duplex with autoneg enabled. */
    s->bmcr = MII_BMCR_AUTOEN | MII_BMCR_SPEED1000 | MII_BMCR_FD;
    s->bmsr = MII_BMSR_100TX_FD | MII_BMSR_100TX_HD |
              MII_BMSR_10T_FD | MII_BMSR_10T_HD |
              MII_BMSR_EXTSTAT | MII_BMSR_AUTONEG | MII_BMSR_EXTCAP;
    s->anar = MII_ANAR_TXFD | MII_ANAR_TX | MII_ANAR_10FD |
              MII_ANAR_10 | MII_ANAR_CSMACD;
    s->ctrl1000 = MII_CTRL1000_FULL | MII_CTRL1000_HALF;
    s->phyctrl = 0;
    s->physts = 0;
    s->micr = 0;
    s->cfg2 = 0;
    s->cfg3 = 0;
    s->mmd_ctrl = 0;
    s->mmd_addr = 0;
    s->int_mask = 0;
    s->ints = 0;
    dp83867_phy_update_link(s, s->link_down);
}

static void dp83867_phy_reset_hold(Object *obj, ResetType type)
{
    DP83867PhyState *s = DP83867_PHY(obj);

    dp83867_phy_reset(s);
}

static void dp83867_phy_init(Object *obj)
{
    DP83867PhyState *s = DP83867_PHY(obj);

    qdev_init_gpio_out(DEVICE(s), &s->irq, 1);
}

static const VMStateDescription vmstate_dp83867_phy = {
    .name = "dp83867-phy",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(bmcr, DP83867PhyState),
        VMSTATE_UINT16(bmsr, DP83867PhyState),
        VMSTATE_UINT16(anar, DP83867PhyState),
        VMSTATE_UINT16(ctrl1000, DP83867PhyState),
        VMSTATE_UINT16(phyctrl, DP83867PhyState),
        VMSTATE_UINT16(physts, DP83867PhyState),
        VMSTATE_UINT16(micr, DP83867PhyState),
        VMSTATE_UINT16(cfg2, DP83867PhyState),
        VMSTATE_UINT16(cfg3, DP83867PhyState),
        VMSTATE_UINT16(mmd_ctrl, DP83867PhyState),
        VMSTATE_UINT16(mmd_addr, DP83867PhyState),
        VMSTATE_UINT16(ints, DP83867PhyState),
        VMSTATE_UINT16(int_mask, DP83867PhyState),
        VMSTATE_BOOL(link_down, DP83867PhyState),
        VMSTATE_END_OF_LIST()
    }
};

static void dp83867_phy_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    rc->phases.hold = dp83867_phy_reset_hold;
    dc->vmsd = &vmstate_dp83867_phy;
}

static const TypeInfo types[] = {
    {
        .name          = TYPE_DP83867_PHY,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(DP83867PhyState),
        .instance_init = dp83867_phy_init,
        .class_init    = dp83867_phy_class_init,
    }
};

DEFINE_TYPES(types)
