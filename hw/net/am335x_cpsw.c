/*
 * TI AM335x CPSW 3-port Gigabit Ethernet switch subsystem emulation.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * A functional model of the AM335x Common Platform Ethernet Switch
 * (TRM spruh73q ch.14, base 0x4A100000) covering exactly what the built-in
 * Linux "cpsw-switch" (drivers/net/ethernet/ti/cpsw_new.c) and
 * "davinci_mdio" drivers touch on the unmodified BeagleBone Black DTB:
 *
 *  - CPSW_SS version register reporting CPSW_VERSION_2 (0x19010c), so the
 *    driver recognises the subsystem instead of logging "unknown version".
 *  - The CPDMA descriptor-ring engine.  The AM335x CPDMA descriptor pool
 *    lives in the CPSW's own on-chip 8KB SRAM (CPSW2_BD_OFFSET = 0x2000;
 *    cpsw_new.c passes ss_res->start + 0x2000 as desc_mem_phys), so the
 *    descriptors are modelled as a plain RAM sub-region the guest writes
 *    directly; only packet payloads are DMAed to/from guest DRAM.
 *  - The ALE (address lookup engine) as a register file with a backed
 *    table array (no real switching is needed for a single external port).
 *  - The two slave "sliver" MACs (CPGMAC_SL): self-clearing soft-reset and
 *    a MACSTATUS that always reports the port idle.
 *  - The davinci MDIO controller with a single PHY at bus address 0, so
 *    phylib finds the PHY and sees link up.  Which PHY sits there is a
 *    per-instance choice (the "gigabit-phy" property): the default 10/100
 *    lan9118_phy (the on-board LAN8710A slot of the MII boards), or a TI
 *    DP83867 Gigabit RGMII PHY for RGMII boards (SanCloud Enhanced).
 *  - The wrapper (WR) interrupt-enable registers that gate the four INTC
 *    outputs (rx_thresh/rx/tx/misc = lines 40..43).
 *
 * Register offsets and the descriptor layout are taken from the in-tree
 * kernel driver headers (cpsw_priv.h, cpsw_sl.c, davinci_cpdma.c,
 * davinci_mdio.c, cpsw_ale.c), not guessed.
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
#include "qapi/error.h"
#include "hw/net/am335x_cpsw.h"
#include "hw/net/mii.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/dma.h"
#include "net/eth.h"
#include "trace.h"

/* Description string reported on the link/activity trace events, mirroring
 * the "beaglebone-*" naming TYPE_LED instances use (hw/arm/beaglebone.c) so
 * a host-side relay can key off it the same way. Only one external port is
 * modelled, so this is a fixed literal rather than a per-instance property. */
#define AM335X_CPSW_TRACE_DESC "beaglebone-eth0"

/* --- CPSW subsystem (SS) block, CPSW2_* offsets (cpsw_priv.h) --------- */
#define CPSW_SS_IDVER          0x000   /* must read CPSW_VERSION_2 */
#define CPSW_SS_CONTROL        0x004
#define CPSW_SS_SOFT_RESET     0x008   /* self-clearing bit0 */
#define CPSW_VERSION_2         0x19010c

/* --- CPDMA block, base CPSW2_CPDMA_OFFSET = 0x800 (davinci_cpdma.c) --- */
#define CPDMA_BASE             0x800
#define CPDMA_TXCONTROL        (CPDMA_BASE + 0x04)
#define CPDMA_TXTEARDOWN       (CPDMA_BASE + 0x08)
#define CPDMA_RXCONTROL        (CPDMA_BASE + 0x14)
#define CPDMA_RXTEARDOWN       (CPDMA_BASE + 0x18)
#define CPDMA_SOFTRESET        (CPDMA_BASE + 0x1c)   /* self-clearing bit0 */
#define CPDMA_DMASTATUS        (CPDMA_BASE + 0x24)
#define CPDMA_TXINTSTATRAW     (CPDMA_BASE + 0x80)
#define CPDMA_TXINTSTATMASKED  (CPDMA_BASE + 0x84)
#define CPDMA_TXINTMASKSET     (CPDMA_BASE + 0x88)
#define CPDMA_TXINTMASKCLEAR   (CPDMA_BASE + 0x8c)
#define CPDMA_MACEOIVECTOR     (CPDMA_BASE + 0x94)
#define CPDMA_RXINTSTATRAW     (CPDMA_BASE + 0xa0)
#define CPDMA_RXINTSTATMASKED  (CPDMA_BASE + 0xa4)
#define CPDMA_RXINTMASKSET     (CPDMA_BASE + 0xa8)
#define CPDMA_RXINTMASKCLEAR   (CPDMA_BASE + 0xac)

#define CPDMA_TEARDOWN_VALUE   0xfffffffc

/* Descriptor mode bits (davinci_cpdma.c). */
#define CPDMA_DESC_SOP         (1u << 31)
#define CPDMA_DESC_EOP         (1u << 30)
#define CPDMA_DESC_OWNER       (1u << 29)
#define CPDMA_DESC_EOQ         (1u << 28)
#define CPDMA_TO_PORT_SHIFT    16
#define CPDMA_DESC_LEN_MASK    0x7ff

/* --- CPDMA state RAM, base CPSW2_STATERAM_OFFSET = 0xa00 -------------- */
#define STATERAM_BASE          0xa00
#define STATERAM_TXHDP         (STATERAM_BASE + 0x00)
#define STATERAM_RXHDP         (STATERAM_BASE + 0x20)
#define STATERAM_TXCP          (STATERAM_BASE + 0x40)
#define STATERAM_RXCP          (STATERAM_BASE + 0x60)

/* --- CPTS time-sync, base CPSW2_CPTS_OFFSET = 0xc00 (cpts.h) ---------
 * Modelled just enough to answer the driver's periodic timestamp push so
 * it does not log "cpts: obtain a time stamp timeout"; a push raises the
 * misc IRQ, the driver pops the event and the completion fires. */
#define CPTS_BASE              0xc00
#define CPTS_IDVER             (CPTS_BASE + 0x00)
#define CPTS_CONTROL           (CPTS_BASE + 0x04)
#define CPTS_TS_PUSH           (CPTS_BASE + 0x0c)
#define CPTS_INTSTAT_RAW       (CPTS_BASE + 0x20)
#define CPTS_INTSTAT_MASKED    (CPTS_BASE + 0x24)
#define CPTS_INT_ENABLE        (CPTS_BASE + 0x28)
#define CPTS_EVENT_POP         (CPTS_BASE + 0x30)
#define CPTS_EVENT_LOW         (CPTS_BASE + 0x34)
#define CPTS_EVENT_HIGH        (CPTS_BASE + 0x38)
#define CPTS_TS_PUSH_BIT       (1u << 0)
#define CPTS_TS_PEND           (1u << 0)
#define CPTS_EVENT_POP_BIT     (1u << 0)

/* --- ALE, base CPSW2_ALE_OFFSET = 0xd00 (cpsw_ale.c) ----------------- */
#define ALE_BASE               0xd00
#define ALE_IDVER              (ALE_BASE + 0x00)
#define ALE_TABLE_CONTROL      (ALE_BASE + 0x20)
#define ALE_TABLE              (ALE_BASE + 0x34)   /* 3 words */
#define ALE_TABLE_WRITE        (1u << 31)
#define ALE_TABLE_INDEX_MASK   0x3ff

/* --- Slave slivers (CPGMAC_SL), CPSW2_SLIVER_OFFSET = 0xd80, size 0x40 */
#define SLIVER0_BASE           0xd80
#define SLIVER1_BASE           0xdc0
#define SLIVER_MACSTATUS       0x08   /* bit31 = PN_IDLE */
#define SLIVER_SOFT_RESET      0x0c   /* self-clearing bit0 */
#define SLIVER_STATUS_PN_IDLE  (1u << 31)

/* --- davinci MDIO, base 0x1000 within the window (davinci_mdio.c) ---- */
#define MDIO_BASE              0x1000
#define MDIO_VERSION           (MDIO_BASE + 0x00)
#define MDIO_CONTROL           (MDIO_BASE + 0x04)
#define MDIO_ALIVE             (MDIO_BASE + 0x08)
#define MDIO_LINK              (MDIO_BASE + 0x0c)
#define MDIO_USERACCESS0       (MDIO_BASE + 0x80)
#define MDIO_USERPHYSEL0       (MDIO_BASE + 0x84)
#define MDIO_CONTROL_IDLE      (1u << 31)
#define MDIO_USERACCESS_GO     (1u << 31)
#define MDIO_USERACCESS_WRITE  (1u << 30)
#define MDIO_USERACCESS_ACK    (1u << 29)
#define MDIO_USERACCESS_DATA   0xffff

/* The single PHY we model sits at MDIO bus address 0. */
#define AM335X_CPSW_PHY_ADDR   0

/* --- Wrapper (WR), CPSW2_WR_OFFSET = 0x1200 (cpsw_priv.h) ------------- */
#define WR_BASE                0x1200
#define WR_IDVER               (WR_BASE + 0x00)
#define WR_SOFT_RESET          (WR_BASE + 0x04)  /* self-clearing bit0 */
#define WR_RX_THRESH_EN        (WR_BASE + 0x10)
#define WR_RX_EN               (WR_BASE + 0x14)
#define WR_TX_EN               (WR_BASE + 0x18)
#define WR_MISC_EN             (WR_BASE + 0x1c)

/* Longest frame the model will assemble/deliver (VLAN frame + slack). */
#define AM335X_CPSW_FRAME_MAX  2048

/* ------------------------------------------------------------------- */
/* On-chip descriptor SRAM helpers.  Descriptors are read/written in the
 * device's own modelled RAM; only payload buffers hit guest DRAM. */

static uint32_t cpsw_bd_ld(AM335xCpswState *s, uint32_t desc, int word)
{
    uint32_t off = desc - s->dma_desc_base + (word << 2);

    if (off + 4 > AM335X_CPSW_BD_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: descriptor 0x%08x outside BD RAM\n",
                      TYPE_AM335X_CPSW, desc);
        return 0;
    }
    return ldl_le_p(s->bd + off);
}

static void cpsw_bd_st(AM335xCpswState *s, uint32_t desc, int word,
                       uint32_t val)
{
    uint32_t off = desc - s->dma_desc_base + (word << 2);

    if (off + 4 > AM335X_CPSW_BD_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: descriptor 0x%08x outside BD RAM\n",
                      TYPE_AM335X_CPSW, desc);
        return;
    }
    stl_le_p(s->bd + off, val);
}

/* CPDMA interrupt status is derived from the per-channel completion
 * counters: a channel raises its bit while it has completed, unacked
 * descriptors. */
static uint32_t cpsw_tx_raw(AM335xCpswState *s)
{
    uint32_t raw = 0;
    int ch;

    for (ch = 0; ch < AM335X_CPSW_CHANNELS; ch++) {
        if (s->tx_pending[ch]) {
            raw |= 1u << ch;
        }
    }
    return raw;
}

static uint32_t cpsw_rx_raw(AM335xCpswState *s)
{
    uint32_t raw = 0;
    int ch;

    for (ch = 0; ch < AM335X_CPSW_CHANNELS; ch++) {
        if (s->rx_pending[ch]) {
            raw |= 1u << ch;
        }
    }
    return raw;
}

/* The WR module gates the CPDMA interrupts onto the four INTC lines. */
static void cpsw_update_irq(AM335xCpswState *s)
{
    uint32_t rx_en = s->regs[WR_RX_EN >> 2];
    uint32_t tx_en = s->regs[WR_TX_EN >> 2];

    uint32_t misc_en = s->regs[WR_MISC_EN >> 2];
    bool misc = s->cpts_ts_pend &&
                (s->regs[CPTS_INT_ENABLE >> 2] & CPTS_TS_PEND) &&
                misc_en != 0;

    qemu_set_irq(s->irq[1], (cpsw_rx_raw(s) & s->rx_int_mask & rx_en) != 0);
    qemu_set_irq(s->irq[2], (cpsw_tx_raw(s) & s->tx_int_mask & tx_en) != 0);
    qemu_set_irq(s->irq[3], misc);
    /* rx_thresh (40) is never asserted by this model. */
}

/* ------------------------------------------------------------------- */
/* CPDMA transmit: walk the tx descriptor chain, DMA each payload out of
 * guest DRAM and send it, completing descriptors as we go. */
static void cpsw_tx_process(AM335xCpswState *s, int ch)
{
    uint8_t frame[AM335X_CPSW_FRAME_MAX];
    size_t framelen = 0;

    if (!s->tx_enabled) {
        return;
    }

    while (s->tx_head[ch]) {
        uint32_t desc = s->tx_head[ch];
        uint32_t mode = cpsw_bd_ld(s, desc, 3);
        uint32_t buffer, next, fraglen, done;

        if (!(mode & CPDMA_DESC_OWNER)) {
            break;   /* nothing new to transmit */
        }

        buffer = cpsw_bd_ld(s, desc, 1);
        next = cpsw_bd_ld(s, desc, 0);
        fraglen = mode & CPDMA_DESC_LEN_MASK;

        if (mode & CPDMA_DESC_SOP) {
            framelen = 0;
        }
        if (fraglen && framelen + fraglen <= sizeof(frame)) {
            dma_memory_read(&address_space_memory, buffer,
                            frame + framelen, fraglen,
                            MEMTXATTRS_UNSPECIFIED);
            framelen += fraglen;
        }

        /* Complete this descriptor: clear OWNER, flag end-of-queue if it
         * is the last one, record the completion pointer and count it. */
        done = mode & ~CPDMA_DESC_OWNER;
        if (next == 0) {
            done |= CPDMA_DESC_EOQ;
        }
        cpsw_bd_st(s, desc, 3, done);
        s->regs[(STATERAM_TXCP + (ch << 2)) >> 2] = desc;
        s->tx_pending[ch]++;

        if (mode & CPDMA_DESC_EOP) {
            trace_am335x_cpsw_tx(AM335X_CPSW_TRACE_DESC, framelen);
            qemu_send_packet(qemu_get_queue(s->nic), frame, framelen);
            framelen = 0;
        }

        s->tx_head[ch] = next;
    }

    cpsw_update_irq(s);
}

/* Teardown a channel: park the completion pointer at the teardown marker
 * the driver polls for, and drop any queued descriptors. */
static void cpsw_teardown(AM335xCpswState *s, int ch, bool is_rx)
{
    if (is_rx) {
        s->regs[(STATERAM_RXCP + (ch << 2)) >> 2] = CPDMA_TEARDOWN_VALUE;
        s->rx_head[ch] = 0;
        s->rx_pending[ch] = 0;
    } else {
        s->regs[(STATERAM_TXCP + (ch << 2)) >> 2] = CPDMA_TEARDOWN_VALUE;
        s->tx_head[ch] = 0;
        s->tx_pending[ch] = 0;
    }
    cpsw_update_irq(s);
}

static void cpsw_dma_soft_reset(AM335xCpswState *s)
{
    memset(s->tx_head, 0, sizeof(s->tx_head));
    memset(s->rx_head, 0, sizeof(s->rx_head));
    memset(s->tx_pending, 0, sizeof(s->tx_pending));
    memset(s->rx_pending, 0, sizeof(s->rx_pending));
    s->tx_int_mask = 0;
    s->rx_int_mask = 0;
    s->tx_enabled = false;
    s->rx_enabled = false;
    cpsw_update_irq(s);
}

/* ------------------------------------------------------------------- */
/* Single external MDIO PHY at bus address 0.  The board picks which model
 * via the "gigabit-phy" property (see cpsw_realize): the default 10/100
 * lan9118_phy, or a Gigabit TI DP83867 for RGMII boards.  These wrappers
 * dispatch to whichever one is instantiated so the rest of the model stays
 * PHY-agnostic. */
static uint16_t cpsw_phy_read(AM335xCpswState *s, int reg)
{
    return s->gigabit_phy ? dp83867_phy_read(&s->gmii, reg)
                          : lan9118_phy_read(&s->mii, reg);
}

static void cpsw_phy_write(AM335xCpswState *s, int reg, uint16_t val)
{
    if (s->gigabit_phy) {
        dp83867_phy_write(&s->gmii, reg, val);
    } else {
        lan9118_phy_write(&s->mii, reg, val);
    }
}

static void cpsw_phy_update_link(AM335xCpswState *s, bool link_down)
{
    if (s->gigabit_phy) {
        dp83867_phy_update_link(&s->gmii, link_down);
    } else {
        lan9118_phy_update_link(&s->mii, link_down);
    }
}

/* ------------------------------------------------------------------- */
/* MDIO USERACCESS protocol: complete each transaction synchronously so
 * the driver's "wait for GO to clear" poll returns immediately. */
static void cpsw_mdio_access(AM335xCpswState *s, uint32_t val)
{
    uint32_t phy = (val >> 16) & 0x1f;
    uint32_t reg = (val >> 21) & 0x1f;
    uint32_t result;

    if (!(val & MDIO_USERACCESS_GO)) {
        s->regs[MDIO_USERACCESS0 >> 2] = val;
        return;
    }

    if (phy == AM335X_CPSW_PHY_ADDR) {
        if (val & MDIO_USERACCESS_WRITE) {
            cpsw_phy_write(s, reg, val & MDIO_USERACCESS_DATA);
            result = MDIO_USERACCESS_ACK;
        } else {
            result = MDIO_USERACCESS_ACK |
                     (cpsw_phy_read(s, reg) & MDIO_USERACCESS_DATA);
        }
    } else {
        /* No PHY at this address: complete without ACK. */
        result = 0;
    }
    /* GO is cleared in the result so the poll succeeds. */
    s->regs[MDIO_USERACCESS0 >> 2] = result;
}

/* ------------------------------------------------------------------- */

static uint64_t cpsw_read(void *opaque, hwaddr offset, unsigned size)
{
    AM335xCpswState *s = opaque;

    switch (offset) {
    case CPSW_SS_IDVER:
        return CPSW_VERSION_2;
    case CPSW_SS_SOFT_RESET:
    case CPDMA_SOFTRESET:
    case WR_SOFT_RESET:
    case SLIVER0_BASE + SLIVER_SOFT_RESET:
    case SLIVER1_BASE + SLIVER_SOFT_RESET:
        return 0;   /* reset completes immediately */
    case CPDMA_TXINTSTATRAW:
        return cpsw_tx_raw(s);
    case CPDMA_TXINTSTATMASKED:
        return cpsw_tx_raw(s) & s->tx_int_mask;
    case CPDMA_RXINTSTATRAW:
        return cpsw_rx_raw(s);
    case CPDMA_RXINTSTATMASKED:
        return cpsw_rx_raw(s) & s->rx_int_mask;
    case CPDMA_DMASTATUS:
        return 0;
    case CPTS_IDVER:
        return 0x4e8a0104;   /* CPTS revision (cosmetic) */
    case CPTS_INTSTAT_RAW:
        return s->cpts_ts_pend ? CPTS_TS_PEND : 0;
    case CPTS_INTSTAT_MASKED:
        return (s->cpts_ts_pend && (s->regs[CPTS_INT_ENABLE >> 2] & CPTS_TS_PEND))
               ? CPTS_TS_PEND : 0;
    case ALE_IDVER:
        return 0x00000106;   /* ALE revision 1.6 (cosmetic) */
    case SLIVER0_BASE + SLIVER_MACSTATUS:
    case SLIVER1_BASE + SLIVER_MACSTATUS:
        return SLIVER_STATUS_PN_IDLE;   /* port idle, so wait_for_idle() ends */
    case MDIO_VERSION:
        return 0x00070101;   /* davinci mdio revision 1.1 (cosmetic) */
    case MDIO_CONTROL:
        /* State machine is always idle in this model. */
        return s->regs[MDIO_CONTROL >> 2] | MDIO_CONTROL_IDLE;
    case MDIO_ALIVE:
    case MDIO_LINK:
        return 1u << AM335X_CPSW_PHY_ADDR;   /* PHY present/link at addr 0 */
    case WR_IDVER:
        return 0x4EC80100;   /* wrapper/ti-sysc revision (cosmetic) */
    default:
        return s->regs[offset >> 2];
    }
}

static void cpsw_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    AM335xCpswState *s = opaque;
    uint32_t v = (uint32_t)value;
    int ch;

    switch (offset) {
    case CPSW_SS_SOFT_RESET:
    case CPDMA_SOFTRESET:
        if (v & 1) {
            cpsw_dma_soft_reset(s);
        }
        return;   /* self-clearing */
    case WR_SOFT_RESET:
    case SLIVER0_BASE + SLIVER_SOFT_RESET:
    case SLIVER1_BASE + SLIVER_SOFT_RESET:
        return;   /* self-clearing, read back as done */

    case CPDMA_TXCONTROL:
        s->tx_enabled = v & 1;
        s->regs[offset >> 2] = v;
        return;
    case CPDMA_RXCONTROL:
        s->rx_enabled = v & 1;
        s->regs[offset >> 2] = v;
        if (s->rx_enabled) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        return;

    case CPDMA_TXINTMASKSET:
        s->tx_int_mask |= v;
        cpsw_update_irq(s);
        return;
    case CPDMA_TXINTMASKCLEAR:
        s->tx_int_mask &= ~v;
        cpsw_update_irq(s);
        return;
    case CPDMA_RXINTMASKSET:
        s->rx_int_mask |= v;
        cpsw_update_irq(s);
        return;
    case CPDMA_RXINTMASKCLEAR:
        s->rx_int_mask &= ~v;
        cpsw_update_irq(s);
        return;
    case CPDMA_MACEOIVECTOR:
        /* End-of-interrupt: re-evaluate the gated INTC lines. */
        cpsw_update_irq(s);
        return;
    case CPDMA_TXTEARDOWN:
        cpsw_teardown(s, v & (AM335X_CPSW_CHANNELS - 1), false);
        return;
    case CPDMA_RXTEARDOWN:
        cpsw_teardown(s, v & (AM335X_CPSW_CHANNELS - 1), true);
        return;

    case STATERAM_TXHDP ... STATERAM_TXHDP + 0x1c:
        ch = (offset - STATERAM_TXHDP) >> 2;
        s->regs[offset >> 2] = v;
        s->tx_head[ch] = v;
        if (v) {
            cpsw_tx_process(s, ch);
        }
        return;
    case STATERAM_RXHDP ... STATERAM_RXHDP + 0x1c:
        ch = (offset - STATERAM_RXHDP) >> 2;
        s->regs[offset >> 2] = v;
        s->rx_head[ch] = v;
        if (v && s->rx_enabled) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        return;
    case STATERAM_TXCP ... STATERAM_TXCP + 0x1c:
        ch = (offset - STATERAM_TXCP) >> 2;
        s->regs[offset >> 2] = v;
        if (s->tx_pending[ch]) {
            s->tx_pending[ch]--;
        }
        cpsw_update_irq(s);
        return;
    case STATERAM_RXCP ... STATERAM_RXCP + 0x1c:
        ch = (offset - STATERAM_RXCP) >> 2;
        s->regs[offset >> 2] = v;
        if (s->rx_pending[ch]) {
            s->rx_pending[ch]--;
        }
        cpsw_update_irq(s);
        return;

    case ALE_TABLE_CONTROL: {
        uint32_t idx = v & ALE_TABLE_INDEX_MASK;
        int k;

        if (v & ALE_TABLE_WRITE) {
            for (k = 0; k < 3; k++) {
                s->ale_table[idx][k] = s->regs[(ALE_TABLE + (k << 2)) >> 2];
            }
        } else {
            for (k = 0; k < 3; k++) {
                s->regs[(ALE_TABLE + (k << 2)) >> 2] = s->ale_table[idx][k];
            }
        }
        s->regs[offset >> 2] = v & ~ALE_TABLE_WRITE;
        return;
    }

    case CPTS_TS_PUSH:
        /* Latch the current time as a PUSH event and raise the misc IRQ so
         * the driver's timestamp-push completion fires. */
        if (v & CPTS_TS_PUSH_BIT) {
            uint32_t ts = (uint32_t)qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->regs[CPTS_EVENT_LOW >> 2] = ts;
            s->regs[CPTS_EVENT_HIGH >> 2] = 0;   /* type CPTS_EV_PUSH, port 0 */
            s->cpts_ts_pend = true;
            cpsw_update_irq(s);
        }
        return;
    case CPTS_EVENT_POP:
        if (v & CPTS_EVENT_POP_BIT) {
            s->cpts_ts_pend = false;
            cpsw_update_irq(s);
        }
        return;

    case MDIO_USERACCESS0:
        cpsw_mdio_access(s, v);
        return;

    case WR_RX_THRESH_EN:
    case WR_RX_EN:
    case WR_TX_EN:
    case WR_MISC_EN:
        s->regs[offset >> 2] = v;
        cpsw_update_irq(s);
        return;

    default:
        s->regs[offset >> 2] = v;
        return;
    }
}

static const MemoryRegionOps cpsw_ops = {
    .read = cpsw_read,
    .write = cpsw_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* ------------------------------------------------------------------- */
/* Receive: place an incoming frame into the current rx descriptor's
 * buffer in guest DRAM, complete the descriptor and raise the rx IRQ. */
static bool cpsw_can_receive(NetClientState *nc)
{
    AM335xCpswState *s = qemu_get_nic_opaque(nc);
    uint32_t desc = s->rx_head[0];

    if (!s->rx_enabled || desc == 0) {
        return false;
    }
    return (cpsw_bd_ld(s, desc, 3) & CPDMA_DESC_OWNER) != 0;
}

static ssize_t cpsw_receive(NetClientState *nc, const uint8_t *buf,
                            size_t size)
{
    AM335xCpswState *s = qemu_get_nic_opaque(nc);
    const int ch = 0;
    uint32_t desc = s->rx_head[ch];
    uint32_t mode, buffer, buflen, next, newmode;
    size_t len = size;

    if (!s->rx_enabled || desc == 0) {
        return 0;   /* no posted buffer: ask the core to queue the frame */
    }

    mode = cpsw_bd_ld(s, desc, 3);
    if (!(mode & CPDMA_DESC_OWNER)) {
        return 0;
    }

    buffer = cpsw_bd_ld(s, desc, 1);
    buflen = cpsw_bd_ld(s, desc, 2) & 0xffff;
    next = cpsw_bd_ld(s, desc, 0);

    if (buflen && len > buflen) {
        len = buflen;
    }
    dma_memory_write(&address_space_memory, buffer, buf, len,
                     MEMTXATTRS_UNSPECIFIED);

    /* Report the frame as a complete single-buffer packet sourced from
     * slave port 1 (bits[18:16] = 1), so cpsw_rx_handler() routes it to
     * the port-1 netdev. */
    newmode = CPDMA_DESC_SOP | CPDMA_DESC_EOP |
              (1u << CPDMA_TO_PORT_SHIFT) | (len & CPDMA_DESC_LEN_MASK);
    if (next == 0) {
        newmode |= CPDMA_DESC_EOQ;
    }
    cpsw_bd_st(s, desc, 3, newmode);

    s->regs[(STATERAM_RXCP + (ch << 2)) >> 2] = desc;
    s->rx_head[ch] = next;
    s->rx_pending[ch]++;
    cpsw_update_irq(s);

    trace_am335x_cpsw_rx(AM335X_CPSW_TRACE_DESC, len);

    return size;
}

static void cpsw_set_link(NetClientState *nc)
{
    AM335xCpswState *s = qemu_get_nic_opaque(nc);

    trace_am335x_cpsw_link_status(AM335X_CPSW_TRACE_DESC, !nc->link_down);
    cpsw_phy_update_link(s, nc->link_down);
}

static NetClientInfo net_cpsw_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = cpsw_can_receive,
    .receive = cpsw_receive,
    .link_status_changed = cpsw_set_link,
};

/* ------------------------------------------------------------------- */

static void cpsw_reset(DeviceState *dev)
{
    AM335xCpswState *s = AM335X_CPSW(dev);

    memset(s->regs, 0, sizeof(s->regs));
    memset(s->ale_table, 0, sizeof(s->ale_table));
    memset(s->bd, 0, AM335X_CPSW_BD_SIZE);
    s->cpts_ts_pend = false;
    cpsw_dma_soft_reset(s);

    qemu_set_irq(s->irq[0], 0);
    qemu_set_irq(s->irq[1], 0);
    qemu_set_irq(s->irq[2], 0);
    qemu_set_irq(s->irq[3], 0);

    /* The embedded PHY resets (and reports link) via the reset tree; make
     * the reported link match the netdev peer. */
    trace_am335x_cpsw_link_status(AM335X_CPSW_TRACE_DESC,
                                  !qemu_get_queue(s->nic)->link_down);
    cpsw_phy_update_link(s, qemu_get_queue(s->nic)->link_down);
}

static void cpsw_mii_irq(void *opaque, int n, int level)
{
    /* The PHY interrupt line is unused: phylib polls link state. */
}

static void cpsw_realize(DeviceState *dev, Error **errp)
{
    AM335xCpswState *s = AM335X_CPSW(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    memory_region_init(&s->container, OBJECT(dev), TYPE_AM335X_CPSW,
                       AM335X_CPSW_MMIO_SIZE);
    memory_region_init_io(&s->iomem, OBJECT(dev), &cpsw_ops, s,
                          "am335x-cpsw-regs", AM335X_CPSW_REG_SIZE);
    memory_region_add_subregion(&s->container, 0, &s->iomem);
    memory_region_init_ram(&s->bdram, OBJECT(dev), "am335x-cpsw-bdram",
                           AM335X_CPSW_BD_SIZE, &error_fatal);
    memory_region_add_subregion(&s->container, AM335X_CPSW_REG_SIZE,
                                &s->bdram);
    s->bd = memory_region_get_ram_ptr(&s->bdram);

    sysbus_init_mmio(sbd, &s->container);
    for (i = 0; i < 4; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }

    /*
     * MDIO PHY at bus address 0.  A board that set "gigabit-phy" gets a TI
     * DP83867 Gigabit RGMII PHY; the default is the 10/100 lan9118_phy, so
     * unmodified boards (Black, White) are unaffected.  Only the selected
     * PHY is created and realized.
     */
    qemu_init_irq(&s->mii_irq, cpsw_mii_irq, s, 0);
    if (s->gigabit_phy) {
        object_initialize_child(OBJECT(s), "gmii", &s->gmii, TYPE_DP83867_PHY);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->gmii), errp)) {
            return;
        }
        qdev_connect_gpio_out(DEVICE(&s->gmii), 0, &s->mii_irq);
    } else {
        object_initialize_child(OBJECT(s), "mii", &s->mii, TYPE_LAN9118_PHY);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->mii), errp)) {
            return;
        }
        qdev_connect_gpio_out(DEVICE(&s->mii), 0, &s->mii_irq);
    }

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_cpsw_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static const VMStateDescription vmstate_cpsw = {
    .name = TYPE_AM335X_CPSW,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AM335xCpswState, AM335X_CPSW_REG_SIZE / 4),
        VMSTATE_UINT32_2DARRAY(ale_table, AM335xCpswState, 1024, 3),
        VMSTATE_UINT32_ARRAY(tx_head, AM335xCpswState, AM335X_CPSW_CHANNELS),
        VMSTATE_UINT32_ARRAY(rx_head, AM335xCpswState, AM335X_CPSW_CHANNELS),
        VMSTATE_UINT32_ARRAY(tx_pending, AM335xCpswState, AM335X_CPSW_CHANNELS),
        VMSTATE_UINT32_ARRAY(rx_pending, AM335xCpswState, AM335X_CPSW_CHANNELS),
        VMSTATE_UINT32(tx_int_mask, AM335xCpswState),
        VMSTATE_UINT32(rx_int_mask, AM335xCpswState),
        VMSTATE_BOOL(tx_enabled, AM335xCpswState),
        VMSTATE_BOOL(rx_enabled, AM335xCpswState),
        VMSTATE_BOOL(cpts_ts_pend, AM335xCpswState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property cpsw_properties[] = {
    DEFINE_NIC_PROPERTIES(AM335xCpswState, conf),
    DEFINE_PROP_UINT32("dma-desc-base", AM335xCpswState, dma_desc_base,
                       0x4A102000),
    /*
     * false (default): the 10/100 lan9118_phy, matching the on-board
     * LAN8710A of the MII-wired boards (Black, White) -- unchanged behaviour.
     * true: a TI DP83867 Gigabit RGMII PHY, for boards that mux the CPSW to
     * RGMII (SanCloud BeagleBone Enhanced, Green Eco).
     */
    DEFINE_PROP_BOOL("gigabit-phy", AM335xCpswState, gigabit_phy, false),
};

static void cpsw_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = cpsw_realize;
    device_class_set_legacy_reset(dc, cpsw_reset);
    dc->vmsd = &vmstate_cpsw;
    device_class_set_props(dc, cpsw_properties);
    dc->desc = "TI AM335x CPSW Ethernet switch";
}

static const TypeInfo cpsw_info = {
    .name = TYPE_AM335X_CPSW,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xCpswState),
    .class_init = cpsw_class_init,
};

static void cpsw_register_types(void)
{
    type_register_static(&cpsw_info);
}

type_init(cpsw_register_types)
