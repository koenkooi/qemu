/*
 * TI AM335x CPSW 3-port Gigabit Ethernet switch subsystem emulation.
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

#ifndef HW_NET_AM335X_CPSW_H
#define HW_NET_AM335X_CPSW_H

#include "hw/sysbus.h"
#include "hw/irq.h"
#include "net/net.h"
#include "hw/net/lan9118_phy.h"
#include "qom/object.h"

#define TYPE_AM335X_CPSW "am335x-cpsw"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xCpswState, AM335X_CPSW)

/* Size of the register window (SS/host/slaves/CPDMA/stateram/ALE/sliver/
 * MDIO/WR) and of the on-chip descriptor SRAM, TRM spruh73q ch.14. */
#define AM335X_CPSW_REG_SIZE   0x2000
#define AM335X_CPSW_BD_SIZE    0x2000   /* CPSW_BD_RAM_SIZE, 8KB */
#define AM335X_CPSW_MMIO_SIZE  0x8000   /* full target-module window */

#define AM335X_CPSW_CHANNELS   8        /* cpdma_channels = <8> in the DTB */

struct AM335xCpswState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion container;   /* covers the whole 0x8000 window */
    MemoryRegion iomem;       /* register file, offset 0x0000..0x1FFF */
    MemoryRegion bdram;       /* on-chip descriptor SRAM, offset 0x2000 */
    uint8_t *bd;              /* host pointer into bdram */

    /* INTC outputs: 0=rx_thresh(40) 1=rx(41) 2=tx(42) 3=misc(43). */
    qemu_irq irq[4];

    NICState *nic;
    NICConf conf;
    Lan9118PhyState mii;      /* MDIO PHY at bus address 0 (LAN8710A slot) */
    IRQState mii_irq;

    /* Guest-physical base of the descriptor SRAM (register base + 0x2000). */
    uint32_t dma_desc_base;

    /* Flat backing store for the register file (indexed by offset >> 2). */
    uint32_t regs[AM335X_CPSW_REG_SIZE / 4];

    /* ALE table entries (3 x 32-bit each), backed so read-back works. */
    uint32_t ale_table[1024][3];

    /* CPDMA channel engine state. */
    uint32_t tx_head[AM335X_CPSW_CHANNELS];
    uint32_t rx_head[AM335X_CPSW_CHANNELS];
    uint32_t tx_pending[AM335X_CPSW_CHANNELS];
    uint32_t rx_pending[AM335X_CPSW_CHANNELS];
    uint32_t tx_int_mask;
    uint32_t rx_int_mask;
    bool tx_enabled;
    bool rx_enabled;

    /* CPTS: a single pending time-stamp-push event drives the misc IRQ. */
    bool cpts_ts_pend;
};

#endif /* HW_NET_AM335X_CPSW_H */
