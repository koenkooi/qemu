/*
 * TI AM335x EDMA3 (Enhanced DMA) controller emulation.
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

#ifndef HW_DMA_AM335X_EDMA_H
#define HW_DMA_AM335X_EDMA_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"

#define TYPE_AM335X_EDMA "am335x-edma"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xEdmaState, AM335X_EDMA)

/* TPCC (Third-Party Channel Controller) MMIO window, TRM SPRUH73Q ch.11,
 * base 0x49000000, 64KB (DT target-module@49000000 ranges <0 0x49000000
 * 0x10000>). Covers global regs, the four 0x200-byte shadow regions at
 * 0x2000, and the 256-entry PaRAM at 0x4000. */
#define AM335X_EDMA_MMIO_SIZE   0x10000

/* 64 DMA channels / 256 PaRAM sets on AM335x (from CCCFG, see am335x_edma.c). */
#define AM335X_EDMA_NUM_CHANNELS 64
#define AM335X_EDMA_NUM_PARAM    256

struct AM335xEdmaState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;      /* TPCC register window @ 0x49000000, 64KB */

    /*
     * INTC outputs (TRM Table 6-1, DT interrupts <12 13 14>):
     *   irq[0] = EDMACOMPINT (12, transfer completion)
     *   irq[1] = EDMAMPERR   (13, memory-protection error; driver never
     *                             requests it -- left unconnected/unused)
     *   irq[2] = EDMAERRINT  (14, CC error; never asserted here)
     */
    qemu_irq irq[3];

    /*
     * Shadow-region-0 event/interrupt state (64 bits = one per channel).
     * These registers have set/clear/derived semantics and are NOT part of
     * the flat backing store below (see am335x_edma.c).
     */
    uint64_t er;             /* event pending (SH_ER/ECR) -- no HW events here */
    uint64_t eer;            /* event enable (SH_EER/EESR/EECR) */
    uint64_t ier;            /* interrupt enable (SH_IER/IESR/IECR) */
    uint64_t ipr;            /* interrupt pending (SH_IPR/ICR) */
    uint64_t ser;            /* secondary event (SH_SER/SECR) */

    /*
     * Flat backing store for the rest of the 64KB window (global config,
     * DCHMAP/DMAQNUM/QUEPRI, DRAE/QRAE, and the 256x32-byte PaRAM at 0x4000).
     * Plain read-back-what-was-written; the handful of RO/derived globals
     * (PID, CCCFG, EMR/QEMR/CCERR) are synthesized in the read path.
     */
    uint32_t regs[AM335X_EDMA_MMIO_SIZE / 4];

    /*
     * Cyclic-completion pump. While any channel has its event-enable (EER)
     * bit set and the PaRAM set it points at has TCINTEN, this virtual timer
     * periodically raises the channel's completion interrupt so an ALSA
     * cyclic playback (McASP0 -> EDMA) advances. Dormant whenever eer == 0,
     * so it never runs during boot (edma_probe issues no transfers).
     */
    QEMUTimer *cyclic_timer;
};

#endif /* HW_DMA_AM335X_EDMA_H */
