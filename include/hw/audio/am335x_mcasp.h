/*
 * TI AM335x McASP0 (Multichannel Audio Serial Port) emulation.
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

#ifndef HW_AUDIO_AM335X_MCASP_H
#define HW_AUDIO_AM335X_MCASP_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AM335X_MCASP "am335x-mcasp"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xMcaspState, AM335X_MCASP)

/*
 * Two MMIO windows (TRM SPRUH73Q ch.22; DT am33xx-l4.dtsi mcasp0):
 *   "mpu" config registers @ 0x48038000, 8KB
 *   "dat" data port        @ 0x46000000, 4MB
 */
#define AM335X_MCASP_MPU_SIZE   0x2000
#define AM335X_MCASP_DAT_SIZE   0x00400000

struct AM335xMcaspState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion mpu;        /* config-register window @ 0x48038000 */
    MemoryRegion dat;        /* data port window @ 0x46000000 */

    /*
     * INTC outputs (DT interrupts <80 81>, interrupt-names "tx"/"rx"):
     *   irq[0] = tx underrun (XUNDRN), irq[1] = rx overrun (ROVRN).
     * Error-only lines; never asserted by this model (no under/overruns).
     */
    qemu_irq irq[2];

    /* Flat backing store for the 8KB config window (read-back-what-written);
     * the identity/status registers are synthesized in the read path. */
    uint32_t regs[AM335X_MCASP_MPU_SIZE / 4];
};

#endif /* HW_AUDIO_AM335X_MCASP_H */
