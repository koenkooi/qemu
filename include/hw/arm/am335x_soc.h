/*
 * TI AM335x SoC emulation
 *
 * Copyright (C) 2026
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

#ifndef HW_ARM_AM335X_SOC_H
#define HW_ARM_AM335X_SOC_H

#include "hw/qdev-core.h"
#include "exec/memory.h"
#include "target/arm/cpu.h"
#include "hw/intc/am335x_intc.h"
#include "hw/timer/am335x_timer.h"
#include "hw/misc/am335x_prcm.h"
#include "hw/misc/am335x_wdt.h"
#include "hw/misc/am335x_control.h"
#include "hw/sd/am335x_hsmmc.h"
#include "hw/char/am335x_uart.h"
#include "qom/object.h"

#define TYPE_AM335X_SOC "am335x-soc"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xState, AM335X_SOC)

#define AM335X_NUM_TIMERS 4
#define AM335X_NUM_MMC    2

struct AM335xState {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/

    ARMCPU cpu;
    AM335xIntcState intc;
    AM335xTimerState timer[AM335X_NUM_TIMERS];
    AM335xPrcmState prcm;
    AM335xWdtState wdt;
    AM335xControlState control;
    AM335xHsmmcState mmc[AM335X_NUM_MMC];
    AM335xUartState uart0;
    MemoryRegion ocmc;
};

#endif /* HW_ARM_AM335X_SOC_H */
