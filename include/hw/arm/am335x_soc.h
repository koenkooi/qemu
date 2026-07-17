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
#include "hw/misc/am335x_emif.h"
#include "hw/gpio/am335x_gpio.h"
#include "hw/sd/am335x_hsmmc.h"
#include "hw/char/am335x_uart.h"
#include "hw/i2c/am335x_i2c.h"
#include "hw/rtc/am335x_rtc.h"
#include "hw/display/am335x_lcdc.h"
#include "hw/net/am335x_cpsw.h"
#include "hw/misc/am335x_usbss.h"
#include "hw/dma/am335x_edma.h"
#include "qom/object.h"

#define TYPE_AM335X_SOC "am335x-soc"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xState, AM335X_SOC)

#define AM335X_NUM_TIMERS 4
#define AM335X_NUM_MMC    3   /* MMCHS0 (microSD), MMCHS1 (eMMC), MMCHS2 (SDIO) */
#define AM335X_NUM_GPIO   4
#define AM335X_NUM_I2C    1   /* I2C0 only; I2C1/I2C2 remain unimplemented */

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
    AM335xEmifState emif;
    AM335xGpioState gpio[AM335X_NUM_GPIO];
    AM335xHsmmcState mmc[AM335X_NUM_MMC];
    AM335xUartState uart0;
    AM335xI2cState i2c[AM335X_NUM_I2C];
    AM335xRtcState rtc;
    AM335xLcdcState lcdc;
    AM335xCpswState cpsw;
    AM335xUsbssState usbss;
    AM335xEdmaState edma;
    MemoryRegion ocmc;
    MemoryRegion sram;
};

/*
 * am335x_bootrom.c: the boot ROM's SD card file-system boot (TRM SPRUH73Q
 * 26.1.8.5). Reads the booting file "MLO" off the card's FAT partition;
 * returns a g_malloc()ed buffer (size in *lenp) or NULL with errp set.
 */
uint8_t *am335x_bootrom_read_mlo(BlockBackend *blk, size_t *lenp,
                                 Error **errp);

#endif /* HW_ARM_AM335X_SOC_H */
