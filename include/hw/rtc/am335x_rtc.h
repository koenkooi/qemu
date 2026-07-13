/*
 * TI AM335x RTC ("ti,am3352-rtc") emulation.
 *
 * Reference: TRM spruh73q chapter 20 "RTC_SS", base 0x44E3E000.
 *
 * Unlike the other AM335x modules, the RTC's ti-sysc wrapper is
 * "ti,sysc-omap4-simple" whose capability table has srst_shift = -ENODEV,
 * so no OCP softreset (SYSS.RESETDONE) poll ever runs against it -- there
 * is no reset-done bit to synthesize here. The model provides a BCD
 * calendar (seeded from host wall-clock time at reset), the KICK0/KICK1
 * unlock registers (write protection is not enforced) and the CTRL/STATUS
 * RUN/BUSY semantics the rtc-omap driver probes.
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

#ifndef HW_RTC_AM335X_RTC_H
#define HW_RTC_AM335X_RTC_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AM335X_RTC "am335x-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xRtcState, AM335X_RTC)

struct AM335xRtcState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq_timer;      /* INTC line 75 (periodic) -- never asserted */
    qemu_irq irq_alarm;      /* INTC line 76 (alarm)    -- never asserted */

    /* BCD calendar (0x00..0x18). */
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t days;
    uint8_t months;
    uint8_t years;
    uint8_t weeks;

    /* BCD alarm compare banks (0x20..0x34 and 0x80..0x94):
     * [0]=seconds [1]=minutes [2]=hours [3]=days [4]=months [5]=years. */
    uint8_t alarm[6];
    uint8_t alarm2[6];

    uint32_t ctrl;           /* 0x40 RTC_CTRL_REG  */
    uint32_t status;         /* 0x44 latched event bits (RUN/BUSY synthesized) */
    uint32_t interrupts;     /* 0x48 RTC_INTERRUPTS_REG */
    uint32_t comp_lsb;       /* 0x4c */
    uint32_t comp_msb;       /* 0x50 */
    uint32_t osc;            /* 0x54 RTC_OSC_REG (reset 0x10) */
    uint32_t scratch[3];     /* 0x60/0x64/0x68 */
    uint32_t kick0;          /* 0x6c KICK0_REG */
    uint32_t kick1;          /* 0x70 KICK1_REG */
    uint32_t sysconfig;      /* 0x78 RTC_SYSCONFIG (reset 0x2) */
    uint32_t irqwakeen;      /* 0x7c RTC_IRQWAKEEN */
    uint32_t pmic;           /* 0x98 RTC_PMIC_REG */
};

#endif /* HW_RTC_AM335X_RTC_H */
