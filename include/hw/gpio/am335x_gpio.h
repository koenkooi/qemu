/*
 * TI AM335x GPIO controller ("ti,omap4-gpio").
 *
 * Reference: TRM spruh73q chapter 25 "General-Purpose Input/Output".
 * Four identical 32-bit banks: GPIO0 0x44E07000, GPIO1 0x4804C000,
 * GPIO2 0x481AC000, GPIO3 0x481AE000 (each a single 0x1000 window that
 * also carries the ti-sysc target-module rev/sysc/syss registers).
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

#ifndef HW_GPIO_AM335X_GPIO_H
#define HW_GPIO_AM335X_GPIO_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AM335X_GPIO "am335x-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xGpioState, AM335X_GPIO)

#define AM335X_GPIO_NUM_LINES 32

struct AM335xGpioState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;                            /* combined IRQ to the INTC */
    qemu_irq output[AM335X_GPIO_NUM_LINES];  /* per-line output (e.g. LEDs) */

    /* Register state (omap4 layout; see am335x_gpio.c). */
    uint32_t sysconfig;      /* 0x010 */
    uint32_t irqstatus;      /* 0x02c GPIO_IRQSTATUS_0 (pending, W1C) */
    uint32_t irqstatus2;     /* 0x030 GPIO_IRQSTATUS_1 */
    uint32_t irqenable;      /* enable mask behind SET/CLR_0 (0x034/0x03c) */
    uint32_t irqenable2;     /* enable mask behind SET/CLR_1 (0x038/0x040) */
    uint32_t irqwaken;       /* 0x044 */
    uint32_t irqwaken2;      /* 0x048 */
    uint32_t ctrl;           /* 0x130 */
    uint32_t oe;             /* 0x134 (1 = input) */
    uint32_t datain;         /* 0x138 input line state (RO to guest) */
    uint32_t dataout;        /* 0x13c */
    uint32_t leveldetect0;   /* 0x140 */
    uint32_t leveldetect1;   /* 0x144 */
    uint32_t risingdetect;   /* 0x148 */
    uint32_t fallingdetect;  /* 0x14c */
    uint32_t debouncenable;  /* 0x150 */
    uint32_t debouncingtime; /* 0x154 */

    /* Reset value driven onto GPIO_DATAIN, so a board can present a static
     * input level (e.g. an active-low card-detect asserted low) without
     * wiring a driver to the input line. */
    uint32_t datain_reset;
};

#endif /* HW_GPIO_AM335X_GPIO_H */
