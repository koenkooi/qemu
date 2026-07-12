/*
 * TI AM335x MPU interrupt controller (INTCPS) emulation.
 *
 * Reference: TRM spruh73q, chapter "6 Interrupts / MPU INTC".
 * Base address 0x48200000, 128 input lines, 4 banks of 32.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 */

#ifndef HW_INTC_AM335X_INTC_H
#define HW_INTC_AM335X_INTC_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AM335X_INTC "am335x-intc"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xIntcState, AM335X_INTC)

#define AM335X_INTC_NR_LINES 128
#define AM335X_INTC_NR_BANKS 4

struct AM335xIntcState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;

    /* Output lines to the CPU core (out 0 = IRQ, out 1 = FIQ). */
    qemu_irq parent_irq;
    qemu_irq parent_fiq;

    /* Raw input level per 32-line bank, driven by the gpio-in handler. */
    uint32_t ints[AM335X_INTC_NR_BANKS];
    /* MIR: interrupt mask, 1 = masked. Reset = all 0xFFFFFFFF (all masked). */
    uint32_t mask[AM335X_INTC_NR_BANKS];
    /* ISR: software-triggered interrupts, ORed into the pending set. */
    uint32_t isr[AM335X_INTC_NR_BANKS];

    /* Per-line ILR: bits[7:2] = priority, bit0 = FIQnIRQ routing. */
    uint32_t ilr[AM335X_INTC_NR_LINES];

    /* Priority threshold. 0xFF disables threshold gating. */
    uint32_t threshold;

    uint32_t protection;
    uint32_t idle;
    uint32_t sysconfig;

    /* Latched SIR read values (full 32-bit register contents). */
    uint32_t sir_irq;
    uint32_t sir_fiq;
};

#endif /* HW_INTC_AM335X_INTC_H */
