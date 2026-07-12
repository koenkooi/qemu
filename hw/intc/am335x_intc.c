/*
 * TI AM335x MPU interrupt controller (INTCPS) emulation.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Structural template: hw/intc/omap_intc.c (the OMAP1 INTC). The
 * priority-pick / SIR-code algorithm shape is the same; the register map is
 * different and follows the AM335x TRM (spruh73q, base 0x48200000).
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

#include "qemu/osdep.h"
#include "hw/intc/am335x_intc.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

/* --- Register offsets (TRM spruh73q) ---------------------------------- */
#define INTC_REVISION       0x00
#define INTC_SYSCONFIG      0x10
#define INTC_SYSSTATUS      0x14
#define INTC_SIR_IRQ        0x40
#define INTC_SIR_FIQ        0x44
#define INTC_CONTROL        0x48
#define INTC_PROTECTION     0x4c
#define INTC_IDLE           0x50
#define INTC_IRQ_PRIORITY   0x58
#define INTC_FIQ_PRIORITY   0x5c
#define INTC_THRESHOLD      0x60

/* Per-bank block: base 0x80 + n*0x20, n = 0..3. */
#define INTC_BANK_BASE      0x80
#define INTC_BANK_STRIDE    0x20
#define INTC_BANK_LAST      (INTC_BANK_BASE + \
                             AM335X_INTC_NR_BANKS * INTC_BANK_STRIDE - 1)
#define BANK_ITR            0x00
#define BANK_MIR            0x04
#define BANK_MIR_CLEAR      0x08
#define BANK_MIR_SET        0x0c
#define BANK_ISR_SET        0x10
#define BANK_ISR_CLEAR      0x14
#define BANK_PENDING_IRQ    0x18
#define BANK_PENDING_FIQ    0x1c

/* Per-line ILR: 0x100 + 4*i, i = 0..127. */
#define INTC_ILR_BASE       0x100
#define INTC_ILR_LAST       (INTC_ILR_BASE + AM335X_INTC_NR_LINES * 4 - 1)

/* Field encodings. */
#define ILR_FIQ_NOT_IRQ     0x1         /* bit0: route to FIQ instead of IRQ */
#define ILR_PRIORITY_MASK   0xfc        /* bits[7:2] */
#define ILR_PRIORITY_SHIFT  2

#define SYSCONFIG_AUTOIDLE  0x1         /* bit0 */
#define SYSCONFIG_SOFTRESET 0x2         /* bit1 */

#define SYSSTATUS_RESETDONE 0x1         /* bit0 */

#define CONTROL_NEWIRQAGR   0x1         /* bit0 */
#define CONTROL_NEWFIQAGR   0x2         /* bit1 */

#define INTC_REVISION_VALUE 0x00000040

/* SIR spurious flag: bits[31:7] = 0x1FFFFFF, line field bits[6:0] = 0. */
#define SIR_SPURIOUS        0xffffff80

#define THRESHOLD_DISABLED  0xff

/* --- Core logic ------------------------------------------------------- */

/*
 * Return the ILR priority field for a line (bits[7:2], numerically largest =
 * highest priority per this model's convention).
 */
static inline uint32_t am335x_intc_prio(AM335xIntcState *s, int line)
{
    return (s->ilr[line] & ILR_PRIORITY_MASK) >> ILR_PRIORITY_SHIFT;
}

static inline bool am335x_intc_line_pending(AM335xIntcState *s, int line)
{
    int bank = line >> 5;
    uint32_t bit = 1u << (line & 31);

    return ((s->ints[bank] | s->isr[bank]) & ~s->mask[bank] & bit) != 0;
}

/*
 * Scan all 128 lines, pick the highest-priority pending line routed to the
 * requested output (is_fiq selects the FIQ-routed set), and latch its SIR
 * code. Returns true if such a line exists.
 *
 * Tie-break: lowest line number wins. We iterate ascending and only replace
 * the current best on a strictly-greater priority, so the first (lowest) line
 * at the winning priority is kept.
 *
 * Threshold gating: the AM335x priority threshold suppresses low-priority
 * lines. For the M1 BeagleBone-Black bring-up the Linux omap-intc driver
 * programs THRESHOLD = 0xFF, which disables gating entirely, so the exact
 * comparison never affects the timer IRQ path. We implement the simple form:
 * when threshold != 0xFF, a line is skipped if its priority is greater than
 * the threshold. (Documented assumption; refine if a guest actually uses it.)
 */
static bool am335x_intc_pick(AM335xIntcState *s, bool is_fiq, uint32_t *sir)
{
    int best_line = -1;
    int best_prio = -1;
    int line;

    for (line = 0; line < AM335X_INTC_NR_LINES; line++) {
        bool routed_fiq;
        uint32_t prio;

        if (!am335x_intc_line_pending(s, line)) {
            continue;
        }

        routed_fiq = (s->ilr[line] & ILR_FIQ_NOT_IRQ) != 0;
        if (routed_fiq != is_fiq) {
            continue;
        }

        prio = am335x_intc_prio(s, line);

        if (s->threshold != THRESHOLD_DISABLED && prio > s->threshold) {
            continue;
        }

        if ((int)prio > best_prio) {
            best_prio = prio;
            best_line = line;
        }
    }

    if (best_line < 0) {
        *sir = SIR_SPURIOUS;
        return false;
    }

    *sir = (uint32_t)best_line & 0x7f;
    return true;
}

static void am335x_intc_update(AM335xIntcState *s)
{
    bool irq_active = am335x_intc_pick(s, false, &s->sir_irq);
    bool fiq_active = am335x_intc_pick(s, true, &s->sir_fiq);

    qemu_set_irq(s->parent_irq, irq_active);
    qemu_set_irq(s->parent_fiq, fiq_active);
}

/*
 * gpio-in handler. Level input: the peripheral holds the line asserted until
 * it is serviced, so we track raw level and let update() recompute.
 */
static void am335x_intc_set_irq(void *opaque, int n, int level)
{
    AM335xIntcState *s = AM335X_INTC(opaque);
    int bank = n >> 5;
    uint32_t bit = 1u << (n & 31);

    if (n < 0 || n >= AM335X_INTC_NR_LINES) {
        return;
    }

    if (level) {
        s->ints[bank] |= bit;
    } else {
        s->ints[bank] &= ~bit;
    }

    am335x_intc_update(s);
}

/* --- Reset ------------------------------------------------------------ */

static void am335x_intc_reset_regs(AM335xIntcState *s)
{
    int i;

    for (i = 0; i < AM335X_INTC_NR_BANKS; i++) {
        s->ints[i] = 0;
        s->isr[i] = 0;
        s->mask[i] = 0xffffffff;   /* TRM reset: all lines masked. */
    }
    for (i = 0; i < AM335X_INTC_NR_LINES; i++) {
        s->ilr[i] = 0;             /* TRM reset: priority 0, routed to IRQ. */
    }

    /*
     * TRM reset value for INTC_THRESHOLD is 0x0 (gating enabled at priority
     * 0). We reset to 0xFF (gating disabled) instead: the omap-intc driver
     * writes 0xFF during init anyway, and defaulting to "disabled" avoids
     * spuriously gating the timer IRQ before the driver runs. Gating is a
     * no-op on the M1 boot path either way.
     */
    s->threshold = THRESHOLD_DISABLED;

    s->protection = 0;
    s->idle = 0;
    s->sysconfig = 0;

    s->sir_irq = SIR_SPURIOUS;
    s->sir_fiq = SIR_SPURIOUS;

    qemu_set_irq(s->parent_irq, 0);
    qemu_set_irq(s->parent_fiq, 0);
}

static void am335x_intc_reset(DeviceState *dev)
{
    AM335xIntcState *s = AM335X_INTC(dev);

    am335x_intc_reset_regs(s);
}

/* --- MMIO ------------------------------------------------------------- */

static uint64_t am335x_intc_read(void *opaque, hwaddr addr, unsigned size)
{
    AM335xIntcState *s = AM335X_INTC(opaque);

    /* Per-bank block. */
    if (addr >= INTC_BANK_BASE && addr <= INTC_BANK_LAST) {
        int bank = (addr - INTC_BANK_BASE) / INTC_BANK_STRIDE;
        hwaddr off = (addr - INTC_BANK_BASE) % INTC_BANK_STRIDE;
        uint32_t raw = s->ints[bank] | s->isr[bank];
        uint32_t pending = raw & ~s->mask[bank];
        uint32_t irq_bits = 0, fiq_bits = 0;
        int i;

        switch (off) {
        case BANK_ITR:
            return raw;
        case BANK_MIR:
            return s->mask[bank];
        case BANK_MIR_CLEAR:   /* write-1-to-clear; reads as 0. */
        case BANK_MIR_SET:     /* write-1-to-set; reads as 0. */
            return 0;
        case BANK_ISR_SET:
            return s->isr[bank];
        case BANK_ISR_CLEAR:
            return 0;
        case BANK_PENDING_IRQ:
        case BANK_PENDING_FIQ:
            for (i = 0; i < 32; i++) {
                int line = bank * 32 + i;
                if (!(pending & (1u << i))) {
                    continue;
                }
                if (s->ilr[line] & ILR_FIQ_NOT_IRQ) {
                    fiq_bits |= 1u << i;
                } else {
                    irq_bits |= 1u << i;
                }
            }
            return (off == BANK_PENDING_IRQ) ? irq_bits : fiq_bits;
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return 0;
    }

    /* Per-line ILR block. */
    if (addr >= INTC_ILR_BASE && addr <= INTC_ILR_LAST) {
        int line = (addr - INTC_ILR_BASE) / 4;
        return s->ilr[line];
    }

    switch (addr) {
    case INTC_REVISION:
        return INTC_REVISION_VALUE;
    case INTC_SYSCONFIG:
        return s->sysconfig;
    case INTC_SYSSTATUS:
        return SYSSTATUS_RESETDONE;   /* RESETDONE always 1. */
    case INTC_SIR_IRQ:
        am335x_intc_pick(s, false, &s->sir_irq);
        return s->sir_irq;
    case INTC_SIR_FIQ:
        am335x_intc_pick(s, true, &s->sir_fiq);
        return s->sir_fiq;
    case INTC_CONTROL:
        return 0;
    case INTC_PROTECTION:
        return s->protection;
    case INTC_IDLE:
        return s->idle;
    case INTC_IRQ_PRIORITY:
        /* Current IRQ priority; a low/zero value is acceptable. */
        return 0;
    case INTC_FIQ_PRIORITY:
        return 0;
    case INTC_THRESHOLD:
        return s->threshold;
    }

    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: bad read offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    return 0;
}

static void am335x_intc_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    AM335xIntcState *s = AM335X_INTC(opaque);
    uint32_t val = value;

    /* Per-bank block. */
    if (addr >= INTC_BANK_BASE && addr <= INTC_BANK_LAST) {
        int bank = (addr - INTC_BANK_BASE) / INTC_BANK_STRIDE;
        hwaddr off = (addr - INTC_BANK_BASE) % INTC_BANK_STRIDE;

        switch (off) {
        case BANK_MIR:
            s->mask[bank] = val;
            am335x_intc_update(s);
            return;
        case BANK_MIR_CLEAR:       /* write-1 clears mask bits -> enable */
            s->mask[bank] &= ~val;
            am335x_intc_update(s);
            return;
        case BANK_MIR_SET:         /* write-1 sets mask bits -> disable */
            s->mask[bank] |= val;
            am335x_intc_update(s);
            return;
        case BANK_ISR_SET:         /* write-1 raises a software interrupt */
            s->isr[bank] |= val;
            am335x_intc_update(s);
            return;
        case BANK_ISR_CLEAR:       /* write-1 clears a software interrupt */
            s->isr[bank] &= ~val;
            am335x_intc_update(s);
            return;
        case BANK_ITR:             /* read-only status */
        case BANK_PENDING_IRQ:
        case BANK_PENDING_FIQ:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: write to read-only offset 0x%" HWADDR_PRIx "\n",
                          __func__, addr);
            return;
        }
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return;
    }

    /* Per-line ILR block. */
    if (addr >= INTC_ILR_BASE && addr <= INTC_ILR_LAST) {
        int line = (addr - INTC_ILR_BASE) / 4;
        s->ilr[line] = val & (ILR_PRIORITY_MASK | ILR_FIQ_NOT_IRQ);
        am335x_intc_update(s);
        return;
    }

    switch (addr) {
    case INTC_SYSCONFIG:
        if (val & SYSCONFIG_SOFTRESET) {
            am335x_intc_reset_regs(s);
            return;
        }
        s->sysconfig = val & SYSCONFIG_AUTOIDLE;
        return;
    case INTC_CONTROL:
        /*
         * NewIRQAgr / NewFIQAgr re-arm the output. The line inputs stay
         * asserted (level-triggered) until the peripheral clears them, so a
         * still-pending interrupt re-asserts here. This is exactly what a
         * simple re-run of update() does.
         */
        if (val & (CONTROL_NEWIRQAGR | CONTROL_NEWFIQAGR)) {
            am335x_intc_update(s);
        }
        return;
    case INTC_PROTECTION:
        s->protection = val & 0x1;
        return;
    case INTC_IDLE:
        s->idle = val;
        return;
    case INTC_THRESHOLD:
        s->threshold = val & 0xff;
        am335x_intc_update(s);
        return;
    case INTC_REVISION:
    case INTC_SYSSTATUS:
    case INTC_SIR_IRQ:
    case INTC_SIR_FIQ:
    case INTC_IRQ_PRIORITY:
    case INTC_FIQ_PRIORITY:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write to read-only offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
        return;
    }

    qemu_log_mask(LOG_GUEST_ERROR,
                  "%s: bad write offset 0x%" HWADDR_PRIx "\n", __func__, addr);
}

static const MemoryRegionOps am335x_intc_ops = {
    .read = am335x_intc_read,
    .write = am335x_intc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* --- QOM -------------------------------------------------------------- */

static void am335x_intc_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj);
    AM335xIntcState *s = AM335X_INTC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &am335x_intc_ops, s,
                          TYPE_AM335X_INTC, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    /* out 0 = IRQ to CPU, out 1 = FIQ to CPU. */
    sysbus_init_irq(sbd, &s->parent_irq);
    sysbus_init_irq(sbd, &s->parent_fiq);

    qdev_init_gpio_in(dev, am335x_intc_set_irq, AM335X_INTC_NR_LINES);
}

static const VMStateDescription vmstate_am335x_intc = {
    .name = TYPE_AM335X_INTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(ints, AM335xIntcState, AM335X_INTC_NR_BANKS),
        VMSTATE_UINT32_ARRAY(mask, AM335xIntcState, AM335X_INTC_NR_BANKS),
        VMSTATE_UINT32_ARRAY(isr, AM335xIntcState, AM335X_INTC_NR_BANKS),
        VMSTATE_UINT32_ARRAY(ilr, AM335xIntcState, AM335X_INTC_NR_LINES),
        VMSTATE_UINT32(threshold, AM335xIntcState),
        VMSTATE_UINT32(protection, AM335xIntcState),
        VMSTATE_UINT32(idle, AM335xIntcState),
        VMSTATE_UINT32(sysconfig, AM335xIntcState),
        VMSTATE_UINT32(sir_irq, AM335xIntcState),
        VMSTATE_UINT32(sir_fiq, AM335xIntcState),
        VMSTATE_END_OF_LIST()
    }
};

static void am335x_intc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, am335x_intc_reset);
    dc->vmsd = &vmstate_am335x_intc;
    dc->desc = "TI AM335x MPU interrupt controller (INTCPS)";
}

static const TypeInfo am335x_intc_info = {
    .name          = TYPE_AM335X_INTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xIntcState),
    .instance_init = am335x_intc_init,
    .class_init    = am335x_intc_class_init,
};

static void am335x_intc_register_types(void)
{
    type_register_static(&am335x_intc_info);
}

type_init(am335x_intc_register_types)
