/*
 * TI AM335x EDMA3 (Enhanced DMA) controller emulation -- TPCC channel
 * controller, scoped to what the McASP0 HDMI-audio path needs.
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
 *
 * ---------------------------------------------------------------------------
 * What this is
 * ---------------------------------------------------------------------------
 * EDMA3 is TI's shared "Enhanced DMA" engine (TRM SPRUH73Q ch.11). On AM335x
 * it is used by McASP, MMC, SPI, crypto and others via the DT `dmas =
 * <&edma REQ TC>` bindings. This models the TPCC (Third-Party Channel
 * Controller, the user-facing block at 0x49000000) just far enough for the
 * mainline `ti,edma3-tpcc` dmaengine driver (drivers/dma/ti/edma.c) to probe
 * cleanly AND to drive one specific client -- the McASP0 I2S port's cyclic
 * audio playback DMA -- to periodic completion so ALSA `aplay`/`speaker-test`
 * make forward progress. It is deliberately NOT a general-purpose EDMA3: real
 * sample/byte transport is not modelled (like CPSW's RF-less Ethernet and the
 * wl18xx SDIO "clean probe"), only the register-level and completion-interrupt
 * behaviour the driver stack exercises. Remaining generality gaps are noted at
 * the bottom of this comment.
 *
 * ---------------------------------------------------------------------------
 * The Linux driver boundary this model targets (kernel 7.x, edma.c)
 * ---------------------------------------------------------------------------
 * Probe -- edma_setup_from_hw() (edma.c:2018) reads exactly ONE gating
 * register, CCCFG @ 0x0004, and derives the controller geometry from it
 * (GET_NUM_* macros, edma.c:101-106):
 *   CCCFG = 0x03224445  (TRM 11.4.1.2, offset 4h, reset value) decodes to
 *   64 channels, 8 QDMA channels, 256 PaRAM sets, 3 transfer controllers,
 *   4 shadow regions, CHMAP_EXIST=1.
 * There is NO probe-time poll loop. Probe then memsets all 256 PaRAM sets
 * (edma_write_slot -> memcpy_toio), requests IRQs (completion index 0 =
 * INTC 12, ccerr index 2 = INTC 14; the mperr index 1 = INTC 13 is never
 * requested), and zero-programs DRAE/DRAEH/QRAE for shadow region 0 plus the
 * per-channel queue/chmap tables. All of that is plain register RW here.
 * The PID register @ 0x0000 reads 0 (TRM 11.4.1.1 reset = 0h); the ti-sysc
 * wrapper reads it as "rev" and does not gate on the value.
 *
 * Interrupt model -- the driver enables a channel's completion interrupt at
 * allocation time via edma_setup_interrupt(true), which writes SH_ICR then
 * SH_IESR for the channel (edma.c:401-413), so IER[ch] is set well before any
 * completion. A PaRAM set with OPT.TCINTEN (BIT20) completing sets IPR[TCC]
 * and asserts INTC line 12; the ISR dma_irq_handler() (edma.c:1491-1534) reads
 * SH_IPR, writes SH_ICR to clear the serviced bits, runs the dmaengine cyclic
 * callback (-> snd_pcm_period_elapsed), then writes SH_IEVAL=1 to re-evaluate.
 * So the rule reproduced here is: line 12 is asserted while (IPR & IER) != 0;
 * an SH_ICR write clears IPR bits; SH_IEVAL=1 re-evaluates the line.
 *
 * ---------------------------------------------------------------------------
 * Cyclic-completion pump (the "make aplay progress" part)
 * ---------------------------------------------------------------------------
 * A real McASP playback arms a HW-triggered EDMA channel (SH_EESR sets the
 * event-enable bit) whose PaRAM ring has TCINTEN on every period; the McASP
 * FIFO then pulses the channel's DMA event once per sample and EDMA raises a
 * completion interrupt once per period. Modelling per-sample AXEVT pulses is
 * unnecessary for a structural model, so instead a QEMU_CLOCK_VIRTUAL timer
 * fires while any channel has EER set and its mapped PaRAM set has TCINTEN:
 * each tick sets IPR for that channel's TCC and asserts line 12. edma_stop
 * clears EER and the timer stops. Because edma_probe runs no transfers and no
 * other modelled peripheral arms an EDMA channel during boot, EER stays 0 and
 * the pump is completely dormant until userspace actually plays audio -- so it
 * adds no boot-time risk.
 *
 * ---------------------------------------------------------------------------
 * Documented generality gaps (scoped to McASP's needs)
 * ---------------------------------------------------------------------------
 *  - No actual data transport: PaRAM SRC/DST/counts are stored and read back
 *    but bytes are never copied between guest memory and a peripheral FIFO.
 *    The completion pump raises interrupts on a synthetic cadence, not one
 *    derived from ACNT/BCNT/CCNT or a real McASP sample clock.
 *  - Only shadow region 0 is functional (the only region Linux uses); regions
 *    1-3 back to the flat store.
 *  - Software-triggered (memcpy) channels via SH_ESR complete immediately but
 *    still move no data; the AM335x edma memcpy path is not exercised by this
 *    tree (no dmatest), so this is a convenience, not a tested path.
 *  - The error/mem-protection paths (EMR/QEMR/CCERR, lines 13/14) always read
 *    idle and are never asserted; the three TPTC transfer controllers are
 *    modelled as bare create_unimplemented_device stubs in the SoC (the
 *    edma tptc driver touches no TPTC registers, edma.c:2651-2655).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/dma/am335x_edma.h"
#include "hw/irq.h"
#include "migration/vmstate.h"

/* --- Global TPCC registers (base 0x49000000), offsets from edma.c ---------- */
#define EDMA_PID            0x0000  /* peripheral id / revision (RO, reset 0) */
#define EDMA_CCCFG          0x0004  /* configuration (RO); geometry source    */
#define EDMA_DCHMAP_BASE    0x0100  /* channel->PaRAM map, 64 regs            */
#define EDMA_DCHMAP_END     0x0200
#define EDMA_DMAQNUM_BASE   0x0240  /* channel->queue map, 8 regs             */
#define EDMA_DMAQNUM_END    0x0260
#define EDMA_QUEPRI         0x0284
#define EDMA_EMR            0x0300  /* event missed (RO, idle 0)              */
#define EDMA_EMRH           0x0304
#define EDMA_EMCR           0x0308  /* event-missed clear (W1C)               */
#define EDMA_EMCRH          0x030C
#define EDMA_QEMR           0x0310  /* QDMA event missed (RO, idle 0)         */
#define EDMA_QEMCR          0x0314
#define EDMA_CCERR          0x0318  /* CC error (RO, idle 0)                  */
#define EDMA_CCERRCLR       0x031C
#define EDMA_EEVAL          0x0320  /* error re-evaluate (W)                  */
#define EDMA_DRAE_BASE      0x0340  /* shadow-region access enable, 4x64-bit  */
#define EDMA_DRAE_END       0x0380
#define EDMA_QRAE_BASE      0x0380  /* QDMA region access enable, 4 regs      */
#define EDMA_QRAE_END       0x0390
#define EDMA_ECR            0x1008  /* global-region event clear (W1C)        */
#define EDMA_ECRH           0x100C

/* Config / PID reset values (TRM SPRUH73Q 11.4.1.1 / 11.4.1.2). */
#define EDMA_PID_RESET      0x00000000u
#define EDMA_CCCFG_RESET    0x03224445u

/* --- Shadow region 0 (base 0x2000): SH_* offsets relative to 0x2000 -------- */
#define EDMA_SHADOW0        0x2000
#define EDMA_SHADOW_STRIDE  0x0200  /* four regions at 0x2000/2200/2400/2600 */
#define SH_ER               0x00    /* event pending (RO)          */
#define SH_ECR              0x08    /* event clear (W1C)           */
#define SH_ESR              0x10    /* event set = SW trigger (W1S)*/
#define SH_EER              0x20    /* event enable (RO)           */
#define SH_EECR             0x28    /* event-enable clear (W1C)    */
#define SH_EESR             0x30    /* event-enable set (W1S)      */
#define SH_SER              0x38    /* secondary event (RO)        */
#define SH_SECR             0x40    /* secondary-event clear (W1C) */
#define SH_IER              0x50    /* interrupt enable (RO)       */
#define SH_IECR             0x58    /* interrupt-enable clear (W1C)*/
#define SH_IESR             0x60    /* interrupt-enable set (W1S)  */
#define SH_IPR              0x68    /* interrupt pending (RO)      */
#define SH_ICR              0x70    /* interrupt clear (W1C)       */
#define SH_IEVAL            0x78    /* interrupt re-evaluate (W)   */

/* --- PaRAM (base 0x4000): 256 sets x 32 bytes ------------------------------ */
#define EDMA_PARM_BASE      0x4000
#define EDMA_PARM_SET_SIZE  0x20
#define EDMA_PARM_END       (EDMA_PARM_BASE + \
                             AM335X_EDMA_NUM_PARAM * EDMA_PARM_SET_SIZE)

/* PaRAM OPT (word 0) fields (edma.c:152-162). */
#define PARM_OPT_TCC_SHIFT  12
#define PARM_OPT_TCC_MASK   0x3f
#define PARM_OPT_TCINTEN    (1u << 20)

/*
 * Synthetic cyclic-completion cadence. Arbitrary (real audio timing is not
 * modelled): ~100 completions/s keeps an ALSA playback advancing without an
 * interrupt storm. See the file header.
 */
#define EDMA_CYCLIC_PERIOD_NS  (NANOSECONDS_PER_SECOND / 100)

/* ------------------------------------------------------------------------- */

/* Recompute the completion-interrupt line: asserted while any pending bit is
 * also enabled (matches the driver's IPR/IER gating). */
static void am335x_edma_update_irq(AM335xEdmaState *s)
{
    qemu_set_irq(s->irq[0], (s->ipr & s->ier) != 0);
}

/* PaRAM slot a channel currently points at (DCHMAP holds slot<<5; CHMAP_EXIST
 * is set on AM335x). Falls back to the channel-tied slot number. */
static unsigned am335x_edma_channel_slot(AM335xEdmaState *s, unsigned ch)
{
    uint32_t dchmap = s->regs[(EDMA_DCHMAP_BASE + ch * 4) / 4];
    unsigned slot = dchmap >> 5;

    if (slot >= AM335X_EDMA_NUM_PARAM) {
        slot = ch;
    }
    return slot;
}

/* Read a PaRAM set's OPT word (word 0) from the flat store. */
static uint32_t am335x_edma_slot_opt(AM335xEdmaState *s, unsigned slot)
{
    return s->regs[(EDMA_PARM_BASE + slot * EDMA_PARM_SET_SIZE) / 4];
}

/*
 * Cyclic-completion pump tick: for every channel whose event is enabled and
 * whose PaRAM set requests a transfer-completion interrupt, raise the
 * completion for that set's TCC. Re-arms itself while any channel stays
 * enabled.
 */
static void am335x_edma_cyclic_tick(void *opaque)
{
    AM335xEdmaState *s = opaque;
    bool active = false;

    for (unsigned ch = 0; ch < AM335X_EDMA_NUM_CHANNELS; ch++) {
        if (!(s->eer & (1ull << ch))) {
            continue;
        }
        active = true;

        uint32_t opt = am335x_edma_slot_opt(s, am335x_edma_channel_slot(s, ch));
        if (opt & PARM_OPT_TCINTEN) {
            unsigned tcc = (opt >> PARM_OPT_TCC_SHIFT) & PARM_OPT_TCC_MASK;
            s->ipr |= (1ull << tcc);
        }
    }

    am335x_edma_update_irq(s);

    if (active) {
        timer_mod(s->cyclic_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + EDMA_CYCLIC_PERIOD_NS);
    }
}

/* Start/stop the pump to track whether any channel event is enabled. */
static void am335x_edma_sync_pump(AM335xEdmaState *s)
{
    if (s->eer) {
        if (!timer_pending(s->cyclic_timer)) {
            timer_mod(s->cyclic_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      EDMA_CYCLIC_PERIOD_NS);
        }
    } else {
        timer_del(s->cyclic_timer);
    }
}

/* ------------------------------------------------------------------------- */
/* Shadow region 0 event/interrupt registers (set/clear/derived semantics).  */

static uint64_t am335x_edma_shadow_read(AM335xEdmaState *s, hwaddr soff)
{
    hwaddr lo = soff & ~0x4ull;
    bool hi = (soff & 0x4) != 0;
    uint64_t v;

    switch (lo) {
    case SH_ER:   v = s->er;  break;
    case SH_EER:  v = s->eer; break;
    case SH_IER:  v = s->ier; break;
    case SH_IPR:  v = s->ipr; break;
    case SH_SER:  v = s->ser; break;
    default:      v = 0;      break;   /* ESR/ECR/EESR/... read 0 */
    }
    return hi ? (v >> 32) : (v & 0xffffffffu);
}

static void am335x_edma_shadow_write(AM335xEdmaState *s, hwaddr soff,
                                     uint32_t val)
{
    hwaddr lo = soff & ~0x4ull;
    bool hi = (soff & 0x4) != 0;
    uint64_t bits = hi ? ((uint64_t)val << 32) : val;

    switch (lo) {
    case SH_ECR:                        /* clear event pending */
        s->er &= ~bits;
        break;
    case SH_ESR:                        /* SW trigger: complete immediately */
        for (unsigned ch = 0; ch < AM335X_EDMA_NUM_CHANNELS; ch++) {
            if (!(bits & (1ull << ch))) {
                continue;
            }
            uint32_t opt =
                am335x_edma_slot_opt(s, am335x_edma_channel_slot(s, ch));
            if (opt & PARM_OPT_TCINTEN) {
                unsigned tcc = (opt >> PARM_OPT_TCC_SHIFT) & PARM_OPT_TCC_MASK;
                s->ipr |= (1ull << tcc);
            }
        }
        am335x_edma_update_irq(s);
        break;
    case SH_EESR:                       /* enable event (arm channel) */
        s->eer |= bits;
        am335x_edma_sync_pump(s);
        break;
    case SH_EECR:                       /* disable event (stop channel) */
        s->eer &= ~bits;
        am335x_edma_sync_pump(s);
        break;
    case SH_SECR:                       /* clear secondary event */
        s->ser &= ~bits;
        break;
    case SH_IESR:                       /* enable interrupt */
        s->ier |= bits;
        am335x_edma_update_irq(s);
        break;
    case SH_IECR:                       /* disable interrupt */
        s->ier &= ~bits;
        am335x_edma_update_irq(s);
        break;
    case SH_ICR:                        /* clear interrupt pending */
        s->ipr &= ~bits;
        am335x_edma_update_irq(s);
        break;
    case SH_IEVAL:                      /* re-evaluate interrupt line */
        am335x_edma_update_irq(s);
        break;
    default:
        break;                          /* ER/EER/IER/IPR are read-only */
    }
}

/* ------------------------------------------------------------------------- */

static uint64_t am335x_edma_read(void *opaque, hwaddr offset, unsigned size)
{
    AM335xEdmaState *s = opaque;

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read size %u at 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_EDMA, size, offset);
        return 0;
    }

    /* Shadow region 0: event/interrupt state. Regions 1-3 fall through to the
     * flat store below. */
    if (offset >= EDMA_SHADOW0 && offset < EDMA_SHADOW0 + EDMA_SHADOW_STRIDE) {
        return am335x_edma_shadow_read(s, offset - EDMA_SHADOW0);
    }

    switch (offset) {
    case EDMA_PID:
        return EDMA_PID_RESET;          /* ti-sysc "rev"; value 0 (authentic) */
    case EDMA_CCCFG:
        return EDMA_CCCFG_RESET;        /* controller geometry */
    case EDMA_EMR:
    case EDMA_EMRH:
    case EDMA_QEMR:
    case EDMA_CCERR:
        return 0;                       /* always idle: no missed events/errors */
    default:
        return s->regs[offset / 4];
    }
}

static void am335x_edma_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    AM335xEdmaState *s = opaque;
    uint32_t val = (uint32_t)value;

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write size %u at 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_EDMA, size, offset);
        return;
    }

    if (offset >= EDMA_SHADOW0 && offset < EDMA_SHADOW0 + EDMA_SHADOW_STRIDE) {
        am335x_edma_shadow_write(s, offset - EDMA_SHADOW0, val);
        return;
    }

    switch (offset) {
    case EDMA_PID:
    case EDMA_CCCFG:
    case EDMA_EMR:
    case EDMA_EMRH:
    case EDMA_QEMR:
    case EDMA_CCERR:
        break;                          /* read-only */
    case EDMA_EMCR:
    case EDMA_EMCRH:
    case EDMA_QEMCR:
    case EDMA_CCERRCLR:
    case EDMA_EEVAL:
        break;                          /* error clears: nothing pending */
    case EDMA_ECR:
    case EDMA_ECRH:
        break;                          /* global event clear: no HW events */
    default:
        /* DCHMAP/DMAQNUM/QUEPRI/DRAE/QRAE and the whole PaRAM area are plain
         * read-back storage. */
        s->regs[offset / 4] = val;
        break;
    }
}

static const MemoryRegionOps am335x_edma_ops = {
    .read = am335x_edma_read,
    .write = am335x_edma_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/* ------------------------------------------------------------------------- */

static void am335x_edma_reset(DeviceState *dev)
{
    AM335xEdmaState *s = AM335X_EDMA(dev);

    s->er = 0;
    s->eer = 0;
    s->ier = 0;
    s->ipr = 0;
    s->ser = 0;
    memset(s->regs, 0, sizeof(s->regs));
    timer_del(s->cyclic_timer);
    qemu_set_irq(s->irq[0], 0);
    qemu_set_irq(s->irq[1], 0);
    qemu_set_irq(s->irq[2], 0);
}

static void am335x_edma_init(Object *obj)
{
    AM335xEdmaState *s = AM335X_EDMA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &am335x_edma_ops, s,
                          TYPE_AM335X_EDMA, AM335X_EDMA_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    for (int i = 0; i < 3; i++) {
        sysbus_init_irq(sbd, &s->irq[i]);
    }
}

static void am335x_edma_realize(DeviceState *dev, Error **errp)
{
    AM335xEdmaState *s = AM335X_EDMA(dev);

    s->cyclic_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   am335x_edma_cyclic_tick, s);
}

static void am335x_edma_unrealize(DeviceState *dev)
{
    AM335xEdmaState *s = AM335X_EDMA(dev);

    if (s->cyclic_timer) {
        timer_free(s->cyclic_timer);
        s->cyclic_timer = NULL;
    }
}

static int am335x_edma_post_load(void *opaque, int version_id)
{
    AM335xEdmaState *s = opaque;

    /* Restore the pump and interrupt line from the reloaded event state. */
    am335x_edma_sync_pump(s);
    am335x_edma_update_irq(s);
    return 0;
}

static const VMStateDescription vmstate_am335x_edma = {
    .name = TYPE_AM335X_EDMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = am335x_edma_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT64(er, AM335xEdmaState),
        VMSTATE_UINT64(eer, AM335xEdmaState),
        VMSTATE_UINT64(ier, AM335xEdmaState),
        VMSTATE_UINT64(ipr, AM335xEdmaState),
        VMSTATE_UINT64(ser, AM335xEdmaState),
        VMSTATE_UINT32_ARRAY(regs, AM335xEdmaState, AM335X_EDMA_MMIO_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void am335x_edma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = am335x_edma_realize;
    dc->unrealize = am335x_edma_unrealize;
    device_class_set_legacy_reset(dc, am335x_edma_reset);
    dc->vmsd = &vmstate_am335x_edma;
    dc->desc = "TI AM335x EDMA3 channel controller (TPCC)";
}

static const TypeInfo am335x_edma_info = {
    .name          = TYPE_AM335X_EDMA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xEdmaState),
    .instance_init = am335x_edma_init,
    .class_init    = am335x_edma_class_init,
};

static void am335x_edma_register_types(void)
{
    type_register_static(&am335x_edma_info);
}

type_init(am335x_edma_register_types)
