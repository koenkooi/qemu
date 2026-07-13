/*
 * TI AM335x DMTIMER
 *
 * A minimal but functional model of the AM335x General Purpose Timer
 * (DMTIMER) block. It implements just enough of the TRM (spruh73q)
 * register set for the Linux "clockevent-ti-dm" / omap_dm_timer driver
 * to probe the device via ti-sysc, arm it, and receive periodic overflow
 * interrupts -- i.e. enough to get the kernel's clockevent/clocksource
 * past time init.
 *
 * The counter is a free-running 32-bit up-counter which overflows at
 * 2^32 and, depending on TCLR.AR, either auto-reloads from TLDR or
 * stops. This is modelled internally with a QEMU ptimer configured as a
 * down-counter of "ticks remaining until overflow"; the up-counter view
 * exposed to the guest (TCRR) is derived from that down-counter.
 *
 * Compare-match (MAT) interrupt generation and the PRE/PTV prescaler are
 * not implemented; only overflow (OVF) interrupts are generated, which
 * is sufficient to drive the kernel's periodic and one-shot clockevent
 * modes.
 *
 * This code is licensed under GPL version 2 or later. See the COPYING
 * file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/timer/am335x_timer.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "migration/vmstate.h"

/* TCLR bits */
#define TCLR_ST      (1 << 0)  /* start/stop */
#define TCLR_AR      (1 << 1)  /* autoreload */
#define TCLR_PTV_SHIFT  2
#define TCLR_PTV_MASK   (0x7 << TCLR_PTV_SHIFT)
#define TCLR_PRE     (1 << 5)  /* prescaler enable */
#define TCLR_CE      (1 << 6)  /* compare enable */

/* IRQSTATUS_RAW / IRQSTATUS / IRQENABLE_SET / IRQENABLE_CLR bits */
#define IRQ_MAT      (1 << 0)
#define IRQ_OVF      (1 << 1)
#define IRQ_TCAR     (1 << 2)
#define IRQ_MASK     (IRQ_MAT | IRQ_OVF | IRQ_TCAR)

/*
 * TIOCP_CFG (SYSCONFIG) SOFTRESET bit. Its position depends on the timer
 * variant's OCP wrapper:
 *  - Regular DMTIMER ("ti,am335x-timer", OMAP4-style sysc): bit 0.
 *  - 1ms DMTIMER1    ("ti,am335x-timer-1ms", OMAP2-style sysc): bit 1;
 *    for that variant bit 0 is AUTOIDLE, so misreading it as SOFTRESET
 *    resets the always-on clocksource whenever the kernel enables
 *    autoidle, stopping the free-running counter (see s->one_ms).
 */
#define TIOCP_CFG_SOFTRESET_OMAP4 (1 << 0)
#define TIOCP_CFG_SOFTRESET_OMAP2 (1 << 1)

/* TISTAT bits */
#define TISTAT_RESETDONE (1 << 0)

/* Register offsets (TRM spruh73q) */
#define TIDR            0x00
#define TIOCP_CFG       0x10
#define TISTAT          0x14
#define IRQ_EOI         0x20
#define IRQSTATUS_RAW   0x24
#define IRQSTATUS       0x28
#define IRQENABLE_SET   0x2c
#define IRQENABLE_CLR   0x30
#define IRQWAKEEN       0x34
#define TCLR            0x38
#define TCRR            0x3c
#define TLDR            0x40
#define TTGR            0x44
#define TWPS            0x48
#define TMAR            0x4c
#define TCAR1           0x50
#define TSICR           0x54
#define TCAR2           0x58

#define AM335X_TIMER_TIDR_VALUE 0x40000100

static uint64_t am335x_timer_limit(AM335xTimerState *s)
{
    /* Ticks (at the functional clock rate) until the up-counter TCRR,
     * which starts at TLDR, wraps through 0x100000000. */
    return 0x100000000ULL - s->tldr;
}

/* Effective functional-clock rate (Hz), honouring the PTV/PRE prescaler. */
static uint32_t am335x_timer_freq(AM335xTimerState *s)
{
    uint32_t freq = AM335X_TIMER_FREQ;

    if (s->tclr & TCLR_PRE) {
        uint32_t ptv = (s->tclr & TCLR_PTV_MASK) >> TCLR_PTV_SHIFT;
        freq >>= (ptv + 1);
        if (freq == 0) {
            freq = 1;
        }
    }
    return freq;
}

/*
 * Guest-visible 32-bit up-counter (TCRR), computed directly from
 * QEMU_CLOCK_VIRTUAL. The counter runs over [TLDR, 0x100000000) and
 * reloads to TLDR on overflow, so it advances modulo (0x100000000 - TLDR)
 * from its base. Deriving it from the virtual clock (rather than reading
 * back the internal ptimer's down-counter) is what lets the free-running
 * clocksource use case wrap cleanly and indefinitely through 2^32; the
 * ptimer's periodic reload cannot service a 2^32-tick period, which
 * previously left ktime/sched_clock frozen at the first wrap (~179s).
 */
static uint32_t am335x_timer_get_tcrr(AM335xTimerState *s)
{
    uint64_t elapsed, span;
    int64_t off, now;

    if (!(s->tclr & TCLR_ST)) {
        /* Stopped: the counter holds its last value. */
        return s->base_tcrr;
    }

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    elapsed = muldiv64(now - s->base_time, am335x_timer_freq(s),
                       NANOSECONDS_PER_SECOND);

    span = 0x100000000ULL - s->tldr;
    off = (int64_t)s->base_tcrr - (int64_t)s->tldr;
    if (off < 0) {
        off += span;
    }
    return s->tldr + (uint32_t)(((uint64_t)off + elapsed) % span);
}

/* Re-base the virtual-clock counter so that TCRR == tcrr as of now. */
static void am335x_timer_rebase(AM335xTimerState *s, uint32_t tcrr)
{
    s->base_tcrr = tcrr;
    s->base_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void am335x_timer_update_irq(AM335xTimerState *s)
{
    uint32_t status = s->irqstatus_raw & s->irqenable;

    qemu_set_irq(s->irq, status != 0);
}

/* Must be called from within a ptimer_transaction_begin/commit block. */
static void am335x_timer_update_freq(AM335xTimerState *s)
{
    ptimer_set_freq(s->timer, am335x_timer_freq(s));
}

/* ptimer expiry callback: the up-counter has overflowed past
 * 0xFFFFFFFF. Called from within a ptimer transaction block. */
static void am335x_timer_tick(void *opaque)
{
    AM335xTimerState *s = AM335X_TIMER(opaque);

    s->irqstatus_raw |= IRQ_OVF;

    /* The up-counter has just overflowed and reloaded from TLDR; re-base
     * the virtual-clock counter to match. */
    am335x_timer_rebase(s, s->tldr);

    if (!(s->tclr & TCLR_AR)) {
        /* Hardware clears ST and stops counting when AR is not set. */
        s->tclr &= ~TCLR_ST;
    }

    am335x_timer_update_irq(s);
}

static void am335x_timer_reset_hold(AM335xTimerState *s)
{
    ptimer_transaction_begin(s->timer);
    ptimer_stop(s->timer);

    s->irqstatus_raw = 0;
    s->irqstatus = 0;
    s->irqenable = 0;
    s->irqwakeen = 0;
    s->tclr = 0;
    s->tldr = 0;
    s->tmar = 0;
    s->tcar1 = 0;
    s->tcar2 = 0;
    s->tsicr = 0;

    am335x_timer_update_freq(s);
    ptimer_set_limit(s->timer, am335x_timer_limit(s), 1);
    ptimer_transaction_commit(s->timer);

    /* Counter stopped (TCLR.ST clear) and reads back 0. */
    am335x_timer_rebase(s, 0);

    am335x_timer_update_irq(s);
}

static void am335x_timer_dev_reset(DeviceState *dev)
{
    AM335xTimerState *s = AM335X_TIMER(dev);

    s->tiocp_cfg = 0;
    am335x_timer_reset_hold(s);
}

static void am335x_timer_write_tclr(AM335xTimerState *s, uint32_t value)
{
    /* Snapshot the counter under the old control settings, then re-base so
     * the new settings (start/stop, prescaler) take effect from now. */
    uint32_t cur = am335x_timer_get_tcrr(s);

    s->tclr = value;
    am335x_timer_rebase(s, cur);

    ptimer_transaction_begin(s->timer);
    am335x_timer_update_freq(s);
    ptimer_set_limit(s->timer, am335x_timer_limit(s), 0);
    if (s->tclr & TCLR_ST) {
        /* Arm the ptimer's overflow at (0x100000000 - cur) ticks so the
         * OVF interrupt still fires at the counter wrap. */
        ptimer_set_count(s->timer, 0x100000000ULL - cur);
        ptimer_run(s->timer, (s->tclr & TCLR_AR) ? 0 : 1);
    } else {
        ptimer_stop(s->timer);
    }
    ptimer_transaction_commit(s->timer);
}

static void am335x_timer_write_tldr(AM335xTimerState *s, uint32_t value)
{
    /* Writing TLDR alone does not reload TCRR; the new reload value only
     * takes effect on the next overflow (if AR is set) or an explicit
     * TTGR write. Preserve the current counter value across the change of
     * reload/period by snapshotting and re-basing. */
    uint32_t cur = am335x_timer_get_tcrr(s);

    s->tldr = value;
    am335x_timer_rebase(s, cur);

    ptimer_transaction_begin(s->timer);
    ptimer_set_limit(s->timer, am335x_timer_limit(s), 0);
    ptimer_transaction_commit(s->timer);
}

static void am335x_timer_write_tcrr(AM335xTimerState *s, uint32_t value)
{
    am335x_timer_rebase(s, value);

    ptimer_transaction_begin(s->timer);
    ptimer_set_count(s->timer, 0x100000000ULL - value);
    ptimer_transaction_commit(s->timer);
}

static void am335x_timer_write_ttgr(AM335xTimerState *s, uint32_t value)
{
    /* Any write triggers an immediate reload of TCRR from TLDR. */
    am335x_timer_rebase(s, s->tldr);

    ptimer_transaction_begin(s->timer);
    ptimer_set_count(s->timer, am335x_timer_limit(s));
    ptimer_transaction_commit(s->timer);
}

static void am335x_timer_write_tiocp_cfg(AM335xTimerState *s, uint32_t value)
{
    uint32_t softreset = s->one_ms ? TIOCP_CFG_SOFTRESET_OMAP2
                                   : TIOCP_CFG_SOFTRESET_OMAP4;

    if (value & softreset) {
        /* SOFTRESET is self-clearing; the rest of TIOCP_CFG survives
         * a soft reset in real hardware, but the timer's functional
         * state does not. */
        s->tiocp_cfg = value & ~softreset;
        am335x_timer_reset_hold(s);
    } else {
        s->tiocp_cfg = value;
    }
}

static uint64_t am335x_timer_read(void *opaque, hwaddr offset, unsigned size)
{
    AM335xTimerState *s = AM335X_TIMER(opaque);

    switch (offset) {
    case TIDR:
        return AM335X_TIMER_TIDR_VALUE;
    case TIOCP_CFG:
        return s->tiocp_cfg;
    case TISTAT:
        return TISTAT_RESETDONE;
    case IRQ_EOI:
        return 0;
    case IRQSTATUS_RAW:
        return s->irqstatus_raw;
    case IRQSTATUS:
        return s->irqstatus_raw & s->irqenable;
    case IRQENABLE_SET:
    case IRQENABLE_CLR:
        return s->irqenable;
    case IRQWAKEEN:
        return s->irqwakeen;
    case TCLR:
        return s->tclr;
    case TCRR:
        return am335x_timer_get_tcrr(s);
    case TLDR:
        return s->tldr;
    case TTGR:
        return 0;
    case TWPS:
        /* No posted writes are ever pending in this model. */
        return 0;
    case TMAR:
        return s->tmar;
    case TCAR1:
        return s->tcar1;
    case TSICR:
        return s->tsicr;
    case TCAR2:
        return s->tcar2;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_TIMER, offset);
        return 0;
    }
}

static void am335x_timer_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    AM335xTimerState *s = AM335X_TIMER(opaque);
    uint32_t v = (uint32_t)value;

    switch (offset) {
    case TIDR:
        /* read-only */
        break;
    case TIOCP_CFG:
        am335x_timer_write_tiocp_cfg(s, v);
        break;
    case TISTAT:
        /* read-only */
        break;
    case IRQ_EOI:
        /* End-of-interrupt acknowledge: nothing further to do, the
         * per-source status bits are cleared via IRQSTATUS. */
        break;
    case IRQSTATUS_RAW:
        /* Write-1-to-set, for test/diagnostic purposes. */
        s->irqstatus_raw |= (v & IRQ_MASK);
        am335x_timer_update_irq(s);
        break;
    case IRQSTATUS:
        /* Write-1-to-clear. */
        s->irqstatus_raw &= ~(v & IRQ_MASK);
        am335x_timer_update_irq(s);
        break;
    case IRQENABLE_SET:
        s->irqenable |= (v & IRQ_MASK);
        am335x_timer_update_irq(s);
        break;
    case IRQENABLE_CLR:
        s->irqenable &= ~(v & IRQ_MASK);
        am335x_timer_update_irq(s);
        break;
    case IRQWAKEEN:
        s->irqwakeen = v;
        break;
    case TCLR:
        am335x_timer_write_tclr(s, v);
        break;
    case TCRR:
        am335x_timer_write_tcrr(s, v);
        break;
    case TLDR:
        am335x_timer_write_tldr(s, v);
        break;
    case TTGR:
        am335x_timer_write_ttgr(s, v);
        break;
    case TWPS:
        /* read-only */
        break;
    case TMAR:
        s->tmar = v;
        break;
    case TCAR1:
        s->tcar1 = v;
        break;
    case TSICR:
        s->tsicr = v;
        break;
    case TCAR2:
        s->tcar2 = v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_TIMER, offset);
        break;
    }
}

static const MemoryRegionOps am335x_timer_ops = {
    .read = am335x_timer_read,
    .write = am335x_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static const VMStateDescription am335x_timer_vmstate = {
    .name = TYPE_AM335X_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(tiocp_cfg, AM335xTimerState),
        VMSTATE_UINT32(irqstatus_raw, AM335xTimerState),
        VMSTATE_UINT32(irqstatus, AM335xTimerState),
        VMSTATE_UINT32(irqenable, AM335xTimerState),
        VMSTATE_UINT32(irqwakeen, AM335xTimerState),
        VMSTATE_UINT32(tclr, AM335xTimerState),
        VMSTATE_UINT32(tldr, AM335xTimerState),
        VMSTATE_UINT32(tmar, AM335xTimerState),
        VMSTATE_UINT32(tcar1, AM335xTimerState),
        VMSTATE_UINT32(tcar2, AM335xTimerState),
        VMSTATE_UINT32(tsicr, AM335xTimerState),
        VMSTATE_INT64(base_time, AM335xTimerState),
        VMSTATE_UINT32(base_tcrr, AM335xTimerState),
        VMSTATE_PTIMER(timer, AM335xTimerState),
        VMSTATE_END_OF_LIST()
    }
};

static void am335x_timer_realize(DeviceState *dev, Error **errp)
{
    AM335xTimerState *s = AM335X_TIMER(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init_io(&s->iomem, OBJECT(s), &am335x_timer_ops, s,
                          TYPE_AM335X_TIMER, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    /*
     * QEMU v10.0 hw/ptimer.h has no PTIMER_POLICY_DEFAULT macro (only
     * PTIMER_POLICY_LEGACY and individual PTIMER_POLICY_* bits); this is
     * the combination used by other modern (non-legacy) ptimer clients
     * such as hw/timer/armv7m_systick.c and hw/timer/cmsdk-apb-timer.c.
     * It gives straightforward "counter decrements to 0, then fires and
     * reloads" behaviour with an accurate (not off-by-one) counter value,
     * which matches the AM335x DMTIMER's documented behaviour closely
     * enough for clockevent/clocksource use.
     */
    s->timer = ptimer_init(am335x_timer_tick, s,
                           PTIMER_POLICY_WRAP_AFTER_ONE_PERIOD |
                           PTIMER_POLICY_NO_COUNTER_ROUND_DOWN |
                           PTIMER_POLICY_NO_IMMEDIATE_RELOAD |
                           PTIMER_POLICY_TRIGGER_ONLY_ON_DECREMENT);

    ptimer_transaction_begin(s->timer);
    ptimer_set_freq(s->timer, AM335X_TIMER_FREQ);
    ptimer_transaction_commit(s->timer);
}

static const Property am335x_timer_properties[] = {
    DEFINE_PROP_BOOL("one-ms", AM335xTimerState, one_ms, false),
};

static void am335x_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = am335x_timer_realize;
    device_class_set_legacy_reset(dc, am335x_timer_dev_reset);
    dc->vmsd = &am335x_timer_vmstate;
    device_class_set_props(dc, am335x_timer_properties);
    dc->desc = "TI AM335x DMTIMER";
}

static const TypeInfo am335x_timer_info = {
    .name = TYPE_AM335X_TIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xTimerState),
    .class_init = am335x_timer_class_init,
};

static void am335x_timer_register_types(void)
{
    type_register_static(&am335x_timer_info);
}

type_init(am335x_timer_register_types)
