/*
 * TI AM335x Watchdog Timer (WDT1) emulation.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Structural template: hw/misc/am335x_prcm.c / hw/timer/am335x_timer.c.
 * A per-register store for the WDT1 block (TRM spruh73q ch.20.4, Table
 * 20-111, base 0x44E35000) that models the full start/stop/reload/overflow
 * lifecycle -- an armed watchdog that is never reloaded before WDT_WCRR
 * overflows genuinely resets the guest, matching real silicon.
 *
 * OFFSET CORRECTION (2026-07-21): an earlier version of this file had
 * WISR/WIER/WCLR/WCRR/WLDR/WTGR/WWPS shifted 8 bytes too high (e.g. WCLR
 * assumed at 0x2C instead of 0x24), which happened to not matter for probe
 * (WDSC/WDST at 0x10/0x14 were correct, and the watchdog was never armed
 * by default) but would have silently no-op'd every register the guest's
 * omap_wdt driver (drivers/watchdog/omap_wdt.c, DT compatible
 * "ti,omap3-wdt") actually uses to start/reload the counter. Re-verified
 * against TRM Table 20-111 (WISR=0x18, WIER=0x1C, WCLR=0x24, WCRR=0x28,
 * WLDR=0x2C, WTGR=0x30, WWPS=0x34) and cross-checked byte-for-byte against
 * drivers/watchdog/omap_wdt.h's OMAP_WATCHDOG_* offsets, which match.
 *
 * Two synthesized read-side transforms (unchanged from before, offsets
 * corrected):
 *  - WD_SYSSTATUS (WDST, 0x14) bit0 RESETDONE always reads back 1, so
 *    ti-sysc's OCP-softreset poll (SOFTRESET in WDSC @0x10) completes on
 *    the first read instead of spinning ~220s to a -110 timeout.
 *  - WWPS (write-posted status, 0x34) always reads back 0, so the
 *    omap_wdt driver's post-write latency polls (after WCLR/WCRR/WLDR/
 *    WTGR/WSPR writes) complete immediately.
 *
 * Countdown model: WCRR is not a free-running register here. Instead,
 * arming (the WSPR enable key-sequence completing) or reloading (a WTGR
 * write whose value differs from the last one, per TRM 20.4.3.7) computes
 * the overflow period from the current WCLR prescaler bits and WLDR load
 * value (TRM 20.4.3.3's OVF_Rate formula) and schedules a one-shot
 * QEMU_CLOCK_VIRTUAL timer at now + period. A live WCRR read while running
 * derives the current count from elapsed time (same idiom as
 * am335x_timer.c's TCRR), so tools that poll it (e.g. the watchdog core's
 * WDIOC_GETTIMELEFT) see a sensible value without free-running the counter
 * every tick. The WSPR disable key-sequence cancels the timer. Expiry
 * fires qemu_system_reset_request() -- a real SoC reset, exactly what an
 * unfed AM335x WDT1 does on real hardware.
 *
 * WISR/WIER/WIRQSTATRAW/WIRQSTAT/WIRQENSET/WIRQENCLR (the delay-threshold
 * / overflow interrupt path, driven by WDLY) are plain flat storage: the
 * guest's omap_wdt driver never requests WDT1's interrupt or touches these
 * registers, so there is nothing to synthesize there.
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
#include "hw/misc/am335x_wdt.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "system/runstate.h"

/* Register offsets (TRM spruh73q ch.20.4, Table 20-111). */
#define WIDR            0x00
#define WDSC            0x10
#define WDST            0x14
#define WISR            0x18
#define WIER            0x1C
#define WCLR            0x24
#define WCRR            0x28
#define WLDR            0x2C
#define WTGR            0x30
#define WWPS            0x34
#define WDLY            0x44
#define WSPR            0x48
#define WIRQSTATRAW     0x54
#define WIRQSTAT        0x58
#define WIRQENSET       0x5C
#define WIRQENCLR       0x60

/* WDSC (WD_SYSCONFIG) bits. */
#define WDSC_SOFTRESET  (1 << 1)

/* WDST (WD_SYSSTATUS) bits. */
#define WDST_RESETDONE  (1 << 0)

/* WCLR (Watchdog Control Register) bits (TRM Table 20-103). */
#define WCLR_PRE        (1 << 5)
#define WCLR_PTV_SHIFT  2
#define WCLR_PTV_MASK   0x7

/* WDT1 functional clock: fixed 32kHz (TRM 20.4.3.3). */
#define AM335X_WDT_CLK_HZ 32768

/* Plausible non-zero OMAP watchdog IP revision; Linux logs this but does
 * not validate its contents. */
#define AM335X_WDT_WIDR_VALUE 0x0000012A

/* --- Countdown / bite ------------------------------------------------- */

static uint32_t am335x_wdt_prescale(AM335xWdtState *s)
{
    if (!(s->wclr & WCLR_PRE)) {
        return 1;
    }
    return 1u << ((s->wclr >> WCLR_PTV_SHIFT) & WCLR_PTV_MASK);
}

/* TRM 20.4.3.3: OVF_Rate = (FFFF_FFFFh - WDT_WLDR + 1) * wd_clk_period * PS. */
static uint64_t am335x_wdt_period_ns(AM335xWdtState *s)
{
    uint64_t ticks = 0x100000000ULL - s->wldr;
    uint64_t scaled = ticks * am335x_wdt_prescale(s);

    return muldiv64(scaled, NANOSECONDS_PER_SECOND, AM335X_WDT_CLK_HZ);
}

static uint32_t am335x_wdt_live_wcrr(AM335xWdtState *s)
{
    int64_t elapsed_ns;
    uint64_t elapsed_ticks;

    if (!s->running) {
        return s->wcrr_stopped;
    }

    elapsed_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->reload_ns;
    elapsed_ticks = muldiv64((uint64_t)elapsed_ns, AM335X_WDT_CLK_HZ,
                             NANOSECONDS_PER_SECOND) / am335x_wdt_prescale(s);

    return (uint32_t)(s->wldr + elapsed_ticks);
}

static void am335x_wdt_bite(void *opaque)
{
    /* TRM 20.4.3.2: on WCRR overflow, an active-low reset pulse is
     * generated -- a genuine SoC reset, not a WDT1-local event. */
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static void am335x_wdt_arm(AM335xWdtState *s)
{
    s->running = true;
    s->reload_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->wcrr_stopped = s->wldr;
    timer_mod(s->bite_timer, s->reload_ns + (int64_t)am335x_wdt_period_ns(s));
}

static void am335x_wdt_disarm(AM335xWdtState *s)
{
    s->wcrr_stopped = am335x_wdt_live_wcrr(s);
    s->running = false;
    timer_del(s->bite_timer);
}

static void am335x_wdt_reload(AM335xWdtState *s)
{
    s->wcrr_stopped = s->wldr;
    if (s->running) {
        s->reload_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        timer_mod(s->bite_timer, s->reload_ns + (int64_t)am335x_wdt_period_ns(s));
    }
}

/* --- Reset --------------------------------------------------------------- */

static void am335x_wdt_reset(DeviceState *dev)
{
    AM335xWdtState *s = AM335X_WDT(dev);

    timer_del(s->bite_timer);

    s->wdsc = 0;
    s->wisr = 0;
    s->wier = 0;
    s->wclr = 0;
    s->wcrr_stopped = 0;
    s->wldr = 0;
    s->wtgr = 0;
    s->wdly = 0;
    s->wspr = 0;
    s->wirqstatraw = 0;
    s->wirqstat = 0;
    s->wirqensetclr = 0;
    s->spr_state = AM335X_WDT_WSPR_IDLE;
    s->running = false;
    s->reload_ns = 0;
}

/* --- MMIO ------------------------------------------------------------------ */

static uint64_t am335x_wdt_read(void *opaque, hwaddr offset, unsigned size)
{
    AM335xWdtState *s = AM335X_WDT(opaque);

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read size %u at offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_WDT, size, offset);
        return 0;
    }

    switch (offset) {
    case WIDR:
        return AM335X_WDT_WIDR_VALUE;
    case WDSC:
        return s->wdsc;
    case WDST:
        /* Always report the OCP softreset as complete. */
        return WDST_RESETDONE;
    case WISR:
        return s->wisr;
    case WIER:
        return s->wier;
    case WCLR:
        return s->wclr;
    case WCRR:
        return am335x_wdt_live_wcrr(s);
    case WLDR:
        return s->wldr;
    case WTGR:
        return s->wtgr;
    case WWPS:
        /* No posted writes are ever pending in this model. */
        return 0;
    case WDLY:
        return s->wdly;
    case WSPR:
        return s->wspr;
    case WIRQSTATRAW:
        return s->wirqstatraw;
    case WIRQSTAT:
        return s->wirqstat;
    case WIRQENSET:
    case WIRQENCLR:
        /* Both registers report the same live enable mask (TRM 20.4.4.1.15/.16). */
        return s->wirqensetclr;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_WDT, offset);
        return 0;
    }
}

static void am335x_wdt_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    AM335xWdtState *s = AM335X_WDT(opaque);
    uint32_t v = (uint32_t)value;

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write size %u at offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_WDT, size, offset);
        return;
    }

    switch (offset) {
    case WIDR:
        /* read-only */
        break;
    case WDSC:
        /* SOFTRESET is self-clearing; WDST.RESETDONE always reads 1 (see
         * am335x_wdt_read), so there is no separate reset-in-progress
         * state to track here. */
        s->wdsc = v & ~WDSC_SOFTRESET;
        break;
    case WDST:
        /* read-only */
        break;
    case WISR:
        s->wisr = v;
        break;
    case WIER:
        s->wier = v;
        break;
    case WCLR:
        /* Prescaler config; only takes effect at the next arm/reload
         * (TRM 20.4.3.9 requires the watchdog be stopped to change it,
         * matching real hardware -- we simply apply it lazily). */
        s->wclr = v;
        break;
    case WCRR:
        /* Direct writes are only valid while stopped on real hardware;
         * store it as the stopped baseline either way. */
        s->wcrr_stopped = v;
        break;
    case WLDR:
        s->wldr = v;
        break;
    case WTGR:
        /* TRM 20.4.3.7: the reload sequence fires whenever the written
         * value differs from the register's current content. */
        if (v != s->wtgr) {
            am335x_wdt_reload(s);
        }
        s->wtgr = v;
        break;
    case WWPS:
        /* read-only */
        break;
    case WDLY:
        s->wdly = v;
        break;
    case WSPR: {
        /* Start/stop key sequence (TRM 20.4.3.8): XXXX_AAAA then
         * XXXX_5555 disarms; XXXX_BBBB then XXXX_4444 arms. Any other
         * write sequence has no effect on the start/stop state. */
        uint32_t low = v & 0xFFFF;

        if (low == 0xAAAA) {
            s->spr_state = AM335X_WDT_WSPR_GOT_DISABLE1;
        } else if (low == 0x5555 && s->spr_state == AM335X_WDT_WSPR_GOT_DISABLE1) {
            am335x_wdt_disarm(s);
            s->spr_state = AM335X_WDT_WSPR_IDLE;
        } else if (low == 0xBBBB) {
            s->spr_state = AM335X_WDT_WSPR_GOT_ENABLE1;
        } else if (low == 0x4444 && s->spr_state == AM335X_WDT_WSPR_GOT_ENABLE1) {
            am335x_wdt_arm(s);
            s->spr_state = AM335X_WDT_WSPR_IDLE;
        } else {
            s->spr_state = AM335X_WDT_WSPR_IDLE;
        }
        s->wspr = v;
        break;
    }
    case WIRQSTATRAW:
        s->wirqstatraw = v;
        break;
    case WIRQSTAT:
        s->wirqstat = v;
        break;
    case WIRQENSET:
        s->wirqensetclr |= v;
        break;
    case WIRQENCLR:
        s->wirqensetclr &= ~v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_WDT, offset);
        break;
    }
}

static const MemoryRegionOps am335x_wdt_ops = {
    .read = am335x_wdt_read,
    .write = am335x_wdt_write,
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

/* --- QOM ------------------------------------------------------------------ */

static void am335x_wdt_init(Object *obj)
{
    AM335xWdtState *s = AM335X_WDT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &am335x_wdt_ops, s,
                          TYPE_AM335X_WDT, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);

    s->bite_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, am335x_wdt_bite, s);
}

static void am335x_wdt_finalize(Object *obj)
{
    AM335xWdtState *s = AM335X_WDT(obj);

    timer_free(s->bite_timer);
}

static const VMStateDescription vmstate_am335x_wdt = {
    .name = TYPE_AM335X_WDT,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(wdsc, AM335xWdtState),
        VMSTATE_UINT32(wisr, AM335xWdtState),
        VMSTATE_UINT32(wier, AM335xWdtState),
        VMSTATE_UINT32(wclr, AM335xWdtState),
        VMSTATE_UINT32(wcrr_stopped, AM335xWdtState),
        VMSTATE_UINT32(wldr, AM335xWdtState),
        VMSTATE_UINT32(wtgr, AM335xWdtState),
        VMSTATE_UINT32(wdly, AM335xWdtState),
        VMSTATE_UINT32(wspr, AM335xWdtState),
        VMSTATE_UINT32(wirqstatraw, AM335xWdtState),
        VMSTATE_UINT32(wirqstat, AM335xWdtState),
        VMSTATE_UINT32(wirqensetclr, AM335xWdtState),
        VMSTATE_UINT8(spr_state, AM335xWdtState),
        VMSTATE_BOOL(running, AM335xWdtState),
        VMSTATE_INT64(reload_ns, AM335xWdtState),
        VMSTATE_TIMER_PTR(bite_timer, AM335xWdtState),
        VMSTATE_END_OF_LIST()
    }
};

static void am335x_wdt_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, am335x_wdt_reset);
    dc->vmsd = &vmstate_am335x_wdt;
    dc->desc = "TI AM335x Watchdog Timer (WDT1)";
}

static const TypeInfo am335x_wdt_info = {
    .name          = TYPE_AM335X_WDT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xWdtState),
    .instance_init = am335x_wdt_init,
    .instance_finalize = am335x_wdt_finalize,
    .class_init    = am335x_wdt_class_init,
};

static void am335x_wdt_register_types(void)
{
    type_register_static(&am335x_wdt_info);
}

type_init(am335x_wdt_register_types)
