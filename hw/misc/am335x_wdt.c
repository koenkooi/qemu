/*
 * TI AM335x Watchdog Timer (WDT1) emulation.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Structural template: hw/misc/am335x_prcm.c / hw/timer/am335x_timer.c.
 * A flat, per-register store for the WDT1 block (TRM spruh73q ch.20.4,
 * base 0x44E35000) with two synthesized read-side transforms layered on
 * top of otherwise plain read-back-what-was-written storage:
 *
 *  - WD_SYSSTATUS (WDST, 0x14) bit0 RESETDONE always reads back 1. Linux's
 *    ti-sysc writes SOFTRESET into WD_SYSCONFIG (WDSC, 0x10) on probe and
 *    then polls WDST.RESETDONE; against the unimplemented_device stub
 *    this replaces (which always reads 0), that poll spins until it
 *    times out after ~220s ("OCP softreset timed out ... -110"). Here it
 *    completes on the very first read.
 *  - WWPS (write-posted status, 0x3C) always reads back 0. The omap_wdt
 *    driver polls this register after every write to WCLR/WCRR/WLDR/
 *    WTGR/WSPR/WDLY to wait out the (real hardware) posted-write
 *    latency; with it hardwired to 0, those polls also complete
 *    immediately.
 *
 * WIDR (0x00) is read-only and returns a fixed, plausible non-zero OMAP
 * watchdog revision; the driver logs it but does not gate behaviour on
 * its exact value.
 *
 * This model never starts a countdown and never resets the guest: the
 * WSPR/WCRR/WLDR/WTGR/WDLY writes that make up the omap_wdt start/stop
 * and reload sequences are simply stored. The goal is only for the
 * guest's watchdog driver to probe and (typically) disable WDT1 without
 * stalling module reset or ever biting.
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
#include "qemu/log.h"
#include "qemu/module.h"

/* Register offsets (TRM spruh73q ch.20.4). */
#define WIDR            0x00
#define WDSC            0x10
#define WDST            0x14
#define WISR            0x24
#define WIER            0x28
#define WCLR            0x2C
#define WCRR            0x30
#define WLDR            0x34
#define WTGR            0x38
#define WWPS            0x3C
#define WDLY            0x44
#define WSPR            0x48
#define WIRQSTATRAW     0x50
#define WIRQSTAT        0x54
#define WIRQWAKEEN      0x58

/* WDSC (WD_SYSCONFIG) bits. */
#define WDSC_SOFTRESET  (1 << 1)

/* WDST (WD_SYSSTATUS) bits. */
#define WDST_RESETDONE  (1 << 0)

/* Plausible non-zero OMAP watchdog IP revision; Linux logs this but does
 * not validate its contents. */
#define AM335X_WDT_WIDR_VALUE 0x0000012A

/* --- Reset --------------------------------------------------------------- */

static void am335x_wdt_reset(DeviceState *dev)
{
    AM335xWdtState *s = AM335X_WDT(dev);

    s->wdsc = 0;
    s->wisr = 0;
    s->wier = 0;
    s->wclr = 0;
    s->wcrr = 0;
    s->wldr = 0;
    s->wtgr = 0;
    s->wdly = 0;
    s->wspr = 0;
    s->wirqstatraw = 0;
    s->wirqstat = 0;
    s->wirqwakeen = 0;
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
        return s->wcrr;
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
    case WIRQWAKEEN:
        return s->wirqwakeen;
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
        s->wclr = v;
        break;
    case WCRR:
        s->wcrr = v;
        break;
    case WLDR:
        s->wldr = v;
        break;
    case WTGR:
        s->wtgr = v;
        break;
    case WWPS:
        /* read-only */
        break;
    case WDLY:
        s->wdly = v;
        break;
    case WSPR:
        /* Start/stop key sequence (0xAAAA, then 0x5555 or 0x0000/0x5555).
         * Stored only -- this model never actually arms or bites the
         * watchdog. */
        s->wspr = v;
        break;
    case WIRQSTATRAW:
        s->wirqstatraw = v;
        break;
    case WIRQSTAT:
        s->wirqstat = v;
        break;
    case WIRQWAKEEN:
        s->wirqwakeen = v;
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
}

static const VMStateDescription vmstate_am335x_wdt = {
    .name = TYPE_AM335X_WDT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(wdsc, AM335xWdtState),
        VMSTATE_UINT32(wisr, AM335xWdtState),
        VMSTATE_UINT32(wier, AM335xWdtState),
        VMSTATE_UINT32(wclr, AM335xWdtState),
        VMSTATE_UINT32(wcrr, AM335xWdtState),
        VMSTATE_UINT32(wldr, AM335xWdtState),
        VMSTATE_UINT32(wtgr, AM335xWdtState),
        VMSTATE_UINT32(wdly, AM335xWdtState),
        VMSTATE_UINT32(wspr, AM335xWdtState),
        VMSTATE_UINT32(wirqstatraw, AM335xWdtState),
        VMSTATE_UINT32(wirqstat, AM335xWdtState),
        VMSTATE_UINT32(wirqwakeen, AM335xWdtState),
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
    .class_init    = am335x_wdt_class_init,
};

static void am335x_wdt_register_types(void)
{
    type_register_static(&am335x_wdt_info);
}

type_init(am335x_wdt_register_types)
