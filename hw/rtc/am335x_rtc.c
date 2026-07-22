/*
 * TI AM335x RTC ("ti,am3352-rtc") emulation.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * A minimal model of the AM335x real-time clock (TRM spruh73q ch.20,
 * base 0x44E3E000) covering exactly what the Linux rtc-omap driver
 * touches at probe:
 *
 *  - A BCD calendar (SECONDS..YEARS), seeded once from host wall-clock
 *    time at reset so an in-guest `date`/`hwclock -r` reads a sane value.
 *    The clock does not tick on its own (no periodic timer is modelled);
 *    values simply read back what was seeded or last written.
 *  - KICK0/KICK1 register-write-protection is accepted but not enforced;
 *    the driver performs the unlock->write->lock dance around every update,
 *    so an always-writable register file is guest-visibly identical.
 *  - CTRL.STOP_RTC (bit0, 1 = running) is stored; STATUS.RUN (bit1) mirrors
 *    it and STATUS.BUSY (bit0) always reads 0, so rtc_wait_not_busy()
 *    returns on the first poll.
 *
 * There is deliberately no OCP softreset handling: the RTC's ti-sysc
 * wrapper is "ti,sysc-omap4-simple" (srst_shift = -ENODEV), so no
 * SYSS.RESETDONE poll ever runs against this module.
 *
 * The two interrupt outputs (periodic line 75, alarm line 76) are wired
 * so devm_request_irq() succeeds but are never asserted.
 *
 * RTC_PMIC (0x98) "system-power-controller" poweroff (drivers/rtc/
 * rtc-omap.c omap_rtc_power_off(), registered as pm_power_off because the
 * BeagleBone DT's &rtc has "system-power-controller"): on real hardware
 * this arms an ALARM2 event ~1s out and sets PMIC_POWER_EN, which drives
 * an external pin that cuts board power -- omap_rtc_power_off() then
 * mdelay(1500)s and never returns because the SoC loses power mid-wait.
 * This model doesn't tick the calendar (see above), so there is no
 * meaningful "1 second later" to wait for; instead, the write of the
 * exact bit combination that ONLY omap_rtc_power_off() (not the shared
 * omap_rtc_power_off_program() helper, also called from the RTC-only
 * suspend path in drivers/soc/ti/pm33xx.c, which sets just POWER_EN_EN)
 * produces -- PMIC_POWER_EN_EN | EXT_WKUP_POL(0) | EXT_WKUP_EN(0), bits
 * {16,4,0} -- is treated as the trigger. Without this, mdelay(1500)
 * returns normally, kernel_power_off() returns, sys_reboot() falls
 * through to do_exit(0) on PID 1, and the kernel panics with "Attempted
 * to kill init!".
 *
 * The trigger calls exit(0) directly rather than
 * qemu_system_powerdown_request(): the latter only sets a flag for the
 * main loop to notice, which lost the race in testing -- the guest's own
 * mdelay(1500) busy-wait (no HLT/WFI, so nothing yields) kept running for
 * >400ms and hit the do_exit(0)/panic path before the async shutdown was
 * processed. hw/watchdog/watchdog.c's WATCHDOG_ACTION_POWEROFF faces the
 * identical "must not race with continuing guest execution" problem and
 * uses the same exit(0) pattern for the same reason.
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
#include "hw/rtc/am335x_rtc.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "block/block-global-state.h"
#include "qemu/bcd.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/rtc.h"

/* Register offsets (drivers/rtc/rtc-omap.c; TRM spruh73q §20.3.5). */
#define RTC_SECONDS         0x00
#define RTC_MINUTES         0x04
#define RTC_HOURS           0x08
#define RTC_DAYS            0x0c
#define RTC_MONTHS          0x10
#define RTC_YEARS           0x14
#define RTC_WEEKS           0x18
#define RTC_ALARM_SECONDS   0x20
#define RTC_ALARM_MINUTES   0x24
#define RTC_ALARM_HOURS     0x28
#define RTC_ALARM_DAYS      0x2c
#define RTC_ALARM_MONTHS    0x30
#define RTC_ALARM_YEARS     0x34
#define RTC_CTRL            0x40
#define RTC_STATUS          0x44
#define RTC_INTERRUPTS      0x48
#define RTC_COMP_LSB        0x4c
#define RTC_COMP_MSB        0x50
#define RTC_OSC             0x54
#define RTC_SCRATCH0        0x60
#define RTC_SCRATCH1        0x64
#define RTC_SCRATCH2        0x68
#define RTC_KICK0           0x6c
#define RTC_KICK1           0x70
#define RTC_REVISION        0x74
#define RTC_SYSCONFIG       0x78
#define RTC_IRQWAKEEN       0x7c
#define RTC_ALARM2_SECONDS  0x80
#define RTC_ALARM2_MINUTES  0x84
#define RTC_ALARM2_HOURS    0x88
#define RTC_ALARM2_DAYS     0x8c
#define RTC_ALARM2_MONTHS   0x90
#define RTC_ALARM2_YEARS    0x94
#define RTC_PMIC            0x98

/* RTC_CTRL / RTC_STATUS bits. */
#define RTC_CTRL_RUN        (1 << 0)   /* STOP_RTC: 1 = running */
#define RTC_STATUS_BUSY     (1 << 0)
#define RTC_STATUS_RUN      (1 << 1)

/* Read-only reset constants (TRM spruh73q). */
#define RTC_REVISION_VALUE  0x4EB00904
#define RTC_OSC_VALUE       0x00000010
#define RTC_SYSCONFIG_VALUE 0x00000002

/*
 * RTC_PMIC bits (drivers/rtc/rtc-omap.c OMAP_RTC_PMIC_*). The exact
 * combination omap_rtc_power_off() writes -- POWER_EN_EN plus
 * EXT_WKUP_POL(0)/EXT_WKUP_EN(0) -- is the poweroff signature; see the
 * file header.
 */
#define RTC_PMIC_POWER_EN_EN     (1u << 16)
#define RTC_PMIC_EXT_WKUP_POL0   (1u << 4)
#define RTC_PMIC_EXT_WKUP_EN0    (1u << 0)
#define RTC_PMIC_POWEROFF_SIG    (RTC_PMIC_POWER_EN_EN | RTC_PMIC_EXT_WKUP_POL0 | \
                                  RTC_PMIC_EXT_WKUP_EN0)

static uint64_t am335x_rtc_read(void *opaque, hwaddr offset, unsigned size)
{
    AM335xRtcState *s = AM335X_RTC(opaque);

    switch (offset) {
    case RTC_SECONDS:
        return s->seconds;
    case RTC_MINUTES:
        return s->minutes;
    case RTC_HOURS:
        return s->hours;
    case RTC_DAYS:
        return s->days;
    case RTC_MONTHS:
        return s->months;
    case RTC_YEARS:
        return s->years;
    case RTC_WEEKS:
        return s->weeks;
    case RTC_ALARM_SECONDS ... RTC_ALARM_YEARS:
        return s->alarm[(offset - RTC_ALARM_SECONDS) >> 2];
    case RTC_ALARM2_SECONDS ... RTC_ALARM2_YEARS:
        return s->alarm2[(offset - RTC_ALARM2_SECONDS) >> 2];
    case RTC_CTRL:
        return s->ctrl;
    case RTC_STATUS:
        /* RUN mirrors CTRL.STOP_RTC; BUSY is always clear. */
        return (s->status & ~(RTC_STATUS_RUN | RTC_STATUS_BUSY)) |
               ((s->ctrl & RTC_CTRL_RUN) ? RTC_STATUS_RUN : 0);
    case RTC_INTERRUPTS:
        return s->interrupts;
    case RTC_COMP_LSB:
        return s->comp_lsb;
    case RTC_COMP_MSB:
        return s->comp_msb;
    case RTC_OSC:
        return s->osc;
    case RTC_SCRATCH0:
    case RTC_SCRATCH1:
    case RTC_SCRATCH2:
        return s->scratch[(offset - RTC_SCRATCH0) >> 2];
    case RTC_KICK0:
        return s->kick0;
    case RTC_KICK1:
        return s->kick1;
    case RTC_REVISION:
        return RTC_REVISION_VALUE;
    case RTC_SYSCONFIG:
        return s->sysconfig;
    case RTC_IRQWAKEEN:
        return s->irqwakeen;
    case RTC_PMIC:
        return s->pmic;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_RTC, offset);
        return 0;
    }
}

static void am335x_rtc_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    AM335xRtcState *s = AM335X_RTC(opaque);
    uint32_t v = (uint32_t)value;

    switch (offset) {
    case RTC_SECONDS:
        s->seconds = v & 0xff;
        break;
    case RTC_MINUTES:
        s->minutes = v & 0xff;
        break;
    case RTC_HOURS:
        s->hours = v & 0xff;
        break;
    case RTC_DAYS:
        s->days = v & 0xff;
        break;
    case RTC_MONTHS:
        s->months = v & 0xff;
        break;
    case RTC_YEARS:
        s->years = v & 0xff;
        break;
    case RTC_WEEKS:
        s->weeks = v & 0xff;
        break;
    case RTC_ALARM_SECONDS ... RTC_ALARM_YEARS:
        s->alarm[(offset - RTC_ALARM_SECONDS) >> 2] = v & 0xff;
        break;
    case RTC_ALARM2_SECONDS ... RTC_ALARM2_YEARS:
        s->alarm2[(offset - RTC_ALARM2_SECONDS) >> 2] = v & 0xff;
        break;
    case RTC_CTRL:
        s->ctrl = v;
        break;
    case RTC_STATUS:
        /* Event bits [7:2] are write-1-to-clear; RUN/BUSY are read-only. */
        s->status &= ~(v & 0xfc);
        break;
    case RTC_INTERRUPTS:
        s->interrupts = v;
        break;
    case RTC_COMP_LSB:
        s->comp_lsb = v;
        break;
    case RTC_COMP_MSB:
        s->comp_msb = v;
        break;
    case RTC_OSC:
        s->osc = v;
        break;
    case RTC_SCRATCH0:
    case RTC_SCRATCH1:
    case RTC_SCRATCH2:
        s->scratch[(offset - RTC_SCRATCH0) >> 2] = v;
        break;
    case RTC_KICK0:
        /* Write protection is not enforced; store for vmstate/debug only. */
        s->kick0 = v;
        break;
    case RTC_KICK1:
        s->kick1 = v;
        break;
    case RTC_REVISION:
        /* read-only */
        break;
    case RTC_SYSCONFIG:
        s->sysconfig = v;
        break;
    case RTC_IRQWAKEEN:
        s->irqwakeen = v;
        break;
    case RTC_PMIC:
        s->pmic = v;
        if ((v & RTC_PMIC_POWEROFF_SIG) == RTC_PMIC_POWEROFF_SIG) {
            /*
             * exit(0) bypasses qemu_cleanup(), which is normally what
             * flushes block backends (qcow2's own metadata caches, not
             * just the guest's OS-level page cache) to disk. Flush
             * synchronously first so guest writes actually land -- this
             * is a bounded call, not the async shutdown sequence that
             * lost the race against mdelay(1500) (see above), so it
             * doesn't reintroduce that bug. Callable here: MMIO write
             * callbacks always run on a vCPU thread holding the BQL,
             * which qemu_in_main_thread() (what GLOBAL_STATE_CODE()
             * checks) treats as the block layer's main thread for
             * system emulators.
             */
            bdrv_flush_all();
            exit(0);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_RTC, offset);
        break;
    }
}

static const MemoryRegionOps am335x_rtc_ops = {
    .read = am335x_rtc_read,
    .write = am335x_rtc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    /* rtc-omap uses 8-bit accesses for the BCD calendar and 32-bit for
     * the control/status/scratch registers. */
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void am335x_rtc_reset(DeviceState *dev)
{
    AM335xRtcState *s = AM335X_RTC(dev);
    struct tm now;

    /* Seed the BCD calendar from host wall-clock time (24h). The rtc-omap
     * driver reconstructs the year as bcd2bin(YEARS) + 100, i.e. the
     * register holds the year within the century. */
    qemu_get_timedate(&now, 0);
    s->seconds = to_bcd(now.tm_sec);
    s->minutes = to_bcd(now.tm_min);
    s->hours   = to_bcd(now.tm_hour);
    s->days    = to_bcd(now.tm_mday);
    s->months  = to_bcd(now.tm_mon + 1);
    s->years   = to_bcd(now.tm_year % 100);
    s->weeks   = to_bcd(now.tm_wday);

    memset(s->alarm, 0, sizeof(s->alarm));
    memset(s->alarm2, 0, sizeof(s->alarm2));
    /* ALARM2 DAYS/MONTHS reset to 1 (poweroff-program defaults). */
    s->alarm2[3] = 1;
    s->alarm2[4] = 1;

    s->ctrl = 0;
    s->status = 0;
    s->interrupts = 0;
    s->comp_lsb = 0;
    s->comp_msb = 0;
    s->osc = RTC_OSC_VALUE;
    memset(s->scratch, 0, sizeof(s->scratch));
    s->kick0 = 0;
    s->kick1 = 0;
    s->sysconfig = RTC_SYSCONFIG_VALUE;
    s->irqwakeen = 0;
    s->pmic = 0;
}

static void am335x_rtc_init(Object *obj)
{
    AM335xRtcState *s = AM335X_RTC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &am335x_rtc_ops, s,
                          TYPE_AM335X_RTC, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq_timer);
    sysbus_init_irq(sbd, &s->irq_alarm);
}

static const VMStateDescription vmstate_am335x_rtc = {
    .name = TYPE_AM335X_RTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(seconds, AM335xRtcState),
        VMSTATE_UINT8(minutes, AM335xRtcState),
        VMSTATE_UINT8(hours, AM335xRtcState),
        VMSTATE_UINT8(days, AM335xRtcState),
        VMSTATE_UINT8(months, AM335xRtcState),
        VMSTATE_UINT8(years, AM335xRtcState),
        VMSTATE_UINT8(weeks, AM335xRtcState),
        VMSTATE_UINT8_ARRAY(alarm, AM335xRtcState, 6),
        VMSTATE_UINT8_ARRAY(alarm2, AM335xRtcState, 6),
        VMSTATE_UINT32(ctrl, AM335xRtcState),
        VMSTATE_UINT32(status, AM335xRtcState),
        VMSTATE_UINT32(interrupts, AM335xRtcState),
        VMSTATE_UINT32(comp_lsb, AM335xRtcState),
        VMSTATE_UINT32(comp_msb, AM335xRtcState),
        VMSTATE_UINT32(osc, AM335xRtcState),
        VMSTATE_UINT32_ARRAY(scratch, AM335xRtcState, 3),
        VMSTATE_UINT32(kick0, AM335xRtcState),
        VMSTATE_UINT32(kick1, AM335xRtcState),
        VMSTATE_UINT32(sysconfig, AM335xRtcState),
        VMSTATE_UINT32(irqwakeen, AM335xRtcState),
        VMSTATE_UINT32(pmic, AM335xRtcState),
        VMSTATE_END_OF_LIST()
    }
};

static void am335x_rtc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, am335x_rtc_reset);
    dc->vmsd = &vmstate_am335x_rtc;
    dc->desc = "TI AM335x RTC";
}

static const TypeInfo am335x_rtc_info = {
    .name          = TYPE_AM335X_RTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xRtcState),
    .instance_init = am335x_rtc_init,
    .class_init    = am335x_rtc_class_init,
};

static void am335x_rtc_register_types(void)
{
    type_register_static(&am335x_rtc_info);
}

type_init(am335x_rtc_register_types)
