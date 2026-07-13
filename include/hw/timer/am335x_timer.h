/*
 * TI AM335x DMTIMER
 *
 * QEMU model of the AM335x General Purpose / DM Timer block, sufficient to
 * deliver a periodic overflow interrupt so that the Linux kernel's
 * clockevent/clocksource (omap_dm_timer / clockevent-ti-dm) can be probed
 * and started.
 *
 * This code is licensed under GPL version 2 or later. See the COPYING
 * file in the top-level directory.
 */

#ifndef HW_TIMER_AM335X_TIMER_H
#define HW_TIMER_AM335X_TIMER_H

#include "hw/sysbus.h"
#include "hw/ptimer.h"
#include "qom/object.h"

#define TYPE_AM335X_TIMER "am335x-timer"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xTimerState, AM335X_TIMER)

/* Fixed functional-clock frequency (Hz). The guest clock tree is stubbed,
 * so DMTIMER runs at a plausible fixed rate -- only interrupt delivery
 * matters, not real-world timing accuracy. */
#define AM335X_TIMER_FREQ (24000000)

struct AM335xTimerState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;
    ptimer_state *timer;

    /*
     * True for the "1ms" DMTIMER variant (DMTIMER1, "ti,am335x-timer-1ms")
     * whose OCP wrapper uses the OMAP2-style SYSCONFIG layout: SOFTRESET is
     * bit 1 and bit 0 is AUTOIDLE. False for the regular OMAP4-style timers
     * where SOFTRESET is bit 0. Set as a device property by the SoC.
     */
    bool one_ms;

    uint32_t tiocp_cfg;
    uint32_t irqstatus_raw;
    uint32_t irqstatus;
    uint32_t irqenable;
    uint32_t irqwakeen;
    uint32_t tclr;
    uint32_t tldr;
    uint32_t tmar;
    uint32_t tcar1;
    uint32_t tcar2;
    uint32_t tsicr;

    /*
     * Virtual-clock time base for the guest-visible up-counter (TCRR).
     * The counter is derived directly from QEMU_CLOCK_VIRTUAL rather than
     * read back from the internal ptimer: TCRR at base_time is base_tcrr,
     * and it counts up at the functional-clock rate, wrapping through
     * 0x100000000 back to TLDR. This makes the free-running clocksource
     * use case (a full 2^32 counter read for ktime/sched_clock) wrap
     * cleanly and indefinitely, which the ptimer's periodic reload cannot
     * do for a 2^32-tick period. The ptimer is retained only to schedule
     * the overflow (OVF) interrupt.
     */
    int64_t base_time;
    uint32_t base_tcrr;
};

#endif /* HW_TIMER_AM335X_TIMER_H */
