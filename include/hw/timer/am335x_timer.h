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
};

#endif /* HW_TIMER_AM335X_TIMER_H */
