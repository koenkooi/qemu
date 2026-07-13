/*
 * TI AM335x Control Module (System Control Module, SCM) emulation.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Structural template: hw/misc/am335x_wdt.c. The Control Module is a
 * large, mostly-flat register window (TRM spruh73q ch.9, base
 * 0x44E10000, 128KB). This model backs the whole window with a plain
 * read/write store -- which is what the Linux syscon/regmap users of
 * scm_conf (pinctrl-single, the CPSW PHY selection, RTC, DDR raminit,
 * ...) expect from their read-modify-write accesses -- and layers a
 * single functionally-important synthesized register on top:
 *
 *  - CONTROL_STATUS (0x40) bits[23:22] (SYSBOOT1) encode the board input
 *    crystal frequency. The Linux "sys_clkin_ck" ti,mux-clock (see
 *    am33xx-clocks.dtsi) selects one of {19.2, 24, 25, 26} MHz from this
 *    field. The BeagleBone Black uses a 24MHz crystal, which is SYSBOOT1
 *    == 0b01. Reporting this is what makes the kernel's derived sys_clkin
 *    rate (and hence the dmtimer clocksource rate) match the 24MHz our
 *    DMTIMER model actually runs at. Getting it wrong (the 0b00 == 19.2MHz
 *    that the previous unimplemented_device stub returned) leaves the
 *    generic sched_clock wrap-handling epoch update miscalibrated against
 *    the true counter wrap period, so sched_clock saturates at the 32-bit
 *    wrap (~223s) and every printk timestamp freezes there.
 *
 * DEVICE_ID (0x600), the JTAG/silicon-revision id, is currently read back
 * from the plain store (0) like the rest of the window; a real value can
 * be added later without affecting the clock behaviour above.
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
#include "hw/misc/am335x_control.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/module.h"

/* Register offsets (TRM spruh73q ch.9). */
#define CONTROL_STATUS  0x040

/*
 * CONTROL_STATUS.SYSBOOT1 (bits[23:22]) input-crystal selector, matching
 * the "sys_clkin_ck" ti,mux-clock parent order
 * {19.2, 24, 25, 26} MHz -> index {0, 1, 2, 3}. The BeagleBone Black has
 * a 24MHz crystal (index 1).
 */
#define CONTROL_STATUS_SYSBOOT1_SHIFT   22
#define CONTROL_STATUS_SYSBOOT1_24MHZ   (0x1u << CONTROL_STATUS_SYSBOOT1_SHIFT)

/* --- MMIO ------------------------------------------------------------------ */

static uint64_t am335x_control_read(void *opaque, hwaddr offset, unsigned size)
{
    AM335xControlState *s = AM335X_CONTROL(opaque);

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read size %u at offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_CONTROL, size, offset);
        return 0;
    }

    switch (offset) {
    case CONTROL_STATUS:
        /* Report a 24MHz input crystal (SYSBOOT1 == 0b01). The rest of
         * the SYSBOOT/boot-status field is left at 0; nothing in this
         * boot path depends on it. */
        return CONTROL_STATUS_SYSBOOT1_24MHZ;
    default:
        return s->regs[offset / 4];
    }
}

static void am335x_control_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    AM335xControlState *s = AM335X_CONTROL(opaque);

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write size %u at offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_CONTROL, size, offset);
        return;
    }

    switch (offset) {
    case CONTROL_STATUS:
        /* read-only status register */
        break;
    default:
        s->regs[offset / 4] = (uint32_t)value;
        break;
    }
}

static const MemoryRegionOps am335x_control_ops = {
    .read = am335x_control_read,
    .write = am335x_control_write,
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

static void am335x_control_reset(DeviceState *dev)
{
    AM335xControlState *s = AM335X_CONTROL(dev);

    memset(s->regs, 0, AM335X_CONTROL_SIZE);
}

static void am335x_control_realize(DeviceState *dev, Error **errp)
{
    AM335xControlState *s = AM335X_CONTROL(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->regs = g_malloc0(AM335X_CONTROL_SIZE);

    memory_region_init_io(&s->iomem, OBJECT(s), &am335x_control_ops, s,
                          TYPE_AM335X_CONTROL, AM335X_CONTROL_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void am335x_control_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = am335x_control_realize;
    device_class_set_legacy_reset(dc, am335x_control_reset);
    /*
     * No vmsd: this device carries a heap-allocated 128KB backing store
     * and the beaglebone-black machine is not migratable anyway (the SoC
     * wires up serial_hd() and is not user-creatable).
     */
    dc->desc = "TI AM335x Control Module (SCM)";
}

static const TypeInfo am335x_control_info = {
    .name          = TYPE_AM335X_CONTROL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xControlState),
    .class_init    = am335x_control_class_init,
};

static void am335x_control_register_types(void)
{
    type_register_static(&am335x_control_info);
}

type_init(am335x_control_register_types)
