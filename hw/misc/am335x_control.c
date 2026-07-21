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
 *  - mac_id0_lo/hi (0x630/0x634, TRM 9.3.1.24/.25) mirror the QEMU-side
 *    am335x-cpsw device's actual configured MAC address. On real silicon
 *    these are read-only, factory-EFUSE-programmed registers; the AM335x
 *    cpsw driver's ti_cm_get_macid() (drivers/net/ethernet/ti/
 *    cpsw-common.c) reads them via the "syscon" phandle whenever the DT
 *    has no local-mac-address property (which am335x-bone-common.dtsi
 *    does not). Before this was modeled, the window's plain flat store
 *    read back all-zero here, is_valid_ether_addr() rejected it, and the
 *    driver fell back to eth_random_addr() -- so the guest's eth0 had a
 *    random MAC (a2:...) instead of the one QEMU actually configured for
 *    the netdev (e.g. via -device am335x-cpsw.0,mac=...). See
 *    am335x_control_set_mac_id0(), called from am335x_soc.c once the CPSW
 *    device has realized (and thus finalized its MAC address).
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
#define CONTROL_STATUS      0x040
#define CONTROL_MAC_ID0_LO  0x630
#define CONTROL_MAC_ID0_HI  0x634

/*
 * VTP0_CTRL (0xE0C, VTP0_CTRL_ADDR in u-boot arch-am33xx/hardware_am33xx.h).
 * The AM335x SPL's DDR3 impedance calibration, config_vtp()
 * (arch/arm/mach-omap2/am33xx/emif4.c), enables the VTP block, writes
 * START_EN (bit 0), then busy-waits for READY (bit 5). Real hardware sets
 * READY a few cycles after the START_EN rising edge; there is no separate
 * calibration to model here, so READY is synthesized as "set whenever
 * START_EN is set" -- the same read-side transform idiom as CONTROL_STATUS.
 */
#define CONTROL_VTP0        0xE0C
#define VTP_CTRL_START_EN   (1u << 0)
#define VTP_CTRL_READY      (1u << 5)

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
    case CONTROL_VTP0: {
        /* Synthesize the VTP calibration READY bit once START_EN is set. */
        uint32_t v = s->regs[offset / 4];
        if (v & VTP_CTRL_START_EN) {
            v |= VTP_CTRL_READY;
        }
        return v;
    }
    case CONTROL_MAC_ID0_LO:
        return ((uint32_t)s->mac_id0[5] << 8) | s->mac_id0[4];
    case CONTROL_MAC_ID0_HI:
        return ((uint32_t)s->mac_id0[3] << 24) | ((uint32_t)s->mac_id0[2] << 16) |
               ((uint32_t)s->mac_id0[1] << 8) | s->mac_id0[0];
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
    case CONTROL_MAC_ID0_LO:
    case CONTROL_MAC_ID0_HI:
        /* read-only, EFUSE-backed on real silicon (TRM 9.3.1.24/.25) */
        break;
    default:
        s->regs[offset / 4] = (uint32_t)value;
        break;
    }
}

void am335x_control_set_mac_id0(AM335xControlState *s, const uint8_t *mac)
{
    memcpy(s->mac_id0, mac, sizeof(s->mac_id0));
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
