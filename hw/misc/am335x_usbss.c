/*
 * TI AM335x USB Subsystem (USBSS) "clean probe" stub.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Structural template: hw/misc/am335x_control.c (flat backing store with a
 * few synthesized registers layered on top). Covers the whole 32KB "usb"
 * target-module window (TRM spruh73q ch.16, base 0x47400000,
 * am33xx.dtsi target-module@47400000), which holds four sub-blocks:
 *
 *   0x0000 ti-sysc target-module wrapper (rev @0x00, sysconfig @0x10)
 *   0x1000 USB0 "control" wrapper (revision/control/mode/phy_utmi/...)
 *   0x1300 USB0 PHY window (never ioremap'd by any Linux driver, see below)
 *   0x1400 USB0 "mc" block -- the Mentor musb-hdrc core registers
 *   0x1800 USB1 "control" wrapper
 *   0x1b00 USB1 PHY window (likewise unused)
 *   0x1c00 USB1 "mc" block
 *   0x2000 CPPI4.1 DMA glue/controller/scheduler/queue-manager (out of scope)
 *
 * Goal and non-goal
 * ------------------
 * This model exists to let drivers/usb/musb/{musb_dsps,musb_core}.c probe
 * *cleanly* for both instances -- musb_init_controller() completing and
 * the controller registering a host or gadget instance (usb_add_hcd() /
 * usb_add_gadget_udc()) -- without modelling USB device enumeration,
 * transfers, or the CPPI4.1 DMA engine. No USBBus/USBPort is created here;
 * a real host/gadget model is a separate, future project.
 *
 * The one load-bearing register
 * ------------------------------
 * Today, with nothing mapped at 0x47400000, dsps_musb_init() (musb_dsps.c)
 * reads the USBnREV "revision" register (control-wrapper offset 0x00,
 * abs 0x47401000/0x47401800) as part of its probe, gets back the
 * unassigned-memory value of 0, and bails:
 *
 *   rev = musb_readl(reg_base, wrp->revision);
 *   if (!rev)
 *           return -ENODEV;         // "Returns zero if e.g. not clocked"
 *
 * Real silicon reports 0x4EA20800 here (TRM Table 16-73, SS16.4.2.1); that
 * is the one register whose exact value actually gates anything. Everything
 * else below is either read-write scratch the driver never gates on, or a
 * handful of extra registers needed so musb_core_init() (musb_core.c)
 * finishes instead of tripping a *different* early return once the first
 * gate is cleared:
 *
 *   - CONFIGDATA (an AM335x/DSPS "indexed" register living at mc+0x1F,
 *     musb_read_configdata() in musb_regs.h) must read back with DYNFIFO
 *     (bit2) set, or musb_core_init() takes the ep_config_from_hw() branch
 *     that probes real per-endpoint TXMAXP/FIFOSIZE registers this model
 *     does not implement. This device hardcodes mc+0x1F rather than
 *     implementing full 16-endpoint INDEX-register muxing (MUSB_INDEXED_EP,
 *     musb_dsps.c:692) -- clean-probe is the only indexed access musb_core
 *     performs before falling into the pure-software ep_config_from_table()
 *     path (fifo_setup() writes TXFIFOSZ/TXFIFOADD/RXFIFOSZ/RXFIFOADD at
 *     *fixed* mc offsets 0x62/0x64/0x63/0x66, not through the indexed
 *     window, so no further muxing is ever needed for probe to complete).
 *     0xde (MPRXE|MPTXE|HBRXE|HBTXE|DYNFIFO|SOFTCONE) is the commonly-cited
 *     real DSPS CONFIGDATA value; the TRM does not publish a formal bit
 *     table for this Mentor-licensed register (see musb_regs.h), so this is
 *     a pragmatic match to known-good silicon rather than a TRM citation.
 *   - USBnPHY_UTMI/USBnMODE reset to their documented TRM values (Table
 *     16-73, SS16.4.2.35): OTGDISABLE set (dsps_musb_init() explicitly
 *     clears it) and IDDIG=1 (peripheral/B-device default, which
 *     dsps_musb_set_mode() flips for USB1's host dr_mode).
 *
 * ti-sysc SOFTRESET and the per-instance SOFT_RESET bit self-clear on
 * write, mirroring the WDT1/I2C0 idiom elsewhere in this SoC
 * (am335x_wdt.c, am335x_i2c.c): the "usb" target-module is TI_SYSC_OMAP4
 * type with ti,sysc-mask selecting SOFTRESET but no ti,syss-mask/"syss"
 * reg-name, so drivers/bus/ti-sysc.c infers SYSC_QUIRK_RESET_STATUS and
 * polls the *sysconfig* register itself (sysc_poll_reset_sysconfig())
 * rather than a separate status register; without the self-clear this
 * spins for the full 10ms MAX_MODULE_SOFTRESET_WAIT and logs "OCP
 * softreset timed out" on every boot.
 *
 * What is deliberately left alone
 * --------------------------------
 * - The two USBn PHY windows (abs 0x47401300/0x47401b00, DT
 *   "usb-phy@1300"/"usb-phy@1b00") are never ioremap'd by phy-am335x.c --
 *   that driver only pokes the SoC Control Module via its "ti,ctrl_mod"
 *   phandle (0x44E10620/0x628/0x648), already modelled by
 *   hw/misc/am335x_control.c. They fall through to the generic backing
 *   store here like any other unused offset.
 * - The CPPI4.1 DMA glue/controller/scheduler/queue-manager (0x2000-0x7fff)
 *   is backed by the same flat store, reading as zero like the
 *   create_unimplemented_device() stub it replaces. drivers/dma/ti/cppi41.c
 *   (a separate module, "ti,am3359-cppi41") does unconditional register
 *   writes during its own probe with no blocking status polls, so this is
 *   sufficient for its probe to complete without a dedicated model; if
 *   musb's own DMA controller creation (dsps_dma_controller_create(),
 *   gated behind the "use_dma" module parameter, musb_core.c:2473) can't
 *   get channels before cppi41 finishes probing, musb_init_controller()
 *   simply returns -EPROBE_DEFER and is retried -- ordinary deferred-probe
 *   behaviour, not an error, and identical to what happens on real
 *   hardware since cppi41.ko is a module there too.
 * - MUSB_DEVCTL/BABBLE_CTL/INTRTX/INTRRX/FIFO ports/busctl/ULPI/EPINFO etc. are
 *   plain read-write-back scratch; dsps_check_status()'s periodic timer
 *   pokes DEVCTL.SESSION forever (no device ever attaches, so it never
 *   observes a connect) exactly as it would on a real, otherwise-idle USB
 *   port with nothing plugged in.
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
#include "hw/misc/am335x_usbss.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"

/* ti-sysc target-module wrapper (relative to the 0x47400000 base). */
#define USBSS_SYSCONFIG          0x10
#define USBSS_SYSCONFIG_SOFTRESET (1 << 0)

/* Per-instance "control" wrapper windows (am33xx_driver_data,
 * musb_dsps.c:926-951) and "mc" (musb core) windows -- both relative to
 * the 0x47400000 base. */
#define USB0_CTRL_BASE   0x1000
#define USB1_CTRL_BASE   0x1800
#define USB0_MC_BASE     0x1400
#define USB1_MC_BASE     0x1c00

/* Offsets within a "control" wrapper block. */
#define CTRL_REVISION    0x00
#define CTRL_CONTROL     0x14
#define CTRL_PHY_UTMI    0xe0
#define CTRL_MODE        0xe8

#define CTRL_CONTROL_SOFT_RESET  (1 << 0)

/* Offset within an "mc" block: the AM335x/DSPS indexed EP0
 * FIFOSIZE/CONFIGDATA register (musb_read_configdata(), musb_regs.h). */
#define MC_CONFIGDATA    0x1f

/* Reset/fixed values (TRM spruh73q Table 16-73 unless noted). */
#define USB_REVISION_VALUE    0x4ea20800  /* SS16.4.2.1 */
#define USB_PHY_UTMI_RESET    0x00200002  /* SS16.4.2.35: OTGDISABLE|FSDATAEXT */
#define USB_MODE_RESET        0x00000100  /* SS16.4.2.35: IDDIG (B-device) */
#define USB_CONFIGDATA_VALUE  0xde        /* MPRXE|MPTXE|HBRXE|HBTXE|DYNFIFO|SOFTCONE; see file header */

/* --- Flat byte-store helpers ------------------------------------------- */

static uint64_t am335x_usbss_load(const uint8_t *regs, hwaddr offset,
                                  unsigned size)
{
    switch (size) {
    case 1:
        return (uint8_t)ldub_p(regs + offset);
    case 2:
        return (uint16_t)lduw_le_p(regs + offset);
    case 4:
        return (uint32_t)ldl_le_p(regs + offset);
    default:
        return 0;
    }
}

static void am335x_usbss_store(uint8_t *regs, hwaddr offset, uint64_t value,
                               unsigned size)
{
    switch (size) {
    case 1:
        stb_p(regs + offset, (uint8_t)value);
        break;
    case 2:
        stw_le_p(regs + offset, (uint16_t)value);
        break;
    case 4:
        stl_le_p(regs + offset, (uint32_t)value);
        break;
    default:
        break;
    }
}

/* --- MMIO ------------------------------------------------------------------ */

static uint64_t am335x_usbss_read(void *opaque, hwaddr offset, unsigned size)
{
    AM335xUsbssState *s = AM335X_USBSS(opaque);

    if (offset + size > AM335X_USBSS_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: read outside window at offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_USBSS, offset);
        return 0;
    }

    /*
     * USBnREV: the one register that gates dsps_musb_init(); see the file
     * header. Always a plain 32-bit musb_readl() in the driver, but answer
     * any width to stay well-defined for a stray probe/debugfs peek.
     */
    if (offset == USB0_CTRL_BASE + CTRL_REVISION ||
        offset == USB1_CTRL_BASE + CTRL_REVISION) {
        return extract64(USB_REVISION_VALUE, 0, size * 8);
    }

    /* CONFIGDATA: read-only IP-configuration byte, see the file header. */
    if (offset == USB0_MC_BASE + MC_CONFIGDATA ||
        offset == USB1_MC_BASE + MC_CONFIGDATA) {
        return USB_CONFIGDATA_VALUE;
    }

    return am335x_usbss_load(s->regs, offset, size);
}

static void am335x_usbss_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    AM335xUsbssState *s = AM335X_USBSS(opaque);

    if (offset + size > AM335X_USBSS_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write outside window at offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_USBSS, offset);
        return;
    }

    /* USBnREV and CONFIGDATA are read-only. */
    if (offset == USB0_CTRL_BASE + CTRL_REVISION ||
        offset == USB1_CTRL_BASE + CTRL_REVISION ||
        offset == USB0_MC_BASE + MC_CONFIGDATA ||
        offset == USB1_MC_BASE + MC_CONFIGDATA) {
        return;
    }

    am335x_usbss_store(s->regs, offset, value, size);

    /*
     * USBnCTRL.SOFT_RESET (bit0) and the ti-sysc wrapper's
     * SYSCONFIG.SOFTRESET (bit0) both self-clear; see the file header.
     * Neither driver actually polls for this today, but modelling it
     * keeps a debugfs/regdump peek honest and avoids the 10ms ti-sysc
     * softreset-timeout path if that ever changes. Clearing byte 0 of
     * the stored word is correct regardless of the access width, since
     * any write that lands exactly on the register's base offset
     * necessarily includes its low byte.
     */
    if (offset == USB0_CTRL_BASE + CTRL_CONTROL ||
        offset == USB1_CTRL_BASE + CTRL_CONTROL) {
        s->regs[offset] &= ~CTRL_CONTROL_SOFT_RESET;
    } else if (offset == USBSS_SYSCONFIG) {
        s->regs[offset] &= ~USBSS_SYSCONFIG_SOFTRESET;
    }
}

static const MemoryRegionOps am335x_usbss_ops = {
    .read = am335x_usbss_read,
    .write = am335x_usbss_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* --- QOM ------------------------------------------------------------------ */

static void am335x_usbss_reset(DeviceState *dev)
{
    AM335xUsbssState *s = AM335X_USBSS(dev);

    memset(s->regs, 0, AM335X_USBSS_SIZE);

    stl_le_p(s->regs + USB0_CTRL_BASE + CTRL_PHY_UTMI, USB_PHY_UTMI_RESET);
    stl_le_p(s->regs + USB1_CTRL_BASE + CTRL_PHY_UTMI, USB_PHY_UTMI_RESET);
    stl_le_p(s->regs + USB0_CTRL_BASE + CTRL_MODE, USB_MODE_RESET);
    stl_le_p(s->regs + USB1_CTRL_BASE + CTRL_MODE, USB_MODE_RESET);

    qemu_set_irq(s->irq[0], 0);
    qemu_set_irq(s->irq[1], 0);
}

static void am335x_usbss_realize(DeviceState *dev, Error **errp)
{
    AM335xUsbssState *s = AM335X_USBSS(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->regs = g_malloc0(AM335X_USBSS_SIZE);

    memory_region_init_io(&s->iomem, OBJECT(s), &am335x_usbss_ops, s,
                          TYPE_AM335X_USBSS, AM335X_USBSS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    /* Per-instance musb "mc" interrupt outputs -> INTC 18 (USB0)/19 (USB1). */
    sysbus_init_irq(sbd, &s->irq[0]);
    sysbus_init_irq(sbd, &s->irq[1]);
}

static void am335x_usbss_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = am335x_usbss_realize;
    device_class_set_legacy_reset(dc, am335x_usbss_reset);
    /*
     * No vmsd: heap-allocated backing store, and the beaglebone-black
     * machine is not migratable (SoC uses serial_hd(), user_creatable =
     * false) -- same rationale as am335x_control.c.
     */
    dc->desc = "TI AM335x USB Subsystem (USBSS, clean-probe stub)";
}

static const TypeInfo am335x_usbss_info = {
    .name          = TYPE_AM335X_USBSS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xUsbssState),
    .class_init    = am335x_usbss_class_init,
};

static void am335x_usbss_register_types(void)
{
    type_register_static(&am335x_usbss_info);
}

type_init(am335x_usbss_register_types)
