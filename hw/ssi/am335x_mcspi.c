/*
 * TI AM335x McSPI0 (Multichannel Serial Port Interface) "clean probe" stub.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Structural template: hw/misc/am335x_usbss.c (flat backing store with a
 * few synthesized registers layered on top). Covers the "McSPI0
 * Registers" target-module window (TRM SPRUH73Q ch.24, base 0x48030000,
 * Table 2-3 "L4_PER Peripheral Memory Map", 4KB; DT
 * arch/arm/boot/dts/ti/omap/am33xx-l4.dtsi target-module@30000, compatible
 * "ti,sysc-omap2" wrapping a "ti,omap4-mcspi" child at the same base).
 *
 * Goal and non-goal
 * ------------------
 * This model exists to silence the recurring boot-time dmesg noise
 *
 *   ti-sysc 48030000.target-module: Reset failed with -110
 *   ti-sysc 48030000.target-module: probe with driver ti-sysc failed with
 *     error -110
 *
 * by letting the generic ti-sysc bus wrapper's OCP softreset handshake
 * complete, and then letting drivers/spi/spi-omap2-mcspi.c's
 * omap2_mcspi_probe() run to completion and register an (empty) SPI
 * controller. No SPI transfer, chip-select sequencing, FIFO, DMA, or
 * attached device is modelled -- no BeagleBone-family board in this
 * project boots from SPI, and nothing populates McSPI0/1's pins with a
 * device on the expansion headers, so there is nothing for a real
 * transaction engine or SSI bus to talk to. Same "structural, not
 * functional" bar as USBSS / wl18xx-SDIO.
 *
 * The actual root cause of the -110 (not the SPI driver's fault)
 * ------------------------------------------------------------------
 * omap2_mcspi_probe() (spi-omap2-mcspi.c:1475) itself never gates on a
 * register *value*: it only WRITEs MCSPI_WAKEUPENABLE and
 * MCSPI_MODULCTRL from omap2_mcspi_controller_setup() (line 1373), and
 * MCSPI_SYSSTATUS (0x14 relative to the driver's own regs_offset-shifted
 * base) is #define'd but never referenced anywhere else in the driver --
 * dead code left over from an older register map. DMA channel requests
 * (drivers/spi/spi-omap2-mcspi.c:1556, dma-names "tx0"/"rx0"/"tx1"/"rx1"
 * -> &edma 16..19) only abort probe on -EPROBE_DEFER; any other failure
 * (including "no such channel") just leaves that chip-select on PIO and
 * probe continues regardless.
 *
 * The actual failure happens one layer down, in the generic ti-sysc bus
 * wrapper (drivers/bus/ti-sysc.c) that the am33xx-l4.dtsi target-module@
 * 30000 node describes as "ti,sysc-omap2", with three separate
 * register windows (reg-names "rev"/"sysc"/"syss" = offsets 0x0/0x110/
 * 0x114 from the 0x48030000 base) and `ti,syss-mask = <1>`. Because a
 * "syss" reg is present, ti-sysc's sysc_wait_softreset() polls
 * MCSPI_SYSSTATUS.RESETDONE (sysc_poll_reset_sysstatus(), ti-sysc.c:250)
 * rather than self-clearing SYSCONFIG bits -- against the unmapped
 * memory this device replaces, that poll always reads 0 and spins for
 * the full MAX_MODULE_SOFTRESET_WAIT before sysc_reset() returns
 * -ETIMEDOUT (-110), which sysc_init_module() propagates straight out of
 * sysc_probe() (ti-sysc.c:2212-2214). ti-sysc's own probe failing means
 * of_platform_populate() -- which is what would create the mcspi0 child
 * platform device in the first place -- never runs, so
 * omap2_mcspi_probe() never gets a chance to be called at all. This is
 * the same class of bug as WDT1/I2C0 before those were modelled (see
 * am335x_wdt.c, am335x_i2c.c): the fix lives in the sysc wrapper
 * registers, not in anything SPI-specific.
 *
 * The two load-bearing registers (TRM ch.24.4.1, Table 24-10)
 * -------------------------------------------------------------
 *  - MCSPI_SYSCONFIG (0x110, TRM 24.4.1.2): bit1 SOFTRESET is
 *    "automatically reset by the hardware" on write (self-clearing, like
 *    WDT1's WDSC/I2C0's I2C_SYSC and USBSS's SOFT_RESET/SOFTRESET bits).
 *  - MCSPI_SYSSTATUS (0x114, TRM 24.4.1.3): bit0 RESETDONE always reads
 *    1 here, exactly the WDT1/I2C0 idiom -- so sysc_poll_reset_sysstatus()
 *    completes on the very first read instead of timing out.
 *
 * MCSPI_REVISION (0x0, TRM 24.4.1.1) is read-only and returns the
 * documented reset value 0x00300000; ti-sysc logs it (sysc_show_rev(),
 * ti-sysc.c:931) but does not gate behaviour on its contents, and
 * omap2_mcspi_probe() never reads it either (the driver's own
 * MCSPI_REVISION, at its regs_offset-shifted 0x100, is likewise unused).
 *
 * Everything else (MCSPI_IRQSTATUS/IRQENABLE/SYST/MODULCTRL, the four
 * per-channel CONF/STAT/CTRL/TX/RX blocks, XFERLEVEL, DAFTX/DAFRX) is
 * plain read-write-back scratch: probe only writes MODULCTRL and
 * WAKEUPENABLE (the latter at driver offset 0x120 -- curiously not in
 * TRM Table 24-10 at all, i.e. genuinely unimplemented on this SoC
 * variant per Table 24-1 "Unsupported McSPI Features"; the write lands
 * harmlessly in the flat store like any other reserved offset), and no
 * transfer ever runs against this stub, so nothing else is ever polled.
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
#include "hw/ssi/am335x_mcspi.h"
#include "hw/sysbus.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"

/* Register offsets (TRM SPRUH73Q Table 24-10). */
#define MCSPI_REVISION    0x00
#define MCSPI_SYSCONFIG   0x110
#define MCSPI_SYSSTATUS   0x114

#define MCSPI_SYSCONFIG_SOFTRESET (1 << 1)
#define MCSPI_SYSSTATUS_RESETDONE (1 << 0)

/* Reset value (TRM 24.4.1.1, "reset = 300000h"). */
#define MCSPI_REVISION_VALUE 0x00300000

/* --- Flat byte-store helpers (see am335x_usbss.c) ---------------------- */

static uint64_t am335x_mcspi_load(const uint8_t *regs, hwaddr offset,
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

static void am335x_mcspi_store(uint8_t *regs, hwaddr offset, uint64_t value,
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

static uint64_t am335x_mcspi_read(void *opaque, hwaddr offset, unsigned size)
{
    AM335xMcspiState *s = AM335X_MCSPI(opaque);

    if (offset + size > AM335X_MCSPI_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: read outside window at offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_MCSPI, offset);
        return 0;
    }

    /* MCSPI_REVISION: fixed IP revision, see the file header. */
    if (offset == MCSPI_REVISION) {
        return extract64(MCSPI_REVISION_VALUE, 0, size * 8);
    }

    /*
     * MCSPI_SYSSTATUS.RESETDONE: the one register that actually gates
     * ti-sysc's OCP softreset poll (sysc_poll_reset_sysstatus()); see
     * the file header. Always report reset-complete.
     */
    if (offset == MCSPI_SYSSTATUS) {
        return extract64(MCSPI_SYSSTATUS_RESETDONE, 0, size * 8);
    }

    return am335x_mcspi_load(s->regs, offset, size);
}

static void am335x_mcspi_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    AM335xMcspiState *s = AM335X_MCSPI(opaque);

    if (offset + size > AM335X_MCSPI_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: write outside window at offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_MCSPI, offset);
        return;
    }

    /* MCSPI_REVISION and MCSPI_SYSSTATUS are read-only. */
    if (offset == MCSPI_REVISION || offset == MCSPI_SYSSTATUS) {
        return;
    }

    am335x_mcspi_store(s->regs, offset, value, size);

    /*
     * MCSPI_SYSCONFIG.SOFTRESET self-clears in hardware (TRM 24.4.1.2:
     * "automatically reset by the hardware"); see the file header.
     * Nothing actually polls it today since SYSSTATUS.RESETDONE already
     * always reads 1, but modelling it keeps a debugfs/regdump peek
     * honest, same rationale as USBSS's SOFT_RESET/SOFTRESET bits.
     * Clearing byte 0 of the stored word is correct regardless of the
     * access width, since any write that lands exactly on the
     * register's base offset necessarily includes its low byte.
     */
    if (offset == MCSPI_SYSCONFIG) {
        s->regs[offset] &= ~MCSPI_SYSCONFIG_SOFTRESET;
    }
}

static const MemoryRegionOps am335x_mcspi_ops = {
    .read = am335x_mcspi_read,
    .write = am335x_mcspi_write,
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

static void am335x_mcspi_reset(DeviceState *dev)
{
    AM335xMcspiState *s = AM335X_MCSPI(dev);

    memset(s->regs, 0, AM335X_MCSPI_SIZE);
}

static void am335x_mcspi_realize(DeviceState *dev, Error **errp)
{
    AM335xMcspiState *s = AM335X_MCSPI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->regs = g_malloc0(AM335X_MCSPI_SIZE);

    memory_region_init_io(&s->iomem, OBJECT(s), &am335x_mcspi_ops, s,
                          TYPE_AM335X_MCSPI, AM335X_MCSPI_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    /* INTC line 65, McSPI0INT (TRM Table 6-1). Never asserted -- this
     * stub runs no SPI transfers -- see the file header. */
    sysbus_init_irq(sbd, &s->irq);
}

static void am335x_mcspi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = am335x_mcspi_realize;
    device_class_set_legacy_reset(dc, am335x_mcspi_reset);
    /*
     * No vmsd: heap-allocated backing store, and the beaglebone-black
     * machine is not migratable (SoC uses serial_hd(), user_creatable =
     * false) -- same rationale as am335x_control.c/am335x_usbss.c.
     */
    dc->desc = "TI AM335x McSPI0 (clean-probe stub)";
}

static const TypeInfo am335x_mcspi_info = {
    .name          = TYPE_AM335X_MCSPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xMcspiState),
    .class_init    = am335x_mcspi_class_init,
};

static void am335x_mcspi_register_types(void)
{
    type_register_static(&am335x_mcspi_info);
}

type_init(am335x_mcspi_register_types)
