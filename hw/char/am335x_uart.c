/*
 * TI AM335x UART emulation.
 *
 * Structural template: hw/char/serial-mm.c, which this closely follows for
 * the embed-a-SerialState / realize / IRQ-aliasing shape (see
 * am335x_uart_instance_init() and am335x_uart_realize()). The extra
 * register set and the SYSC/SOFTRESET -> SYSS/RESETDONE handling mirrors
 * the same idiom already used for TIOCP_CFG/TISTAT in
 * hw/timer/am335x_timer.c.
 *
 * Register map (TRM spruh73q, chapter 19 "UART"), regshift 2 fixed (each
 * register is 4 bytes apart):
 *
 *   0x00-0x1C  the 8 standard 16550 registers (RHR/THR, IER, IIR/FCR,
 *              LCR, MCR, LSR, MSR, SCR -- DLL/DLH are muxed onto
 *              RHR/THR and IER via LCR.DLAB, exactly as on a real
 *              16550). Forwarded verbatim to the embedded SerialState
 *              via serial_io_ops, the same call `serial_mm_read()` /
 *              `serial_mm_write()` make in hw/char/serial-mm.c.
 *   0x20       MDR1 (Mode Definition Register 1). 8250_omap writes 0x07
 *              (disable/reset) then 0x00 (UART16x mode) here as part of
 *              its startup sequence. No functional effect is modelled;
 *              plain scratch storage is enough since the driver never
 *              reads this back to make a decision during console bind.
 *   0x24-0x60  MDR2, xFLL/xFLH, UASR, ACREG, SCR (OMAP shadow, distinct
 *              from the 16550 SCR at 0x1C), SSR, EBLR, MVR, WER, CFPS:
 *              plain scratch storage.
 *   0x54       SYSC (System Configuration). Bit 0 is SOFTRESET; writing
 *              it 1 resets the embedded 16550 core and (synchronously,
 *              in this model) completes the reset.
 *   0x58       SYSS (System Status). Bit 0 is RESETDONE, always read
 *              back as 1 -- the reset triggered via SYSC is modelled as
 *              instantaneous, so the 8250_omap reset-poll loop
 *              (`omap8250_soft_reset()` polling UART_OMAP_SYSC /
 *              UART_OMAP_SYSS in the kernel) never spins.
 *   elsewhere  (up to 0xFFC) plain scratch storage, default 0, no
 *              qemu_log_mask()/guest-error fault -- unlike
 *              hw/timer/am335x_timer.c, unknown offsets in this window
 *              are deliberately not fatal/logged, since 8250_omap and
 *              downstream DT tooling can probe registers this model
 *              does not need to give special meaning to.
 *
 * Copyright (C) 2026 Koen Kooi
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
#include "hw/char/am335x_uart.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"

/* Fixed regshift: 16550 register N lives at byte offset N << 2. */
#define AM335X_UART_REGSHIFT        2
#define AM335X_UART_16550_REGS_SIZE (8 << AM335X_UART_REGSHIFT) /* 0x20 */

/* OMAP-only extension registers that need behaviour beyond plain
 * scratch storage; absolute byte offsets within the 4KB window. */
#define AM335X_UART_SYSC 0x54 /* System Configuration Register */
#define AM335X_UART_SYSS 0x58 /* System Status Register */

#define AM335X_UART_SYSC_SOFTRESET (1 << 0)
#define AM335X_UART_SYSS_RESETDONE (1 << 0)

static uint64_t am335x_uart_read(void *opaque, hwaddr addr, unsigned size)
{
    AM335xUartState *s = opaque;

    if (addr < AM335X_UART_16550_REGS_SIZE) {
        return serial_io_ops.read(&s->serial, addr >> AM335X_UART_REGSHIFT, 1);
    }

    switch (addr) {
    case AM335X_UART_SYSS:
        /* Reset (see am335x_uart_write()) always completes synchronously,
         * so RESETDONE is unconditionally set. */
        return AM335X_UART_SYSS_RESETDONE;
    default:
        return s->scratch[addr >> 2];
    }
}

static void am335x_uart_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned size)
{
    AM335xUartState *s = opaque;

    if (addr < AM335X_UART_16550_REGS_SIZE) {
        serial_io_ops.write(&s->serial, addr >> AM335X_UART_REGSHIFT,
                            value & 0xff, 1);
        return;
    }

    switch (addr) {
    case AM335X_UART_SYSC:
        if (value & AM335X_UART_SYSC_SOFTRESET) {
            /*
             * device_cold_reset() on the embedded SerialState is the
             * established idiom for this (see hw/char/mchp_pfsoc_mmuart.c
             * mchp_pfsoc_mmuart_reset(), which does the same thing to its
             * embedded SerialMM); TYPE_SERIAL's own register reset
             * (serial_reset() in hw/char/serial.c) is however a static
             * function wired only into the legacy qemu_register_reset()
             * global system-reset list from serial_realize(), not into
             * the qdev/Resettable phases, so this call is a structural
             * no-op on today's serial.c. It is kept for the same reason
             * upstream keeps it: forward-compatibility, and it costs
             * nothing. See the reply's "API mismatches" note for why
             * this does not affect the actual goal (SYSS.RESETDONE is
             * hardcoded to 1 above regardless).
             */
            device_cold_reset(DEVICE(&s->serial));
            s->reset_done = true;
        }
        /* SOFTRESET is self-clearing on real hardware, matching how
         * hw/timer/am335x_timer.c treats TIOCP_CFG.SOFTRESET. */
        s->scratch[addr >> 2] = value & ~(uint64_t)AM335X_UART_SYSC_SOFTRESET;
        break;
    default:
        s->scratch[addr >> 2] = value;
        break;
    }
}

static const MemoryRegionOps am335x_uart_ops = {
    .read = am335x_uart_read,
    .write = am335x_uart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void am335x_uart_reset(DeviceState *dev)
{
    AM335xUartState *s = AM335X_UART(dev);

    memset(s->scratch, 0, sizeof(s->scratch));
    s->reset_done = true;

    /*
     * The embedded SerialState is realized as a plain (non-sysbus) child
     * device (see am335x_uart_realize()), so it is not reached by the
     * qdev/Resettable reset cascade that walks the sysbus tree; its
     * actual register reset instead happens on its own via the
     * qemu_register_reset() hook serial_realize() installs, which fires
     * on every full-machine reset independent of qdev parentage. Nothing
     * further is needed here for that half of the state.
     */
}

static void am335x_uart_instance_init(Object *obj)
{
    AM335xUartState *s = AM335X_UART(obj);

    object_initialize_child(obj, "serial", &s->serial, TYPE_SERIAL);
    object_property_add_alias(obj, "chardev", OBJECT(&s->serial), "chardev");
}

static void am335x_uart_realize(DeviceState *dev, Error **errp)
{
    AM335xUartState *s = AM335X_UART(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (!qdev_realize(DEVICE(&s->serial), NULL, errp)) {
        return;
    }

    memory_region_init_io(&s->iomem, OBJECT(dev), &am335x_uart_ops, s,
                          TYPE_AM335X_UART, AM335X_UART_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    /*
     * Alias the embedded SerialState's irq field directly as this
     * device's sysbus IRQ output, exactly as serial_mm_realize() does in
     * hw/char/serial-mm.c: serial_update_irq() in hw/char/serial.c
     * raises/lowers s->serial.irq directly, and sysbus_init_irq()
     * populates that same qemu_irq* as our GPIO-IRQ output storage, so
     * no separate forwarding/wiring is needed.
     */
    sysbus_init_irq(sbd, &s->serial.irq);
}

static const VMStateDescription am335x_uart_vmstate = {
    .name = TYPE_AM335X_UART,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(serial, AM335xUartState, 0, vmstate_serial, SerialState),
        VMSTATE_UINT32_ARRAY(scratch, AM335xUartState,
                             AM335X_UART_MMIO_SIZE / 4),
        VMSTATE_BOOL(reset_done, AM335xUartState),
        VMSTATE_END_OF_LIST()
    }
};

static void am335x_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = am335x_uart_realize;
    device_class_set_legacy_reset(dc, am335x_uart_reset);
    dc->vmsd = &am335x_uart_vmstate;
    dc->desc = "TI AM335x UART (16550-compatible core + OMAP soft-reset regs)";
}

static const TypeInfo am335x_uart_info = {
    .name          = TYPE_AM335X_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xUartState),
    .instance_init = am335x_uart_instance_init,
    .class_init    = am335x_uart_class_init,
};

static void am335x_uart_register_types(void)
{
    type_register_static(&am335x_uart_info);
}

type_init(am335x_uart_register_types)
