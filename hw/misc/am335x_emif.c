/*
 * TI AM335x EMIF (External Memory Interface / DDR3 controller) emulation.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Structural template: hw/misc/am335x_control.c. The EMIF is programmed by
 * the AM335x SPL (MLO) during DDR3 bring-up (config_ddr() ->
 * config_ddr_phy()/set_sdram_timings()/config_sdram() in u-boot's
 * arch/arm/mach-omap2/am33xx/ddr.c), but the DRAM itself is backed by QEMU's
 * plain RAM at 0x80000000, so the controller only needs to accept those
 * register writes and read them back. This model backs the whole window with
 * a plain read/write store and synthesizes two read-only registers on top:
 *
 *  - EMIF_MOD_ID_REV (0x00) returns the AM335x reset value 0x40443403 (TRM
 *    spruh73q 7.3.5.1). u-boot's get_emif_rev() reads the MAJOR_REVISION
 *    field (bits[10:8] = 4) to pick the register-programming variant; 4
 *    (!= EMIF_4D5 == 5) selects the simple config_sdram() path, which does
 *    no EMIF status polling at all on this SoC.
 *  - EMIF_STATUS (0x04) reports the PHY DLL ready (bit 2) and no
 *    read/write-leveling timeout (bits[6:4] == 0). The AM335x SPL DDR3 path
 *    never polls this (only the EMIF_4D5 leveling path used by dra7/am43xx
 *    does), but reporting "ready" keeps any status reader -- e.g. Linux PM
 *    code -- from ever seeing an in-progress/failed state.
 *
 * The VTP impedance-calibration register the SPL *does* busy-wait on lives
 * in the Control Module window (0x44E10E0C), not here; its READY bit is
 * synthesized in hw/misc/am335x_control.c.
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
#include "hw/misc/am335x_emif.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "qemu/module.h"

/* Register offsets (TRM spruh73q ch.7.3.5). */
#define EMIF_MOD_ID_REV 0x00
#define EMIF_STATUS     0x04

/* EMIF_MOD_ID_REV reset value (TRM 7.3.5.1); MAJOR_REVISION (bits[10:8]) = 4. */
#define EMIF_MOD_ID_REV_VALUE 0x40443403

/* EMIF_STATUS bits (TRM 7.3.5.2 / u-boot asm/emif.h). */
#define EMIF_STATUS_PHY_DLL_READY   (1u << 2)
#define EMIF_STATUS_LEVELING_TO_MASK (0x7u << 4)

/* --- MMIO ------------------------------------------------------------------ */

static uint64_t am335x_emif_read(void *opaque, hwaddr offset, unsigned size)
{
    AM335xEmifState *s = AM335X_EMIF(opaque);

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read size %u at offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_EMIF, size, offset);
        return 0;
    }

    switch (offset) {
    case EMIF_MOD_ID_REV:
        return EMIF_MOD_ID_REV_VALUE;
    case EMIF_STATUS:
        /* Always report the PHY ready and no leveling timeout. */
        return (s->regs[offset / 4] & ~EMIF_STATUS_LEVELING_TO_MASK) |
               EMIF_STATUS_PHY_DLL_READY;
    default:
        return s->regs[offset / 4];
    }
}

static void am335x_emif_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    AM335xEmifState *s = AM335X_EMIF(opaque);

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write size %u at offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_EMIF, size, offset);
        return;
    }

    switch (offset) {
    case EMIF_MOD_ID_REV:
    case EMIF_STATUS:
        /* read-only */
        break;
    default:
        s->regs[offset / 4] = (uint32_t)value;
        break;
    }
}

static const MemoryRegionOps am335x_emif_ops = {
    .read = am335x_emif_read,
    .write = am335x_emif_write,
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

static void am335x_emif_reset(DeviceState *dev)
{
    AM335xEmifState *s = AM335X_EMIF(dev);

    memset(s->regs, 0, AM335X_EMIF_SIZE);
}

static void am335x_emif_realize(DeviceState *dev, Error **errp)
{
    AM335xEmifState *s = AM335X_EMIF(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    s->regs = g_malloc0(AM335X_EMIF_SIZE);

    memory_region_init_io(&s->iomem, OBJECT(s), &am335x_emif_ops, s,
                          TYPE_AM335X_EMIF, AM335X_EMIF_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void am335x_emif_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = am335x_emif_realize;
    device_class_set_legacy_reset(dc, am335x_emif_reset);
    /*
     * No vmsd: this device carries a heap-allocated backing store and the
     * beaglebone-black machine is not migratable anyway (the SoC wires up
     * serial_hd() and is not user-creatable).
     */
    dc->desc = "TI AM335x EMIF (DDR controller)";
}

static const TypeInfo am335x_emif_info = {
    .name          = TYPE_AM335X_EMIF,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xEmifState),
    .class_init    = am335x_emif_class_init,
};

static void am335x_emif_register_types(void)
{
    type_register_static(&am335x_emif_info);
}

type_init(am335x_emif_register_types)
