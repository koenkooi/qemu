/*
 * TI AM335x MMC/SD host controller (MMCHS, "ti,omap4-hsmmc") emulation.
 *
 * Structural template: hw/sd/cadence_sdhci.c. An outer container
 * MemoryRegion (0x1000) holds:
 *   - the TI "highlander" wrapper registers at offset 0x000 (0x000..0x1FF),
 *     backed by a small register array; and
 *   - the embedded SD Host Standard (TYPE_SYSBUS_SDHCI) core mapped at
 *     offset 0x200 (TRM spruh73q ch.18: SD_SDMASA is the first SDHCI-
 *     standard register and sits at container offset 0x200).
 *
 * The wrapper only needs enough behaviour for the sdhci-omap driver reset
 * path to make progress: SD_SYSSTATUS reports RESETDONE=1, SD_SYSCONFIG's
 * SOFTRESET bit self-clears, and everything else reads back what was
 * written. The real data path (commands, DMA, the SDCLK/INT_STABLE
 * handshake) is handled by the embedded SDHCI core.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "hw/sd/am335x_hsmmc.h"

/* --- TI wrapper registers (offsets within 0x000..0x1FC) ---------------- */
/* TRM spruh73q ch.18.5 (MMCHS register summary). */
#define SD_HL_REV            0x000  /* IP revision */
#define SD_HL_HWINFO         0x004  /* HW configuration */
#define SD_HL_SYSCONFIG      0x010  /* highlander system config */
#define SD_SYSCONFIG         0x110  /* system config (bit1 SOFTRESET) */
#define SD_SYSSTATUS         0x114  /* system status (bit0 RESETDONE) */
#define SD_CSRE              0x124  /* card status response error */
#define SD_SYSTEST           0x128  /* system test */
#define SD_CON               0x12C  /* configuration */
#define SD_PWCNT             0x130  /* power counter */

#define SD_SYSCONFIG_SOFTRESET  BIT(1)  /* self-clearing soft reset */
#define SD_SYSSTATUS_RESETDONE  BIT(0)  /* reset complete (always 1 here) */

/*
 * SD_HL_HWINFO.MADMA_EN (bit0): "the controller has an integrated ADMA master".
 * The mainline sdhci-omap driver reads this bit (sdhci_omap_has_adma(),
 * sdhci-omap.c:731-738) and, only if it is *clear*, switches MMC data transfer
 * to an external EDMA channel (sdhci_switch_external_dma(), sdhci-omap.c:1327).
 * Real AM335x MMCHS reports 0 here and therefore uses EDMA -- but this tree's
 * EDMA3 model (hw/dma/am335x_edma.c) is deliberately scoped to McASP and moves
 * no MMC data. The embedded SD Host Standard core, by contrast, does implement
 * ADMA2 (advertised in AM335X_HSMMC_CAPAB) and the internal path QEMU already
 * drives correctly. So we report MADMA_EN=1: sdhci-omap keeps using that
 * working internal ADMA engine (exactly as it did before EDMA3 was modelled)
 * instead of routing SD/eMMC transfers through the data-less EDMA model. This
 * is the documented MMC-side boundary of the EDMA model's generality.
 */
#define SD_HL_HWINFO_MADMA_EN   BIT(0)

/*
 * Capabilities advertised by the embedded SD Host Standard core.
 *
 * 0x057834b4 is the generic-sdhci reset default (and the value fsl-imx6
 * uses): timeout clock 52 MHz, base clock 52 MHz (BASECLKFREQ != 0, so the
 * SDCLK divider maths in the guest driver never divides by zero and the
 * clock-stable wait terminates), high-speed, SDMA + ADMA1 + ADMA2, 512-byte
 * max block length, and 3.3 V (V33) + 1.8 V (V18) bus voltages -- the two
 * voltages the sdhci-omap driver ORs into the CAPA register.
 */
#define AM335X_HSMMC_CAPAB   0x057834b4ULL

/* --- TI wrapper MMIO ---------------------------------------------------- */

static uint64_t am335x_hsmmc_wrap_read(void *opaque, hwaddr addr,
                                       unsigned size)
{
    AM335xHsmmcState *s = opaque;

    if (size != 4 || addr >= AM335X_HSMMC_WRAP_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read size %u at offset 0x%" HWADDR_PRIx "\n",
                      __func__, size, addr);
        return 0;
    }

    switch (addr) {
    case SD_SYSSTATUS:
        /* Software reset is instantaneous in the model. */
        return SD_SYSSTATUS_RESETDONE;
    case SD_HL_HWINFO:
        /* Report an integrated ADMA master so sdhci-omap uses the embedded
         * core's ADMA rather than the (data-less) EDMA3 model (see the
         * SD_HL_HWINFO_MADMA_EN comment above). */
        return SD_HL_HWINFO_MADMA_EN;
    default:
        return s->wrap_regs[addr >> 2];
    }
}

static void am335x_hsmmc_wrap_write(void *opaque, hwaddr addr, uint64_t val,
                                    unsigned size)
{
    AM335xHsmmcState *s = opaque;
    uint32_t val32 = (uint32_t)val;

    if (size != 4 || addr >= AM335X_HSMMC_WRAP_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write size %u at offset 0x%" HWADDR_PRIx "\n",
                      __func__, size, addr);
        return;
    }

    switch (addr) {
    case SD_SYSCONFIG:
        /*
         * SOFTRESET (bit1) is self-clearing: the reset completes
         * immediately, so drop the bit before storing.
         */
        val32 &= ~SD_SYSCONFIG_SOFTRESET;
        s->wrap_regs[addr >> 2] = val32;
        break;
    case SD_SYSSTATUS:
        /* Read-only status register. */
        break;
    default:
        s->wrap_regs[addr >> 2] = val32;
        break;
    }
}

static const MemoryRegionOps am335x_hsmmc_wrap_ops = {
    .read = am335x_hsmmc_wrap_read,
    .write = am335x_hsmmc_wrap_write,
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

/* --- QOM ---------------------------------------------------------------- */

static void am335x_hsmmc_init(Object *obj)
{
    AM335xHsmmcState *s = AM335X_HSMMC(obj);

    object_initialize_child(obj, "sdhci", &s->sdhci, TYPE_SYSBUS_SDHCI);
}

static void am335x_hsmmc_realize(DeviceState *dev, Error **errp)
{
    AM335xHsmmcState *s = AM335X_HSMMC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    SysBusDevice *sbd_sdhci = SYS_BUS_DEVICE(&s->sdhci);

    /* Outer module window is what the SoC maps. */
    memory_region_init(&s->container, OBJECT(s),
                       "am335x-hsmmc-container", AM335X_HSMMC_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->container);

    /* TI wrapper registers cover the low 0x200 bytes of the window. */
    memory_region_init_io(&s->wrapper_iomem, OBJECT(s), &am335x_hsmmc_wrap_ops,
                          s, "am335x-hsmmc-wrapper", AM335X_HSMMC_WRAP_SIZE);
    memory_region_add_subregion(&s->container, 0, &s->wrapper_iomem);

    /*
     * Configure the embedded SD Host Standard core before realizing it:
     * SDHCI spec v3 and a capabilities value with a non-zero base clock and
     * V33/V18 so the sdhci-omap probe does not spin waiting for a stable
     * clock. DMA is left at the default (system memory), which is where the
     * AM335x DRAM lives -- same idiom as cadence_sdhci / fsl-imx6.
     */
    if (!object_property_set_uint(OBJECT(&s->sdhci), "sd-spec-version", 3,
                                  errp)) {
        return;
    }
    if (!object_property_set_uint(OBJECT(&s->sdhci), "capareg",
                                  AM335X_HSMMC_CAPAB, errp)) {
        return;
    }
    /*
     * The MMCHS latches the whole 136-bit R2 response, CRC7 included, into
     * SD_RSP10..76 rather than dropping the CRC and right-justifying the
     * payload the way the SD Host Standard specifies (TRM spruh73q ch.18,
     * SD_RSP* description). The Linux sdhci-omap driver reflects this with
     * SDHCI_QUIRK2_RSP_136_HAS_CRC and reads the registers without the usual
     * 8-bit realignment, so the embedded core must present R2 in that form or
     * every CID/CSD field the guest decodes is off by one byte.
     */
    if (!object_property_set_bool(OBJECT(&s->sdhci), "r2-has-crc", true,
                                  errp)) {
        return;
    }
    /*
     * The MMCHS SD_HCTL.SDBP (SD bus power) bit is a sticky software-
     * controlled bit: once the sdhci-omap driver's conf_bus_power() writes it
     * (with SD_HCTL.SDVS selecting 3.3 V) it must read back set, independent
     * of the SD-standard card-present/voltage-capability gating the generic
     * core applies (TRM spruh73q ch.18, SD_HCTL). Without this the driver's
     * 1ms read-back poll times out and sdhci_omap_start_signal_voltage_switch
     * WARNs on every mmc_power_up and runtime-PM resume.
     */
    if (!object_property_set_bool(OBJECT(&s->sdhci), "power-on-sticky", true,
                                  errp)) {
        return;
    }
    if (!sysbus_realize(sbd_sdhci, errp)) {
        return;
    }

    /* SD Host Standard registers begin at container offset 0x200. */
    memory_region_add_subregion(&s->container, AM335X_HSMMC_SDHCI_OFFSET,
                                sysbus_mmio_get_region(sbd_sdhci, 0));

    /* Re-export the SDHCI IRQ and "sd-bus" on the wrapper device. */
    sysbus_pass_irq(sbd, sbd_sdhci);
    object_property_add_alias(OBJECT(s), "sd-bus",
                              OBJECT(&s->sdhci), "sd-bus");
}

static void am335x_hsmmc_reset(DeviceState *dev)
{
    AM335xHsmmcState *s = AM335X_HSMMC(dev);

    memset(s->wrap_regs, 0, sizeof(s->wrap_regs));
    device_cold_reset(DEVICE(&s->sdhci));
}

static const VMStateDescription vmstate_am335x_hsmmc = {
    .name = TYPE_AM335X_HSMMC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(wrap_regs, AM335xHsmmcState,
                             AM335X_HSMMC_NUM_WRAP_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void am335x_hsmmc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "TI AM335x MMC/SD host controller (MMCHS)";
    dc->realize = am335x_hsmmc_realize;
    device_class_set_legacy_reset(dc, am335x_hsmmc_reset);
    dc->vmsd = &vmstate_am335x_hsmmc;
}

static const TypeInfo am335x_hsmmc_info = {
    .name          = TYPE_AM335X_HSMMC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xHsmmcState),
    .instance_init = am335x_hsmmc_init,
    .class_init    = am335x_hsmmc_class_init,
};

static void am335x_hsmmc_register_types(void)
{
    type_register_static(&am335x_hsmmc_info);
}

type_init(am335x_hsmmc_register_types)
