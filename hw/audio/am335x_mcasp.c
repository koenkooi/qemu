/*
 * TI AM335x McASP0 (Multichannel Audio Serial Port) emulation -- scoped to
 * the HDMI-audio I2S playback path.
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
 *
 * ---------------------------------------------------------------------------
 * What this is
 * ---------------------------------------------------------------------------
 * McASP0 is the AM335x audio serial port (TRM SPRUH73Q ch.22) used on the
 * BeagleBone Black as the I2S source feeding the on-board TDA19988 HDMI
 * transmitter (the DT `simple-audio-card` "TI BeagleBone Black": McASP0 = CPU
 * DAI, TDA19988 = codec). It is DMA-driven -- it cannot function without the
 * EDMA3 engine (hw/dma/am335x_edma.c), which is why until now the mainline
 * `davinci-mcasp` driver failed to probe with "No DMA controller found (-19)".
 *
 * This models McASP0 just far enough for that driver to probe cleanly, for
 * `snd_soc_register_component` and the ALSA `simple-audio-card` to register a
 * playback-capable sound card, and for a `dmaengine_prep_dma_cyclic` playback
 * to start and run (its periodic completions come from the EDMA model). Same
 * "structural, not functional" bar as CPSW / wl18xx-SDIO / USBSS: no real
 * audio samples are transported and nothing audible is produced -- the I2S
 * frames the driver hands to EDMA are discarded.
 *
 * ---------------------------------------------------------------------------
 * The Linux driver boundary this model targets (davinci-mcasp.c, kernel 7.x)
 * ---------------------------------------------------------------------------
 * Probe (davinci_mcasp_probe) has NO register read with a required value: the
 * PID/REV register @ 0x00 is defined but never referenced by the driver, and
 * probe otherwise only WRITES config registers (PFUNC=0 at 0x10, plus a
 * zero-initialised context block on first runtime-resume). The one thing that
 * blocked probe was dma_request_chan(dev,"tx") returning -ENODEV
 * (davinci-mcasp.c:2343 -> :2752); with the EDMA3 model resolving the DT
 * `dmas = <&edma 8 2>` binding that now succeeds -> PCM_EDMA ->
 * edma_pcm_platform_register -> snd_soc_register_component (pure software).
 *
 * So the 8KB "mpu" config window is a flat read-back-what-was-written store,
 * with exactly the reads the driver's runtime start path gates on synthesized:
 *
 *  - PID/REV @ 0x00 reads a fixed McASP IP revision (TRM 22.4.1.1 reset
 *    0x44307B02). The ti-sysc wrapper reads it as "rev" and logs but does not
 *    validate it (its target-module node has no ti,sysc-mask, so ti-sysc does
 *    not attempt an OCP softreset -> no reset-done poll to satisfy, unlike the
 *    WDT/I2C blocks).
 *  - TXSTAT @ 0xC0 and RXSTAT @ 0x80 read with XRDATA (BIT5) clear, so the
 *    start-TX wait loop (davinci-mcasp.c:351-354, bounded 100000) exits at
 *    once. The driver clears them by writing 0xFFFFFFFF (absorbed). Modelled
 *    as read-0 (idle: no data pending, no error).
 *  - GBLCTLX @ 0xA0 and GBLCTLR @ 0x60 read back what was written, so the
 *    mcasp_set_ctl_reg() verify loop (davinci-mcasp.c:171-186, bounded 1000)
 *    breaks on iteration 0 instead of printing "GBLCTL write error". Plain
 *    flat storage already gives this.
 *
 * The 4MB "dat" data-port window is where EDMA writes TX samples (and the PIO
 * XBUF fallback lives); writes are absorbed and reads return 0 -- samples are
 * discarded (see file header).
 *
 * McASP's own tx/rx interrupts (INTC 80/81) are error-only (TX underrun /
 * RX overrun) and optional; the DMA data-movement progress that advances the
 * ALSA ring comes entirely from EDMA completion interrupts, not from these.
 * This model never asserts them.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/audio/am335x_mcasp.h"
#include "hw/irq.h"
#include "migration/vmstate.h"

/* --- "mpu" config register offsets (TRM SPRUH73Q ch.22; davinci-mcasp.h) --- */
#define MCASP_PID       0x00    /* REV/PID (RO)                                */
#define MCASP_RXSTAT    0x80    /* RX status (RO here; XRDATA must read 0)     */
#define MCASP_GBLCTLR   0x60    /* RX global control (read-back verify)        */
#define MCASP_GBLCTLX   0xA0    /* TX global control (read-back verify)        */
#define MCASP_TXSTAT    0xC0    /* TX status (RO here; XRDATA must read 0)     */

/* REV register reset value (TRM SPRUH73Q 22.4.1.1, offset 0h). Logged by
 * ti-sysc; not validated by any driver path. */
#define MCASP_REV_RESET 0x44307B02u

/* ------------------------------------------------------------------------- */
/* "mpu" config window.                                                      */

static uint64_t am335x_mcasp_mpu_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    AM335xMcaspState *s = opaque;

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read size %u at 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_MCASP, size, offset);
        return 0;
    }

    switch (offset) {
    case MCASP_PID:
        return MCASP_REV_RESET;
    case MCASP_TXSTAT:
    case MCASP_RXSTAT:
        /* Idle: no data-pending (XRDATA=0) and no error, so the driver's
         * bounded start/stop status polls resolve immediately. */
        return 0;
    default:
        return s->regs[offset / 4];
    }
}

static void am335x_mcasp_mpu_write(void *opaque, hwaddr offset, uint64_t value,
                                   unsigned size)
{
    AM335xMcaspState *s = opaque;

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write size %u at 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_MCASP, size, offset);
        return;
    }

    switch (offset) {
    case MCASP_PID:
        break;                          /* read-only */
    case MCASP_TXSTAT:
    case MCASP_RXSTAT:
        break;                          /* W1C of status bits; nothing pending */
    default:
        /* GBLCTLX/GBLCTLR and every other config register are plain
         * read-back storage (the GBLCTL verify loops depend on it). */
        s->regs[offset / 4] = (uint32_t)value;
        break;
    }
}

static const MemoryRegionOps am335x_mcasp_mpu_ops = {
    .read = am335x_mcasp_mpu_read,
    .write = am335x_mcasp_mpu_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/* ------------------------------------------------------------------------- */
/* "dat" data-port window: EDMA TX destination / PIO XBUF. Samples discarded. */

static uint64_t am335x_mcasp_dat_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    return 0;
}

static void am335x_mcasp_dat_write(void *opaque, hwaddr offset, uint64_t value,
                                   unsigned size)
{
    /* absorbed */
}

static const MemoryRegionOps am335x_mcasp_dat_ops = {
    .read = am335x_mcasp_dat_read,
    .write = am335x_mcasp_dat_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* ------------------------------------------------------------------------- */

static void am335x_mcasp_reset(DeviceState *dev)
{
    AM335xMcaspState *s = AM335X_MCASP(dev);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_set_irq(s->irq[0], 0);
    qemu_set_irq(s->irq[1], 0);
}

static void am335x_mcasp_init(Object *obj)
{
    AM335xMcaspState *s = AM335X_MCASP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mpu, obj, &am335x_mcasp_mpu_ops, s,
                          "am335x-mcasp.mpu", AM335X_MCASP_MPU_SIZE);
    memory_region_init_io(&s->dat, obj, &am335x_mcasp_dat_ops, s,
                          "am335x-mcasp.dat", AM335X_MCASP_DAT_SIZE);
    sysbus_init_mmio(sbd, &s->mpu);
    sysbus_init_mmio(sbd, &s->dat);
    sysbus_init_irq(sbd, &s->irq[0]);
    sysbus_init_irq(sbd, &s->irq[1]);
}

static const VMStateDescription vmstate_am335x_mcasp = {
    .name = TYPE_AM335X_MCASP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AM335xMcaspState, AM335X_MCASP_MPU_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void am335x_mcasp_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, am335x_mcasp_reset);
    dc->vmsd = &vmstate_am335x_mcasp;
    dc->desc = "TI AM335x McASP0 audio serial port";
}

static const TypeInfo am335x_mcasp_info = {
    .name          = TYPE_AM335X_MCASP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xMcaspState),
    .instance_init = am335x_mcasp_init,
    .class_init    = am335x_mcasp_class_init,
};

static void am335x_mcasp_register_types(void)
{
    type_register_static(&am335x_mcasp_info);
}

type_init(am335x_mcasp_register_types)
