/*
 * TI AM335x LCD Controller (LCDC / "ti,am33xx-tilcdc") emulation.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * A minimal raster-mode model of the AM335x LCDC (TRM spruh73q ch.13,
 * base 0x4830E000, INTC line 36) covering exactly what the Linux tilcdc
 * driver (drivers/gpu/drm/tilcdc/) touches to bring up a DRM/fbdev
 * framebuffer:
 *
 *  - PID (0x00) reads 0x4F201000 so tilcdc_pdev_probe() selects the rev-2
 *    IP (tilcdc_drv.c:307-321); the previous unimplemented-device stub read
 *    0 and made the driver log "Unknown PID Reg value 0x00000000".
 *    The ti-sysc target-module wrapper (am33xx-l4.dtsi target-module@e000,
 *    reg = <0xe000 rev> <0xe054 sysc>) reads this same PID as its REVISION
 *    register and SYSCONFIG at 0x54.
 *  - RASTER_CTRL (0x28) RASTER_ENABLE/PALETTE_LOAD_MODE gate scanout.
 *  - RASTER_TIMING_0/1 (0x2c/0x30, plus LPP_B10 in RASTER_TIMING_2 0x34)
 *    are decoded back into the display width/height, using the exact field
 *    packing tilcdc_crtc_set_mode() (tilcdc_crtc.c) writes.
 *  - LCDDMA_FB0_BASE (0x44) is the physical DRAM scanout address written by
 *    set_scanout() (tilcdc_crtc.c:60-86); the LCDC has no IOMMU, so the model
 *    reads the framebuffer straight out of get_system_memory() with the
 *    framebuffer.c dirty-bitmap helper (same as every other QEMU LCD
 *    controller, e.g. pl110.c / omap_lcdc.c).
 *
 * The board's DT sets blue-and-red-wiring = "straight", so the fbdev client's
 * preferred format is DRM_FORMAT_RGB565 (16bpp) -- the only format modelled.
 *
 * Interrupts (INTC line 36): modelled faithfully so the tilcdc IRQ handler
 * (tilcdc_crtc_irq(), tilcdc_crtc.c:848-957) completes its handshakes:
 *  - END_OF_FRAME0 (vblank) is raised ~60x/s by a periodic timer while the
 *    raster engine is scanning out. Without it, drm_atomic_helper_commit_tail
 *    blocks in drm_atomic_helper_wait_for_vblanks() ("vblank wait timed out")
 *    and tilcdc's page-flip queue never drains ("already pending page flip").
 *  - PL_LOAD_DONE is raised once when the driver enables the raster in
 *    PALETTE_ONLY mode, completing tilcdc_crtc_load_palette()'s 50ms wait.
 *  - FRAME_DONE is raised when the driver clears RASTER_ENABLE, completing
 *    tilcdc_crtc_off()'s 500ms wait.
 * All three obey the rev-2 IRQENABLE_SET/CLEAR mask; the line is asserted only
 * while (raw status & enable) is non-zero, and the masked status register
 * (0x5c) the driver reads/clears reflects that same mask.
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
#include "hw/display/am335x_lcdc.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "ui/pixel_ops.h"
#include "framebuffer.h"
#include "exec/address-spaces.h"

/* Register offsets (TRM Table 13-13; tilcdc_regs.h). */
#define LCDC_PID_REG                    0x00
#define LCDC_CTRL_REG                   0x04
#define LCDC_RASTER_CTRL_REG            0x28
#define LCDC_RASTER_TIMING_0_REG        0x2c
#define LCDC_RASTER_TIMING_1_REG        0x30
#define LCDC_RASTER_TIMING_2_REG        0x34
#define LCDC_DMA_CTRL_REG               0x40
#define LCDC_DMA_FB_BASE_ADDR_0_REG     0x44
#define LCDC_DMA_FB_CEILING_ADDR_0_REG  0x48
#define LCDC_DMA_FB_BASE_ADDR_1_REG     0x4c
#define LCDC_DMA_FB_CEILING_ADDR_1_REG  0x50
#define LCDC_SYSCONFIG_REG              0x54
#define LCDC_RAW_STAT_REG               0x58
#define LCDC_MASKED_STAT_REG            0x5c
#define LCDC_INT_ENABLE_SET_REG         0x60
#define LCDC_INT_ENABLE_CLR_REG         0x64
#define LCDC_END_OF_INT_IND_REG         0x68
#define LCDC_CLK_ENABLE_REG             0x6c
#define LCDC_CLK_RESET_REG              0x70

/* RASTER_CTRL bits. */
#define LCDC_RASTER_ENABLE      (1u << 0)
#define LCDC_PALETTE_ONLY       0x1     /* PALETTE_LOAD_MODE == 1 */

/* Status / IRQENABLE bits (shared bit positions on rev-2; tilcdc_regs.h). */
#define LCDC_STAT_FRAME_DONE    (1u << 0)
#define LCDC_STAT_SYNC_LOST     (1u << 2)
#define LCDC_STAT_FIFO_UNDER    (1u << 5)
#define LCDC_STAT_PL_LOAD_DONE  (1u << 6)
#define LCDC_STAT_EOF0          (1u << 8)
#define LCDC_STAT_EOF1          (1u << 9)

/* RASTER_TIMING_2 bit 10 of line count. */
#define LCDC_LPP_B10            (1u << 26)

/* PID reset value for the AM335x (rev-2 LCDC). */
#define AM335X_LCDC_PID_VALUE   0x4F201000

/* Emulated vblank cadence (~60Hz). */
#define AM335X_LCDC_VBLANK_NS   (NANOSECONDS_PER_SECOND / 60)

/*
 * Recover the active display resolution from the raster-timing registers,
 * using the exact field packing tilcdc_crtc_set_mode() applies:
 *   RASTER_TIMING_0 bits[9:4] = ((hdisplay>>4)-1)[5:0]; rev-2 puts bit 6 of
 *   that value at bit 3.
 *   RASTER_TIMING_1 bits[9:0]  = (vdisplay-1)[9:0]; bit 10 lives in
 *   RASTER_TIMING_2 as LPP_B10.
 */
static void am335x_lcdc_decode_size(AM335xLcdcState *s, int *width, int *height)
{
    uint32_t t0 = s->raster_timing[0];
    uint32_t t1 = s->raster_timing[1];
    uint32_t t2 = s->raster_timing[2];
    uint32_t hval = ((t0 >> 4) & 0x3f) | (((t0 >> 3) & 0x1) << 6);
    uint32_t vval = (t1 & 0x3ff) | ((t2 & LCDC_LPP_B10) ? 0x400 : 0);

    *width = (hval + 1) << 4;
    *height = vval + 1;
}

/* RGB565 -> x8r8g8b8 surface (verbatim omap_lcdc.c draw_line16_32). */
static void am335x_lcdc_draw_line_rgb565(void *opaque, uint8_t *d,
                                         const uint8_t *s, int width,
                                         int deststep)
{
    uint16_t v;
    uint8_t r, g, b;

    do {
        v = lduw_le_p((const void *)s);
        r = (v >> 8) & 0xf8;
        g = (v >> 3) & 0xfc;
        b = (v << 3) & 0xf8;
        ((uint32_t *)d)[0] = rgb_to_pixel32(r, g, b);
        s += 2;
        d += 4;
    } while (--width != 0);
}

static void am335x_lcdc_update_display(void *opaque)
{
    AM335xLcdcState *s = AM335X_LCDC(opaque);
    DisplaySurface *surface;
    int width, height, first = 0, last = 0;
    int src_width;

    /* Nothing to scan out unless the raster engine is running with pixel
     * data (a PALETTE_ONLY pass carries no framebuffer content). */
    if (!(s->raster_ctrl & LCDC_RASTER_ENABLE)) {
        return;
    }
    if (((s->raster_ctrl >> 20) & 0x3) == LCDC_PALETTE_ONLY) {
        return;
    }

    am335x_lcdc_decode_size(s, &width, &height);
    if (width <= 0 || height <= 0 || s->fb0_base == 0) {
        return;
    }

    surface = qemu_console_surface(s->con);
    if (!surface_bits_per_pixel(surface)) {
        return;
    }

    if (width != s->width || height != s->height) {
        s->width = width;
        s->height = height;
        qemu_console_resize(s->con, width, height);
        surface = qemu_console_surface(s->con);
        s->invalidate = 1;
    }

    src_width = width * 2;  /* RGB565 */
    if (s->invalidate) {
        framebuffer_update_memory_section(&s->fbsection, get_system_memory(),
                                          s->fb0_base, height, src_width);
    }

    framebuffer_update_display(surface, &s->fbsection, width, height,
                               src_width, surface_stride(surface), 0,
                               s->invalidate, am335x_lcdc_draw_line_rgb565,
                               NULL, &first, &last);

    if (first >= 0) {
        dpy_gfx_update(s->con, 0, first, width, last - first + 1);
    }
    s->invalidate = 0;
}

static void am335x_lcdc_invalidate_display(void *opaque)
{
    AM335xLcdcState *s = AM335X_LCDC(opaque);

    s->invalidate = 1;
}

static void am335x_lcdc_update_irq(AM335xLcdcState *s)
{
    qemu_set_irq(s->irq, (s->irqstatus & s->irqenable) != 0);
}

/* True while the raster engine is actively scanning pixel data. */
static bool am335x_lcdc_scanning(AM335xLcdcState *s)
{
    return (s->raster_ctrl & LCDC_RASTER_ENABLE) &&
           ((s->raster_ctrl >> 20) & 0x3) != LCDC_PALETTE_ONLY;
}

/* Periodic end-of-frame (vblank): drives tilcdc's vblank/page-flip machinery. */
static void am335x_lcdc_vblank(void *opaque)
{
    AM335xLcdcState *s = AM335X_LCDC(opaque);

    if (!am335x_lcdc_scanning(s)) {
        return;
    }
    s->irqstatus |= LCDC_STAT_EOF0;
    am335x_lcdc_update_irq(s);
    timer_mod(s->vblank_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + AM335X_LCDC_VBLANK_NS);
}

static void am335x_lcdc_raster_ctrl_write(AM335xLcdcState *s, uint32_t v)
{
    bool was_scanning = am335x_lcdc_scanning(s);
    bool was_enabled = s->raster_ctrl & LCDC_RASTER_ENABLE;
    bool now_enabled, now_scanning, palette_only;

    s->raster_ctrl = v;
    now_enabled = v & LCDC_RASTER_ENABLE;
    now_scanning = am335x_lcdc_scanning(s);
    palette_only = now_enabled && !now_scanning;

    /* Redraw from scratch on any enable/mode change. */
    s->invalidate = 1;

    if (palette_only && !was_enabled) {
        /* PALETTE_ONLY pass: signal the palette load completed immediately. */
        s->irqstatus |= LCDC_STAT_PL_LOAD_DONE;
        am335x_lcdc_update_irq(s);
    }

    if (now_scanning && !was_scanning) {
        timer_mod(s->vblank_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + AM335X_LCDC_VBLANK_NS);
    } else if (!now_enabled && was_enabled) {
        /* Raster stopped: deliver the frame-done the off path waits for. */
        timer_del(s->vblank_timer);
        s->irqstatus |= LCDC_STAT_FRAME_DONE;
        am335x_lcdc_update_irq(s);
    }
}

static uint64_t am335x_lcdc_read(void *opaque, hwaddr offset, unsigned size)
{
    AM335xLcdcState *s = AM335X_LCDC(opaque);

    switch (offset) {
    case LCDC_PID_REG:
        return AM335X_LCDC_PID_VALUE;
    case LCDC_CTRL_REG:
        return s->ctrl;
    case LCDC_RASTER_CTRL_REG:
        return s->raster_ctrl;
    case LCDC_RASTER_TIMING_0_REG:
        return s->raster_timing[0];
    case LCDC_RASTER_TIMING_1_REG:
        return s->raster_timing[1];
    case LCDC_RASTER_TIMING_2_REG:
        return s->raster_timing[2];
    case LCDC_DMA_CTRL_REG:
        return s->dma_ctrl;
    case LCDC_DMA_FB_BASE_ADDR_0_REG:
        return s->fb0_base;
    case LCDC_DMA_FB_CEILING_ADDR_0_REG:
        return s->fb0_ceiling;
    case LCDC_DMA_FB_BASE_ADDR_1_REG:
        return s->fb1_base;
    case LCDC_DMA_FB_CEILING_ADDR_1_REG:
        return s->fb1_ceiling;
    case LCDC_SYSCONFIG_REG:
        return s->sysconfig;
    case LCDC_RAW_STAT_REG:
        return s->irqstatus;
    case LCDC_MASKED_STAT_REG:
        return s->irqstatus & s->irqenable;
    case LCDC_INT_ENABLE_SET_REG:
    case LCDC_INT_ENABLE_CLR_REG:
        return s->irqenable;
    case LCDC_CLK_ENABLE_REG:
        return s->clkc_enable;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_LCDC, offset);
        return 0;
    }
}

static void am335x_lcdc_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    AM335xLcdcState *s = AM335X_LCDC(opaque);
    uint32_t v = (uint32_t)value;

    switch (offset) {
    case LCDC_PID_REG:
        break;  /* read-only */
    case LCDC_CTRL_REG:
        s->ctrl = v;
        break;
    case LCDC_RASTER_CTRL_REG:
        am335x_lcdc_raster_ctrl_write(s, v);
        break;
    case LCDC_RASTER_TIMING_0_REG:
        s->raster_timing[0] = v;
        break;
    case LCDC_RASTER_TIMING_1_REG:
        s->raster_timing[1] = v;
        break;
    case LCDC_RASTER_TIMING_2_REG:
        s->raster_timing[2] = v;
        break;
    case LCDC_DMA_CTRL_REG:
        s->dma_ctrl = v;
        break;
    case LCDC_DMA_FB_BASE_ADDR_0_REG:
        s->fb0_base = v;
        s->invalidate = 1;
        break;
    case LCDC_DMA_FB_CEILING_ADDR_0_REG:
        s->fb0_ceiling = v;
        break;
    case LCDC_DMA_FB_BASE_ADDR_1_REG:
        s->fb1_base = v;
        break;
    case LCDC_DMA_FB_CEILING_ADDR_1_REG:
        s->fb1_ceiling = v;
        break;
    case LCDC_SYSCONFIG_REG:
        s->sysconfig = v;
        break;
    case LCDC_RAW_STAT_REG:
    case LCDC_MASKED_STAT_REG:
        /* Status bits are write-1-to-clear. */
        s->irqstatus &= ~v;
        am335x_lcdc_update_irq(s);
        break;
    case LCDC_INT_ENABLE_SET_REG:
        s->irqenable |= v;
        am335x_lcdc_update_irq(s);
        break;
    case LCDC_INT_ENABLE_CLR_REG:
        s->irqenable &= ~v;
        am335x_lcdc_update_irq(s);
        break;
    case LCDC_END_OF_INT_IND_REG:
        break;  /* EOI kick: pure sink (TRM 13.3.6.1.6) */
    case LCDC_CLK_ENABLE_REG:
        s->clkc_enable = v;
        break;
    case LCDC_CLK_RESET_REG:
        break;  /* MAIN_RESET pulse: accepted and ignored */
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_LCDC, offset);
        break;
    }
}

static const MemoryRegionOps am335x_lcdc_ops = {
    .read = am335x_lcdc_read,
    .write = am335x_lcdc_write,
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

static const GraphicHwOps am335x_lcdc_gfx_ops = {
    .invalidate = am335x_lcdc_invalidate_display,
    .gfx_update = am335x_lcdc_update_display,
};

static void am335x_lcdc_reset(DeviceState *dev)
{
    AM335xLcdcState *s = AM335X_LCDC(dev);

    s->ctrl = 0;
    s->raster_ctrl = 0;
    s->raster_timing[0] = 0;
    s->raster_timing[1] = 0;
    s->raster_timing[2] = 0;
    s->dma_ctrl = 0;
    s->fb0_base = 0;
    s->fb0_ceiling = 0;
    s->fb1_base = 0;
    s->fb1_ceiling = 0;
    s->sysconfig = 0;
    s->irqstatus = 0;
    s->irqenable = 0;
    s->clkc_enable = 0;
    s->invalidate = 1;
    s->width = 0;
    s->height = 0;

    if (s->vblank_timer) {
        timer_del(s->vblank_timer);
    }
    qemu_set_irq(s->irq, 0);
}

static void am335x_lcdc_init(Object *obj)
{
    AM335xLcdcState *s = AM335X_LCDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &am335x_lcdc_ops, s,
                          TYPE_AM335X_LCDC, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void am335x_lcdc_realize(DeviceState *dev, Error **errp)
{
    AM335xLcdcState *s = AM335X_LCDC(dev);

    s->vblank_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, am335x_lcdc_vblank, s);
    s->con = graphic_console_init(dev, 0, &am335x_lcdc_gfx_ops, s);
}

static const VMStateDescription vmstate_am335x_lcdc = {
    .name = TYPE_AM335X_LCDC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, AM335xLcdcState),
        VMSTATE_UINT32(raster_ctrl, AM335xLcdcState),
        VMSTATE_UINT32_ARRAY(raster_timing, AM335xLcdcState, 3),
        VMSTATE_UINT32(dma_ctrl, AM335xLcdcState),
        VMSTATE_UINT32(fb0_base, AM335xLcdcState),
        VMSTATE_UINT32(fb0_ceiling, AM335xLcdcState),
        VMSTATE_UINT32(fb1_base, AM335xLcdcState),
        VMSTATE_UINT32(fb1_ceiling, AM335xLcdcState),
        VMSTATE_UINT32(sysconfig, AM335xLcdcState),
        VMSTATE_UINT32(irqstatus, AM335xLcdcState),
        VMSTATE_UINT32(irqenable, AM335xLcdcState),
        VMSTATE_UINT32(clkc_enable, AM335xLcdcState),
        VMSTATE_INT32(invalidate, AM335xLcdcState),
        VMSTATE_INT32(width, AM335xLcdcState),
        VMSTATE_INT32(height, AM335xLcdcState),
        VMSTATE_TIMER_PTR(vblank_timer, AM335xLcdcState),
        VMSTATE_END_OF_LIST()
    }
};

static void am335x_lcdc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = am335x_lcdc_realize;
    device_class_set_legacy_reset(dc, am335x_lcdc_reset);
    dc->vmsd = &vmstate_am335x_lcdc;
    dc->desc = "TI AM335x LCD Controller";
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo am335x_lcdc_info = {
    .name          = TYPE_AM335X_LCDC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xLcdcState),
    .instance_init = am335x_lcdc_init,
    .class_init    = am335x_lcdc_class_init,
};

static void am335x_lcdc_register_types(void)
{
    type_register_static(&am335x_lcdc_info);
}

type_init(am335x_lcdc_register_types)
