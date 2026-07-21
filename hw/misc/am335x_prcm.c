/*
 * TI AM335x Clock Module / PRCM (PRM+CM) emulation.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Structural template: hw/intc/am335x_intc.c. The register file here is a
 * flat read/write store (TRM spruh73q ch.8, base 0x44E00000) with two
 * targeted read-side transforms layered on top:
 *
 *  - CLKCTRL registers synthesize IDLEST (bits[17:16]) from the MODULEMODE
 *    field (bits[1:0]) that the guest last wrote, so the ti-sysc /
 *    ti-clkctrl enable-poll (wait IDLEST==0) and disable-poll (wait
 *    IDLEST==3) both terminate.
 *  - DPLL *_IDLEST registers report the DPLL locked (or bypassed) based on
 *    the DPLL_EN field of the paired *_CLKMODE_DPLL register, so DPLL
 *    lock-wait loops terminate immediately.
 *
 * Everything else is plain read-back-what-was-written storage.
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
#include "hw/misc/am335x_prcm.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/runstate.h"

/* --- CLKCTRL offset set (absolute within the 0x44E00000 window) ------- */
/*
 * Every CM_*_*_CLKCTRL register in CM_PER, CM_WKUP, CM_MPU, CM_RTC, CM_GFX
 * and CM_CEFUSE. Layout per TRM spruh73q 8.1.12/8.1.13/8.1.14/8.1.15/
 * 8.1.16/8.1.17. Bits[1:0] = MODULEMODE, bits[17:16] = IDLEST (read-only on
 * real HW, synthesized here).
 */
static const hwaddr am335x_prcm_clkctrl_offsets[] = {
    /* CM_PER (base 0x000) -- full CLKCTRL set per TRM spruh73q Table 8-5 */
    0x014, 0x018, 0x01C, 0x020, 0x024, 0x028, 0x02C, 0x030, 0x034, 0x038,
    0x03C, 0x040, 0x044, 0x048, 0x04C, 0x050, 0x060, 0x068, 0x06C, 0x070,
    0x074, 0x078, 0x07C, 0x080, 0x084, 0x088, 0x0AC, 0x0B0, 0x0B4, 0x0B8,
    0x0BC, 0x0C0, 0x0C4, 0x0C8, 0x0CC, 0x0D4, 0x0D8, 0x0DC, 0x0E0, 0x0E4,
    0x0E8, 0x0EC, 0x0F0, 0x0F4, 0x0F8, 0x0FC, 0x100, 0x10C, 0x110, 0x120,
    0x130, 0x14C,
    /* CM_WKUP (base 0x400) */
    0x404, 0x408, 0x40C, 0x410, 0x414, 0x4B0, 0x4B4, 0x4B8, 0x4BC, 0x4C0,
    0x4C4, 0x4C8, 0x4D4,
    /* CM_MPU (base 0x600) */
    0x604,
    /* CM_RTC (base 0x800) */
    0x800,
    /* CM_GFX (base 0x900) */
    0x904, 0x910, 0x914,
    /* CM_CEFUSE (base 0xA00) */
    0xA20,
};

/*
 * PRM_RSTCTRL (PRM_DEVICE_MOD 0xF00 + offset 0x0 -- TRM spruh73q 8.1.4.2 /
 * kernel arch/arm/mach-omap2/prm33xx.h AM33XX_PRM_RSTCTRL). am33xx_restart()
 * (mach-omap2/am33xx-restart.c) is the actual `reboot` syscall handler on
 * this SoC: it writes RST_GLOBAL_WARM_SW (bit0, or COLD_SW bit1 for
 * REBOOT_COLD) here and never touches WDT1. Without a side effect on this
 * write the guest reboot request is silently absorbed into the flat store.
 */
#define PRCM_RSTCTRL_OFFSET          0xF00
#define PRCM_RST_GLOBAL_WARM_SW_MASK (1 << 0)
#define PRCM_RST_GLOBAL_COLD_SW_MASK (1 << 1)

/* --- DPLL IDLEST -> paired CLKMODE offset (absolute) ------------------- */
struct am335x_prcm_dpll_pair {
    hwaddr idlest;
    hwaddr clkmode;
};

static const struct am335x_prcm_dpll_pair am335x_prcm_dpll_pairs[] = {
    { 0x420, 0x488 }, /* CM_IDLEST_DPLL_MPU  -> CM_CLKMODE_DPLL_MPU  */
    { 0x434, 0x494 }, /* CM_IDLEST_DPLL_DDR  -> CM_CLKMODE_DPLL_DDR  */
    { 0x448, 0x498 }, /* CM_IDLEST_DPLL_DISP -> CM_CLKMODE_DPLL_DISP */
    { 0x45C, 0x490 }, /* CM_IDLEST_DPLL_CORE -> CM_CLKMODE_DPLL_CORE */
    { 0x470, 0x48C }, /* CM_IDLEST_DPLL_PER  -> CM_CLKMODE_DPLL_PER  */
};

/* CLKCTRL MODULEMODE field (bits[1:0]). */
#define CLKCTRL_MODULEMODE_MASK   0x3
#define CLKCTRL_IDLEST_SHIFT      16
#define CLKCTRL_IDLEST_MASK       (0x3u << CLKCTRL_IDLEST_SHIFT)
#define CLKCTRL_IDLEST_FUNC       0x0   /* module is fully functional */
#define CLKCTRL_IDLEST_DISABLED   0x3   /* module is disabled */

/* DPLL CLKMODE DPLL_EN field (bits[2:0]) and the two "running" encodings. */
#define DPLL_EN_MASK              0x7
#define DPLL_EN_MN_BYPASS         0x4
#define DPLL_EN_LOW_POWER_BYPASS  0x5

#define DPLL_IDLEST_ST_MN_BYPASS  0x00000100 /* bit8 */
#define DPLL_IDLEST_ST_DPLL_CLK   0x00000001 /* bit0 */

static bool am335x_prcm_is_clkctrl(hwaddr offset)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(am335x_prcm_clkctrl_offsets); i++) {
        if (am335x_prcm_clkctrl_offsets[i] == offset) {
            return true;
        }
    }
    return false;
}

/* Returns the paired CLKMODE offset for a DPLL IDLEST offset, or -1 if
 * `offset` is not a DPLL IDLEST register. */
static hwaddr am335x_prcm_dpll_clkmode_for(hwaddr offset)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(am335x_prcm_dpll_pairs); i++) {
        if (am335x_prcm_dpll_pairs[i].idlest == offset) {
            return am335x_prcm_dpll_pairs[i].clkmode;
        }
    }
    return (hwaddr)-1;
}

static uint32_t am335x_prcm_clkctrl_value(AM335xPrcmState *s, hwaddr offset)
{
    uint32_t v = s->regs[offset >> 2];
    uint32_t modulemode = v & CLKCTRL_MODULEMODE_MASK;
    uint32_t idlest = modulemode ? CLKCTRL_IDLEST_FUNC : CLKCTRL_IDLEST_DISABLED;

    return (v & ~CLKCTRL_IDLEST_MASK) | (idlest << CLKCTRL_IDLEST_SHIFT);
}

static uint32_t am335x_prcm_dpll_idlest_value(AM335xPrcmState *s,
                                              hwaddr clkmode_offset)
{
    uint32_t en = s->regs[clkmode_offset >> 2] & DPLL_EN_MASK;

    if (en == DPLL_EN_MN_BYPASS || en == DPLL_EN_LOW_POWER_BYPASS) {
        return DPLL_IDLEST_ST_MN_BYPASS;
    }
    return DPLL_IDLEST_ST_DPLL_CLK;
}

/* --- Reset -------------------------------------------------------------- */

/*
 * DPLL post-divider registers (CM_DIV_Mx_DPLL_*, CM_WKUP base 0x400).  The
 * Linux ti divider clocks read these at registration and fault with "Zero
 * divisor" (CLK_DIVIDER_ALLOW_ZERO is not set) if the divider field is 0, so
 * reset them to a divisor of 1 until software programs the real value.
 */
static const hwaddr am335x_prcm_divider_offsets[] = {
    0x480, /* CM_DIV_M4_DPLL_CORE */
    0x484, /* CM_DIV_M5_DPLL_CORE */
    0x4D8, /* CM_DIV_M6_DPLL_CORE */
    0x4A0, /* CM_DIV_M2_DPLL_DDR  */
    0x4A4, /* CM_DIV_M2_DPLL_DISP */
    0x4A8, /* CM_DIV_M2_DPLL_MPU  */
    0x4AC, /* CM_DIV_M2_DPLL_PER  */
};

/*
 * DPLL multiplier/divider (CM_CLKSEL_DPLL_*, DPLL_MULT bits[18:8], DPLL_DIV
 * bits[6:0]).  We bypass u-boot -- which normally locks the DPLLs -- yet
 * report them locked (see am335x_prcm_dpll_idlest_value), so the kernel does
 * not reprogram them and instead reads these registers to derive the DPLL
 * output rate.  Seed a plausible locked ratio so the derived L3/L4 interface
 * and functional clocks have a non-zero rate; otherwise the ti-sysc timer
 * probe fails with -ENODEV because clk_get_rate() returns 0.
 */
static const hwaddr am335x_prcm_clksel_dpll_offsets[] = {
    0x42C, /* CM_CLKSEL_DPLL_MPU    */
    0x468, /* CM_CLKSEL_DPLL_CORE   */
    0x440, /* CM_CLKSEL_DPLL_DDR    */
    0x454, /* CM_CLKSEL_DPLL_DISP   */
    0x49C, /* CM_CLKSEL_DPLL_PERIPH */
};
#define AM335X_PRCM_DPLL_MN  ((1000u << 8) | 23u) /* DPLL_MULT=1000, DPLL_DIV=23 */

static void am335x_prcm_reset_regs(AM335xPrcmState *s)
{
    int i;

    memset(s->regs, 0, sizeof(s->regs));

    for (i = 0; i < ARRAY_SIZE(am335x_prcm_divider_offsets); i++) {
        s->regs[am335x_prcm_divider_offsets[i] >> 2] = 1;
    }
    for (i = 0; i < ARRAY_SIZE(am335x_prcm_clksel_dpll_offsets); i++) {
        s->regs[am335x_prcm_clksel_dpll_offsets[i] >> 2] = AM335X_PRCM_DPLL_MN;
    }
}

static void am335x_prcm_reset(DeviceState *dev)
{
    AM335xPrcmState *s = AM335X_PRCM(dev);

    am335x_prcm_reset_regs(s);
}

/* --- MMIO ---------------------------------------------------------------- */

static uint64_t am335x_prcm_read(void *opaque, hwaddr addr, unsigned size)
{
    AM335xPrcmState *s = AM335X_PRCM(opaque);
    hwaddr dpll_clkmode;

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read size %u at offset 0x%" HWADDR_PRIx "\n",
                      __func__, size, addr);
        return 0;
    }

    if (am335x_prcm_is_clkctrl(addr)) {
        return am335x_prcm_clkctrl_value(s, addr);
    }

    dpll_clkmode = am335x_prcm_dpll_clkmode_for(addr);
    if (dpll_clkmode != (hwaddr)-1) {
        return am335x_prcm_dpll_idlest_value(s, dpll_clkmode);
    }

    return s->regs[addr >> 2];
}

static void am335x_prcm_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned size)
{
    AM335xPrcmState *s = AM335X_PRCM(opaque);

    if (size != 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write size %u at offset 0x%" HWADDR_PRIx "\n",
                      __func__, size, addr);
        return;
    }

    s->regs[addr >> 2] = (uint32_t)value;

    if (addr == PRCM_RSTCTRL_OFFSET &&
        (value & (PRCM_RST_GLOBAL_WARM_SW_MASK |
                  PRCM_RST_GLOBAL_COLD_SW_MASK))) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static const MemoryRegionOps am335x_prcm_ops = {
    .read = am335x_prcm_read,
    .write = am335x_prcm_write,
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

static void am335x_prcm_init(Object *obj)
{
    AM335xPrcmState *s = AM335X_PRCM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &am335x_prcm_ops, s,
                          TYPE_AM335X_PRCM, AM335X_PRCM_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const VMStateDescription vmstate_am335x_prcm = {
    .name = TYPE_AM335X_PRCM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, AM335xPrcmState,
                             AM335X_PRCM_MMIO_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void am335x_prcm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, am335x_prcm_reset);
    dc->vmsd = &vmstate_am335x_prcm;
    dc->desc = "TI AM335x Clock Module / PRCM (PRM+CM)";
}

static const TypeInfo am335x_prcm_info = {
    .name          = TYPE_AM335X_PRCM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xPrcmState),
    .instance_init = am335x_prcm_init,
    .class_init    = am335x_prcm_class_init,
};

static void am335x_prcm_register_types(void)
{
    type_register_static(&am335x_prcm_info);
}

type_init(am335x_prcm_register_types)
