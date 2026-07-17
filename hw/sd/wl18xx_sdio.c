/*
 * TI WiLink8 (wl1835 / wl18xx) SDIO WiFi function -- "clean probe" model.
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
 * The BeagleBone Green Wireless carries a TI WiLink8 module (wl1835) whose
 * WiFi side is an SDIO card wired to the AM335x third MMC/SD host controller
 * (MMCHS2). This models that SDIO card just far enough for the mainline Linux
 * wlcore/wl18xx driver stack to enumerate it, bind, and run its probe up to
 * (but not including) firmware download -- the same "structural, not
 * functional" bar this project uses for its other new-peripheral stubs
 * (hw/ssi/am335x_mcspi.c "clean probe", hw/net/dp83867_phy.c "clean attach").
 * No 802.11 MAC/PHY/RF is modelled; there is no radio here, only the register-
 * and enumeration-level behaviour the driver's probe path exercises.
 *
 * ---------------------------------------------------------------------------
 * Why it is an SDMMC_COMMON subclass on the SD bus
 * ---------------------------------------------------------------------------
 * QEMU's SD framework routes every card access through hw/sd/core.c, whose
 * get_card() casts the single bus child with SDMMC_COMMON(). A card must
 * therefore be a QOM subtype of TYPE_SDMMC_COMMON (like sd-card and emmc).
 * The generic SD memory-card state machine in hw/sd/sd.c (sd_do_command)
 * bails out immediately when the card has no block backend and only stubs
 * CMD5/52/53 as sd_cmd_optional, so it cannot serve an I/O-only SDIO card.
 * This device therefore subclasses TYPE_SDMMC_COMMON but overrides the whole
 * SDCardClass method set (do_command / read_byte / write_byte / ...) with an
 * SDIO command engine of its own. It embeds an SDState purely for the QOM/bus
 * layout get_card() requires; it keeps all of its own state in the fields
 * below and never touches SDState's SD-memory registers. (SDState's struct was
 * moved to hw/sd/sdmmc-internal.h so this sibling file can embed it.)
 *
 * ---------------------------------------------------------------------------
 * The Linux driver probe boundary this model targets (kernel 7.x)
 * ---------------------------------------------------------------------------
 * SDIO enumeration -- drivers/mmc/core/{sdio.c,sdio_cis.c}:
 *   - CMD5 IO_SEND_OP_COND: R4 must report 2 I/O functions (OCR bits 30:28)
 *     and be "ready" (bit 31). Memory-present (bit 27) is 0: this is an
 *     I/O-only card, so the SD-memory path (CMD2/ACMD41) is skipped.
 *   - CMD3 gets an RCA (R6); CMD7 selects the card (R1).
 *   - CCCR (sdio_read_cccr, sdio.c:144): register 0x00 low nibble is the CCCR
 *     format version (must be <= 3), high nibble is the SDIO spec version and
 *     is stored as card->cccr.sdio_vsn. wl18xx is told apart from wl12xx by
 *     sdio_vsn == SDIO_SDIO_REV_3_00 (== 4) (wlcore/sdio.c:310); we report
 *     0x43 so it is detected as wl18xx.
 *   - CIS (sdio_read_cis / cistpl_manfid / cistpl_funce_func): the driver
 *     match key is SDIO vendor 0x0097 (TI) / device 0x4076 from CISTPL_MANFID
 *     (include/linux/mmc/sdio_ids.h SDIO_VENDOR_ID_TI /
 *     SDIO_DEVICE_ID_TI_WL1271; wl18xx reuses the wl1271 id). Function 2 is
 *     the WLAN function (FBR standard interface code 0x07) and is the only one
 *     wlcore binds (wlcore/sdio.c:266 "if (func->num != 0x02) return
 *     -ENODEV"). The CISTPL_FUNCE (function) tuple must carry >= 42 data bytes
 *     for an SDIO 3.00 card (sdio_cis.c cistpl_funce_func min_size).
 *   - sdio_enable_func(fn2): writes CCCR I/O-Enable (0x02) and polls CCCR
 *     I/O-Ready (0x03); we mirror enable->ready so the poll completes at once.
 *
 * wl18xx power-on / get_hw_info -- drivers/net/wireless/ti/wlcore + wl18xx:
 *   - wlcore addresses the chip through a driver-programmed partition table
 *     (wlcore/io.c wlcore_translate_addr): a virtual chip address is mapped
 *     into the SDIO function-2 byte space. The table is written as eight
 *     raw 32-bit words at HW_PARTITION_REGISTERS_ADDR (0x1FFC0) via CMD53 on
 *     function 2 (wlcore/io.h HW_PART*_{SIZE,START}_ADDR). We track those
 *     eight words and reverse-translate every function-2 windowed access back
 *     to a chip address, so the chip-register reads below resolve regardless
 *     of which partition (PART_BOOT / PART_DOWN / ...) is active.
 *   - The ELP wake-up write, raw_write32(HW_ACCESS_ELP_CTRL_REG=0x1FFFC, 1),
 *     goes via the function-0 byte path (wlcore/sdio.c:106 sdio_f0_writeb);
 *     it is accepted and stored.
 *   - The one register value that must be correct is REG_CHIP_ID_B: for
 *     wl18xx that is chip address 0x0081542C (wl18xx/reg.h WL18XX_REG_CHIP_ID_B
 *     = WL18XX_REGISTERS_BASE + 0x1542C). wl18xx_identify_chip
 *     (wl18xx/main.c) requires it to read back CHIP_ID_185x_PG20 == 0x06030111
 *     or probe fails -ENODEV. Fuse/MAC reads (0xA026xx) may read 0 -- the
 *     driver then falls back to a random MAC (wl18xx/main.c wl18xx_get_mac).
 *
 * ---------------------------------------------------------------------------
 * Where the model deliberately stops (documented gap)
 * ---------------------------------------------------------------------------
 * The chip firmware blob "ti-connectivity/wl18xx-fw-4.bin" is only requested
 * (request_firmware) at interface-up, well after probe (wlcore/main.c
 * wl12xx_chip_wakeup -> wl12xx_fetch_firmware, reached from mac80211 .start).
 * This model does NOT implement the firmware upload/boot handshake or any
 * 802.11 behaviour: windowed writes to the chip register/memory space are
 * accepted and discarded, and every chip register other than CHIP_ID_B reads
 * back 0. So a clean probe reaches driver bind + chip identification and the
 * creation of the wlan interface's phy; bringing the interface up (which needs
 * the real firmware) is out of scope, the same structural boundary as this
 * project's other clean-probe/clean-attach peripheral stubs.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "migration/vmstate.h"
#include "hw/sd/sd.h"
#include "hw/sd/wl18xx_sdio.h"
#include "sdmmc-internal.h"
#include "trace.h"

/* ------------------------------------------------------------------------- */
/* Identity / fixed values (see file header for the driver-source citations). */

/* SDIO vendor/device the wlcore SDIO glue matches (TI / WL1271, reused by
 * wl18xx). include/linux/mmc/sdio_ids.h. */
#define WL18XX_SDIO_VENDOR      0x0097
#define WL18XX_SDIO_DEVICE      0x4076

/* Number of SDIO I/O functions (R4 bits 30:28). wl18xx exposes 2; the WLAN
 * function that wlcore binds is #2. */
#define WL18XX_NUM_FUNCS        2

/*
 * R4 OCR voltage window reported to CMD5; bit 31 = ready, bits 30:28 =
 * #functions, bit 27 (memory present) = 0 (I/O-only card). We advertise both
 * the usual 2.7-3.6V range (bits 15..23) AND 1.8V (MMC_VDD_165_195, bit 7):
 * on the Green Wireless the mmc3 vmmc-supply is the fixed 1.8V wlan_en_reg, so
 * the host's ocr_avail is 1.8V-only and mmc_select_voltage() needs the card to
 * advertise 1.8V or SDIO init fails -EINVAL (mmc_attach_sdio). Advertising the
 * wider range keeps the model usable on a 3.3V host too.
 */
#define WL18XX_OCR_WINDOW       0x00FF8080u

/*
 * R5 status half-word for a good CMD52/CMD53: IO_CURRENT_STATE = CMD (1) in
 * bits 13:12, no error bits set. Linux reads the CMD52 data byte as
 * resp[0] & 0xFF and the status as resp[0] & 0xCB00 (include/linux/mmc/sdio.h
 * R5_STATUS / R5_IO_CURRENT_STATE).
 */
#define WL18XX_R5_OK            0x1000u

/* CCCR register 0x00: high nibble = SDIO spec 3.00 (== 4, selects wl18xx),
 * low nibble = CCCR/FBR format 3.00 (== 3, must be <= 3). */
#define WL18XX_CCCR_REV_BYTE    0x43

/* CCCR capabilities (0x08): SMB = support multi-block transfer. */
#define WL18XX_CCCR_CAPS        0x02

/* Layout of the (function-0) config address space we synthesise. */
#define WL18XX_FBR1_BASE        0x0100
#define WL18XX_FBR2_BASE        0x0200
#define WL18XX_COMMON_CIS_ADDR  0x1000
#define WL18XX_FN1_CIS_ADDR     0x1080
#define WL18XX_FN2_CIS_ADDR     0x1100

/* wlcore fixed control registers in the function-2 byte space
 * (wlcore/io.h, wlcore/wlcore.h). */
#define WL18XX_HW_PART_REGS_ADDR 0x1FFC0  /* eight partition words: 0x1FFC0.. */
#define WL18XX_HW_PART_REGS_END  0x1FFE0
#define WL18XX_HW_ELP_CTRL_REG   0x1FFFC  /* ELP wake-up control */

/* The one chip register whose value matters, and the value the wl18xx driver
 * demands to continue (wl18xx/reg.h WL18XX_REG_CHIP_ID_B, CHIP_ID_185x_PG20). */
#define WL18XX_CHIP_ID_B_ADDR    0x0081542Cu
#define WL18XX_CHIP_ID_185x_PG20 0x06030111u

/* ------------------------------------------------------------------------- */

OBJECT_DECLARE_SIMPLE_TYPE(Wl18xxSdioState, WL18XX_SDIO)

struct Wl18xxSdioState {
    SDState parent_obj;         /* embedded for QOM/bus layout only */

    /* SDIO card enumeration state */
    uint16_t rca;               /* assigned by CMD3, matched by CMD7 */
    bool selected;              /* CMD7 has selected this card */

    /* CCCR (function-0) writable registers we track */
    uint8_t io_enable;          /* CCCR 0x02 IOEx */
    uint8_t io_ready;           /* CCCR 0x03 IORx (mirrors io_enable) */
    uint8_t int_enable;         /* CCCR 0x04 IENx */
    uint8_t bus_iface;          /* CCCR 0x07 bus interface control */
    uint16_t fn0_blksize;       /* CCCR 0x10-0x11 */
    uint16_t fn_blksize[WL18XX_NUM_FUNCS + 1]; /* per-function FBR block size */

    /* wl18xx register-window state (function 2) */
    uint32_t part[8];           /* partition table words at 0x1FFC0.. */
    uint32_t elp;               /* ELP control register */

    /* CMD53 data-transfer cursor */
    bool xfer_active;
    bool xfer_write;
    bool xfer_incr;             /* CMD53 OP bit: increment address vs FIFO */
    uint8_t xfer_fn;
    uint32_t xfer_addr;
    uint32_t xfer_len;
    uint32_t xfer_pos;
    uint8_t xfer_buf[512];      /* read pre-fill buffer (probe xfers are <=4B) */
};

/* Common CIS (function 0): manufacturer id + fn0 FUNCE. */
static const uint8_t wl18xx_common_cis[] = {
    /* CISTPL_MANFID: TI vendor 0x0097, device 0x4076 (little-endian) */
    0x20, 0x04, 0x97, 0x00, 0x76, 0x40,
    /* CISTPL_FUNCE common (type 0): fn0 max block size 512, max speed code */
    0x22, 0x04, 0x00, 0x00, 0x02, 0x32,
    /* CISTPL_END */
    0xFF,
};

/*
 * Per-function CIS (functions 1 and 2). Carries the manufacturer id again (so
 * func->vendor/device match without relying on the common-CIS fallback) plus
 * a CISTPL_FUNCE (function, type 1) tuple. For an SDIO 3.00 card the FUNCE
 * function tuple must be >= 42 data bytes (drivers/mmc/core/sdio_cis.c
 * cistpl_funce_func); byte [12..13] is TPLFE_MAX_BLK_SIZE (512 here).
 */
static const uint8_t wl18xx_func_cis[] = {
    /* CISTPL_MANFID: TI vendor 0x0097, device 0x4076 */
    0x20, 0x04, 0x97, 0x00, 0x76, 0x40,
    /* CISTPL_FUNCE function (type 1), 42 data bytes */
    0x22, 0x2A,
    0x01,                   /* [0]  TPLFE_TYPE = 1 (function) */
    0x00,                   /* [1]  function info */
    0x00,                   /* [2]  standard IO device rev */
    0x00, 0x00, 0x00, 0x00, /* [3..6]   card PSN */
    0x00, 0x00, 0x00, 0x00, /* [7..10]  CSA size */
    0x00,                   /* [11] CSA property */
    0x00, 0x02,             /* [12..13] TPLFE_MAX_BLK_SIZE = 512 */
    0x00, 0x00, 0x00, 0x00, /* [14..17] OCR */
    0x00, 0x00, 0x00, 0x00, /* [18..21] power */
    0x00, 0x00,             /* [22..23] power */
    0x00, 0x00,             /* [24..25] min bandwidth */
    0x00, 0x00,             /* [26..27] optimal bandwidth */
    0x00, 0x00,             /* [28..29] enable timeout (x10 ms) */
    0x00, 0x00,             /* [30..31] high-power */
    0x00, 0x00, 0x00, 0x00, /* [32..35] */
    0x00, 0x00, 0x00, 0x00, /* [36..39] */
    0x00, 0x00,             /* [40..41] -> 42 bytes total */
    /* CISTPL_END */
    0xFF,
};

/* ------------------------------------------------------------------------- */
/* Function-0 config space: CCCR + FBR + CIS + ELP control.                  */

static uint8_t wl18xx_cfg_readb(Wl18xxSdioState *s, uint32_t addr)
{
    if (addr < 0x100) {                 /* CCCR */
        switch (addr) {
        case 0x00: return WL18XX_CCCR_REV_BYTE;
        case 0x01: return 0x00;         /* SD spec revision */
        case 0x02: return s->io_enable;
        case 0x03: return s->io_ready;
        case 0x04: return s->int_enable;
        case 0x05: return 0x00;         /* INTx pending */
        case 0x07: return s->bus_iface;
        case 0x08: return WL18XX_CCCR_CAPS;
        case 0x09: return WL18XX_COMMON_CIS_ADDR & 0xFF;
        case 0x0A: return (WL18XX_COMMON_CIS_ADDR >> 8) & 0xFF;
        case 0x0B: return (WL18XX_COMMON_CIS_ADDR >> 16) & 0xFF;
        case 0x10: return s->fn0_blksize & 0xFF;
        case 0x11: return (s->fn0_blksize >> 8) & 0xFF;
        default:   return 0x00;         /* power/speed/UHS/... */
        }
    }
    if (addr >= WL18XX_FBR1_BASE && addr < WL18XX_FBR1_BASE + 0x100) {
        switch (addr - WL18XX_FBR1_BASE) {
        case 0x00: return 0x00;         /* fn1 interface code: no standard */
        case 0x09: return WL18XX_FN1_CIS_ADDR & 0xFF;
        case 0x0A: return (WL18XX_FN1_CIS_ADDR >> 8) & 0xFF;
        case 0x0B: return (WL18XX_FN1_CIS_ADDR >> 16) & 0xFF;
        case 0x10: return s->fn_blksize[1] & 0xFF;
        case 0x11: return (s->fn_blksize[1] >> 8) & 0xFF;
        default:   return 0x00;
        }
    }
    if (addr >= WL18XX_FBR2_BASE && addr < WL18XX_FBR2_BASE + 0x100) {
        switch (addr - WL18XX_FBR2_BASE) {
        case 0x00: return 0x07;         /* fn2 interface code: WLAN */
        case 0x09: return WL18XX_FN2_CIS_ADDR & 0xFF;
        case 0x0A: return (WL18XX_FN2_CIS_ADDR >> 8) & 0xFF;
        case 0x0B: return (WL18XX_FN2_CIS_ADDR >> 16) & 0xFF;
        case 0x10: return s->fn_blksize[2] & 0xFF;
        case 0x11: return (s->fn_blksize[2] >> 8) & 0xFF;
        default:   return 0x00;
        }
    }
    if (addr >= WL18XX_COMMON_CIS_ADDR &&
        addr < WL18XX_COMMON_CIS_ADDR + sizeof(wl18xx_common_cis)) {
        return wl18xx_common_cis[addr - WL18XX_COMMON_CIS_ADDR];
    }
    if (addr >= WL18XX_FN1_CIS_ADDR &&
        addr < WL18XX_FN1_CIS_ADDR + sizeof(wl18xx_func_cis)) {
        return wl18xx_func_cis[addr - WL18XX_FN1_CIS_ADDR];
    }
    if (addr >= WL18XX_FN2_CIS_ADDR &&
        addr < WL18XX_FN2_CIS_ADDR + sizeof(wl18xx_func_cis)) {
        return wl18xx_func_cis[addr - WL18XX_FN2_CIS_ADDR];
    }
    if ((addr & ~3u) == WL18XX_HW_ELP_CTRL_REG) {
        return (s->elp >> (8 * (addr & 3))) & 0xFF;
    }
    return 0x00;
}

static void wl18xx_cfg_writeb(Wl18xxSdioState *s, uint32_t addr, uint8_t val)
{
    if (addr < 0x100) {                 /* CCCR */
        switch (addr) {
        case 0x02:                      /* IOEx: enabling completes at once */
            s->io_enable = val;
            s->io_ready = val;
            break;
        case 0x04: s->int_enable = val; break;
        case 0x06: break;               /* I/O abort: no-op */
        case 0x07: s->bus_iface = val; break;
        case 0x10: s->fn0_blksize = (s->fn0_blksize & 0xFF00) | val; break;
        case 0x11: s->fn0_blksize = (s->fn0_blksize & 0x00FF) | (val << 8); break;
        default: break;
        }
        return;
    }
    if (addr == WL18XX_FBR1_BASE + 0x10) {
        s->fn_blksize[1] = (s->fn_blksize[1] & 0xFF00) | val;
    } else if (addr == WL18XX_FBR1_BASE + 0x11) {
        s->fn_blksize[1] = (s->fn_blksize[1] & 0x00FF) | (val << 8);
    } else if (addr == WL18XX_FBR2_BASE + 0x10) {
        s->fn_blksize[2] = (s->fn_blksize[2] & 0xFF00) | val;
    } else if (addr == WL18XX_FBR2_BASE + 0x11) {
        s->fn_blksize[2] = (s->fn_blksize[2] & 0x00FF) | (val << 8);
    } else if ((addr & ~3u) == WL18XX_HW_ELP_CTRL_REG) {
        int sh = 8 * (addr & 3);
        s->elp = (s->elp & ~(0xFFu << sh)) | ((uint32_t)val << sh);
    }
    /* other config writes are absorbed */
}

/* ------------------------------------------------------------------------- */
/* Function-2 window: partition table + reverse-translated chip registers.   */

/*
 * Inverse of wlcore_translate_addr() (wlcore/io.c): given an SDIO function-2
 * byte offset produced by the driver's virtual->physical translation, recover
 * the chip address it referred to under the currently-programmed partition.
 * part[] holds the eight words as written at 0x1FFC0:
 *   [0]=mem.size  [1]=mem.start  [2]=reg.size  [3]=reg.start
 *   [4]=mem2.size [5]=mem2.start [6]=mem3.size [7]=mem3.start
 */
static uint32_t wl18xx_reverse_translate(Wl18xxSdioState *s, uint32_t phys)
{
    uint32_t mem_sz = s->part[0], mem_st = s->part[1];
    uint32_t reg_sz = s->part[2], reg_st = s->part[3];
    uint32_t m2_sz = s->part[4], m2_st = s->part[5];
    uint32_t m3_st = s->part[7];

    if (phys < mem_sz) {
        return phys + mem_st;
    }
    if (phys < mem_sz + reg_sz) {
        return phys - mem_sz + reg_st;
    }
    if (phys < mem_sz + reg_sz + m2_sz) {
        return phys - mem_sz - reg_sz + m2_st;
    }
    return phys - mem_sz - reg_sz - m2_sz + m3_st;
}

static uint32_t wl18xx_chip_reg_read(Wl18xxSdioState *s, uint32_t chip_addr)
{
    switch (chip_addr) {
    case WL18XX_CHIP_ID_B_ADDR:
        /* wl18xx_identify_chip() requires exactly this to bind. */
        return WL18XX_CHIP_ID_185x_PG20;
    default:
        /* Fuse/MAC and everything else read 0: the driver copes (random MAC,
         * default config). No firmware/register behaviour is modelled. */
        return 0;
    }
}

static uint8_t wl18xx_win_readb(Wl18xxSdioState *s, uint32_t addr)
{
    if (addr >= WL18XX_HW_PART_REGS_ADDR && addr < WL18XX_HW_PART_REGS_END) {
        uint32_t w = s->part[(addr - WL18XX_HW_PART_REGS_ADDR) >> 2];
        return (w >> (8 * (addr & 3))) & 0xFF;
    }
    if ((addr & ~3u) == WL18XX_HW_ELP_CTRL_REG) {
        return (s->elp >> (8 * (addr & 3))) & 0xFF;
    }
    {
        uint32_t chip = wl18xx_reverse_translate(s, addr & ~3u);
        uint32_t val = wl18xx_chip_reg_read(s, chip);
        return (val >> (8 * (addr & 3))) & 0xFF;
    }
}

static void wl18xx_win_writeb(Wl18xxSdioState *s, uint32_t addr, uint8_t val)
{
    if (addr >= WL18XX_HW_PART_REGS_ADDR && addr < WL18XX_HW_PART_REGS_END) {
        uint32_t idx = (addr - WL18XX_HW_PART_REGS_ADDR) >> 2;
        int sh = 8 * (addr & 3);
        s->part[idx] = (s->part[idx] & ~(0xFFu << sh)) | ((uint32_t)val << sh);
        return;
    }
    if ((addr & ~3u) == WL18XX_HW_ELP_CTRL_REG) {
        int sh = 8 * (addr & 3);
        s->elp = (s->elp & ~(0xFFu << sh)) | ((uint32_t)val << sh);
        return;
    }
    /* Windowed writes (firmware upload, chip register writes) are accepted
     * and discarded -- beyond the clean-probe boundary (see file header). */
}

/* ------------------------------------------------------------------------- */
/* Byte access dispatch by SDIO function number.                             */

static uint8_t wl18xx_sdio_readb(Wl18xxSdioState *s, int fn, uint32_t addr)
{
    if (fn == 0) {
        return wl18xx_cfg_readb(s, addr);
    }
    if (fn == 2) {
        return wl18xx_win_readb(s, addr);
    }
    return 0;                           /* fn1 is never accessed at runtime */
}

static void wl18xx_sdio_writeb(Wl18xxSdioState *s, int fn, uint32_t addr,
                               uint8_t val)
{
    if (fn == 0) {
        wl18xx_cfg_writeb(s, addr, val);
    } else if (fn == 2) {
        wl18xx_win_writeb(s, addr, val);
    }
}

/* ------------------------------------------------------------------------- */
/* SDCardClass method overrides.                                             */

static uint32_t wl18xx_r1_status(Wl18xxSdioState *s)
{
    /* CURRENT_STATE (bits 12:9) | READY_FOR_DATA (bit 8) */
    int st = s->selected ? sd_transfer_state : sd_standby_state;
    return ((uint32_t)st << 9) | (1u << 8);
}

static int wl18xx_sdio_do_command(SDState *sd, SDRequest *req,
                                  uint8_t *response)
{
    Wl18xxSdioState *s = WL18XX_SDIO(sd);
    uint32_t arg = req->arg;

    trace_wl18xx_sdio_command(req->cmd, arg);

    switch (req->cmd) {
    case 0:   /* GO_IDLE_STATE */
        s->selected = false;
        s->xfer_active = false;
        return 0;                       /* no response */

    case 5: { /* IO_SEND_OP_COND -> R4 */
        uint32_t r4 = (1u << 31)                             /* ready */
                    | ((uint32_t)WL18XX_NUM_FUNCS << 28)     /* # functions */
                    | WL18XX_OCR_WINDOW;                     /* voltage window */
        stl_be_p(response, r4);
        return 4;
    }

    case 3: { /* SEND_RELATIVE_ADDR -> R6 */
        s->rca = 0x0001;
        stl_be_p(response, ((uint32_t)s->rca << 16) | 0x0000);
        return 4;
    }

    case 7: { /* SELECT/DESELECT_CARD -> R1(b) */
        uint16_t rca = arg >> 16;
        s->selected = (rca != 0) && (rca == s->rca);
        stl_be_p(response, wl18xx_r1_status(s));
        return 4;
    }

    case 52: { /* IO_RW_DIRECT -> R5 */
        bool wr = arg & (1u << 31);
        int fn = (arg >> 28) & 0x7;
        bool raw = arg & (1u << 27);
        uint32_t addr = (arg >> 9) & 0x1FFFF;
        uint8_t data = arg & 0xFF;
        uint8_t out;

        if (wr) {
            wl18xx_sdio_writeb(s, fn, addr, data);
            out = raw ? wl18xx_sdio_readb(s, fn, addr) : data;
        } else {
            out = wl18xx_sdio_readb(s, fn, addr);
        }
        trace_wl18xx_sdio_cmd52(wr ? "wr" : "rd", fn, addr, data, out);
        stl_be_p(response, WL18XX_R5_OK | out);
        return 4;
    }

    case 53: { /* IO_RW_EXTENDED -> R5, data on the data lines */
        bool wr = arg & (1u << 31);
        int fn = (arg >> 28) & 0x7;
        bool block = arg & (1u << 27);
        bool incr = arg & (1u << 26);
        uint32_t addr = (arg >> 9) & 0x1FFFF;
        uint32_t count = arg & 0x1FF;
        uint32_t bs = (fn <= WL18XX_NUM_FUNCS) ? s->fn_blksize[fn] : 0;
        uint32_t len;

        if (block) {
            len = count * (bs ? bs : 512);
        } else {
            len = count ? count : 512;
        }
        if (len > sizeof(s->xfer_buf)) {
            /* Only <=512B probe transfers are modelled; a larger transfer
             * would be a post-probe firmware upload (out of scope). */
            qemu_log_mask(LOG_UNIMP,
                          "wl18xx-sdio: CMD53 %u-byte transfer clamped to %zu "
                          "(firmware upload not modelled)\n",
                          len, sizeof(s->xfer_buf));
            len = sizeof(s->xfer_buf);
        }

        s->xfer_active = len != 0;
        s->xfer_write = wr;
        s->xfer_incr = incr;
        s->xfer_fn = fn;
        s->xfer_addr = addr;
        s->xfer_len = len;
        s->xfer_pos = 0;

        if (!wr) {
            for (uint32_t i = 0; i < len; i++) {
                uint32_t a = incr ? addr + i : addr;
                s->xfer_buf[i] = wl18xx_sdio_readb(s, fn, a);
            }
        }
        trace_wl18xx_sdio_cmd53(wr ? "wr" : "rd", fn, addr,
                                block ? "blk" : "byte", incr, len);
        stl_be_p(response, WL18XX_R5_OK);
        return 4;
    }

    default:
        /* CMD8 (SEND_IF_COND) and anything else: no response. The MMC core
         * treats a missing CMD8 response as "pre-2.0 / not an SD card" and
         * moves on to the SDIO path, which is what we want. */
        return 0;
    }
}

static uint8_t wl18xx_sdio_read_byte(SDState *sd)
{
    Wl18xxSdioState *s = WL18XX_SDIO(sd);
    uint8_t v;

    if (!s->xfer_active || s->xfer_write) {
        return 0;
    }
    v = s->xfer_buf[s->xfer_pos];
    if (++s->xfer_pos >= s->xfer_len) {
        s->xfer_active = false;
    }
    return v;
}

static void wl18xx_sdio_write_byte(SDState *sd, uint8_t value)
{
    Wl18xxSdioState *s = WL18XX_SDIO(sd);
    uint32_t a;

    if (!s->xfer_active || !s->xfer_write || s->xfer_pos >= s->xfer_len) {
        return;
    }
    a = s->xfer_incr ? s->xfer_addr + s->xfer_pos : s->xfer_addr;
    wl18xx_sdio_writeb(s, s->xfer_fn, a, value);
    if (++s->xfer_pos >= s->xfer_len) {
        s->xfer_active = false;
    }
}

static bool wl18xx_sdio_data_ready(SDState *sd)
{
    Wl18xxSdioState *s = WL18XX_SDIO(sd);
    return s->xfer_active && !s->xfer_write;
}

static bool wl18xx_sdio_receive_ready(SDState *sd)
{
    Wl18xxSdioState *s = WL18XX_SDIO(sd);
    return s->xfer_active && s->xfer_write;
}

static bool wl18xx_sdio_get_inserted(SDState *sd)
{
    return true;                        /* on-package, always present */
}

static bool wl18xx_sdio_get_readonly(SDState *sd)
{
    return false;
}

static void wl18xx_sdio_set_voltage(SDState *sd, uint16_t millivolts)
{
    /* accepted; no voltage-dependent behaviour is modelled */
}

static uint8_t wl18xx_sdio_get_dat_lines(SDState *sd)
{
    return 0x0F;                        /* 4-bit bus, idle high */
}

static bool wl18xx_sdio_get_cmd_line(SDState *sd)
{
    return true;
}

/* ------------------------------------------------------------------------- */

static void wl18xx_sdio_reset(DeviceState *dev)
{
    Wl18xxSdioState *s = WL18XX_SDIO(dev);

    s->rca = 0;
    s->selected = false;
    s->io_enable = 0;
    s->io_ready = 0;
    s->int_enable = 0;
    s->bus_iface = 0;
    s->fn0_blksize = 0;
    memset(s->fn_blksize, 0, sizeof(s->fn_blksize));
    memset(s->part, 0, sizeof(s->part));
    s->elp = 0;
    s->xfer_active = false;
    s->xfer_write = false;
    s->xfer_incr = false;
    s->xfer_fn = 0;
    s->xfer_addr = 0;
    s->xfer_len = 0;
    s->xfer_pos = 0;
}

static const VMStateDescription vmstate_wl18xx_sdio = {
    .name = TYPE_WL18XX_SDIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(rca, Wl18xxSdioState),
        VMSTATE_BOOL(selected, Wl18xxSdioState),
        VMSTATE_UINT8(io_enable, Wl18xxSdioState),
        VMSTATE_UINT8(io_ready, Wl18xxSdioState),
        VMSTATE_UINT8(int_enable, Wl18xxSdioState),
        VMSTATE_UINT8(bus_iface, Wl18xxSdioState),
        VMSTATE_UINT16(fn0_blksize, Wl18xxSdioState),
        VMSTATE_UINT16_ARRAY(fn_blksize, Wl18xxSdioState, WL18XX_NUM_FUNCS + 1),
        VMSTATE_UINT32_ARRAY(part, Wl18xxSdioState, 8),
        VMSTATE_UINT32(elp, Wl18xxSdioState),
        VMSTATE_BOOL(xfer_active, Wl18xxSdioState),
        VMSTATE_BOOL(xfer_write, Wl18xxSdioState),
        VMSTATE_BOOL(xfer_incr, Wl18xxSdioState),
        VMSTATE_UINT8(xfer_fn, Wl18xxSdioState),
        VMSTATE_UINT32(xfer_addr, Wl18xxSdioState),
        VMSTATE_UINT32(xfer_len, Wl18xxSdioState),
        VMSTATE_UINT32(xfer_pos, Wl18xxSdioState),
        VMSTATE_UINT8_ARRAY(xfer_buf, Wl18xxSdioState, 512),
        VMSTATE_END_OF_LIST()
    }
};

static void wl18xx_sdio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = SDMMC_COMMON_CLASS(klass);

    dc->desc = "TI WiLink8 (wl1835) SDIO WiFi function";
    dc->vmsd = &vmstate_wl18xx_sdio;
    device_class_set_legacy_reset(dc, wl18xx_sdio_reset);
    /* Reason: on-package SDIO device, wired from board code (like emmc). */
    dc->user_creatable = false;

    sc->do_command = wl18xx_sdio_do_command;
    sc->read_byte = wl18xx_sdio_read_byte;
    sc->write_byte = wl18xx_sdio_write_byte;
    sc->data_ready = wl18xx_sdio_data_ready;
    sc->receive_ready = wl18xx_sdio_receive_ready;
    sc->get_inserted = wl18xx_sdio_get_inserted;
    sc->get_readonly = wl18xx_sdio_get_readonly;
    sc->set_voltage = wl18xx_sdio_set_voltage;
    sc->get_dat_lines = wl18xx_sdio_get_dat_lines;
    sc->get_cmd_line = wl18xx_sdio_get_cmd_line;
}

static const TypeInfo wl18xx_sdio_info = {
    .name          = TYPE_WL18XX_SDIO,
    .parent        = TYPE_SDMMC_COMMON,
    .instance_size = sizeof(Wl18xxSdioState),
    .class_init    = wl18xx_sdio_class_init,
};

static void wl18xx_sdio_register_types(void)
{
    type_register_static(&wl18xx_sdio_info);
}

type_init(wl18xx_sdio_register_types)
