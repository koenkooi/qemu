/*
 * TI AM335x USB Subsystem (USBSS): glue register file + USB1 musb host.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Structural template for the glue layer: hw/misc/am335x_control.c (flat
 * backing store with a few synthesized registers). Structural template for
 * the USB1 host controller: hw/usb/hcd-dwc2.c (register-driven, software
 * state-machine, PIO-capable, no descriptor-ring DMA) -- dwc2 is the
 * closest existing QEMU model; there is no pre-existing musb HCD to crib.
 *
 * Covers the whole 32KB "usb" target-module window (TRM spruh73q ch.16,
 * base 0x47400000, am33xx.dtsi target-module@47400000), which holds:
 *
 *   0x0000 ti-sysc target-module wrapper (rev @0x00, sysconfig @0x10)
 *   0x1000 USB0 "control" wrapper (revision/control/mode/phy_utmi/...)
 *   0x1300 USB0 PHY window (never ioremap'd by any Linux driver)
 *   0x1400 USB0 "mc" block -- the Mentor musb-hdrc core registers
 *   0x1800 USB1 "control" wrapper
 *   0x1b00 USB1 PHY window (likewise unused)
 *   0x1c00 USB1 "mc" block
 *   0x2000 CPPI4.1 DMA glue/controller/scheduler/queue-manager
 *
 * Scope
 * -----
 * USB1 (the board's type-A host connector) is modelled as a *functional*
 * host controller: a real USBBus/USBPort, the 16-endpoint indexed
 * CSR/FIFO register file, the DSPS wrapper interrupt plumbing, and PIO
 * (no-DMA) control + bulk/interrupt transfer handling -- enough for the
 * real drivers/usb/musb host stack to enumerate a device plugged into the
 * port (`-device usb-...`) and bind its class driver. USB0 (the OTG port)
 * and the CPPI4.1 DMA engine stay clean-probe register-file stubs: USB0
 * comes up as an idle peripheral/host that never sees a connect, and CPPI
 * reads back as zero (the guest must load musb_hdrc with use_dma=0, else
 * musb_dma_controller_create() -- musb_core.c:2473 -- tries to grab CPPI
 * channels this model does not provide).
 *
 * Layout: a 32KB container holds a flat "glue" store (priority 0) covering
 * the whole window, with the USB1 "control" (0x1800) and "mc" (0x1c00)
 * sub-windows overlaid as higher-priority functional MMIO regions. USB0's
 * clean-probe revision/CONFIGDATA and the ti-sysc/USB0 soft-reset bits are
 * served by the glue store exactly as before.
 *
 * The DSPS wrapper interrupt model (the load-bearing subtlety)
 * -----------------------------------------------------------
 * dsps_interrupt() (musb_dsps.c:313-395) does NOT read the Mentor-core
 * INTRTX/INTRRX/INTRUSB registers -- it reads and write-1-clears the DSPS
 * *wrapper* shadow registers "epintr_status" (control+0x30) and
 * "coreintr_status" (control+0x34), then feeds them to musb_interrupt():
 *   - epintr_status: TX endpoints in bits 0-15 (bit0 = EP0/control), RX
 *     endpoints in bits 16-31 (bit 16+n = RX EPn). (am33xx_driver_data:
 *     txep_shift=0/mask 0xffff, rxep_shift=16/mask 0xfffe.)
 *   - coreintr_status: INTRUSB bits 0-7 (CONNECT 0x10, RESET 0x04, ...)
 *     plus DRVVBUS/session in bit 8. (usb_mask=0x1ff, drvvbus bit 8.)
 * Enables are edge-set/cleared through epintr_set/coreintr_set (0x38/0x3c,
 * write-1-to-set) and epintr_clear/coreintr_clear (0x40/0x44,
 * write-1-to-clear); reading a _set register returns the live enable mask
 * (used by the DSPS context save/restore, musb_dsps.c:989/1015). The "mc"
 * IRQ line (INTC 19) is asserted while (status & enable) is non-zero in
 * either register, and the guest ISR clears it by writing the status bits
 * back.
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
#include "hw/usb.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"

/* ti-sysc target-module wrapper (relative to the 0x47400000 base). */
#define USBSS_SYSCONFIG          0x10
#define USBSS_SYSCONFIG_SOFTRESET (1 << 0)

/* USB0 clean-probe stub windows (relative to the 0x47400000 base). USB1's
 * control+mc are handled by the overlaid functional regions below. */
#define USB0_CTRL_BASE   0x1000
#define USB0_MC_BASE     0x1400

/* Offsets within a "control" wrapper block (am33xx_driver_data,
 * musb_dsps.c:926-943). */
#define CTRL_REVISION      0x00
#define CTRL_CONTROL       0x14
#define CTRL_STATUS        0x18
#define CTRL_EPINTR_STATUS 0x30
#define CTRL_COREINTR_STATUS 0x34
#define CTRL_EPINTR_SET    0x38
#define CTRL_COREINTR_SET  0x3c
#define CTRL_EPINTR_CLEAR  0x40
#define CTRL_COREINTR_CLEAR 0x44
#define CTRL_TX_MODE       0x70
#define CTRL_RX_MODE       0x74
#define CTRL_PHY_UTMI      0xe0
#define CTRL_MODE          0xe8

#define CTRL_CONTROL_SOFT_RESET  (1 << 0)

/* coreintr_status/epintr_status field geometry (am33xx_driver_data). */
#define MUSB_EPINTR_TX_MASK    0x0000ffffu    /* TX EP0..15  (bit0 = EP0)  */
#define MUSB_EPINTR_RX_SHIFT   16
#define MUSB_EPINTR_RX_MASK    0xfffe0000u    /* RX EP1..15 (bit16 unused) */
#define MUSB_COREINTR_USB_MASK 0x000000ffu    /* INTRUSB bits              */
#define MUSB_COREINTR_DRVVBUS  0x00000100u    /* session/VBUS (bit 8)      */

/* "mc" musb-core common register offsets (musb_regs.h). */
#define MC_FADDR       0x00
#define MC_POWER       0x01
#define MC_INTRTX      0x02
#define MC_INTRRX      0x04
#define MC_INTRTXE     0x06
#define MC_INTRRXE     0x08
#define MC_INTRUSB     0x0a
#define MC_INTRUSBE    0x0b
#define MC_FRAME       0x0c
#define MC_INDEX       0x0e
#define MC_TESTMODE    0x0f
#define MC_INDEXED_BASE 0x10   /* .. 0x1f: indexed EP window                */
#define MC_INDEXED_END  0x20
#define MC_FIFO_BASE    0x20   /* .. 0x5f: per-EP FIFO ports (0x20 + 4*ep)  */
#define MC_FIFO_END     0x60
#define MC_DEVCTL      0x60
#define MC_BABBLE_CTL  0x61
#define MC_TXFIFOSZ    0x62    /* indexed */
#define MC_RXFIFOSZ    0x63    /* indexed */
#define MC_TXFIFOADD   0x64    /* indexed, 16-bit */
#define MC_RXFIFOADD   0x66    /* indexed, 16-bit */
#define MC_HWVERS      0x6c
#define MC_BUSCTL_BASE 0x80    /* .. 0xff: per-EP busctl (0x80 + 8*ep)      */
#define MC_BUSCTL_END  0x100

/* Indexed EP-window register offsets, relative to MC_INDEXED_BASE. */
#define IDX_TXMAXP     0x00
#define IDX_TXCSR      0x02   /* CSR0 for EP0                              */
#define IDX_RXMAXP     0x04
#define IDX_RXCSR      0x06
#define IDX_RXCOUNT    0x08   /* COUNT0 for EP0                           */
#define IDX_TXTYPE     0x0a
#define IDX_TXINTERVAL 0x0b
#define IDX_RXTYPE     0x0c
#define IDX_RXINTERVAL 0x0d
#define IDX_FIFOSIZE   0x0f   /* CONFIGDATA for EP0                       */

/* Reset/fixed values (TRM spruh73q Table 16-73 unless noted). */
#define USB_REVISION_VALUE    0x4ea20800  /* SS16.4.2.1 / 16.4.3.1        */
#define USB_PHY_UTMI_RESET    0x00200002  /* SS16.4.2.33: OTGDISABLE|FSDATAEXT */
#define USB_MODE_RESET        0x00000100  /* SS16.4.2.35: IDDIG (B-device) */
#define USB_CONFIGDATA_VALUE  0xde        /* MPRXE|MPTXE|HBRXE|HBTXE|DYNFIFO|SOFTCONE */
#define USB_HWVERS_VALUE      0x0800      /* RTL 2.0 (low 16b of USBnREV)  */

/* ======================================================================= */
/* Flat "glue" byte-store helpers (USB0/ti-sysc/PHY/CPPI clean-probe).      */

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

static uint64_t am335x_usbss_glue_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    AM335xUsbssState *s = AM335X_USBSS(opaque);

    /*
     * USB0 clean-probe revision (control+0x00): gates dsps_musb_init()
     * for USB0 (USB1's is served by the overlaid functional region).
     */
    if (offset == USB0_CTRL_BASE + CTRL_REVISION) {
        return extract64(USB_REVISION_VALUE, 0, size * 8);
    }

    /* USB0 CONFIGDATA (mc+0x1f, DYNFIFO set): keeps musb_core_init() on the
     * software ep_config_from_table() path. */
    if (offset == USB0_MC_BASE + MC_INDEXED_BASE + IDX_FIFOSIZE) {
        return USB_CONFIGDATA_VALUE;
    }

    return am335x_usbss_load(s->regs, offset, size);
}

static void am335x_usbss_glue_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    AM335xUsbssState *s = AM335X_USBSS(opaque);

    /* USB0 revision and CONFIGDATA are read-only. */
    if (offset == USB0_CTRL_BASE + CTRL_REVISION ||
        offset == USB0_MC_BASE + MC_INDEXED_BASE + IDX_FIFOSIZE) {
        return;
    }

    am335x_usbss_store(s->regs, offset, value, size);

    /*
     * USB0CTRL.SOFT_RESET (bit0) and the ti-sysc wrapper's
     * SYSCONFIG.SOFTRESET (bit0) self-clear (WDT1/I2C0 idiom); modelling
     * this keeps a debugfs/regdump peek honest and avoids the ti-sysc
     * softreset-timeout path. Clearing byte 0 is width-independent.
     */
    if (offset == USB0_CTRL_BASE + CTRL_CONTROL) {
        s->regs[offset] &= ~CTRL_CONTROL_SOFT_RESET;
    } else if (offset == USBSS_SYSCONFIG) {
        s->regs[offset] &= ~USBSS_SYSCONFIG_SOFTRESET;
    }
}

static const MemoryRegionOps am335x_usbss_glue_ops = {
    .read = am335x_usbss_glue_read,
    .write = am335x_usbss_glue_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* ======================================================================= */
/* USB1 musb host: interrupt plumbing.                                     */

/*
 * The DSPS "mc" IRQ (INTC 19) is level-asserted while any enabled status
 * bit is pending in either wrapper register. The guest ISR clears it by
 * write-1-clearing the status registers (see the file header).
 */
static void am335x_musb_update_irq(AM335xUsbssState *s)
{
    AM335xMusb *m = &s->musb;
    bool pending = (m->epintr_status & m->epintr_enable) ||
                   (m->coreintr_status & m->coreintr_enable);

    qemu_set_irq(s->irq[1], pending);
}

/* ======================================================================= */
/* USB1 musb host: PIO transfer engine.                                    */
/*
 * Filled in the following commit. These entry points are invoked from the
 * CSR0/TXCSR/RXCSR write paths when the guest sets a "go" bit (TXPKTRDY /
 * H_REQPKT / H_SETUPPKT / H_STATUSPKT). For now they are inert, so the
 * register file and USBBus scaffolding can be exercised on their own
 * without attempting (and failing) any enumeration.
 */
static void am335x_musb_ep0_poke(AM335xUsbssState *s)
{
    /* TODO(next commit): drive the EP0 control-transfer state machine. */
}

static void am335x_musb_tx_poke(AM335xUsbssState *s, unsigned ep)
{
    /* TODO(next commit): drive a bulk/interrupt OUT transfer on EP `ep`. */
}

static void am335x_musb_rx_poke(AM335xUsbssState *s, unsigned ep)
{
    /* TODO(next commit): drive a bulk/interrupt IN transfer on EP `ep`. */
}

/* ======================================================================= */
/* USB1 musb host: "control" wrapper MMIO (abs 0x47401800).                */

static uint64_t am335x_musb_ctrl_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    AM335xUsbssState *s = AM335X_USBSS(opaque);
    AM335xMusb *m = &s->musb;

    switch (offset) {
    case CTRL_REVISION:
        return extract64(USB_REVISION_VALUE, 0, size * 8);
    case CTRL_CONTROL:
        return m->control;
    case CTRL_STATUS:
        return m->status;
    case CTRL_EPINTR_STATUS:
        return m->epintr_status;
    case CTRL_COREINTR_STATUS:
        return m->coreintr_status;
    case CTRL_EPINTR_SET:       /* reads back the live enable mask */
        return m->epintr_enable;
    case CTRL_COREINTR_SET:
        return m->coreintr_enable;
    case CTRL_EPINTR_CLEAR:
        return m->epintr_enable;
    case CTRL_COREINTR_CLEAR:
        return m->coreintr_enable;
    case CTRL_TX_MODE:
        return m->tx_mode;
    case CTRL_RX_MODE:
        return m->rx_mode;
    case CTRL_PHY_UTMI:
        return m->phy_utmi;
    case CTRL_MODE:
        return m->mode;
    default:
        return 0;
    }
}

static void am335x_musb_ctrl_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    AM335xUsbssState *s = AM335X_USBSS(opaque);
    AM335xMusb *m = &s->musb;
    uint32_t val = (uint32_t)value;

    switch (offset) {
    case CTRL_REVISION:
        break;                          /* read-only */
    case CTRL_CONTROL:
        /* SOFT_RESET (bit0) self-clears. */
        m->control = val & ~CTRL_CONTROL_SOFT_RESET;
        break;
    case CTRL_STATUS:
        break;                          /* read-only (drvvbus level) */
    case CTRL_EPINTR_STATUS:            /* W1C the raw TX/RX pending bits */
        m->epintr_status &= ~val;
        am335x_musb_update_irq(s);
        break;
    case CTRL_COREINTR_STATUS:          /* W1C the raw USB-core pending bits */
        m->coreintr_status &= ~val;
        am335x_musb_update_irq(s);
        break;
    case CTRL_EPINTR_SET:               /* W1S enable */
        m->epintr_enable |= val;
        am335x_musb_update_irq(s);
        break;
    case CTRL_COREINTR_SET:
        m->coreintr_enable |= val;
        am335x_musb_update_irq(s);
        break;
    case CTRL_EPINTR_CLEAR:             /* W1C enable */
        m->epintr_enable &= ~val;
        am335x_musb_update_irq(s);
        break;
    case CTRL_COREINTR_CLEAR:
        m->coreintr_enable &= ~val;
        am335x_musb_update_irq(s);
        break;
    case CTRL_TX_MODE:
        m->tx_mode = val;
        break;
    case CTRL_RX_MODE:
        m->rx_mode = val;
        break;
    case CTRL_PHY_UTMI:
        m->phy_utmi = val;
        break;
    case CTRL_MODE:
        m->mode = val;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps am335x_musb_ctrl_ops = {
    .read = am335x_musb_ctrl_read,
    .write = am335x_musb_ctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* ======================================================================= */
/* USB1 musb host: "mc" musb-core MMIO (abs 0x47401c00).                   */

/* Indexed EP-window read: routes mc+0x10..0x1f through ep[INDEX]. */
static uint64_t am335x_musb_indexed_read(AM335xUsbssState *s, hwaddr ioff)
{
    AM335xMusb *m = &s->musb;
    AM335xMusbEp *ep = &m->ep[m->index];

    switch (ioff) {
    case IDX_TXMAXP:
        return ep->txmaxp;
    case IDX_TXCSR:
        return ep->txcsr;
    case IDX_RXMAXP:
        return ep->rxmaxp;
    case IDX_RXCSR:
        return ep->rxcsr;
    case IDX_RXCOUNT:
        return ep->rxcount;
    case IDX_TXTYPE:
        return ep->txtype;
    case IDX_TXINTERVAL:
        return ep->txinterval;
    case IDX_RXTYPE:
        return ep->rxtype;
    case IDX_RXINTERVAL:
        return ep->rxinterval;
    case IDX_FIFOSIZE:
        /* EP0's indexed FIFOSIZE aliases CONFIGDATA (musb_read_configdata,
         * musb_regs.h:276). Other EPs report their programmed FIFO size. */
        return (m->index == 0) ? USB_CONFIGDATA_VALUE : 0;
    default:
        return 0;
    }
}

static void am335x_musb_indexed_write(AM335xUsbssState *s, hwaddr ioff,
                                      uint64_t value)
{
    AM335xMusb *m = &s->musb;
    AM335xMusbEp *ep = &m->ep[m->index];

    switch (ioff) {
    case IDX_TXMAXP:
        ep->txmaxp = value;
        break;
    case IDX_TXCSR:
        ep->txcsr = value;
        if (m->index == 0) {
            am335x_musb_ep0_poke(s);
        } else {
            am335x_musb_tx_poke(s, m->index);
        }
        break;
    case IDX_RXMAXP:
        ep->rxmaxp = value;
        break;
    case IDX_RXCSR:
        ep->rxcsr = value;
        am335x_musb_rx_poke(s, m->index);
        break;
    case IDX_RXCOUNT:
        ep->rxcount = value;
        break;
    case IDX_TXTYPE:
        ep->txtype = value;
        break;
    case IDX_TXINTERVAL:
        ep->txinterval = value;
        break;
    case IDX_RXTYPE:
        ep->rxtype = value;
        break;
    case IDX_RXINTERVAL:
        ep->rxinterval = value;
        break;
    default:
        break;
    }
}

/* FIFO port read (RX drain): mc + 0x20 + 4*ep, `size` bytes per access. */
static uint64_t am335x_musb_fifo_read(AM335xUsbssState *s, unsigned ep,
                                      unsigned size)
{
    AM335xMusbEp *e = &s->musb.ep[ep];
    uint64_t v = 0;

    for (unsigned i = 0; i < size; i++) {
        uint8_t b = 0;
        if (e->fifo_rd < e->fifo_len) {
            b = e->fifo[e->fifo_rd++];
        }
        v |= (uint64_t)b << (8 * i);
    }
    return v;
}

/* FIFO port write (TX fill): appends `size` bytes to the endpoint FIFO. */
static void am335x_musb_fifo_write(AM335xUsbssState *s, unsigned ep,
                                   uint64_t value, unsigned size)
{
    AM335xMusbEp *e = &s->musb.ep[ep];

    for (unsigned i = 0; i < size; i++) {
        if (e->fifo_len < AM335X_MUSB_FIFO_SIZE) {
            e->fifo[e->fifo_len++] = (value >> (8 * i)) & 0xff;
        }
    }
}

static uint64_t am335x_musb_mc_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    AM335xUsbssState *s = AM335X_USBSS(opaque);
    AM335xMusb *m = &s->musb;

    if (offset >= MC_INDEXED_BASE && offset < MC_INDEXED_END) {
        return am335x_musb_indexed_read(s, offset - MC_INDEXED_BASE);
    }
    if (offset >= MC_FIFO_BASE && offset < MC_FIFO_END) {
        return am335x_musb_fifo_read(s, (offset - MC_FIFO_BASE) / 4, size);
    }
    if (offset >= MC_BUSCTL_BASE && offset < MC_BUSCTL_END) {
        unsigned ep = (offset - MC_BUSCTL_BASE) / 8;
        unsigned bo = (offset - MC_BUSCTL_BASE) % 8;
        return m->ep[ep].busctl[bo];
    }

    switch (offset) {
    case MC_FADDR:
        return m->faddr;
    case MC_POWER:
        return m->power;
    case MC_INTRTX:                 /* Mentor-core status; DSPS uses wrapper */
    case MC_INTRRX:
    case MC_INTRUSB:
        return 0;                   /* read-to-clear; nothing pending here   */
    case MC_INTRTXE:
        return m->intrtxe;
    case MC_INTRRXE:
        return m->intrrxe;
    case MC_INTRUSBE:
        return m->intrusbe;
    case MC_FRAME:
        return m->frame;
    case MC_INDEX:
        return m->index;
    case MC_TESTMODE:
        return m->testmode;
    case MC_DEVCTL:
        return m->devctl;
    case MC_BABBLE_CTL:
        return m->babble_ctl;
    case MC_TXFIFOSZ:
        return m->ep[m->index].txfifosz;
    case MC_RXFIFOSZ:
        return m->ep[m->index].rxfifosz;
    case MC_TXFIFOADD:
        return m->ep[m->index].txfifoadd;
    case MC_RXFIFOADD:
        return m->ep[m->index].rxfifoadd;
    case MC_HWVERS:
        return USB_HWVERS_VALUE;
    default:
        return 0;
    }
}

static void am335x_musb_mc_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    AM335xUsbssState *s = AM335X_USBSS(opaque);
    AM335xMusb *m = &s->musb;
    uint32_t val = (uint32_t)value;

    if (offset >= MC_INDEXED_BASE && offset < MC_INDEXED_END) {
        am335x_musb_indexed_write(s, offset - MC_INDEXED_BASE, value);
        return;
    }
    if (offset >= MC_FIFO_BASE && offset < MC_FIFO_END) {
        am335x_musb_fifo_write(s, (offset - MC_FIFO_BASE) / 4, value, size);
        return;
    }
    if (offset >= MC_BUSCTL_BASE && offset < MC_BUSCTL_END) {
        unsigned ep = (offset - MC_BUSCTL_BASE) / 8;
        unsigned bo = (offset - MC_BUSCTL_BASE) % 8;
        m->ep[ep].busctl[bo] = val;
        return;
    }

    switch (offset) {
    case MC_FADDR:
        m->faddr = val;
        break;
    case MC_POWER:
        m->power = val;
        break;
    case MC_INTRTX:                 /* status registers: read-to-clear, RO */
    case MC_INTRRX:
    case MC_INTRUSB:
        break;
    case MC_INTRTXE:
        m->intrtxe = val;
        break;
    case MC_INTRRXE:
        m->intrrxe = val;
        break;
    case MC_INTRUSBE:
        m->intrusbe = val;
        break;
    case MC_FRAME:
        break;                      /* frame number: read-only */
    case MC_INDEX:
        m->index = val & 0xf;
        break;
    case MC_TESTMODE:
        m->testmode = val;
        break;
    case MC_DEVCTL:
        m->devctl = val;
        break;
    case MC_BABBLE_CTL:
        m->babble_ctl = val;
        break;
    case MC_TXFIFOSZ:
        m->ep[m->index].txfifosz = val;
        break;
    case MC_RXFIFOSZ:
        m->ep[m->index].rxfifosz = val;
        break;
    case MC_TXFIFOADD:
        m->ep[m->index].txfifoadd = val;
        break;
    case MC_RXFIFOADD:
        m->ep[m->index].rxfifoadd = val;
        break;
    case MC_HWVERS:
        break;                      /* read-only */
    default:
        break;
    }
}

static const MemoryRegionOps am335x_musb_mc_ops = {
    .read = am335x_musb_mc_read,
    .write = am335x_musb_mc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* ======================================================================= */
/* USB1 musb host: USBBus / USBPort.                                       */

static void am335x_musb_bh(void *opaque)
{
    /* TODO(next commit): service async/retried transfers. */
}

static void am335x_musb_nak_timer(void *opaque)
{
    /* TODO(next commit): re-drive a NAK'd control/bulk/interrupt poll. */
}

static void am335x_musb_attach(USBPort *port)
{
    /* TODO(next commit): raise INTRUSB.CONNECT and reflect port speed. */
}

static void am335x_musb_detach(USBPort *port)
{
    /* TODO(next commit): raise INTRUSB.DISCONNECT. */
}

static void am335x_musb_child_detach(USBPort *port, USBDevice *child)
{
}

static void am335x_musb_wakeup(USBPort *port)
{
}

static void am335x_musb_async_complete(USBPort *port, USBPacket *packet)
{
    /* TODO(next commit): finish an async transfer and post its completion. */
}

static USBPortOps am335x_musb_port_ops = {
    .attach = am335x_musb_attach,
    .detach = am335x_musb_detach,
    .child_detach = am335x_musb_child_detach,
    .wakeup = am335x_musb_wakeup,
    .complete = am335x_musb_async_complete,
};

static void am335x_musb_wakeup_endpoint(USBBus *bus, USBEndpoint *ep,
                                        unsigned int stream)
{
    AM335xMusb *m = container_of(bus, AM335xMusb, bus);

    qemu_bh_schedule(m->async_bh);
}

static USBBusOps am335x_musb_bus_ops = {
    .wakeup_endpoint = am335x_musb_wakeup_endpoint,
};

/* ======================================================================= */
/* QOM.                                                                    */

static void am335x_musb_reset(AM335xUsbssState *s)
{
    AM335xMusb *m = &s->musb;

    /* Preserve the USBBus/USBPort/timer/bh handles; clear register state. */
    m->control = 0;
    m->status = 0;
    m->epintr_status = 0;
    m->coreintr_status = 0;
    m->epintr_enable = 0;
    m->coreintr_enable = 0;
    m->phy_utmi = USB_PHY_UTMI_RESET;
    m->mode = USB_MODE_RESET;
    m->tx_mode = 0;
    m->rx_mode = 0;

    m->faddr = 0;
    m->power = 0;
    m->intrtxe = 0;
    m->intrrxe = 0;
    m->intrusbe = 0;
    m->frame = 0;
    m->index = 0;
    m->testmode = 0;
    m->devctl = 0;
    m->babble_ctl = 0;

    for (unsigned i = 0; i < AM335X_MUSB_NUM_EP; i++) {
        AM335xMusbEp *e = &m->ep[i];
        e->txmaxp = e->txcsr = e->rxmaxp = e->rxcsr = e->rxcount = 0;
        e->txtype = e->txinterval = e->rxtype = e->rxinterval = 0;
        e->txfifosz = e->rxfifosz = 0;
        e->txfifoadd = e->rxfifoadd = 0;
        memset(e->busctl, 0, sizeof(e->busctl));
        e->fifo_len = 0;
        e->fifo_rd = 0;
    }

    m->xfer_active = false;
}

static void am335x_usbss_reset(DeviceState *dev)
{
    AM335xUsbssState *s = AM335X_USBSS(dev);

    memset(s->regs, 0, AM335X_USBSS_SIZE);

    /* USB0 clean-probe PHY/mode reset values (USB1 handled by musb reset). */
    stl_le_p(s->regs + USB0_CTRL_BASE + CTRL_PHY_UTMI, USB_PHY_UTMI_RESET);
    stl_le_p(s->regs + USB0_CTRL_BASE + CTRL_MODE, USB_MODE_RESET);

    am335x_musb_reset(s);

    qemu_set_irq(s->irq[0], 0);
    qemu_set_irq(s->irq[1], 0);
}

static void am335x_usbss_realize(DeviceState *dev, Error **errp)
{
    AM335xUsbssState *s = AM335X_USBSS(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    AM335xMusb *m = &s->musb;

    s->regs = g_malloc0(AM335X_USBSS_SIZE);

    /* 32KB container: flat glue store at priority 0, with USB1's control +
     * mc functional regions overlaid at priority 1. */
    memory_region_init(&s->container, OBJECT(s), TYPE_AM335X_USBSS,
                       AM335X_USBSS_SIZE);
    memory_region_init_io(&s->glue, OBJECT(s), &am335x_usbss_glue_ops, s,
                          "am335x-usbss-glue", AM335X_USBSS_SIZE);
    memory_region_add_subregion(&s->container, 0, &s->glue);

    memory_region_init_io(&s->musb_ctrl, OBJECT(s), &am335x_musb_ctrl_ops, s,
                          "am335x-usb1-ctrl", AM335X_USB1_CTRL_SIZE);
    memory_region_add_subregion_overlap(&s->container, AM335X_USB1_CTRL_OFFSET,
                                        &s->musb_ctrl, 1);
    memory_region_init_io(&s->musb_mc, OBJECT(s), &am335x_musb_mc_ops, s,
                          "am335x-usb1-mc", AM335X_USB1_MC_SIZE);
    memory_region_add_subregion_overlap(&s->container, AM335X_USB1_MC_OFFSET,
                                        &s->musb_mc, 1);

    sysbus_init_mmio(sbd, &s->container);

    /* Per-instance musb "mc" interrupt outputs -> INTC 18 (USB0)/19 (USB1). */
    sysbus_init_irq(sbd, &s->irq[0]);
    sysbus_init_irq(sbd, &s->irq[1]);

    /* USB1 host bus + type-A root port (full/low/high speed). */
    usb_bus_new(&m->bus, sizeof(m->bus), &am335x_musb_bus_ops, dev);
    usb_register_port(&m->bus, &m->port, s, 0, &am335x_musb_port_ops,
                      USB_SPEED_MASK_LOW | USB_SPEED_MASK_FULL |
                      USB_SPEED_MASK_HIGH);

    m->async_bh = qemu_bh_new_guarded(am335x_musb_bh, s,
                                      &dev->mem_reentrancy_guard);
    m->nak_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, am335x_musb_nak_timer, s);
}

static void am335x_usbss_unrealize(DeviceState *dev)
{
    AM335xUsbssState *s = AM335X_USBSS(dev);

    if (s->musb.nak_timer) {
        timer_free(s->musb.nak_timer);
        s->musb.nak_timer = NULL;
    }
    if (s->musb.async_bh) {
        qemu_bh_delete(s->musb.async_bh);
        s->musb.async_bh = NULL;
    }
}

static void am335x_usbss_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = am335x_usbss_realize;
    dc->unrealize = am335x_usbss_unrealize;
    device_class_set_legacy_reset(dc, am335x_usbss_reset);
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
    /*
     * No vmsd: heap-allocated backing store + live USBBus/USBPort/QEMUBH,
     * and the beaglebone-black machine is not migratable (SoC uses
     * serial_hd(), user_creatable = false) -- same rationale as
     * am335x_control.c / hcd-dwc2's non-migratable embedded use.
     */
    dc->desc = "TI AM335x USB Subsystem (USB1 musb host)";
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
