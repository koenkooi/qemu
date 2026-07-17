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
 * CSR/FIFO register file, the DSPS wrapper interrupt plumbing, PIO control +
 * bulk/interrupt transfer handling, AND a functional CPPI4.1 DMA data path
 * so the guest's *default* use_dma=true works: devices enumerate and move
 * bulk/interrupt data with no `musb_hdrc.use_dma=0` override. USB0 (the OTG
 * port) stays a clean-probe register-file stub (an idle peripheral/host that
 * never sees a connect), and its 15+15 CPPI channels stay inert.
 *
 * Layout: a 32KB container holds a flat "glue" store (priority 0) covering
 * the whole window, with the USB1 "control" (0x1800) and "mc" (0x1c00)
 * sub-windows and the CPPI4.1 queue-manager (0x4000) overlaid as higher-
 * priority functional MMIO regions. USB0's clean-probe revision/CONFIGDATA,
 * the ti-sysc/USB0 soft-reset bits, and the CPPI controller/scheduler
 * probe-time registers are served by the flat glue store as before.
 *
 * The CPPI4.1 DMA data path (drivers/dma/ti/cppi41.c + musb_cppi41.c)
 * ----------------------------------------------------------------
 * With use_dma=true, musb_ep_program() routes every EP1-15 bulk/interrupt
 * transfer through CPPI and bypasses the PIO FIFO. The provider builds an
 * 8-word host descriptor in guest DMA memory and pushes its physical address
 * onto the endpoint's submit queue (a QMGR_QUEUE_D write); this model decodes
 * it, moves the buffer through the USB1 endpoint with the same
 * usb_handle_packet() plumbing as the PIO engine, writes the transferred
 * length back into the descriptor, and posts it onto the endpoint's
 * completion queue -- raising USBSS_IRQ_STATUS.PD_COMP (INTC 17). See the
 * "CPPI4.1 DMA" section below.
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
#include "system/dma.h"
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

/* musb register bit fields (musb_regs.h). */
#define MUSB_POWER_HSENAB      0x20
#define MUSB_POWER_HSMODE      0x10
#define MUSB_POWER_RESET       0x08

#define MUSB_INTR_RESET        0x04
#define MUSB_INTR_CONNECT      0x10
#define MUSB_INTR_DISCONNECT   0x20

#define MUSB_DEVCTL_BDEVICE    0x80
#define MUSB_DEVCTL_FSDEV      0x40
#define MUSB_DEVCTL_LSDEV      0x20
#define MUSB_DEVCTL_VBUS       0x18   /* VBUS-valid field (3 << 3)          */
#define MUSB_DEVCTL_HM         0x04
#define MUSB_DEVCTL_SESSION    0x01

/* CSR0 (EP0), host mode. */
#define MUSB_CSR0_H_STATUSPKT  0x0040
#define MUSB_CSR0_H_REQPKT     0x0020
#define MUSB_CSR0_H_ERROR      0x0010
#define MUSB_CSR0_H_SETUPPKT   0x0008
#define MUSB_CSR0_H_RXSTALL    0x0004
#define MUSB_CSR0_TXPKTRDY     0x0002
#define MUSB_CSR0_RXPKTRDY     0x0001

/* TXCSR / RXCSR, host mode. */
#define MUSB_TXCSR_MODE        0x2000
#define MUSB_TXCSR_DMAENAB     0x1000
#define MUSB_TXCSR_H_RXSTALL   0x0020
#define MUSB_TXCSR_H_ERROR     0x0004
#define MUSB_TXCSR_TXPKTRDY    0x0001

#define MUSB_RXCSR_DMAENAB     0x2000
#define MUSB_RXCSR_H_RXSTALL   0x0040
#define MUSB_RXCSR_H_REQPKT    0x0020
#define MUSB_RXCSR_DATAERROR   0x0008
#define MUSB_RXCSR_H_ERROR     0x0004
#define MUSB_RXCSR_FIFOFULL    0x0002
#define MUSB_RXCSR_RXPKTRDY    0x0001

/*
 * "Write zero to clear" status bits: the guest read-modify-writes the CSR
 * with these bits set to *preserve* them, and clear to reset them; the
 * model owns setting them (musb_regs.h MUSB_*_H_WZC_BITS). New value =
 * (written & ~WZC) | (old & written & WZC).
 */
#define MUSB_CSR0_H_WZC_BITS   (0x0080 | MUSB_CSR0_H_RXSTALL | MUSB_CSR0_RXPKTRDY)
#define MUSB_TXCSR_FIFONOTEMPTY 0x0002
#define MUSB_TXCSR_H_WZC_BITS  (0x0080 | MUSB_TXCSR_H_RXSTALL | \
                                MUSB_TXCSR_H_ERROR | MUSB_TXCSR_FIFONOTEMPTY)
#define MUSB_RXCSR_H_WZC_BITS  (MUSB_RXCSR_H_RXSTALL | MUSB_RXCSR_DATAERROR | \
                                MUSB_RXCSR_H_ERROR | MUSB_RXCSR_RXPKTRDY)

/* busctl per-EP offsets (musb_regs.h). */
#define MUSB_TXFUNCADDR        0x00
#define MUSB_RXFUNCADDR        0x04

/* TXTYPE/RXTYPE fields (musb_regs.h): protocol (bits 5:4) + remote endpoint. */
#define MUSB_TYPE_PROTO        0x30
#define MUSB_TYPE_PROTO_SHIFT  4
#define MUSB_TYPE_REMOTE_END   0x0f

/*
 * NAK re-poll cadence for endpoints that return no data yet. Retries always
 * go through a QEMU_CLOCK_VIRTUAL timer (never a bottom half), so guest time
 * advances between polls -- a self-rescheduling bottom half would freeze the
 * virtual clock. 8ms keeps interrupt/bulk endpoints responsive without a
 * poll storm; an endpoint wakeup re-polls sooner (125us).
 */
#define AM335X_MUSB_NAK_RETRY_NS   8000000  /* 8ms  */
#define AM335X_MUSB_WAKE_RETRY_NS   125000  /* 125us */

/* ======================================================================= */
/* CPPI4.1 DMA (drivers/dma/ti/cppi41.c + drivers/usb/musb/musb_cppi41.c).  */
/*
 * The USBSS window carries the CPPI4.1 glue/controller/scheduler/queue-
 * manager sub-blocks. Probe writes to all four are RAM-backed by the flat
 * glue store (nothing in the driver reads them back), but two windows are
 * overlaid with functional MMIO so the data path works with the guest's
 * default use_dma=true: the queue-manager (submit push -> transfer ->
 * completion pop) and the controller (channel teardown).
 */

/* Sub-window container offsets (SoC-abs = 0x47400000 + these). */
#define CPPI_CTRL_OFFSET   0x2000
#define CPPI_CTRL_SIZE     0x1000
#define CPPI_QMGR_OFFSET   0x4000
#define CPPI_QMGR_SIZE     0x4000

/* Glue interrupt aggregator (musb_dsps.c:159-163). */
#define USBSS_IRQ_STATUS   0x28
#define USBSS_IRQ_ENABLER  0x2c
#define USBSS_IRQ_CLEARR   0x30
#define USBSS_IRQ_PD_COMP  (1u << 2)

/* Controller window offsets, relative to CPPI_CTRL_OFFSET (cppi41.c:31-38). */
#define CPPI_DMA_TXGCR(x)  (0x800 + (x) * 0x20)
#define CPPI_DMA_RXGCR(x)  (0x808 + (x) * 0x20)
#define CPPI_GCR_TEARDOWN  (1u << 30)

/* Queue-manager window offsets, relative to CPPI_QMGR_OFFSET (cppi41.c:72-80).
 * QUEUE_D(n) is push (submit, write) and pop (complete, read). */
#define CPPI_QMGR_PEND(i)     (0x90 + (i) * 4)
#define CPPI_QMGR_PEND_SLOTS  5           /* qmgr_num_pend (cppi41.c:1000) */
#define CPPI_QMGR_QUEUE_BASE  0x2000
#define CPPI_QMGR_QUEUE_D_OFF 0xc         /* QUEUE_D within a 0x10 stride */
#define CPPI_QMGR_QUEUE_END   (0x2000 + AM335X_CPPI_NUM_QUEUES * 0x10)

/* Host packet descriptor (cppi41.c:107-116, built at cppi41.c:513-570). */
#define CPPI_DESC_BYTES       32          /* 8 x u32, 32-byte aligned      */
#define CPPI_PD0_LEN_MASK     0x003fffffu /* pd_trans_len(): (1<<22)-1     */
#define CPPI_PD_DESC_ALIGN     0x1fu      /* QUEUE_D low bits (cppi41.c:298) */
#define CPPI_DESC_TYPE_TEARD  0x13        /* pd0 >> 27 for a teardown desc */

/*
 * USB1 submit-queue numbers (cppi41.c:155-225). The dmaengine port_num
 * 15..29 == USB1 EP1..15; USB0 (port 0..14) submit queues stay inert.
 *   TX submit: q = 62,64,..,90 (even)  -> EPn = (q-62)/2 + 1
 *   RX submit: q = 16,17,..,30         -> EPn =  q-16  + 1
 * Completion queues:  TX EPn -> 124+n,  RX EPn -> 140+n.
 * Teardown queue (cppi41.c:998): submit=31, complete=0.
 */
#define CPPI_USB1_TX_SUBMIT_LO 62
#define CPPI_USB1_TX_SUBMIT_HI 90
#define CPPI_USB1_RX_SUBMIT_LO 16
#define CPPI_USB1_RX_SUBMIT_HI 30
#define CPPI_TD_SUBMIT_Q       31
#define CPPI_TD_COMPLETE_Q     0

static void am335x_cppi_update_irq(AM335xUsbssState *s);

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

    /* USBSS interrupt aggregator (musb_dsps.c): PD_COMP is the only bit the
     * driver touches. Reading _ENABLER/_CLEARR returns the live enable. */
    if (offset == USBSS_IRQ_STATUS) {
        return s->cppi.irq_status;
    }
    if (offset == USBSS_IRQ_ENABLER || offset == USBSS_IRQ_CLEARR) {
        return s->cppi.irq_enable;
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

    /* USBSS interrupt aggregator: STATUS is write-1-to-clear, ENABLER sets
     * and CLEARR clears the enable mask (musb_dsps.c:647-681). */
    if (offset == USBSS_IRQ_STATUS) {
        s->cppi.irq_status &= ~(uint32_t)value;
        am335x_cppi_update_irq(s);
        return;
    }
    if (offset == USBSS_IRQ_ENABLER) {
        s->cppi.irq_enable |= (uint32_t)value;
        am335x_cppi_update_irq(s);
        return;
    }
    if (offset == USBSS_IRQ_CLEARR) {
        s->cppi.irq_enable &= ~(uint32_t)value;
        am335x_cppi_update_irq(s);
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

/* Post a TX-endpoint (or EP0/control) completion into epintr_status. */
static void am335x_musb_raise_tx(AM335xUsbssState *s, unsigned ep)
{
    s->musb.epintr_status |= (1u << ep) & MUSB_EPINTR_TX_MASK;
    am335x_musb_update_irq(s);
}

/* Post an RX-endpoint completion into epintr_status. */
static void am335x_musb_raise_rx(AM335xUsbssState *s, unsigned ep)
{
    s->musb.epintr_status |= (1u << (MUSB_EPINTR_RX_SHIFT + ep)) &
                             MUSB_EPINTR_RX_MASK;
    am335x_musb_update_irq(s);
}

/* Post a USB-core (INTRUSB) event into coreintr_status. */
static void am335x_musb_raise_core(AM335xUsbssState *s, uint32_t intrusb_bits)
{
    s->musb.coreintr_status |= intrusb_bits & MUSB_COREINTR_USB_MASK;
    am335x_musb_update_irq(s);
}

/* ======================================================================= */
/* USB1 musb host: connect detection.                                      */

/* The single device (if any) attached to the port, addressed by `addr`. */
static USBDevice *am335x_musb_find_dev(AM335xUsbssState *s, uint8_t addr)
{
    if (!s->musb.port.dev || !s->musb.port.dev->attached) {
        return NULL;
    }
    return usb_find_device(&s->musb.port, addr);
}

/* EP0 max packet, used to chunk control-IN reads one packet at a time. */
static unsigned am335x_musb_ep0_maxp(USBDevice *dev)
{
    unsigned mp = dev->ep_ctl.max_packet_size;

    return mp ? mp : 64;
}

/*
 * Signal a host-mode connect once a device is attached AND the guest has
 * started a session (DEVCTL.SESSION). The device is coldplugged before the
 * driver runs, so the connect is deferred to musb_start()'s DEVCTL.SESSION
 * write; the CONNECT status bit latches in coreintr_status until the guest
 * enables + acks it. musb_handle_intr_connect() (musb_core.c:885-936)
 * requires DEVCTL to read back VBUS-valid (0x18), host-mode, and the
 * device's speed (LSDEV set only for low speed).
 */
static void am335x_musb_eval_connect(AM335xUsbssState *s)
{
    AM335xMusb *m = &s->musb;
    USBDevice *dev = m->port.dev;

    if (!dev || !dev->attached || m->connected ||
        !(m->devctl & MUSB_DEVCTL_SESSION)) {
        return;
    }
    m->connected = true;

    m->devctl &= ~(MUSB_DEVCTL_BDEVICE | MUSB_DEVCTL_FSDEV | MUSB_DEVCTL_LSDEV);
    m->devctl |= MUSB_DEVCTL_HM | MUSB_DEVCTL_VBUS;
    if (dev->speed == USB_SPEED_LOW) {
        m->devctl |= MUSB_DEVCTL_LSDEV;
    } else {
        m->devctl |= MUSB_DEVCTL_FSDEV;
    }
    am335x_musb_raise_core(s, MUSB_INTR_CONNECT);
}

/* ======================================================================= */
/* USB1 musb host: PIO transfer engine.                                    */
/*
 * The guest drives one PIO transfer at a time per the poll-based musb host
 * driver, so a single in-flight USBPacket (m->packet) suffices. A CSR "go"
 * bit (SETUPPKT/REQPKT/STATUSPKT/TXPKTRDY) invokes the poke hook for its
 * endpoint, which issues a USBPacket; the result -- synchronous, async, or
 * NAK-retry -- is turned back into CSR/FIFO/COUNT register state plus the
 * matching wrapper interrupt (epintr_status TX bit n / RX bit 16+n).
 */

/* xfer_kind: which phase/direction the in-flight m->packet represents. */
enum {
    XFER_EP0_SETUP,
    XFER_EP0_IN,
    XFER_EP0_OUT,
    XFER_EP0_STATUS_IN,
    XFER_EP0_STATUS_OUT,
    XFER_TX,            /* bulk/interrupt OUT on ep >= 1 (PIO)  */
    XFER_RX,            /* bulk/interrupt IN  on ep >= 1 (PIO)  */
    XFER_CPPI_TX,       /* bulk/interrupt OUT on ep >= 1 (CPPI) */
    XFER_CPPI_RX,       /* bulk/interrupt IN  on ep >= 1 (CPPI) */
};

static void am335x_musb_issue(AM335xUsbssState *s, unsigned ep, bool is_rx);
static void am335x_musb_arm(AM335xUsbssState *s);
static void am335x_cppi_finish(AM335xUsbssState *s, unsigned ep, bool is_rx);

/*
 * How long before the `is_rx` half of endpoint `ep` may be (re)issued.
 * Interrupt endpoints are paced to their polling interval (TX/RXINTERVAL, in
 * frames == ms at full speed) so a still-pending device (e.g. a hub reporting
 * an un-serviced port change) cannot be re-polled in a tight loop that
 * starves the guest's hub thread; a 4ms floor guards against a driver
 * programming interval 0/1. Control (EP0) and bulk use the NAK retry cadence.
 */
static int64_t am335x_musb_poll_ns(AM335xUsbssState *s, unsigned ep, bool is_rx)
{
    AM335xMusbEp *e = &s->musb.ep[ep];
    unsigned type = is_rx ? e->rxtype : e->txtype;
    unsigned intv = is_rx ? e->rxinterval : e->txinterval;

    if (ep == 0) {
        return AM335X_MUSB_NAK_RETRY_NS;
    }
    if (((type & MUSB_TYPE_PROTO) >> MUSB_TYPE_PROTO_SHIFT) ==
        USB_ENDPOINT_XFER_INT) {
        unsigned ms = intv ? intv : 1;
        return (int64_t)(ms < 4 ? 4 : ms) * 1000000;
    }
    return AM335X_MUSB_NAK_RETRY_NS;
}

/* Post an error completion on the `is_rx` half of `ep` (device absent). */
static void am335x_musb_fail_nodev(AM335xUsbssState *s, unsigned ep, bool is_rx)
{
    AM335xMusbEp *e = &s->musb.ep[ep];

    (is_rx ? &e->rx : &e->tx)->active = false;

    if (ep == 0) {
        e->txcsr &= ~(MUSB_CSR0_TXPKTRDY | MUSB_CSR0_H_SETUPPKT |
                      MUSB_CSR0_H_REQPKT | MUSB_CSR0_H_STATUSPKT);
        e->txcsr |= MUSB_CSR0_H_ERROR;
        am335x_musb_raise_tx(s, 0);
    } else if (!is_rx) {
        e->txcsr &= ~MUSB_TXCSR_TXPKTRDY;
        e->txcsr |= MUSB_TXCSR_H_ERROR;
        am335x_musb_raise_tx(s, ep);
    } else {
        e->rxcsr &= ~MUSB_RXCSR_H_REQPKT;
        e->rxcsr |= MUSB_RXCSR_H_ERROR;
        am335x_musb_raise_rx(s, ep);
    }
}

/* Turn the `is_rx` half of `ep`'s finished packet into CSR/FIFO/IRQ state. */
static void am335x_musb_finish(AM335xUsbssState *s, unsigned ep, bool is_rx)
{
    AM335xMusbEp *e = &s->musb.ep[ep];
    AM335xMusbHalf *h = is_rx ? &e->rx : &e->tx;
    USBPacket *p = &h->packet;
    int status = p->status;
    int actual = p->actual_length;

    if (status == USB_RET_NAK) {
        /* Device has nothing to give/take yet: re-poll at the endpoint's
         * pace (interrupt interval / generic NAK cadence). Stays active.
         * A CPPI transfer re-polls the same way -- on real silicon the
         * STARV_RETRY/AUTOREQ hardware keeps retrying until data moves. */
        usb_packet_cleanup(p);
        h->next_poll = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                       am335x_musb_poll_ns(s, ep, is_rx);
        am335x_musb_arm(s);
        return;
    }

    if (h->cppi) {
        am335x_cppi_finish(s, ep, is_rx);
        return;
    }

    if (ep == 0) {
        uint16_t csr = e->txcsr;

        csr &= ~(MUSB_CSR0_TXPKTRDY | MUSB_CSR0_H_SETUPPKT |
                 MUSB_CSR0_H_REQPKT | MUSB_CSR0_H_STATUSPKT);
        if (status == USB_RET_STALL) {
            csr |= MUSB_CSR0_H_RXSTALL;
            e->rxcount = e->rx.fifo_len = e->rx.fifo_rd = 0;
        } else if (status < 0) {
            csr |= MUSB_CSR0_H_ERROR;
            e->rxcount = e->rx.fifo_len = e->rx.fifo_rd = 0;
        } else if (h->kind == XFER_EP0_IN) {
            e->rx.fifo_len = actual;    /* IN data goes to the RX FIFO */
            e->rx.fifo_rd = 0;
            e->rxcount = actual;
            csr |= MUSB_CSR0_RXPKTRDY;
        } else {
            e->tx.fifo_len = e->tx.fifo_rd = 0;   /* SETUP/OUT/STATUS consumed */
        }
        e->txcsr = csr;
        am335x_musb_raise_tx(s, 0);
    } else if (!is_rx) {                     /* bulk/interrupt OUT */
        e->txcsr &= ~MUSB_TXCSR_TXPKTRDY;
        e->tx.fifo_len = e->tx.fifo_rd = 0;
        if (status == USB_RET_STALL) {
            e->txcsr |= MUSB_TXCSR_H_RXSTALL;
        } else if (status < 0) {
            e->txcsr |= MUSB_TXCSR_H_ERROR;
        }
        am335x_musb_raise_tx(s, ep);
    } else {                                 /* bulk/interrupt IN */
        e->rxcsr &= ~MUSB_RXCSR_H_REQPKT;
        if (status == USB_RET_STALL) {
            e->rxcsr |= MUSB_RXCSR_H_RXSTALL;
        } else if (status < 0) {
            e->rxcsr |= MUSB_RXCSR_H_ERROR;
        } else {
            e->rx.fifo_len = actual;
            e->rx.fifo_rd = 0;
            e->rxcount = actual;
            e->rxcsr |= MUSB_RXCSR_RXPKTRDY;
        }
        am335x_musb_raise_rx(s, ep);
    }

    usb_packet_cleanup(p);
    h->active = false;
}

/*
 * Build + submit the `is_rx` half of endpoint `ep`. OUT-direction data is
 * sourced from the TX FIFO (where guest writes land); IN-direction data is
 * delivered into the RX FIFO (where guest reads drain). The device address
 * comes from the endpoint's busctl block and the target device endpoint from
 * TXTYPE/RXTYPE (host multipoint routing); EP0 uses busctl slot 0.
 */
static void am335x_musb_issue(AM335xUsbssState *s, unsigned ep, bool is_rx)
{
    AM335xMusb *m = &s->musb;
    AM335xMusbEp *e = &m->ep[ep];
    AM335xMusbHalf *h = is_rx ? &e->rx : &e->tx;
    USBPacket *p = &h->packet;
    USBDevice *dev;
    USBEndpoint *uep;
    uint8_t *buf = NULL;
    unsigned devaddr, target_ep, len = 0;
    int pid;

    switch (h->kind) {
    case XFER_EP0_SETUP:
        pid = USB_TOKEN_SETUP;
        devaddr = m->ep[0].busctl[MUSB_TXFUNCADDR];
        target_ep = 0;
        buf = e->tx.fifo;
        len = 8;
        break;
    case XFER_EP0_IN:
    case XFER_EP0_STATUS_IN:
        pid = USB_TOKEN_IN;
        devaddr = m->ep[0].busctl[MUSB_TXFUNCADDR];
        target_ep = 0;
        buf = e->rx.fifo;
        break;
    case XFER_EP0_OUT:
    case XFER_EP0_STATUS_OUT:
        pid = USB_TOKEN_OUT;
        devaddr = m->ep[0].busctl[MUSB_TXFUNCADDR];
        target_ep = 0;
        buf = e->tx.fifo;
        len = (h->kind == XFER_EP0_OUT) ? e->tx.fifo_len : 0;
        break;
    case XFER_TX:
        pid = USB_TOKEN_OUT;
        devaddr = e->busctl[MUSB_TXFUNCADDR];
        target_ep = e->txtype & MUSB_TYPE_REMOTE_END;
        buf = e->tx.fifo;
        len = e->tx.fifo_len;
        break;
    case XFER_CPPI_TX:
        /* CPPI bulk/interrupt OUT: same endpoint routing as PIO XFER_TX,
         * but the data was DMA'd from the guest buffer into cppi_buf. */
        pid = USB_TOKEN_OUT;
        devaddr = e->busctl[MUSB_TXFUNCADDR];
        target_ep = e->txtype & MUSB_TYPE_REMOTE_END;
        buf = h->cppi_buf;
        len = h->cppi_req_len;
        break;
    case XFER_CPPI_RX:
        /* CPPI bulk/interrupt IN: routing as PIO XFER_RX; received data
         * lands in cppi_buf and is DMA'd to the guest buffer on finish. */
        pid = USB_TOKEN_IN;
        devaddr = e->busctl[MUSB_RXFUNCADDR];
        target_ep = e->rxtype & MUSB_TYPE_REMOTE_END;
        buf = h->cppi_buf;
        len = h->cppi_req_len;
        break;
    case XFER_RX:
    default:
        pid = USB_TOKEN_IN;
        devaddr = e->busctl[MUSB_RXFUNCADDR];
        target_ep = e->rxtype & MUSB_TYPE_REMOTE_END;
        buf = e->rx.fifo;
        break;
    }

    dev = am335x_musb_find_dev(s, devaddr);
    if (!dev) {
        am335x_musb_fail_nodev(s, ep, is_rx);
        return;
    }
    uep = usb_ep_get(dev, pid, target_ep);

    if (h->kind == XFER_EP0_IN) {
        len = am335x_musb_ep0_maxp(dev);
    } else if (h->kind == XFER_RX) {
        len = e->rxmaxp & 0x7ff;
    }

    usb_packet_init(p);
    /* short_not_ok is false for IN (a short packet legitimately ends the
     * transfer); true otherwise -- mirrors hcd-dwc2.c. */
    usb_packet_setup(p, pid, uep, 0, 0, pid != USB_TOKEN_IN, true);
    if (len) {
        usb_packet_addbuf(p, buf, len);
    }
    h->active = true;
    usb_handle_packet(dev, p);

    if (p->status == USB_RET_ASYNC) {
        return;                         /* completed via the port .complete op */
    }
    am335x_musb_finish(s, ep, is_rx);
}

/*
 * CSR0 write with a "go" bit -> drive the next EP0 control-transfer phase.
 * EP0 is half-duplex control, so all phases share the endpoint's TX-half
 * slot; the IN data phase still delivers into the RX FIFO (see finish()).
 */
static void am335x_musb_ep0_poke(AM335xUsbssState *s)
{
    AM335xMusbEp *e0 = &s->musb.ep[0];
    uint16_t csr = e0->txcsr;

    if (e0->tx.active) {
        return;
    }
    if (csr & MUSB_CSR0_H_SETUPPKT) {
        e0->tx.kind = XFER_EP0_SETUP;
        am335x_musb_issue(s, 0, false);
    } else if (csr & MUSB_CSR0_H_STATUSPKT) {
        e0->tx.kind = (csr & MUSB_CSR0_H_REQPKT) ? XFER_EP0_STATUS_IN
                                                 : XFER_EP0_STATUS_OUT;
        am335x_musb_issue(s, 0, false);
    } else if (csr & MUSB_CSR0_H_REQPKT) {
        e0->tx.kind = XFER_EP0_IN;
        am335x_musb_issue(s, 0, false);
    } else if (csr & MUSB_CSR0_TXPKTRDY) {
        e0->tx.kind = XFER_EP0_OUT;
        am335x_musb_issue(s, 0, false);
    }
}

/* TXCSR.TXPKTRDY write -> launch a bulk/interrupt OUT packet on EP `ep`. */
static void am335x_musb_tx_poke(AM335xUsbssState *s, unsigned ep)
{
    AM335xMusbEp *e = &s->musb.ep[ep];

    if (e->tx.active) {
        return;
    }
    /* When DMA is armed the transfer is driven by the CPPI submit-queue
     * push, not the FIFO; the driver never sets TXPKTRDY in that case, but
     * guard explicitly so a stray write cannot start a parallel PIO OUT. */
    if (e->txcsr & MUSB_TXCSR_DMAENAB) {
        return;
    }
    if (e->txcsr & MUSB_TXCSR_TXPKTRDY) {
        e->tx.kind = XFER_TX;
        am335x_musb_issue(s, ep, false);
    }
}

/*
 * RXCSR.H_REQPKT write -> request a bulk/interrupt IN packet on EP `ep`.
 *
 * When CPPI DMA is active (the guest created the dma_controller, so PD_COMP
 * is enabled -- i.e. the default use_dma=true), every IN transfer on EP1-15
 * is driven by a CPPI submit-queue push, never PIO: musb_ep_program() always
 * allocates a DMA channel for epnum>0. Worse, on each DMA completion
 * musb_host_rx() rewrites RXCSR twice -- first clearing H_REQPKT
 * (musb_host.c:1869-1873), then re-setting it while clearing DMAENAB
 * (musb_host.c:1879-1883) -- a genuine 0->1 edge that must NOT launch a stray
 * PIO IN token (it would steal a packet from the DMA endpoint and desync the
 * class driver). So suppress PIO IN on EP1-15 entirely while CPPI is active;
 * the well-exercised use_dma=0 PIO path (PD_COMP disabled) is unchanged.
 */
static void am335x_musb_rx_poke(AM335xUsbssState *s, unsigned ep)
{
    AM335xMusbEp *e = &s->musb.ep[ep];

    if (e->rx.active) {
        return;
    }
    if (ep >= 1 && (s->cppi.irq_enable & USBSS_IRQ_PD_COMP)) {
        return;
    }
    if (e->rxcsr & MUSB_RXCSR_H_REQPKT) {
        e->rx.kind = XFER_RX;
        /*
         * An interrupt IN is paced to its polling interval rather than
         * issued at once: the guest resubmits it as soon as it completes,
         * so issuing immediately would busy-loop against a device that
         * keeps returning the same data (e.g. a hub's port-change endpoint
         * before the hub thread clears the change). Bulk IN issues now.
         */
        if (((e->rxtype & MUSB_TYPE_PROTO) >> MUSB_TYPE_PROTO_SHIFT) ==
            USB_ENDPOINT_XFER_INT) {
            e->rx.active = true;
            e->rx.next_poll = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                              am335x_musb_poll_ns(s, ep, true);
            am335x_musb_arm(s);
        } else {
            am335x_musb_issue(s, ep, true);
        }
    }
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
        if (m->index == 0) {
            uint16_t wzc = MUSB_CSR0_H_WZC_BITS;
            ep->txcsr = ((uint16_t)value & ~wzc) |
                        (ep->txcsr & (uint16_t)value & wzc);
            am335x_musb_ep0_poke(s);
        } else {
            uint16_t wzc = MUSB_TXCSR_H_WZC_BITS;
            ep->txcsr = ((uint16_t)value & ~wzc) |
                        (ep->txcsr & (uint16_t)value & wzc);
            am335x_musb_tx_poke(s, m->index);
        }
        break;
    case IDX_RXMAXP:
        ep->rxmaxp = value;
        break;
    case IDX_RXCSR: {
        uint16_t wzc = MUSB_RXCSR_H_WZC_BITS;
        ep->rxcsr = ((uint16_t)value & ~wzc) |
                    (ep->rxcsr & (uint16_t)value & wzc);
        am335x_musb_rx_poke(s, m->index);
        break;
    }
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

/* FIFO port read (drains the RX FIFO): mc + 0x20 + 4*ep, `size` bytes. */
static uint64_t am335x_musb_fifo_read(AM335xUsbssState *s, unsigned ep,
                                      unsigned size)
{
    AM335xMusbHalf *h = &s->musb.ep[ep].rx;
    uint64_t v = 0;

    for (unsigned i = 0; i < size; i++) {
        uint8_t b = 0;
        if (h->fifo_rd < h->fifo_len) {
            b = h->fifo[h->fifo_rd++];
        }
        v |= (uint64_t)b << (8 * i);
    }
    return v;
}

/* FIFO port write (fills the TX FIFO): appends `size` bytes. */
static void am335x_musb_fifo_write(AM335xUsbssState *s, unsigned ep,
                                   uint64_t value, unsigned size)
{
    AM335xMusbHalf *h = &s->musb.ep[ep].tx;

    for (unsigned i = 0; i < size; i++) {
        if (h->fifo_len < AM335X_MUSB_FIFO_SIZE) {
            h->fifo[h->fifo_len++] = (value >> (8 * i)) & 0xff;
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
    case MC_POWER: {
        uint8_t old = m->power;

        m->power = val;
        /*
         * Port reset: the guest asserts POWER.RESET, waits (delayed work),
         * then deasserts it (musb_port_reset, musb_virthub.c:167-186). On
         * the deassert edge, reset the attached device and present
         * POWER.HSMODE for its speed -- the only register the driver reads
         * back to decide the port speed.
         */
        if ((old & MUSB_POWER_RESET) && !(val & MUSB_POWER_RESET)) {
            USBDevice *dev = m->port.dev;

            if (dev && dev->attached) {
                usb_device_reset(dev);
                if (dev->speed == USB_SPEED_HIGH) {
                    m->power |= MUSB_POWER_HSMODE;
                } else {
                    m->power &= ~MUSB_POWER_HSMODE;
                }
            }
        }
        break;
    }
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
        /* musb_start()'s host branch sets DEVCTL.SESSION; that is our cue
         * that the guest is ready to see the coldplugged device connect. */
        am335x_musb_eval_connect(s);
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
/* CPPI4.1 DMA: queue-manager submit -> transfer -> completion data path.  */
/*
 * With the guest's default use_dma=true, musb routes every bulk/interrupt
 * transfer on EP1-15 through CPPI (musb_host.c:670-894 bypasses the PIO
 * FIFO). The provider driver builds an 8-word host descriptor in guest DMA
 * memory and pushes its physical address onto the endpoint's submit queue
 * (QMGR_QUEUE_D write). This model decodes that descriptor, moves the buffer
 * through the USB1 endpoint using the same usb_handle_packet() plumbing as
 * the PIO engine, writes the transferred length back into the descriptor,
 * and posts it onto the endpoint's completion queue -- setting the queue's
 * QMGR_PEND bit and raising USBSS_IRQ_STATUS.PD_COMP (INTC 17) so the
 * guest's cppi41_irq()/cppi41_pop_desc() drain it.
 */

/* True if any completion queue still holds an undrained descriptor. */
static bool am335x_cppi_cq_any_pending(AM335xUsbssState *s)
{
    for (unsigned q = 0; q < AM335X_CPPI_NUM_QUEUES; q++) {
        if (s->cppi.cq[q].count) {
            return true;
        }
    }
    return false;
}

/*
 * Level-drive the CPPI "glue" completion line (INTC 17).
 *
 * PD_COMP re-latches from completion-queue occupancy, not just the guest's
 * last W1C: on real hardware the status bit reflects "a completion queue has
 * an undrained descriptor", so a guest write that clears it while another
 * completion is still queued does not silence the line. This matters because
 * the guest's dsps ISR callback does a plain read-then-W1C of this bit once
 * per popped descriptor (musb_dsps.c:647-649) with the BQL dropped between
 * the two MMIOs; without re-latching from occupancy, a bottom-half raise
 * landing in that window is destroyed by the following W1C even though a
 * descriptor is still sitting in a cq[] ring -- silently losing the
 * interrupt and hanging a multi-packet (reload-chain) CPPI transfer. This
 * only manifested intermittently under fast host timing.
 */
static void am335x_cppi_update_irq(AM335xUsbssState *s)
{
    bool pending;

    if (am335x_cppi_cq_any_pending(s)) {
        s->cppi.irq_status |= USBSS_IRQ_PD_COMP;
    }
    pending = (s->cppi.irq_status & s->cppi.irq_enable &
               USBSS_IRQ_PD_COMP) != 0;
    qemu_set_irq(s->irq[2], pending);
}

/* Push a descriptor phys onto completion queue `q`'s ring. */
static void am335x_cppi_cq_push(AM335xUsbssState *s, unsigned q, uint32_t desc)
{
    AM335xCppiCq *cq;

    if (q >= AM335X_CPPI_NUM_QUEUES) {
        return;
    }
    cq = &s->cppi.cq[q];
    if (cq->count >= AM335X_CPPI_CQ_DEPTH) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "am335x-usbss: CPPI completion queue %u overflow\n", q);
        return;
    }
    cq->desc[cq->tail] = desc;
    cq->tail = (cq->tail + 1) % AM335X_CPPI_CQ_DEPTH;
    cq->count++;
}

/*
 * Bottom half: post every pending completed descriptor onto its completion
 * queue, then re-derive PD_COMP from occupancy. Runs in the main loop, so the
 * IRQ is (re)asserted outside any guest cppi41_irq context (see the
 * AM335xCppi.comp[] rationale and am335x_cppi_update_irq()).
 */
static void am335x_cppi_comp_bh(void *opaque)
{
    AM335xUsbssState *s = AM335X_USBSS(opaque);
    AM335xCppi *c = &s->cppi;

    while (c->comp_count) {
        AM335xCppiComp *e = &c->comp[c->comp_head];

        am335x_cppi_cq_push(s, e->q, e->desc);
        c->comp_head = (c->comp_head + 1) % AM335X_CPPI_COMP_RING;
        c->comp_count--;
    }
    am335x_cppi_update_irq(s);
}

/* Queue a finished descriptor for deferred completion posting (see above). */
static void am335x_cppi_defer_completion(AM335xUsbssState *s, unsigned q,
                                         uint32_t desc)
{
    AM335xCppi *c = &s->cppi;

    if (c->comp_count >= AM335X_CPPI_COMP_RING) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "am335x-usbss: CPPI completion ring overflow\n");
        return;
    }
    c->comp[c->comp_tail].q = q;
    c->comp[c->comp_tail].desc = desc;
    c->comp_tail = (c->comp_tail + 1) % AM335X_CPPI_COMP_RING;
    c->comp_count++;
    qemu_bh_schedule(s->cppi_comp_bh);
}

/*
 * Turn a finished CPPI transfer into descriptor write-back + completion-queue
 * post (happy path), or -- for a protocol stall/error -- into a musb endpoint
 * interrupt (INTC 19) so musb_host_{tx,rx}() aborts the channel, exactly as
 * the core would on real silicon.
 */
static void am335x_cppi_finish(AM335xUsbssState *s, unsigned ep, bool is_rx)
{
    AM335xMusbEp *e = &s->musb.ep[ep];
    AM335xMusbHalf *h = is_rx ? &e->rx : &e->tx;
    USBPacket *p = &h->packet;
    int status = p->status;
    uint32_t actual = (status >= 0) ? p->actual_length : 0;
    uint32_t pd0;
    uint8_t w[4];

    if (status == USB_RET_STALL || status < 0) {
        if (is_rx) {
            e->rxcsr |= (status == USB_RET_STALL) ? MUSB_RXCSR_H_RXSTALL
                                                  : MUSB_RXCSR_H_ERROR;
            am335x_musb_raise_rx(s, ep);
        } else {
            e->txcsr |= (status == USB_RET_STALL) ? MUSB_TXCSR_H_RXSTALL
                                                  : MUSB_TXCSR_H_ERROR;
            am335x_musb_raise_tx(s, ep);
        }
        h->cppi = false;
        h->active = false;
        usb_packet_cleanup(p);
        return;
    }

    /* IN: deliver the received bytes to the guest DMA buffer. */
    if (is_rx && actual) {
        dma_memory_write(&address_space_memory, h->cppi_buf_phys,
                         h->cppi_buf, actual, MEMTXATTRS_UNSPECIFIED);
    }

    /*
     * Write the actual transferred length into pd0 (keeping the descriptor
     * type). cppi41_irq() reads len = pd_trans_len(pd0) and residue =
     * pd_trans_len(pd6) - len (cppi41.c:349-354); pd6 still holds the
     * requested length, so residue = requested - actual as the driver
     * expects for short-packet detection.
     */
    pd0 = (h->cppi_pd0 & ~CPPI_PD0_LEN_MASK) | (actual & CPPI_PD0_LEN_MASK);
    stl_le_p(w, pd0);
    dma_memory_write(&address_space_memory, h->cppi_desc_phys, w, sizeof(w),
                     MEMTXATTRS_UNSPECIFIED);

    am335x_cppi_defer_completion(s, h->cppi_qcomp, h->cppi_desc_phys);

    h->cppi = false;
    h->active = false;
    usb_packet_cleanup(p);
}

/*
 * Decode the host descriptor at `desc_phys` and start the transfer on USB1
 * endpoint `ep` (direction from the submit queue). Reuses am335x_musb_issue()
 * -- the transfer completes synchronously, asynchronously (port .complete),
 * or NAK-repolls, all handled by am335x_musb_finish() -> am335x_cppi_finish.
 */
static void am335x_cppi_start(AM335xUsbssState *s, unsigned ep, bool is_rx,
                              uint32_t desc_phys)
{
    AM335xMusbHalf *h = is_rx ? &s->musb.ep[ep].rx : &s->musb.ep[ep].tx;
    uint8_t raw[CPPI_DESC_BYTES];
    uint32_t pd0, pd4, req_len;

    if (ep == 0 || ep >= AM335X_MUSB_NUM_EP) {
        return;
    }
    if (h->active) {
        /* The driver serialises one descriptor per channel; a push while a
         * transfer is still in flight would corrupt state, so drop it. */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "am335x-usbss: CPPI submit while EP%u %s busy\n",
                      ep, is_rx ? "RX" : "TX");
        return;
    }

    dma_memory_read(&address_space_memory, desc_phys, raw, sizeof(raw),
                    MEMTXATTRS_UNSPECIFIED);
    pd0 = ldl_le_p(raw + 0);
    pd4 = ldl_le_p(raw + 16);
    req_len = pd0 & CPPI_PD0_LEN_MASK;

    h->cppi = true;
    h->kind = is_rx ? XFER_CPPI_RX : XFER_CPPI_TX;
    h->cppi_desc_phys = desc_phys;
    h->cppi_buf_phys = pd4;
    h->cppi_pd0 = pd0;
    h->cppi_req_len = req_len;
    h->cppi_qcomp = is_rx ? (140 + ep) : (124 + ep);

    if (h->cppi_buf_cap < req_len) {
        h->cppi_buf = g_realloc(h->cppi_buf, req_len);
        h->cppi_buf_cap = req_len;
    }
    /* OUT: gather the data to send from the guest DMA buffer now. */
    if (!is_rx && req_len) {
        dma_memory_read(&address_space_memory, pd4, h->cppi_buf, req_len,
                        MEMTXATTRS_UNSPECIFIED);
    }

    am335x_musb_issue(s, ep, is_rx);
}

/* QMGR_QUEUE_D(n) write == a descriptor push onto submit queue `n`. */
static void am335x_cppi_submit(AM335xUsbssState *s, unsigned q, uint32_t val)
{
    uint32_t desc_phys = val & ~CPPI_PD_DESC_ALIGN;
    unsigned ep;

    if (q == CPPI_TD_SUBMIT_Q) {
        /* Teardown descriptor: remember it for the channel_abort handler. */
        s->cppi.td_desc_phys = desc_phys;
        return;
    }
    if (q >= CPPI_USB1_TX_SUBMIT_LO && q <= CPPI_USB1_TX_SUBMIT_HI &&
        !((q - CPPI_USB1_TX_SUBMIT_LO) & 1)) {
        ep = (q - CPPI_USB1_TX_SUBMIT_LO) / 2 + 1;
        am335x_cppi_start(s, ep, false, desc_phys);
    } else if (q >= CPPI_USB1_RX_SUBMIT_LO && q <= CPPI_USB1_RX_SUBMIT_HI) {
        ep = q - CPPI_USB1_RX_SUBMIT_LO + 1;
        am335x_cppi_start(s, ep, true, desc_phys);
    }
    /* USB0 (inert host) and unused submit slots: ignore. */
}

/* ----- CPPI DMA controller window MMIO (abs 0x47402000, 4KB) ----------- */
/*
 * The controller window is otherwise probe-time-only (channel-enable and
 * RXHPCRA0 writes that nothing ever reads back, cppi41.c:391,445,668) and is
 * served by the flat glue store like the scheduler window. The one write
 * with data-path meaning is a TXGCR(port)/RXGCR(port) write with
 * GCR_TEARDOWN set: cppi41_tear_down_chan() (cppi41.c:637-730) writes this
 * right after pushing a teardown descriptor onto submit queue 31 (captured
 * as cppi.td_desc_phys, see am335x_cppi_submit()), then polls the
 * teardown-complete queue (0) for both the in-flight transfer's own
 * descriptor and the teardown descriptor before considering the channel
 * torn down.
 */

/*
 * Abort the in-flight CPPI transfer (if any) on USB1 endpoint `port`-14's
 * `is_tx` half, and hand the driver's teardown poll loop the descriptor(s)
 * it looks for on the teardown-complete queue: the in-flight transfer's own
 * descriptor (if one was outstanding) and the teardown descriptor itself
 * (already latched from the submit-queue-31 push that precedes this GCR
 * write per the driver's own sequencing).
 */
static void am335x_cppi_teardown(AM335xUsbssState *s, unsigned port, bool is_tx)
{
    unsigned ep;
    AM335xMusbHalf *h;

    if (port < 15 || port >= 30) {
        return;                         /* USB0 (inert) or out of range */
    }
    ep = port - 14;
    h = is_tx ? &s->musb.ep[ep].tx : &s->musb.ep[ep].rx;

    if (h->cppi && h->active) {
        if (usb_packet_is_inflight(&h->packet)) {
            usb_cancel_packet(&h->packet);
        }
        usb_packet_cleanup(&h->packet);
        am335x_cppi_defer_completion(s, CPPI_TD_COMPLETE_Q, h->cppi_desc_phys);
        h->cppi = false;
        h->active = false;
    }
    if (s->cppi.td_desc_phys) {
        am335x_cppi_defer_completion(s, CPPI_TD_COMPLETE_Q, s->cppi.td_desc_phys);
    }
}

static uint64_t am335x_cppi_ctrl_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    AM335xUsbssState *s = AM335X_USBSS(opaque);

    return am335x_usbss_load(s->regs, CPPI_CTRL_OFFSET + offset, size);
}

static void am335x_cppi_ctrl_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    AM335xUsbssState *s = AM335X_USBSS(opaque);

    am335x_usbss_store(s->regs, CPPI_CTRL_OFFSET + offset, value, size);

    if (size == 4 && (value & CPPI_GCR_TEARDOWN) &&
        offset >= CPPI_DMA_TXGCR(0)) {
        unsigned rel = offset - CPPI_DMA_TXGCR(0);
        unsigned port = rel / 0x20;
        unsigned sub = rel % 0x20;

        if (sub == 0) {
            am335x_cppi_teardown(s, port, true);   /* TXGCR(port) */
        } else if (sub == (CPPI_DMA_RXGCR(0) - CPPI_DMA_TXGCR(0))) {
            am335x_cppi_teardown(s, port, false);  /* RXGCR(port) */
        }
    }
}

static const MemoryRegionOps am335x_cppi_ctrl_ops = {
    .read = am335x_cppi_ctrl_read,
    .write = am335x_cppi_ctrl_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* ----- CPPI queue-manager window MMIO (abs 0x47404000, 16KB) ----------- */

static uint64_t am335x_cppi_qmgr_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    AM335xUsbssState *s = AM335X_USBSS(opaque);

    /* QMGR_PEND(i): completion-queue pending bitmap (cppi41.c:315). */
    if (offset >= CPPI_QMGR_PEND(0) &&
        offset < CPPI_QMGR_PEND(CPPI_QMGR_PEND_SLOTS) &&
        ((offset - CPPI_QMGR_PEND(0)) & 3) == 0 && size == 4) {
        unsigned slot = (offset - CPPI_QMGR_PEND(0)) / 4;
        uint32_t v = 0;

        for (unsigned b = 0; b < 32; b++) {
            unsigned q = slot * 32 + b;
            if (q < AM335X_CPPI_NUM_QUEUES && s->cppi.cq[q].count) {
                v |= 1u << b;
            }
        }
        return v;
    }

    /* QMGR_QUEUE_D(n) read: pop a descriptor from completion queue n. */
    if (offset >= CPPI_QMGR_QUEUE_BASE && offset < CPPI_QMGR_QUEUE_END &&
        (offset & 0xf) == CPPI_QMGR_QUEUE_D_OFF && size == 4) {
        unsigned q = (offset - CPPI_QMGR_QUEUE_BASE) / 0x10;
        AM335xCppiCq *cq = &s->cppi.cq[q];

        if (cq->count) {
            uint32_t d = cq->desc[cq->head];

            cq->head = (cq->head + 1) % AM335X_CPPI_CQ_DEPTH;
            cq->count--;
            /* A direct pop outside the STATUS-read/W1C ISR pairing (e.g. the
             * teardown path's cppi41_pop_desc calls) can drain the last
             * occupied queue; let the level react immediately rather than
             * only on the next STATUS write. */
            am335x_cppi_update_irq(s);
            return d;
        }
        return 0;
    }

    /* Probe-time LRAM/MEMBASE/MEMCTRL/QUEUE_{A,B,C}: flat glue store. */
    return am335x_usbss_load(s->regs, CPPI_QMGR_OFFSET + offset, size);
}

static void am335x_cppi_qmgr_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    AM335xUsbssState *s = AM335X_USBSS(opaque);

    /* QMGR_QUEUE_D(n) write: descriptor push onto submit queue n. */
    if (offset >= CPPI_QMGR_QUEUE_BASE && offset < CPPI_QMGR_QUEUE_END &&
        (offset & 0xf) == CPPI_QMGR_QUEUE_D_OFF && size == 4) {
        unsigned q = (offset - CPPI_QMGR_QUEUE_BASE) / 0x10;

        am335x_cppi_submit(s, q, (uint32_t)value);
        return;
    }

    am335x_usbss_store(s->regs, CPPI_QMGR_OFFSET + offset, value, size);
}

static const MemoryRegionOps am335x_cppi_qmgr_ops = {
    .read = am335x_cppi_qmgr_read,
    .write = am335x_cppi_qmgr_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* ======================================================================= */
/* USB1 musb host: USBBus / USBPort.                                       */

/* Arm the shared poll timer for the earliest pending half's deadline. */
static void am335x_musb_arm(AM335xUsbssState *s)
{
    int64_t best = INT64_MAX;

    for (unsigned ep = 0; ep < AM335X_MUSB_NUM_EP; ep++) {
        AM335xMusbEp *e = &s->musb.ep[ep];

        if (e->tx.active && !usb_packet_is_inflight(&e->tx.packet) &&
            e->tx.next_poll < best) {
            best = e->tx.next_poll;
        }
        if (e->rx.active && !usb_packet_is_inflight(&e->rx.packet) &&
            e->rx.next_poll < best) {
            best = e->rx.next_poll;
        }
    }
    if (best != INT64_MAX) {
        timer_mod(s->musb.nak_timer, best);
    }
}

/*
 * Re-issue every endpoint half whose (re)poll deadline has arrived and whose
 * packet is not still async-inflight. Runs off the QEMU_CLOCK_VIRTUAL timer
 * so guest time advances between polls; per-half next_poll deadlines keep
 * interrupt endpoints at their interval.
 */
static void am335x_musb_retry_all(AM335xUsbssState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    for (unsigned ep = 0; ep < AM335X_MUSB_NUM_EP; ep++) {
        AM335xMusbEp *e = &s->musb.ep[ep];

        if (e->tx.active && !usb_packet_is_inflight(&e->tx.packet) &&
            now >= e->tx.next_poll) {
            am335x_musb_issue(s, ep, false);
        }
        if (e->rx.active && !usb_packet_is_inflight(&e->rx.packet) &&
            now >= e->rx.next_poll) {
            am335x_musb_issue(s, ep, true);
        }
    }
    am335x_musb_arm(s);
}

static void am335x_musb_nak_timer(void *opaque)
{
    am335x_musb_retry_all(opaque);
}

/* A device signalled an endpoint has data/space: run the poll pass soon
 * (via the timer, never a bottom half, so the virtual clock keeps
 * advancing). Per-endpoint next_poll deadlines still gate each endpoint. */
static void am335x_musb_wake(AM335xUsbssState *s)
{
    timer_mod(s->musb.nak_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                 AM335X_MUSB_WAKE_RETRY_NS);
}

static void am335x_musb_attach(USBPort *port)
{
    AM335xUsbssState *s = port->opaque;

    if (!port->dev || !port->dev->attached) {
        return;
    }
    /* If the session is already active, connect now; otherwise the connect
     * is raised when the guest sets DEVCTL.SESSION (see eval_connect). */
    am335x_musb_eval_connect(s);
}

static void am335x_musb_detach(USBPort *port)
{
    AM335xUsbssState *s = port->opaque;
    AM335xMusb *m = &s->musb;

    m->connected = false;
    m->devctl &= ~(MUSB_DEVCTL_VBUS | MUSB_DEVCTL_FSDEV | MUSB_DEVCTL_LSDEV);
    am335x_musb_raise_core(s, MUSB_INTR_DISCONNECT);
}

static void am335x_musb_child_detach(USBPort *port, USBDevice *child)
{
}

static void am335x_musb_wakeup(USBPort *port)
{
    AM335xUsbssState *s = port->opaque;

    am335x_musb_wake(s);
}

static void am335x_musb_async_complete(USBPort *port, USBPacket *packet)
{
    AM335xUsbssState *s = port->opaque;

    for (unsigned ep = 0; ep < AM335X_MUSB_NUM_EP; ep++) {
        AM335xMusbEp *e = &s->musb.ep[ep];

        for (unsigned r = 0; r < 2; r++) {
            AM335xMusbHalf *h = r ? &e->rx : &e->tx;

            if (&h->packet != packet) {
                continue;
            }
            if (packet->status == USB_RET_REMOVE_FROM_QUEUE) {
                usb_cancel_packet(packet);
                usb_packet_cleanup(packet);
                h->active = false;
                return;
            }
            am335x_musb_finish(s, ep, r != 0);
            return;
        }
    }
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
    AM335xUsbssState *s = container_of(m, AM335xUsbssState, musb);

    am335x_musb_wake(s);
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

    if (m->nak_timer) {
        timer_del(m->nak_timer);
    }
    if (s->cppi_comp_bh) {
        qemu_bh_cancel(s->cppi_comp_bh);
    }

    for (unsigned i = 0; i < AM335X_MUSB_NUM_EP; i++) {
        AM335xMusbEp *e = &m->ep[i];
        e->txmaxp = e->txcsr = e->rxmaxp = e->rxcsr = e->rxcount = 0;
        e->txtype = e->txinterval = e->rxtype = e->rxinterval = 0;
        e->txfifosz = e->rxfifosz = 0;
        e->txfifoadd = e->rxfifoadd = 0;
        memset(e->busctl, 0, sizeof(e->busctl));
        for (unsigned r = 0; r < 2; r++) {
            AM335xMusbHalf *h = r ? &e->rx : &e->tx;
            h->fifo_len = 0;
            h->fifo_rd = 0;
            if (usb_packet_is_inflight(&h->packet)) {
                usb_cancel_packet(&h->packet);
                usb_packet_cleanup(&h->packet);
            }
            h->active = false;
            h->cppi = false;
            g_free(h->cppi_buf);
            h->cppi_buf = NULL;
            h->cppi_buf_cap = 0;
        }
    }

    m->connected = false;
}

static void am335x_usbss_reset(DeviceState *dev)
{
    AM335xUsbssState *s = AM335X_USBSS(dev);

    memset(s->regs, 0, AM335X_USBSS_SIZE);

    /* USB0 clean-probe PHY/mode reset values (USB1 handled by musb reset). */
    stl_le_p(s->regs + USB0_CTRL_BASE + CTRL_PHY_UTMI, USB_PHY_UTMI_RESET);
    stl_le_p(s->regs + USB0_CTRL_BASE + CTRL_MODE, USB_MODE_RESET);

    am335x_musb_reset(s);

    /* CPPI4.1 queue-manager completion state. */
    memset(&s->cppi, 0, sizeof(s->cppi));

    qemu_set_irq(s->irq[0], 0);
    qemu_set_irq(s->irq[1], 0);
    qemu_set_irq(s->irq[2], 0);
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

    /* CPPI4.1 queue-manager window: functional submit/completion data path.
     * The controller window is functional only for GCR_TEARDOWN (channel
     * abort); the scheduler/glue-probe windows stay flat-store clean-probe. */
    memory_region_init_io(&s->cppi_ctrl, OBJECT(s), &am335x_cppi_ctrl_ops, s,
                          "am335x-usbss-cppi-ctrl", CPPI_CTRL_SIZE);
    memory_region_add_subregion_overlap(&s->container, CPPI_CTRL_OFFSET,
                                        &s->cppi_ctrl, 1);
    memory_region_init_io(&s->cppi_qmgr, OBJECT(s), &am335x_cppi_qmgr_ops, s,
                          "am335x-usbss-cppi-qmgr", CPPI_QMGR_SIZE);
    memory_region_add_subregion_overlap(&s->container, CPPI_QMGR_OFFSET,
                                        &s->cppi_qmgr, 1);

    sysbus_init_mmio(sbd, &s->container);

    /*
     * Interrupt outputs: musb "mc" lines -> INTC 18 (USB0)/19 (USB1), and the
     * CPPI4.1 DMA completion "glue" line -> INTC 17 (irq[2]).
     */
    sysbus_init_irq(sbd, &s->irq[0]);
    sysbus_init_irq(sbd, &s->irq[1]);
    sysbus_init_irq(sbd, &s->irq[2]);

    /* USB1 host bus + type-A root port (full/low/high speed). */
    usb_bus_new(&m->bus, sizeof(m->bus), &am335x_musb_bus_ops, dev);
    usb_register_port(&m->bus, &m->port, s, 0, &am335x_musb_port_ops,
                      USB_SPEED_MASK_LOW | USB_SPEED_MASK_FULL |
                      USB_SPEED_MASK_HIGH);

    m->nak_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, am335x_musb_nak_timer, s);
    s->cppi_comp_bh = qemu_bh_new(am335x_cppi_comp_bh, s);
}

static void am335x_usbss_unrealize(DeviceState *dev)
{
    AM335xUsbssState *s = AM335X_USBSS(dev);

    if (s->musb.nak_timer) {
        timer_free(s->musb.nak_timer);
        s->musb.nak_timer = NULL;
    }
    if (s->cppi_comp_bh) {
        qemu_bh_delete(s->cppi_comp_bh);
        s->cppi_comp_bh = NULL;
    }
    for (unsigned i = 0; i < AM335X_MUSB_NUM_EP; i++) {
        AM335xMusbEp *e = &s->musb.ep[i];

        g_free(e->tx.cppi_buf);
        e->tx.cppi_buf = NULL;
        g_free(e->rx.cppi_buf);
        e->rx.cppi_buf = NULL;
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
