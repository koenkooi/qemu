/*
 * TI AM335x USB Subsystem (USBSS): glue register file + USB1 musb host.
 *
 * Reference: TRM spruh73q chapter 16 "Universal Serial Bus", base
 * 0x47400000, 32KB window (am33xx.dtsi target-module@47400000). Kernel
 * driver: drivers/usb/musb/musb_dsps.c (DSPS glue) + drivers/usb/musb/
 * musb_core.c + musb_host.c + musb_virthub.c (Mentor musb-hdrc core),
 * matched against DT compatible "ti,musb-am33xx".
 *
 * The 32KB window holds two musb instances plus glue:
 *   0x0000 ti-sysc target-module wrapper (rev @0x00, sysconfig @0x10)
 *   0x1000 USB0 "control" wrapper      | left as a clean-probe register
 *   0x1400 USB0 "mc" musb-core         | file (peripheral/OTG, out of scope)
 *   0x1800 USB1 "control" wrapper      | modelled as a functional
 *   0x1c00 USB1 "mc" musb-core         | host controller (see am335x_usbss.c)
 *   0x2000 CPPI4.1 DMA glue/queue-mgr  (out of scope, flat store)
 *
 * USB1 is the board's type-A host port. It is modelled well enough for the
 * real musb-hdrc host stack to enumerate a device attached to its USBPort
 * (`-device usb-...`) over PIO -- control + bulk/interrupt transfers, no
 * CPPI4.1 DMA (the guest must load musb_hdrc with use_dma=0). USB0 and the
 * CPPI DMA engine remain clean-probe stubs.
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

#ifndef HW_MISC_AM335X_USBSS_H
#define HW_MISC_AM335X_USBSS_H

#include "hw/sysbus.h"
#include "hw/usb.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_AM335X_USBSS "am335x-usbss"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xUsbssState, AM335X_USBSS)

/* Size of the whole "usb" target-module window (am33xx.dtsi ranges
 * property: <0x0 0x47400000 0x8000>). */
#define AM335X_USBSS_SIZE 0x8000

/* USB1 sub-block offsets within the 32KB window (overlaid on the flat
 * glue store as higher-priority functional regions). */
#define AM335X_USB1_CTRL_OFFSET  0x1800
#define AM335X_USB1_CTRL_SIZE    0x0200
#define AM335X_USB1_MC_OFFSET    0x1c00
#define AM335X_USB1_MC_SIZE      0x0400

/*
 * The Mentor musb-hdrc core has 16 hardware endpoints (EP0 + EP1..15),
 * TX and RX shared per endpoint index. Host mode addresses them through
 * the INDEX register (mc+0x0E), which selects whose window (mc+0x10..0x1F)
 * and busctl block are visible.
 */
#define AM335X_MUSB_NUM_EP   16

/*
 * Per-endpoint PIO FIFO. musb host loads at most one max-packet before
 * setting TXPKTRDY / draining after RXPKTRDY; 4KB covers any programmed
 * TXMAXP/RXMAXP (incl. high-bandwidth ISO 3x1024) with margin.
 */
#define AM335X_MUSB_FIFO_SIZE 4096

/* Per-endpoint musb-core register + FIFO state (host mode). */
typedef struct AM335xMusbEp {
    uint16_t txmaxp;        /* indexed 0x10 */
    uint16_t txcsr;         /* indexed 0x12 (CSR0 for EP0) */
    uint16_t rxmaxp;        /* indexed 0x14 */
    uint16_t rxcsr;         /* indexed 0x16 */
    uint16_t rxcount;       /* indexed 0x18 (COUNT0 for EP0) */
    uint8_t  txtype;        /* indexed 0x1a (TYPE0 for EP0) */
    uint8_t  txinterval;    /* indexed 0x1b (NAKLIMIT0 for EP0) */
    uint8_t  rxtype;        /* indexed 0x1c */
    uint8_t  rxinterval;    /* indexed 0x1d */
    uint8_t  txfifosz;      /* mc 0x62 (indexed) */
    uint8_t  rxfifosz;      /* mc 0x63 (indexed) */
    uint16_t txfifoadd;     /* mc 0x64 (indexed) */
    uint16_t rxfifoadd;     /* mc 0x66 (indexed) */
    uint8_t  busctl[8];     /* mc 0x80+8*ep: TXFUNCADDR..RXHUBPORT */

    uint8_t  fifo[AM335X_MUSB_FIFO_SIZE];
    uint32_t fifo_len;      /* valid bytes in fifo                       */
    uint32_t fifo_rd;       /* RX drain cursor                           */
} AM335xMusbEp;

/*
 * USB1 musb host controller state. Embedded in AM335xUsbssState; not a
 * separate QOM device, so its MMIO ops share the parent's opaque.
 */
typedef struct AM335xMusb {
    /* "control" wrapper block (abs 0x47401800), DSPS am33xx offsets */
    uint32_t control;           /* 0x14 */
    uint32_t status;            /* 0x18 */
    uint32_t epintr_status;     /* 0x30: raw pending TX(0-15)/RX(16-31)   */
    uint32_t coreintr_status;   /* 0x34: raw pending INTRUSB(0-7)/vbus(8) */
    uint32_t epintr_enable;     /* via epintr_set 0x38 / epintr_clear 0x40 */
    uint32_t coreintr_enable;   /* via coreintr_set 0x3c / coreintr_clear 0x44 */
    uint32_t phy_utmi;          /* 0xe0 */
    uint32_t mode;              /* 0xe8 */
    uint32_t tx_mode;           /* 0x70 */
    uint32_t rx_mode;           /* 0x74 */

    /* "mc" musb-core common registers (abs 0x47401c00) */
    uint8_t  faddr;             /* 0x00 */
    uint8_t  power;             /* 0x01 */
    uint16_t intrtxe;           /* 0x06 */
    uint16_t intrrxe;           /* 0x08 */
    uint8_t  intrusbe;          /* 0x0b */
    uint16_t frame;             /* 0x0c */
    uint8_t  index;             /* 0x0e: selects the indexed EP window    */
    uint8_t  testmode;          /* 0x0f */
    uint8_t  devctl;            /* 0x60 */
    uint8_t  babble_ctl;        /* 0x61 */

    AM335xMusbEp ep[AM335X_MUSB_NUM_EP];

    /* USB host framework */
    USBBus    bus;
    USBPort   port;
    USBPacket packet;           /* single in-flight PIO transfer          */
    QEMUBH   *async_bh;         /* completes async/retried transfers      */
    QEMUTimer *nak_timer;       /* re-drives a NAK'd control/bulk/int poll */

    /* in-flight transfer bookkeeping (valid while async/NAK pending) */
    bool     xfer_active;
    bool     xfer_is_rx;        /* IN (device->host) vs OUT/SETUP         */
    uint8_t  xfer_ep;           /* hardware endpoint index                */
    uint8_t  xfer_devaddr;
    uint8_t  xfer_pid;          /* USB_TOKEN_SETUP/IN/OUT                 */
} AM335xMusb;

struct AM335xUsbssState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion container;     /* the 32KB sysbus window                 */
    MemoryRegion glue;          /* flat store, priority 0                 */
    MemoryRegion musb_ctrl;     /* USB1 wrapper, priority 1               */
    MemoryRegion musb_mc;       /* USB1 musb-core, priority 1             */

    /*
     * Per-instance musb "mc" interrupt outputs -> INTC 18 (USB0)/19 (USB1).
     * irq[1] (USB1) is driven from the wrapper interrupt-status registers
     * by the functional host model; irq[0] (USB0) stays idle.
     */
    qemu_irq irq[2];

    /*
     * Flat, byte-addressable backing store for the whole window (glue).
     * The USB1 control+mc sub-windows are overlaid by musb_ctrl/musb_mc
     * and never reach this store; everything else (ti-sysc wrapper, USB0
     * control+mc clean-probe stub, PHY windows, CPPI DMA range) does.
     */
    uint8_t *regs;

    /* USB1 functional host controller. */
    AM335xMusb musb;
};

#endif /* HW_MISC_AM335X_USBSS_H */
