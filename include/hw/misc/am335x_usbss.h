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
 *   0x2000 CPPI4.1 DMA controller/sched (probe-time flat store)
 *   0x4000 CPPI4.1 DMA queue-manager   (functional submit/completion path)
 *
 * USB1 is the board's type-A host port. It is modelled well enough for the
 * real musb-hdrc host stack to enumerate a device attached to its USBPort
 * (`-device usb-...`) and move bulk/interrupt data -- both over PIO and,
 * with the guest's default use_dma=true, over a functional CPPI4.1 DMA data
 * path (queue-manager submit -> transfer -> completion, INTC 17). USB0
 * remains a clean-probe stub and its CPPI channels stay inert.
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

/*
 * One direction (TX or RX) of a hardware endpoint. musb endpoints are
 * bidirectional: the TX and RX halves of the same hardware endpoint can be
 * assigned to two different device endpoints and carry transfers at the
 * same time (e.g. a hub's interrupt IN on the RX half while a mass-storage
 * bulk OUT uses the TX half). Each half therefore owns an independent
 * in-flight transfer and its own PIO FIFO (writes to the FIFO port land in
 * the TX FIFO, reads drain the RX FIFO).
 */
typedef struct AM335xMusbHalf {
    USBPacket packet;
    bool      active;       /* async-inflight or awaiting a (re)poll        */
    uint8_t   kind;         /* AM335xMusbXfer phase/direction (in .c)       */
    int64_t   next_poll;    /* earliest QEMU_CLOCK_VIRTUAL time to (re)issue */
    uint8_t   fifo[AM335X_MUSB_FIFO_SIZE];
    uint32_t  fifo_len;     /* valid bytes                                  */
    uint32_t  fifo_rd;      /* drain cursor                                 */

    /*
     * CPPI4.1 DMA-driven transfer state. Set when this half's in-flight
     * transfer was armed by a queue-manager descriptor push (default
     * use_dma=true) rather than a FIFO/CSR poke: the data moves between the
     * guest DMA buffer (cppi_buf_phys) and the USB endpoint via cppi_buf
     * instead of the PIO fifo, and completion posts a descriptor onto the
     * endpoint's CPPI completion queue + raises the USBSS PD_COMP interrupt.
     */
    bool      cppi;             /* in-flight transfer is CPPI-driven        */
    /*
     * RX only: set once DMAENAB (or a CPPI submit) has armed DMA for the
     * current transfer setup, cleared by the next FIFO flush / data-toggle
     * clear (musb_rx_reinit, i.e. a fresh transfer). Lets rx_poke tell a
     * genuine PIO IN request (H_REQPKT with no DMA ever armed -- the driver's
     * dma_channel==NULL fallback, which must be serviced) apart from the
     * stray H_REQPKT musb_host_rx leaves set after a *completed* CPPI IN
     * (which must be ignored, else it steals a packet from the DMA endpoint).
     */
    bool      dma_epoch;
    uint32_t  cppi_desc_phys;   /* descriptor guest phys (for write-back)   */
    uint32_t  cppi_buf_phys;    /* data buffer guest phys (pd4)             */
    uint32_t  cppi_pd0;         /* descriptor pd0 (type|len) as submitted   */
    uint32_t  cppi_req_len;     /* requested transfer length                */
    uint16_t  cppi_qcomp;       /* completion queue number                  */
    uint8_t  *cppi_buf;         /* bounce buffer (grown to cppi_req_len)    */
    uint32_t  cppi_buf_cap;     /* allocated size of cppi_buf               */
} AM335xMusbHalf;

/* Per-endpoint musb-core register state (host mode). */
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

    /*
     * tx = OUT direction (guest fills fifo, host sends; EP0 SETUP/OUT/
     * OUT-status use this half). rx = IN direction (host fills fifo, guest
     * reads; EP0 IN/IN-status use this half).
     */
    AM335xMusbHalf tx;
    AM335xMusbHalf rx;
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
    QEMUTimer *nak_timer;       /* re-polls NAK'd / woken bulk+int endpoints */

    bool     connected;         /* CONNECT signalled for the attached dev  */
} AM335xMusb;

/*
 * CPPI4.1 queue-manager completion model. On a submit-queue push the model
 * decodes the 8-word host descriptor, moves the buffer through the USB1
 * endpoint, then posts the descriptor onto the endpoint's completion queue
 * so the guest's cppi41_pop_desc()/cppi41_irq() drain it. The am335x queue
 * tables (drivers/dma/ti/cppi41.c:155-225) use queue numbers up to 155; a
 * small per-queue ring holds descriptors posted to a completion (or the
 * teardown-complete) queue until the guest pops them via QMGR_QUEUE_D.
 */
#define AM335X_CPPI_NUM_QUEUES 156
#define AM335X_CPPI_CQ_DEPTH   8
#define AM335X_CPPI_COMP_RING  64

typedef struct AM335xCppiCq {
    uint32_t desc[AM335X_CPPI_CQ_DEPTH];
    uint8_t  head;
    uint8_t  tail;
    uint8_t  count;
} AM335xCppiCq;

/* A finished transfer awaiting completion-queue posting from the bottom half. */
typedef struct AM335xCppiComp {
    uint16_t q;
    uint32_t desc;
} AM335xCppiComp;

typedef struct AM335xCppi {
    uint32_t irq_status;    /* USBSS_IRQ_STATUS  (glue+0x28): PD_COMP=bit2  */
    uint32_t irq_enable;    /* USBSS_IRQ_ENABLER/CLEARR (glue+0x2c/0x30)    */
    uint32_t td_desc_phys;  /* teardown descriptor last pushed to queue 31  */
    AM335xCppiCq cq[AM335X_CPPI_NUM_QUEUES];

    /*
     * Completions are posted (queue + PD_COMP IRQ) from a bottom half, not
     * inline from the transfer's finish: a CPPI RX transfer's completion IRQ
     * makes the guest reload the next packet, whose submit can complete
     * synchronously and would otherwise re-post the IRQ *while the guest is
     * still inside cppi41_irq* -- the dsps callback clears PD_COMP mid-ISR,
     * so an inline re-raise races the interrupt-controller ack and is lost.
     * Deferring to the main loop serialises each reload iteration cleanly.
     */
    AM335xCppiComp comp[AM335X_CPPI_COMP_RING];
    uint8_t  comp_head;
    uint8_t  comp_tail;
    uint8_t  comp_count;
} AM335xCppi;

struct AM335xUsbssState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion container;     /* the 32KB sysbus window                 */
    MemoryRegion glue;          /* flat store, priority 0                 */
    MemoryRegion musb_ctrl;     /* USB1 wrapper, priority 1               */
    MemoryRegion musb_mc;       /* USB1 musb-core, priority 1             */
    MemoryRegion cppi_ctrl;     /* CPPI DMA controller window, priority 1 */
    MemoryRegion cppi_qmgr;     /* CPPI queue-manager window, priority 1  */

    /*
     * Interrupt outputs. irq[0]/irq[1] are the per-instance musb "mc" lines
     * -> INTC 18 (USB0)/19 (USB1); irq[1] (USB1) is driven from the wrapper
     * interrupt-status registers by the functional host model, irq[0] (USB0)
     * stays idle. irq[2] is the CPPI4.1 DMA completion ("glue") line -> INTC
     * 17, asserted while USBSS_IRQ_STATUS.PD_COMP is pending+enabled.
     */
    qemu_irq irq[3];

    /*
     * Flat, byte-addressable backing store for the whole window (glue).
     * The USB1 control+mc sub-windows are overlaid by musb_ctrl/musb_mc
     * and never reach this store; everything else (ti-sysc wrapper, USB0
     * control+mc clean-probe stub, PHY windows, CPPI DMA range) does.
     */
    uint8_t *regs;

    /* USB1 functional host controller. */
    AM335xMusb musb;

    /* CPPI4.1 DMA queue-manager completion state (USB1 endpoints). */
    AM335xCppi cppi;
    QEMUBH *cppi_comp_bh;       /* drains cppi.comp[] -> completion queues */
};

#endif /* HW_MISC_AM335X_USBSS_H */
