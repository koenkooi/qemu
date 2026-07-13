/*
 * TI AM335x UART emulation.
 *
 * Wraps the generic 16550 (hw/char/serial.c, embedded the same way
 * hw/char/serial-mm.c does) with the small set of extra OMAP-only
 * registers -- MDR1/MDR2, SYSC, SYSS, and friends -- that the Linux
 * 8250_omap console driver pokes at during probe. In particular,
 * 8250_omap's soft-reset sequence writes SYSC.SOFTRESET and then polls
 * SYSS.RESETDONE; a plain serial_mm_init() only maps the 8 standard
 * 16550 registers (32 bytes), so that poll reads unmapped MMIO and can
 * hang the console bind. See hw/char/am335x_uart.c for the register
 * map and reset handling.
 *
 * Copyright (C) 2026 Koen Kooi
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

#ifndef HW_CHAR_AM335X_UART_H
#define HW_CHAR_AM335X_UART_H

#include "hw/sysbus.h"
#include "hw/char/serial.h"
#include "qom/object.h"

#define TYPE_AM335X_UART "am335x-uart"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xUartState, AM335X_UART)

/* Full L4 peripheral window mapped for one AM335x UART instance. */
#define AM335X_UART_MMIO_SIZE 0x1000

struct AM335xUartState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    SerialState serial;
    MemoryRegion iomem;

    /*
     * Flat backing store for the whole 4KB window, indexed by
     * offset >> 2. Registers 0-7 (offsets 0x00-0x1C) are never read from
     * or written to this array -- those are forwarded live to `serial`
     * (see am335x_uart.c) -- so the corresponding low entries are unused.
     * Everything from MDR1 (0x20) up is plain read-back-what-was-written
     * storage here, except SYSC/SYSS which get the special handling
     * described in am335x_uart.c.
     */
    uint32_t scratch[AM335X_UART_MMIO_SIZE / 4];

    /*
     * Set by a SYSC.SOFTRESET write. The reset modelled here always
     * completes synchronously, so SYSS.RESETDONE reads back 1
     * unconditionally regardless of this flag (see am335x_uart_read());
     * it is kept for migration/introspection rather than to gate any
     * behaviour.
     */
    bool reset_done;
};

#endif /* HW_CHAR_AM335X_UART_H */
