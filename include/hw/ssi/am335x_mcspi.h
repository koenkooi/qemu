/*
 * TI AM335x McSPI0 (Multichannel Serial Port Interface) "clean probe" stub.
 *
 * Reference: TRM SPRUH73Q ch.24 "Multichannel Serial Port Interface
 * (McSPI)", base 0x48030000, 4KB window (Table 2-3 "L4_PER Peripheral
 * Memory Map"; DT compatible "ti,omap4-mcspi",
 * arch/arm/boot/dts/ti/omap/am33xx-l4.dtsi target-module@30000). Kernel
 * driver: drivers/spi/spi-omap2-mcspi.c.
 *
 * Scope: this model exists solely to let the ti-sysc bus wrapper's OCP
 * softreset handshake complete -- which today times out against
 * unmapped memory and stops omap2_mcspi_probe() from ever running -- and
 * then to accept the handful of register writes that probe performs. No
 * SPI transfer, chip-select sequencing, FIFO, or attached device is
 * modelled: no BeagleBone-family board in this project boots from SPI or
 * populates McSPI0/1's expansion-header pins with a device. See
 * am335x_mcspi.c for the register-by-register rationale.
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

#ifndef HW_SSI_AM335X_MCSPI_H
#define HW_SSI_AM335X_MCSPI_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_AM335X_MCSPI "am335x-mcspi"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xMcspiState, AM335X_MCSPI)

/* Whole "McSPI0 Registers" target-module window (TRM Table 2-3: base
 * 0x4803_0000, size 4KB). The real per-register file (MCSPI_REVISION..
 * MCSPI_DAFRX, Table 24-10) only spans offsets 0x0-0x1A0; the remainder
 * of the 4KB window is reserved and backed by the same flat store. */
#define AM335X_MCSPI_SIZE 0x1000

struct AM335xMcspiState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;

    /* INTC output line 65 (McSPI0INT, TRM Table 6-1). Wired but never
     * asserted -- see am335x_mcspi.c. */
    qemu_irq irq;

    /*
     * Flat, byte-addressable backing store for the whole window,
     * allocated at realize -- structural template hw/misc/am335x_usbss.c.
     * Every access from the guest is a plain 32-bit readl/writel
     * (spi-omap2-mcspi.c's mcspi_read_reg()/mcspi_write_reg()), but a
     * byte store keeps offset arithmetic simple and matches the USBSS
     * precedent. Reads/writes hit this array directly except for the
     * two synthesized registers handled in am335x_mcspi.c (MCSPI_REVISION
     * and MCSPI_SYSSTATUS).
     */
    uint8_t *regs;
};

#endif /* HW_SSI_AM335X_MCSPI_H */
