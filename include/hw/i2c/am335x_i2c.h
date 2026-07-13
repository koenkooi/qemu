/*
 * TI AM335x I2C controller ("ti,omap4-i2c").
 *
 * Reference: TRM spruh73q chapter 21 "Multimaster High-Speed I2C
 * Controller", I2C0 base 0x44E0B000 (single 0x1000 window that also
 * carries the ti-sysc target-module rev/sysc/syss registers at
 * offsets 0x00/0x10/0x90).
 *
 * This models the OMAP4+ "v2" register layout (SCHEME 1) that the Linux
 * i2c-omap driver selects for AM335x -- register-incompatible with the
 * OMAP2/3 map in hw/i2c/omap_i2c.c. It is a master-only model: it drives
 * the generic QEMU I2C bus so on-board slaves (board-ID EEPROM, TPS65217
 * PMIC) can be probed, and it reports the OCP softreset as complete so
 * neither the ti-sysc wrapper nor the driver's own reset poll stalls.
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

#ifndef HW_I2C_AM335X_I2C_H
#define HW_I2C_AM335X_I2C_H

#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_AM335X_I2C "am335x-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xI2cState, AM335X_I2C)

struct AM335xI2cState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;

    /*
     * Register storage (v2 layout; see am335x_i2c.c). REVNB_LO/HI, SYSS
     * and BUFSTAT are read-only constants with no backing field, and the
     * IRQSTATUS.BB/BF/RRDY/XRDY bits are synthesized from the transfer
     * state below rather than stored.
     */
    uint16_t sysc;           /* I2C_SYSC        (0x10) */
    uint16_t irqstatus;      /* latched event bits (ARDY/NACK/AL); W1C */
    uint16_t irqenable;      /* mask behind IRQENABLE_SET/CLR (0x2c/0x30) */
    uint16_t we;             /* I2C_WE          (0x34) */
    uint16_t buf;            /* I2C_BUF         (0x94) */
    uint16_t cnt;            /* I2C_CNT DCOUNT  (0x98) */
    uint8_t  data;           /* latched RX byte / I2C_DATA (0x9c) */
    uint16_t con;            /* I2C_CON         (0xa4) */
    uint16_t oa;             /* I2C_OA          (0xa8) */
    uint16_t sa;             /* I2C_SA          (0xac) */
    uint8_t  psc;            /* I2C_PSC         (0xb0) */
    uint8_t  scll;           /* I2C_SCLL        (0xb4) */
    uint8_t  sclh;           /* I2C_SCLH        (0xb8) */
    uint16_t systest;        /* I2C_SYSTEST     (0xbc) */

    /* Master-transfer FSM state (not directly register-mapped). */
    bool busy;               /* a transfer is open on the bus */
    bool recv;               /* current transfer direction is read */
    int  count_cur;          /* bytes remaining in the current message */
};

/* Board accessor: the I2C bus the on-board slaves attach to. */
I2CBus *am335x_i2c_bus(DeviceState *dev);

#endif /* HW_I2C_AM335X_I2C_H */
