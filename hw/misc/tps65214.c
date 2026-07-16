/*
 * TI TPS65214 Power Management IC emulation (I2C slave).
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Minimal register-file model of the TPS65214 PMIC as found on the Seeed
 * BeagleBone Green Eco (I2C0, 7-bit address 0x30; the orderable part on that
 * board is the TPS6521403). The TPS65214 is part of TI's TPS65219/TPS65215/
 * TPS65214 family and, unlike the BeagleBone Black's TPS65217C
 * (hw/misc/tps65217.c), is driven by the *regmap*-based mfd driver
 * drivers/mfd/tps65219.c (compatible "ti,tps65214").
 *
 * That driver's probe path (tps65219_probe(), chip_id == TPS65214) makes only
 * two kinds of I2C access this model has to answer:
 *
 *   1. A single register *write* to unlock the register file: it writes
 *      TPS65214_LOCK_ACCESS_CMD (0x5A) to TPS65214_REG_LOCK (0x03). If that
 *      transaction NAKs, probe bails out with "Failed to unlock registers" and
 *      no child device (regulators, GPIO) is registered.
 *   2. The regmap-irq setup (devm_regmap_add_irq_chip()): masks/clears the
 *      INT_SOURCE (0x2B) main-status register and its LDO/BUCK/SYS sub-status
 *      + mask registers. These are plain 8-bit-register reads and writes.
 *
 * Crucially, the driver performs *no* chip-ID read or value check at all -- it
 * trusts the devicetree "compatible" and selects the chip variant from the
 * of_match_table match data, not from any silicon register. So, unlike the
 * TPS65217 model (whose CHIPID read at 0x00 was the one value that had to be
 * right), a byte-addressed read/write register file with power-on-zero
 * contents is byte-for-byte sufficient here: the unlock write is ACKed and
 * read back, and every IRQ status register reads back 0 (== "no interrupt
 * pending"), which is exactly what the driver wants at init. The datasheet's
 * regulator/sequencing defaults are not modelled because nothing in the guest
 * driver path validates them (the regulators are all "regulator-always-on" in
 * the Green Eco DT and are set up by the bootloader on real hardware).
 *
 * The register map is the regmap window drivers/mfd/tps65219.c declares:
 * reg_bits = 8, val_bits = 8, max_register = TPS65219_REG_FACTORY_CONFIG_2
 * (0x41) -- so registers 0x00..0x41 inclusive.
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
#include "hw/misc/tps65214.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qom/object.h"

/* Registers 0x00..0x41 (max_register = TPS65219_REG_FACTORY_CONFIG_2). */
#define TPS65214_NR_REGS    0x42

/* Register-file unlock, checked by tps65219_probe() for chip_id == TPS65214.
 * (include/linux/mfd/tps65219.h: TPS65214_REG_LOCK / TPS65214_LOCK_ACCESS_CMD.)
 * Not enforced here -- the model is a dumb register file, so the write is
 * simply ACKed and stored, which is all the driver needs. */
#define TPS65214_REG_LOCK           0x03
#define TPS65214_LOCK_ACCESS_CMD    0x5A

OBJECT_DECLARE_SIMPLE_TYPE(Tps65214State, TPS65214_PMU)

struct Tps65214State {
    /*< private >*/
    I2CSlave parent_obj;

    /*< public >*/
    uint8_t regs[TPS65214_NR_REGS];  /* register file */
    uint8_t ptr;                     /* current register pointer */
    uint8_t count;                   /* 0 => next byte selects the register */
};

static void tps65214_reset_enter(Object *obj, ResetType type)
{
    Tps65214State *s = TPS65214_PMU(obj);

    /*
     * Power-on-zero contents are sufficient: the driver validates no register
     * value, and 0 in every INT status register reads as "no pending IRQ",
     * which is what regmap-irq expects at init. See the file header.
     */
    memset(s->regs, 0, sizeof(s->regs));
    s->ptr = 0;
    s->count = 0;
}

static int tps65214_event(I2CSlave *i2c, enum i2c_event event)
{
    Tps65214State *s = TPS65214_PMU(i2c);

    s->count = 0;
    return 0;
}

static uint8_t tps65214_rx(I2CSlave *i2c)
{
    Tps65214State *s = TPS65214_PMU(i2c);
    uint8_t ret = 0xff;

    if (s->ptr < TPS65214_NR_REGS) {
        ret = s->regs[s->ptr];
    }
    /* Sequential reads auto-increment the register pointer. */
    s->ptr++;
    return ret;
}

static int tps65214_tx(I2CSlave *i2c, uint8_t data)
{
    Tps65214State *s = TPS65214_PMU(i2c);

    if (s->count == 0) {
        /* First byte of a write selects the register. */
        s->ptr = data;
        s->count++;
    } else {
        if (s->ptr < TPS65214_NR_REGS) {
            s->regs[s->ptr] = data;
        }
        s->ptr++;
    }
    return 0;
}

static const VMStateDescription vmstate_tps65214 = {
    .name = TYPE_TPS65214_PMU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, Tps65214State),
        VMSTATE_UINT8_ARRAY(regs, Tps65214State, TPS65214_NR_REGS),
        VMSTATE_UINT8(ptr, Tps65214State),
        VMSTATE_UINT8(count, Tps65214State),
        VMSTATE_END_OF_LIST()
    }
};

static void tps65214_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *isc = I2C_SLAVE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    rc->phases.enter = tps65214_reset_enter;
    dc->vmsd = &vmstate_tps65214;
    dc->desc = "TI TPS65214 PMIC";
    isc->event = tps65214_event;
    isc->recv = tps65214_rx;
    isc->send = tps65214_tx;
}

static const TypeInfo tps65214_info = {
    .name          = TYPE_TPS65214_PMU,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Tps65214State),
    .class_init    = tps65214_class_init,
};

static void tps65214_register_types(void)
{
    type_register_static(&tps65214_info);
}

type_init(tps65214_register_types)
