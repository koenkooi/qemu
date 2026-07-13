/*
 * TI TPS65217C Power Management IC emulation (I2C slave).
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Minimal register-file model of the TPS65217C PMIC as found on the
 * BeagleBone Black (I2C0, 7-bit address 0x24). Reset values follow the TI
 * datasheet SLVSB64I "Register Address Map" (registers 0x00..0x1E).
 *
 * The only access the Linux mfd driver (drivers/mfd/tps65217.c) hard-depends
 * on is a successful read of CHIPID at register 0x00: if that transaction
 * NAKs, tps65217_probe() fails and no child device (regulators, charger,
 * backlight, power button) is ever registered. The CHIPID *value* is never
 * validated -- 0xE2 identifies the TPS65217C variant (CHIP=0xE, REV=0x2) and
 * makes the driver log "TPS65217 ID 0xe version 1.2".
 *
 * The datasheet's password-protected write protocol (registers 0x0B..0x1E)
 * is intentionally not enforced: the driver is the party that performs the
 * PASSWORD unlock dance around each protected write, so a dumb
 * read-back-what-was-written register file is byte-for-byte compatible with
 * far less code.
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
#include "hw/misc/tps65217.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TPS65217_NR_REGS    0x1f    /* registers 0x00..0x1e */

/* STATUS (0x0A) power-present bits. */
#define TPS65217_STATUS_ACPWR   (1 << 3)
#define TPS65217_STATUS_USBPWR  (1 << 2)

OBJECT_DECLARE_SIMPLE_TYPE(Tps65217State, TPS65217_PMU)

struct Tps65217State {
    /*< private >*/
    I2CSlave parent_obj;

    /*< public >*/
    uint8_t regs[TPS65217_NR_REGS];  /* register file */
    uint8_t ptr;                     /* current register pointer */
    uint8_t count;                   /* 0 => next byte selects the register */
};

static void tps65217_reset_enter(Object *obj, ResetType type)
{
    Tps65217State *s = TPS65217_PMU(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->ptr = 0;
    s->count = 0;

    /* Datasheet SLVSB64I reset values. "X" (OTP/board-specific) registers
     * are seeded with an arbitrary in-range selector. */
    s->regs[0x00] = 0xE2;  /* CHIPID: TPS65217C (CHIP 0xE, REV 0x2 = v1.2) */
    s->regs[0x01] = 0x3D;  /* PPATH */
    s->regs[0x02] = 0x80;  /* INT: power-button mask set at reset */
    s->regs[0x03] = 0x00;  /* CHGCONFIG0 */
    s->regs[0x04] = 0xB1;  /* CHGCONFIG1 */
    s->regs[0x05] = 0x80;  /* CHGCONFIG2 */
    s->regs[0x06] = 0xB2;  /* CHGCONFIG3 */
    s->regs[0x07] = 0xB1;  /* WLEDCTRL1 */
    s->regs[0x08] = 0x00;  /* WLEDCTRL2 */
    s->regs[0x09] = 0x00;  /* MUXCTRL */
    /* STATUS: report AC+USB present so the charger cell sees valid input. */
    s->regs[0x0A] = TPS65217_STATUS_ACPWR | TPS65217_STATUS_USBPWR;
    s->regs[0x0B] = 0x00;  /* PASSWORD */
    s->regs[0x0C] = 0x00;  /* PGOOD */
    s->regs[0x0D] = 0x0C;  /* DEFPG */
    s->regs[0x0E] = 0x08;  /* DEFDCDC1 (X) */
    s->regs[0x0F] = 0x08;  /* DEFDCDC2 vdd_mpu (X) */
    s->regs[0x10] = 0x08;  /* DEFDCDC3 */
    s->regs[0x11] = 0x06;  /* DEFSLEW */
    s->regs[0x12] = 0x09;  /* DEFLDO1 */
    s->regs[0x13] = 0x38;  /* DEFLDO2 */
    s->regs[0x14] = 0x00;  /* DEFLS1 (X) */
    s->regs[0x15] = 0x00;  /* DEFLS2 (X) */
    s->regs[0x16] = 0x00;  /* ENABLE */
    s->regs[0x18] = 0x03;  /* DEFUVLO */
    s->regs[0x19] = 0x00;  /* SEQ1 (X) */
    s->regs[0x1A] = 0x00;  /* SEQ2 (X) */
    s->regs[0x1B] = 0x00;  /* SEQ3 (X) */
    s->regs[0x1C] = 0x40;  /* SEQ4 */
    s->regs[0x1D] = 0x00;  /* SEQ5 (X) */
    s->regs[0x1E] = 0x00;  /* SEQ6 */
}

static int tps65217_event(I2CSlave *i2c, enum i2c_event event)
{
    Tps65217State *s = TPS65217_PMU(i2c);

    s->count = 0;
    return 0;
}

static uint8_t tps65217_rx(I2CSlave *i2c)
{
    Tps65217State *s = TPS65217_PMU(i2c);
    uint8_t ret = 0xff;

    if (s->ptr < TPS65217_NR_REGS) {
        ret = s->regs[s->ptr];
    }
    /* Sequential reads auto-increment the register pointer. */
    s->ptr++;
    return ret;
}

static int tps65217_tx(I2CSlave *i2c, uint8_t data)
{
    Tps65217State *s = TPS65217_PMU(i2c);

    if (s->count == 0) {
        /* First byte of a write selects the register. */
        s->ptr = data;
        s->count++;
    } else {
        if (s->ptr < TPS65217_NR_REGS) {
            s->regs[s->ptr] = data;
        }
        s->ptr++;
    }
    return 0;
}

static const VMStateDescription vmstate_tps65217 = {
    .name = TYPE_TPS65217_PMU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, Tps65217State),
        VMSTATE_UINT8_ARRAY(regs, Tps65217State, TPS65217_NR_REGS),
        VMSTATE_UINT8(ptr, Tps65217State),
        VMSTATE_UINT8(count, Tps65217State),
        VMSTATE_END_OF_LIST()
    }
};

static void tps65217_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *isc = I2C_SLAVE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    rc->phases.enter = tps65217_reset_enter;
    dc->vmsd = &vmstate_tps65217;
    dc->desc = "TI TPS65217C PMIC";
    isc->event = tps65217_event;
    isc->recv = tps65217_rx;
    isc->send = tps65217_tx;
}

static const TypeInfo tps65217_info = {
    .name          = TYPE_TPS65217_PMU,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Tps65217State),
    .class_init    = tps65217_class_init,
};

static void tps65217_register_types(void)
{
    type_register_static(&tps65217_info);
}

type_init(tps65217_register_types)
