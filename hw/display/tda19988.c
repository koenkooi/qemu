/*
 * NXP TDA19988 HDMI encoder (DRM bridge "nxp,tda998x") -- minimal I2C model.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Just enough of the TDA19988 for the Linux tda998x driver to bind as a DRM
 * bridge on the BeagleBone Black, report an always-connected HDMI sink, and
 * serve one fixed-mode EDID so tilcdc picks a mode and scans out a
 * framebuffer. All references are to drivers/gpu/drm/bridge/tda998x_drv.c.
 *
 * Two I2C addresses are modelled (as on real silicon):
 *
 *   - 0x70 (this device, TYPE_TDA19988): a paged register file. A write of
 *     sub-address REG_CURPAGE (0xff) selects the page; other accesses hit
 *     (page, addr) with an auto-incrementing address pointer. The driver only
 *     validates REG_VERSION_LSB/MSB (-> 0x0301 = TDA19988, tda998x_drv.c
 *     :1861-1898) and reads the 128-byte EDID from page 0x09; everything else
 *     is accepted and discarded.
 *
 *   - 0x34 (TYPE_TDA19988_CEC): an unpaged bank sharing this device's state.
 *     tda998x_conn_detect() reads REG_CEC_RXSHPDLEV here (HPD level, always
 *     connected) and the threaded IRQ reads REG_CEC_INTSTATUS/RXSHPDINT.
 *
 * The HPD/EDID interrupt (GPIO1 line 25, IRQ_TYPE_LEVEL_LOW) is NOT optional:
 * because the DT gives the node a valid virq, read_edid_block()
 * (tda998x_drv.c:1221-1283) takes the wait_event_timeout() path and hard-fails
 * with -ETIMEDOUT (reporting zero modes) unless the interrupt actually fires.
 * The model serves EDID instantly, so it sets edid_pending and asserts the
 * line the moment it sees the REG_EDID_CTRL 1->0 write pair; the guest's
 * threaded ISR then reads REG_INT_FLAGS_2 and wakes the EDID waiter.
 *
 * Interrupt line polarity/idle: the line idles high and is driven low while
 * REG_CEC_INTSTATUS.HDMI (hpd_pending || edid_pending) is set. It is left high
 * at reset -- unlike the datasheet's "cable-already-connected" latch we do NOT
 * pre-arm hpd_pending, because (a) tda998x_conn_detect() only reads the HPD
 * *level* register (always connected here), so no initial HPD edge is needed,
 * and (b) the AM335x GPIO input model only latches on a level *transition*, so
 * holding the line low from reset would mask the later EDID falling edge.
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
#include "hw/display/tda19988.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qom/object.h"

/* Sub-addresses (tda998x_drv.c). Paged registers are page<<8 | addr; the
 * constants below are the raw sub-addresses used on the wire. */
#define TDA_REG_CURPAGE         0xff
#define TDA_REG_VERSION_LSB     0x00    /* page 0x00 */
#define TDA_REG_VERSION_MSB     0x02    /* page 0x00 */
#define TDA_REG_INT_FLAGS_2     0x11    /* page 0x00 */
#define INT_FLAGS_2_EDID_BLK_RD (1 << 1)
#define TDA_REG_EDID_CTRL       0xfa    /* page 0x09 */

/* CEC (unpaged, at address 0x34). */
#define TDA_REG_CEC_INTSTATUS   0xee
#define CEC_INTSTATUS_HDMI      (1 << 1)
#define TDA_REG_CEC_RXSHPDINT   0xfd
#define CEC_RXSHPDINT_HPD       (1 << 1)
#define TDA_REG_CEC_RXSHPDLEV   0xfe
#define CEC_RXSHPDLEV_HPD       (1 << 1)

/* Version identifying this part as a TDA19988 (rev = lsb | msb<<8 = 0x0301). */
#define TDA19988_VERSION_LSB    0x01
#define TDA19988_VERSION_MSB    0x03

static void tda19988_update_irq(TDA19988State *s)
{
    /* IRQ_TYPE_LEVEL_LOW: asserted == line driven low. */
    bool active = s->hpd_pending || s->edid_pending;

    qemu_set_irq(s->irq, active ? 0 : 1);
}

/* ---- HDMI address (0x70), paged register file ---- */

static uint8_t tda19988_reg_read(TDA19988State *s, uint8_t page, uint8_t addr)
{
    if (page == 0x00) {
        switch (addr) {
        case TDA_REG_VERSION_LSB:
            return TDA19988_VERSION_LSB;
        case TDA_REG_VERSION_MSB:
            return TDA19988_VERSION_MSB;
        case TDA_REG_INT_FLAGS_2: {
            uint8_t v = s->edid_pending ? INT_FLAGS_2_EDID_BLK_RD : 0;

            /* Read-to-clear; releases the interrupt line. */
            s->edid_pending = false;
            tda19988_update_irq(s);
            return v;
        }
        default:
            return 0;
        }
    }
    if (page == 0x09 && addr < sizeof(s->edid)) {
        return s->edid[addr];
    }
    return 0;
}

static void tda19988_reg_write(TDA19988State *s, uint8_t page, uint8_t addr,
                               uint8_t data)
{
    if (page == 0x09 && addr == TDA_REG_EDID_CTRL) {
        if (data & 0x1) {
            s->edid_armed = true;
        } else if (s->edid_armed) {
            /* The 1->0 write pair triggers the (instant) EDID block read. */
            s->edid_armed = false;
            s->edid_pending = true;
            tda19988_update_irq(s);
        }
    }
    /* Every other register is a sink. */
}

static int tda19988_event(I2CSlave *i2c, enum i2c_event event)
{
    TDA19988State *s = TDA19988(i2c);

    if (event == I2C_START_SEND) {
        s->first = true;
    }
    return 0;
}

static int tda19988_tx(I2CSlave *i2c, uint8_t data)
{
    TDA19988State *s = TDA19988(i2c);

    if (s->first) {
        s->first = false;
        if (data == TDA_REG_CURPAGE) {
            s->page_select = true;
        } else {
            s->addr_ptr = data;
        }
        return 0;
    }

    if (s->page_select) {
        s->current_page = data;
        s->page_select = false;
    } else {
        tda19988_reg_write(s, s->current_page, s->addr_ptr, data);
        s->addr_ptr++;
    }
    return 0;
}

static uint8_t tda19988_rx(I2CSlave *i2c)
{
    TDA19988State *s = TDA19988(i2c);
    uint8_t v = tda19988_reg_read(s, s->current_page, s->addr_ptr);

    s->addr_ptr++;
    return v;
}

/* ---- CEC address (0x34), unpaged bank sharing TDA19988State ---- */

static uint8_t tda19988_cec_read(TDA19988State *s, uint8_t addr)
{
    switch (addr) {
    case TDA_REG_CEC_INTSTATUS:
        /* Gates the whole threaded ISR; no read-to-clear here. */
        return (s->hpd_pending || s->edid_pending) ? CEC_INTSTATUS_HDMI : 0;
    case TDA_REG_CEC_RXSHPDINT: {
        uint8_t v = s->hpd_pending ? CEC_RXSHPDINT_HPD : 0;

        s->hpd_pending = false;
        tda19988_update_irq(s);
        return v;
    }
    case TDA_REG_CEC_RXSHPDLEV:
        return CEC_RXSHPDLEV_HPD;    /* live HPD level: always connected */
    default:
        return 0;
    }
}

static int tda19988_cec_event(I2CSlave *i2c, enum i2c_event event)
{
    TDA19988CecState *c = TDA19988_CEC(i2c);

    if (event == I2C_START_SEND) {
        c->first = true;
    }
    return 0;
}

static int tda19988_cec_tx(I2CSlave *i2c, uint8_t data)
{
    TDA19988CecState *c = TDA19988_CEC(i2c);

    if (c->first) {
        c->addr_ptr = data;
        c->first = false;
    }
    /* CEC-address register writes (calibration, enable masks) are all sinks. */
    return 0;
}

static uint8_t tda19988_cec_rx(I2CSlave *i2c)
{
    TDA19988CecState *c = TDA19988_CEC(i2c);

    return tda19988_cec_read(c->hdmi, c->addr_ptr);
}

/* ---- QOM plumbing ---- */

static void tda19988_reset_enter(Object *obj, ResetType type)
{
    TDA19988State *s = TDA19988(obj);

    s->current_page = 0xff;
    s->addr_ptr = 0;
    s->first = false;
    s->page_select = false;
    s->hpd_pending = false;
    s->edid_pending = false;
    s->edid_armed = false;
}

static void tda19988_reset_hold(Object *obj, ResetType type)
{
    TDA19988State *s = TDA19988(obj);

    /* Idle the interrupt line high (see file header). */
    qemu_set_irq(s->irq, 1);
}

static void tda19988_realize(DeviceState *dev, Error **errp)
{
    TDA19988State *s = TDA19988(dev);

    qemu_edid_generate(s->edid, sizeof(s->edid), &s->edid_info);
    qdev_init_gpio_out(dev, &s->irq, 1);
}

static const VMStateDescription vmstate_tda19988 = {
    .name = TYPE_TDA19988,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, TDA19988State),
        VMSTATE_UINT8(current_page, TDA19988State),
        VMSTATE_UINT8(addr_ptr, TDA19988State),
        VMSTATE_BOOL(first, TDA19988State),
        VMSTATE_BOOL(page_select, TDA19988State),
        VMSTATE_BOOL(hpd_pending, TDA19988State),
        VMSTATE_BOOL(edid_pending, TDA19988State),
        VMSTATE_BOOL(edid_armed, TDA19988State),
        VMSTATE_END_OF_LIST()
    }
};

/* refresh_rate is in milli-Hz for qemu_edid_generate(). */
static const Property tda19988_properties[] = {
    DEFINE_PROP_UINT32("xres", TDA19988State, edid_info.prefx, 1024),
    DEFINE_PROP_UINT32("yres", TDA19988State, edid_info.prefy, 768),
    DEFINE_PROP_UINT32("refresh_rate", TDA19988State,
                       edid_info.refresh_rate, 60000),
};

static void tda19988_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *isc = I2C_SLAVE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    rc->phases.enter = tda19988_reset_enter;
    rc->phases.hold = tda19988_reset_hold;
    dc->realize = tda19988_realize;
    dc->vmsd = &vmstate_tda19988;
    dc->desc = "NXP TDA19988 HDMI encoder";
    device_class_set_props(dc, tda19988_properties);
    isc->event = tda19988_event;
    isc->recv = tda19988_rx;
    isc->send = tda19988_tx;
}

static const TypeInfo tda19988_info = {
    .name          = TYPE_TDA19988,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TDA19988State),
    .class_init    = tda19988_class_init,
};

static void tda19988_cec_reset_enter(Object *obj, ResetType type)
{
    TDA19988CecState *c = TDA19988_CEC(obj);

    c->addr_ptr = 0;
    c->first = false;
}

static void tda19988_cec_realize(DeviceState *dev, Error **errp)
{
    TDA19988CecState *c = TDA19988_CEC(dev);

    if (!c->hdmi) {
        error_setg(errp, "'hdmi' link property was not set");
        return;
    }
}

static const VMStateDescription vmstate_tda19988_cec = {
    .name = TYPE_TDA19988_CEC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, TDA19988CecState),
        VMSTATE_UINT8(addr_ptr, TDA19988CecState),
        VMSTATE_BOOL(first, TDA19988CecState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property tda19988_cec_properties[] = {
    DEFINE_PROP_LINK("hdmi", TDA19988CecState, hdmi, TYPE_TDA19988,
                     TDA19988State *),
};

static void tda19988_cec_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *isc = I2C_SLAVE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    rc->phases.enter = tda19988_cec_reset_enter;
    dc->realize = tda19988_cec_realize;
    dc->vmsd = &vmstate_tda19988_cec;
    dc->desc = "NXP TDA19988 CEC front-end";
    device_class_set_props(dc, tda19988_cec_properties);
    isc->event = tda19988_cec_event;
    isc->recv = tda19988_cec_rx;
    isc->send = tda19988_cec_tx;
}

static const TypeInfo tda19988_cec_info = {
    .name          = TYPE_TDA19988_CEC,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TDA19988CecState),
    .class_init    = tda19988_cec_class_init,
};

static void tda19988_register_types(void)
{
    type_register_static(&tda19988_info);
    type_register_static(&tda19988_cec_info);
}

type_init(tda19988_register_types)
