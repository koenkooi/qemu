/*
 * NXP TDA19988 HDMI encoder (DRM bridge "nxp,tda998x") -- minimal I2C model.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Enough of the TDA19988 for the Linux tda998x driver to bind as a DRM
 * bridge on the BeagleBone Black, report a hotpluggable HDMI sink, serve a
 * multi-mode EDID so tilcdc picks a mode and scans out a framebuffer, and
 * for the companion tda9950 driver to register a working CEC adapter.
 * References are to drivers/gpu/drm/bridge/tda998x_drv.c unless noted.
 *
 * Two I2C addresses are modelled (as on real silicon):
 *
 *   - 0x70 (this device, TYPE_TDA19988): a paged register file. A write of
 *     sub-address REG_CURPAGE (0xff) selects the page; other accesses hit
 *     (page, addr) with an auto-incrementing address pointer. The driver only
 *     validates REG_VERSION_LSB/MSB (-> 0x0301 = TDA19988, tda998x_drv.c
 *     :1861-1898) and reads the EDID from page 0x09; everything else is
 *     accepted and discarded. The EDID itself comes from qemu_edid_generate()
 *     (hw/display/edid-generate.c) plus one hand-spliced entry -- see
 *     tda19988_edid_add_1280x720() -- and, with the default xmax/ymax=0
 *     (unclamped), already carries established timings (640x480, 800x600,
 *     1024x768) and up to eight EDID standard-timing entries including
 *     1920x1080@60; the preferred/detailed-timing mode stays 1024x768@60
 *     (the "xres"/"yres" properties) so the default boot resolution -- and
 *     thus the proven-working scanout -- is unchanged.
 *
 *   - 0x34 (TYPE_TDA19988_CEC): an unpaged bank sharing this device's state,
 *     answering TWO disjoint register ranges (as on real silicon, where the
 *     TDA9950 CEC core is the same die at the same bus address):
 *       - tda998x_drv.c's own HPD/CEC-status registers (0xee/0xfd/0xfe):
 *         tda998x_conn_detect() reads REG_CEC_RXSHPDLEV (HPD level, now
 *         reflecting the "connected" property below) and the threaded IRQ
 *         reads REG_CEC_INTSTATUS/RXSHPDINT.
 *       - the TDA9950 CEC command-processor "mailbox" (REG_CSR/CVR/CCR/
 *         ACKH/ACKL/CCONR/CDR0.., drivers/media/cec/i2c/tda9950.c): probed
 *         as a *separate* i2c_client at this same address sharing the
 *         primary device's IRQ (tda998x_drv.c:1971-1976), so the model
 *         reports a non-zero hardware version (REG_CVR) and completes any
 *         CEC transmit request written to REG_CDR0 instantly with a
 *         CDR1_CNF/CDR2_CNF_SUCCESS reply -- there is no other CEC-capable
 *         peer on this synthetic bus to arbitrate against or be NACKed by,
 *         so inbound messages (CDR1_IND) are not modelled; cec_register_
 *         adapter() succeeds and /dev/cecN appears with an empty RX side.
 *
 * The HPD/EDID/CEC interrupt (GPIO1 line 25, IRQ_TYPE_LEVEL_LOW) is NOT
 * optional: because the DT gives the node a valid virq, read_edid_block()
 * (tda998x_drv.c:1221-1283) takes the wait_event_timeout() path and hard-fails
 * with -ETIMEDOUT (reporting zero modes) unless the interrupt actually fires.
 * The model serves EDID instantly, so it sets edid_pending and asserts the
 * line the moment it sees the REG_EDID_CTRL 1->0 write pair; the guest's
 * threaded ISR then reads REG_INT_FLAGS_2 and wakes the EDID waiter. The same
 * line is shared with the tda9950 CEC driver's threaded IRQ (both request the
 * same underlying host IRQ number, IRQF_SHARED): tda998x_irq_thread() only
 * acts when REG_CEC_INTSTATUS.HDMI is set, so an interrupt raised purely for
 * a CEC mailbox reply (cec_pending) is correctly ignored (IRQ_NONE) by it,
 * and vice versa.
 *
 * Interrupt line polarity/idle: the line idles high and is driven low while
 * hpd_pending || edid_pending || cec_pending is set. It is left high at
 * reset -- unlike the datasheet's "cable-already-connected" latch we do NOT
 * pre-arm hpd_pending, because (a) tda998x_conn_detect() only reads the HPD
 * *level* register, so no initial HPD edge is needed for the default
 * connected=true state, and (b) the AM335x GPIO input model only latches on
 * a level *transition*, so holding the line low from reset would mask a
 * later falling edge (EDID, or a genuine post-boot disconnect).
 *
 * Hotplug: the "connected" QOM bool property (default true) models the HDMI
 * cable. It is a hand-written property (object_property_add_bool() in
 * tda19988_instance_init()), not a plain qdev Property, specifically so it
 * can be toggled at runtime -- e.g. "qom-set /machine/soc/hdmi connected
 * off" over HMP/QMP -- and have that raise a real HPD interrupt, the same
 * way "-device tda19988,connected=false" would at machine construction.
 * See tda19988_set_connected().
 *
 * HDMI audio: McASP0 (hw/audio/am335x_mcasp.c) and EDMA3 (hw/dma/am335x_edma.c)
 * now model enough of the DMA-driven I2S chain for davinci-mcasp to probe
 * cleanly and the DT's `simple-audio-card` ("TI BeagleBone Black") to
 * register -- confirmed via a live boot: `modprobe snd-soc-davinci-mcasp`
 * succeeds and /proc/asound/cards lists the card, with TDA19988 bound as the
 * ASoC codec DAI. Notably, THIS file needed no changes for that: the kernel's
 * tda998x_drv.c already implements its own ASoC codec-DAI glue in software
 * (probe/hw_params/etc. do not depend on any TDA19988 register beyond what is
 * already modelled for DRM/CEC above), so the codec side of the audio link
 * resolves for free once McASP0/EDMA3 exist. What remains structural/unverified:
 * no real I2S sample data is transported -- McASP0's "dat" data-port window
 * discards writes and reads zero (see am335x_mcasp.c), and EDMA3's
 * completion-interrupt cadence is a synthetic timer, not driven by any real
 * McASP FIFO/sample-clock event -- so nothing audible reaches this model's
 * (nonexistent) audio-in path, and an actual aplay/speaker-test playback
 * completing end-to-end was not exercised (the test rootfs used lacks
 * alsa-utils and there was no way to build/install it offline).
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

/* CEC (unpaged, at address 0x34): tda998x_drv.c's own HPD/CEC-status half. */
#define TDA_REG_CEC_INTSTATUS   0xee
#define CEC_INTSTATUS_HDMI      (1 << 1)
#define TDA_REG_CEC_RXSHPDINT   0xfd
#define CEC_RXSHPDINT_HPD       (1 << 1)
#define TDA_REG_CEC_RXSHPDLEV   0xfe
#define CEC_RXSHPDLEV_HPD       (1 << 1)

/* CEC (unpaged, at address 0x34): TDA9950 command-processor mailbox half
 * (drivers/media/cec/i2c/tda9950.c). Disjoint from the range above -- real
 * silicon answers both from the same die at the same bus address. */
#define TDA_REG_CSR             0x00
#define CSR_INT                 (1 << 6)
#define TDA_REG_CVR             0x02
#define TDA_REG_CCR             0x03
#define CCR_RESET               (1 << 7)
#define TDA_REG_ACKH            0x04
#define TDA_REG_ACKL            0x05
#define TDA_REG_CCONR           0x06
#define TDA_REG_CDR0            0x07

#define CDR1_REQ                0x00    /* host -> device: transmit request */
#define CDR1_CNF                0x01    /* device -> host: transmit result */
#define CDR2_CNF_SUCCESS        0x00

/* Arbitrary but non-zero "hardware version 1.1" (tda9950_probe() logs
 * cvr>>4 . cvr&15); no public datasheet revision to match against, the
 * driver only requires the read to succeed and prints whatever it gets. */
#define TDA9950_HW_VERSION      0x11

/* Version identifying this part as a TDA19988 (rev = lsb | msb<<8 = 0x0301). */
#define TDA19988_VERSION_LSB    0x01
#define TDA19988_VERSION_MSB    0x03

/* EDID standard-timing table (bytes 38..53, 8 slots of 2 bytes each). */
#define EDID_STD_TIMING_START   38
#define EDID_STD_TIMING_SLOTS   8

static void tda19988_update_irq(TDA19988State *s)
{
    /* IRQ_TYPE_LEVEL_LOW: asserted == line driven low. Shared with the
     * tda9950 CEC driver, which requests this same host IRQ number. */
    bool active = s->hpd_pending || s->edid_pending || s->cec_pending;

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

static uint8_t tda19988_cec_read(TDA19988CecState *c, uint8_t addr)
{
    TDA19988State *s = c->hdmi;

    switch (addr) {
    /* tda998x_drv.c's own HPD/CEC-status registers. */
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
        return s->connected ? CEC_RXSHPDLEV_HPD : 0;    /* live HPD level */

    /* TDA9950 mailbox registers. */
    case TDA_REG_CSR: {
        /* Read-to-clear, same idiom as REG_INT_FLAGS_2/RXSHPDINT above:
         * releases the (shared) interrupt line once the host has noticed
         * a message is ready. */
        uint8_t v = s->cec_pending ? CSR_INT : 0;

        s->cec_pending = false;
        tda19988_update_irq(s);
        return v;
    }
    case TDA_REG_CVR:
        return TDA9950_HW_VERSION;
    case TDA_REG_CCR:
        return c->ccr;
    case TDA_REG_ACKH:
        return c->log_addrs >> 8;
    case TDA_REG_ACKL:
        return c->log_addrs & 0xff;
    case TDA_REG_CCONR:
        return c->cconr;
    default:
        if (addr >= TDA_REG_CDR0 &&
            addr - TDA_REG_CDR0 < sizeof(c->mailbox)) {
            return c->mailbox[addr - TDA_REG_CDR0];
        }
        return 0;
    }
}

static void tda19988_cec_write(TDA19988CecState *c, uint8_t addr,
                               uint8_t data)
{
    switch (addr) {
    case TDA_REG_CCR:
        c->ccr = data;    /* CCR_RESET/CCR_ON: no separate state machine
                            * needed -- the mailbox is always ready. */
        break;
    case TDA_REG_ACKH:
        c->log_addrs = (c->log_addrs & 0x00ff) | ((uint16_t)data << 8);
        break;
    case TDA_REG_ACKL:
        c->log_addrs = (c->log_addrs & 0xff00) | data;
        break;
    case TDA_REG_CCONR:
        c->cconr = data;
        break;
    default:
        if (addr >= TDA_REG_CDR0 &&
            addr - TDA_REG_CDR0 < sizeof(c->mailbox)) {
            c->mailbox[addr - TDA_REG_CDR0] = data;
        }
        /* CSR/CVR are read-only; calibration/enable-mask registers above
         * 0xf0 are sinks, as before. */
        break;
    }
}

static int tda19988_cec_event(I2CSlave *i2c, enum i2c_event event)
{
    TDA19988CecState *c = TDA19988_CEC(i2c);

    switch (event) {
    case I2C_START_SEND:
        c->first = true;
        c->mailbox_write = false;
        break;
    case I2C_FINISH:
        /*
         * tda9950_cec_transmit() writes REG_CDR0.. as a single contiguous
         * transaction: [len][CDR1_REQ][msg bytes...]. There is no other
         * CEC-capable peer on this synthetic bus to arbitrate against or
         * be NACKed by, so model every request as transmitted and acked
         * instantly (mirrors how edid_pending is served instantly, see
         * the file header) by overwriting the mailbox with a CDR1_CNF/
         * CDR2_CNF_SUCCESS reply and raising the shared interrupt.
         */
        if (c->mailbox_write && c->mailbox[1] == CDR1_REQ) {
            c->mailbox[0] = 3;
            c->mailbox[1] = CDR1_CNF;
            c->mailbox[2] = CDR2_CNF_SUCCESS;
            memset(&c->mailbox[3], 0, sizeof(c->mailbox) - 3);
            c->hdmi->cec_pending = true;
            tda19988_update_irq(c->hdmi);
        }
        c->mailbox_write = false;
        break;
    default:
        break;
    }
    return 0;
}

static int tda19988_cec_tx(I2CSlave *i2c, uint8_t data)
{
    TDA19988CecState *c = TDA19988_CEC(i2c);

    if (c->first) {
        c->addr_ptr = data;
        c->first = false;
        c->mailbox_write = (data == TDA_REG_CDR0);
        return 0;
    }

    tda19988_cec_write(c, c->addr_ptr, data);
    c->addr_ptr++;
    return 0;
}

static uint8_t tda19988_cec_rx(I2CSlave *i2c)
{
    TDA19988CecState *c = TDA19988_CEC(i2c);
    uint8_t v = tda19988_cec_read(c, c->addr_ptr);

    c->addr_ptr++;
    return v;
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
    s->cec_pending = false;
    /* s->connected is a wire state (the cable), not a register -- it must
     * survive a SoC reset, so it is deliberately left untouched here. */
}

static void tda19988_reset_hold(Object *obj, ResetType type)
{
    TDA19988State *s = TDA19988(obj);

    /* Idle the interrupt line high (see file header). */
    qemu_set_irq(s->irq, 1);
}

/*
 * qemu_edid_generate() already fills in established timings (640x480,
 * 800x600, 1024x768) and, with xmax/ymax left at 0 (unclamped, the
 * default), up to eight EDID "standard timing" entries from its built-in
 * table -- for this device's default properties that list already includes
 * 1920x1080@60 among others (hw/display/edid-generate.c:modes[]). That
 * table has no entry for 1280x720 though, a common CEA/HDMI mode plenty of
 * real displays advertise, so splice one in by hand: reuse an empty
 * standard-timing slot if xmax/ymax trimmed the list down, otherwise evict
 * the lowest-priority (last-filled) slot, then redo the base-block
 * checksum qemu_edid_generate() already wrote.
 */
static void tda19988_edid_checksum(uint8_t *edid)
{
    uint32_t sum = 0;
    int i;

    for (i = 0; i < 127; i++) {
        sum += edid[i];
    }
    sum &= 0xff;
    edid[127] = sum ? 0x100 - sum : 0;
}

static void tda19988_edid_add_1280x720(uint8_t *edid)
{
    int slot = EDID_STD_TIMING_SLOTS - 1;
    int i;
    uint8_t *e;

    for (i = 0; i < EDID_STD_TIMING_SLOTS; i++) {
        e = &edid[EDID_STD_TIMING_START + i * 2];
        if (e[0] == 0x01 && e[1] == 0x01) {
            slot = i;    /* empty placeholder: prefer this over evicting */
            break;
        }
    }

    e = &edid[EDID_STD_TIMING_START + slot * 2];
    e[0] = (1280 / 8) - 31;         /* xres field */
    e[1] = (3 << 6) | (60 - 60);    /* aspect 16:9, refresh 60Hz */

    tda19988_edid_checksum(edid);
}

static bool tda19988_get_connected(Object *obj, Error **errp)
{
    return TDA19988(obj)->connected;
}

static void tda19988_set_connected(Object *obj, bool value, Error **errp)
{
    TDA19988State *s = TDA19988(obj);

    if (s->connected == value) {
        return;
    }
    s->connected = value;

    /* Mirror a real plug/unplug: latch an HPD edge and drive it out, the
     * same handshake tda998x_irq_thread() expects (RXSHPDINT then
     * RXSHPDLEV). Skip this before realize (qemu_irq doesn't exist yet) --
     * e.g. when "-device tda19988,connected=false" sets the property while
     * the machine is still being constructed. */
    if (DEVICE(obj)->realized) {
        s->hpd_pending = true;
        tda19988_update_irq(s);
    }
}

static void tda19988_instance_init(Object *obj)
{
    TDA19988State *s = TDA19988(obj);

    s->connected = true;
    object_property_add_bool(obj, "connected", tda19988_get_connected,
                             tda19988_set_connected);
    object_property_set_description(obj, "connected",
        "HDMI sink present (cable plugged in). Toggling at runtime, e.g. "
        "via qom-set, fires an HPD interrupt exactly like a real "
        "connect/disconnect.");
}

static void tda19988_realize(DeviceState *dev, Error **errp)
{
    TDA19988State *s = TDA19988(dev);

    qemu_edid_generate(s->edid, sizeof(s->edid), &s->edid_info);
    tda19988_edid_add_1280x720(s->edid);
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
        VMSTATE_BOOL(cec_pending, TDA19988State),
        VMSTATE_BOOL(connected, TDA19988State),
        VMSTATE_END_OF_LIST()
    }
};

/*
 * refresh_rate is in milli-Hz for qemu_edid_generate(). xmax/ymax (default
 * 0 = unclamped, matching hw/display/edid.h's DEFINE_EDID_PROPERTIES field
 * names) let an operator trim the established/standard-timing list
 * qemu_edid_generate() fills in below the preferred mode, e.g.
 * "-device tda19988,xmax=1280,ymax=720" to cap it at a specific target
 * display. The "connected" hotplug property is registered by hand in
 * tda19988_instance_init() instead of here, since toggling it needs to run
 * code (an HPD interrupt), not just store a value.
 */
static const Property tda19988_properties[] = {
    DEFINE_PROP_UINT32("xres", TDA19988State, edid_info.prefx, 1024),
    DEFINE_PROP_UINT32("yres", TDA19988State, edid_info.prefy, 768),
    DEFINE_PROP_UINT32("refresh_rate", TDA19988State,
                       edid_info.refresh_rate, 60000),
    DEFINE_PROP_UINT32("xmax", TDA19988State, edid_info.maxx, 0),
    DEFINE_PROP_UINT32("ymax", TDA19988State, edid_info.maxy, 0),
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
    .instance_init = tda19988_instance_init,
    .class_init    = tda19988_class_init,
};

static void tda19988_cec_reset_enter(Object *obj, ResetType type)
{
    TDA19988CecState *c = TDA19988_CEC(obj);

    c->addr_ptr = 0;
    c->first = false;
    c->mailbox_write = false;
    c->ccr = 0;
    c->log_addrs = 0;
    c->cconr = 0;
    memset(c->mailbox, 0, sizeof(c->mailbox));
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
        VMSTATE_BOOL(mailbox_write, TDA19988CecState),
        VMSTATE_UINT8(ccr, TDA19988CecState),
        VMSTATE_UINT16(log_addrs, TDA19988CecState),
        VMSTATE_UINT8(cconr, TDA19988CecState),
        VMSTATE_UINT8_ARRAY(mailbox, TDA19988CecState, 19),
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
