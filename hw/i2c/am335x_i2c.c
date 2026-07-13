/*
 * TI AM335x I2C controller ("ti,omap4-i2c") emulation.
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * Master-only model of the AM335x I2C0 controller in the OMAP4+ "v2"
 * register layout (TRM spruh73q ch.21, base 0x44E0B000). The Linux
 * i2c-omap driver selects the v2 register map when I2C_REVNB_HI bits
 * [15:14] decode to SCHEME 1 (reset value 0x5040), which is
 * register-incompatible with the OMAP2/3 map implemented in
 * hw/i2c/omap_i2c.c -- hence a dedicated device.
 *
 * Two synthesized read-side behaviours make the probe/transfer path
 * complete without stalling:
 *
 *  - I2C_SYSS (0x90) bit0 RESETDONE always reads 1, so both the ti-sysc
 *    target-module softreset poll and the driver's own omap_i2c_reset()
 *    SYSS poll complete immediately (same idiom as the sibling WDT/GPIO
 *    devices; without it the module softreset stalls boot, cf. WDT1).
 *
 *  - The bus-busy (BB, bit12) and bus-free (BF, bit8) status bits, plus
 *    RRDY (bit3) and XRDY (bit4), are computed live from the internal
 *    transfer state on every I2C_IRQSTATUS[_RAW] read rather than stored.
 *    omap_i2c_wait_for_bb_valid()/omap_i2c_wait_for_bb() spin for up to
 *    OMAP_I2C_TIMEOUT (1s) unless they immediately observe BB or BF, and
 *    the driver force-invalidates its bb_valid cache after every softreset
 *    -- deriving BB=busy / BF=!busy guarantees those polls return on the
 *    first read regardless of reset history, avoiding a ~1s-per-transfer
 *    stall.
 *
 * A single 32-byte software FIFO is not modelled; the transfer engine is
 * byte-granular and signals RRDY/XRDY whenever a byte is available/space
 * exists, which is a strict superset of what the interrupt-driven driver
 * polls for.
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
#include "hw/i2c/am335x_i2c.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

/* Register offsets (v2 map; TRM spruh73q Table 21-8). */
#define I2C_REVNB_LO        0x00
#define I2C_REVNB_HI        0x04
#define I2C_SYSC            0x10
#define I2C_IRQSTATUS_RAW   0x24
#define I2C_IRQSTATUS       0x28
#define I2C_IRQENABLE_SET   0x2c
#define I2C_IRQENABLE_CLR   0x30
#define I2C_WE              0x34
#define I2C_SYSS            0x90
#define I2C_BUF             0x94
#define I2C_CNT             0x98
#define I2C_DATA            0x9c
#define I2C_CON             0xa4
#define I2C_OA              0xa8
#define I2C_SA              0xac
#define I2C_PSC             0xb0
#define I2C_SCLL            0xb4
#define I2C_SCLH            0xb8
#define I2C_SYSTEST         0xbc
#define I2C_BUFSTAT         0xc0

/* I2C_IRQSTATUS / IRQSTATUS_RAW bits. */
#define I2C_STAT_AL         (1 << 0)
#define I2C_STAT_NACK       (1 << 1)
#define I2C_STAT_ARDY       (1 << 2)
#define I2C_STAT_RRDY       (1 << 3)
#define I2C_STAT_XRDY       (1 << 4)
#define I2C_STAT_BF         (1 << 8)
#define I2C_STAT_BB         (1 << 12)

/* I2C_CON bits. */
#define I2C_CON_STT         (1 << 0)
#define I2C_CON_STP         (1 << 1)
#define I2C_CON_XSA         (1 << 8)
#define I2C_CON_TRX         (1 << 9)
#define I2C_CON_MST         (1 << 10)
#define I2C_CON_EN          (1 << 15)

/* I2C_SYSC / I2C_SYSS bits. */
#define I2C_SYSC_SRST       (1 << 1)
#define I2C_SYSS_RESETDONE  (1 << 0)

/*
 * Silicon reset values. REVNB_HI[15:14] must decode to SCHEME 1 so the
 * driver picks the v2 register map; the combined revision
 * (HI << 16 | LO) = 0x50400002 lands exactly on OMAP_I2C_REV_ON_4430_PLUS,
 * which disables the (OMAP2/3-only) I207 errata. BUFSTAT[15:14] = 2 reports
 * a 32-byte FIFO.
 */
#define I2C_REVNB_LO_VALUE  0x0002
#define I2C_REVNB_HI_VALUE  0x5040
#define I2C_BUFSTAT_VALUE   0x8000

/*
 * Live status word: the stored latched bits (ARDY/NACK/AL) plus the
 * transfer-state-derived BB/BF and RRDY/XRDY. See the file banner for why
 * these are synthesized rather than stored.
 */
static uint16_t am335x_i2c_stat(AM335xI2cState *s)
{
    uint16_t stat = s->irqstatus;

    if (s->busy) {
        stat |= I2C_STAT_BB;
        if (s->count_cur > 0) {
            stat |= s->recv ? I2C_STAT_RRDY : I2C_STAT_XRDY;
        }
    } else {
        stat |= I2C_STAT_BF;
    }
    return stat;
}

static void am335x_i2c_update_irq(AM335xI2cState *s)
{
    qemu_set_irq(s->irq, (am335x_i2c_stat(s) & s->irqenable) != 0);
}

/*
 * Finish the data phase of the current message: the byte counter has
 * reached zero. Real hardware clears MST automatically at the end of a
 * transfer; if a stop was requested, release the bus. When no stop was
 * requested (the first half of a register read: address write + repeated
 * start), the transfer stays open for the follow-up start.
 */
static void am335x_i2c_xfer_complete(AM335xI2cState *s)
{
    s->irqstatus |= I2C_STAT_ARDY;
    s->con &= ~I2C_CON_MST;
    if (s->con & I2C_CON_STP) {
        i2c_end_transfer(s->bus);
        s->con &= ~I2C_CON_STP;
        s->busy = false;
    }
}

/* Handle a write to I2C_CON that may arm a (repeated) start. */
static void am335x_i2c_con_write(AM335xI2cState *s, uint16_t value)
{
    bool recv;
    int nack;

    s->con = value;

    if (!(value & I2C_CON_EN)) {
        /*
         * Module disabled: clear the FIFO/status and abort any transfer.
         * CON/OA/SA/PSC/SCLL/SCLH are explicitly not reset by this (TRM
         * 21.4.1.19); the driver rewrites them before re-enabling.
         */
        if (s->busy) {
            i2c_end_transfer(s->bus);
        }
        s->busy = false;
        s->count_cur = 0;
        s->irqstatus = 0;
        am335x_i2c_update_irq(s);
        return;
    }

    /* Enable/config-only write (no start requested). */
    if (!(value & I2C_CON_STT)) {
        return;
    }

    if (!(value & I2C_CON_MST)) {
        qemu_log_mask(LOG_UNIMP, "%s: slave mode not supported\n",
                      TYPE_AM335X_I2C);
        s->con &= ~I2C_CON_STT;
        return;
    }
    if (value & I2C_CON_XSA) {
        qemu_log_mask(LOG_UNIMP, "%s: 10-bit addressing not supported\n",
                      TYPE_AM335X_I2C);
        s->con &= ~I2C_CON_STT;
        return;
    }

    /* STT self-clears once the start condition is generated (TRM). */
    s->con &= ~I2C_CON_STT;

    recv = !(value & I2C_CON_TRX);
    nack = i2c_start_transfer(s->bus, s->sa & 0x7f, recv);
    if (nack) {
        /*
         * No slave acknowledged the address. Real hardware ends the failed
         * access and raises ARDY together with NACK; the driver's ISR
         * treats a lone NACK (no other status bit) as -EAGAIN and never
         * completes the transfer, so ARDY must accompany it or every
         * transaction to an absent address stalls for OMAP_I2C_TIMEOUT
         * (1s). On this (initial) start the i2c core has already released
         * the bus, so the controller is idle.
         */
        s->irqstatus |= I2C_STAT_NACK | I2C_STAT_ARDY;
        s->busy = false;
        s->con &= ~I2C_CON_STP;
    } else {
        s->busy = true;
        s->recv = recv;
        s->count_cur = s->cnt;
        if (recv && s->count_cur > 0) {
            /* Pre-fetch the first byte so I2C_DATA has something to hand
             * back on the first read. */
            s->data = i2c_recv(s->bus);
        }
    }
    am335x_i2c_update_irq(s);
}

static uint64_t am335x_i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    AM335xI2cState *s = AM335X_I2C(opaque);
    uint8_t ret;

    switch (offset) {
    case I2C_REVNB_LO:
        return I2C_REVNB_LO_VALUE;
    case I2C_REVNB_HI:
        return I2C_REVNB_HI_VALUE;
    case I2C_SYSC:
        return s->sysc;
    case I2C_IRQSTATUS_RAW:
    case I2C_IRQSTATUS:
        return am335x_i2c_stat(s);
    case I2C_IRQENABLE_SET:
    case I2C_IRQENABLE_CLR:
        return s->irqenable;
    case I2C_WE:
        return s->we;
    case I2C_SYSS:
        /* Always report the OCP softreset as complete. */
        return I2C_SYSS_RESETDONE;
    case I2C_BUF:
        return s->buf;
    case I2C_CNT:
        return s->busy ? s->count_cur : s->cnt;
    case I2C_DATA:
        ret = s->data;
        if (s->busy && s->recv && s->count_cur > 0) {
            s->count_cur--;
            if (s->count_cur > 0) {
                s->data = i2c_recv(s->bus);
            } else {
                am335x_i2c_xfer_complete(s);
            }
            am335x_i2c_update_irq(s);
        }
        return ret;
    case I2C_CON:
        return s->con;
    case I2C_OA:
        return s->oa;
    case I2C_SA:
        return s->sa;
    case I2C_PSC:
        return s->psc;
    case I2C_SCLL:
        return s->scll;
    case I2C_SCLH:
        return s->sclh;
    case I2C_SYSTEST:
        return s->systest;
    case I2C_BUFSTAT:
        return I2C_BUFSTAT_VALUE;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_I2C, offset);
        return 0;
    }
}

static void am335x_i2c_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    AM335xI2cState *s = AM335X_I2C(opaque);
    uint16_t v = (uint16_t)value;

    switch (offset) {
    case I2C_REVNB_LO:
    case I2C_REVNB_HI:
    case I2C_SYSS:
    case I2C_BUFSTAT:
        /* read-only */
        break;
    case I2C_SYSC:
        /* SOFTRESET self-clears; SYSS.RESETDONE always reads 1. */
        s->sysc = v & ~I2C_SYSC_SRST;
        break;
    case I2C_IRQSTATUS_RAW:
        /* Write-1-to-set (diagnostic force). */
        s->irqstatus |= v;
        am335x_i2c_update_irq(s);
        break;
    case I2C_IRQSTATUS:
        /* Write-1-to-clear. Synthesized bits (BB/BF/RRDY/XRDY) are not
         * stored, so clearing them here is a harmless no-op. */
        s->irqstatus &= ~v;
        am335x_i2c_update_irq(s);
        break;
    case I2C_IRQENABLE_SET:
        s->irqenable |= v;
        am335x_i2c_update_irq(s);
        break;
    case I2C_IRQENABLE_CLR:
        s->irqenable &= ~v;
        am335x_i2c_update_irq(s);
        break;
    case I2C_WE:
        s->we = v;
        break;
    case I2C_BUF:
        /* RXFIFO_CLR/TXFIFO_CLR are self-clearing; keep only config bits. */
        s->buf = v & ~((1 << 14) | (1 << 6));
        break;
    case I2C_CNT:
        s->cnt = v;
        break;
    case I2C_DATA:
        if (s->busy && !s->recv && s->count_cur > 0) {
            i2c_send(s->bus, v & 0xff);
            s->count_cur--;
            if (s->count_cur == 0) {
                am335x_i2c_xfer_complete(s);
            }
            am335x_i2c_update_irq(s);
        }
        break;
    case I2C_CON:
        am335x_i2c_con_write(s, v);
        break;
    case I2C_OA:
        s->oa = v;
        break;
    case I2C_SA:
        s->sa = v & 0x3ff;
        break;
    case I2C_PSC:
        s->psc = v;
        break;
    case I2C_SCLL:
        s->scll = v;
        break;
    case I2C_SCLH:
        s->sclh = v;
        break;
    case I2C_SYSTEST:
        s->systest = v;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      TYPE_AM335X_I2C, offset);
        break;
    }
}

static const MemoryRegionOps am335x_i2c_ops = {
    .read = am335x_i2c_read,
    .write = am335x_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    /*
     * The i2c-omap driver uses 16-bit accesses for the functional
     * registers; the ti-sysc wrapper uses 32-bit accesses for
     * rev/sysc/syss. Accept both.
     */
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void am335x_i2c_reset(DeviceState *dev)
{
    AM335xI2cState *s = AM335X_I2C(dev);

    if (s->busy) {
        i2c_end_transfer(s->bus);
    }
    s->sysc = 0;
    s->irqstatus = 0;
    s->irqenable = 0;
    s->we = 0;
    s->buf = 0;
    s->cnt = 0;
    s->data = 0;
    s->con = 0;
    s->oa = 0;
    s->sa = 0;
    s->psc = 0;
    s->scll = 0;
    s->sclh = 0;
    s->systest = 0;
    s->busy = false;
    s->recv = false;
    s->count_cur = 0;

    am335x_i2c_update_irq(s);
}

static void am335x_i2c_init(Object *obj)
{
    AM335xI2cState *s = AM335X_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &am335x_i2c_ops, s,
                          TYPE_AM335X_I2C, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), NULL);
}

static const VMStateDescription vmstate_am335x_i2c = {
    .name = TYPE_AM335X_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(sysc, AM335xI2cState),
        VMSTATE_UINT16(irqstatus, AM335xI2cState),
        VMSTATE_UINT16(irqenable, AM335xI2cState),
        VMSTATE_UINT16(we, AM335xI2cState),
        VMSTATE_UINT16(buf, AM335xI2cState),
        VMSTATE_UINT16(cnt, AM335xI2cState),
        VMSTATE_UINT8(data, AM335xI2cState),
        VMSTATE_UINT16(con, AM335xI2cState),
        VMSTATE_UINT16(oa, AM335xI2cState),
        VMSTATE_UINT16(sa, AM335xI2cState),
        VMSTATE_UINT8(psc, AM335xI2cState),
        VMSTATE_UINT8(scll, AM335xI2cState),
        VMSTATE_UINT8(sclh, AM335xI2cState),
        VMSTATE_UINT16(systest, AM335xI2cState),
        VMSTATE_BOOL(busy, AM335xI2cState),
        VMSTATE_BOOL(recv, AM335xI2cState),
        VMSTATE_INT32(count_cur, AM335xI2cState),
        VMSTATE_END_OF_LIST()
    }
};

static void am335x_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, am335x_i2c_reset);
    dc->vmsd = &vmstate_am335x_i2c;
    dc->desc = "TI AM335x I2C controller";
}

static const TypeInfo am335x_i2c_info = {
    .name          = TYPE_AM335X_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AM335xI2cState),
    .instance_init = am335x_i2c_init,
    .class_init    = am335x_i2c_class_init,
};

static void am335x_i2c_register_types(void)
{
    type_register_static(&am335x_i2c_info);
}

type_init(am335x_i2c_register_types)

I2CBus *am335x_i2c_bus(DeviceState *dev)
{
    return AM335X_I2C(dev)->bus;
}
