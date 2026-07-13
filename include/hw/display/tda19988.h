/*
 * NXP TDA19988 HDMI encoder (DRM bridge "nxp,tda998x") -- minimal I2C model.
 *
 * Reference: drivers/gpu/drm/bridge/tda998x_drv.c. On the BeagleBone Black the
 * encoder sits on I2C0 at 0x70 (paged register file) with a second, unpaged
 * register bank at the CEC address 0x34, and drives an HPD/EDID interrupt into
 * GPIO1 line 25 (interrupts-extended = <&gpio1 25 IRQ_TYPE_LEVEL_LOW>).
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

#ifndef HW_DISPLAY_TDA19988_H
#define HW_DISPLAY_TDA19988_H

#include "hw/i2c/i2c.h"
#include "hw/display/edid.h"
#include "qom/object.h"

#define TYPE_TDA19988 "tda19988"
OBJECT_DECLARE_SIMPLE_TYPE(TDA19988State, TDA19988)

#define TYPE_TDA19988_CEC "tda19988-cec"
OBJECT_DECLARE_SIMPLE_TYPE(TDA19988CecState, TDA19988_CEC)

/*
 * Main HDMI-address (0x70) front-end. Holds all shared device state: the
 * paged register access pointers, the fixed EDID blob, the pending-interrupt
 * flags and the single HPD/EDID interrupt output.
 */
struct TDA19988State {
    /*< private >*/
    I2CSlave parent_obj;

    /*< public >*/
    /* Paged-register transaction state (page<<8 | addr addressing). */
    uint8_t current_page;
    uint8_t addr_ptr;
    bool first;          /* next byte is the sub-address (or REG_CURPAGE) */
    bool page_select;    /* next byte selects the page (after a 0xff write) */

    /* One fixed-mode EDID served from page 0x09 offsets 0x00..0x7f. */
    uint8_t edid[128];
    qemu_edid_info edid_info;

    /* One-shot pending interrupt sources (see tda19988.c). */
    bool hpd_pending;
    bool edid_pending;
    bool edid_armed;     /* saw REG_EDID_CTRL <- 1, awaiting the <- 0 */

    /* HPD/EDID interrupt out to GPIO1 line 25 (idle high, asserted low). */
    qemu_irq irq;
};

/*
 * Secondary CEC-address (0x34) front-end. On real silicon this is the same die
 * answering a second bus address; here it is a thin I2C slave sharing the main
 * device's state via the "hdmi" QOM link.
 */
struct TDA19988CecState {
    /*< private >*/
    I2CSlave parent_obj;

    /*< public >*/
    TDA19988State *hdmi;
    uint8_t addr_ptr;
    bool first;
};

#endif /* HW_DISPLAY_TDA19988_H */
