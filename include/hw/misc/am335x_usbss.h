/*
 * TI AM335x USB Subsystem (USBSS) "clean probe" stub.
 *
 * Reference: TRM spruh73q chapter 16 "Universal Serial Bus", base
 * 0x47400000, 32KB window (am33xx.dtsi target-module@47400000). Kernel
 * driver: drivers/usb/musb/musb_dsps.c (glue) + drivers/usb/musb/musb_core.c
 * (Mentor musb-hdrc core), matched against DT compatible "ti,musb-am33xx".
 *
 * Scope: this model exists solely to let the musb-hdrc/musb_dsps driver
 * *probe cleanly* for both USB0 and USB1 -- i.e. musb_init_controller()
 * completes and the controller registers a host or gadget instance --
 * without modelling any actual USB device enumeration, transfer, or the
 * CPPI4.1 DMA engine. See am335x_usbss.c for the register-by-register
 * rationale.
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
#include "qom/object.h"

#define TYPE_AM335X_USBSS "am335x-usbss"
OBJECT_DECLARE_SIMPLE_TYPE(AM335xUsbssState, AM335X_USBSS)

/* Size of the whole "usb" target-module window (am33xx.dtsi ranges
 * property: <0x0 0x47400000 0x8000>). Covers, at their real sub-offsets,
 * the ti-sysc wrapper (0x0000), USB0 control+mc (0x1000/0x1400), USB0 PHY
 * (0x1300, unused by any Linux driver), USB1 control+mc (0x1800/0x1c00),
 * USB1 PHY (0x1b00, likewise unused) and the CPPI4.1 DMA glue/queue
 * manager (0x2000-0x7fff, out of scope -- see am335x_usbss.c). */
#define AM335X_USBSS_SIZE 0x8000

struct AM335xUsbssState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;

    /*
     * Flat, byte-addressable backing store for the whole window,
     * allocated at realize. musb-hdrc/musb_dsps mixes 8/16/32-bit
     * accesses within the same register block (FADDR/POWER are 8-bit,
     * INTRTX/INTRRX are 16-bit, the DSPS wrapper registers are 32-bit,
     * ...), unlike the other flat-store am335x models in this tree
     * (am335x_control.c, am335x_wdt.c) which are 32-bit-only, hence
     * byte instead of uint32_t granularity here. Reads/writes hit this
     * array directly except for the handful of synthesized/read-only
     * registers handled in am335x_usbss.c (the two USBnREV wrapper
     * revisions, the two CONFIGDATA bytes, and the self-clearing
     * SOFT_RESET/SOFTRESET bits).
     */
    uint8_t *regs;
};

#endif /* HW_MISC_AM335X_USBSS_H */
