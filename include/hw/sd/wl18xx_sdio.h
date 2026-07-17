/*
 * TI WiLink8 (wl1835) SDIO WiFi function
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 */

#ifndef HW_SD_WL18XX_SDIO_H
#define HW_SD_WL18XX_SDIO_H

/*
 * TYPE_WL18XX_SDIO -- the TI WiLink8 (wl1835) SDIO WiFi function as an
 * SD-bus card. A board attaches it to an MMC/SD host controller's "sd-bus"
 * exactly the way it attaches TYPE_SD_CARD / TYPE_EMMC (see the BeagleBone
 * Green Wireless, hw/arm/beaglebone-green-wireless.c). Only the type name is
 * public; the device state embeds the hw/sd-internal SDState, so its struct
 * lives in hw/sd/wl18xx_sdio.c rather than here.
 */
#define TYPE_WL18XX_SDIO "wl18xx-sdio"

#endif /* HW_SD_WL18XX_SDIO_H */
