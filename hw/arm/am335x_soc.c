/*
 * TI AM335x SoC emulation (M1 skeleton)
 *
 * Copyright (C) 2026
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

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/arm/am335x_soc.h"
#include "hw/sysbus.h"
#include "hw/char/am335x_uart.h"
#include "hw/net/am335x_cpsw.h"
#include "hw/qdev-properties.h"
#include "hw/misc/unimp.h"
#include "net/net.h"
#include "system/system.h"
#include "exec/address-spaces.h"
#include "target/arm/cpu-qom.h"

/* On-chip memory (L3 OCMC RAM, 64KB, TRM spruh73q ch.7.2). */
#define AM335X_OCMC_BASE        0x40300000
#define AM335X_OCMC_SIZE        (64 * KiB)

/*
 * Lower internal SRAM bank (TRM spruh73q Table 2-x memory map: "SRAM
 * internal" 0x402F0400..0x402FFFFF, with 0x402F0000..0x402F03FF reserved).
 * On a GP device the ROM downloads the SPL/MLO image directly here at
 * 0x402F0400 (CONFIG_SPL_TEXT_BASE) and it runs across this bank and the
 * adjacent OCMC RAM as one contiguous 0x402F0400..0x4030FFFF window
 * (NON_SECURE_SRAM_START..END in u-boot's arch-am33xx/omap.h). Only the
 * OCMC half was modelled before; SPL text/BSS/stack straddle both, so the
 * lower bank must exist too. Mapped page-aligned at 0x402F0000 (the
 * reserved 1KB below TEXT_BASE is harmless as plain RAM).
 */
#define AM335X_SRAM_BASE        0x402F0000
#define AM335X_SRAM_SIZE        (64 * KiB)

/*
 * UART0 (16550-compatible core + OMAP soft-reset regs, regshift fixed at 2
 * inside AM335xUartState). Real hardware's UART functional clock is 48MHz
 * (AM335X_UART0_CLK / 16 = 3MHz baud generator input); AM335xUartState does
 * not expose a "baudbase" property (out of scope -- see am335x_uart.c), so
 * the embedded SerialState falls back to TYPE_SERIAL's 115200 default. That
 * only affects host-side transmit pacing math for a real serial backend,
 * not the pty/stdio chardevs this board actually uses.
 */
#define AM335X_UART0_BASE       0x44E09000

/* INTC input line numbers (TRM spruh73q ch.6) */
#define AM335X_IRQ_UART0        72

/* LCD Controller (TRM spruh73q ch.13; DT lcdc@0 at target-module@e000,
 * interrupts <36> in am33xx-l4.dtsi). */
#define AM335X_LCDC_BASE        0x4830E000
#define AM335X_IRQ_LCDC         36

/* CPSW Ethernet switch subsystem (TRM spruh73q ch.14; DT switch@0 at
 * target-module@100000, base 0x4A100000, interrupts <40 41 42 43> =
 * rx_thresh/rx/tx/misc in am33xx-l4.dtsi). */
#define AM335X_CPSW_BASE        0x4A100000
#define AM335X_IRQ_CPSW_RXTHR   40
#define AM335X_IRQ_CPSW_RX      41
#define AM335X_IRQ_CPSW_TX      42
#define AM335X_IRQ_CPSW_MISC    43

/* USB Subsystem "clean probe" stub (TRM spruh73q ch.16; DT
 * target-module@47400000, ranges <0x0 0x47400000 0x8000>). No IRQ is
 * wired -- see hw/misc/am335x_usbss.c -- since this model never generates
 * USB controller activity. */
#define AM335X_USBSS_BASE       0x47400000

/*
 * EDMA3 (TRM spruh73q ch.11). TPCC (Third-Party Channel Controller, the
 * user-facing DMA block) @ 0x49000000; INTC lines 12/13/14 = EDMACOMPINT/
 * EDMAMPERR/EDMAERRINT (TRM Table 6-1; DT target-module@49000000 interrupts
 * <12 13 14>). The three Transfer Controllers (TPTC0/1/2) @ 0x49800000/
 * 0x49900000/0x49A00000 (1MB each) are register-file stubs -- the mainline
 * edma tptc driver touches no TPTC registers (edma.c edma_tptc_probe). */
#define AM335X_EDMA_BASE        0x49000000
#define AM335X_IRQ_EDMA_COMP    12
#define AM335X_IRQ_EDMA_MPERR   13
#define AM335X_IRQ_EDMA_ERR     14
#define AM335X_EDMA_TPTC0_BASE  0x49800000
#define AM335X_EDMA_TPTC1_BASE  0x49900000
#define AM335X_EDMA_TPTC2_BASE  0x49A00000
#define AM335X_EDMA_TPTC_SIZE   0x100000

/*
 * McASP0 audio serial port (TRM spruh73q ch.22). Config ("mpu") registers @
 * 0x48038000 (8KB) and the data port ("dat") @ 0x46000000 (4MB); INTC lines
 * 80/81 = tx/rx (DT mcasp0 interrupts <80 81>). DMA-driven off EDMA3 -- feeds
 * the on-board TDA19988 HDMI encoder as the ALSA simple-audio-card CPU DAI. */
#define AM335X_MCASP0_MPU_BASE  0x48038000
#define AM335X_MCASP0_DAT_BASE  0x46000000
#define AM335X_IRQ_MCASP0_TX    80
#define AM335X_IRQ_MCASP0_RX    81

/* DMTIMER0..3 MMIO bases and INTC input lines (TRM spruh73q ch.6/20).
 * one_ms marks the "ti,am335x-timer-1ms" variant (DMTIMER1), whose OCP
 * SYSCONFIG has SOFTRESET at bit 1 (bit 0 is AUTOIDLE); the regular timers
 * have SOFTRESET at bit 0. Linux uses DMTIMER1 as the always-on
 * clocksource, so getting this wrong stops timekeeping. */
static const struct {
    hwaddr addr;
    unsigned int irq;
    bool one_ms;
} am335x_timer_table[AM335X_NUM_TIMERS] = {
    { 0x44E05000, 66, false }, /* DMTIMER0 */
    { 0x44E31000, 67, true  }, /* DMTIMER1 (1ms, always-on clocksource) */
    { 0x48040000, 68, false }, /* DMTIMER2 */
    { 0x48042000, 69, false }, /* DMTIMER3 */
};

/*
 * MMCHS0/1/2 MMIO bases and INTC input lines (TRM spruh73q ch.6/18).
 * All three MMC/SD host controllers exist in AM335x silicon regardless of
 * board; which ones a board wires out is board-level policy handled in the
 * board file's card-attach loop. MMCHS2's base 0x47810000 and INTC line 29
 * (MMCSD2INT) are from the TRM L3/L4 memory map and Table 6-1 interrupt map,
 * cross-checked against the mainline DT (am33xx.dtsi target-module@47810000,
 * mmc@0 "ti,am335-sdhci" interrupts = <29>). Only the BeagleBone Green
 * Wireless (beaglebone-green-wireless.c) actually populates MMCHS2, with the
 * on-package TI WiLink8 (wl1835) SDIO WiFi function; on every other board the
 * card-attach loop finds no -sd drive at index 2 and leaves it unconnected,
 * matching real hardware where MMCHS2's pins are simply not routed out.
 */
static const struct {
    hwaddr addr;
    unsigned int irq;
} am335x_mmc_table[AM335X_NUM_MMC] = {
    { 0x48060000, 64 }, /* MMC0 -> mmcblk0 (microSD)         */
    { 0x481D8000, 28 }, /* MMC1 -> mmcblk1 (eMMC)            */
    { 0x47810000, 29 }, /* MMC2 -> SDIO (wl1835 WiFi on BBGW) */
};

/* I2C0 MMIO base and INTC input line (TRM spruh73q ch.6/21; DT
 * interrupts <70> in am33xx-l4.dtsi). I2C1/I2C2 stay unimplemented. */
static const struct {
    hwaddr addr;
    unsigned int irq;
} am335x_i2c_table[AM335X_NUM_I2C] = {
    { 0x44E0B000, 70 }, /* I2C0 */
};

/* GPIO0..3 MMIO bases and INTC input lines (TRM spruh73q ch.6/25;
 * DT interrupts 96/98/32/62 in am33xx-l4.dtsi). datain_reset seeds the
 * static input level: GPIO0 line 6 is the microSD card-detect
 * (cd-gpios = <&gpio0 6 GPIO_ACTIVE_LOW>), so leaving it low (0) reports a
 * card present. */
static const struct {
    hwaddr addr;
    unsigned int irq;
    uint32_t datain_reset;
} am335x_gpio_table[AM335X_NUM_GPIO] = {
    { 0x44E07000, 96, 0 }, /* GPIO0 (microSD card-detect on line 6) */
    { 0x4804C000, 98, 0 }, /* GPIO1 (USR LEDs 21-24) */
    { 0x481AC000, 32, 0 }, /* GPIO2 */
    { 0x481AE000, 62, 0 }, /* GPIO3 */
};

static void am335x_soc_init(Object *obj)
{
    AM335xState *s = AM335X_SOC(obj);
    int i;

    object_initialize_child(obj, "cpu", &s->cpu,
                            ARM_CPU_TYPE_NAME("cortex-a8"));
    object_initialize_child(obj, "intc", &s->intc, TYPE_AM335X_INTC);
    for (i = 0; i < AM335X_NUM_TIMERS; i++) {
        object_initialize_child(obj, "timer[*]", &s->timer[i],
                                TYPE_AM335X_TIMER);
    }
    object_initialize_child(obj, "prcm", &s->prcm, TYPE_AM335X_PRCM);
    object_initialize_child(obj, "wdt", &s->wdt, TYPE_AM335X_WDT);
    object_initialize_child(obj, "control", &s->control, TYPE_AM335X_CONTROL);
    object_initialize_child(obj, "emif", &s->emif, TYPE_AM335X_EMIF);
    for (i = 0; i < AM335X_NUM_GPIO; i++) {
        object_initialize_child(obj, "gpio[*]", &s->gpio[i],
                                TYPE_AM335X_GPIO);
    }
    for (i = 0; i < AM335X_NUM_MMC; i++) {
        object_initialize_child(obj, "mmc[*]", &s->mmc[i],
                                TYPE_AM335X_HSMMC);
    }
    for (i = 0; i < AM335X_NUM_I2C; i++) {
        object_initialize_child(obj, "i2c[*]", &s->i2c[i], TYPE_AM335X_I2C);
    }
    object_initialize_child(obj, "rtc", &s->rtc, TYPE_AM335X_RTC);
    object_initialize_child(obj, "lcdc", &s->lcdc, TYPE_AM335X_LCDC);
    object_initialize_child(obj, "cpsw", &s->cpsw, TYPE_AM335X_CPSW);
    object_initialize_child(obj, "usbss", &s->usbss, TYPE_AM335X_USBSS);
    object_initialize_child(obj, "edma", &s->edma, TYPE_AM335X_EDMA);
    object_initialize_child(obj, "mcasp0", &s->mcasp0, TYPE_AM335X_MCASP);
    object_initialize_child(obj, "uart0", &s->uart0, TYPE_AM335X_UART);
}

static void am335x_soc_realize(DeviceState *dev, Error **errp)
{
    AM335xState *s = AM335X_SOC(dev);
    int i;

    if (!qdev_realize(DEVICE(&s->cpu), NULL, errp)) {
        return;
    }

    /* On-chip SRAM (OCMC RAM) */
    memory_region_init_ram(&s->ocmc, OBJECT(dev), "am335x.ocmc",
                           AM335X_OCMC_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(), AM335X_OCMC_BASE,
                                &s->ocmc);

    /* Lower internal SRAM bank, contiguous below the OCMC (see above). */
    memory_region_init_ram(&s->sram, OBJECT(dev), "am335x.sram",
                           AM335X_SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(), AM335X_SRAM_BASE,
                                &s->sram);

    /* MPU interrupt controller (INTCPS) @ 0x48200000, out 0 = IRQ, 1 = FIQ. */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->intc), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->intc), 0, 0x48200000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->intc), 0,
                       qdev_get_gpio_in(DEVICE(&s->cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->intc), 1,
                       qdev_get_gpio_in(DEVICE(&s->cpu), ARM_CPU_FIQ));
    /* Re-export the 128 INTC input lines on the SoC container. */
    qdev_pass_gpios(DEVICE(&s->intc), dev, NULL);

    /*
     * UART0 (16550-compatible core + OMAP soft-reset regs), IRQ line 72
     * on the INTC. The OMAP-only MDR1/SYSC/SYSS window (see
     * hw/char/am335x_uart.c) is what lets the Linux 8250_omap console
     * driver's soft-reset probe sequence complete instead of hanging on
     * an unmapped SYSS.RESETDONE poll, which is why this is a dedicated
     * device instead of a plain serial_mm_init().
     * FIXME use a qdev chardev prop instead of serial_hd()
     */
    qdev_prop_set_chr(DEVICE(&s->uart0), "chardev", serial_hd(0));
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->uart0), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->uart0), 0, AM335X_UART0_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart0), 0,
                       qdev_get_gpio_in(dev, AM335X_IRQ_UART0));

    /* DMTIMER0..3: real devices, needed for the kernel clockevent/
     * clocksource to make progress past time init. */
    for (i = 0; i < AM335X_NUM_TIMERS; i++) {
        qdev_prop_set_bit(DEVICE(&s->timer[i]), "one-ms",
                          am335x_timer_table[i].one_ms);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->timer[i]), 0,
                        am335x_timer_table[i].addr);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->timer[i]), 0,
                           qdev_get_gpio_in(dev, am335x_timer_table[i].irq));
    }

    /*
     * Clock Module / PRCM @ 0x44E00000. Needed so ti-sysc can enable
     * module functional clocks (CLKCTRL IDLEST) and DPLLs report locked,
     * which unblocks the DMTIMER probes above.
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->prcm), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->prcm), 0, 0x44E00000);

    /*
     * Watchdog Timer 1 (WDT1) @ 0x44E35000. A benign stub whose ti-sysc OCP
     * softreset completes immediately (WD_SYSSTATUS.RESETDONE reads 1); it
     * never arms or bites. Without it the module's softreset times out
     * against the unimplemented-device stub, stalling boot ~220s.
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->wdt), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->wdt), 0, 0x44E35000);

    /*
     * Control Module (System Control Module) @ 0x44E10000. Its sole
     * functional job here is to report a 24MHz input crystal via
     * CONTROL_STATUS.SYSBOOT1, so the kernel's derived sys_clkin (and the
     * dmtimer clocksource rate) match the 24MHz our DMTIMER model runs at.
     * A mismatch makes the generic sched_clock 32-bit wrap handling
     * miscalibrate and freeze every timestamp at ~223s. See
     * hw/misc/am335x_control.c.
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->control), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->control), 0, 0x44E10000);

    /*
     * EMIF (DDR controller) @ 0x4C000000. Programmed by the SPL (MLO)
     * during DDR3 bring-up when booting from scratch via -bios; a passive
     * register store since the DRAM itself is plain RAM at 0x80000000.
     * See hw/misc/am335x_emif.c.
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->emif), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->emif), 0, 0x4C000000);

    /*
     * GPIO0..3. GPIO0's card-detect input (line 6) gates the microSD
     * controller's probe, so this must be a real device rather than an
     * unimplemented stub. The per-line outputs are also where on-board
     * LEDs (GPIO1) will attach.
     */
    for (i = 0; i < AM335X_NUM_GPIO; i++) {
        qdev_prop_set_uint32(DEVICE(&s->gpio[i]), "datain-reset",
                             am335x_gpio_table[i].datain_reset);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio[i]), 0,
                        am335x_gpio_table[i].addr);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio[i]), 0,
                           qdev_get_gpio_in(dev, am335x_gpio_table[i].irq));
    }

    /*
     * MMCHS0/1 (SDHCI behind a TI wrapper). Needed so the guest can mount a
     * rootfs from an SD image. The board attaches the actual SD cards to
     * each controller's "sd-bus".
     */
    for (i = 0; i < AM335X_NUM_MMC; i++) {
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->mmc[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->mmc[i]), 0,
                        am335x_mmc_table[i].addr);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->mmc[i]), 0,
                           qdev_get_gpio_in(dev, am335x_mmc_table[i].irq));
    }

    /*
     * I2C0 @ 0x44E0B000, IRQ 70. A real master controller so the on-board
     * I2C slaves (board-ID EEPROM, TPS65217 PMIC) can be probed; also
     * services the ti-sysc OCP softreset that would otherwise stall boot
     * against an unimplemented stub (cf. WDT1). The board attaches the
     * slaves to this controller's I2C bus.
     */
    for (i = 0; i < AM335X_NUM_I2C; i++) {
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->i2c[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[i]), 0,
                        am335x_i2c_table[i].addr);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[i]), 0,
                           qdev_get_gpio_in(dev, am335x_i2c_table[i].irq));
    }

    /*
     * RTC @ 0x44E3E000, IRQ 75 (periodic) and 76 (alarm). Its ti-sysc
     * wrapper does no OCP softreset (srst_shift = -ENODEV), so unlike the
     * blocks above there is no reset-done poll to satisfy.
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rtc), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rtc), 0, 0x44E3E000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc), 0, qdev_get_gpio_in(dev, 75));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc), 1, qdev_get_gpio_in(dev, 76));

    /*
     * LCD Controller @ 0x4830E000, IRQ 36. Its PID register reports the
     * rev-2 LCDC so the Linux tilcdc driver binds and, together with the
     * on-board TDA19988 HDMI encoder (wired on I2C0 by the board), scans out
     * a DRM/fbdev framebuffer. The INTC line carries the end-of-frame (vblank)
     * and frame/palette-done interrupts (see am335x_lcdc.c).
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->lcdc), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->lcdc), 0, AM335X_LCDC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->lcdc), 0,
                       qdev_get_gpio_in(dev, AM335X_IRQ_LCDC));

    /*
     * CPSW Ethernet switch subsystem @ 0x4A100000 (with the davinci MDIO
     * sub-block at 0x4A101000 and the on-chip descriptor SRAM at
     * 0x4A102000). A real device so the built-in cpsw-switch/davinci_mdio
     * drivers bind, the on-board PHY reports link, and the guest gets a
     * DHCP lease over the SoC NIC. The four INTC lines are rx_thresh/rx/
     * tx/misc (40..43); only rx (41) and tx (42) are driven.
     * qemu_configure_nic_device() binds the default -nic/-netdev before
     * realize so the embedded NIC connects to it.
     */
    qemu_configure_nic_device(DEVICE(&s->cpsw), true, NULL);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->cpsw), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->cpsw), 0, AM335X_CPSW_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->cpsw), 0,
                       qdev_get_gpio_in(dev, AM335X_IRQ_CPSW_RXTHR));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->cpsw), 1,
                       qdev_get_gpio_in(dev, AM335X_IRQ_CPSW_RX));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->cpsw), 2,
                       qdev_get_gpio_in(dev, AM335X_IRQ_CPSW_TX));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->cpsw), 3,
                       qdev_get_gpio_in(dev, AM335X_IRQ_CPSW_MISC));

    /*
     * USB Subsystem (USBSS) @ 0x47400000, a "clean probe" register-file
     * stub covering the whole 32KB target-module window (USB0/USB1
     * control+mc blocks and the CPPI4.1 DMA glue/queue-manager range).
     * Its sole functional job is to let musb-hdrc/musb_dsps probe
     * without the -ENODEV that a fully-unmapped window causes (real
     * silicon's USBnREV wrapper-revision register reads back non-zero;
     * unassigned memory does not) and register a host/gadget instance.
     * No USB device enumeration, transfer, or CPPI DMA is modelled --
     * see hw/misc/am335x_usbss.c -- and no IRQ line is connected since
     * this stub never generates controller activity.
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->usbss), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->usbss), 0, AM335X_USBSS_BASE);

    /*
     * EDMA3 channel controller (TPCC) @ 0x49000000. A real device so the
     * mainline ti,edma3-tpcc dmaengine driver probes and provides DMA
     * channels to its clients -- most importantly McASP0, whose davinci-mcasp
     * driver otherwise fails with "No DMA controller found". Completion line
     * 12 drives cyclic-audio-playback progress; error line 14 is wired but
     * never asserted, and the mem-protect line 13 is not requested by Linux.
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->edma), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->edma), 0, AM335X_EDMA_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->edma), 0,
                       qdev_get_gpio_in(dev, AM335X_IRQ_EDMA_COMP));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->edma), 1,
                       qdev_get_gpio_in(dev, AM335X_IRQ_EDMA_MPERR));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->edma), 2,
                       qdev_get_gpio_in(dev, AM335X_IRQ_EDMA_ERR));

    /*
     * EDMA3 Transfer Controllers TPTC0/1/2. Register-file stubs (see the
     * address-define comment): the ti,edma3-tptc driver touches none of their
     * registers, and the ti-sysc wrapper needs only a readable REV plus to
     * have its SYSC softreset write absorbed (no reset-done poll, so no
     * boot stall).
     */
    create_unimplemented_device("edma-tptc0", AM335X_EDMA_TPTC0_BASE,
                                AM335X_EDMA_TPTC_SIZE);
    create_unimplemented_device("edma-tptc1", AM335X_EDMA_TPTC1_BASE,
                                AM335X_EDMA_TPTC_SIZE);
    create_unimplemented_device("edma-tptc2", AM335X_EDMA_TPTC2_BASE,
                                AM335X_EDMA_TPTC_SIZE);

    /*
     * McASP0 audio serial port: config window @ 0x48038000 and data port @
     * 0x46000000, INTC lines 80/81 (tx/rx). With EDMA3 above providing its DMA
     * channel, davinci-mcasp probes, snd_soc registers the simple-audio-card,
     * and playback DMA can run (completions come from EDMA). The tx/rx lines
     * are error-only (under/overrun) and never asserted by the model.
     */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->mcasp0), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->mcasp0), 0, AM335X_MCASP0_MPU_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->mcasp0), 1, AM335X_MCASP0_DAT_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->mcasp0), 0,
                       qdev_get_gpio_in(dev, AM335X_IRQ_MCASP0_TX));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->mcasp0), 1,
                       qdev_get_gpio_in(dev, AM335X_IRQ_MCASP0_RX));

    /*
     * Placeholders for peripherals that become real devices in later
     * milestones. Mapping them as unimplemented devices means stray guest
     * MMIO is logged instead of aborting the machine.
     */
    create_unimplemented_device("tscadc",          0x44E0D000, 0x1000);
    create_unimplemented_device("counter32k",      0x44E86000, 0x1000);
}

static void am335x_soc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = am335x_soc_realize;
    /* Reason: Uses serial_hd() in realize function */
    dc->user_creatable = false;
}

static const TypeInfo am335x_soc_type_info = {
    .name = TYPE_AM335X_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(AM335xState),
    .instance_init = am335x_soc_init,
    .class_init = am335x_soc_class_init,
};

static void am335x_soc_register_types(void)
{
    type_register_static(&am335x_soc_type_info);
}

type_init(am335x_soc_register_types)
