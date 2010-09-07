/*
 * Texas Instruments TNETV107X SoC Specific Defines
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __ASM_ARCH_DAVINCI_TNETV107X_H
#define __ASM_ARCH_DAVINCI_TNETV107X_H

#include <asm/sizes.h>
#include <linux/if_ether.h>

#define TNETV107X_DDR_BASE	0x80000000

/*
 * Fixed mapping for early init starts here. If low-level debug is enabled,
 * this area also gets mapped via io_pg_offset and io_phys by the boot code.
 * To fit in with the io_pg_offset calculation, the io base address selected
 * here _must_ be a multiple of 2^20.
 */
#define TNETV107X_IO_BASE	0x08000000
#define TNETV107X_IO_VIRT	(IO_VIRT + SZ_1M)

#define TNETV107X_N_GPIO	65

#ifndef __ASSEMBLY__

#include <linux/serial_8250.h>
#include <mach/mmc.h>
#include <mach/nand.h>
#include <mach/serial.h>
#include <mach/ti_ssp.h>

struct tnetv107x_keypad_data {
	int		*keymap;
	const char	**keynames;
	int		keymap_size;
	int		rows;
	int		cols;
	u32		debounce;
	u32		stable;
};

struct tnetv107x_tsc_data {
	int		xres, yres;

	/*
	 * Calibration info:
	 *   out_x = (C0 * in_x + C1 * in_y + C2) / C6
	 *   out_y = (C3 * in_x + C4 * in_y + C5) / C6
	 */
#define TSC_CAL_SIZE	7
	int		calibration_data[TSC_CAL_SIZE];
};

struct tnetv107x_cpsw_info {
	void		(*phy_control)(bool enabled);
	int		rx_descs;
	const char	*phy_id[2];
	int		phy_if[2];
	unsigned char	mac_addr[ETH_ALEN];
};

struct tnetv107x_device_info {
	struct davinci_uart_config	*serial_config;
	struct davinci_mmc_config	*mmc_config[2];  /* 2 controllers */
	struct davinci_nand_pdata	*nand_config[4]; /* 4 chipsels */
	struct tnetv107x_keypad_data	*keypad_config;
	struct tnetv107x_tsc_data	*tsc_config;
	struct tnetv107x_cpsw_info	*cpsw_config;
	struct ti_ssp_data		*ssp_config;
};

extern struct platform_device tnetv107x_wdt_device;
extern struct platform_device tnetv107x_serial_device;

extern void __init tnetv107x_init(void);
extern void __init tnetv107x_devices_init(struct tnetv107x_device_info *);
extern void __init tnetv107x_irq_init(void);

#endif

#endif /* __ASM_ARCH_DAVINCI_TNETV107X_H */
