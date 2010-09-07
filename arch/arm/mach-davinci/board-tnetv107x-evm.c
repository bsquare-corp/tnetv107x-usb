/*
 * Texas Instruments TNETV107X EVM Board Support
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/ratelimit.h>
#include <linux/input.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/irqs.h>
#include <mach/edma.h>
#include <mach/mux.h>
#include <mach/cp_intc.h>
#include <mach/ti_ssp.h>
#include <mach/tnetv107x.h>

#define EVM_MMC_WP_GPIO		21
#define EVM_MMC_CD_GPIO		24

#define TNETV107X_VTP_BASE	0x0803d800
#define TNETV107X_LATCH_BASE	0x48000000

#define TNETV107X_SSP_DEVICE_TPS6116X	1
#define TNETV107X_SSP_DEVICE_TPS65241	2

static int initialize_gpio(int gpio, char *desc)
{
	int ret;

	ret = gpio_request(gpio, desc);
	if (ret < 0) {
		pr_err_ratelimited("cannot open %s gpio\n", desc);
		return -ENOSYS;
	}
	gpio_direction_input(gpio);
	return gpio;
}

static int mmc_get_cd(int index)
{
	static int gpio;

	if (!gpio)
		gpio = initialize_gpio(EVM_MMC_CD_GPIO, "mmc card detect");

	if (gpio < 0)
		return gpio;

	return gpio_get_value(gpio) ? 0 : 1;
}

static int mmc_get_ro(int index)
{
	static int gpio;

	if (!gpio)
		gpio = initialize_gpio(EVM_MMC_WP_GPIO, "mmc write protect");

	if (gpio < 0)
		return gpio;

	return gpio_get_value(gpio) ? 1 : 0;
}

static int __init cpsw_phy_init(void)
{
	void __iomem *latch;

	latch = ioremap(TNETV107X_LATCH_BASE, SZ_4K);
	__raw_writel(0x00000000, latch); mdelay(1);
	__raw_writel(0xffffffff, latch); mdelay(1);
	iounmap(latch);
	return 0;
}
arch_initcall(cpsw_phy_init);

static void cpsw_phy_control(bool enabled)
{
	static struct clk	*rgmii_clk;
	static void __iomem	*vtp_regs;

	if (!rgmii_clk)
		rgmii_clk = clk_get(NULL, "clk_ethss_rgmii");
	if (!vtp_regs)
		vtp_regs = ioremap(TNETV107X_VTP_BASE, SZ_4K);
	if (WARN_ON(!rgmii_clk || !vtp_regs))
		return;

	if (enabled) {
		/* First enable the rgmii module */
		clk_enable(rgmii_clk);

		/*
		 * This piece of hardware is horribly mangled.  For one, port
		 * 0 and port 1 configurations are strangely mixed up in the
		 * register space, i.e., writing to port 0 registers affects
		 * port 1 as well.   Second, for some other equally mysterious
		 * reason, port 1 MUST be configured before port 0.
		 */
		__raw_writel(0x00000000, vtp_regs + 0x104); /* single mode  */
		__raw_writel(0x000f0000, vtp_regs + 0x110); /* slew slowest */
		__raw_writel(0x00000002, vtp_regs + 0x114); /* start	    */

		__raw_writel(0x00000000, vtp_regs + 0x004); /* single mode  */
		__raw_writel(0x000f0000, vtp_regs + 0x010); /* slew slowest */
		__raw_writel(0x00000002, vtp_regs + 0x014); /* start	    */
	} else {
		clk_disable(rgmii_clk);
	}
}

static struct tnetv107x_cpsw_info cpsw_config = {
	.phy_control	= cpsw_phy_control,
	.phy_id		= { "0:00", "0:01" },
};

static struct davinci_mmc_config mmc_config = {
	.get_cd		= mmc_get_cd,
	.get_ro		= mmc_get_ro,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_1,
};

static const short sdio1_pins[] __initdata = {
	TNETV107X_SDIO1_CLK_1,		TNETV107X_SDIO1_CMD_1,
	TNETV107X_SDIO1_DATA0_1,	TNETV107X_SDIO1_DATA1_1,
	TNETV107X_SDIO1_DATA2_1,	TNETV107X_SDIO1_DATA3_1,
	TNETV107X_GPIO21,		TNETV107X_GPIO24,
	-1
};

static const short uart1_pins[] __initdata = {
	TNETV107X_UART1_RD,		TNETV107X_UART1_TD,
	-1
};

static const short ssp_pins[] __initdata = {
	TNETV107X_SSP0_0, TNETV107X_SSP0_1, TNETV107X_SSP0_2,
	TNETV107X_SSP1_0, TNETV107X_SSP1_1, TNETV107X_SSP1_2,
	TNETV107X_SSP1_3, -1
};

static struct mtd_partition nand_partitions[] = {
	/* bootloader (U-Boot, etc) in first 12 sectors */
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= (12*SZ_128K),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	/* bootloader params in the next sector */
	{
		.name		= "params",
		.offset		= MTDPART_OFS_NXTBLK,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	/* kernel */
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_NXTBLK,
		.size		= SZ_4M,
		.mask_flags	= 0,
	},
	/* file system */
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_NXTBLK,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
};

static struct davinci_nand_pdata nand_config = {
	.mask_cle	= 0x4000,
	.mask_ale	= 0x2000,
	.parts		= nand_partitions,
	.nr_parts	= ARRAY_SIZE(nand_partitions),
	.ecc_mode	= NAND_ECC_HW,
	.options	= NAND_USE_FLASH_BBT,
	.ecc_bits	= 1,
};

static struct davinci_uart_config serial_config __initconst = {
	.enabled_uarts	= BIT(1),
};

static int keymap[] = {
	/* row 0 */
	KEY_NUMERIC_1, KEY_NUMERIC_2, KEY_NUMERIC_3, KEY_FN_F1, KEY_MENU,
	/* row 1 */
	KEY_NUMERIC_4, KEY_NUMERIC_5, KEY_NUMERIC_6, KEY_UP, KEY_FN_F2,
	/* row 2 */
	KEY_NUMERIC_7, KEY_NUMERIC_8, KEY_NUMERIC_9, KEY_LEFT, KEY_ENTER,
	/* row 3 */
	KEY_NUMERIC_STAR, KEY_NUMERIC_0, KEY_NUMERIC_POUND, KEY_DOWN, KEY_RIGHT,
	/* row 4 */
	KEY_FN_F3, KEY_FN_F4, KEY_MUTE, KEY_HOME, KEY_BACK,
	/* row 5 */
	KEY_VOLUMEDOWN, KEY_VOLUMEUP, KEY_F1, KEY_F2, KEY_F3,
};

static const char *keynames[] = {
	/* row 0 */
	"1", "2", "3", "S1 (FN_F1)", "MENU",
	/* row 1 */
	"4", "5", "6", "UP", "S2 (FN_F2)",
	/* row 2 */
	"7", "8", "9", "LEFT", "ENTER",
	/* row 3 */
	"*", "0", "#", "DOWN", "RIGHT",
	/* row 4 */
	"SPEAKER (FN_F3)", "HEADSET (FN_F4)", "MUTE", "HOME", "BACK",
	/* row 5 */
	"VOL_DOWN", "VOL_UP", "F1", "F2", "F3",
};

static struct tnetv107x_keypad_data keypad_config = {
	.keynames	= keynames,
	.keymap		= keymap,
	.keymap_size	= ARRAY_SIZE(keymap),
	.rows		= 6,
	.cols		= 5,
	.debounce	= 0x400,
	.stable		= 0x3,
};

static struct tnetv107x_tsc_data tsc_config = {
	.xres	= 800,
	.yres	= 480,
	.calibration_data = {
		217, 14019, -2540712, 8690, -140, -1651470, 65536,
	},
};

static struct ti_ssp_device ssp_devices[] = {
	{
		.id		= TNETV107X_SSP_DEVICE_TPS6116X,
		.name		= "tps6116x",
		.port		= 0,
		.min_clock	= 100*1000,
		.max_clock	= 250*1000,
		.iosel		=
			SSP_PIN_SEL(2, SSP_DATA)	|
			SSP_PIN_SEL(3, SSP_CLOCK)	|
			SSP_INPUT_SEL(0),
		.config		= 0,
	},
	{
		.id		= TNETV107X_SSP_DEVICE_TPS65241,
		.name		= "tps6524x",
		.port		= 1,
		.min_clock	= 100*1000,
		.max_clock	= 5*1000*1000,
		.iosel		=
			SSP_PIN_SEL(0, SSP_CLOCK)	|
			SSP_PIN_SEL(1, SSP_DATA)	|
			SSP_PIN_SEL(2, SSP_CHIPSEL)	|
			SSP_PIN_SEL(3, SSP_IN)		|
			SSP_INPUT_SEL(3),
		.config		= SSP_EARLY_DIN
	},
};

static struct ti_ssp_data ssp_config = {
	.num_devices	= ARRAY_SIZE(ssp_devices),
	.devices	= ssp_devices,
};

static struct tnetv107x_device_info evm_device_info __initconst = {
	.serial_config		= &serial_config,
	.mmc_config[1]		= &mmc_config,	/* controller 1 */
	.nand_config[0]		= &nand_config,	/* chip select 0 */
	.keypad_config		= &keypad_config,
	.tsc_config		= &tsc_config,
	.cpsw_config		= &cpsw_config,
	.ssp_config		= &ssp_config,
};

static __init void tnetv107x_evm_board_init(void)
{
	davinci_cfg_reg_list(sdio1_pins);
	davinci_cfg_reg_list(uart1_pins);
	davinci_cfg_reg_list(ssp_pins);

	tnetv107x_devices_init(&evm_device_info);
}

#ifdef CONFIG_SERIAL_8250_CONSOLE
static int __init tnetv107x_evm_console_init(void)
{
	return add_preferred_console("ttyS", 0, "115200");
}
console_initcall(tnetv107x_evm_console_init);
#endif

MACHINE_START(TNETV107X, "TNETV107X EVM")
	.phys_io	= TNETV107X_IO_BASE,
	.io_pg_offst	= (TNETV107X_IO_VIRT >> 18) & 0xfffc,
	.boot_params	= (TNETV107X_DDR_BASE + 0x100),
	.map_io		= tnetv107x_init,
	.init_irq	= cp_intc_init,
	.timer		= &davinci_timer,
	.init_machine	= tnetv107x_evm_board_init,
MACHINE_END
