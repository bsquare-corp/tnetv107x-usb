/*
 * Texas Instruments TNETV107X SoC devices
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
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/phy.h>

#include <mach/common.h>
#include <mach/irqs.h>
#include <mach/edma.h>
#include <mach/tnetv107x.h>
#include <mach/cpsw.h>
#include <mach/cppi41.h>


#include "clock.h"

/* Base addresses for on-chip devices */
#define TNETV107X_TPCC_BASE			0x01c00000
#define TNETV107X_TPTC0_BASE			0x01c10000
#define TNETV107X_TPTC1_BASE			0x01c10400
#define TNETV107X_CPSW_BASE			0x0803c000
#define TNETV107X_WDOG_BASE			0x08086700
#define TNETV107X_TSC_BASE			0x08088500
#define TNETV107X_SDIO0_BASE			0x08088700
#define TNETV107X_SDIO1_BASE			0x08088800
#define TNETV107X_MDIO_BASE			0x08088900
#define TNETV107X_KEYPAD_BASE			0x08088a00
#define TNETV107X_SSP_BASE			0x08088c00
#define TNETV107X_ASYNC_EMIF_CNTRL_BASE		0x08200000
#define TNETV107X_ASYNC_EMIF_DATA_CE0_BASE	0x30000000
#define TNETV107X_ASYNC_EMIF_DATA_CE1_BASE	0x40000000
#define TNETV107X_ASYNC_EMIF_DATA_CE2_BASE	0x44000000
#define TNETV107X_ASYNC_EMIF_DATA_CE3_BASE	0x48000000
#define TNETV107X_USB1_BASE			0x08120800


/* TNETV107X specific EDMA3 information */
#define EDMA_TNETV107X_NUM_DMACH	64
#define EDMA_TNETV107X_NUM_TCC		64
#define EDMA_TNETV107X_NUM_PARAMENTRY	128
#define EDMA_TNETV107X_NUM_EVQUE	2
#define EDMA_TNETV107X_NUM_TC		2
#define EDMA_TNETV107X_CHMAP_EXIST	0
#define EDMA_TNETV107X_NUM_REGIONS	4
#define TNETV107X_DMACH2EVENT_MAP0	0x3C0CE000u
#define TNETV107X_DMACH2EVENT_MAP1	0x000FFFFFu

#define TNETV107X_DMACH_SDIO0_RX		26
#define TNETV107X_DMACH_SDIO0_TX		27
#define TNETV107X_DMACH_SDIO1_RX		28
#define TNETV107X_DMACH_SDIO1_TX		29

static const s8 edma_tc_mapping[][2] = {
	/* event queue no	TC no	*/
	{	 0,		 0	},
	{	 1,		 1	},
	{	-1,		-1	}
};

static const s8 edma_priority_mapping[][2] = {
	/* event queue no	Prio	*/
	{	 0,		 3	},
	{	 1,		 7	},
	{	-1,		-1	}
};

static struct edma_soc_info edma_cc0_info = {
	.n_channel		= EDMA_TNETV107X_NUM_DMACH,
	.n_region		= EDMA_TNETV107X_NUM_REGIONS,
	.n_slot			= EDMA_TNETV107X_NUM_PARAMENTRY,
	.n_tc			= EDMA_TNETV107X_NUM_TC,
	.n_cc			= 1,
	.queue_tc_mapping	= edma_tc_mapping,
	.queue_priority_mapping	= edma_priority_mapping,
};

static struct edma_soc_info *tnetv107x_edma_info[EDMA_MAX_CC] = {
	&edma_cc0_info,
};

static struct resource edma_resources[] = {
	{
		.name	= "edma_cc0",
		.start	= TNETV107X_TPCC_BASE,
		.end	= TNETV107X_TPCC_BASE + SZ_32K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma_tc0",
		.start	= TNETV107X_TPTC0_BASE,
		.end	= TNETV107X_TPTC0_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma_tc1",
		.start	= TNETV107X_TPTC1_BASE,
		.end	= TNETV107X_TPTC1_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma0",
		.start	= IRQ_TNETV107X_TPCC,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "edma0_err",
		.start	= IRQ_TNETV107X_TPCC_ERR,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device edma_device = {
	.name		= "edma",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(edma_resources),
	.resource	= edma_resources,
	.dev.platform_data = tnetv107x_edma_info,
};

static struct plat_serial8250_port serial_data[] = {
	{
		.mapbase	= TNETV107X_UART0_BASE,
		.irq		= IRQ_TNETV107X_UART0,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST |
					UPF_FIXED_TYPE | UPF_IOREMAP,
		.type		= PORT_AR7,
		.iotype		= UPIO_MEM32,
		.regshift	= 2,
	},
	{
		.mapbase	= TNETV107X_UART1_BASE,
		.irq		= IRQ_TNETV107X_UART1,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST |
					UPF_FIXED_TYPE | UPF_IOREMAP,
		.type		= PORT_AR7,
		.iotype		= UPIO_MEM32,
		.regshift	= 2,
	},
	{
		.mapbase	= TNETV107X_UART2_BASE,
		.irq		= IRQ_TNETV107X_UART2,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST |
					UPF_FIXED_TYPE | UPF_IOREMAP,
		.type		= PORT_AR7,
		.iotype		= UPIO_MEM32,
		.regshift	= 2,
	},
	{
		.flags	= 0,
	},
};

struct platform_device tnetv107x_serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev.platform_data	= serial_data,
};

static struct resource mmc0_resources[] = {
	{ /* Memory mapped registers */
		.start	= TNETV107X_SDIO0_BASE,
		.end	= TNETV107X_SDIO0_BASE + 0x0ff,
		.flags	= IORESOURCE_MEM
	},
	{ /* MMC interrupt */
		.start	= IRQ_TNETV107X_MMC0,
		.flags	= IORESOURCE_IRQ
	},
	{ /* SDIO interrupt */
		.start	= IRQ_TNETV107X_SDIO0,
		.flags	= IORESOURCE_IRQ
	},
	{ /* DMA RX */
		.start	= EDMA_CTLR_CHAN(0, TNETV107X_DMACH_SDIO0_RX),
		.flags	= IORESOURCE_DMA
	},
	{ /* DMA TX */
		.start	= EDMA_CTLR_CHAN(0, TNETV107X_DMACH_SDIO0_TX),
		.flags	= IORESOURCE_DMA
	},
};

static struct resource mmc1_resources[] = {
	{ /* Memory mapped registers */
		.start	= TNETV107X_SDIO1_BASE,
		.end	= TNETV107X_SDIO1_BASE + 0x0ff,
		.flags	= IORESOURCE_MEM
	},
	{ /* MMC interrupt */
		.start	= IRQ_TNETV107X_MMC1,
		.flags	= IORESOURCE_IRQ
	},
	{ /* SDIO interrupt */
		.start	= IRQ_TNETV107X_SDIO1,
		.flags	= IORESOURCE_IRQ
	},
	{ /* DMA RX */
		.start	= EDMA_CTLR_CHAN(0, TNETV107X_DMACH_SDIO1_RX),
		.flags	= IORESOURCE_DMA
	},
	{ /* DMA TX */
		.start	= EDMA_CTLR_CHAN(0, TNETV107X_DMACH_SDIO1_TX),
		.flags	= IORESOURCE_DMA
	},
};

static u64 mmc0_dma_mask = DMA_BIT_MASK(32);
static u64 mmc1_dma_mask = DMA_BIT_MASK(32);

static struct platform_device mmc_devices[2] = {
	{
		.name		= "davinci_mmc",
		.id		= 0,
		.dev		= {
			.dma_mask		= &mmc0_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
		.num_resources	= ARRAY_SIZE(mmc0_resources),
		.resource	= mmc0_resources
	},
	{
		.name		= "davinci_mmc",
		.id		= 1,
		.dev		= {
			.dma_mask		= &mmc1_dma_mask,
			.coherent_dma_mask	= DMA_BIT_MASK(32),
		},
		.num_resources	= ARRAY_SIZE(mmc1_resources),
		.resource	= mmc1_resources
	},
};

static const u32 emif_windows[] = {
	TNETV107X_ASYNC_EMIF_DATA_CE0_BASE, TNETV107X_ASYNC_EMIF_DATA_CE1_BASE,
	TNETV107X_ASYNC_EMIF_DATA_CE2_BASE, TNETV107X_ASYNC_EMIF_DATA_CE3_BASE,
};

static const u32 emif_window_sizes[] = { SZ_256M, SZ_64M, SZ_64M, SZ_64M };

static struct resource wdt_resources[] = {
	{
		.start	= TNETV107X_WDOG_BASE,
		.end	= TNETV107X_WDOG_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device tnetv107x_wdt_device = {
	.name		= "tnetv107x_wdt",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(wdt_resources),
	.resource	= wdt_resources,
};

static int __init nand_init(int chipsel, struct davinci_nand_pdata *data)
{
	struct resource res[2];
	struct platform_device *pdev;
	u32	range;
	int	ret;

	/* Figure out the resource range from the ale/cle masks */
	range = max(data->mask_cle, data->mask_ale);
	range = PAGE_ALIGN(range + 4) - 1;

	if (range >= emif_window_sizes[chipsel])
		return -EINVAL;

	pdev = kzalloc(sizeof(*pdev), GFP_KERNEL);
	if (!pdev)
		return -ENOMEM;

	pdev->name		= "davinci_nand";
	pdev->id		= chipsel;
	pdev->dev.platform_data	= data;

	memset(res, 0, sizeof(res));

	res[0].start	= emif_windows[chipsel];
	res[0].end	= res[0].start + range;
	res[0].flags	= IORESOURCE_MEM;

	res[1].start	= TNETV107X_ASYNC_EMIF_CNTRL_BASE;
	res[1].end	= res[1].start + SZ_4K - 1;
	res[1].flags	= IORESOURCE_MEM;

	ret = platform_device_add_resources(pdev, res, ARRAY_SIZE(res));
	if (ret < 0) {
		kfree(pdev);
		return ret;
	}

	return platform_device_register(pdev);
}

static struct resource keypad_resources[] = {
	{
		.start	= TNETV107X_KEYPAD_BASE,
		.end	= TNETV107X_KEYPAD_BASE + 0xff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_TNETV107X_KEYPAD,
		.flags	= IORESOURCE_IRQ,
		.name	= "press",
	},
	{
		.start	= IRQ_TNETV107X_KEYPAD_FREE,
		.flags	= IORESOURCE_IRQ,
		.name	= "release",
	},
};

static struct platform_device keypad_device = {
	.name		= "tnetv107x-keypad",
	.num_resources	= ARRAY_SIZE(keypad_resources),
	.resource	= keypad_resources,
};

static struct resource tsc_resources[] = {
	{
		.start	= TNETV107X_TSC_BASE,
		.end	= TNETV107X_TSC_BASE + 0xff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_TNETV107X_TSC,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tsc_device = {
	.name		= "tnetv107x-ts",
	.num_resources	= ARRAY_SIZE(tsc_resources),
	.resource	= tsc_resources,
};

static struct resource mdio_resources[] = {
	{
		.start	= TNETV107X_MDIO_BASE,
		.end	= TNETV107X_MDIO_BASE + 0xff,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device mdio_device = {
	.name		= "davinci_mdio",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(mdio_resources),
	.resource	= mdio_resources,
};

static u64 cpsw_dma_mask = DMA_BIT_MASK(32);

struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x14,
		.sliver_reg_ofs	= 0x80,
	},
	{
		.slave_reg_ofs	= 0x34,
		.sliver_reg_ofs	= 0xc0,
	},
};

struct cpsw_platform_data cpsw_data = {
	.channels		= 8,
	.cpdma_reg_ofs		= 0x100,
	.slaves			= 2,
	.slave_data		= cpsw_slaves,
	.ale_reg_ofs		= 0x500,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x54,
	.hw_stats_reg_ofs	= 0x400,
	.rx_descs		= 512,
	.mac_control		= BIT(18)	| /* IFCTLA	*/
				  BIT(15)	| /* EXTEN	*/
				  BIT(5)	| /* MIIEN	*/
				  BIT(4)	| /* TXFLOWEN	*/
				  BIT(3),	  /* RXFLOWEN	*/
};

static struct resource cpsw_resources[] = {
	{
		.start	= TNETV107X_CPSW_BASE,
		.end	= TNETV107X_CPSW_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_TNETV107X_ETHSS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device cpsw_device = {
	.name			= "cpsw",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(cpsw_resources),
	.resource		= cpsw_resources,
	.dev = {
		.platform_data		= &cpsw_data,
		.dma_mask		= &cpsw_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};

static struct resource ssp_resources [] = {
	{
		.start	= TNETV107X_SSP_BASE,
		.end	= TNETV107X_SSP_BASE + 0x1ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_TNETV107X_SSP,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device ssp_device = {
	.name		= "ti-ssp",
	.num_resources	= ARRAY_SIZE(ssp_resources),
	.resource	= ssp_resources,
};


#ifdef	CONFIG_CPPI41
static const struct cppi41_tx_ch tx_ch_info[] = {
	[0] = {
		.port_num	= 1,
		.num_tx_queue	= 2,
		.tx_queue	= { { 0, 16 }, { 0, 17 } }
	},
	[1] = {
		.port_num	= 2,
		.num_tx_queue	= 2,
		.tx_queue	= { { 0, 18 }, { 0, 19 } }
	},
	[2] = {
		.port_num	= 3,
		.num_tx_queue	= 2,
		.tx_queue	= { { 0, 20 }, { 0, 21 } }
	},
	[3] = {
		.port_num	= 4,
		.num_tx_queue	= 2,
		.tx_queue	= { { 0, 22 }, { 0, 23 } }
	}
};

/* DMA block configuration */
const struct cppi41_dma_block cppi41_dma_block[1] = {
	[0] = {
		.global_ctrl_base	= 0xfee21000,
		.ch_ctrl_stat_base	= 0xfee21800,
		.sched_ctrl_base	= 0xfee22000,
		.sched_table_base	= 0xfee22800,
		.num_tx_ch		= 4,
		.num_rx_ch		= 4,
		.tx_ch_info		= tx_ch_info
	}
};
EXPORT_SYMBOL(cppi41_dma_block);

/* Queues 0 to 27 are pre-assigned, others are spare */
static const u32 assigned_queues[] = { 0x0fffffff, 0 };

/* Queue manager information */
const struct cppi41_queue_mgr cppi41_queue_mgr[1] = {
	[0] = {
		.q_mgr_rgn_base 	= 0xfee24000,
		.desc_mem_rgn_base	= 0xfee25000,
		.q_mgmt_rgn_base	= 0xfee26000,
		.q_stat_rgn_base	= 0xfee26400,

		.num_queue		= 64,
		.queue_types		= CPPI41_FREE_DESC_BUF_QUEUE |
					  CPPI41_UNASSIGNED_QUEUE,
		.base_fdbq_num		= 0,
		.assigned		= assigned_queues
	}
};

const u8 cppi41_num_queue_mgr = 1;
const u8 cppi41_num_dma_block = 1;

/* Fair DMA scheduling */
static const u8 dma_sched_table[] = {
	0x00, 0x80, 0x01, 0x81, 0x02, 0x82, 0x03, 0x83
};

int __init tnetv107x_cppi41_init(void)
{
	int ret;
	pr_debug("initialising cppi41\n");
	ret = cppi41_queue_mgr_init(0, 0, 0);
	if (ret) {
		pr_warning("%s: queue manager initialization failed: %d\n",
			   __func__, ret);
		return ret;
	}

	ret = cppi41_dma_ctrlr_init(0, 0, 5);
	if (ret) {
		pr_warning("%s: DMA controller initialization failed: %d\n",
			   __func__, ret);
		return ret;
	}

	ret = cppi41_dma_sched_init(0, dma_sched_table,
				    sizeof(dma_sched_table));
	if (ret)
		printk("%s: DMA scheduler initialization failed: %d\n",
		       __func__, ret);
	return ret;
}
EXPORT_SYMBOL(tnetv107x_cppi41_init);

#endif	/* CONFIG_CPPI41 */


void __init tnetv107x_devices_init(struct tnetv107x_device_info *info)
{
	int i;

	platform_device_register(&edma_device);
	platform_device_register(&mdio_device);
	platform_device_register(&tnetv107x_wdt_device);
	platform_device_register(&tsc_device);

	if (info->serial_config)
		davinci_serial_init(info->serial_config);

	for (i = 0; i < 2; i++)
		if (info->mmc_config[i]) {
			mmc_devices[i].dev.platform_data = info->mmc_config[i];
			platform_device_register(&mmc_devices[i]);
		}

	for (i = 0; i < 4; i++)
		if (info->nand_config[i])
			nand_init(i, info->nand_config[i]);

	if (info->keypad_config) {
		keypad_device.dev.platform_data = info->keypad_config;
		platform_device_register(&keypad_device);
	}

	if (info->cpsw_config) {
		for (i = 0; i < 2; i++) {
			cpsw_slaves[i].phy_id = info->cpsw_config->phy_id[i];
			cpsw_slaves[i].phy_if = info->cpsw_config->phy_if[i] ?
						info->cpsw_config->phy_if[i] :
						PHY_INTERFACE_MODE_RGMII_ID;
		}
		if (info->cpsw_config->rx_descs)
			cpsw_data.rx_descs = info->cpsw_config->rx_descs;
		cpsw_data.phy_control = info->cpsw_config->phy_control;
		memcpy(&cpsw_data.mac_addr, info->cpsw_config->mac_addr,
		       ETH_ALEN);
		platform_device_register(&cpsw_device);
	}

	if (info->ssp_config) {
		ssp_device.dev.platform_data = info->ssp_config;
		platform_device_register(&ssp_device);
	}
//#ifdef CONFIG_CPPI41
//	tnetv107x_cppi41_init();
//#endif
}
