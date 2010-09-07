/*
 * Sequencer Serial Port (SSP) driver for Texas Instruments' SoCs
 *
 * Copyright (C) 2010 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <mach/ti_ssp.h>

/* Register Offsets */
#define SSP_REG_REV		0x00
#define SSP_REG_IOSEL_1		0x04
#define SSP_REG_IOSEL_2		0x08
#define SSP_REG_PREDIV		0x0c
#define SSP_REG_INTR_STAT	0x10
#define SSP_REG_INTR_EN		0x14
#define SSP_REG_TEST_CTRL	0x18

/* Per port registers */
#define SSP_PORT_REG_CFG_2	0x00
#define SSP_PORT_REG_ADDR	0x04
#define SSP_PORT_REG_DATA	0x08
#define SSP_PORT_REG_CFG_1	0x0c
#define SSP_PORT_REG_STATE	0x10

#define SSP_PORT_CONFIG_MASK	(SSP_EARLY_DIN | SSP_DELAY_DOUT)
#define SSP_PORT_CLKRATE_MASK	0x0f

#define SSP_SEQRAM_WR_EN	(1 << 4)
#define SSP_SEQRAM_RD_EN	(1 << 5)
#define SSP_START		(1 << 15)
#define SSP_BUSY		(1 << 10)
#define SSP_PORT_ASL		(1 << 7)
#define SSP_PORT_CFO1		(1 << 6)

#define SSP_PORT_SEQRAM_SIZE	32

static int ssp_port_base[]   = {0x040, 0x080};
static int ssp_port_seqram[] = {0x100, 0x180};

/* Register Access Macros */
#define ssp_read(ssp, reg)						\
	__raw_readl((ssp)->virt_base + SSP_REG_##reg)

#define ssp_write(ssp, reg, val)					\
	__raw_writel(val, (ssp)->virt_base + SSP_REG_##reg)

#define ssp_rmw(ssp, reg, mask, bits)					\
	ssp_write(ssp, reg, (ssp_read(ssp, reg) & ~(mask)) | (bits))

#define ssp_port_read(ssp, port, reg)					\
	__raw_readl((ssp)->virt_base + ssp_port_base[port] +		\
		    SSP_PORT_REG_##reg)

#define ssp_port_write(ssp, port, reg, val)				\
	__raw_writel(val, (ssp)->virt_base + ssp_port_base[port] +	\
		     SSP_PORT_REG_##reg)

#define ssp_port_rmw(ssp, port, reg, mask, bits)			\
	ssp_port_write(ssp, port, reg, (ssp_port_read(ssp, port, reg) &	\
		       ~(mask)) | (bits))

#define ssp_port_clr_bits(ssp, port, reg, bits)	\
	ssp_port_rmw(ssp, port, reg, bits, 0)

#define ssp_port_set_bits(ssp, port, reg, bits)	\
	ssp_port_rmw(ssp, port, reg, 0, bits)

struct ti_ssp {
	struct ti_ssp_data		data;
	struct resource			*res;
	void __iomem			*virt_base;
	struct timer_list		timer;
	struct ti_ssp_device		*owner[2];
	spinlock_t			lock;
	struct clk			*clk;
	struct device			*dev;
	int				prediv;
	bool				suspended:1;
	struct completion		complete;
};

/* Helper routine for ti_ssp_find() {see below} */
struct lookup {
	const char		*name;
	struct ti_ssp_device	*device;
};

/* Helper routine for ti_ssp_find() below... */
static int _ti_ssp_match(struct device *dev, void *data)
{
	struct lookup *lk = data;
	struct platform_device *pdev = to_platform_device(dev);
	struct ti_ssp *ssp;
	int i;

	if (strcmp(pdev->name, "ti-ssp"))
		return 0;

	ssp = (struct ti_ssp *)dev_get_drvdata(dev);
	if (!ssp)
		return 0;

	for (i = 0; i < ssp->data.num_devices; i++) {
		if (strcmp(ssp->data.devices[i].name, lk->name) == 0) {
			lk->device = &ssp->data.devices[i];
			return 1;
		}
	}

	return 0;
}

/*
 * Find an SSP device that matches a specified name.
 *
 * When a protocol driver (e.g. spi) attempts to open its SSP device, we don't
 * have a direct handle to the SSP private data.  Maintaining a global pointer
 * to the SSP data would restrict us to a single SSP instance.  Therefore, we
 * scan the platform bus, looking for SSP instances and searching for the
 * appropriate device to open.
 */
static struct ti_ssp_device *ti_ssp_find(const char *name)
{
	struct lookup lk = { name, NULL };
	bus_for_each_dev(&platform_bus_type, NULL, &lk, _ti_ssp_match);
	return lk.device;
}

/*
 * Calculate the lowest possible predivider value that can then be further
 * divided down into output clocks within the constraints of device min and
 * max specifications
 */
static int ti_ssp_prediv(struct ti_ssp *ssp)
{
	unsigned long sys_clk = clk_get_rate(ssp->clk);
	unsigned long mod_clk = 0, out = 0;
	int i;

	for (ssp->prediv = 0; ssp->prediv <= 0xff; ssp->prediv++) {
		bool in_range = true;

		mod_clk = sys_clk / (ssp->prediv + 1);
		out = mod_clk / 2; /* Per-port CLKRATE always 1 */

		for (i = 0; i < ssp->data.num_devices; i++) {
			if ((ssp->data.devices[i].min_clock >= out) ||
			    (ssp->data.devices[i].max_clock <= out)) {
				in_range = false;
				break;
			}
		}

		if (in_range)
			break;
	}

	if (ssp->prediv > 0xff) {
		dev_err(ssp->dev, "could not determine predivider\n");
		return -EINVAL;
	}

	dev_dbg(ssp->dev, "programmed for %lu Hz output clock\n", out);
	return 0;
}

struct ti_ssp_device *ti_ssp_open(const char* name)
{
	struct ti_ssp	*ssp;
	unsigned long	flags;
	int		ret = 0;
	unsigned int	val;
	struct ti_ssp_device *dev;

	/* Find associated controller and matching device */
	ret = -ENOENT;
	dev = ti_ssp_find(name);
	if (!dev)
		goto fail0;

	ssp = dev->priv;
	if (!ssp)
		goto fail0;

	ret = -ESHUTDOWN;
	if (ssp->suspended)
		goto fail0;

	spin_lock_irqsave(&ssp->lock, flags);

	ret = -EINVAL;
	if (!get_device(ssp->dev))
		goto fail1;

	/* Is the requested port free? */
	if (ssp->owner[dev->port]) {
		/*
		 * port is in use, for now we simply bail out, later we may
		 * want to wait for completion
		 */
		dev_err(ssp->dev, "attempted open for device %s failed "
			"on port %d, owner=%s\n",
			name, dev->port, ssp->owner[dev->port]->name);
		ret = -EINPROGRESS;
		goto fail2;
	}
	ssp->owner[dev->port] = dev;

	if (ssp->data.select)
		ssp->data.select(dev->id, 1);

	/* IOSEL1 gets the least significant 16 bits */
	val = ssp_read(ssp, IOSEL_1);
	val &= 0xffff << (dev->port ? 0 : 16);
	val |= (dev->iosel & 0xffff) << (dev->port ? 16 : 0);
	ssp_write(ssp, IOSEL_1, val);

	/* IOSEL2 gets the most significant 16 bits */
	val = ssp_read(ssp, IOSEL_2);
	val &= 0x0007 << (dev->port ? 0 : 16);
	val |= (dev->iosel & 0x00070000) >> (dev->port ? 0 : 16);
	ssp_write(ssp, IOSEL_2, val);

	ssp_port_rmw(ssp, dev->port, CFG_1, SSP_PORT_CONFIG_MASK, dev->config);
	ssp_port_rmw(ssp, dev->port, CFG_2, SSP_PORT_CLKRATE_MASK, 0);

	spin_unlock_irqrestore(&ssp->lock, flags);
	return dev;

fail2:
	put_device(ssp->dev);
fail1:
	spin_unlock_irqrestore(&ssp->lock, flags);
fail0:
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(ti_ssp_open);

int ti_ssp_load(struct ti_ssp_device *dev, int offs, u32* prog, int len)
{
	struct ti_ssp *ssp;
	int i;

	if (!dev || !dev->priv)
		return -EINVAL;
	ssp = dev->priv;

	if (len > SSP_PORT_SEQRAM_SIZE)
		return -ENOSPC;

	/* Enable SeqRAM access */
	ssp_port_set_bits(ssp, dev->port, CFG_2, SSP_SEQRAM_WR_EN);

	/* Copy code */
	for (i = 0; i < len; i++) {
		__raw_writel(prog[i], ssp->virt_base + offs + 4*i +
			     ssp_port_seqram[dev->port]);
	}

	/* Disable SeqRAM access */
	ssp_port_clr_bits(ssp, dev->port, CFG_2, SSP_SEQRAM_WR_EN);

	return 0;
}
EXPORT_SYMBOL(ti_ssp_load);

int ti_ssp_raw_read(struct ti_ssp_device *dev)
{
	struct ti_ssp *ssp;
	u32 val;

	if (!dev || !dev->priv)
		return -EINVAL;
	ssp = dev->priv;

	val = ssp_read(ssp, IOSEL_2);
	val >>= (dev->port ? 27 : 11);
	return val & 0x0f;
}
EXPORT_SYMBOL(ti_ssp_raw_read);

int ti_ssp_raw_write(struct ti_ssp_device *dev, u32 val)
{
	struct ti_ssp *ssp;
	u32 mask;

	if (!dev || !dev->priv)
		return -EINVAL;
	ssp = dev->priv;

	val &= 0x0f;
	val <<= (dev->port ? 22 : 6);
	mask = 0x0f;
	mask <<= (dev->port ? 22 : 6);
	ssp_rmw(ssp, IOSEL_2, mask, val);

	return 0;
}
EXPORT_SYMBOL(ti_ssp_raw_write);

int ti_ssp_run(struct ti_ssp_device *dev, u32 pc, u32 input, u32 *output)
{
	struct ti_ssp *ssp;
	int count;

	if (!dev || !dev->priv)
		return -EINVAL;
	ssp = dev->priv;

	if (pc & ~(0x3f))
		return -EINVAL;

	ssp_port_write(ssp, dev->port, ADDR, input >> 16);
	ssp_port_write(ssp, dev->port, DATA, input & 0xffff);
	ssp_port_rmw(ssp, dev->port, CFG_1, 0x3f, pc);

	ssp_port_set_bits(ssp, dev->port, CFG_1, SSP_START);

	for (count = 10000; count; count--) {
		if ((ssp_port_read(ssp, dev->port, CFG_1) & SSP_BUSY) == 0)
			break;
		udelay(1);
	}

	if (output) {
		*(output) = (ssp_port_read(ssp, dev->port, ADDR) << 16) |
			    (ssp_port_read(ssp, dev->port, DATA) &  0xffff);
	}

	if (!count) {
		dev_err(ssp->dev, "timed out waiting for SSP operation\n");
		return -EIO;
	}

	/* return stop address */
	return ssp_port_read(ssp, dev->port, STATE) & 0x3f;
}
EXPORT_SYMBOL(ti_ssp_run);

int ti_ssp_close(struct ti_ssp_device *dev)
{
	struct ti_ssp	*ssp;
	unsigned long	flags;

	if (!dev || !dev->priv)
		return -EINVAL;
	ssp = dev->priv;

	spin_lock_irqsave(&ssp->lock, flags);

	if (ssp->data.select)
		ssp->data.select(dev->id, 0);

	ssp->owner[dev->port] = NULL;

	spin_unlock_irqrestore(&ssp->lock, flags);

	put_device(ssp->dev);

	complete(&ssp->complete);

	return 0;
}
EXPORT_SYMBOL(ti_ssp_close);

static int ti_ssp_probe(struct platform_device *pdev)
{
	struct ti_ssp_data *pdata = pdev->dev.platform_data;
	struct ti_ssp *ssp;
	int ret = 0, i;
	struct device *dev = &pdev->dev;
	struct ti_ssp_device *devices;

	ssp = kmalloc(sizeof(struct ti_ssp), GFP_KERNEL);
	if (!ssp) {
		dev_err(dev, "cannot allocate device info\n");
		return -ENOMEM;
	}

	memset(ssp, 0, sizeof(struct ti_ssp));

	ret = -EINVAL;
	if (!pdata) {
		dev_err(dev, "cannot find device data\n");
		goto error0;
	}
	ssp->data = *pdata;
	ssp->dev = dev;

	ret = -ENOMEM;
	devices = kmalloc(sizeof(struct ti_ssp_device)*
			ssp->data.num_devices, GFP_KERNEL);
	if (!devices)
		goto error0;

	memcpy(devices, ssp->data.devices,
			sizeof(struct ti_ssp_device) *
			ssp->data.num_devices);
	ssp->data.devices = devices;

	for (i = 0; i < ssp->data.num_devices; i++)
		ssp->data.devices[i].priv = ssp;

	platform_set_drvdata(pdev, ssp);

	ret = -ENODEV;
	ssp->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!ssp->res) {
		dev_err(dev, "cannot determine register area\n");
		goto error1;
	}

	ret = -EINVAL;
	if (!request_mem_region(ssp->res->start, resource_size(ssp->res),
				pdev->name)) {
		dev_err(dev, "cannot claim register memory\n");
		goto error1;
	}

	ret = -ENOMEM;
	ssp->virt_base = ioremap(ssp->res->start, resource_size(ssp->res));
	if (!ssp->virt_base) {
		dev_err(dev, "cannot map register memory\n");
		goto error2;
	}

	ret = -EINVAL;
	ssp->clk = clk_get(dev, NULL);
	if (IS_ERR(ssp->clk)) {
		dev_err(dev, "cannot claim device clock (%s)\n", dev_name(dev));
		goto error3;
	}

	/* calculate and program predivider */
	ret = ti_ssp_prediv(ssp);
	if (ret)
		goto error4;

	spin_lock_init(&ssp->lock);
	init_completion(&ssp->complete);

	dev_set_drvdata(&pdev->dev, ssp);

	/* Power on and initialize SSP */
	ret = clk_enable(ssp->clk);
	if (ret)
		goto error4;

	/* Reset registers to a sensible known state */
	ssp_write(ssp, IOSEL_1, 0);
	ssp_write(ssp, IOSEL_2, 0);
	ssp_write(ssp, INTR_EN, 0);
	ssp_write(ssp, TEST_CTRL, 0);
	ssp_port_write(ssp, 0, CFG_1, SSP_PORT_ASL);
	ssp_port_write(ssp, 1, CFG_1, SSP_PORT_ASL);
	ssp_port_write(ssp, 0, CFG_2, SSP_PORT_CFO1);
	ssp_port_write(ssp, 1, CFG_2, SSP_PORT_CFO1);
	ssp_rmw(ssp, PREDIV, 0xff, ssp->prediv);

	return 0;

error4:
	clk_put(ssp->clk);
error3:
	iounmap(ssp->virt_base);
error2:
	release_mem_region(ssp->res->start, resource_size(ssp->res));
error1:
	kfree(ssp->data.devices);
error0:
	kfree(ssp);
	return ret;
}

static int ti_ssp_remove(struct platform_device *pdev)
{
	struct ti_ssp *ssp = dev_get_drvdata(&pdev->dev);

	if (ssp) {
		clk_disable(ssp->clk);
		clk_put(ssp->clk);
		iounmap(ssp->virt_base);
		release_mem_region(ssp->res->start, resource_size(ssp->res));
		kfree(ssp->data.devices);
		kfree(ssp);
	}
	return 0;
}

static int ti_ssp_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ti_ssp *ssp = dev_get_drvdata(&pdev->dev);
	int ret;

	if (!ssp)
		return -ENODEV;

	ssp->suspended = true;
	while (1) {
		if (!ssp->owner[0] && !ssp->owner[1])
			break;
		ret = wait_for_completion_interruptible(&ssp->complete);
		if (ret < 0) {
			ssp->suspended = false;
			return ret;
		}
	}

	clk_disable(ssp->clk);

	return 0;
}

static int ti_ssp_resume(struct platform_device *pdev)
{
	struct ti_ssp *ssp = dev_get_drvdata(&pdev->dev);

	if (!ssp)
		return -ENODEV;
	clk_enable(ssp->clk);
	ssp->suspended = false;
	return 0;
}

static struct platform_driver ti_ssp_driver = {
	.probe		= ti_ssp_probe,
	.remove		= ti_ssp_remove,
	.suspend	= ti_ssp_suspend,
	.resume		= ti_ssp_resume,
	.driver = {
		.name	= "ti-ssp",
		.owner	= THIS_MODULE,
	},
};

static int __init ti_ssp_init(void)
{
	return platform_driver_register(&ti_ssp_driver);
}

static void __exit ti_ssp_exit(void)
{
	platform_driver_unregister(&ti_ssp_driver);
}

subsys_initcall(ti_ssp_init);
module_exit(ti_ssp_exit);

MODULE_DESCRIPTION("Sequencer Serial Port (SSP) Driver");
MODULE_LICENSE("GPL");
