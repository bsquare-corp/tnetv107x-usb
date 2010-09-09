/*
 * Texas Instruments TNETV107X Keypad Driver
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
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>

#include <mach/tnetv107x.h>

#define KEYPAD_ROWS		9
#define KEYPAD_COLS		9

struct keypad_regs {
	u32	rev;
	u32	mode;
	u32	mask;
	u32	pol;
	u32	dclock;
	u32	rclock;
	u32	stable_cnt;
	u32	in_en;
	u32	out;
	u32	out_en;
	u32	in;
	u32	lock;
	u32	pres[3];
};

#define keypad_read(kp, reg)		__raw_readl(&(kp)->regs->reg)
#define keypad_write(kp, reg, val)	__raw_writel(val, &(kp)->regs->reg)

struct keypad_data {
	struct tnetv107x_keypad_data	data;
	struct input_dev		*input_dev;
	struct resource			*res;
	struct keypad_regs __iomem	*regs;
	struct clk			*clk;
	struct device			*dev;
	u32				irq_press;
	u32				irq_release;
	u32				curr_keys[3];
	u32				prev_keys[3];
};

static void handle_change(struct keypad_data *kp)
{
	int bit, i;

	for (bit = 0; bit < (KEYPAD_ROWS * KEYPAD_COLS); bit++) {
		int idx		= bit / 32;
		u32 mask	= 1 << (bit % 32);
		u32 curr	= kp->curr_keys[idx] & mask;
		u32 prev	= kp->prev_keys[idx] & mask;
		int row		= bit / KEYPAD_COLS;
		int col		= bit % KEYPAD_COLS;
		int ofs		= row * kp->data.cols + col;

		if (col >= kp->data.cols || row >= kp->data.rows)
			continue;

		if (curr && !prev) {
			/* Report key press */
			if (kp->data.keynames && kp->data.keynames[ofs])
				dev_dbg(kp->dev, "%s (%d) pressed\n",
					kp->data.keynames[ofs], ofs);
			input_report_key(kp->input_dev,
					 kp->data.keymap[ofs], 1);
		} else if (!curr && prev) {
			/* Report key release */
			if (kp->data.keynames && kp->data.keynames[ofs])
				dev_dbg(kp->dev, "%s (%d) released\n",
					kp->data.keynames[ofs], ofs);
			input_report_key(kp->input_dev,
					 kp->data.keymap[ofs], 0);
		}
	}

	/* Update shadow copy */
	for (i = 0; i < 3; i++)
		kp->prev_keys[i] = kp->curr_keys[i];

	input_sync(kp->input_dev);
}

static irqreturn_t keypad_irq_press(int irq, void *data)
{
	struct keypad_data *kp = (struct keypad_data *)data;
	int i;

	for (i = 0; i < 3; i++)
		kp->curr_keys[i] = keypad_read(kp, pres[i]);
	handle_change(kp);
	keypad_write(kp, lock, 0); /* Allow hardware updates */
	return IRQ_HANDLED;
}

static irqreturn_t keypad_irq_release(int irq, void *data)
{
	struct keypad_data *kp = (struct keypad_data *)data;
	int i;

	/* Hardware says all keys have been released */
	for (i = 0; i < 3; i++)
		kp->curr_keys[i] = 0;
	handle_change(kp);
	return IRQ_HANDLED;
}

static int tnetv107x_keypad_probe(struct platform_device *pdev)
{
	struct tnetv107x_keypad_data *pdata = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct keypad_data *kp;
	int i, ret = 0;
	u32 rev = 0;

	ret = -EINVAL;
	if (!pdata) {
		dev_err(dev, "cannot find device data\n");
		return ret;
	}

	ret = -ENOMEM;
	kp = kzalloc(sizeof(struct keypad_data), GFP_KERNEL);
	if (!kp) {
		dev_err(dev, "cannot allocate device info\n");
		return ret;
	}

	dev_set_drvdata(dev, kp);
	kp->data = *pdata;
	kp->dev = dev;

	ret = -ENOMEM;
	kp->input_dev = input_allocate_device();
	if (!kp->input_dev) {
		dev_err(dev, "cannot allocate input device\n");
		goto error0;
	}

	ret = -ENODEV;
	kp->irq_press   = platform_get_irq_byname(pdev, "press");
	kp->irq_release = platform_get_irq_byname(pdev, "release");
	if (kp->irq_press < 0 || kp->irq_release < 0) {
		dev_err(dev, "cannot determine device interrupts\n");
		goto error1;
	}

	ret = -ENODEV;
	kp->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!kp->res) {
		dev_err(dev, "cannot determine register area\n");
		goto error1;
	}

	ret = -EINVAL;
	if (!request_mem_region(kp->res->start, resource_size(kp->res),
				pdev->name)) {
		dev_err(dev, "cannot claim register memory\n");
		goto error1;
	}

	ret = -ENOMEM;
	kp->regs = ioremap(kp->res->start, resource_size(kp->res));
	if (!kp->regs) {
		dev_err(dev, "cannot map register memory\n");
		goto error2;
	}

	ret = -EINVAL;
	kp->clk = clk_get(dev, NULL);
	if (!kp->clk) {
		dev_err(dev, "cannot claim device clock\n");
		goto error3;
	}
	clk_enable(kp->clk);

	/* Initialize device registers */
	keypad_write(kp, mode, 0);
	keypad_write(kp, mask, ~((((1 << kp->data.rows)-1) << 9) |
			   (((1 << kp->data.cols)-1))));
	keypad_write(kp, pol, 0x3ffff);
	keypad_write(kp, dclock, kp->data.debounce);
	keypad_write(kp, rclock, 4*kp->data.debounce);
	keypad_write(kp, stable_cnt, kp->data.stable);

	/* Enable Input */
	keypad_write(kp, in_en, 0);
	mdelay(1);
	keypad_write(kp, in_en, 1);

	ret = request_irq(kp->irq_press, keypad_irq_press, 0,
			  "keypad-press", kp);
	if (ret < 0) {
		dev_err(dev, "Could not allocate keypad press key irq\n");
		goto error4;
	}

	ret = request_irq(kp->irq_release, keypad_irq_release, 0,
			  "keypad-release", kp);
	if (ret < 0) {
		dev_err(dev, "Could not allocate keypad release key irq\n");
		goto error5;
	}

	set_bit(EV_KEY, kp->input_dev->evbit);
	for (i = 0; i < kp->data.keymap_size; i++)
		set_bit(kp->data.keymap[i] & KEY_MAX, kp->input_dev->keybit);

	kp->input_dev->name = "tnetv107x-keypad";
	kp->input_dev->phys = "tnetv107x-keypad/input0";
	kp->input_dev->dev.parent = &pdev->dev;
	kp->input_dev->id.bustype = BUS_HOST;
	kp->input_dev->id.vendor = 0x0001;

	rev = keypad_read(kp, rev);
	kp->input_dev->id.product = ((rev >>  8) & 0x07);
	kp->input_dev->id.version = ((rev >> 16) & 0xfff);

	kp->input_dev->keycode = kp->data.keymap;
	kp->input_dev->keycodesize = sizeof(int);
	kp->input_dev->keycodemax = kp->data.keymap_size;
	ret = input_register_device(kp->input_dev);
	if (ret < 0) {
		dev_err(dev, "Could not register input device\n");
		goto error6;
	}

	dev_info(dev, "registered keypad device\n");
	return 0;

error6:
	free_irq(kp->irq_release, kp);
error5:
	free_irq(kp->irq_press, kp);
error4:
	clk_disable(kp->clk);
	clk_put(kp->clk);
error3:
	iounmap(kp->regs);
error2:
	release_mem_region(kp->res->start, resource_size(kp->res));
error1:
	input_free_device(kp->input_dev);
error0:
	platform_set_drvdata(pdev, NULL);
	kfree(kp);
	return ret;
}

static int tnetv107x_keypad_remove(struct platform_device *pdev)
{
	struct keypad_data *kp = dev_get_drvdata(&pdev->dev);

	if (kp) {
		input_unregister_device(kp->input_dev);
		free_irq(kp->irq_release, kp);
		free_irq(kp->irq_press, kp);
		clk_disable(kp->clk);
		clk_put(kp->clk);
		iounmap(kp->regs);
		release_mem_region(kp->res->start, resource_size(kp->res));
		input_free_device(kp->input_dev);
		platform_set_drvdata(pdev, NULL);
		kfree(kp);
	}
	return 0;
}

static int tnetv107x_keypad_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	/* Nothing yet */
	return 0;
}

static int tnetv107x_keypad_resume(struct platform_device *pdev)
{
	/* Nothing yet */
	return 0;
}

static struct platform_driver tnetv107x_keypad_driver = {
	.probe		= tnetv107x_keypad_probe,
	.remove		= tnetv107x_keypad_remove,
	.suspend	= tnetv107x_keypad_suspend,
	.resume		= tnetv107x_keypad_resume,
	.driver.name	= "tnetv107x-keypad",
};

static int __init tnetv107x_keypad_init(void)
{
	return platform_driver_register(&tnetv107x_keypad_driver);
}

static void __exit tnetv107x_keypad_exit(void)
{
	platform_driver_unregister(&tnetv107x_keypad_driver);
}

module_init(tnetv107x_keypad_init);
module_exit(tnetv107x_keypad_exit);

MODULE_AUTHOR("Cyril Chemparathy");
MODULE_DESCRIPTION("TNETV107X Keypad Driver");
MODULE_LICENSE("GPL");
