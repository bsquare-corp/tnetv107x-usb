/*
 * Texas Instruments TNETV107X Touchscreen Driver
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
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/clk.h>

#include <mach/tnetv107x.h>

/* Poll Rates */
#define TSC_PENUP_POLL		(HZ / 5)

/*
 * The first and last samples of a touch interval are usually garbage and need
 * to be filtered out with these devices.  The following definitions control
 * the number of samples skipped.
 */
#define TSC_HEAD_SKIP		1
#define TSC_TAIL_SKIP		1
#define TSC_SKIP		(TSC_HEAD_SKIP + TSC_TAIL_SKIP + 1)
#define TSC_SAMPLES		(TSC_SKIP + 1)

/* Register Offsets */
struct tsc_regs {
	u32	rev;
	u32	tscm;
	u32	bwcm;
	u32	swc;
	u32	adcchnl;
	u32	adcdata;
	u32	chval[4];
};

/* TSC Mode Configuration Register (tscm) bits */
#define WMODE		BIT(0)
#define TSKIND		BIT(1)
#define ZMEASURE_EN	BIT(2)
#define IDLE		BIT(3)
#define TSC_EN		BIT(4)
#define STOP		BIT(5)
#define ONE_SHOT	BIT(6)
#define SINGLE		BIT(7)
#define AVG		BIT(8)
#define AVGNUM(x)	(((x) & 0x03) <<  9)
#define PVSTC(x)	(((x) & 0x07) << 11)
#define PON		BIT(14)
#define PONBG		BIT(15)
#define AFERST		BIT(16)

/* ADC DATA Capture Register bits */
#define DATA_VALID	BIT(16)

/* Register Access Macros */
#define tsc_read(ts, reg)		__raw_readl(&(ts)->regs->reg)
#define tsc_write(ts, reg, val)		__raw_writel(val, &(ts)->regs->reg);
#define tsc_set_bits(ts, reg, val)	\
	tsc_write(ts, reg, tsc_read(ts, reg) | (val))
#define tsc_clr_bits(ts, reg, val)	\
	tsc_write(ts, reg, tsc_read(ts, reg) & ~(val))

struct sample {
	int x, y, p;
};

struct tsc_data {
	struct tnetv107x_tsc_data	data;
	struct input_dev		*input_dev;
	struct resource			*res;
	struct tsc_regs __iomem		*regs;
	struct timer_list		timer;
	spinlock_t			lock;
	struct clk			*clk;
	struct device			*dev;
	int				sample_count;
	struct sample			samples[TSC_SAMPLES];
	int				tsc_irq;
	int				cal[TSC_CAL_SIZE];
};

/* default calibration that works for most evm boards */
static int tscal_set;
static int tscal[TSC_CAL_SIZE];
module_param_array(tscal, int, &tscal_set, 0);

static inline int
tsc_read_sample(struct tsc_data *ts, struct sample* sample)
{
	int	x, y, z1, z2, t, p = 0;
	u32	val;

	val = tsc_read(ts, chval[0]);
	if (val & DATA_VALID)
		x = val & 0xffff;
	else
		return -EINVAL;

	y  = tsc_read(ts, chval[1]) & 0xffff;
	z1 = tsc_read(ts, chval[2]) & 0xffff;
	z2 = tsc_read(ts, chval[3]) & 0xffff;

	if (z1) {
		t = ((600 * x) * (z2 - z1));
		p = t / (u32) (z1 << 12);
		if (p < 0)
			p = 0;
	}

	sample->x  = (ts->cal[2] + ts->cal[0] * x + ts->cal[1] * y);
	sample->x /= ts->cal[6];
	sample->y  = (ts->cal[5] + ts->cal[3] * x + ts->cal[4] * y);
	sample->y /= ts->cal[6];
	sample->p  = p;

	return 0;
}

static inline void tsc_report_up(struct tsc_data *ts)
{
	input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);
}

static inline void tsc_report(struct tsc_data *ts, int x, int y, int p)
{
	input_report_abs(ts->input_dev, ABS_X, x);
	input_report_abs(ts->input_dev, ABS_Y, y);
	input_report_abs(ts->input_dev, ABS_PRESSURE, p);
}

static inline void tsc_report_down(struct tsc_data *ts, bool touch)
{
	if (touch)
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_sync(ts->input_dev);
}

static inline void tsc_poll(unsigned long data)
{
	struct tsc_data *ts = (struct tsc_data *)data;
	unsigned long flags;
	int i, val, x, y, p;

	spin_lock_irqsave(&ts->lock, flags);

	if (ts->sample_count >= TSC_SKIP) {
		tsc_report_up(ts);
	} else if (ts->sample_count > 0) {
		/*
		 * A touch event lasted less than our skip count.  Salvage and
		 * report anyway.
		 */
		for (i = 0, val = 0; i < ts->sample_count; i++)
			val += ts->samples[i].x;
		x = val / ts->sample_count;

		for (i = 0, val = 0; i < ts->sample_count; i++)
			val += ts->samples[i].y;
		y = val / ts->sample_count;

		for (i = 0, val = 0; i < ts->sample_count; i++)
			val += ts->samples[i].p;
		p = val / ts->sample_count;

		tsc_report(ts, x, y, p);
		tsc_report_down(ts, true);
	}

	ts->sample_count = 0;

	spin_unlock_irqrestore(&ts->lock, flags);
}

static irqreturn_t tsc_irq(int irq, void *dev_id)
{
	struct tsc_data *ts = (struct tsc_data *)dev_id;
	struct sample *sample;
	int index;

	spin_lock(&ts->lock);

	index = ts->sample_count % TSC_SAMPLES;
	sample = &ts->samples[index];
	if (tsc_read_sample(ts, sample) >= 0) {
		++ts->sample_count;

		if (ts->sample_count < TSC_SKIP)
			goto done;

		index = (ts->sample_count - TSC_TAIL_SKIP - 1) % TSC_SAMPLES;
		sample = &ts->samples[index];

		tsc_report(ts, sample->x, sample->y, sample->y);
		tsc_report_down(ts, (ts->sample_count == TSC_SKIP));
done:
		mod_timer(&ts->timer, jiffies + TSC_PENUP_POLL);
	}

	spin_unlock(&ts->lock);
	return IRQ_HANDLED;
}

static ssize_t tsc_cal_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct tsc_data *ts = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i;

	for (i = 0; i < 6; i++)
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", ts->cal[i]);

	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", ts->cal[6]);

	return len;
}

/*
 * The calibration data format is identical to the contents of tslib's
 * generated output
 */
static ssize_t tsc_cal_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct tsc_data *ts = dev_get_drvdata(dev);
	int index = 0;
	int cal[TSC_CAL_SIZE];
	const char *cur = buf, *end = buf + count;
	char *tmp;
	unsigned long flags;

	for (index = 0; index < TSC_CAL_SIZE; index++) {
		while (isspace(*cur) && cur < end)
			cur++;
		if (cur >= end) {
			dev_err(ts->dev, "premature end in calib data\n");
			return -EINVAL;
		}

		if (isdigit(*cur) || *cur == '-') {
			cal[index] = simple_strtol(cur, &tmp, 0);
			cur = tmp;
		}
	}

	spin_lock_irqsave(&ts->lock, flags);
	memcpy(ts->cal, cal, sizeof(cal));
	spin_unlock_irqrestore(&ts->lock, flags);

	return count;
}

static struct device_attribute tsc_attr_cal =
	__ATTR(tscal, S_IWUSR | S_IRUGO, tsc_cal_show, tsc_cal_store);

static int tsc_probe(struct platform_device *pdev)
{
	struct tnetv107x_tsc_data *pdata = pdev->dev.platform_data;
	struct tsc_data *ts;
	int ret = 0, *cal;
	u32 rev = 0, val = 0;
	struct device *dev = &pdev->dev;

	if (!pdata) {
		dev_err(dev, "could not find platform data\n");
		return -EINVAL;
	}

	ts = kzalloc(sizeof(struct tsc_data), GFP_KERNEL);
	if (!ts) {
		dev_err(dev, "cannot allocate device info\n");
		return -ENOMEM;
	}

	ts->dev = dev;
	ts->data = *pdata;
	dev_set_drvdata(dev, ts);

	cal = (tscal_set == TSC_CAL_SIZE) ? tscal : ts->data.calibration_data;
	memcpy(ts->cal, cal, TSC_CAL_SIZE * sizeof(int));

	ret = -ENOMEM;
	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		dev_err(dev, "cannot allocate input device\n");
		goto error0;
	}

	ret = -ENODEV;
	ts->tsc_irq = platform_get_irq(pdev, 0);
	if (ts->tsc_irq < 0) {
		dev_err(dev, "cannot determine device interrupt\n");
		goto error1;
	}

	ret = -ENODEV;
	ts->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!ts->res) {
		dev_err(dev, "cannot determine register area\n");
		goto error1;
	}

	ret = -EINVAL;
	if (!request_mem_region(ts->res->start, resource_size(ts->res),
				pdev->name)) {
		dev_err(dev, "cannot claim register memory\n");
		goto error1;
	}

	ret = -ENOMEM;
	ts->regs = ioremap(ts->res->start, resource_size(ts->res));
	if (!ts->regs) {
		dev_err(dev, "cannot map register memory\n");
		goto error2;
	}

	ret = -EINVAL;
	ts->clk = clk_get(dev, NULL);
	if (!ts->clk) {
		dev_err(dev, "cannot claim device clock\n");
		goto error3;
	}
	clk_enable(ts->clk);

	spin_lock_init(&ts->lock);

	init_timer(&ts->timer);
	setup_timer(&ts->timer, tsc_poll, (unsigned long)ts);

	/* Go to idle mode, before any initialization */
	while ((tsc_read(ts, tscm) & IDLE) == 0)
		barrier();

	/* Configure the TSC Control register*/
	val = (PONBG | PON | PVSTC(4) | ONE_SHOT | ZMEASURE_EN);
	tsc_write(ts, tscm, val);

	/* Bring TSC out of reset: Clear AFE reset bit */
	val &= ~(AFERST);
	tsc_write(ts, tscm, val);
	udelay(10);

	/* Configure all pins for hardware control*/
	tsc_write(ts, bwcm, 0);

	/* Finally enable the TSC */
	tsc_set_bits(ts, tscm, TSC_EN);

	ret = -EINVAL;
	if (request_irq(ts->tsc_irq, tsc_irq, 0, "tnetv107x-ts", ts)) {
		dev_err(dev, "Could not allocate ts irq\n");
		goto error4;
	}

	ret = device_create_file(ts->dev, &tsc_attr_cal);
	if (ret < 0) {
		dev_err(dev, "Could not create sysfs entry!\n");
		goto error5;
	}

	rev = tsc_read(ts, rev);
	ts->input_dev->name       = "tnetv107x-ts";
	ts->input_dev->phys       = "tnetv107x-ts/input0";
	ts->input_dev->id.bustype = BUS_HOST;
	ts->input_dev->id.vendor  = 0x0001;
	ts->input_dev->id.product = ((rev >>  8) & 0x07);
	ts->input_dev->id.version = ((rev >> 16) & 0xfff);
	ts->input_dev->dev.parent = &pdev->dev;

	/* Declare capabilities */
	set_bit(EV_KEY,       ts->input_dev->evbit);
	set_bit(BTN_TOUCH,    ts->input_dev->keybit);
	set_bit(EV_ABS,       ts->input_dev->evbit);
	set_bit(ABS_X,        ts->input_dev->absbit);
	set_bit(ABS_Y,        ts->input_dev->absbit);
	set_bit(ABS_PRESSURE, ts->input_dev->absbit);

	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->data.xres, 5, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->data.yres, 5, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 4095, 128, 0);

	ret = input_register_device(ts->input_dev);
	if (ret < 0)
		goto error6;

	return 0;

error6:
	device_remove_file(ts->dev, &tsc_attr_cal);
error5:
	free_irq(ts->tsc_irq, ts);
error4:
	clk_disable(ts->clk);
	clk_put(ts->clk);
error3:
	iounmap(ts->regs);
error2:
	release_mem_region(ts->res->start, resource_size(ts->res));
error1:
	input_free_device(ts->input_dev);
error0:
	kfree(ts);
	dev_set_drvdata(dev, NULL);
	return ret;
}

static int tsc_remove(struct platform_device *pdev)
{
	struct tsc_data *ts = dev_get_drvdata(&pdev->dev);

	if (!ts)
		return 0;

	tsc_clr_bits(ts, tscm, TSC_EN);
	del_timer_sync(&ts->timer);
	device_remove_file(ts->dev, &tsc_attr_cal);
	free_irq(ts->tsc_irq, ts);
	clk_disable(ts->clk);
	clk_put(ts->clk);
	iounmap(ts->regs);
	release_mem_region(ts->res->start, resource_size(ts->res));
	input_free_device(ts->input_dev);
	kfree(ts);
	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

static int tsc_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* Nothing yet */
	return 0;
}

static int tsc_resume(struct platform_device *pdev)
{
	/* Nothing yet */
	return 0;
}

static struct platform_driver tsc_driver = {
	.probe		= tsc_probe,
	.remove		= tsc_remove,
	.suspend	= tsc_suspend,
	.resume		= tsc_resume,
	.driver.name	= "tnetv107x-ts",
};

static int __init tsc_init(void)
{
	return platform_driver_register(&tsc_driver);
}

static void __exit tsc_exit(void)
{
	platform_driver_unregister(&tsc_driver);
}

module_init(tsc_init);
module_exit(tsc_exit);

MODULE_DESCRIPTION("TNETV107X Touchscreen Driver");
MODULE_LICENSE("GPL");
