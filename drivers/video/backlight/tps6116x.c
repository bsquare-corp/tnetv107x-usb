/*
 * TPS6116X LCD Backlight Controller Driver
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#if 0
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/suspend.h>
#include <linux/spinlock.h>
#include <linux/miscdevice.h>
#endif

#include <mach/ti_ssp.h>

#define TPS6116X_MAX_INTENSITY		31
#define TPS6116X_DEFAULT_INTENSITY	10

struct tps6116x {
	struct ti_ssp_device		*handle;
	struct device			*dev;
	struct mutex			lock;
	int				intensity;
	struct backlight_properties	props;
	struct backlight_device		*bl;
	int				suspended:1;
	struct regulator		*regulator;
};

static u32 tps6116x_seqmap[] = {
#define TPS6116X_SHUTDOWN_OFFSET	0
	SSP_OPCODE_STOP | SSP_OUT_MODE | SSP_DATA_LOW,

#define TPS6116X_STARTUP_OFFSET		1
	SSP_COUNT(60)   | SSP_OUT_MODE | SSP_DATA_HIGH,
	SSP_COUNT(40)   | SSP_OUT_MODE | SSP_DATA_HIGH,
	SSP_COUNT(60)   | SSP_OUT_MODE | SSP_DATA_LOW,
	SSP_COUNT(60)   | SSP_OUT_MODE | SSP_DATA_LOW,
	SSP_COUNT(60)   | SSP_OUT_MODE | SSP_DATA_LOW,
	SSP_COUNT(60)   | SSP_OUT_MODE | SSP_DATA_LOW,
	SSP_COUNT(10)   | SSP_OUT_MODE | SSP_DATA_HIGH,
	SSP_OPCODE_STOP | SSP_OUT_MODE | SSP_DATA_HIGH,

#define TPS6116X_SEND_OFFSET		9
	SSP_COUNT(10)   | SSP_OUT_MODE | SSP_DATA_HIGH,
	SSP_COUNT(31)   | SSP_OUT_MODE | SSP_OPCODE_SHIFT | SSP_ADDR_REG,
	SSP_COUNT(31)   | SSP_OUT_MODE | SSP_OPCODE_SHIFT | SSP_DATA_REG,
	SSP_COUNT(2)    | SSP_OUT_MODE | SSP_DATA_LOW,
	SSP_OPCODE_STOP | SSP_OUT_MODE | SSP_DATA_HIGH,
};

/*
 * Convert a 8-bit data value into an encoded 32-bit output.
 * Encoded as follows:
 *	Logic Low  - L-L-L-H	(0x1)
 *	Logic High - L-H-H-H	(0x7)
 */
static inline u32 __tps6116x_encode(u8 data)
{
	u32 ret = 0, i;

	for (i = 0; i < 8; i++, data <<= 1)
		ret = (ret << 4) | ((data & 0x80) ? 0x7 : 0x1);
	return ret;
}

static int __tps6116x_write(struct tps6116x *hw, u8 val)
{
	int ret;
	u32 data;

	if (!hw)
		return -EINVAL;

	data = __tps6116x_encode(0x72);
	ret = ti_ssp_run(hw->handle, TPS6116X_SEND_OFFSET,
			data, &data);
	if (ret < 0)
		return ret;

	data = __tps6116x_encode(val);
	ret = ti_ssp_run(hw->handle, TPS6116X_SEND_OFFSET,
			data, &data);
	if (ret < 0)
		return ret;

	return 0;
}

static int __tps6116x_set_intensity_direct(struct tps6116x *hw, int intensity)
{
	int ret;
	u32 data = 0;

	if (!intensity) {
		ret = ti_ssp_run(hw->handle, TPS6116X_SHUTDOWN_OFFSET,
				data, &data);
		if (ret < 0) {
			dev_err(hw->dev, "failed to run shutdown seq\n");
			goto fail1;
		}
		mdelay(3);
	} else {
		if (!hw->intensity) {
			ret = ti_ssp_run(hw->handle,
					TPS6116X_STARTUP_OFFSET, data, &data);
			if (ret < 0) {
				dev_err(hw->dev, "startup failed\n");
				goto fail1;
			}
		}

		ret = __tps6116x_write(hw, intensity);
	}

	hw->intensity = intensity;
fail1:
	return ret;
}

static int __tps6116x_set_intensity(struct tps6116x *hw, int intensity)
{
	int ret;

	if (intensity < 0 || intensity > TPS6116X_MAX_INTENSITY)
		return -EINVAL;

	ret = -ENODEV;
	hw->handle = ti_ssp_open(dev_name(hw->dev));
	if (IS_ERR(hw->handle)) {
		dev_err(hw->dev, "failed to open SSP device\n");
		goto fail0;
	}

	ret = ti_ssp_load(hw->handle, 0, tps6116x_seqmap,
			  ARRAY_SIZE(tps6116x_seqmap));
	if (ret) {
		dev_err(hw->dev, "failed to load SSP seqram\n");
		goto fail1;
	}

	ret = __tps6116x_set_intensity_direct(hw, intensity);

fail1:
	ti_ssp_close(hw->handle);
fail0:
	return ret;
}

static int tps6116x_set_intensity(struct tps6116x *hw, int intensity)
{
	int ret;

	mutex_lock(&hw->lock);

	if (hw->intensity < 0)
		dev_warn(hw->dev, "setting intensity without reset\n");

	ret = __tps6116x_set_intensity(hw, intensity);

	mutex_unlock(&hw->lock);

	return ret;
}

static int tps6116x_reset(struct tps6116x *hw)
{
	u32 data = 0;
	int ret;

	mutex_lock(&hw->lock);

	ret = -ENODEV;
	hw->handle = ti_ssp_open("tps6116x");
	if (IS_ERR(hw->handle)) {
		dev_err(hw->dev, "failed to open SSP device\n");
		goto fail0;
	}

	ret = ti_ssp_load(hw->handle, 0, tps6116x_seqmap,
			ARRAY_SIZE(tps6116x_seqmap));
	if (ret) {
		dev_err(hw->dev, "failed to load SSP seqram\n");
		goto fail1;
	}

	ret = ti_ssp_run(hw->handle, TPS6116X_SHUTDOWN_OFFSET,
			data, &data);
	if (ret < 0) {
		dev_err(hw->dev, "failed to run shutdown seq\n");
		goto fail1;
	}
	mdelay(3);

	hw->intensity = 0;

	ret = __tps6116x_set_intensity_direct(hw, TPS6116X_DEFAULT_INTENSITY);
	if (ret < 0) {
		dev_err(hw->dev, "failed to set default intensity\n");
		goto fail1;
	}

fail1:
	ti_ssp_close(hw->handle);
fail0:
	mutex_unlock(&hw->lock);
	return ret;
}

static ssize_t
tps6116x_intensity_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct tps6116x *hw = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "%d\n", hw->intensity);

	return len;
}

static ssize_t
tps6116x_intensity_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct tps6116x *hw = dev_get_drvdata(dev);
	unsigned long intensity;
	int ret;

	ret = strict_strtoul(buf, 10, &intensity);
	if (ret < 0)
		return ret;

	if (intensity > TPS6116X_MAX_INTENSITY)
		intensity = TPS6116X_MAX_INTENSITY;

	if (intensity < 0)
		intensity = 0;

	tps6116x_set_intensity(hw, intensity);

	return count;
}

static struct device_attribute tps6116x_attr_intensity =
	__ATTR(intensity, S_IWUSR | S_IRUGO,
		tps6116x_intensity_show,
		tps6116x_intensity_store);

static int tps6116x_get_brightness(struct backlight_device *bl)
{
	struct tps6116x *hw = bl_get_data(bl);

	return hw->intensity;
}

static int tps6116x_update_status(struct backlight_device *bl)
{
	struct tps6116x *hw = bl_get_data(bl);
	int intensity = bl->props.brightness;
	int ret;

	mutex_lock(&hw->lock);

	if (hw->suspended)
		intensity = 0;
	if (bl->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;

	ret = __tps6116x_set_intensity(hw, intensity);

	mutex_unlock(&hw->lock);

	return ret;
}

static const struct backlight_ops tps6116x_backlight_ops = {
	.get_brightness	= tps6116x_get_brightness,
	.update_status	= tps6116x_update_status,
};

static int tps6116x_probe(struct platform_device *pdev)
{
	struct tps6116x *hw;
	struct device *dev = &pdev->dev;
	struct backlight_properties props;
	int ret;

	ret = -ENOMEM;
	hw = kzalloc(sizeof(struct tps6116x), GFP_KERNEL);
	if (!hw) {
		dev_err(dev, "cannot allocate driver data\n");
		goto fail0;
	}
	platform_set_drvdata(pdev, hw);

	memset(hw, 0, sizeof(struct tps6116x));
	hw->dev = dev;

	mutex_init(&hw->lock);

	ret = device_create_file(dev, &tps6116x_attr_intensity);
	if (ret < 0) {
		dev_err(dev, "cannot create device attributes\n");
		goto fail1;
	}

	hw->regulator = regulator_get(dev, "vlcd");
	if (IS_ERR(hw->regulator)) {
		dev_err(dev, "cannot claim regulator\n");
		ret = PTR_ERR(hw->regulator);
		goto fail2;
	}

	ret = regulator_enable(hw->regulator);
	if (ret < 0) {
		dev_err(dev, "cannot enable regulator\n");
		goto fail3;
	}

	ret = tps6116x_reset(hw);
	if (ret < 0) {
		dev_err(dev, "device reset failed\n");
		goto fail4;
	}

	memset(&props, 0, sizeof(props));
	props.max_brightness = TPS6116X_MAX_INTENSITY;
	props.brightness     = TPS6116X_DEFAULT_INTENSITY;

	hw->bl = backlight_device_register("tps6116x", hw->dev, hw,
					   &tps6116x_backlight_ops, &props);
	if (IS_ERR(hw->bl)) {
		dev_err(dev, "backlight registration failed\n");
		ret = PTR_ERR(hw->bl);
		goto fail4;
	}

	dev_info(hw->dev, "initialized tps6116x backlight driver\n");
	return 0;

fail4:
	regulator_disable(hw->regulator);
fail3:
	regulator_put(hw->regulator);
fail2:
	device_remove_file(dev, &tps6116x_attr_intensity);
fail1:
	kfree(hw);
	platform_set_drvdata(pdev, NULL);
fail0:
	return ret;
}

static int tps6116x_remove(struct platform_device *pdev)
{
	struct tps6116x *hw = platform_get_drvdata(pdev);

	backlight_device_unregister(hw->bl);
	regulator_disable(hw->regulator);
	regulator_put(hw->regulator);
	device_remove_file(hw->dev, &tps6116x_attr_intensity);
	kfree(hw);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int tps6116x_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tps6116x *hw = platform_get_drvdata(pdev);
	hw->suspended = 1;
	tps6116x_update_status(hw->bl);
	return 0;
}

static int tps6116x_resume(struct platform_device *pdev)
{
	struct tps6116x *hw = platform_get_drvdata(pdev);
	hw->suspended = 0;
	tps6116x_update_status(hw->bl);
	return 0;
}

static struct platform_driver tps6116x_driver = {
	.probe		= tps6116x_probe,
	.remove		= tps6116x_remove,
	.suspend	= tps6116x_suspend,
	.resume		= tps6116x_resume,
	.driver = {
		.name	= "tps6116x",
		.owner	= THIS_MODULE,
	},
};

static int __init tps6116x_init(void)
{
	return platform_driver_register(&tps6116x_driver);
}

static void __exit tps6116x_exit(void)
{
	platform_driver_unregister(&tps6116x_driver);
}

module_init(tps6116x_init);
module_exit(tps6116x_exit);

MODULE_DESCRIPTION("SSP TPS6116X Driver");
MODULE_LICENSE("GPL");
