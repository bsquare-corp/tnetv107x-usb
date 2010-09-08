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
#include <linux/gpio.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#define TPS6116X_MAX_INTENSITY		31
#define TPS6116X_DEFAULT_INTENSITY	10

/* Easyscale timing w/ margin (usecs) */
#define T_POWER_SETTLE			2000
#define T_ES_DELAY			120
#define T_ES_DETECT			280
#define T_ES_WINDOW			(1000 - T_ES_DELAY - T_ES_DETECT)
#define T_START				3
#define T_EOS				3
#define T_INACTIVE			3
#define T_ACTIVE			(3 * T_INACTIVE)

#define CMD_SET				0x72

struct tps6116x {
	struct ti_ssp_device		*handle;
	struct device			*dev;
	int				gpio;
	struct mutex			lock;
	int				intensity;
	struct backlight_properties	props;
	struct backlight_device		*bl;
	struct regulator		*regulator;
	bool				power;
	bool				suspended;
};

static int __set_power(struct tps6116x *hw, bool power)
{
	unsigned long flags;
	int error;

	if (power == hw->power)
		return 0; /* nothing to do */

	/* disabling is simple... choke power */
	if (!power) {
		error = regulator_disable(hw->regulator);
		goto done;
	}

	/* set ctrl pin init state for easyscale detection */
	gpio_set_value(hw->gpio, 0);

	error = regulator_enable(hw->regulator);
	if (error < 0)
		goto done;

	udelay(T_POWER_SETTLE);

	/*
	 * Now that the controller is powered up, we need to put it into 1-wire
	 * mode.  This is a timing sensitive operation, hence the irq disable.
	 * Ideally, this should happen rarely, and mostly at init, so disabling
	 * interrupts for the duration should not be a problem.
	 */
	local_irq_save(flags);

	gpio_set_value(hw->gpio, 1);
	udelay(T_ES_DELAY);
	gpio_set_value(hw->gpio, 0);
	udelay(T_ES_DETECT);
	gpio_set_value(hw->gpio, 1);

	local_irq_restore(flags);

done:
	if (error >= 0)
		hw->power = power;

	return error;
}

static void __write_byte(struct tps6116x *hw, u8 data)
{
	int bit;

	gpio_set_value(hw->gpio, 1);
	udelay(T_START);

	for (bit = 0; bit < 8; bit++, data <<= 1) {
		int val = data & 0x80;
		int t_lo = val ? T_INACTIVE : T_ACTIVE;
		int t_hi = val ? T_ACTIVE : T_INACTIVE;

		gpio_set_value(hw->gpio, 0);
		udelay(t_lo);
		gpio_set_value(hw->gpio, 1);
		udelay(t_hi);
	}

	gpio_set_value(hw->gpio, 0);
	udelay(T_EOS);
	gpio_set_value(hw->gpio, 1);
}

static void __set_intensity(struct tps6116x *hw, int intensity)
{
	unsigned long flags;

	intensity = clamp(intensity, 0, TPS6116X_MAX_INTENSITY);

	local_irq_save(flags);
	__write_byte(hw, CMD_SET);
	__write_byte(hw, intensity);
	local_irq_restore(flags);
}

static int set_intensity(struct tps6116x *hw, int intensity)
{
	int error = 0;

	if (intensity == hw->intensity)
		return 0;

	mutex_lock(&hw->lock);

	error = __set_power(hw, intensity ? true : false);
	if (error < 0)
		goto error;

	if (intensity > 0)
		__set_intensity(hw, intensity);

	hw->intensity = intensity;
error:
	mutex_unlock(&hw->lock);

	return error;
}

static int get_brightness(struct backlight_device *bl)
{
	struct tps6116x *hw = bl_get_data(bl);

	return hw->intensity;
}

static int update_status(struct backlight_device *bl)
{
	struct tps6116x *hw = bl_get_data(bl);
	int intensity = bl->props.brightness;

	if (hw->suspended || hw->props.state & BL_CORE_SUSPENDED)
		intensity = 0;
	if (bl->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;

	return set_intensity(hw, intensity);
}

static const struct backlight_ops tps6116x_backlight_ops = {
	.options	= BL_CORE_SUSPENDRESUME,
	.get_brightness	= get_brightness,
	.update_status	= update_status,
};

static int __devinit tps6116x_probe(struct platform_device *pdev)
{
	struct tps6116x *hw;
	struct device *dev = &pdev->dev;
	struct backlight_properties props;
	int error;

	hw = kzalloc(sizeof(struct tps6116x), GFP_KERNEL);
	if (!hw) {
		dev_err(dev, "cannot allocate driver data\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, hw);

	hw->gpio = (int)dev->platform_data;
	hw->dev = dev;

	mutex_init(&hw->lock);

	hw->regulator = regulator_get(dev, "vlcd");
	if (IS_ERR(hw->regulator)) {
		error = PTR_ERR(hw->regulator);
		dev_err(dev, "cannot claim regulator\n");
		goto error_regulator;
	}

	error = gpio_request_one(hw->gpio, GPIOF_DIR_OUT, dev_name(dev));
	if (error < 0) {
		dev_err(dev, "cannot claim gpio\n");
		goto error_gpio;
	}

	memset(&props, 0, sizeof(props));
	props.max_brightness = TPS6116X_MAX_INTENSITY;
	props.brightness     = TPS6116X_DEFAULT_INTENSITY;
	props.power          = FB_BLANK_UNBLANK;

	hw->bl = backlight_device_register("tps6116x", hw->dev, hw,
					   &tps6116x_backlight_ops, &props);
	if (IS_ERR(hw->bl)) {
		error = PTR_ERR(hw->bl);
		dev_err(dev, "backlight registration failed\n");
		goto error_register;
	}

	update_status(hw->bl);
	dev_info(dev, "registered backlight controller\n");
	return 0;

error_register:
	gpio_free(hw->gpio);
error_gpio:
	regulator_put(hw->regulator);
error_regulator:
	kfree(hw);
	platform_set_drvdata(pdev, NULL);
	return error;
}

static int __devexit tps6116x_remove(struct platform_device *pdev)
{
	struct tps6116x *hw = platform_get_drvdata(pdev);

	backlight_device_unregister(hw->bl);
	regulator_disable(hw->regulator);
	regulator_put(hw->regulator);
	gpio_free(hw->gpio);
	kfree(hw);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int tps6116x_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct tps6116x *hw = platform_get_drvdata(pdev);
	hw->suspended = true;
	update_status(hw->bl);
	return 0;
}

static int tps6116x_resume(struct platform_device *pdev)
{
	struct tps6116x *hw = platform_get_drvdata(pdev);
	hw->suspended = false;
	update_status(hw->bl);
	return 0;
}

static struct platform_driver tps6116x_driver = {
	.probe		= tps6116x_probe,
	.remove		= __devexit_p(tps6116x_remove),
	.suspend	= tps6116x_suspend,
	.resume		= tps6116x_resume,
	.driver		= {
		.name	= "tps6116x",
		.owner	= THIS_MODULE,
	},
};

static int __init tps6116x_init(void)
{
	return platform_driver_register(&tps6116x_driver);
}
module_init(tps6116x_init);

static void __exit tps6116x_exit(void)
{
	platform_driver_unregister(&tps6116x_driver);
}
module_exit(tps6116x_exit);

MODULE_DESCRIPTION("SSP TPS6116X Driver");
MODULE_AUTHOR("Cyril Chemparathy");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tps6116x");
