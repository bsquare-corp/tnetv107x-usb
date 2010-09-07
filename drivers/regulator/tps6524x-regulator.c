/*
 * Regulator driver for TPS6524x PMIC
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
#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include <mach/ti_ssp.h>

#define TPS6524X_REG_LDO_SET		0x0
#define TPS6524X_LDO_ILIM_MASK		1	/* 0 = 400-800, 1 = 900-1500 */
#define TPS6524X_LDO_VSEL_MASK		0x0f
#define TPS6524X_LDO2_ILIM_SHIFT	12
#define TPS6524X_LDO2_VSEL_SHIFT	4
#define TPS6524X_LDO1_ILIM_SHIFT	8
#define TPS6524X_LDO1_VSEL_SHIFT	0

#define TPS6524X_REG_BLOCK_EN		0x1
#define TPS6524X_BLOCK_MASK		1
#define TPS6524X_BLOCK_LDO1_SHIFT	0
#define TPS6524X_BLOCK_LDO2_SHIFT	1
#define TPS6524X_BLOCK_LCD_SHIFT	2
#define TPS6524X_BLOCK_USB_SHIFT	3

#define TPS6524X_REG_DCDC_SET		0x2
#define TPS6524X_DCDC_VDCDC_MASK	0x1f
#define TPS6524X_DCDC_VDCDC1_SHIFT	0
#define TPS6524X_DCDC_VDCDC2_SHIFT	5
#define TPS6524X_DCDC_VDCDC3_SHIFT	10

#define TPS6524X_REG_DCDC_EN		0x3
#define TPS6524X_DCDCDCDC_EN_MASK	0x1
#define TPS6524X_DCDCDCDC1_EN_SHIFT	0
#define TPS6524X_DCDCDCDC1_PG_MSK	BIT(1)
#define TPS6524X_DCDCDCDC2_EN_SHIFT	2
#define TPS6524X_DCDCDCDC2_PG_MSK	BIT(3)
#define TPS6524X_DCDCDCDC3_EN_SHIFT	4
#define TPS6524X_DCDCDCDC3_PG_MSK	BIT(5)

#define TPS6524X_REG_USB		0x4
#define TPS6524X_USB_ILIM_SHIFT		0
#define TPS6524X_USB_ILIM_MASK		0x3
#define TPS6524X_USB_TSD_SHIFT		2
#define TPS6524X_USB_TSD_MASK		0x3
#define TPS6524X_USB_TWARN_SHIFT	4
#define TPS6524X_USB_TWARN_MASK		0x3
#define TPS6524X_USB_IWARN_SD		BIT(6)
#define TPS6524X_USB_FAST_LOOP		BIT(7)

#define TPS6524X_REG_ALARM		0x5
#define TPS6524X_ALARM_LDO1		BIT(0)
#define TPS6524X_ALARM_DCDC1		BIT(1)
#define TPS6524X_ALARM_DCDC2		BIT(2)
#define TPS6524X_ALARM_DCDC3		BIT(3)
#define TPS6524X_ALARM_LDO2		BIT(4)
#define TPS6524X_ALARM_USB_WARN		BIT(5)
#define TPS6524X_ALARM_USB_ALARM	BIT(6)
#define TPS6524X_ALARM_LCD		BIT(9)
#define TPS6524X_ALARM_TEMP_WARM	BIT(10)
#define TPS6524X_ALARM_TEMP_HOT		BIT(11)
#define TPS6524X_ALARM_NRST		BIT(14)
#define TPS6524X_ALARM_POWERUP		BIT(15)

#define TPS6524X_REG_INT_ENABLE		0x6
#define TPS6524X_INT_LDO1		BIT(0)
#define TPS6524X_INT_DCDC1		BIT(1)
#define TPS6524X_INT_DCDC2		BIT(2)
#define TPS6524X_INT_DCDC3		BIT(3)
#define TPS6524X_INT_LDO2		BIT(4)
#define TPS6524X_INT_USB_WARN		BIT(5)
#define TPS6524X_INT_USB_ALARM		BIT(6)
#define TPS6524X_INT_LCD		BIT(9)
#define TPS6524X_INT_TEMP_WARM		BIT(10)
#define TPS6524X_INT_TEMP_HOT		BIT(11)
#define TPS6524X_INT_GLOBAL_EN		BIT(15)

#define TPS6524X_REG_INT_STATUS		0x7
#define TPS6524X_STATUS_LDO1		BIT(0)
#define TPS6524X_STATUS_DCDC1		BIT(1)
#define TPS6524X_STATUS_DCDC2		BIT(2)
#define TPS6524X_STATUS_DCDC3		BIT(3)
#define TPS6524X_STATUS_LDO2		BIT(4)
#define TPS6524X_STATUS_USB_WARN	BIT(5)
#define TPS6524X_STATUS_USB_ALARM	BIT(6)
#define TPS6524X_STATUS_LCD		BIT(9)
#define TPS6524X_STATUS_TEMP_WARM	BIT(10)
#define TPS6524X_STATUS_TEMP_HOT	BIT(11)

#define TPS6524X_REG_SOFTWARE_RESET	0xb
#define TPS6524X_REG_WRITE_ENABLE	0xd
#define TPS6524X_REG_REV_ID		0xf

#define TPS6524X_ADDR_MASK		0x03f
#define TPS6524X_READ			0x000
#define TPS6524X_WRITE			0x200
#define TPS6524X_ADDR(a, w)		(((a)<<10) | w)

#define TPS6524X_N_DCDC			3
#define TPS6524X_N_LDO			2
#define TPS6524X_N_SWITCH		2
#define TPS6524X_N_REGULATORS		(TPS6524X_N_DCDC + TPS6524X_N_LDO + \
					 TPS6524X_N_SWITCH)

#define TPS6524X_FIXED_ILIMSEL		0x1
#define TPS6524X_FIXED_VOLTAGE		0x2

struct tps6524x_field {
	int		reg;
	int		shift;
	int		mask;
};

struct tps6524x_info {
	const char		*name;
	int			n_voltages;
	int			*voltages;
	int			fixed_voltage;
	int			n_ilimsels;
	int			*ilimsels;
	int			fixed_ilimsel;
	int			flags;
	struct tps6524x_field	enable;
	struct tps6524x_field	voltage;
	struct tps6524x_field	ilimsel;
};

struct tps6524x {
	struct ti_ssp_device	*handle;
	struct device		*dev;
	struct mutex		lock;
	struct regulator_desc	desc[TPS6524X_N_REGULATORS];
	struct regulator_dev	*rdev[TPS6524X_N_REGULATORS];
	struct tps6524x_info	*info;
};

static u32 tps6524x_seqmap[] = {
#define TPS6524X_WRITE_OFFSET		0
	/* Write */
	SSP_OPCODE_SHIFT | SSP_OUT_MODE | SSP_ADDR_REG | SSP_COUNT(23),
	SSP_OPCODE_SHIFT | SSP_OUT_MODE | SSP_DATA_REG | SSP_COUNT(31),
	SSP_OPCODE_SHIFT | SSP_IN_MODE  | SSP_ADDR_REG | SSP_COUNT(7),
	SSP_OPCODE_STOP  | SSP_OUT_MODE | SSP_CS_HIGH,
#define TPS6524X_WRITE_STOP		3

#define TPS6524X_READ_OFFSET		4
	/* Read */
	SSP_OPCODE_SHIFT | SSP_OUT_MODE | SSP_ADDR_REG | SSP_COUNT(23),
	SSP_OPCODE_SHIFT | SSP_IN_MODE  | SSP_DATA_REG | SSP_COUNT(31),
	SSP_OPCODE_SHIFT | SSP_IN_MODE  | SSP_ADDR_REG | SSP_COUNT(7),
	SSP_OPCODE_STOP  | SSP_OUT_MODE | SSP_CS_HIGH,
#define TPS6524X_READ_STOP		7
};

static int tps6524x_dcdc1_voltages[] = {
	 800000,  825000,  850000,  875000,
	 900000,  925000,  950000,  975000,
	1000000, 1025000, 1050000, 1075000,
	1100000, 1125000, 1150000, 1175000,
	1200000, 1225000, 1250000, 1275000,
	1300000, 1325000, 1350000, 1375000,
	1400000, 1425000, 1450000, 1475000,
	1500000, 1525000, 1550000, 1575000,
};

static int tps6524x_dcdc2_voltages[] = {
	1400000, 1450000, 1500000, 1550000,
	1600000, 1650000, 1700000, 1750000,
	1800000, 1850000, 1900000, 1950000,
	2000000, 2050000, 2100000, 2150000,
	2200000, 2250000, 2300000, 2350000,
	2400000, 2450000, 2500000, 2550000,
	2600000, 2650000, 2700000, 2750000,
	2800000, 2850000, 2900000, 2950000,
};

static int tps6524x_dcdc3_voltages[] = {
	2400000, 2450000, 2500000, 2550000, 2600000,
	2650000, 2700000, 2750000, 2800000, 2850000,
	2900000, 2950000, 3000000, 3050000, 3100000,
	3150000, 3200000, 3250000, 3300000, 3350000,
	3400000, 3450000, 3500000, 3550000, 3600000,
};

static int tps6524x_ldo1_voltages[] = {
	4300000, 4350000, 4400000, 4450000,
	4500000, 4550000, 4600000, 4650000,
	4700000, 4750000, 4800000, 4850000,
	4900000, 4950000, 5000000, 5050000,
};

static int tps6524x_ldo2_voltages[] = {
	1100000, 1150000, 1200000, 1250000,
	1300000, 1700000, 1750000, 1800000,
	1850000, 1900000, 3150000, 3200000,
	3250000, 3300000, 3350000, 3400000,
};

static int tps6524x_ldo_ilimsel[] = {
	400000, 1500000
};

static int tps6524x_usb_ilimsel[] = {
	200000, 400000, 800000, 1000000
};

static struct tps6524x_info tps6524x_info[] = {
	{
		.name		= "DCDC1",
		.flags		= TPS6524X_FIXED_ILIMSEL,
		.n_voltages	= ARRAY_SIZE(tps6524x_dcdc1_voltages),
		.voltages	= tps6524x_dcdc1_voltages,
		.fixed_ilimsel	= 2400000,
		.enable		= {
			.reg	= TPS6524X_REG_DCDC_EN,
			.mask	= TPS6524X_DCDCDCDC_EN_MASK,
			.shift	= TPS6524X_DCDCDCDC1_EN_SHIFT,
		},
		.voltage	= {
			.reg	= TPS6524X_REG_DCDC_SET,
			.mask	= TPS6524X_DCDC_VDCDC_MASK,
			.shift	= TPS6524X_DCDC_VDCDC1_SHIFT,
		},
	},
	{
		.name		= "DCDC2",
		.flags		= TPS6524X_FIXED_ILIMSEL,
		.n_voltages	= ARRAY_SIZE(tps6524x_dcdc2_voltages),
		.voltages	= tps6524x_dcdc2_voltages,
		.fixed_ilimsel	= 1200000,
		.enable		= {
			.reg	= TPS6524X_REG_DCDC_EN,
			.mask	= TPS6524X_DCDCDCDC_EN_MASK,
			.shift	= TPS6524X_DCDCDCDC2_EN_SHIFT,
		},
		.voltage	= {
			.reg	= TPS6524X_REG_DCDC_SET,
			.mask	= TPS6524X_DCDC_VDCDC_MASK,
			.shift	= TPS6524X_DCDC_VDCDC2_SHIFT,
		},
	},
	{
		.name		= "DCDC3",
		.flags		= TPS6524X_FIXED_ILIMSEL,
		.n_voltages	= ARRAY_SIZE(tps6524x_dcdc3_voltages),
		.voltages	= tps6524x_dcdc3_voltages,
		.fixed_ilimsel	= 1200000,
		.enable		= {
			.reg	= TPS6524X_REG_DCDC_EN,
			.mask	= TPS6524X_DCDCDCDC_EN_MASK,
			.shift	= TPS6524X_DCDCDCDC3_EN_SHIFT,
		},
		.voltage	= {
			.reg	= TPS6524X_REG_DCDC_SET,
			.mask	= TPS6524X_DCDC_VDCDC_MASK,
			.shift	= TPS6524X_DCDC_VDCDC3_SHIFT,
		},
	},
	{
		.name		= "LDO1",
		.n_voltages	= ARRAY_SIZE(tps6524x_ldo1_voltages),
		.voltages	= tps6524x_ldo1_voltages,
		.n_ilimsels	= ARRAY_SIZE(tps6524x_ldo_ilimsel),
		.ilimsels	= tps6524x_ldo_ilimsel,
		.enable		= {
			.reg	= TPS6524X_REG_BLOCK_EN,
			.mask	= TPS6524X_BLOCK_MASK,
			.shift	= TPS6524X_BLOCK_LDO1_SHIFT,
		},
		.voltage	= {
			.reg	= TPS6524X_REG_LDO_SET,
			.mask	= TPS6524X_LDO_VSEL_MASK,
			.shift	= TPS6524X_LDO1_VSEL_SHIFT,
		},
		.ilimsel	= {
			.reg	= TPS6524X_REG_LDO_SET,
			.mask	= TPS6524X_LDO_ILIM_MASK,
			.shift	= TPS6524X_LDO1_ILIM_SHIFT,
		},
	},
	{
		.name		= "LDO2",
		.n_voltages	= ARRAY_SIZE(tps6524x_ldo2_voltages),
		.voltages	= tps6524x_ldo2_voltages,
		.n_ilimsels	= ARRAY_SIZE(tps6524x_ldo_ilimsel),
		.ilimsels	= tps6524x_ldo_ilimsel,
		.enable		= {
			.reg	= TPS6524X_REG_BLOCK_EN,
			.mask	= TPS6524X_BLOCK_MASK,
			.shift	= TPS6524X_BLOCK_LDO2_SHIFT,
		},
		.voltage	= {
			.reg	= TPS6524X_REG_LDO_SET,
			.mask	= TPS6524X_LDO_VSEL_MASK,
			.shift	= TPS6524X_LDO2_VSEL_SHIFT,
		},
		.ilimsel	= {
			.reg	= TPS6524X_REG_LDO_SET,
			.mask	= TPS6524X_LDO_ILIM_MASK,
			.shift	= TPS6524X_LDO2_ILIM_SHIFT,
		},
	},
	{
		.name		= "USB",
		.flags		= TPS6524X_FIXED_VOLTAGE,
		.fixed_voltage	= 5000000,
		.n_ilimsels	= ARRAY_SIZE(tps6524x_usb_ilimsel),
		.ilimsels	= tps6524x_usb_ilimsel,
		.enable		= {
			.reg	= TPS6524X_REG_BLOCK_EN,
			.mask	= TPS6524X_BLOCK_MASK,
			.shift	= TPS6524X_BLOCK_USB_SHIFT,
		},
		.ilimsel	= {
			.reg	= TPS6524X_REG_USB,
			.mask	= TPS6524X_USB_ILIM_MASK,
			.shift	= TPS6524X_USB_ILIM_SHIFT,
		},
	},
	{
		.name		= "LCD",
		.flags		= TPS6524X_FIXED_VOLTAGE |
				  TPS6524X_FIXED_ILIMSEL,
		.fixed_voltage	= 5000000,
		.fixed_ilimsel	=  400000,
		.enable		= {
			.reg	= TPS6524X_REG_BLOCK_EN,
			.mask	= TPS6524X_BLOCK_MASK,
			.shift	= TPS6524X_BLOCK_LCD_SHIFT,
		},
	},
};


static int __tps6524x_read(struct tps6524x *hw, int reg)
{
	int ret;
	u32 data;

	if (!hw)
		return -EINVAL;
	data = TPS6524X_ADDR(reg, TPS6524X_READ) << 16;

	ret = ti_ssp_run(hw->handle, TPS6524X_READ_OFFSET,
			data, &data);

	if (ret < 0)
		return ret;

	return data & 0xffff;
}

static int __tps6524x_write(struct tps6524x *hw, int reg, int val)
{
	int ret;
	u32 data;

	if (!hw)
		return -EINVAL;
	data  = TPS6524X_ADDR(reg, TPS6524X_WRITE) << 16;
	data |= (val & 0xffff);

	ret = ti_ssp_run(hw->handle, TPS6524X_WRITE_OFFSET,
			data, &data);

	if (ret < 0)
		return ret;

	return 0;
}

static int __tps6524x_rmw(struct tps6524x *hw, int reg, int mask, int val)
{
	int ret;

	ret = __tps6524x_read(hw, reg);
	if (ret < 0)
		return ret;

	ret &= ~mask;
	ret |= val;

	ret = __tps6524x_write(hw, reg, ret);

	return (ret < 0) ? ret : 0;
}

static int tps6524x_read(struct tps6524x *hw, int reg)
{
	int ret;

	mutex_lock(&hw->lock);

	ret = -ENODEV;
	hw->handle = ti_ssp_open(dev_name(hw->dev));
	if (IS_ERR(hw->handle)) {
		dev_err(hw->dev, "failed to open SSP device\n");
		goto fail0;
	}

	ret = ti_ssp_load(hw->handle, 0, tps6524x_seqmap,
			ARRAY_SIZE(tps6524x_seqmap));
	if (ret) {
		dev_err(hw->dev, "failed to load SSP seqram\n");
		goto fail1;
	}

	ret = __tps6524x_read(hw, reg);
fail1:
	ti_ssp_close(hw->handle);
fail0:
	mutex_unlock(&hw->lock);

	return ret;
}

static int tps6524x_write(struct tps6524x *hw, int reg, int val)
{
	int ret;

	mutex_lock(&hw->lock);

	ret = -ENODEV;
	hw->handle = ti_ssp_open("tps6524x");
	if (IS_ERR(hw->handle)) {
		dev_err(hw->dev, "failed to open SSP device\n");
		goto fail0;
	}

	ret = ti_ssp_load(hw->handle, 0, tps6524x_seqmap,
			ARRAY_SIZE(tps6524x_seqmap));
	if (ret) {
		dev_err(hw->dev, "failed to load SSP seqram\n");
		goto fail1;
	}

	ret = __tps6524x_write(hw, reg, val);
fail1:
	ti_ssp_close(hw->handle);
fail0:
	mutex_unlock(&hw->lock);

	return ret;
}

static int tps6524x_rmw(struct tps6524x *hw, int reg, int mask, int val)
{
	int ret;

	mutex_lock(&hw->lock);

	ret = -ENODEV;
	hw->handle = ti_ssp_open("tps6524x");
	if (IS_ERR(hw->handle)) {
		dev_err(hw->dev, "failed to open SSP device\n");
		goto fail0;
	}

	ret = ti_ssp_load(hw->handle, 0, tps6524x_seqmap,
			ARRAY_SIZE(tps6524x_seqmap));
	if (ret) {
		dev_err(hw->dev, "failed to load SSP seqram\n");
		goto fail1;
	}

	ret = __tps6524x_write(hw, TPS6524X_REG_WRITE_ENABLE, 1);
	if (ret) {
		dev_err(hw->dev, "failed to set write enable\n");
		goto fail1;
	}

	ret = __tps6524x_rmw(hw, reg, mask, val);
	if (ret) {
		dev_err(hw->dev, "failed to rmw register %d\n", reg);
		goto fail1;
	}

	ret = __tps6524x_write(hw, TPS6524X_REG_WRITE_ENABLE, 0);
	if (ret) {
		dev_err(hw->dev, "failed to clear write enable\n");
		goto fail1;
	}

fail1:
	ti_ssp_close(hw->handle);
fail0:
	mutex_unlock(&hw->lock);

	return ret;
}

static int tps6524x_read_field(struct tps6524x *hw,
			       struct tps6524x_field *field)
{
	int tmp;

	tmp = tps6524x_read(hw, field->reg);
	if (tmp < 0)
		return tmp;

	return (tmp >> field->shift) & field->mask;
}

static int tps6524x_write_field(struct tps6524x *hw,
				struct tps6524x_field *field, int val)
{
	if (val & ~field->mask)
		return -EOVERFLOW;

	return tps6524x_rmw(hw, field->reg, field->mask << field->shift,
			    val << field->shift);
}

static int tps6524x_list_voltage(struct regulator_dev *rdev,
		unsigned selector)
{
	struct tps6524x		*hw;
	struct tps6524x_info	*info;

	hw	= rdev_get_drvdata(rdev);
	info	= &hw->info[rdev_get_id(rdev)];

	if (info->flags & TPS6524X_FIXED_VOLTAGE)
		return selector ? -EINVAL : info->fixed_voltage;
	else
		return (selector < 0 || selector >= info->n_voltages) ?
			-EINVAL : info->voltages[selector];
}

static int tps6524x_set_voltage(struct regulator_dev *rdev,
				int min_uV, int max_uV)
{
	struct tps6524x		*hw;
	struct tps6524x_info	*info;
	int i;

	hw	= rdev_get_drvdata(rdev);
	info	= &hw->info[rdev_get_id(rdev)];

	if (info->flags & TPS6524X_FIXED_VOLTAGE)
		return -EINVAL;

	for (i = 0; i < info->n_voltages; i++)
		if (min_uV <= info->voltages[i] &&
		    max_uV >= info->voltages[i])
			break;

	if (i >= info->n_voltages)
		return -EINVAL;

	return tps6524x_write_field(hw, &info->voltage, i);
}

static int tps6524x_get_voltage(struct regulator_dev *rdev)
{
	struct tps6524x		*hw;
	struct tps6524x_info	*info;
	int ret;

	hw	= rdev_get_drvdata(rdev);
	info	= &hw->info[rdev_get_id(rdev)];

	if (info->flags & TPS6524X_FIXED_VOLTAGE)
		return info->fixed_voltage;

	ret = tps6524x_read_field(hw, &info->voltage);
	if (ret < 0)
		return ret;
	if (ret >= info->n_voltages)
		return -EINVAL;
	return info->voltages[ret];
}

static int tps6524x_set_current_limit(struct regulator_dev *rdev,
				      int min_uA, int max_uA)
{
	struct tps6524x		*hw;
	struct tps6524x_info	*info;
	int i;

	hw	= rdev_get_drvdata(rdev);
	info	= &hw->info[rdev_get_id(rdev)];

	if (info->flags & TPS6524X_FIXED_ILIMSEL)
		return -EINVAL;

	for (i = 0; i < info->n_ilimsels; i++)
		if (min_uA <= info->ilimsels[i] &&
		    max_uA >= info->ilimsels[i])
			break;

	if (i >= info->n_ilimsels)
		return -EINVAL;

	return tps6524x_write_field(hw, &info->ilimsel, i);
}

static int tps6524x_get_current_limit(struct regulator_dev *rdev)
{
	struct tps6524x		*hw;
	struct tps6524x_info	*info;
	int ret;

	hw	= rdev_get_drvdata(rdev);
	info	= &hw->info[rdev_get_id(rdev)];

	if (info->flags & TPS6524X_FIXED_ILIMSEL)
		return info->fixed_ilimsel;

	ret = tps6524x_read_field(hw, &info->ilimsel);
	if (ret < 0)
		return ret;
	if (ret >= info->n_ilimsels)
		return -EINVAL;
	return info->ilimsels[ret];
}

static int tps6524x_enable(struct regulator_dev *rdev)
{
	struct tps6524x		*hw;
	struct tps6524x_info	*info;

	hw	= rdev_get_drvdata(rdev);
	info	= &hw->info[rdev_get_id(rdev)];

	return tps6524x_write_field(hw, &info->enable, 1);
}

static int tps6524x_disable(struct regulator_dev *rdev)
{
	struct tps6524x		*hw;
	struct tps6524x_info	*info;

	hw	= rdev_get_drvdata(rdev);
	info	= &hw->info[rdev_get_id(rdev)];

	return tps6524x_write_field(hw, &info->enable, 0);
}

static int tps6524x_is_enabled(struct regulator_dev *rdev)
{
	struct tps6524x		*hw;
	struct tps6524x_info	*info;

	hw	= rdev_get_drvdata(rdev);
	info	= &hw->info[rdev_get_id(rdev)];

	return tps6524x_read_field(hw, &info->enable);
}

static struct regulator_ops tps6524x_regulator_ops = {
	.is_enabled		= tps6524x_is_enabled,
	.enable			= tps6524x_enable,
	.disable		= tps6524x_disable,
	.get_voltage		= tps6524x_get_voltage,
	.set_voltage		= tps6524x_set_voltage,
	.list_voltage		= tps6524x_list_voltage,
	.set_current_limit	= tps6524x_set_current_limit,
	.get_current_limit	= tps6524x_get_current_limit,
};

struct tps6524x_reg_map {
	struct device_attribute *attr;
	int reg;
};

static ssize_t tps6524x_show_reg(struct device *dev,
		struct device_attribute *attr,
		char *buf);

static ssize_t tps6524x_store_reg(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size);

#define __TPS6524X_ATTR(x)	\
static DEVICE_ATTR(x, 0664, tps6524x_show_reg, tps6524x_store_reg)

__TPS6524X_ATTR(ldo_set);
__TPS6524X_ATTR(block_en);
__TPS6524X_ATTR(dcdc_set);
__TPS6524X_ATTR(dcdc_en);
__TPS6524X_ATTR(usb);
__TPS6524X_ATTR(alarm);
__TPS6524X_ATTR(int_enable);
__TPS6524X_ATTR(int_status);
__TPS6524X_ATTR(rev_id);
__TPS6524X_ATTR(write_enable);
__TPS6524X_ATTR(software_reset);

static struct attribute *tps6524x_regs[] = {
	&dev_attr_ldo_set.attr,
	&dev_attr_block_en.attr,
	&dev_attr_dcdc_set.attr,
	&dev_attr_dcdc_en.attr,
	&dev_attr_usb.attr,
	&dev_attr_alarm.attr,
	&dev_attr_int_enable.attr,
	&dev_attr_int_status.attr,
	&dev_attr_software_reset.attr,
	&dev_attr_write_enable.attr,
	&dev_attr_rev_id.attr,
	NULL,
};

static struct tps6524x_reg_map tps6524x_reg_map[] = {
	{&dev_attr_ldo_set,		TPS6524X_REG_LDO_SET		},
	{&dev_attr_block_en,		TPS6524X_REG_BLOCK_EN		},
	{&dev_attr_dcdc_set,		TPS6524X_REG_DCDC_SET		},
	{&dev_attr_dcdc_en,		TPS6524X_REG_DCDC_EN		},
	{&dev_attr_usb,			TPS6524X_REG_USB		},
	{&dev_attr_alarm,		TPS6524X_REG_ALARM		},
	{&dev_attr_int_enable,		TPS6524X_REG_INT_ENABLE		},
	{&dev_attr_int_status,		TPS6524X_REG_INT_STATUS		},
	{&dev_attr_software_reset,	TPS6524X_REG_SOFTWARE_RESET	},
	{&dev_attr_write_enable,	TPS6524X_REG_WRITE_ENABLE	},
	{&dev_attr_rev_id,		TPS6524X_REG_REV_ID		},
	{NULL,				-1},
};

static int tps6524x_reg_addr(struct device_attribute *attr)
{
	int i;

	for (i = 0; tps6524x_reg_map[i].attr; i++) {
		if (tps6524x_reg_map[i].attr == attr)
			return tps6524x_reg_map[i].reg;
	}
	return -EINVAL;
}

static ssize_t tps6524x_show_reg(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int r, ret;
	struct tps6524x *hw = dev_get_drvdata(dev);

	BUG_ON(!hw);

	r = tps6524x_reg_addr(attr);
	if (r < 0)
		return r;

	ret = snprintf(buf, PAGE_SIZE, "%08x\n", tps6524x_read(hw, r));

	return ret;
}

static ssize_t tps6524x_store_reg(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int r, v;
	struct tps6524x *hw = dev_get_drvdata(dev);

	BUG_ON(!hw);

	r = tps6524x_reg_addr(attr);
	if (r < 0)
		return r;

	if (sscanf(buf, "%d", &v) != 1)
		return -EINVAL;

	r = tps6524x_write(hw, r, v);

	if (r < 0)
		return r;

	return size;
}

static struct attribute_group tps6524x_registers = {
	.name  = "registers",
	.attrs = tps6524x_regs,
};

static int tps6524x_remove(struct platform_device *pdev)
{
	struct tps6524x *hw = platform_get_drvdata(pdev);
	int i;

	if (!hw)
		return 0;
	for (i = 0; i < TPS6524X_N_REGULATORS; i++) {
		if (!hw->rdev[i])
			continue;
		regulator_unregister(hw->rdev[i]);
		hw->rdev[i] = NULL;
	}
	sysfs_remove_group(&hw->dev->kobj, &tps6524x_registers);
	platform_set_drvdata(pdev, NULL);
	kfree(hw);
	return 0;
}

static int tps6524x_probe(struct platform_device *pdev)
{
	struct tps6524x *hw;
	struct device *dev = &pdev->dev;
	struct tps6524x_info *info;
	struct regulator_init_data *init_data;
	int ret = 0, i;

	init_data = dev->platform_data;
	if (!init_data) {
		dev_err(dev, "could not find regulator platform data\n");
		return -EIO;
	}

	hw = kzalloc(sizeof(struct tps6524x), GFP_KERNEL);
	if (!hw) {
		dev_err(dev, "cannot allocate regulator private data\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, hw);

	memset(hw, 0, sizeof(struct tps6524x));
	hw->dev = dev;
	hw->info = info = tps6524x_info;
	mutex_init(&hw->lock);

	ret = sysfs_create_group(&dev->kobj, &tps6524x_registers);
	if (ret)
		dev_err(dev, "failed to create sysfs entries\n");

	for (i = 0; i < TPS6524X_N_REGULATORS; i++, info++, init_data++) {
		hw->desc[i].name	= info->name;
		hw->desc[i].id		= i;
		hw->desc[i].n_voltages	= info->n_voltages;
		hw->desc[i].ops		= &tps6524x_regulator_ops;
		hw->desc[i].type	= REGULATOR_VOLTAGE;
		hw->desc[i].owner	= THIS_MODULE;

		if (info->flags & TPS6524X_FIXED_VOLTAGE)
			hw->desc[i].n_voltages = 1;

		hw->rdev[i] = regulator_register(&hw->desc[i], dev,
						 init_data, hw);
		if (IS_ERR(hw->rdev[i])) {
			ret = PTR_ERR(hw->rdev[i]);
			hw->rdev[i] = NULL;
			goto fail;
		}
	}

	dev_info(hw->dev, "initialized power management device driver\n");
	return 0;

fail:
	tps6524x_remove(pdev);
	return ret;
}

#ifdef CONFIG_PM
static int tps6524x_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* Nothing yet */
	return 0;
}

static int tps6524x_resume(struct platform_device *pdev)
{
	/* Nothing yet */
	return 0;
}
#else
#define tps6524x_suspend NULL
#define tps6524x_resume NULL
#endif

static struct platform_driver tps6524x_driver = {
	.probe		= tps6524x_probe,
	.remove		= tps6524x_remove,
	.suspend	= tps6524x_suspend,
	.resume		= tps6524x_resume,
	.driver = {
		.name	= "tps6524x",
		.owner	= THIS_MODULE,
	},
};

static int __init tps6524x_init(void)
{
	return platform_driver_register(&tps6524x_driver);
}

static void __exit tps6524x_exit(void)
{
	platform_driver_unregister(&tps6524x_driver);
}

subsys_initcall_sync(tps6524x_init);
module_exit(tps6524x_exit);

MODULE_AUTHOR("Cyril Chemparathy");
MODULE_DESCRIPTION("TPS6524X PMIC Driver");
MODULE_LICENSE("GPL");
