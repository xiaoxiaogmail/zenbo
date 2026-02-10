/*
 * Battery charger driver for TI BQ24725A
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/workqueue.h>
#include <linux/power/bq24725a_charger.h>

#define BQ24725A_CHG_OPT				0x12
#define BQ24725A_CHG_OPT_CHARGE_DISABLE	(1 << 0)
#define BQ24725A_CHG_OPT_AC_PRESENT		(1 << 4)
#define BQ24725A_CHARGE_CURRENT			0x14
#define BQ24725A_CHARGE_CURRENT_MASK	0x1fc0
#define BQ24725A_CHARGE_VOLTAGE			0x15
#define BQ24725A_CHARGE_VOLTAGE_MASK	0x7ff0
#define BQ24725A_INPUT_CURRENT			0x3f
#define BQ24725A_INPUT_CURRENT_MASK		0x1f80

#define BQ24725A_MANUFACTURER_ADDRESS	0xFE
#define BQ24725A_DEVICE_ADDRESS			0xFF
#define BQ24725A_MANUFACTURER_ID		0x40
#define BQ24725A_DEVICE_ID				0x0B

static struct bq24725a *bq24725a_charger;

static inline int bq24725a_write_word(struct i2c_client *client, u8 reg,
				     u16 value)
{
	return i2c_smbus_write_word_data(client, reg, le16_to_cpu(value));
}

static inline int bq24725a_read_word(struct i2c_client *client, u8 reg)
{
	s32 ret = i2c_smbus_read_word_data(client, reg);

	return ret < 0 ? ret : le16_to_cpu(ret);
}

static int bq24725a_update_word(struct i2c_client *client, u8 reg,
		u16 mask, u16 value)
{
	unsigned int tmp;
	int ret;

	ret = bq24725a_read_word(client, reg);
	if (ret < 0)
		return ret;

	tmp = ret & ~mask;
	tmp |= value & mask;

	return bq24725a_write_word(client, reg, tmp);
}

static ssize_t bq24725a_charger_voltage_show(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = bq24725a_charger->client;
	int ret;

	ret = bq24725a_read_word(client, BQ24725A_CHARGE_VOLTAGE);
	if (ret < 0) {
		pr_err("Failed to read charger voltage : %d\n", ret);
		return sprintf(buf, "%d mV\n", ret);
	} else
		return sprintf(buf, "%d mV\n", ret);
}
static DEVICE_ATTR(charger_voltage, S_IRUGO,
		bq24725a_charger_voltage_show, NULL);


static ssize_t bq24725a_charger_current_show(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = bq24725a_charger->client;
	int ret;

	ret = bq24725a_read_word(client, BQ24725A_CHARGE_CURRENT);
	if (ret < 0) {
		pr_err("Failed to read charger current : %d\n", ret);
		return sprintf(buf, "%d mA\n", ret);
	} else
		return sprintf(buf, "%d mA\n", ret);
}
static DEVICE_ATTR(charger_current, S_IRUGO,
		bq24725a_charger_current_show, NULL);


static ssize_t bq24725a_charger_status_show(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = bq24725a_charger->client;
	int ret;

	ret = bq24725a_read_word(client, BQ24725A_DEVICE_ADDRESS);
	if (ret < 0) {
		pr_err("Failed to read device id : %d\n", ret);
		return sprintf(buf, "%d\n", 0);
	} else if (ret != 0x000B) {
		pr_err("device id mismatch. 0x000b != 0x%04x\n", ret);
		return sprintf(buf, "%d\n", 0);
	} else
		return sprintf(buf, "%d\n", 1);
}
static DEVICE_ATTR(charger_status, S_IRUGO,
		bq24725a_charger_status_show, NULL);

static struct attribute *charger_attributes[] = {
	&dev_attr_charger_status.attr,
	&dev_attr_charger_current.attr,
	&dev_attr_charger_voltage.attr,
	NULL
};

static const struct attribute_group charger_group = {
	.attrs = charger_attributes,
};

int bq24725a_enable_charging(void)
{
	if (!bq24725a_charger)
		return -ENXIO;
	else
		return bq24725a_update_word(bq24725a_charger->client,
					BQ24725A_CHG_OPT,
					BQ24725A_CHG_OPT_CHARGE_DISABLE,
					~BQ24725A_CHG_OPT_CHARGE_DISABLE);
}

int bq24725a_disable_charging(void)
{
	if (!bq24725a_charger)
		return -ENXIO;
	else
		return bq24725a_update_word(bq24725a_charger->client,
					BQ24725A_CHG_OPT,
					BQ24725A_CHG_OPT_CHARGE_DISABLE,
					BQ24725A_CHG_OPT_CHARGE_DISABLE);
}

int bq24725a_config_charger(uint32_t charge_current, uint32_t charge_voltage)
{
	int ret;
	u16 value;

	if (!bq24725a_charger)
		return -ENXIO;

	bq24725a_charger->charge_current = charge_current;
	bq24725a_charger->charge_voltage = charge_voltage;

	pr_debug("charge current %d\n", bq24725a_charger->charge_current);
	pr_debug("charge voltage %d\n", bq24725a_charger->charge_voltage);
	pr_debug("input current 0x%04x\n", bq24725a_charger->input_current);

	if (bq24725a_charger->charge_current) {
		value = bq24725a_charger->charge_current &
				BQ24725A_CHARGE_CURRENT_MASK;

		ret = bq24725a_write_word(bq24725a_charger->client,
				BQ24725A_CHARGE_CURRENT, value);
		if (ret < 0) {
			dev_err(&bq24725a_charger->client->dev,
				"Failed to write charger current : %d\n",
				ret);
			return ret;
		}
		if (charge_current != bq24725a_read_word(
				bq24725a_charger->client,
				BQ24725A_CHARGE_CURRENT)) {
			pr_err("charge_current value write to BQ24725A_CHARGE_CURRENT fail!!!\n");
			return -ENXIO;
		}
	}

	if (bq24725a_charger->charge_voltage) {
		value = bq24725a_charger->charge_voltage &
				BQ24725A_CHARGE_VOLTAGE_MASK;

		ret = bq24725a_write_word(bq24725a_charger->client,
				BQ24725A_CHARGE_VOLTAGE, value);
		if (ret < 0) {
			dev_err(&bq24725a_charger->client->dev,
				"Failed to write charger voltage : %d\n",
				ret);
			return ret;
		}
	}

	if (bq24725a_charger->input_current) {
		value = bq24725a_charger->input_current &
				BQ24725A_INPUT_CURRENT_MASK;

		ret = bq24725a_write_word(bq24725a_charger->client,
				BQ24725A_INPUT_CURRENT, value);
		if (ret < 0) {
			dev_err(&bq24725a_charger->client->dev,
				"Failed to write input current : %d\n",
				ret);
			return ret;
		}
	}

	return 0;
}

bool bq24725a_charger_is_present(void)
{
	int ac = 0;

	if (bq24725a_charger) {
		ac = bq24725a_read_word(bq24725a_charger->client,
				BQ24725A_CHG_OPT);
		if (ac < 0) {
			dev_err(&bq24725a_charger->client->dev,
				"Failed to read charger options : %d\n", ac);
			return false;
		}
		pr_debug("ac %d\n",
			(ac & BQ24725A_CHG_OPT_AC_PRESENT) ? true : false);
		return (ac & BQ24725A_CHG_OPT_AC_PRESENT) ? true : false;
	} else
		return false;
}

void bq24725a_learn_enable(bool enable)
{
	int ret = 0;
	unsigned int tmp;

	if (bq24725a_charger) {
		pr_debug("enable %d\n", enable);
		tmp = bq24725a_read_word(bq24725a_charger->client,
				BQ24725A_CHG_OPT);
		if (enable)
			tmp |= (1 << 6);
		else
			tmp &= 0xffbf;

		ret = bq24725a_write_word(bq24725a_charger->client,
			BQ24725A_CHG_OPT, tmp);
		tmp = bq24725a_read_word(bq24725a_charger->client,
			BQ24725A_CHG_OPT);
		pr_debug("read BQ24725A_CHG_OPT 0x%04x\n", tmp);
	}
}

int bq24725a_setting_watchdog(int timer)
{
	int ret = 0;
	unsigned int tmp;

	if (bq24725a_charger) {
		tmp = bq24725a_read_word(bq24725a_charger->client,
				BQ24725A_CHG_OPT);
		if (tmp < 0)
			return tmp;

		switch (timer) {
		case 0:
			tmp = tmp & 0x9fff;
			break;
		case 44:
			tmp = tmp & 0xbfff;
			break;
		case 88:
			tmp = tmp & 0xdfff;
			break;
		default:
			break;
		}
		ret = bq24725a_write_word(bq24725a_charger->client,
				BQ24725A_CHG_OPT, tmp);
		return ret;
	} else
		return -ENXIO;
}

void bq24725a_charger_init(void)
{
	int ret = 0;

	if (bq24725a_charger) {
		/*
		set pre charging default value
		current:256mA.
		voltage:16400mV.
		input current:45W,1.92A.
		*/
		bq24725a_charger->charge_current = 0x0100;
		bq24725a_charger->charge_voltage = 0x4010;
		bq24725a_charger->input_current = 0x0780;

		ret = bq24725a_read_word(bq24725a_charger->client,
				BQ24725A_CHG_OPT);
		pr_debug("read BQ24725A_CHG_OPT 0x%04x\n", ret);
	}
}

static int bq24725a_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int ret;
	struct bq24725a *charger;

	pr_info("+++\n");

	charger = devm_kzalloc(&client->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	if (!charger->name) {
		charger->name = kasprintf(GFP_KERNEL, "bq24725a@%s",
				 dev_name(&client->dev));
		if (!charger->name) {
			dev_err(&client->dev,
				"Failed to alloc device name\n");
			return -ENOMEM;
		}
	}

	charger->client = client;
	i2c_set_clientdata(client, charger);

	/* Try to test chip via i2c bus */
	ret = bq24725a_read_word(client, BQ24725A_MANUFACTURER_ADDRESS);
	if (ret < 0)
		dev_err(&client->dev,
			"Failed to read manufacturer id : %d\n", ret);
	else if (ret != 0x0040)
		dev_err(&client->dev,
			"manufacturer id mismatch. 0x0040 != 0x%04x\n", ret);

	bq24725a_charger = charger;

	/* Register sysfs */
	ret = sysfs_create_group(&client->dev.kobj, &charger_group);
	if (ret)
		pr_err("unable to create the sysfs\n");

	pr_info("---\n");
	return 0;
}

static int bq24725a_suspend(struct device *dev)
{
	pr_info("+++ ---\n");
	return 0;
}

static int bq24725a_resume(struct device *dev)
{
	pr_info("+++ ---\n");
	return 0;
}

static void bq24725a_shutdown(struct i2c_client *client)
{
	pr_info("+++\n");
	/* remove sysfs */
	sysfs_remove_group(&client->dev.kobj, &charger_group);
	pr_info("---\n");
}

static const struct i2c_device_id bq24725a_charger_id[] = {
	{ "bq24725a-charger", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, bq24725a_charger_id);

#ifdef CONFIG_ACPI
static struct acpi_device_id bq24725_acpi_match[] = {
	{"TBQ24725", 0},
	{}
};
MODULE_DEVICE_TABLE(acpi, bq24725_acpi_match);
#endif

static const struct dev_pm_ops bq24725a_pm_ops = {
	.suspend = bq24725a_suspend,
	.resume	 = bq24725a_resume,
};

static struct i2c_driver bq24725a_charger_driver = {
	.driver = {
#ifdef CONFIG_ACPI
		.name = "TBQ24725",
#else
		.name = "bq24725a-charger",
#endif
		.owner	= THIS_MODULE,
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(bq24725_acpi_match),
#endif
		.pm	= &bq24725a_pm_ops,
	},
	.probe = bq24725a_probe,
	.shutdown = bq24725a_shutdown,
	.id_table = bq24725a_charger_id,
};

static int __init bq24725a_init(void)
{
	return i2c_add_driver(&bq24725a_charger_driver);
}
module_init(bq24725a_init);

static void __exit bq24725a_exit(void)
{
	i2c_del_driver(&bq24725a_charger_driver);
}
module_exit(bq24725a_exit);


MODULE_DESCRIPTION("bq24725a battery charging driver");
MODULE_LICENSE("GPL");

