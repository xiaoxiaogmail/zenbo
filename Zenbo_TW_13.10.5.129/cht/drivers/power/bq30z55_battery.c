/* Copyright (c) 2012-2013 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * High Level description:
 * http://www.ti.com/lit/ds/symlink/bq30z55.pdf
 * Thechnical Reference:
 * http://www.ti.com/lit/ug/sluu431/sluu431.pdf
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/regulator/consumer.h>
#include <linux/printk.h>
#include <linux/acpi.h>
#include <linux/power/bq30z55_battery.h>
#include <linux/power/bq24725a_charger.h>
#include <linux/mfd/intel_soc_pmic.h>
#include <linux/mfd/intel_soc_pmic_wcove.h>
#include <linux/alarmtimer.h>
#include <linux/wakelock.h>
#include <linux/rtc.h>
#include <linux/usb.h>

#define BQ30Z55_NAME "bq30z55"

/* SBS Commands (page 63) */
#define SBS_MANUFACTURER_ACCESS	0x00
#define SBS_BATTERY_MODE		0x03
#define SBS_TEMPERATURE			0x08
#define SBS_VOLTAGE				0x09
#define SBS_CURRENT				0x0A
#define SBS_AVG_CURRENT			0x0B
#define SBS_MAX_ERROR			0x0C

/* Relative State Of Charge */
#define SBS_RSOC				0x0D
#define SBS_REMAIN_CAPACITY		0x0F
#define SBS_FULL_CAPACITY		0x10
#define SBS_CHG_CURRENT			0x14
#define SBS_CHG_VOLTAGE			0x15
#define SBS_BATTERY_STATUS		0x16
#define SBS_CYCLE_COUNT			0x17
#define SBS_DESIGN_CAPACITY		0x18
#define SBS_DESIGN_VOLTAGE		0x19
#define SBS_SPEC_INFO			0x1A
#define SBS_MANUFACTURE_DATE	0x1B
#define SBS_SERIAL_NUMBER		0x1C
#define SBS_MANUFACTURER_NAME	0x20
#define SBS_DEVICE_NAME			0x21
#define SBS_DEVICE_CHEMISTRY	0x22
#define SBS_MANUFACTURER_DATA	0x23
#define SBS_AUTHENTICATE		0x2F
#define SBS_CELL_VOLTAGE1		0x3C
#define SBS_CELL_VOLTAGE2		0x3D
#define SBS_CELL_VOLTAGE3		0x3E
#define SBS_CELL_VOLTAGE4		0x3F

/* Extended SBS Commands (page 71) */
#define SBS_SAFETY_ALERT		0x50
#define SBS_SAFETY_STATUS		0x51
#define SBS_PE_ALERT			0x52
#define SBS_PE_STATUS			0x53
#define SBS_OPERATION_STATUS	0x54
#define SBS_CHARGING_STATUS		0x55
#define SBS_GAUGING_STATUS		0x56
#define SBS_MANUFACTURER_INFO	0x70

/* SBS Sub-Commands (16 bits) */
/* SBS_MANUFACTURER_ACCESS CMD */
#define SUBCMD_DEVICE_TYPE		0x01
#define SUBCMD_FIRMWARE_VERSION	0x02
#define SUBCMD_HARDWARE_VERSION	0x03
#define SUBCMD_SHUTDOWN_MODE	0x10

/* SBS_BATTERY_STATUS */
#define BAT_STATUS_SBS_ERROR	0x0F
#define BAT_STATUS_EMPTY		BIT(4)
#define BAT_STATUS_FULL			BIT(5)
#define BAT_STATUS_DISCHARGING	BIT(6)
#define BAT_STATUS_TDA			BIT(11)
#define BAT_STATUS_OTA			BIT(12)
#define BAT_STATUS_TCA			BIT(14)
#define BAT_STATUS_OCA			BIT(15)

/* SBS_SAFETY_STATUS */
#define SAFETY_STATUS_UNDERVOLTAGE	(1 << 0)
#define SAFETY_STATUS_OVERVOLTAGE	(1 << 1)
#define SAFETY_STATUS_OVERHEAT		(3 << 12)

#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN	(-2731)
#define BQ_TERMINATION_CURRENT_MA			200
#define BQ_MAX_STR_LEN						32
#define BQ30Z55_INVALID_TEMPERATURE			-999
#define ACOK		493	//SW79
#define DRIVER_POWER	329	//E15
#define LED_POWER	292	//SE64
#define CAP_POWER	394	//N53
#define CIR_POWER	387	//N46

#define HW_ID_PR 3

static struct bq30z55_device *bq30z55_dev;
static bool dc_on;
static int fake_battery = -EINVAL;
module_param(fake_battery, int, 0644);
char checkmode[8] = {0,};
bool enable_ship, cos_mode;
static struct alarm bq30z55_alarm_timer;
struct timespec ts;
struct wake_lock bq30z55_wakelock;
static bool enable_bat_trace_log, detect_dock;
static int check_num;

#define BQ30Z55_DEBUG_REG(x) {#x, SBS_##x, 0}
#define BQ30Z55_DEBUG_SUBREG(x, y) {#y, SBS_##x, SUBCMD_##y}

/* Note: Some register can be read only in Unsealed mode */
static struct debug_reg bq30z55_debug_regs[] = {
	BQ30Z55_DEBUG_REG(MANUFACTURER_ACCESS),
	BQ30Z55_DEBUG_REG(BATTERY_MODE),
	BQ30Z55_DEBUG_REG(TEMPERATURE),
	BQ30Z55_DEBUG_REG(VOLTAGE),
	BQ30Z55_DEBUG_REG(CURRENT),
	BQ30Z55_DEBUG_REG(AVG_CURRENT),
	BQ30Z55_DEBUG_REG(MAX_ERROR),
	BQ30Z55_DEBUG_REG(RSOC),
	BQ30Z55_DEBUG_REG(REMAIN_CAPACITY),
	BQ30Z55_DEBUG_REG(FULL_CAPACITY),
	BQ30Z55_DEBUG_REG(CHG_CURRENT),
	BQ30Z55_DEBUG_REG(CHG_VOLTAGE),
	BQ30Z55_DEBUG_REG(BATTERY_STATUS),
	BQ30Z55_DEBUG_REG(CYCLE_COUNT),
	BQ30Z55_DEBUG_REG(DESIGN_CAPACITY),
	BQ30Z55_DEBUG_REG(DESIGN_VOLTAGE),
	BQ30Z55_DEBUG_REG(SPEC_INFO),
	BQ30Z55_DEBUG_REG(MANUFACTURE_DATE),
	BQ30Z55_DEBUG_REG(SERIAL_NUMBER),
	BQ30Z55_DEBUG_REG(MANUFACTURER_NAME),
	BQ30Z55_DEBUG_REG(DEVICE_NAME),
	BQ30Z55_DEBUG_REG(DEVICE_CHEMISTRY),
	BQ30Z55_DEBUG_REG(MANUFACTURER_DATA),
	BQ30Z55_DEBUG_REG(AUTHENTICATE),
	BQ30Z55_DEBUG_REG(CELL_VOLTAGE1),
	BQ30Z55_DEBUG_REG(CELL_VOLTAGE2),
	BQ30Z55_DEBUG_REG(CELL_VOLTAGE3),
	BQ30Z55_DEBUG_REG(CELL_VOLTAGE4),
	BQ30Z55_DEBUG_REG(SAFETY_ALERT),
	BQ30Z55_DEBUG_REG(SAFETY_STATUS),
	BQ30Z55_DEBUG_REG(PE_ALERT),
	BQ30Z55_DEBUG_REG(PE_STATUS),
	BQ30Z55_DEBUG_REG(OPERATION_STATUS),
	BQ30Z55_DEBUG_REG(CHARGING_STATUS),
	BQ30Z55_DEBUG_REG(GAUGING_STATUS),
	BQ30Z55_DEBUG_REG(MANUFACTURER_INFO),
	BQ30Z55_DEBUG_SUBREG(MANUFACTURER_ACCESS, DEVICE_TYPE),
	BQ30Z55_DEBUG_SUBREG(MANUFACTURER_ACCESS, FIRMWARE_VERSION),
	BQ30Z55_DEBUG_SUBREG(MANUFACTURER_ACCESS, HARDWARE_VERSION),
};

static int bq30z55_read_reg(struct i2c_client *client, u8 reg)
{
	int val;

	val = i2c_smbus_read_word_data(client, reg);
	if (val < 0)
		pr_debug("i2c read fail. reg = 0x%x.ret = %d.\n", reg, val);
	else
		pr_debug("reg = 0x%02X. val = 0x%04X.\n", reg , val);

	return val;
}

static int bq30z55_write_reg(struct i2c_client *client, u8 reg, u16 val)
{
	int ret;

	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0)
		pr_debug("i2c read fail. reg = 0x%x.val = 0x%x.ret = %d.\n",
		       reg, val, ret);
	else
		pr_debug("reg = 0x%02X.val = 0x%02X.\n", reg , val);

	return ret;
}

static int bq30z55_read_subcmd(struct i2c_client *client, u8 reg, u16 subcmd)
{
	int ret;
	u8 buf[4];
	u16 val = 0;

	buf[0] = reg;
	buf[1] = subcmd & 0xFF;
	buf[2] = (subcmd >> 8) & 0xFF;

	/* Control sub-command */
	ret = i2c_master_send(client, buf, 3);
	if (ret < 0) {
		pr_err("i2c tx fail. reg = 0x%x.ret = %d.\n", reg, ret);
		return ret;
	}
	udelay(66);

	/* Read Result of subcmd */
	ret = i2c_master_send(client, buf, 1);
	memset(buf, 0xAA, sizeof(buf));
	ret = i2c_master_recv(client, buf, 2);
	if (ret < 0) {
		pr_err("i2c rx fail. reg = 0x%x.ret = %d.\n", reg, ret);
		return ret;
	}
	val = (buf[1] << 8) + buf[0];

	pr_debug("reg = 0x%02X.subcmd = 0x%x.val = 0x%04X.\n",
		 reg , subcmd, val);

	return val;
}

/*
 * Read a string from a device.
 * Returns string length on success or error on failure (negative value).
 */
static int bq30z55_read_string(struct i2c_client *client, u8 reg, char *str,
			       u8 max_len)
{
	s32 ret = 0, len = 0;
	u8 block_buffer[max_len+1];

	len = i2c_smbus_read_byte_data(client, reg);
	if (len < 0)
		return len;

	if (len > max_len)
		pr_err("len %d > max_len %d\n", len, max_len);

	ret = i2c_smbus_read_i2c_block_data(client, reg, len + 1, block_buffer);
	if (ret < 0)
		return ret;

	/* block_buffer[0] == block_length */
	memcpy(str, block_buffer + 1, len);
	str[len] = '\0';
	pr_debug("len = %d.str = %s.\n", len, str);

	return le16_to_cpu(len);
}

static int bq30z55_read_safety_status(struct i2c_client *client)
{
	int ret, i, len;
	u8 data[5];
	u32 val;

	ret = bq30z55_write_reg(client, SBS_MANUFACTURER_ACCESS, SBS_SAFETY_STATUS);
	ret = i2c_smbus_read_i2c_block_data(client, SBS_MANUFACTURER_DATA, 5, data);
	len = data[0];

	val = data[1] + (data[2] << 8) + (data[3] << 16) + (data[4] << 24);
	for (i = 0; i < len + 1; i ++)
		pr_debug("data %d = %d\n", i, data[i]);

	pr_debug("len = %d val %#X\n", len, val);
	return val;
}

void enable_battery_ship_mode(bool enable)
{
	struct i2c_client *client = bq30z55_dev->client;
	int ret;
	u8 srcwakecfg = 0;

	if (enable) {
		pr_info("Enable battery ship mode and disable pmic wakeup !!!\n");
		intel_soc_pmic_writeb(0x6E8B, 0x30);
		srcwakecfg = intel_soc_pmic_readb(0x6E8B);
		pr_info("srcwakecfg = 0x%.4x\n", srcwakecfg);

		ret = bq30z55_write_reg(client, SBS_MANUFACTURER_ACCESS, SUBCMD_SHUTDOWN_MODE);
		msleep(500);
		ret = bq30z55_write_reg(client, SBS_MANUFACTURER_ACCESS, SUBCMD_SHUTDOWN_MODE);
		if (ret < 0)
			pr_err("I2C failure when writing SUBCMD_SHUTDOWN_MODE.\n");
	}
}

static ssize_t show_battery_ship_mode(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = bq30z55_dev->client;
	int rsoc, ret;

	rsoc = bq30z55_read_reg(client, SBS_RSOC);
	pr_info("rsoc %d\n", rsoc);

	if (enable_ship) {
		if (rsoc < 0) {
			bq30z55_dev->ship_mode = true;
			return sprintf(buf, "%d\n", bq30z55_dev->ship_mode);
		} else if(!gpio_get_value(ACOK)) {
			ret = 2;
			pr_info("Please plug in AC adapter\n");
			return sprintf(buf, "%d\n", ret);
		} else if (rsoc < 60) {
			ret = 3;
			pr_info("%s\n", "RSOC below the 60%");
			return sprintf(buf, "%d\n", ret);
		} else if (rsoc > 80) {
			ret = 4;
			pr_info("%s\n", "RSOC exceeds the 80%");
			return sprintf(buf, "%d\n", ret);
		} else {
			cancel_delayed_work(&bq30z55_dev->periodic_update_work);
			bq30z55_dev->stop_wq = 1;
			bq24725a_learn_enable(false);
			enable_battery_ship_mode(true);
			dc_on = true;
			power_supply_changed(&bq30z55_dev->ac_psy);
			bq30z55_dev->ship_mode = false;
			return sprintf(buf, "%d\n", bq30z55_dev->ship_mode);
		}
	} else
		return sprintf(buf, "%d\n", bq30z55_dev->ship_mode);

}
static DEVICE_ATTR(battery_ship, S_IRUGO,
		show_battery_ship_mode, NULL);


#ifdef DEBUG_IMAGE
static ssize_t store_battery_stop_wq(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	if (buf[0] ==  '1')
		bq30z55_dev->stop_wq = 1;

	if (buf[0] ==  '0')
		bq30z55_dev->stop_wq = 0;

	return count;
}
static ssize_t show_battery_stop_wq(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", bq30z55_dev->stop_wq);
}
static DEVICE_ATTR(battery_stop_wq, S_IWUSR | S_IRUGO,
		show_battery_stop_wq, store_battery_stop_wq);

static ssize_t store_lower_upper_limit(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	int tmp1, tmp2;

	sscanf(buf, "%d %d", &tmp1, &tmp2);
	if (tmp2 > tmp1) {
		if (tmp1 >= 1 && tmp1 <= 99)
			bq30z55_dev->lower_limit = tmp1;
		else {
			pr_info("lower_limit must more than or equal to 1\n");
			bq30z55_dev->lower_limit = 0;
		}

		if (tmp2 <= 100)
			bq30z55_dev->upper_limit = tmp2;
		else {
			pr_info("upper_limit must less than or equal to 100\n");
			bq30z55_dev->upper_limit = 0;
		}

		pr_info("lower_limit %d\n", bq30z55_dev->lower_limit);
		pr_info("upper_limit %d\n", bq30z55_dev->upper_limit);
	} else {
		bq30z55_dev->lower_limit = 0;
		bq30z55_dev->upper_limit = 0;
		pr_info("upper limit must more than the lower limit\n");
	}

	return count;
}
static ssize_t show_lower_upper_limit(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "lower_limit %d, upper_limit %d\n",
		bq30z55_dev->lower_limit, bq30z55_dev->upper_limit);
}
static DEVICE_ATTR(lower_upper_limit, S_IWUSR | S_IRUGO,
		show_lower_upper_limit, store_lower_upper_limit);

static ssize_t battery_limit_enable_store(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	if (buf[0] == '1') {
		bq30z55_dev->bat_limit_enable = 1;
		pr_info("charger disable and enable ATD battery limit\n");
	}
	if (buf[0] == '0') {
		bq30z55_dev->bat_limit_enable = 0;
		pr_info("charger enable and disable ATD battery limit\n");
	}
	return count;
}
static ssize_t battery_limit_enable_show(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", bq30z55_dev->bat_limit_enable);
}
static DEVICE_ATTR(battery_limit_enable, S_IWUSR | S_IRUGO,
		battery_limit_enable_show, battery_limit_enable_store);

static ssize_t ac_periodic_store(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	int tmp = 0;
	sscanf(buf, "%d", &tmp);
	bq30z55_dev->ac_periodic = tmp;
	pr_info("change ac_periodic to %d\n", bq30z55_dev->ac_periodic);
	cancel_delayed_work(&bq30z55_dev->periodic_update_work);
	if (!bq30z55_dev->stop_wq) {
		queue_delayed_work(bq30z55_dev->bq30z55_wq,
			&bq30z55_dev->periodic_update_work, 0 * HZ);
	}
	return count;
}
static ssize_t ac_periodic_show(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", bq30z55_dev->ac_periodic);
}
static DEVICE_ATTR(ac_periodic, S_IWUSR | S_IRUGO,
		ac_periodic_show, ac_periodic_store);

static ssize_t bat_periodic_store(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	int tmp = 0;
	sscanf(buf, "%d", &tmp);
	bq30z55_dev->bat_periodic = tmp;
	pr_info("change bat_periodic to %d\n", bq30z55_dev->bat_periodic);
	cancel_delayed_work(&bq30z55_dev->periodic_update_work);
	if (!bq30z55_dev->stop_wq) {
		queue_delayed_work(bq30z55_dev->bq30z55_wq,
			&bq30z55_dev->periodic_update_work, 0 * HZ);
	}
	return count;
}
static ssize_t bat_periodic_show(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", bq30z55_dev->bat_periodic);
}
static DEVICE_ATTR(bat_periodic, S_IWUSR | S_IRUGO,
		bat_periodic_show, bat_periodic_store);

static ssize_t bat_log_store(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	if (buf[0] == '1') {
		enable_bat_trace_log = true;
		pr_info("enable battery trace log\n");
	}
	if (buf[0] == '0') {
		enable_bat_trace_log = false;
		pr_info("disable battery trace log\n");
	}
	return count;
}
static ssize_t bat_log_show(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", enable_bat_trace_log);
}
static DEVICE_ATTR(bat_trace_log, S_IWUSR | S_IRUGO,
		bat_log_show, bat_log_store);

#else

static ssize_t show_battery_stop_wq(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", bq30z55_dev->stop_wq);
}
static DEVICE_ATTR(battery_stop_wq, S_IRUGO,
		show_battery_stop_wq, NULL);

#endif
static ssize_t show_battery_input_voltage(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	int in_volt;
	struct i2c_client *client = bq30z55_dev->client;

	in_volt = bq30z55_read_reg(client, SBS_VOLTAGE);
	return sprintf(buf, "%d mV\n", in_volt);
}
static DEVICE_ATTR(battery_input_voltage, S_IRUGO,
		show_battery_input_voltage, NULL);

static ssize_t show_battery_input_current(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	s16 in_current;
	struct i2c_client *client = bq30z55_dev->client;

	in_current = bq30z55_read_reg(client, SBS_CURRENT);
	return sprintf(buf, "%d mA\n", in_current);
}
static DEVICE_ATTR(battery_input_current, S_IRUGO,
		show_battery_input_current, NULL);

static ssize_t show_battery_chg_voltage(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	int chg_volt;
	struct i2c_client *client = bq30z55_dev->client;

	chg_volt = bq30z55_read_reg(client, SBS_CHG_VOLTAGE);
	if (chg_volt < 0) {
		pr_err("I2C failure when reading CHG_VOLTAGE.\n");
		return sprintf(buf, "%d mV\n", chg_volt);
	} else
		return sprintf(buf, "%d mV\n", chg_volt);
}
static DEVICE_ATTR(battery_chg_voltage, S_IRUGO,
		show_battery_chg_voltage, NULL);

static ssize_t show_battery_chg_current(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	int chg_current;
	struct i2c_client *client = bq30z55_dev->client;

	chg_current = bq30z55_read_reg(client, SBS_CHG_CURRENT);
	if (chg_current < 0) {
		pr_err("I2C failure when reading CHG_CURRENT.\n");
		return sprintf(buf, "%d mA\n", chg_current);
	} else
		return sprintf(buf, "%d mA\n", chg_current);
}
static DEVICE_ATTR(battery_chg_current, S_IRUGO,
		show_battery_chg_current, NULL);

static ssize_t show_battery_temperature(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	int temp;
	struct i2c_client *client = bq30z55_dev->client;

	/* temperature resolution 0.1 Kelvin */
	temp = bq30z55_read_reg(client, SBS_TEMPERATURE);

	if (temp < 0) {
		pr_err("I2C failure when reading temperature.\n");
		return sprintf(buf, "%d\n", BQ30Z55_INVALID_TEMPERATURE);
	}

	/* Set 75 (3481 - 2731) degrees to high temperature */
	if (temp > 3481) {
		pr_err("Reading high temperature, try again!!!\n");
		temp = bq30z55_read_reg(client, SBS_TEMPERATURE);
		if (temp > 3481)
			pr_err("High temperature\n");
	}

	temp = (temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN) / 10;
	return sprintf(buf, "%d C\n", temp);
}
static DEVICE_ATTR(battery_temperature, S_IRUGO,
		show_battery_temperature, NULL);

static ssize_t show_battery_capacity(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	int capacity;
	struct i2c_client *client = bq30z55_dev->client;

	capacity = bq30z55_read_reg(client, SBS_RSOC);
	if (capacity < 0)
		pr_err("I2C failure when reading rsoc.\n");

	return sprintf(buf, "%d %%\n", capacity);
}
static DEVICE_ATTR(battery_capacity, S_IRUGO,
		show_battery_capacity, NULL);

static ssize_t show_battery_status(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	u16 battery_status;
	struct i2c_client *client = bq30z55_dev->client;

	battery_status = bq30z55_read_reg(client, SBS_BATTERY_STATUS);
	if ((battery_status & BAT_STATUS_SBS_ERROR) != 0x0) {
		pr_info("battery_status 0x%04X\n",
			(battery_status & BAT_STATUS_SBS_ERROR));
		return sprintf(buf, "%d\n", 0);
	} else
		return sprintf(buf, "%d\n", 1);
}
static DEVICE_ATTR(battery_status, S_IRUGO,
		show_battery_status, NULL);

static struct attribute *battery_attributes[] = {
	&dev_attr_battery_status.attr,
	&dev_attr_battery_capacity.attr,
	&dev_attr_battery_temperature.attr,
	&dev_attr_battery_chg_current.attr,
	&dev_attr_battery_chg_voltage.attr,
	&dev_attr_battery_input_current.attr,
	&dev_attr_battery_input_voltage.attr,
	&dev_attr_battery_ship.attr,
	&dev_attr_battery_stop_wq.attr,
#ifdef DEBUG_IMAGE
	&dev_attr_battery_limit_enable.attr,
	&dev_attr_lower_upper_limit.attr,
	&dev_attr_ac_periodic.attr,
	&dev_attr_bat_periodic.attr,
	&dev_attr_bat_trace_log.attr,
#endif
	NULL
};

static const struct attribute_group battery_group = {
	.attrs = battery_attributes,
};

bool check_battery_ship(void)
{
	if (enable_ship)
		return true;
	else
		return false;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or -99.9 C if something fails.
 */
static int bq30z55_read_temperature(struct i2c_client *client)
{
	int temp;

	/* temperature resolution 0.1 Kelvin */
	temp = bq30z55_read_reg(client, SBS_TEMPERATURE);

	if (temp < 0) {
		pr_err("I2C failure when reading temperature.\n");
		return BQ30Z55_INVALID_TEMPERATURE;
	}

	/* Set 75 (3481 - 2731) degrees to high temperature */
	if (temp > 3481) {
		pr_err("Reading high temperature, try again!!!\n");
		temp = bq30z55_read_reg(client, SBS_TEMPERATURE);
		if (temp > 3481)
			pr_err("High temperature\n");
	}

	temp = temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
	pr_debug("temp = %d C\n", temp / 10);
	return temp;
}

/*
 * Return the battery Voltage in microvolts 0..20 V
 * Or < 0 if something fails.
 */
static int bq30z55_read_voltage(struct i2c_client *client)
{
	int mvolt = 0;

	mvolt = bq30z55_read_reg(client, SBS_VOLTAGE);
	if (mvolt < 0)
		return mvolt;

	pr_debug("mvolt = %d mV.\n", mvolt);

	return mvolt;
}

static int bq30z55_read_charge_voltage(struct i2c_client *client)
{
	int mvolt = 0;

	mvolt = bq30z55_read_reg(client, SBS_CHG_VOLTAGE);
	if (mvolt < 0)
		return mvolt;

	pr_debug("charge volt = %d mV.\n", mvolt);

	return mvolt;
}

/*
 * Return the battery Current in miliamps
 * Or 0 if something fails.
 * Positive current indicates charging
 * Negative current indicates discharging.
 * Current-now is calculated every second.
 */
static int bq30z55_read_current(struct i2c_client *client)
{
	s16 current_ma = 0;

	current_ma = bq30z55_read_reg(client, SBS_CURRENT);

	pr_debug("current = %d mA.\n", current_ma);

	return current_ma;
}

static int bq30z55_read_charge_current(struct i2c_client *client)
{
	int current_ma = 0;

	current_ma = bq30z55_read_reg(client, SBS_CHG_CURRENT);

	pr_debug("charge curr = %d mA.\n", current_ma);

	return current_ma;
}

/*
 * Return the Average battery Current in miliamps
 * Or 0 if something fails.
 * Positive current indicates charging
 * Negative current indicates discharging.
 * Average Current is the rolling 1 minute average current.
 */
static int bq30z55_read_avg_current(struct i2c_client *client)
{
	s16 current_ma = 0;

	current_ma = bq30z55_read_reg(client, SBS_AVG_CURRENT);

	pr_debug("avg_current=%d mA.\n", current_ma);

	return current_ma;
}

static int tune_rsoc(int rsoc)
{
	int rrsoc;

	if (rsoc >= 90)
		rrsoc = rsoc + 5;
	else if (rsoc < 90 && rsoc >= 80)
		rrsoc = rsoc + 4;
	else if (rsoc < 80 && rsoc >= 70)
		rrsoc = rsoc + 3;
	else if (rsoc < 70 && rsoc >= 60)
		rrsoc = rsoc + 2;
	else if (rsoc < 60 && rsoc >= 50)
		rrsoc = rsoc + 1;
	else
		rrsoc = rsoc;

	if (rrsoc >= 100)
		rrsoc = 100;

	return rrsoc;
}

/*
 * Return the battery Relative-State-Of-Charge 0..100 %
 * Or negative value if something fails.
 */
static int bq30z55_read_rsoc(struct i2c_client *client)
{
	int rsoc = 0;

	if (fake_battery != -EINVAL) {
		pr_debug("Reporting Fake SOC = %d\n", fake_battery);
		return fake_battery;
	}

	rsoc = bq30z55_read_reg(client, SBS_RSOC);
	if (rsoc < 0) {
		pr_err("I2C failure when reading rsoc.\n");
		return rsoc;
	}

	pr_debug("real rsoc = %d.\n", rsoc);
	rsoc = tune_rsoc(rsoc);
	pr_debug("tune rsoc = %d.\n", rsoc);

	return rsoc;
}

/*
 * Return the battery Capacity in mAh.
 * Or 0 if something fails.
 */
static int bq30z55_read_full_capacity(struct i2c_client *client)
{
	int capacity = 0;

	capacity = bq30z55_read_reg(client, SBS_FULL_CAPACITY);
	if (capacity < 0)
		return 0;

	pr_debug("full-capacity = %d mAh.\n", capacity);

	return capacity;
}

/*
 * Return the battery Capacity in mAh.
 * Or 0 if something fails.
 */
static int bq30z55_read_remain_capacity(struct i2c_client *client)
{
	int capacity = 0;

	capacity = bq30z55_read_reg(client, SBS_REMAIN_CAPACITY);
	if (capacity < 0)
		return 0;

	pr_debug("remain-capacity = %d mAh.\n", capacity);

	return capacity;
}

static int bq30z55_get_health(struct i2c_client *client)
{
	int safety_status;

	safety_status = bq30z55_read_safety_status(client);
	pr_debug("safety_status 0x%08X\n", safety_status);

	if (safety_status < 0) {
		pr_debug("POWER_SUPPLY_HEALTH_UNKNOWN\n");
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	} else if (safety_status & SAFETY_STATUS_UNDERVOLTAGE) {
		pr_debug("POWER_SUPPLY_HEALTH_DEAD\n");
		return POWER_SUPPLY_HEALTH_DEAD;
	} else if (safety_status & SAFETY_STATUS_OVERVOLTAGE) {
		pr_debug("POWER_SUPPLY_HEALTH_OVERVOLTAGE\n");
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	} else if (safety_status & SAFETY_STATUS_OVERHEAT) {
		pr_debug("POWER_SUPPLY_HEALTH_OVERHEAT\n");
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	} else {
		pr_debug("POWER_SUPPLY_HEALTH_GOOD\n");
		return POWER_SUPPLY_HEALTH_GOOD;
	}
}

static int bq30z55_get_prop_status(struct i2c_client *client)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	int rsoc;
	s16 current_ma = 0;
	u16 battery_status;

	battery_status = bq30z55_read_reg(client, SBS_BATTERY_STATUS);
	pr_info("battery_status 0x%04X\n", battery_status);

	rsoc = bq30z55_read_rsoc(client);
	current_ma = bq30z55_read_current(client);

	if (battery_status & BAT_STATUS_EMPTY)
		pr_info("Battery report Empty.\n");

	/* Battery may report FULL before rsoc is 100%
	 * for protection and cell-balancing.
	 * The FULL report may remain when rsoc drops from 100%.
	 * If battery is full but DC-Jack is removed then report discahrging.
	 */
	if (battery_status & BAT_STATUS_FULL || rsoc == 100) {
		pr_debug("Battery report Full.\n");
		if (!gpio_get_value(ACOK))
			return POWER_SUPPLY_STATUS_DISCHARGING;
		return POWER_SUPPLY_STATUS_FULL;
	}

	/*
	* Positive current indicates charging
	* Negative current indicates discharging.
	* Charging is stopped at termination-current.
	*/
	if (gpio_get_value(ACOK)) {
		pr_info("Charging.\n");
		status = POWER_SUPPLY_STATUS_CHARGING;
	} else if (current_ma < 0) {
		pr_info("Discharging.\n");
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		pr_info("Not Charging.\n");
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	return status;
}

static bool bq30z55_get_prop_present(struct i2c_client *client)
{
	int val;

	val = bq30z55_read_reg(client, SBS_BATTERY_STATUS);

	/* If the bq30z55 is inside the battery pack
	 * then when battery is removed the i2c transfer will fail.
	 */

	if (val < 0)
		return false;

	/* TODO - support when bq30z55 is not embedded in battery pack */

	return true;
}

/*
 * User sapce read the battery info.
 * Get data online via I2C from the battery gauge.
 */
static int bq30z55_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	int ret = 0;
	struct bq30z55_device *dev = container_of(psy,
						  struct bq30z55_device,
						  batt_psy);
	struct i2c_client *client = dev->client;
	static char str[BQ_MAX_STR_LEN + 1];

	pr_debug("psp %d\n", psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq30z55_get_prop_status(client);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq30z55_get_health(client);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq30z55_get_prop_present(client);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq30z55_read_voltage(client);
		val->intval *= 1000; /* mV to uV */
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq30z55_read_rsoc(client);
		if (val->intval < 0)
			ret = -EINVAL;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* Positive current indicates drawing */
		val->intval = bq30z55_read_current(client);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		/* Positive current indicates drawing */
		val->intval = bq30z55_read_avg_current(client);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq30z55_read_temperature(client);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = bq30z55_read_full_capacity(client);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = bq30z55_read_remain_capacity(client);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		bq30z55_read_string(client, SBS_DEVICE_NAME, str, 32);
		val->strval = str;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		bq30z55_read_string(client, SBS_MANUFACTURER_NAME, str, 32);
		val->strval = str;
		break;
	default:
		pr_err("psp %d Not supoprted.\n", psp);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS && dc_on)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
	return -EINVAL;
	}
	return ret;
}

static int bq30z55_set_reg(void *data, u64 val)
{
	struct debug_reg *dbg = data;
	u8 reg = dbg->reg;
	int ret;
	struct i2c_client *client = bq30z55_dev->client;

	ret = bq30z55_write_reg(client, reg, val);

	return ret;
}

static int bq30z55_get_reg(void *data, u64 *val)
{
	struct debug_reg *dbg = data;
	u8 reg = dbg->reg;
	u16 subcmd = dbg->subcmd;
	int ret;
	struct i2c_client *client = bq30z55_dev->client;

	if (subcmd)
		ret = bq30z55_read_subcmd(client, reg, subcmd);
	else
		ret = bq30z55_read_reg(client, reg);
	if (ret < 0)
		return ret;

	*val = ret;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(reg_fops, bq30z55_get_reg, bq30z55_set_reg,
			"0x%04llx\n");

static int bq30z55_create_debugfs_entries(struct bq30z55_device *bq30z55_dev)
{
	int i;

	bq30z55_dev->dent = debugfs_create_dir(BQ30Z55_NAME, NULL);
	if (IS_ERR(bq30z55_dev->dent)) {
		pr_err("bq30z55 driver couldn't create debugfs dir\n");
		return -EFAULT;
	}

	for (i = 0 ; i < ARRAY_SIZE(bq30z55_debug_regs) ; i++) {
		char *name = bq30z55_debug_regs[i].name;
		struct dentry *file;
		void *data = &bq30z55_debug_regs[i];

		file = debugfs_create_file(name, 0644, bq30z55_dev->dent,
					   data, &reg_fops);
		if (IS_ERR(file)) {
			pr_err("debugfs_create_file %s failed.\n", name);
			return -EFAULT;
		}
	}

	return 0;
}

static int bq30z55_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	pr_debug("psp = %d.val = %d.\n", psp, val->intval);

	return -EINVAL;
}

static int bq30z55_register_bat_psy(struct bq30z55_device *bq30z55_dev)
{
	int ret;

	bq30z55_dev->batt_psy.name = "battery";
	bq30z55_dev->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	bq30z55_dev->batt_psy.num_supplicants = 0;
	bq30z55_dev->batt_psy.properties = pm_power_props;
	bq30z55_dev->batt_psy.num_properties = ARRAY_SIZE(pm_power_props);
	bq30z55_dev->batt_psy.get_property = bq30z55_get_property;
	bq30z55_dev->batt_psy.set_property = bq30z55_set_property;

	ret = power_supply_register(&bq30z55_dev->client->dev,
				&bq30z55_dev->batt_psy);
	if (ret) {
		pr_err("failed to register battery supply. ret=%d.\n", ret);
		return ret;
	}

	return 0;
}

static int bq30z55_register_ac_psy(struct bq30z55_device *bq30z55_dev)
{
	int ret;

	bq30z55_dev->ac_psy.name = "ac";
	bq30z55_dev->ac_psy.type = POWER_SUPPLY_TYPE_MAINS;
	bq30z55_dev->ac_psy.num_supplicants = 0;
	bq30z55_dev->ac_psy.properties = power_online_prop;
	bq30z55_dev->ac_psy.num_properties = ARRAY_SIZE(power_online_prop);
	bq30z55_dev->ac_psy.get_property = power_get_property;

	ret = power_supply_register(&bq30z55_dev->client->dev,
				&bq30z55_dev->ac_psy);
	if (ret) {
		pr_err("failed to register dc supply. ret=%d.\n", ret);
		return ret;
	}

	return 0;
}

static uint32_t resolution_current(uint32_t charge_curr)
{
	uint32_t current_mask = 0x1fc0;
	uint32_t current_result = 0x0;
	int match = 0;

	match = charge_curr % 64;
	if (!match)
		pr_debug("require current match charge current resolution\n");
	else
		pr_debug("require current dismatch charge current resolution\n");

	current_result = charge_curr / 64;
	current_result = current_result << 6;
	current_result = current_result & current_mask;
	pr_debug("current %d\n", current_result);
	return current_result;
}


static uint32_t resolution_voltage(uint32_t charge_volt)
{
	uint32_t voltage_mask = 0x7ff0;
	uint32_t voltage_result = 0x0;
	int match = 0;

	match = charge_volt % 16;
	if (!match)
		pr_debug("battery require voltage match charge voltage resolution\n");
	else
		pr_debug("battery require voltage dismatch charge voltage resolution\n");

	voltage_result = charge_volt / 16;
	voltage_result = voltage_result << 4;
	voltage_result = voltage_result & voltage_mask;
	pr_debug("voltage %d\n", voltage_result);
	return voltage_result;
}

/**
 * Update userspace every 10 seconds when plugged-in AC adapter.
 * Normally it takes more than 120 minutes (two hours) to
 * charge/discahrge the battery,
 * so updating every 10 seconds should be enough for 1% change
 * detection.
 */
static void bq30z55_periodic_update_worker(struct work_struct *work)
{
	struct bq30z55_device *dev = container_of(work,
						struct bq30z55_device,
						periodic_update_work.work);
	uint32_t charge_curr, charge_volt, volt;
	u16 battery_status;
	int ret, temp, rsoc, RC, FCC, cycle_count, err;
	s16 curr;
	struct rtc_time tm;
	struct rtc_device *rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);

	if (bq30z55_dev->stop_wq) {
		pr_info("stop_wq will return");
		return;
	}

	pr_debug("Notify battery status to user space.\n");
	pr_debug("battery ship mode %d\n", bq30z55_dev->ship_mode);
	pr_debug("check_num %d\n", check_num);

	if (gpio_get_value(ACOK)) {
		// Need to initial charge parameters.
		bq24725a_charger_init();

		battery_status = bq30z55_read_reg(dev->client,
							SBS_BATTERY_STATUS);
		rsoc = bq30z55_read_reg(dev->client, SBS_RSOC);
		temp = bq30z55_read_temperature(dev->client);
		temp = temp / 10; /* in degree celsius */
		dc_on = true;

		if (temp > 0 && rsoc > 0 && bq30z55_dev->ship_mode) {
			bq30z55_dev->ship_mode = false;
			fake_battery = -EINVAL;
		}

		if (rsoc != 100 &&
			!(battery_status & BAT_STATUS_TCA) &&
			!(battery_status & BAT_STATUS_OTA) &&
			!(battery_status & BAT_STATUS_OCA) &&
			!(battery_status & BAT_STATUS_FULL) &&
			!bq30z55_dev->ship_mode) {

			charge_curr = bq30z55_read_charge_current(dev->client);
			charge_curr = resolution_current(charge_curr);
			charge_volt = bq30z55_read_charge_voltage(dev->client);
			charge_volt = resolution_voltage(charge_volt);

			ret = bq24725a_config_charger(charge_curr, charge_volt);
			if (ret < 0) {
				pr_err("failed in configuring charge\n");
				dc_on = false;
			} else {
#ifdef DEBUG_IMAGE
				pr_debug("bat_limit_enable %d rsoc %d\n",
					bq30z55_dev->bat_limit_enable, rsoc);
				if (bq30z55_dev->bat_limit_enable) {
					if (rsoc <= bq30z55_dev->lower_limit) {
						ret = bq24725a_enable_charging();
						if (ret < 0) {
							pr_err("failed in enable charging\n");
							dc_on = false;
						}
						bq24725a_learn_enable(false);
					} else if (rsoc == bq30z55_dev->upper_limit) {
						ret = bq24725a_disable_charging();
						if (ret < 0)
							pr_err("failed in disable charging\n");
						dc_on = false;
						bq24725a_learn_enable(false);
					} else if (rsoc > bq30z55_dev->upper_limit) {
						ret = bq24725a_disable_charging();
						if (ret < 0)
							pr_err("failed in disable charging\n");
						dc_on = false;
						bq24725a_learn_enable(true);
					}
				} else {
					ret = bq24725a_enable_charging();
					if (ret < 0) {
						pr_err("failed in enable charging\n");
						dc_on = false;
					}
					bq24725a_learn_enable(false);
				}
#else
				ret = bq24725a_enable_charging();
				if (ret < 0) {
					pr_err("failed in enable charging\n");
					dc_on = false;
				}
#endif
			}
		} else {
			if (bq30z55_dev->ship_mode) {
				charge_curr = resolution_current(256);
				charge_volt = resolution_voltage(16400);
				ret = bq24725a_config_charger(charge_curr, charge_volt);
				if (ret < 0) {
					pr_err("failed in configuring charge\n");
					dc_on = false;
				} else {
					ret = bq24725a_enable_charging();
					if (ret < 0) {
						pr_err("failed in enable charging\n");
						dc_on = false;
					}
				}
			} else {
				pr_debug("temperature %d\n", temp);
				pr_debug("rsoc %d\n", rsoc);
				pr_debug("battery_status 0x%04X\n", battery_status);
				ret = bq24725a_disable_charging();
				if (ret < 0)
					pr_err("failed in disable charging\n");
			}
		}

		queue_delayed_work(dev->bq30z55_wq,
			&dev->periodic_update_work, bq30z55_dev->ac_periodic * HZ);

		if (check_num == 0) {
			/* Notify user space via kobject_uevent change notification */
			power_supply_changed(&dev->ac_psy);
		}
		check_num ++;
		if (check_num >= 6)
			check_num = 0;
	} else {
		dc_on = false;
		check_num = 0;
		queue_delayed_work(dev->bq30z55_wq,
			&dev->periodic_update_work, bq30z55_dev->bat_periodic * HZ);

		/* Notify user space via kobject_uevent change notification */
		power_supply_changed(&dev->ac_psy);
	}

	if (wake_lock_active(&bq30z55_wakelock))
		wake_unlock(&bq30z55_wakelock);

	if (enable_bat_trace_log) {
		volt = bq30z55_read_reg(dev->client, SBS_VOLTAGE);
		curr = bq30z55_read_reg(dev->client, SBS_CURRENT);
		rsoc = bq30z55_read_reg(dev->client, SBS_RSOC);
		RC = bq30z55_read_reg(dev->client, SBS_REMAIN_CAPACITY);
		FCC = bq30z55_read_reg(dev->client, SBS_FULL_CAPACITY);
		charge_volt = bq30z55_read_reg(dev->client, SBS_CHG_VOLTAGE);
		charge_curr = bq30z55_read_reg(dev->client, SBS_CHG_CURRENT);
		cycle_count = bq30z55_read_reg(dev->client, SBS_CYCLE_COUNT);

		if (rtc == NULL) {
			pr_err("unable to open rtc device (%s)\n", CONFIG_RTC_HCTOSYS_DEVICE);
		} else {
			err = rtc_read_time(rtc, &tm);
			if (err) {
				pr_err("unable to read the hardware clock\n");
			} else {
				err = rtc_valid_tm(&tm);
				if (err) {
					pr_err("invalid date/time\n");
				} else {
					printk(KERN_INFO "bat_trace: volt=%d, curr=%d, rsoc=%d, "
						"remaining_cap=%d, FCC=%d, chg_volt=%d, "
						"chg_curr=%d, cycle_count=%d %d-%d-%d %d:%d:%d\n",
						volt, curr, rsoc, RC, FCC, charge_volt, charge_curr, cycle_count,
						tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
						tm.tm_hour, tm.tm_min, tm.tm_sec);
				}
			}
		}
	}
}

static irqreturn_t acok_isr_handler(int irq, void *devid)
{
	struct bq30z55_device *dev = devid;

	pr_info("ACOK = %d\n", gpio_get_value(ACOK) ? 1 : 0);
	cancel_delayed_work(&dev->periodic_update_work);
	if (!bq30z55_dev->stop_wq) {
		queue_delayed_work(dev->bq30z55_wq,
			&dev->periodic_update_work, 0 * HZ);
	}
	return IRQ_HANDLED;
}

static void mcu_feedback_handler(struct notifier_block *self, u8 header,
				struct payload *sub_payload)
{
	int temp = 0;

	if (header == 0x1 && sub_payload->dataLength == 14) {
		pr_debug("sub_payload->data[11] = %d\n", sub_payload->data[11]);
		temp = sub_payload->data[11] & 128;
		if (temp == 128)
			detect_dock = true;
		else
			detect_dock = false;
	}
}

static int bq30z55_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	int ret = 0, rsoc = 0;
	u16 battery_status;

	pr_info("+++\n");

	dc_on = false;
	enable_bat_trace_log = false;
	bq30z55_dev = kzalloc(sizeof(*bq30z55_dev), GFP_KERNEL);
	if (!bq30z55_dev) {
		pr_err("alloc fail.\n");
		goto err_register_psy;
	}

	bq30z55_dev->client = client;
	bq30z55_dev->stop_wq = 0;
	bq30z55_dev->ac_periodic = 10;
	bq30z55_dev->bat_periodic = 60;
	check_num = 0;
	detect_dock = false;
	i2c_set_clientdata(client, bq30z55_dev);
#ifdef FACTORY_IMAGE
	bq30z55_dev->bat_limit_enable = 1;
	bq30z55_dev->upper_limit = 62;
	bq30z55_dev->lower_limit = 60;
#else
	bq30z55_dev->bat_limit_enable = 0;
#endif

	rsoc = bq30z55_read_reg(client, SBS_RSOC);
	battery_status = bq30z55_read_reg(client, SBS_BATTERY_STATUS);
	pr_info("rsoc %d\n", rsoc);
	pr_info("battery_status 0x%04X\n", battery_status);
	if ((battery_status & BAT_STATUS_SBS_ERROR) != 0x0 && rsoc < 0) {
		bq30z55_dev->ship_mode = true;
		fake_battery = 5;
	} else
		bq30z55_dev->ship_mode = false;

	pr_info("ship mode %d\n", bq30z55_dev->ship_mode);

	if (enable_ship && rsoc >= 60 && rsoc <= 80 &&
		gpio_get_value(ACOK) && !bq30z55_dev->ship_mode) {
		bq30z55_dev->stop_wq = 1;
		ret = bq30z55_register_ac_psy(bq30z55_dev);
		if (ret)
			pr_err("bq30z55_register_ac_psy fail.\n");

		enable_battery_ship_mode(true);
		dc_on = true;
		power_supply_changed(&bq30z55_dev->ac_psy);
	} else {
		ret = bq30z55_register_bat_psy(bq30z55_dev);
		if (ret) {
			pr_err("bq30z55_register_bat_psy fail.\n");
			goto err_register_psy;
		}
		ret = bq30z55_register_ac_psy(bq30z55_dev);
		if (ret) {
			pr_err("bq30z55_register_ac_psy fail.\n");
			goto err_register_psy;
		}
	}

	ret = bq30z55_create_debugfs_entries(bq30z55_dev);
	if (ret) {
		pr_err("bq30z55_create_debugfs_entries fail.\n");
		goto err_debugfs;
	}

	/* Register sysfs */
	ret = sysfs_create_group(&client->dev.kobj, &battery_group);
	if (ret)
		pr_err("unable to create the sysfs\n");

	bq30z55_dev->bq30z55_wq = create_singlethread_workqueue("bq30z55_wq");
	INIT_DELAYED_WORK(&bq30z55_dev->periodic_update_work,
		bq30z55_periodic_update_worker);

	ret = gpio_request(ACOK, "acok");
	if (ret)
		pr_info("gpio %d request failed\n", ACOK);

	client->irq = gpio_to_irq(ACOK);
	if (client->irq) {
		ret = request_irq(client->irq, acok_isr_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
				| IRQF_SHARED, "acok_irq", bq30z55_dev);
		if (ret) {
			dev_err(&client->dev,
				"Unable to register IRQ %d err %d\n",
				client->irq, ret);
			goto err_debugfs;
		}
	}

	alarm_init(&bq30z55_alarm_timer, ALARM_BOOTTIME, NULL);
	wake_lock_init(&bq30z55_wakelock, WAKE_LOCK_SUSPEND, "bq30z55_wakelock");

	if (cos_mode) {
		bq30z55_dev->nb.notifier_call = mcu_feedback_handler;
		mcu_feedback_register_client(&bq30z55_dev->nb);
	}

	queue_delayed_work(bq30z55_dev->bq30z55_wq,
		&bq30z55_dev->periodic_update_work, 0 * HZ);

	pr_info("---\n");
	return 0;

err_debugfs:
	debugfs_remove_recursive(bq30z55_dev->dent);
	power_supply_unregister(&bq30z55_dev->batt_psy);
	power_supply_unregister(&bq30z55_dev->ac_psy);
	destroy_workqueue(bq30z55_dev->bq30z55_wq);
	free_irq(client->irq, bq30z55_dev);
err_register_psy:
	kfree(bq30z55_dev);

	return ret;
}

static void bq30z55_shutdown(struct i2c_client *client)
{
	int ret, tmp;

	pr_info("+++\n");
	bq30z55_dev->stop_wq = 1;
	ret = cancel_delayed_work_sync(&bq30z55_dev->periodic_update_work);
	flush_workqueue(bq30z55_dev->bq30z55_wq);
	destroy_workqueue(bq30z55_dev->bq30z55_wq);
	debugfs_remove_recursive(bq30z55_dev->dent);
	free_irq(client->irq, bq30z55_dev);
	if (gpio_get_value(ACOK)) {
		tmp = bq24725a_disable_charging();
		if (tmp < 0)
			pr_err("failed in disable charging\n");
	}
	kfree(bq30z55_dev);
	pr_info("%d ---\n", ret);
}

static int bq30z55_prepare(struct device *dev)
{
	int ret;
	u16 battery_status;

	pr_info("+++\n");

	battery_status = bq30z55_read_reg(bq30z55_dev->client, SBS_BATTERY_STATUS);

	if (gpio_get_value(ACOK)) {
		get_monotonic_boottime(&ts);
		if (!(battery_status & BAT_STATUS_FULL)) {
			ts.tv_sec += 150;
		} else {
			ts.tv_sec += 28800;
		}
		ret = alarm_start(&bq30z55_alarm_timer, timespec_to_ktime(ts));
		if (ret)
			pr_info("set alarm fail %d\n", ret);
	}

	flush_workqueue(bq30z55_dev->bq30z55_wq);

	pr_info("---\n");
	return 0;
}

static int bq30z55_suspend(struct device *dev)
{
	int ret, hw_id;
	u16 battery_status;

	pr_info("+++\n");

	battery_status = bq30z55_read_reg(bq30z55_dev->client, SBS_BATTERY_STATUS);
	hw_id = asustek_get_hw_rev();

	if (cos_mode) {
		if (hw_id >= HW_ID_PR) {
			if (!detect_dock && gpio_get_value(ACOK)) {
				pr_info("Turn off driver board power\n");
				ret = gpio_direction_output(DRIVER_POWER, 0);
				if (ret)
					pr_info("gpio %d unavaliable for output\n", DRIVER_POWER);
			}
			ret = gpio_direction_output(CAP_POWER, 0);
			if (ret)
				pr_info("gpio %d unavaliable for output\n", CAP_POWER);

			ret = gpio_direction_output(CIR_POWER, 0);
			if (ret)
				pr_info("gpio %d unavaliable for output\n", CIR_POWER);
		}

		if (gpio_get_value(ACOK)) {
			if ((battery_status & BAT_STATUS_FULL)) {
				pr_info("Turn off LED power\n");

				if (hw_id >= HW_ID_PR)
					ret = gpio_direction_output(LED_POWER, 1);
				else
					ret = gpio_direction_output(LED_POWER, 0);

				if (ret)
					pr_info("gpio %d unavaliable for output\n", LED_POWER);
			}
		}
	}

	pr_info("---\n");
	return 0;
}

static int bq30z55_resume(struct device *dev)
{
	int ret, tmp, hw_id;

	pr_info("+++\n");

	hw_id = asustek_get_hw_rev();
	ret = alarm_try_to_cancel(&bq30z55_alarm_timer);

	if (gpio_get_value(ACOK)) {
		wake_lock(&bq30z55_wakelock);
		check_num = 0;
	}

	if (cos_mode) {
		if (!gpio_get_value(ACOK) && !gpio_get_value(DRIVER_POWER))
			gpio_direction_output(DRIVER_POWER, 1);

		if (hw_id >= HW_ID_PR)
			tmp = gpio_direction_output(LED_POWER, 0);
		else
			tmp = gpio_direction_output(LED_POWER, 1);

		if (tmp)
			pr_info("gpio %d unavaliable for output\n", LED_POWER);
	}

	flush_workqueue(bq30z55_dev->bq30z55_wq);

	pr_info("%d ---\n", ret);
	return 0;
}

int __init asustek_battery_ship(char *s)
{
	int n;

	if (*s == '=')
		s++;
	n = snprintf(checkmode, sizeof(checkmode), "%s", s);
	checkmode[n] = '\0';

	pr_info("checkmode %s\n", checkmode);
	if (!strcmp(checkmode, "1"))
		enable_ship = true;
	else
		enable_ship = false;

	return 1;
}
__setup("androidboot.ship_mode", asustek_battery_ship);


static const struct i2c_device_id bq30z55_id[] = {
	{ "bq3055", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq30z55_id);

#ifdef CONFIG_ACPI
static struct acpi_device_id bq30z55_acpi_match[] = {
	{"TBQ3055", 0},
	{}
};
MODULE_DEVICE_TABLE(acpi, bq30z55_acpi_match);
#endif

static const struct dev_pm_ops bq30z55_pm_ops = {
	.prepare = bq30z55_prepare,
	.suspend = bq30z55_suspend,
	.resume = bq30z55_resume,
};

static struct i2c_driver bq30z55_driver = {
	.driver	= {
#ifdef CONFIG_ACPI
		.name = "TBQ3055",
#else
		.name = "bq3055",
#endif
		.owner	= THIS_MODULE,
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(bq30z55_acpi_match),
#endif
		.pm	= &bq30z55_pm_ops,
	},
	.probe		= bq30z55_probe,
	.shutdown	= bq30z55_shutdown,
	.id_table	= bq30z55_id,
};

static int __init bq30z55_init(void)
{
	cos_mode = check_cos_mode();
	return i2c_add_driver(&bq30z55_driver);
}
module_init(bq30z55_init);

static void __exit bq30z55_exit(void)
{
	i2c_del_driver(&bq30z55_driver);
}
module_exit(bq30z55_exit);


MODULE_DESCRIPTION("bq30z55 gauge driver");
MODULE_LICENSE("GPL");
