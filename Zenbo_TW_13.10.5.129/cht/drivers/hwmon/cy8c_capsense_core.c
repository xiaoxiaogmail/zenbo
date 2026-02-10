/*
 * drivers/hwmon/cy8c_capsense_core.c - capacitive touch sensor.
 *
 * Device Driver for cypress of PSoC family CY8C4014SXI-420
 *
 * Copyright (C) ASUSTeK Computer Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/irq.h>
#include <linux/i2c/cy8c_capsense.h>
#include <linux/board_asustek.h>

#ifdef CONFIG_ACPI
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#endif
#include <linux/spinlock.h>
#include <linux/input.h>

#define CAP_VEN_GPIO 394

#define MIDPRESSCOUNT 0xC8
#define IGNORECOUNT 0x46
#define EXTREMECOUNT 0x1

#ifdef FACTORY_IMAGE
#define LONGPRESSCOUNT 0x4B
#else
#define LONGPRESSCOUNT 0x96
#endif

/*
 * Debug Utility
 */
#define CAP_DEBUG(fmt, arg...)  \
	pr_debug("CY8C4014SXI: [%s] " fmt , __func__ , ##arg)
#define CAP_INFO(fmt, arg...)   \
	pr_info("CY8C4014SXI: [%s] " fmt , __func__ , ##arg)
#define CAP_ERROR(fmt, arg...)  \
	pr_err("CY8C4014SXI: [%s] " fmt , __func__ , ##arg)

/*
 * Global Variable
 */
struct cy8c_capsense_dev {
	struct i2c_client *client;
	struct attribute_group attrs;
	struct workqueue_struct *cap_wq;
	struct delayed_work work;
	int enable;
	unsigned char num_reg;
	struct input_dev *input;
	spinlock_t irq_enabled_lock;
};

static DEFINE_MUTEX(cap_mutex);
static int reg_init;
static int key_status;
static int fw_flag;
static int sensor_mode;

/*
 * Function Declaration
 */
static int cy8c_capsense_probe(struct i2c_client *client,
			       const struct i2c_device_id *id);
static void cy8c_capsense_shutdown(struct i2c_client *client);
static int cy8c_capsense_init(void);
static void cy8c_capsense_exit(void);
static irqreturn_t cy8c_capsense_interrupt_handler(int irq, void *dev);
static int cy8c_capsense_config_irq(struct i2c_client *client);
static int cy8c_capsense_i2c_read_base(struct i2c_client *client, u8 reg,
				       u8 *rdata);
static int cy8c_capsense_i2c_write_base(struct i2c_client *client,
					u8 reg, u8 wdata);
static ssize_t show_attrs_handler(struct device *dev,
				  struct device_attribute *devattr, char *buf);
static ssize_t store_attrs_handler(struct device *dev,
				   struct device_attribute *devattr,
				   const char *buf, size_t count);
static ssize_t reg_dump_show(struct device *dev,
			     struct device_attribute *devattr, char *buf);
static ssize_t debug_value_show(struct device *dev,
				struct device_attribute *devattr, char *buf);
static void cy8c_capsense_work_function(struct work_struct *work);
static int cy8c_capsense_init_sensor(struct i2c_client *client);

/*
 * I2C and ACPI Driver Structure
 */
static struct i2c_device_id cy8c_capsense_id[] = {
	{CY8C4014SXI_DRIVER_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, cy8c_capsense_id);

#ifdef CONFIG_ACPI
static struct acpi_device_id cy8c_capsense_acpi_match[] = {
	{"CYPS4420", 0},
	{"", 0},
};

MODULE_DEVICE_TABLE(acpi, cy8c_capsense_acpi_match);
#endif

static struct i2c_driver cy8c_capsense_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = cy8c_capsense_probe,
	.shutdown = cy8c_capsense_shutdown,
	.id_table = cy8c_capsense_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = CY8C4014SXI_DRIVER_NAME,
#ifdef CONFIG_ACPI
		   .acpi_match_table = ACPI_PTR(cy8c_capsense_acpi_match),
#endif
		   },
};

static DEVICE_ATTR(key_status, S_IRUGO, show_attrs_handler, NULL);
static DEVICE_ATTR(fw_version, S_IRUGO, show_attrs_handler, NULL);
static DEVICE_ATTR(enter_bootloader, S_IRUGO | S_IWUSR,
		show_attrs_handler, store_attrs_handler);
static DEVICE_ATTR(extreme_time, S_IRUGO | S_IWUSR,
		   show_attrs_handler, store_attrs_handler);
static DEVICE_ATTR(ignore_time, S_IRUGO | S_IWUSR,
		   show_attrs_handler, store_attrs_handler);
static DEVICE_ATTR(mid_press_time, S_IRUGO | S_IWUSR,
		   show_attrs_handler, store_attrs_handler);
static DEVICE_ATTR(press_count, S_IRUGO | S_IWUSR,
		   show_attrs_handler, store_attrs_handler);
static DEVICE_ATTR(interrupt_mode, S_IRUGO | S_IWUSR,
		   show_attrs_handler, store_attrs_handler);
static DEVICE_ATTR(reg_dump, S_IRUGO, reg_dump_show, NULL);
static DEVICE_ATTR(debug_value, S_IRUGO, debug_value_show, NULL);
static DEVICE_ATTR(reg_init, S_IRUGO | S_IWUSR, show_attrs_handler,
		   store_attrs_handler);
static DEVICE_ATTR(key_status_btn, S_IRUGO, show_attrs_handler, NULL);
static DEVICE_ATTR(debug_mode, S_IRUGO | S_IWUSR,
		   show_attrs_handler, store_attrs_handler);
static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR,
		   show_attrs_handler, store_attrs_handler);
static DEVICE_ATTR(power_en, S_IRUGO | S_IWUSR,
		   show_attrs_handler, store_attrs_handler);

static struct attribute *cy8c_capsense_attrs[] = {
	&dev_attr_key_status.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_enter_bootloader.attr,
	&dev_attr_press_count.attr,
	&dev_attr_interrupt_mode.attr,
	&dev_attr_reg_dump.attr,
	&dev_attr_reg_init.attr,
	&dev_attr_key_status_btn.attr,
	&dev_attr_debug_mode.attr,
	&dev_attr_debug_value.attr,
	&dev_attr_ignore_time.attr,
	&dev_attr_mid_press_time.attr,
	&dev_attr_extreme_time.attr,
	&dev_attr_mode.attr,
	&dev_attr_power_en.attr,
	NULL
};

static ssize_t show_attrs_handler(struct device *dev,
				  struct device_attribute *devattr, char *buf)
{
	int ret = -1;
	u8 rdata;
	int error;
	int hw_id = 0;

	hw_id = asustek_get_hw_rev();

	struct i2c_client *client = to_i2c_client(dev);
	struct cy8c_capsense_dev *cy8c_capsense_dev =
	    i2c_get_clientdata(client);
	const char *devattr_name = devattr->attr.name;

	CAP_DEBUG("devattr->attr->name: %s\n", devattr->attr.name);

	mutex_lock(&cap_mutex);
	if (cy8c_capsense_dev->enable) {
		if (!strcmp(devattr_name,
				dev_attr_enter_bootloader.attr.name)) {
			error =
				cy8c_capsense_i2c_read_base(client,
						REG_ENTERBOOTLOADER,
						&rdata);
			if (!error)
				ret = sprintf(buf, "%#2x\n", rdata);
		} else if (!strcmp(devattr_name, dev_attr_key_status.attr.name)) {
			error =
			    cy8c_capsense_i2c_read_base(client, REG_KEYSTATUS,
							&rdata);
			if (!error)
				ret = sprintf(buf, "%#2x\n", rdata);
		} else if (!strcmp(devattr_name, dev_attr_fw_version.attr.name)) {
			error =
			    cy8c_capsense_i2c_read_base(client,
							REG_FW_REVERSION,
							&rdata);
			if (!error)
				ret = sprintf(buf, "%d\n", rdata);
		} else
		    if (!strcmp(devattr_name, dev_attr_press_count.attr.name)) {
			error =
			    cy8c_capsense_i2c_read_base(client, REG_PRESSCOUNT,
							&rdata);
			if (!error)
				ret = sprintf(buf, "%#2x\n", rdata);
		} else
		    if (!strcmp
			(devattr_name, dev_attr_interrupt_mode.attr.name)) {
			error =
			    cy8c_capsense_i2c_read_base(client,
							REG_INTERRUPT_MODE,
							&rdata);
			if (!error)
				ret = sprintf(buf, "%#2x\n", rdata);
		} else if (!strcmp(devattr_name, dev_attr_reg_init.attr.name)) {
			ret = sprintf(buf, "%#2x\n", reg_init);
		} else
		    if (!strcmp
			(devattr_name, dev_attr_key_status_btn.attr.name)) {
			ret = sprintf(buf, "%d\n", key_status);
			key_status = 0;
		} else if (!strcmp(devattr_name, dev_attr_debug_mode.attr.name)) {
			error = cy8c_capsense_i2c_read_base(client,
							    REG_DEBUG_MODE,
							    &rdata);
			if (!error)
				ret = sprintf(buf, "%#2x\n", rdata);
		} else
		    if (!strcmp(devattr_name, dev_attr_ignore_time.attr.name)) {
			error =
			    cy8c_capsense_i2c_read_base(client, REG_IGNORE_TIME,
							&rdata);
			if (!error)
				ret = sprintf(buf, "%#2x\n", rdata);
		} else
		    if (!strcmp
			(devattr_name, dev_attr_mid_press_time.attr.name)) {
			error =
			    cy8c_capsense_i2c_read_base(client,
							REG_MID_PRESS_TIME,
							&rdata);
			if (!error)
				ret = sprintf(buf, "%#2x\n", rdata);
		} else
		    if (!strcmp(devattr_name, dev_attr_extreme_time.attr.name)) {
			error =
			    cy8c_capsense_i2c_read_base(client, REG_EXTREME_TIME,
					&rdata);
			if (!error)
				ret = sprintf(buf, "%#2x\n", rdata);
		} else if (!strcmp(devattr_name, dev_attr_mode.attr.name)) {
			ret = sprintf(buf, "%d\n", sensor_mode);
		} else if (!strcmp(devattr_name, dev_attr_power_en.attr.name)) {
			if (hw_id != 5) {
				ret = sprintf(buf, "%d\n",
						gpio_get_value(CAP_VEN_GPIO));
			} else
				ret = sprintf(buf, "%s\n",
						"could not support power_en");
		}
	} else {
		ret = sprintf(buf, "SENSOR DISABLED\n");
	}
	mutex_unlock(&cap_mutex);

	return ret;
}

static ssize_t store_attrs_handler(struct device *dev,
				   struct device_attribute *devattr,
				   const char *buf, size_t count)
{
	unsigned long wdata;
	struct i2c_client *client = to_i2c_client(dev);
	struct cy8c_capsense_dev *cy8c_capsense_dev =
	    i2c_get_clientdata(client);
	const char *devattr_name = devattr->attr.name;
	int ret;
	int error;
	int hw_id = 0;

	hw_id = asustek_get_hw_rev();

	error = kstrtoul(buf, 16, &wdata);
	if (error)
		return error;

	CAP_DEBUG("devattr->attr->name: %s, wdata: %lu\n",
		  devattr->attr.name, wdata);

	mutex_lock(&cap_mutex);
	if (cy8c_capsense_dev->enable) {
		if (!strcmp(devattr_name, dev_attr_enter_bootloader.attr.name)) {
			ret = i2c_smbus_write_byte_data(client,
							REG_ENTERBOOTLOADER,
							(wdata & 0x01));
			if (ret)
				CAP_ERROR
				    ("failed to write the REG_ENTERBOOTLOADER\n");
		} else
		    if (!strcmp(devattr_name, dev_attr_press_count.attr.name)) {
			ret =
			    i2c_smbus_write_byte_data(client, REG_PRESSCOUNT,
						      (wdata & 0xFF));
			if (ret)
				CAP_ERROR
				    ("failed to write the REG_PRESSCOUNT\n");
		} else
		    if (!strcmp
			(devattr_name, dev_attr_interrupt_mode.attr.name)) {
			ret =
			    i2c_smbus_write_byte_data(client,
						      REG_INTERRUPT_MODE,
						      (wdata & 0xFF));
			if (ret)
				CAP_ERROR
				    ("failed to write the REG_INTERRUPT_MODE\n");
		} else if (!strcmp(devattr_name, dev_attr_reg_init.attr.name)) {
			reg_init = (wdata & 0xFF);
			cy8c_capsense_init_sensor(client);
		} else if (!strcmp(devattr_name, dev_attr_debug_mode.attr.name)) {
			ret =
			    i2c_smbus_write_byte_data(client,
						      REG_DEBUG_MODE,
						      (wdata & 0xFF));
			if (ret)
				CAP_ERROR
				    ("failed to write the REG_BTN_RAWCOUNT_LB\n");
		} else
		    if (!strcmp(devattr_name, dev_attr_ignore_time.attr.name)) {
			ret =
			    i2c_smbus_write_byte_data(client, REG_IGNORE_TIME,
						      (wdata & 0xFF));
			if (ret)
				CAP_ERROR
				    ("failed to write the REG_IGNORE_TIME\n");
		} else
		    if (!strcmp
			(devattr_name, dev_attr_mid_press_time.attr.name)) {
			ret =
			    i2c_smbus_write_byte_data(client,
						      REG_MID_PRESS_TIME,
						      (wdata & 0xFF));
			if (ret)
				CAP_ERROR
				    ("failed to write the REG_MID_PRESS_TIME\n");
		} else
		    if (!strcmp(devattr_name,
				dev_attr_extreme_time.attr.name)) {
			ret =
			    i2c_smbus_write_byte_data(client, REG_EXTREME_TIME,
						      (wdata & 0xFF));
			if (ret)
				CAP_ERROR
				    ("failed to write the REG_EXTREME_TIME\n");
		} else if (!strcmp(devattr_name, dev_attr_mode.attr.name)) {
			sensor_mode = wdata & 0x01;
		} else if (!strcmp(devattr_name,
				dev_attr_power_en.attr.name)) {
			if (hw_id != 5) {
				ret = gpio_direction_output(CAP_VEN_GPIO,
						(wdata & 0x01));
				if (ret)
					CAP_ERROR
					    ("gpio %d unavaliable for output\n",
							CAP_VEN_GPIO);
			}
		}
	}
	mutex_unlock(&cap_mutex);

	return strnlen(buf, count);;
}

static ssize_t debug_value_show(struct device *dev,
				struct device_attribute *devattr, char *buf)
{
	ssize_t bytes_printed = 0;
	struct i2c_client *client = to_i2c_client(dev);

	u8 debug_value[6] = {0};
	cy8c_capsense_i2c_read_base(client, REG_BTN1_HB, &debug_value[0]);
	cy8c_capsense_i2c_read_base(client, REG_BTN1_LB, &debug_value[1]);
	cy8c_capsense_i2c_read_base(client, REG_BTN2_HB, &debug_value[2]);
	cy8c_capsense_i2c_read_base(client, REG_BTN2_LB, &debug_value[3]);
	cy8c_capsense_i2c_read_base(client, REG_BTN3_HB, &debug_value[4]);
	cy8c_capsense_i2c_read_base(client, REG_BTN3_LB, &debug_value[5]);
	bytes_printed = sprintf(buf, "%d, %d, %d\n", (debug_value[1] +
						      (debug_value[0] << 8)),
				(debug_value[3] + (debug_value[2] << 8)),
				(debug_value[5] + (debug_value[4] << 8)));
	return bytes_printed;
}

static ssize_t reg_dump_show(struct device *dev,
			     struct device_attribute *devattr, char *buf)
{
	int dump_count;
	ssize_t bytes_printed = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct cy8c_capsense_dev *cy8c_capsense_dev =
	    i2c_get_clientdata(client);
	int ret;
	u8 data[cy8c_capsense_dev->num_reg];
	memset(data, 0, sizeof(data));

	ret = i2c_smbus_read_i2c_block_data(client, 0, cy8c_capsense_dev->num_reg, data);
	if (ret == cy8c_capsense_dev->num_reg) {
		for (dump_count = 0; dump_count < cy8c_capsense_dev->num_reg; dump_count++)
			bytes_printed += sprintf(buf + bytes_printed, "%#2x: %#2x\n", dump_count, data[dump_count]);
	}
	return bytes_printed;
}

static int cy8c_capsense_i2c_read_base(struct i2c_client *client,
				       u8 reg, u8 *rdata)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		CAP_ERROR("i2c read fail: can't read from %02x: %d\n", reg,
			  ret);
		return -1;
	} else {
		*rdata = (u8)ret;
	}
	return 0;
}

static int cy8c_capsense_i2c_write_base(struct i2c_client *client,
					u8 reg, u8 wdata)
{
	int ret;
	int time = 0;

	while (time < 3) {
		time++;
		ret = i2c_smbus_write_byte_data(client, reg, wdata);
		if (ret)
			CAP_ERROR
			    ("cy8c_capsense_i2c_write_base error: reg:%u ret: %d time: %d\n",
			     reg, ret, time);
		else
			return 0;
	}
	return -1;
}

static irqreturn_t cy8c_capsense_interrupt_handler(int irq, void *dev)
{
	struct cy8c_capsense_dev *cy8c_capsense_dev = i2c_get_clientdata(dev);
	unsigned long flags;
	spin_lock_irqsave(&cy8c_capsense_dev->irq_enabled_lock, flags);
	disable_irq_nosync(cy8c_capsense_dev->client->irq);
	CAP_INFO("=== cap touch sensor irq ===");
	queue_delayed_work(cy8c_capsense_dev->cap_wq, &cy8c_capsense_dev->work,
			   0);
	enable_irq(cy8c_capsense_dev->client->irq);
	spin_unlock_irqrestore(&cy8c_capsense_dev->irq_enabled_lock, flags);
	return IRQ_HANDLED;
}

static int cy8c_capsense_config_irq(struct i2c_client *client)
{
	int ret = 0;

	/* Get interrupt GPIO pin number */
	ret = gpio_request(GPIO_IRQ, GPIO_IRQ_NAME);
	if (ret) {
		CAP_ERROR("Failed to request GPIO %d, error %d\n",
			  GPIO_IRQ, ret);
		goto err_gpio_request_fail;
	}

	ret = gpio_direction_input(GPIO_IRQ);
	if (ret) {
		CAP_ERROR
		    ("Failed to configure input direction for GPIO %d, error %d\n",
		     GPIO_IRQ, ret);
		goto err_gpio_direction_input_fail;
	}

	client->irq = gpio_to_irq(GPIO_IRQ);
	ret = request_irq(client->irq, cy8c_capsense_interrupt_handler,
			  IRQF_TRIGGER_FALLING, GPIO_IRQ_NAME, client);
	if (ret) {
		CAP_ERROR("IRQ %d busy? error %d\n", client->irq, ret);
		goto err_request_irq_fail;
	}

	CAP_INFO("IRQ request success, GPI = %3d, VALUE=%d\n",
		 GPIO_IRQ, gpio_get_value(GPIO_IRQ));

	return 0;

err_request_irq_fail:
err_gpio_direction_input_fail:
	if (gpio_is_valid(GPIO_IRQ))
		gpio_free(GPIO_IRQ);
err_gpio_request_fail:
	return ret;
}

static int cy8c_capsense_init_sensor(struct i2c_client *client)
{
	int ret;
	int reg_count;
	u8 rdata;
	struct cy8c_capsense_dev *cy8c_capsense_dev =
	    i2c_get_clientdata(client);

	ret =
	    cy8c_capsense_i2c_read_base(cy8c_capsense_dev->client,
					REG_FW_REVERSION, &rdata);

	if (!ret) {
		if (rdata < 0x14)
			fw_flag = 0;
		else
			fw_flag = 1;
	} else {
		CAP_ERROR("%s : %s\n", __func__, "Read firmware version fail");
		return ret;
	}

	for (reg_count = 0; reg_count < cy8c_capsense_dev->num_reg; reg_count++) {
		switch (reg_count) {
		case REG_PRESSCOUNT:
			ret =
			    cy8c_capsense_i2c_write_base(client, REG_PRESSCOUNT,
							 LONGPRESSCOUNT);
			if (ret)
				CAP_ERROR
				    ("Sensor init REG_PRESSCOUNT failed\n");
			break;
		case REG_IGNORE_TIME:
			ret = cy8c_capsense_i2c_write_base(client, REG_IGNORE_TIME,
					IGNORECOUNT);
			if (ret)
				CAP_ERROR("Sensor init REG_IGNORE_TIME failed\n");
			break;
		case REG_MID_PRESS_TIME:
			ret = cy8c_capsense_i2c_write_base(client, REG_MID_PRESS_TIME,
					MIDPRESSCOUNT);
			if (ret)
				CAP_ERROR("Sensor init REG_MID_PRESS_TIME failed\n");
			break;
		case REG_EXTREME_TIME:
			ret = cy8c_capsense_i2c_write_base(client, REG_EXTREME_TIME,
					EXTREMECOUNT);
			if (ret)
				CAP_ERROR("Sensor init REG_EXTREME_TIME failed\n");
			break;
		case REG_DEBUG_MODE:
			ret = cy8c_capsense_i2c_write_base(client,
							   REG_DEBUG_MODE,
							   0x3);
			if (ret)
				CAP_ERROR
				    ("Sensor init REG_BTN_RAWCOUNT_LB failed\n");
			break;
		}
	}

	CAP_INFO("I2C_NAME: %s, I2C_ADDR: 0x%X %s\n",
		 client->name, client->addr, ret ? "FAIL" : "OK");
	sensor_mode = 0;
	return ret;
}

static void cy8c_capsense_work_function(struct work_struct *work)
{
	struct cy8c_capsense_dev *cy8c_capsense_dev =
	    container_of((struct delayed_work *)work, struct cy8c_capsense_dev,
			 work);

	u8 press_count[2];
	int ret;
	int key_status_value = 0;
	int press_time;
	u8 data[cy8c_capsense_dev->num_reg];
	mutex_lock(&cap_mutex);
	memset(data, 0, sizeof(data));
	ret = i2c_smbus_read_i2c_block_data(cy8c_capsense_dev->client, 0, cy8c_capsense_dev->num_reg, data);
	mutex_unlock(&cap_mutex);
	if (sensor_mode) {
		if (ret != cy8c_capsense_dev->num_reg) {
			CAP_ERROR("%s : %s ret is %d\n", __func__, "Read register fail", ret);
		} else {
			memset(press_count, 0, sizeof(press_count));
			CAP_INFO("=== %s : %#2x ===\n", __func__, data[REG_KEYSTATUS]);
			key_status_value = data[REG_KEYSTATUS];

			if (key_status_value == 4) {
				key_status = data[REG_SENSOR_CP] & 0xFF;
				CAP_INFO("=== key_status is %d ===\n", key_status);
			} else if (key_status_value == 1 || key_status_value == 2 || key_status_value == 3)
				CAP_INFO("=== Very short touch or short touch or mid touch ignore ===\n");

			input_event(cy8c_capsense_dev->input, EV_MSC, MSC_CAP, data[REG_KEYSTATUS]);
			if (fw_flag == 1) {
				press_count[0] = data[REG_PRESS_TIME_HB];
				press_count[1] = data[REG_PRESS_TIME_LB];
				press_time = ((press_count[0] << 8) + press_count[1]) * 5;
				CAP_INFO("=== %s : Cap press time = %d ===\n", __func__, press_time);
				input_event(cy8c_capsense_dev->input, EV_MSC, MSC_RAW, press_time);
				input_sync(cy8c_capsense_dev->input);
			} else {
				input_sync(cy8c_capsense_dev->input);
			}
		}
	} else
		CAP_INFO("=== %s Skip cap sensor event===\n", __func__);
}

static int cy8c_capsense_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	int ret;
	int hw_id = 0;

	hw_id = asustek_get_hw_rev();

	struct cy8c_capsense_dev *cy8c_capsense_dev;
	struct input_dev *dev;

	CAP_INFO("Start to probe!\n");

	reg_init = 0;
	key_status = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		return -ENODEV;
	}

	cy8c_capsense_dev = kzalloc(sizeof(*cy8c_capsense_dev), GFP_KERNEL);
	if (!cy8c_capsense_dev) {
		CAP_ERROR("kzalloc failed!\n");
		ret = -ENOMEM;
		goto err_kzalloc_failed;
	}

	if (hw_id != 5) {
		CAP_INFO("cap sensor hardware reset\n");
		ret = gpio_request(CAP_VEN_GPIO, "CAP_VEN");
		if (ret) {
			CAP_ERROR("gpio %d request failed\n", CAP_VEN_GPIO);
			goto err_create_singlethread_workqueue_failed;
		}

		ret = gpio_direction_output(CAP_VEN_GPIO, 0);
		if (ret) {
			CAP_ERROR("gpio %d unavaliable for output\n",
					CAP_VEN_GPIO);
			goto err_free_gpio;
		}
		msleep(100);
		ret = gpio_direction_input(CAP_VEN_GPIO);
		if (ret) {
			CAP_ERROR("gpio %d unavaliable for input\n",
					CAP_VEN_GPIO);
			goto err_free_gpio;
		}
		msleep(100);
	}

	cy8c_capsense_dev->cap_wq = create_singlethread_workqueue("cap_wq");
	if (!cy8c_capsense_dev->cap_wq) {
		CAP_ERROR("create_singlethread_workqueue failed!\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}
	INIT_DELAYED_WORK(&cy8c_capsense_dev->work,
			  cy8c_capsense_work_function);
	spin_lock_init(&cy8c_capsense_dev->irq_enabled_lock);
	/* Set cy8c_capsense_dev relative members */
	cy8c_capsense_dev->client = client;
	i2c_set_clientdata(client, cy8c_capsense_dev);
	cy8c_capsense_dev->client->flags = 0;
	cy8c_capsense_dev->num_reg = CY8C4014SXI_REG_MAX;
	cy8c_capsense_dev->enable = 0;

	dev = input_allocate_device();

	if (!dev) {
		ret = -1;
		goto err_init_sensor_failed;
	}

	dev->name = "captouch";
	dev->id.bustype = BUS_I2C;
	input_set_capability(dev, EV_MSC, MSC_CAP);
	input_set_capability(dev, EV_MSC, MSC_RAW);
	ret = input_register_device(dev);

	if (ret < 0) {
		input_free_device(dev);
		goto err_init_sensor_failed;
	}

	cy8c_capsense_dev->input = dev;

	/* Initialize cap sensor */
	ret = cy8c_capsense_init_sensor(cy8c_capsense_dev->client);
	if (ret) {
		CAP_ERROR("Sensor initialization failed!\n");
		/*goto err_init_sensor_failed; */
	}

	/* Create default capsense device attributes */
	cy8c_capsense_dev->attrs.attrs = cy8c_capsense_attrs;
	ret =
	    sysfs_create_group(&cy8c_capsense_dev->client->dev.kobj,
			       &cy8c_capsense_dev->attrs);
	if (ret) {
		CAP_ERROR("Create the sysfs group failed!\n");
		goto err_create_sysfs_group_failed;
	}

	/* Request GPIO IRQ */
	ret = cy8c_capsense_config_irq(cy8c_capsense_dev->client);
	if (ret) {
		CAP_ERROR("Sensor INT configuration failed!\n");
		goto err_config_irq_failed;
	}

	cy8c_capsense_dev->enable = 1;

	return 0;

err_config_irq_failed:
	sysfs_remove_group(&cy8c_capsense_dev->client->dev.kobj,
			   &cy8c_capsense_dev->attrs);
err_create_sysfs_group_failed:
err_init_sensor_failed:
	destroy_workqueue(cy8c_capsense_dev->cap_wq);
err_free_gpio:
	if (gpio_is_valid(CAP_VEN_GPIO))
		gpio_free(CAP_VEN_GPIO);
err_create_singlethread_workqueue_failed:
	kfree(cy8c_capsense_dev);
err_kzalloc_failed:
	return ret;
}

static void cy8c_capsense_shutdown(struct i2c_client *client)
{
	struct cy8c_capsense_dev *cy8c_capsense_dev =
	    i2c_get_clientdata(client);
	int ret;
	int hw_id = 0;

	hw_id = asustek_get_hw_rev();

	sysfs_remove_group(&client->dev.kobj, &cy8c_capsense_dev->attrs);
	free_irq(client->irq, client);
	if (cy8c_capsense_dev->cap_wq)
		destroy_workqueue(cy8c_capsense_dev->cap_wq);
	input_unregister_device(cy8c_capsense_dev->input);
	kfree(cy8c_capsense_dev);

	if (hw_id != 5) {
		CAP_INFO("keep CAP_VEN_GPIO low\n");
		ret = gpio_direction_output(CAP_VEN_GPIO, 0);
		if (ret)
			CAP_ERROR("gpio %d unavaliable for output\n",
					CAP_VEN_GPIO);
	}
}

static int __init cy8c_capsense_init(void)
{
	return i2c_add_driver(&cy8c_capsense_driver);
}

static void __exit cy8c_capsense_exit(void)
{
	i2c_del_driver(&cy8c_capsense_driver);
}

module_init(cy8c_capsense_init);
module_exit(cy8c_capsense_exit);

MODULE_AUTHOR("ASUSTeK Computer Inc.");
MODULE_DESCRIPTION("Cypress capacitive  driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS(CY8C4014SXI_DRIVER_NAME);
