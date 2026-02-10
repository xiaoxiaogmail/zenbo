/*
 * drivers/hwmon/drop_laser.c - drop laser sensor.
 *
 * Device Driver for drop laser
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/usb.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
MODULE_LICENSE("Dual BSD/GPL");

extern int mcu_feedback_register_client(struct notifier_block *nb);

struct drop_laser_dev {
	struct input_dev *dev;
	struct workqueue_struct *drop_laser_wq;
	struct delayed_work work;
	struct notifier_block nb;
	struct attribute_group attrs;
	struct kobject *drop_laser_control_kobj;
	u16 value_fifth[2];
	u16 value_fourth[2];
	u16 value_third[2];
	u16 value_second[2];
	u16 value_first[2];
	s32 timestamp;
};

static u16 value_fifth_sys[2];
static u16 value_fourth_sys[2];
static u16 value_third_sys[2];
static u16 value_second_sys[2];
static u16 value_first_sys[2];
static s32 timestamp_sys;

static u16 value_fifth_before[2];
static u16 value_fourth_before[2];
static u16 value_third_before[2];
static u16 value_second_before[2];
static u16 value_first_before[2];
static int sensor_mode;
static int set_mode_first_time;

static ssize_t timestamp_info_show(struct device *dev,
				   struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", timestamp_sys);
}

static ssize_t data_info_show(struct device *dev,
			      struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d\n", value_fifth_sys[0],
			value_fourth_sys[0], value_third_sys[0],
			value_second_sys[0], value_first_sys[0],
			value_fifth_sys[1], value_fourth_sys[1],
			value_third_sys[1], value_second_sys[1],
			value_first_sys[1]);
}

static ssize_t mode_show(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", sensor_mode);
}

static ssize_t mode_store(struct device *dev,
		struct device_attribute *devattr, char *buf,
		size_t count)
{
	unsigned long val;
	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 1))
		return -EINVAL;
	sensor_mode = val;
	if (sensor_mode)
		set_mode_first_time = 1;
	return count;
}

static DEVICE_ATTR(timestamp_info, S_IRUGO, timestamp_info_show, NULL);
static DEVICE_ATTR(data_info, S_IRUGO, data_info_show, NULL);
static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, mode_show, mode_store);

static struct attribute *drop_laser_attrs[] = {
	&dev_attr_timestamp_info.attr,
	&dev_attr_data_info.attr,
	&dev_attr_mode.attr,
	NULL
};

static void drop_laser_work_function(struct work_struct *work)
{
	struct drop_laser_dev *drop_laser_dev =
	    container_of((struct delayed_work *)work, struct drop_laser_dev,
			 work);
	value_fifth_sys[0] = drop_laser_dev->value_fifth[0];
	value_fourth_sys[0] = drop_laser_dev->value_fourth[0];
	value_third_sys[0] = drop_laser_dev->value_third[0];
	value_second_sys[0] = drop_laser_dev->value_second[0];
	value_first_sys[0] = drop_laser_dev->value_first[0];

	value_fifth_sys[1] = drop_laser_dev->value_fifth[1];
	value_fourth_sys[1] = drop_laser_dev->value_fourth[1];
	value_third_sys[1] = drop_laser_dev->value_third[1];
	value_second_sys[1] = drop_laser_dev->value_second[1];
	value_first_sys[1] = drop_laser_dev->value_first[1];

	if (!set_mode_first_time &&
			value_fifth_sys[0] == value_fifth_before[0] &&
			value_fourth_sys[0] == value_fourth_before[0] &&
			value_third_sys[0] == value_third_before[0] &&
			value_second_sys[0] == value_second_before[0] &&
			value_first_sys[0] == value_first_before[0] &&
			value_fifth_sys[1] == value_fifth_before[1] &&
			value_fourth_sys[1] == value_fourth_before[1] &&
			value_third_sys[1] == value_third_before[1] &&
			value_second_sys[1] == value_second_before[1] &&
			value_first_sys[1] == value_first_before[1]) {
		return;
	} else {
		value_fifth_before[0] = value_fifth_sys[0];
		value_fourth_before[0] = value_fourth_sys[0];
		value_third_before[0] = value_third_sys[0];
		value_second_before[0] = value_second_sys[0];
		value_first_before[0] = value_first_sys[0];

		value_fifth_before[1] = value_fifth_sys[1];
		value_fourth_before[1] = value_fourth_sys[1];
		value_third_before[1] = value_third_sys[1];
		value_second_before[1] = value_second_sys[1];
		value_first_before[1] = value_first_sys[1];
	}

	input_report_abs(drop_laser_dev->dev, 1,
			 drop_laser_dev->value_fifth[0]);
	input_report_abs(drop_laser_dev->dev, 2,
			 drop_laser_dev->value_fourth[0]);
	input_report_abs(drop_laser_dev->dev, 3,
			 drop_laser_dev->value_third[0]);
	input_report_abs(drop_laser_dev->dev, 4,
			 drop_laser_dev->value_second[0]);
	input_report_abs(drop_laser_dev->dev, 5,
			 drop_laser_dev->value_first[0]);

	input_report_abs(drop_laser_dev->dev, 6,
			drop_laser_dev->value_fifth[1]);
	input_report_abs(drop_laser_dev->dev, 7,
			drop_laser_dev->value_fourth[1]);
	input_report_abs(drop_laser_dev->dev, 8,
			drop_laser_dev->value_third[1]);
	input_report_abs(drop_laser_dev->dev, 9,
			drop_laser_dev->value_second[1]);
	input_report_abs(drop_laser_dev->dev, 10,
			drop_laser_dev->value_first[1]);
	input_report_abs(drop_laser_dev->dev, ABS_TIMESTAMP,
			 drop_laser_dev->timestamp);
	timestamp_sys = drop_laser_dev->timestamp;
	input_sync(drop_laser_dev->dev);
	if (set_mode_first_time)
		set_mode_first_time = 0;
}

static int mcu_feedback_handler(struct notifier_block *self, u8 header,
				struct payload *sub_payload)
{
	struct drop_laser_dev *drop_laser_dev =
	    container_of(self, struct drop_laser_dev, nb);

	if (sensor_mode == 0)
		return 0;

	if (header == 2 && sub_payload->dataLength == 10) {
		drop_laser_dev->value_fifth[0] = (sub_payload->data[0] + (sub_payload->data[1] << 8));
		drop_laser_dev->value_fourth[0] = (sub_payload->data[2] + (sub_payload->data[3] << 8));
		drop_laser_dev->value_third[0] = (sub_payload->data[4] + (sub_payload->data[5] << 8));
		drop_laser_dev->value_second[0] = (sub_payload->data[6] + (sub_payload->data[7] << 8));
		drop_laser_dev->value_first[0] = (sub_payload->data[8] + (sub_payload->data[9] << 8));
	} else if (header == 0xD && sub_payload->dataLength == 10) {
		drop_laser_dev->timestamp = (s32)sub_payload->timestamp;
		drop_laser_dev->value_fifth[1] = (sub_payload->data[0] + (sub_payload->data[1] << 8));
		drop_laser_dev->value_fourth[1] = (sub_payload->data[2] + (sub_payload->data[3] << 8));
		drop_laser_dev->value_third[1] = (sub_payload->data[4] + (sub_payload->data[5] << 8));
		drop_laser_dev->value_second[1] = (sub_payload->data[6] + (sub_payload->data[7] << 8));
		drop_laser_dev->value_first[1] = (sub_payload->data[8] + (sub_payload->data[9] << 8));
	} else if (header == 0xFF && sub_payload->dataLength == 0)
		queue_delayed_work(drop_laser_dev->drop_laser_wq, &drop_laser_dev->work, 0);

	return 0;
}

static int drop_laser_init(void)
{
	int ret;
	struct drop_laser_dev *drop_laser_dev;
	sensor_mode = 0;
	set_mode_first_time = 0;

	pr_info("=== drop_laser_init ===\n");
	drop_laser_dev = kzalloc(sizeof(*drop_laser_dev), GFP_KERNEL);
	if (!drop_laser_dev) {
		pr_err("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_kzalloc_failed;
	}

	drop_laser_dev->drop_laser_wq =
			create_singlethread_workqueue("drop_laser_wq");
	if (!drop_laser_dev->drop_laser_wq) {
		pr_err("create_singlethread_workqueue failed!\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}

	INIT_DELAYED_WORK(&drop_laser_dev->work, drop_laser_work_function);
	drop_laser_dev->dev = input_allocate_device();

	if (!drop_laser_dev->dev) {
		ret = -ENOMEM;
		goto err_allocate_input_device;
	}

	drop_laser_dev->dev->name = "drop_laser";
	drop_laser_dev->dev->id.bustype = BUS_I2C;
	input_set_capability(drop_laser_dev->dev, EV_ABS, ABS_MISC);
	input_set_abs_params(drop_laser_dev->dev, 1, 0, 65535, 0, 0);
	input_set_abs_params(drop_laser_dev->dev, 2, 0, 65535, 0, 0);
	input_set_abs_params(drop_laser_dev->dev, 3, 0, 65535, 0, 0);
	input_set_abs_params(drop_laser_dev->dev, 4, 0, 65535, 0, 0);
	input_set_abs_params(drop_laser_dev->dev, 5, 0, 65535, 0, 0);
	input_set_abs_params(drop_laser_dev->dev, 6, 0, 65535, 0, 0);
	input_set_abs_params(drop_laser_dev->dev, 7, 0, 65535, 0, 0);
	input_set_abs_params(drop_laser_dev->dev, 8, 0, 65535, 0, 0);
	input_set_abs_params(drop_laser_dev->dev, 9, 0, 65535, 0, 0);
	input_set_abs_params(drop_laser_dev->dev, 10, 0, 65535, 0, 0);
	input_set_abs_params(drop_laser_dev->dev, ABS_TIMESTAMP, -2147483648, 2147483647, 0, 0);

	ret = input_register_device(drop_laser_dev->dev);
	if (ret < 0) {
		input_free_device(drop_laser_dev->dev);
		goto err_allocate_input_device;
	}

	memset(drop_laser_dev->value_fifth, 0, sizeof(drop_laser_dev->value_fifth));
	memset(drop_laser_dev->value_fourth, 0, sizeof(drop_laser_dev->value_fourth));
	memset(drop_laser_dev->value_third, 0, sizeof(drop_laser_dev->value_third));
	memset(drop_laser_dev->value_second, 0, sizeof(drop_laser_dev->value_second));
	memset(drop_laser_dev->value_first, 0, sizeof(drop_laser_dev->value_first));
	drop_laser_dev->timestamp = 0;

	memset(value_fifth_sys, 0, sizeof(value_fifth_sys));
	memset(value_fourth_sys, 0, sizeof(value_fourth_sys));
	memset(value_third_sys, 0, sizeof(value_third_sys));
	memset(value_second_sys, 0, sizeof(value_second_sys));
	memset(value_first_sys, 0, sizeof(value_first_sys));
	timestamp_sys = 0;

	memset(value_fifth_before, 65535, sizeof(value_fifth_before));
	memset(value_fourth_before, 65535, sizeof(value_fourth_before));
	memset(value_third_before, 65535, sizeof(value_third_before));
	memset(value_second_before, 65535, sizeof(value_second_before));
	memset(value_first_before, 65535, sizeof(value_first_before));

	drop_laser_dev->nb.notifier_call = mcu_feedback_handler;
	mcu_feedback_register_client(&drop_laser_dev->nb);

	drop_laser_dev->drop_laser_control_kobj =
	    kobject_create_and_add("drop_laser", NULL);
	drop_laser_dev->attrs.attrs = drop_laser_attrs;
	if (drop_laser_dev->drop_laser_control_kobj == NULL) {
		pr_err("%s: subsystem_register failed\n", __func__);
		goto err_allocate_input_device;
	}

	ret =
	    sysfs_create_group(drop_laser_dev->drop_laser_control_kobj,
			       &drop_laser_dev->attrs);

	if (ret) {
		pr_err("%s: sysfs_create_group: sonar_attrs failed\n",
				__func__);
		goto err_allocate_input_device;
	}

	return 0;
err_allocate_input_device:
	destroy_workqueue(drop_laser_dev->drop_laser_wq);
err_create_singlethread_workqueue_failed:
	kfree(drop_laser_dev);
err_kzalloc_failed:
	return ret;
}

static void drop_laser_exit(void)
{
	pr_info("=== drop_laser_exit ===\n");
}

module_init(drop_laser_init);
module_exit(drop_laser_exit);
