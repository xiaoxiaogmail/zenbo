/*
 * drivers/hwmon/sonar.c - sonar sensor.
 *
 * Device Driver for sonar
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

struct sonar_dev {
	struct input_dev *dev;
	struct workqueue_struct *sonar_wq;
	struct delayed_work work;
	struct notifier_block nb;
	struct attribute_group attrs;
	struct kobject *sonar_control_kobj;
	u16 value_right;
	u16 value_left;
	u16 value_back;
	u16 value_front_right;
	u16 value_front_left;
	u16 value_front_center;
	s32 timestamp;
};

static u16 value_right_sys;
static u16 value_left_sys;
static u16 value_back_sys;
static u16 value_front_right_sys;
static u16 value_front_left_sys;
static u16 value_front_center_sys;

static u16 value_right_before;
static u16 value_left_before;
static u16 value_back_before;
static s32 timestamp_sys;

static u16 value_front_right_before;
static u16 value_front_left_before;
static u16 value_front_center_before;

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
	return sprintf(buf, "%d %d %d %d %d %d\n",
			value_right_sys, value_left_sys, value_back_sys,
			value_front_right_sys, value_front_left_sys,
			value_front_center_sys);
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

static struct attribute *sonar_attrs[] = {
	&dev_attr_timestamp_info.attr,
	&dev_attr_data_info.attr,
	&dev_attr_mode.attr,
	NULL
};

static void sonar_work_function(struct work_struct *work)
{
	struct sonar_dev *sonar_dev =
	    container_of((struct delayed_work *)work, struct sonar_dev,
			 work);
	value_right_sys = sonar_dev->value_right;
	value_left_sys = sonar_dev->value_left;
	value_back_sys = sonar_dev->value_back;
	value_front_right_sys = sonar_dev->value_front_right;
	value_front_left_sys = sonar_dev->value_front_left;
	value_front_center_sys = sonar_dev->value_front_center;

	if (!set_mode_first_time && value_right_sys == value_right_before &&
			value_left_sys == value_left_before &&
			value_back_sys == value_back_before &&
			value_front_right_sys == value_front_right_before &&
			value_front_left_sys == value_front_left_before &&
			value_front_center_sys == value_front_center_before) {
		return;
	} else {
		value_right_before = value_right_sys;
		value_left_before = value_left_sys;
		value_back_before = value_back_sys;
		value_front_right_before = value_front_right_sys;
		value_front_left_before = value_front_left_sys;
		value_front_center_before = value_front_center_sys;
	}

	input_report_abs(sonar_dev->dev, 1, sonar_dev->value_right);
	input_report_abs(sonar_dev->dev, 2, sonar_dev->value_left);
	input_report_abs(sonar_dev->dev, 3, sonar_dev->value_back);
	input_report_abs(sonar_dev->dev, 4, sonar_dev->value_front_right);
	input_report_abs(sonar_dev->dev, 5, sonar_dev->value_front_left);
	input_report_abs(sonar_dev->dev, 6, sonar_dev->value_front_center);
	input_report_abs(sonar_dev->dev, ABS_TIMESTAMP,
			sonar_dev->timestamp);
	if (set_mode_first_time)
		set_mode_first_time = 0;

	timestamp_sys = sonar_dev->timestamp;
	input_sync(sonar_dev->dev);
}


static int mcu_feedback_handler(struct notifier_block *self, u8 header,
				struct payload *sub_payload)
{
	struct sonar_dev *sonar_dev = container_of(self, struct sonar_dev, nb);

	if (sensor_mode == 0)
		return 0;

	if (header == 3 && sub_payload->dataLength == 12) {
		sonar_dev->timestamp = (s32)sub_payload->timestamp;
		sonar_dev->value_right = (sub_payload->data[0] +
				(sub_payload->data[1] << 8));
		sonar_dev->value_left = (sub_payload->data[2] +
				(sub_payload->data[3] << 8));
		sonar_dev->value_back = (sub_payload->data[4] +
				(sub_payload->data[5] << 8));
		sonar_dev->value_front_right = (sub_payload->data[6] +
				(sub_payload->data[7] << 8));
		sonar_dev->value_front_left = (sub_payload->data[8] +
				(sub_payload->data[9] << 8));
		sonar_dev->value_front_center  = (sub_payload->data[10] +
				(sub_payload->data[11] << 8));
		queue_delayed_work(sonar_dev->sonar_wq, &sonar_dev->work, 0);
	}

	return 0;
}

static int sonar_init(void)
{
	int ret;
	struct sonar_dev *sonar_dev;
	sensor_mode = 0;
	set_mode_first_time = 0;

	pr_info("=== sonar_init ===\n");
	sonar_dev = kzalloc(sizeof(*sonar_dev), GFP_KERNEL);
	if (!sonar_dev) {
		pr_err("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_kzalloc_failed;
	}

	sonar_dev->sonar_wq = create_singlethread_workqueue("sonar_wq");
	if (!sonar_dev->sonar_wq) {
		pr_err("create_singlethread_workqueue failed!\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}

	INIT_DELAYED_WORK(&sonar_dev->work, sonar_work_function);
	sonar_dev->dev = input_allocate_device();

	if (!sonar_dev->dev) {
		ret = -ENOMEM;
		goto err_allocate_input_device;
	}

	sonar_dev->dev->name = "sonar";
	sonar_dev->dev->id.bustype = BUS_I2C;
	input_set_capability(sonar_dev->dev, EV_ABS, ABS_MISC);
	input_set_abs_params(sonar_dev->dev, 1, 0, 65535, 0, 0);
	input_set_abs_params(sonar_dev->dev, 2, 0, 65535, 0, 0);
	input_set_abs_params(sonar_dev->dev, 3, 0, 65535, 0, 0);
	input_set_abs_params(sonar_dev->dev, 4, 0, 65535, 0, 0);
	input_set_abs_params(sonar_dev->dev, 5, 0, 65535, 0, 0);
	input_set_abs_params(sonar_dev->dev, 6, 0, 65535, 0, 0);
	input_set_abs_params(sonar_dev->dev, ABS_TIMESTAMP, -2147483648, 2147483647, 0, 0);
	ret = input_register_device(sonar_dev->dev);
	if (ret < 0) {
		input_free_device(sonar_dev->dev);
		goto err_allocate_input_device;
	}

	sonar_dev->value_right = 0;
	sonar_dev->value_left = 0;
	sonar_dev->value_back = 0;
	sonar_dev->value_front_right = 0;
	sonar_dev->value_front_left = 0;
	sonar_dev->value_front_center = 0;
	sonar_dev->timestamp = 0;

	timestamp_sys = 0;
	value_right_sys = 0;
	value_left_sys = 0;
	value_back_sys = 0;
	value_front_right_sys = 0;
	value_front_left_sys = 0;
	value_front_center_sys = 0;

	value_right_before = 65535;
	value_left_before = 65535;
	value_back_before = 65535;
	value_front_right_before = 65535;
	value_front_left_before = 65535;
	value_front_center_before = 65535;

	sonar_dev->nb.notifier_call = mcu_feedback_handler;
	mcu_feedback_register_client(&sonar_dev->nb);

	sonar_dev->sonar_control_kobj = kobject_create_and_add("sonar", NULL);
	sonar_dev->attrs.attrs = sonar_attrs;
	if (sonar_dev->sonar_control_kobj == NULL) {
		pr_err("%s: subsystem_register failed\n", __func__);
		goto err_allocate_input_device;
	}

	ret =
	    sysfs_create_group(sonar_dev->sonar_control_kobj,
			       &sonar_dev->attrs);

	if (ret) {
		pr_err("%s: sysfs_create_group: sonar_attrs failed\n",
		       __func__);
		goto err_allocate_input_device;
	}

	return 0;
err_allocate_input_device:
	destroy_workqueue(sonar_dev->sonar_wq);
err_create_singlethread_workqueue_failed:
	kfree(sonar_dev);
err_kzalloc_failed:
	return ret;
}

static void sonar_exit(void)
{
	pr_info("=== sonar_exit ===\n");
}

module_init(sonar_init);
module_exit(sonar_exit);
