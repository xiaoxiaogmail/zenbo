/*
 * drivers/hwmon/odometry.c - odometry sensor.
 *
 * Device Driver for odometry
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

struct odometry_dev {
	struct input_dev *dev;
	struct workqueue_struct *odometry_wq;
	struct delayed_work work;
	struct notifier_block nb;
	struct attribute_group attrs;
	struct kobject *odometry_control_kobj;
	s32 odometry_x;
	s32 odometry_y;
	s32 odometry_theta;
	s32 timestamp;
};

static s32 odometry_x_sys;
static s32 odometry_y_sys;
static s32 odometry_theta_sys;
static s32 odometry_x_before;
static s32 odometry_y_before;
static s32 odometry_theta_before;
static s32 timestamp_sys;
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
	return sprintf(buf, "%d %d %d\n", odometry_x_sys, odometry_y_sys,
			odometry_theta_sys);
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

static struct attribute *wheel_attrs[] = {
	&dev_attr_timestamp_info.attr,
	&dev_attr_data_info.attr,
	&dev_attr_mode.attr,
	NULL
};

static void odometry_work_function(struct work_struct *work)
{
	struct odometry_dev *odometry_dev =
	    container_of((struct delayed_work *)work, struct odometry_dev,
			 work);
	odometry_x_sys = odometry_dev->odometry_x;
	odometry_y_sys = odometry_dev->odometry_y;
	odometry_theta_sys = odometry_dev->odometry_theta;

	if (!set_mode_first_time && (odometry_x_sys == odometry_x_before &&
			odometry_y_sys == odometry_y_before &&
			odometry_theta_sys == odometry_theta_before)) {
		return;
	} else {
		odometry_x_before = odometry_x_sys;
		odometry_y_before = odometry_y_sys;
		odometry_theta_before = odometry_theta_sys;
	}

	input_report_abs(odometry_dev->dev, 1, odometry_dev->odometry_x);
	input_report_abs(odometry_dev->dev, 2, odometry_dev->odometry_y);
	input_report_abs(odometry_dev->dev, 3, odometry_dev->odometry_theta);
	input_report_abs(odometry_dev->dev, ABS_TIMESTAMP,
			odometry_dev->timestamp);
	if (set_mode_first_time)
		set_mode_first_time = 0;

	timestamp_sys = odometry_dev->timestamp;
	input_sync(odometry_dev->dev);
}

static int mcu_feedback_handler(struct notifier_block *self, u8 header,
				struct payload *sub_payload)
{
	struct odometry_dev *odometry_dev =
	    container_of(self, struct odometry_dev, nb);

	if (sensor_mode == 0)
		return 0;

	if (header == 0x22 && sub_payload->dataLength == 12) {
		odometry_dev->timestamp = (s32)sub_payload->timestamp;
		odometry_dev->odometry_x = (sub_payload->data[0] + (sub_payload->data[1] << 8) +
				(sub_payload->data[2] << 16) + (sub_payload->data[3] << 24));
		odometry_dev->odometry_y = (sub_payload->data[4] + (sub_payload->data[5] << 8) +
				(sub_payload->data[6] << 16) + (sub_payload->data[7] << 24));
		odometry_dev->odometry_theta = (sub_payload->data[8] + (sub_payload->data[9] << 8) +
				(sub_payload->data[10] << 16) + (sub_payload->data[11] << 24));
		queue_delayed_work(odometry_dev->odometry_wq,
				   &odometry_dev->work, 0);
	}

	return 0;
}

static int odometry_init(void)
{
	int ret;
	struct odometry_dev *odometry_dev;
	sensor_mode = 0;
	set_mode_first_time = 0;

	pr_info("=== odometry_init ===\n");
	odometry_dev = kzalloc(sizeof(*odometry_dev), GFP_KERNEL);
	if (!odometry_dev) {
		pr_err("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_kzalloc_failed;
	}

	odometry_dev->odometry_wq =
	    create_singlethread_workqueue("odometry_wq");
	if (!odometry_dev->odometry_wq) {
		pr_err("create_singlethread_workqueue failed!\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}

	INIT_DELAYED_WORK(&odometry_dev->work, odometry_work_function);
	odometry_dev->dev = input_allocate_device();

	if (!odometry_dev->dev) {
		ret = -ENOMEM;
		goto err_allocate_input_device;
	}

	odometry_dev->dev->name = "odometry";
	odometry_dev->dev->id.bustype = BUS_I2C;
	input_set_capability(odometry_dev->dev, EV_ABS, ABS_MISC);
	input_set_abs_params(odometry_dev->dev, 1, -2147483648, 2147483647, 0, 0);
	input_set_abs_params(odometry_dev->dev, 2, -2147483648, 2147483647, 0, 0);
	input_set_abs_params(odometry_dev->dev, 3, -2147483648, 2147483647, 0, 0);
	input_set_abs_params(odometry_dev->dev, ABS_TIMESTAMP, -2147483648,
			2147483647, 0, 0);
	ret = input_register_device(odometry_dev->dev);
	if (ret < 0) {
		input_free_device(odometry_dev->dev);
		goto err_allocate_input_device;
	}

	odometry_dev->timestamp = 0;
	odometry_dev->odometry_x = 0;
	odometry_dev->odometry_y = 0;
	odometry_dev->odometry_theta = 0;

	timestamp_sys = 0;
	odometry_x_sys = 0;
	odometry_y_sys = 0;
	odometry_theta_sys = 0;

	odometry_x_before = -2147483648;
	odometry_y_before= -2147483648;
	odometry_theta_before = -2147483648;

	odometry_dev->nb.notifier_call = mcu_feedback_handler;
	mcu_feedback_register_client(&odometry_dev->nb);
	odometry_dev->odometry_control_kobj =
	    kobject_create_and_add("odometry", NULL);
	odometry_dev->attrs.attrs = wheel_attrs;
	if (odometry_dev->odometry_control_kobj == NULL) {
		pr_err("%s: subsystem_register failed\n", __func__);
		goto err_allocate_input_device;
	}

	ret =
	    sysfs_create_group(odometry_dev->odometry_control_kobj,
			       &odometry_dev->attrs);

	if (ret) {
		pr_err("%s: sysfs_create_group: wheel_attrs failed\n",
		       __func__);
		goto err_allocate_input_device;
	}

	return 0;
err_allocate_input_device:
	destroy_workqueue(odometry_dev->odometry_wq);
err_create_singlethread_workqueue_failed:
	kfree(odometry_dev);
err_kzalloc_failed:
	return ret;
}

static void odometry_exit(void)
{
	pr_info("=== odometry_exit ===\n");
}

module_init(odometry_init);
module_exit(odometry_exit);
