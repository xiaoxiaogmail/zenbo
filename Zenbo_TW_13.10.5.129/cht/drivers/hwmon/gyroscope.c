/*
 * drivers/hwmon/gyroscope.c - gyroscope sensor.
 *
 * Device Driver for gyroscope
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

struct gyroscope_dev {
	struct input_dev *dev;
	struct workqueue_struct *gyroscope_wq;
	struct delayed_work work;
	struct notifier_block nb;
	struct attribute_group attrs;
	struct kobject *gyroscope_control_kobj;
	s16 gyro_x;
	s16 gyro_y;
	s16 gyro_z;
	s32 timestamp;
};

static s16 gyro_x_sys;
static s16 gyro_y_sys;
static s16 gyro_z_sys;
static s16 gyro_x_before;
static s16 gyro_y_before;
static s16 gyro_z_before;
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
	return sprintf(buf, "%hd %hd %hd\n", gyro_x_sys, gyro_y_sys, gyro_z_sys);
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

static struct attribute *gyroscope_attrs[] = {
	&dev_attr_timestamp_info.attr,
	&dev_attr_data_info.attr,
	&dev_attr_mode.attr,
	NULL
};

static void gyroscope_work_function(struct work_struct *work)
{
	struct gyroscope_dev *gyroscope_dev =
	    container_of((struct delayed_work *)work, struct gyroscope_dev,
			 work);
	gyro_x_sys = gyroscope_dev->gyro_x;
	gyro_y_sys = gyroscope_dev->gyro_y;
	gyro_z_sys = gyroscope_dev->gyro_z;

	if (!set_mode_first_time && gyro_x_sys == gyro_x_before &&
			gyro_y_sys == gyro_y_before &&
			gyro_z_sys == gyro_z_before) {
		return;
	} else {
		gyro_x_before = gyro_x_sys;
		gyro_y_before = gyro_y_sys;
		gyro_z_before = gyro_z_sys;
	}

	input_report_abs(gyroscope_dev->dev, ABS_X,
			 gyroscope_dev->gyro_x);
	input_report_abs(gyroscope_dev->dev, ABS_Y,
			 gyroscope_dev->gyro_y);
	input_report_abs(gyroscope_dev->dev, ABS_Z,
			 gyroscope_dev->gyro_z);
	input_report_abs(gyroscope_dev->dev, ABS_TIMESTAMP,
			 gyroscope_dev->timestamp);
	if (set_mode_first_time)
		set_mode_first_time = 0;

	timestamp_sys = gyroscope_dev->timestamp;
	input_sync(gyroscope_dev->dev);
}

static int mcu_feedback_handler(struct notifier_block *self, u8 header,
				struct payload *sub_payload)
{
	struct gyroscope_dev *gyroscope_dev =
	    container_of(self, struct gyroscope_dev, nb);

	if (sensor_mode == 0)
		return 0;

	if (header == 0x6 && sub_payload->dataLength == 6) {
		gyroscope_dev->timestamp = (s32)sub_payload->timestamp;
		gyroscope_dev->gyro_x = (sub_payload->data[0] + (sub_payload->data[1] << 8));
		gyroscope_dev->gyro_y = (sub_payload->data[2] + (sub_payload->data[3] << 8));
		gyroscope_dev->gyro_z = (sub_payload->data[4] + (sub_payload->data[5] << 8));
		queue_delayed_work(gyroscope_dev->gyroscope_wq,
				   &gyroscope_dev->work, 0);
	}

	return 0;
}

static int gyroscope_init(void)
{
	int ret;
	struct gyroscope_dev *gyroscope_dev;
	sensor_mode = 0;
	set_mode_first_time = 0;

	pr_info("=== gyroscope_init ===\n");
	gyroscope_dev = kzalloc(sizeof(*gyroscope_dev), GFP_KERNEL);
	if (!gyroscope_dev) {
		pr_err("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_kzalloc_failed;
	}

	gyroscope_dev->gyroscope_wq =
	    create_singlethread_workqueue("gyroscope_wq");
	if (!gyroscope_dev->gyroscope_wq) {
		pr_err("create_singlethread_workqueue failed!\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}

	INIT_DELAYED_WORK(&gyroscope_dev->work, gyroscope_work_function);
	gyroscope_dev->dev = input_allocate_device();

	if (!gyroscope_dev->dev) {
		ret = -ENOMEM;
		goto err_allocate_input_device;
	}

	gyroscope_dev->dev->name = "gyroscope";
	gyroscope_dev->dev->id.bustype = BUS_I2C;
	input_set_capability(gyroscope_dev->dev, EV_ABS, ABS_MISC);
	input_set_abs_params(gyroscope_dev->dev, ABS_X, -32768, 32767, 0, 0);
	input_set_abs_params(gyroscope_dev->dev, ABS_Y, -32768, 32767, 0, 0);
	input_set_abs_params(gyroscope_dev->dev, ABS_Z, -32768, 32767, 0, 0);
	input_set_abs_params(gyroscope_dev->dev, ABS_TIMESTAMP, -2147483648, 2147483647, 0, 0);
	ret = input_register_device(gyroscope_dev->dev);
	if (ret < 0) {
		input_free_device(gyroscope_dev->dev);
		goto err_allocate_input_device;
	}

	gyroscope_dev->gyro_x = 0;
	gyroscope_dev->gyro_y = 0;
	gyroscope_dev->gyro_z = 0;
	gyroscope_dev->timestamp = 0;

	gyro_x_sys = 0;
	gyro_y_sys = 0;
	gyro_z_sys = 0;
	timestamp_sys = 0;

	gyro_x_before = -32768;
	gyro_y_before = -32768;
	gyro_z_before = -32768;

	gyroscope_dev->nb.notifier_call = mcu_feedback_handler;
	mcu_feedback_register_client(&gyroscope_dev->nb);

	gyroscope_dev->gyroscope_control_kobj =
	    kobject_create_and_add("gyroscope", NULL);
	gyroscope_dev->attrs.attrs = gyroscope_attrs;
	if (gyroscope_dev->gyroscope_control_kobj == NULL) {
		pr_err("%s: subsystem_register failed\n", __func__);
		goto err_allocate_input_device;
	}

	ret =
	    sysfs_create_group(gyroscope_dev->gyroscope_control_kobj,
			       &gyroscope_dev->attrs);

	if (ret) {
		pr_err("%s: sysfs_create_group: wheel_attrs failed\n",
		       __func__);
		goto err_allocate_input_device;
	}

	return 0;
err_allocate_input_device:
	destroy_workqueue(gyroscope_dev->gyroscope_wq);
err_create_singlethread_workqueue_failed:
	kfree(gyroscope_dev);
err_kzalloc_failed:
	return ret;
}

static void gyroscope_exit(void)
{
	pr_info("=== gyroscope_exit ===\n");
}

module_init(gyroscope_init);
module_exit(gyroscope_exit);
