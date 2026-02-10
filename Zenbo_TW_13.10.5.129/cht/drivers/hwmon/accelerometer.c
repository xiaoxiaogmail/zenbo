/*
 * drivers/hwmon/accelerometer.c - accelerometer sensor.
 *
 * Device Driver for accelerometer
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

struct accelerometer_dev {
	struct input_dev *dev;
	struct workqueue_struct *accelerometer_wq;
	struct delayed_work work;
	struct notifier_block nb;
	struct attribute_group attrs;
	struct kobject *accelerometer_control_kobj;
	s16 acc_x;
	s16 acc_y;
	s16 acc_z;
	s32 timestamp;
};

static s16 acc_x_sys;
static s16 acc_y_sys;
static s16 acc_z_sys;
static s32 timestamp_sys;

static s16 acc_x_before;
static s16 acc_y_before;
static s16 acc_z_before;

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
	return sprintf(buf, "%hd %hd %hd\n", acc_x_sys, acc_y_sys, acc_z_sys);
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

static struct attribute *accelerometer_attrs[] = {
	&dev_attr_timestamp_info.attr,
	&dev_attr_data_info.attr,
	&dev_attr_mode.attr,
	NULL
};

static void accelerometer_work_function(struct work_struct *work)
{
	struct accelerometer_dev *accelerometer_dev =
	    container_of((struct delayed_work *)work, struct accelerometer_dev,
			 work);
	acc_x_sys = accelerometer_dev->acc_x;
	acc_y_sys = accelerometer_dev->acc_y;
	acc_z_sys = accelerometer_dev->acc_z;

	if (!set_mode_first_time && acc_x_sys == acc_x_before &&
			acc_y_sys == acc_y_before &&
			acc_z_sys == acc_z_before) {
		return;
	} else {
		acc_x_before = acc_x_sys;
		acc_y_before = acc_y_sys;
		acc_z_before = acc_z_sys;
	}

	input_report_abs(accelerometer_dev->dev, ABS_X,
			accelerometer_dev->acc_x);
	input_report_abs(accelerometer_dev->dev, ABS_Y,
			accelerometer_dev->acc_y);
	input_report_abs(accelerometer_dev->dev, ABS_Z,
			accelerometer_dev->acc_z);
	input_report_abs(accelerometer_dev->dev, ABS_TIMESTAMP,
			accelerometer_dev->timestamp);
	if (set_mode_first_time)
		set_mode_first_time = 0;

	timestamp_sys = accelerometer_dev->timestamp;
	input_sync(accelerometer_dev->dev);
}

static int mcu_feedback_handler(struct notifier_block *self, u8 header,
				struct payload *sub_payload)
{
	struct accelerometer_dev *accelerometer_dev =
	    container_of(self, struct accelerometer_dev, nb);

	if (sensor_mode == 0)
		return 0;

	if (header == 0x5 && sub_payload->dataLength == 6) {
		accelerometer_dev->timestamp = (s32)sub_payload->timestamp;
		accelerometer_dev->acc_x = (sub_payload->data[0] + (sub_payload->data[1] << 8));
		accelerometer_dev->acc_y = (sub_payload->data[2] + (sub_payload->data[3] << 8));
		accelerometer_dev->acc_z = (sub_payload->data[4] + (sub_payload->data[5] << 8));
		queue_delayed_work(accelerometer_dev->accelerometer_wq,
				   &accelerometer_dev->work, 0);
	}

	return 0;
}

static int accelerometer_init(void)
{
	int ret;
	struct accelerometer_dev *accelerometer_dev;
	sensor_mode = 0;
	set_mode_first_time = 0;

	pr_info("=== accelerometer_init ===\n");
	accelerometer_dev = kzalloc(sizeof(*accelerometer_dev), GFP_KERNEL);
	if (!accelerometer_dev) {
		pr_err("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_kzalloc_failed;
	}

	accelerometer_dev->accelerometer_wq =
	    create_singlethread_workqueue("accelerometer_wq");
	if (!accelerometer_dev->accelerometer_wq) {
		pr_err("create_singlethread_workqueue failed!\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}

	INIT_DELAYED_WORK(&accelerometer_dev->work, accelerometer_work_function);
	accelerometer_dev->dev = input_allocate_device();

	if (!accelerometer_dev->dev) {
		ret = -ENOMEM;
		goto err_allocate_input_device;
	}

	accelerometer_dev->dev->name = "accelerometer";
	accelerometer_dev->dev->id.bustype = BUS_I2C;
	input_set_capability(accelerometer_dev->dev, EV_ABS, ABS_MISC);
	input_set_abs_params(accelerometer_dev->dev, ABS_X, -32768, 32767, 0, 0);
	input_set_abs_params(accelerometer_dev->dev, ABS_Y, -32768, 32767, 0, 0);
	input_set_abs_params(accelerometer_dev->dev, ABS_Z, -32768, 32767, 0, 0);
	input_set_abs_params(accelerometer_dev->dev, ABS_TIMESTAMP, -2147483648, 2147483647, 0, 0);
	ret = input_register_device(accelerometer_dev->dev);
	if (ret < 0) {
		input_free_device(accelerometer_dev->dev);
		goto err_allocate_input_device;
	}

	accelerometer_dev->acc_x = 0;
	accelerometer_dev->acc_y = 0;
	accelerometer_dev->acc_z = 0;
	accelerometer_dev->timestamp = 0;

	acc_x_sys = 0;
	acc_y_sys = 0;
	acc_z_sys = 0;
	timestamp_sys = 0;

	acc_x_before = 0;
	acc_y_before = 0;
	acc_z_before = 0;

	accelerometer_dev->nb.notifier_call = mcu_feedback_handler;
	mcu_feedback_register_client(&accelerometer_dev->nb);

	accelerometer_dev->accelerometer_control_kobj =
	    kobject_create_and_add("accelerometer", NULL);
	accelerometer_dev->attrs.attrs = accelerometer_attrs;
	if (accelerometer_dev->accelerometer_control_kobj == NULL) {
		pr_err("%s: subsystem_register failed\n", __func__);
		goto err_allocate_input_device;
	}

	ret =
	    sysfs_create_group(accelerometer_dev->accelerometer_control_kobj,
			       &accelerometer_dev->attrs);

	if (ret) {
		pr_err("%s: sysfs_create_group: wheel_attrs failed\n",
		       __func__);
		goto err_allocate_input_device;
	}

	return 0;
err_allocate_input_device:
	destroy_workqueue(accelerometer_dev->accelerometer_wq);
err_create_singlethread_workqueue_failed:
	kfree(accelerometer_dev);
err_kzalloc_failed:
	return ret;
}

static void accelerometer_exit(void)
{
	pr_info("=== accelerometer_exit ===\n");
}

module_init(accelerometer_init);
module_exit(accelerometer_exit);
