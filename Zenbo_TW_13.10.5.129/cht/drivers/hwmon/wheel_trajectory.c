/*
 * drivers/hwmon/wheel_trajectory.c - wheel trajectory sensor.
 *
 * Device Driver for wheel trajectory sensor
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

struct wheel_trajectory_dev {
	struct input_dev *dev;
	struct workqueue_struct *wheel_trajectory_wq;
	struct delayed_work work;
	struct notifier_block nb;
	struct attribute_group attrs;
	struct kobject *wheel_trajectory_control_kobj;
	s16 left;
	s16 right;
	s32 timestamp;
};

static s16 left_sys;
static s16 right_sys;
static s16 left_before;
static s16 right_before;
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
	return sprintf(buf, "%hd %hd\n", left_sys, right_sys);
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

static struct attribute *wheel_trajectory_attrs[] = {
	&dev_attr_timestamp_info.attr,
	&dev_attr_data_info.attr,
	&dev_attr_mode.attr,
	NULL
};

static void wheel_trajectory_work_function(struct work_struct *work)
{
	struct wheel_trajectory_dev *wheel_trajectory_dev =
	    container_of((struct delayed_work *)work, struct wheel_trajectory_dev,
			 work);
	left_sys = wheel_trajectory_dev->left;
	right_sys = wheel_trajectory_dev->right;

	if (!set_mode_first_time && left_sys == left_before &&
		right_sys == right_before) {
		return;
	} else {
		left_before = left_sys;
		right_before = right_sys;
	}

	input_report_abs(wheel_trajectory_dev->dev, 1,
			 wheel_trajectory_dev->left);
	input_report_abs(wheel_trajectory_dev->dev, 2,
			 wheel_trajectory_dev->right);
	input_report_abs(wheel_trajectory_dev->dev, ABS_TIMESTAMP,
			 wheel_trajectory_dev->timestamp);
	if (set_mode_first_time)
		set_mode_first_time = 0;

	timestamp_sys = wheel_trajectory_dev->timestamp;
	input_sync(wheel_trajectory_dev->dev);
}

static int mcu_feedback_handler(struct notifier_block *self, u8 header,
				struct payload *sub_payload)
{
	struct wheel_trajectory_dev *wheel_trajectory_dev =
	    container_of(self, struct wheel_trajectory_dev, nb);

	if (sensor_mode == 0)
		return 0;

	if (header == 0x31 && sub_payload->dataLength == 4) {
		wheel_trajectory_dev->timestamp = (s32)sub_payload->timestamp;
		wheel_trajectory_dev->left = (sub_payload->data[0] +
				(sub_payload->data[1] << 8));
		wheel_trajectory_dev->right = (sub_payload->data[2] +
				(sub_payload->data[3] << 8));
		queue_delayed_work(wheel_trajectory_dev->wheel_trajectory_wq,
				&wheel_trajectory_dev->work, 0);
	}
	return 0;
}

static int wheel_trajectory_init(void)
{
	int ret;
	struct wheel_trajectory_dev *wheel_trajectory_dev;
	sensor_mode = 0;
	set_mode_first_time = 0;

	pr_info("=== wheel_trajectory_init ===\n");
	wheel_trajectory_dev = kzalloc(sizeof(*wheel_trajectory_dev), GFP_KERNEL);
	if (!wheel_trajectory_dev) {
		pr_err("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_kzalloc_failed;
	}

	wheel_trajectory_dev->wheel_trajectory_wq =
	    create_singlethread_workqueue("wheel_trajectory_wq");
	if (!wheel_trajectory_dev->wheel_trajectory_wq) {
		pr_err("create_singlethread_workqueue failed!\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}

	INIT_DELAYED_WORK(&wheel_trajectory_dev->work, wheel_trajectory_work_function);
	wheel_trajectory_dev->dev = input_allocate_device();

	if (!wheel_trajectory_dev->dev) {
		ret = -ENOMEM;
		goto err_allocate_input_device;
	}

	wheel_trajectory_dev->dev->name = "wheel_trajectory";
	wheel_trajectory_dev->dev->id.bustype = BUS_I2C;
	input_set_capability(wheel_trajectory_dev->dev, EV_ABS, ABS_MISC);
	input_set_abs_params(wheel_trajectory_dev->dev, 1, -500, 500, 0, 0);
	input_set_abs_params(wheel_trajectory_dev->dev, 2, -500, 500, 0, 0);
	input_set_abs_params(wheel_trajectory_dev->dev, ABS_TIMESTAMP, -2147483648, 2147483647, 0, 0);
	ret = input_register_device(wheel_trajectory_dev->dev);
	if (ret < 0) {
		input_free_device(wheel_trajectory_dev->dev);
		goto err_allocate_input_device;
	}

	wheel_trajectory_dev->left = 0;
	wheel_trajectory_dev->right = 0;
	wheel_trajectory_dev->timestamp = 0;

	left_sys = 0;
	right_sys = 0;
	timestamp_sys = 0;

	left_before = -501;
	right_before = -501;

	wheel_trajectory_dev->nb.notifier_call = mcu_feedback_handler;
	mcu_feedback_register_client(&wheel_trajectory_dev->nb);

	wheel_trajectory_dev->wheel_trajectory_control_kobj =
	    kobject_create_and_add("wheel_trajectory", NULL);
	wheel_trajectory_dev->attrs.attrs = wheel_trajectory_attrs;
	if (wheel_trajectory_dev->wheel_trajectory_control_kobj == NULL) {
		pr_err("%s: subsystem_register failed\n", __func__);
		goto err_allocate_input_device;
	}

	ret =
	    sysfs_create_group(wheel_trajectory_dev->wheel_trajectory_control_kobj,
			       &wheel_trajectory_dev->attrs);

	if (ret) {
		pr_err("%s: sysfs_create_group: wheel_attrs failed\n",
		       __func__);
		goto err_allocate_input_device;
	}

	return 0;
err_allocate_input_device:
	destroy_workqueue(wheel_trajectory_dev->wheel_trajectory_wq);
err_create_singlethread_workqueue_failed:
	kfree(wheel_trajectory_dev);
err_kzalloc_failed:
	return ret;
}

static void wheel_trajectory_exit(void)
{
	pr_info("=== wheel_trajectory_exit ===\n");
}

module_init(wheel_trajectory_init);
module_exit(wheel_trajectory_exit);
