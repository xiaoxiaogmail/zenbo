/*
 * drivers/hwmon/neck_trajectory.c - neck trajectory sensor.
 *
 * Device Driver for neck trajectory sensor
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

struct neck_trajectory_dev {
	struct input_dev *dev;
	struct workqueue_struct *neck_trajectory_wq;
	struct delayed_work work;
	struct notifier_block nb;
	struct attribute_group attrs;
	struct kobject *neck_trajectory_control_kobj;
	s16 yaw;
	s16 pitch;
	s32 timestamp;
};

static s16 yaw_sys;
static s16 pitch_sys;
static s32 timestamp_sys;

static s16 yaw_before;
static s16 pitch_before;

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
	return sprintf(buf, "%hd %hd\n", yaw_sys, pitch_sys);
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

static struct attribute *neck_trajectory_attrs[] = {
	&dev_attr_timestamp_info.attr,
	&dev_attr_data_info.attr,
	&dev_attr_mode.attr,
	NULL
};

static void neck_trajectory_work_function(struct work_struct *work)
{
	struct neck_trajectory_dev *neck_trajectory_dev =
	    container_of((struct delayed_work *)work, struct neck_trajectory_dev,
			 work);
	yaw_sys = neck_trajectory_dev->yaw;
	pitch_sys = neck_trajectory_dev->pitch;

	if (!set_mode_first_time && yaw_sys == yaw_before &&
			pitch_sys == pitch_before) {
		return;
	} else {
		yaw_before = yaw_sys;
		pitch_before = pitch_sys;
	}

	input_report_abs(neck_trajectory_dev->dev, 1,
			 neck_trajectory_dev->yaw);
	input_report_abs(neck_trajectory_dev->dev, 2,
			 neck_trajectory_dev->pitch);
	input_report_abs(neck_trajectory_dev->dev, ABS_TIMESTAMP,
			 neck_trajectory_dev->timestamp);
	if (set_mode_first_time)
		set_mode_first_time = 0;

	timestamp_sys = neck_trajectory_dev->timestamp;
	input_sync(neck_trajectory_dev->dev);
}

static int mcu_feedback_handler(struct notifier_block *self, u8 header,
				struct payload *sub_payload)
{
	struct neck_trajectory_dev *neck_trajectory_dev =
	    container_of(self, struct neck_trajectory_dev, nb);

	if (sensor_mode == 0)
		return 0;

	if (header == 0x30 && sub_payload->dataLength == 4) {
		neck_trajectory_dev->timestamp = (s32)sub_payload->timestamp;
		neck_trajectory_dev->yaw = (sub_payload->data[0] +
				(sub_payload->data[1] << 8));
		neck_trajectory_dev->pitch = (sub_payload->data[2] +
				(sub_payload->data[3] << 8));
		queue_delayed_work(neck_trajectory_dev->neck_trajectory_wq,
				&neck_trajectory_dev->work, 0);
	}
	return 0;
}

static int neck_trajectory_init(void)
{
	int ret;
	struct neck_trajectory_dev *neck_trajectory_dev;
	sensor_mode = 0;
	set_mode_first_time = 0;

	pr_info("=== neck_trajectory_init ===\n");
	neck_trajectory_dev = kzalloc(sizeof(*neck_trajectory_dev), GFP_KERNEL);
	if (!neck_trajectory_dev) {
		pr_err("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_kzalloc_failed;
	}

	neck_trajectory_dev->neck_trajectory_wq =
	    create_singlethread_workqueue("neck_trajectory_wq");
	if (!neck_trajectory_dev->neck_trajectory_wq) {
		pr_err("create_singlethread_workqueue failed!\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}

	INIT_DELAYED_WORK(&neck_trajectory_dev->work, neck_trajectory_work_function);
	neck_trajectory_dev->dev = input_allocate_device();

	if (!neck_trajectory_dev->dev) {
		ret = -ENOMEM;
		goto err_allocate_input_device;
	}

	neck_trajectory_dev->dev->name = "neck_trajectory";
	neck_trajectory_dev->dev->id.bustype = BUS_I2C;
	input_set_capability(neck_trajectory_dev->dev, EV_ABS, ABS_MISC);
	input_set_abs_params(neck_trajectory_dev->dev, 1, -450, 550, 0, 0);
	input_set_abs_params(neck_trajectory_dev->dev, 2, -150, 550, 0, 0);
	input_set_abs_params(neck_trajectory_dev->dev, ABS_TIMESTAMP, -2147483648, 2147483647, 0, 0);
	ret = input_register_device(neck_trajectory_dev->dev);
	if (ret < 0) {
		input_free_device(neck_trajectory_dev->dev);
		goto err_allocate_input_device;
	}

	neck_trajectory_dev->yaw = 0;
	neck_trajectory_dev->pitch = 0;
	neck_trajectory_dev->timestamp = 0;

	yaw_sys = 0;
	pitch_sys = 0;
	timestamp_sys = 0;

	yaw_before = -451;
	pitch_before = -151;

	neck_trajectory_dev->nb.notifier_call = mcu_feedback_handler;
	mcu_feedback_register_client(&neck_trajectory_dev->nb);

	neck_trajectory_dev->neck_trajectory_control_kobj =
	    kobject_create_and_add("neck_trajectory", NULL);
	neck_trajectory_dev->attrs.attrs = neck_trajectory_attrs;
	if (neck_trajectory_dev->neck_trajectory_control_kobj == NULL) {
		pr_err("%s: subsystem_register failed\n", __func__);
		goto err_allocate_input_device;
	}

	ret =
	    sysfs_create_group(neck_trajectory_dev->neck_trajectory_control_kobj,
			       &neck_trajectory_dev->attrs);

	if (ret) {
		pr_err("%s: sysfs_create_group: wheel_attrs failed\n",
		       __func__);
		goto err_allocate_input_device;
	}

	return 0;
err_allocate_input_device:
	destroy_workqueue(neck_trajectory_dev->neck_trajectory_wq);
err_create_singlethread_workqueue_failed:
	kfree(neck_trajectory_dev);
err_kzalloc_failed:
	return ret;
}

static void neck_trajectory_exit(void)
{
	pr_info("=== neck_trajectory_exit ===\n");
}

module_init(neck_trajectory_init);
module_exit(neck_trajectory_exit);
