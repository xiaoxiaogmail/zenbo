/*
 * drivers/hwmon/wheel_encoder.c - wheel encoder sensor.
 *
 * Device Driver for wheel encoder
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

struct wheel_encoder_dev {
	struct input_dev *dev;
	struct workqueue_struct *wheel_encoder_wq;
	struct delayed_work work;
	struct notifier_block nb;
	struct attribute_group attrs;
	struct kobject *wheel_control_kobj;
	s32 wheel_encoder_left;
	s32 wheel_encoder_right;
	s32 timestamp;
};

static s32 timestamp_sys;
static s32 wheel_encoder_left_sys;
static s32 wheel_encoder_right_sys;
static s32 wheel_encoder_left_before;
static s32 wheel_encoder_right_before;
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
	return sprintf(buf, "%d %d\n", wheel_encoder_left_sys,
			wheel_encoder_right_sys);
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

static void wheel_encoder_work_function(struct work_struct *work)
{
	struct wheel_encoder_dev *wheel_encoder_dev =
	    container_of((struct delayed_work *)work, struct wheel_encoder_dev,
			 work);
	wheel_encoder_left_sys = wheel_encoder_dev->wheel_encoder_left;
	wheel_encoder_right_sys = wheel_encoder_dev->wheel_encoder_right;

	if (!set_mode_first_time &&
			wheel_encoder_left_sys == wheel_encoder_left_before &&
			wheel_encoder_right_sys == wheel_encoder_right_before) {
		return;
	} else {
		wheel_encoder_left_before = wheel_encoder_left_sys;
		wheel_encoder_right_before = wheel_encoder_right_sys;
	}

	input_report_abs(wheel_encoder_dev->dev, 1,
			 wheel_encoder_dev->wheel_encoder_left);
	input_report_abs(wheel_encoder_dev->dev, 2,
			 wheel_encoder_dev->wheel_encoder_right);
	input_report_abs(wheel_encoder_dev->dev, ABS_TIMESTAMP,
			 wheel_encoder_dev->timestamp);
	if (set_mode_first_time)
		set_mode_first_time = 0;

	timestamp_sys = wheel_encoder_dev->timestamp;
	input_sync(wheel_encoder_dev->dev);
}

static int mcu_feedback_handler(struct notifier_block *self, u8 header,
				struct payload *sub_payload)
{
	struct wheel_encoder_dev *wheel_encoder_dev =
	    container_of(self, struct wheel_encoder_dev, nb);

	if (sensor_mode == 0)
		return 0;

	if (header == 0x20 && sub_payload->dataLength == 8) {
		wheel_encoder_dev->timestamp = (s32)sub_payload->timestamp;
		wheel_encoder_dev->wheel_encoder_left = (sub_payload->data[0] +
				(sub_payload->data[1] << 8) + (sub_payload->data[2] << 16) +
				(sub_payload->data[3] << 24));
		wheel_encoder_dev->wheel_encoder_right = (sub_payload->data[4] +
				(sub_payload->data[5] << 8) + (sub_payload->data[6] << 16) +
				(sub_payload->data[7] << 24));
		queue_delayed_work(wheel_encoder_dev->wheel_encoder_wq,
				   &wheel_encoder_dev->work, 0);
	}

	return 0;
}

static int wheel_encoder_init(void)
{
	int ret;
	struct wheel_encoder_dev *wheel_encoder_dev;
	sensor_mode = 0;
	set_mode_first_time = 0;

	pr_info("=== wheel_encoder_init ===\n");
	wheel_encoder_dev = kzalloc(sizeof(*wheel_encoder_dev), GFP_KERNEL);
	if (!wheel_encoder_dev) {
		pr_err("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_kzalloc_failed;
	}

	wheel_encoder_dev->wheel_encoder_wq =
	    create_singlethread_workqueue("wheel_encoder_wq");
	if (!wheel_encoder_dev->wheel_encoder_wq) {
		pr_err("create_singlethread_workqueue failed!\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}

	INIT_DELAYED_WORK(&wheel_encoder_dev->work,
			  wheel_encoder_work_function);
	wheel_encoder_dev->dev = input_allocate_device();

	if (!wheel_encoder_dev->dev) {
		ret = -ENOMEM;
		goto err_allocate_input_device;
	}

	wheel_encoder_dev->dev->name = "wheel_encoder";
	wheel_encoder_dev->dev->id.bustype = BUS_I2C;
	input_set_capability(wheel_encoder_dev->dev, EV_ABS, ABS_MISC);
	input_set_abs_params(wheel_encoder_dev->dev, 1, -2147483648, 2147483647, 0, 0);
	input_set_abs_params(wheel_encoder_dev->dev, 2, -2147483648, 2147483647, 0, 0);
	input_set_abs_params(wheel_encoder_dev->dev, ABS_TIMESTAMP, -2147483648, 2147483647, 0,
			     0);
	ret = input_register_device(wheel_encoder_dev->dev);
	if (ret < 0) {
		input_free_device(wheel_encoder_dev->dev);
		goto err_allocate_input_device;
	}

	wheel_encoder_dev->timestamp = 0;
	wheel_encoder_dev->wheel_encoder_left = 0;
	wheel_encoder_dev->wheel_encoder_right = 0;

	timestamp_sys = 0;
	wheel_encoder_left_sys = 0;
	wheel_encoder_right_sys = 0;

	wheel_encoder_left_before = -2147483648;
	wheel_encoder_right_before = -2147483648;

	wheel_encoder_dev->nb.notifier_call = mcu_feedback_handler;
	mcu_feedback_register_client(&wheel_encoder_dev->nb);
	wheel_encoder_dev->wheel_control_kobj =
	    kobject_create_and_add("wheel", NULL);
	wheel_encoder_dev->attrs.attrs = wheel_attrs;
	if (wheel_encoder_dev->wheel_control_kobj == NULL) {
		pr_err("%s: subsystem_register failed\n", __func__);
		goto err_allocate_input_device;
	}

	ret =
	    sysfs_create_group(wheel_encoder_dev->wheel_control_kobj,
			       &wheel_encoder_dev->attrs);

	if (ret) {
		pr_err("%s: sysfs_create_group: wheel_attrs failed\n",
		       __func__);
		goto err_allocate_input_device;
	}

	return 0;

err_allocate_input_device:
	destroy_workqueue(wheel_encoder_dev->wheel_encoder_wq);
err_create_singlethread_workqueue_failed:
	kfree(wheel_encoder_dev);
err_kzalloc_failed:
	return ret;
}

static void wheel_encoder_exit(void)
{
	pr_info("=== wheel_encoder_exit ===\n");
}

module_init(wheel_encoder_init);
module_exit(wheel_encoder_exit);
