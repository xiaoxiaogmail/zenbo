/*
 * drivers/hwmon/dock_ir.c - dock ir sensor.
 *
 * Device Driver for dock ir
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

struct dock_ir_dev {
	struct input_dev *dev;
	struct workqueue_struct *dock_ir_wq;
	struct delayed_work work;
	struct notifier_block nb;
	struct attribute_group attrs;
	struct kobject *dock_ir_control_kobj;
	u16 dock_ir_right;
	u16 dock_ir_center;
	u16 dock_ir_left;
	s32 timestamp;
};

static u16 dock_ir_right_sys;
static u16 dock_ir_center_sys;
static u16 dock_ir_left_sys;
static s32 timestamp_sys;
static u16 dock_ir_right_before;
static u16 dock_ir_center_before;
static u16 dock_ir_left_before;
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
	return sprintf(buf, "%d %d %d\n", dock_ir_right_sys,
			dock_ir_center_sys, dock_ir_left_sys);
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

static struct attribute *dock_ir_attrs[] = {
	&dev_attr_timestamp_info.attr,
	&dev_attr_data_info.attr,
	&dev_attr_mode.attr,
	NULL
};

static void dock_ir_work_function(struct work_struct *work)
{
	struct dock_ir_dev *dock_ir_dev =
	    container_of((struct delayed_work *)work, struct dock_ir_dev,
			 work);
	dock_ir_right_sys = dock_ir_dev->dock_ir_right;
	dock_ir_center_sys = dock_ir_dev->dock_ir_center;
	dock_ir_left_sys = dock_ir_dev->dock_ir_left;

	if (!set_mode_first_time && dock_ir_right_sys == dock_ir_right_before &&
			dock_ir_center_sys == dock_ir_center_before &&
			dock_ir_left_sys == dock_ir_left_before) {
		return;
	} else {
		dock_ir_right_before = dock_ir_right_sys;
		dock_ir_center_before = dock_ir_center_sys;
		dock_ir_left_before = dock_ir_left_sys;
	}

	input_report_abs(dock_ir_dev->dev, 1,
			 dock_ir_dev->dock_ir_right);
	input_report_abs(dock_ir_dev->dev, 2,
			 dock_ir_dev->dock_ir_center);
	input_report_abs(dock_ir_dev->dev, 3,
			 dock_ir_dev->dock_ir_left);
	input_report_abs(dock_ir_dev->dev, ABS_TIMESTAMP,
			 dock_ir_dev->timestamp);

	if (set_mode_first_time)
		set_mode_first_time = 0;

	timestamp_sys = dock_ir_dev->timestamp;
	input_sync(dock_ir_dev->dev);
}

static int mcu_feedback_handler(struct notifier_block *self, u8 header,
				struct payload *sub_payload)
{
	struct dock_ir_dev *dock_ir_dev =
	    container_of(self, struct dock_ir_dev, nb);

	if (sensor_mode == 0)
		return 0;

	if (header == 0x4 && sub_payload->dataLength == 6) {
		dock_ir_dev->timestamp = (s32)sub_payload->timestamp;
		dock_ir_dev->dock_ir_right = (sub_payload->data[0] + (sub_payload->data[1] << 8));
		dock_ir_dev->dock_ir_center = (sub_payload->data[2] + (sub_payload->data[3] << 8));
		dock_ir_dev->dock_ir_left = (sub_payload->data[4] + (sub_payload->data[5] << 8));
		queue_delayed_work(dock_ir_dev->dock_ir_wq,
				   &dock_ir_dev->work, 0);
	}

	return 0;
}

static int dock_ir_init(void)
{
	int ret;
	struct dock_ir_dev *dock_ir_dev;
	sensor_mode = 0;
	set_mode_first_time = 0;

	pr_info("=== dock_ir_init ===\n");
	dock_ir_dev = kzalloc(sizeof(*dock_ir_dev), GFP_KERNEL);
	if (!dock_ir_dev) {
		pr_err("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_kzalloc_failed;
	}

	dock_ir_dev->dock_ir_wq =
	    create_singlethread_workqueue("dock_ir_wq");
	if (!dock_ir_dev->dock_ir_wq) {
		pr_err("create_singlethread_workqueue failed!\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}

	INIT_DELAYED_WORK(&dock_ir_dev->work, dock_ir_work_function);
	dock_ir_dev->dev = input_allocate_device();

	if (!dock_ir_dev->dev) {
		ret = -ENOMEM;
		goto err_allocate_input_device;
	}

	dock_ir_dev->dev->name = "dock_ir";
	dock_ir_dev->dev->id.bustype = BUS_I2C;
	input_set_capability(dock_ir_dev->dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dock_ir_dev->dev, 1, 0, 7, 0, 0);
	input_set_abs_params(dock_ir_dev->dev, 2, 0, 7, 0, 0);
	input_set_abs_params(dock_ir_dev->dev, 3, 0, 7, 0, 0);
	input_set_abs_params(dock_ir_dev->dev, ABS_TIMESTAMP, -2147483648, 2147483647, 0, 0);
	ret = input_register_device(dock_ir_dev->dev);
	if (ret < 0) {
		input_free_device(dock_ir_dev->dev);
		goto err_allocate_input_device;
	}

	dock_ir_dev->dock_ir_right = 0;
	dock_ir_dev->dock_ir_center = 0;
	dock_ir_dev->dock_ir_left = 0;
	dock_ir_dev->timestamp = 0;

	dock_ir_right_sys = 0;
	dock_ir_center_sys = 0;
	dock_ir_left_sys = 0;
	timestamp_sys = 0;

	dock_ir_right_before = 8;
	dock_ir_center_before = 8;
	dock_ir_left_before = 8;

	dock_ir_dev->nb.notifier_call = mcu_feedback_handler;
	mcu_feedback_register_client(&dock_ir_dev->nb);

	dock_ir_dev->dock_ir_control_kobj =
	    kobject_create_and_add("dock_ir", NULL);
	dock_ir_dev->attrs.attrs = dock_ir_attrs;
	if (dock_ir_dev->dock_ir_control_kobj == NULL) {
		pr_err("%s: subsystem_register failed\n", __func__);
		goto err_allocate_input_device;
	}

	ret =
	    sysfs_create_group(dock_ir_dev->dock_ir_control_kobj,
			       &dock_ir_dev->attrs);

	if (ret) {
		pr_err("%s: sysfs_create_group: wheel_attrs failed\n",
		       __func__);
		goto err_allocate_input_device;
	}

	return 0;
err_allocate_input_device:
	destroy_workqueue(dock_ir_dev->dock_ir_wq);
err_create_singlethread_workqueue_failed:
	kfree(dock_ir_dev);
err_kzalloc_failed:
	return ret;
}

static void dock_ir_exit(void)
{
	pr_info("=== dock_ir_exit ===\n");
}

module_init(dock_ir_init);
module_exit(dock_ir_exit);
