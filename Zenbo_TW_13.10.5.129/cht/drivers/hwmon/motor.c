/*
 * drivers/hwmon/motor.c - motor sensor.
 *
 * Device Driver for motor sensor
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

struct motor_dev {
	struct input_dev *dev;
	struct workqueue_struct *motor_wq;
	struct delayed_work work;
	struct notifier_block nb;
	struct attribute_group attrs;
	struct kobject *motor_control_kobj;
	s16 neck_yaw_current;
	s16 neck_pitch_current;
	s16 wheel_left_current;
	s16 wheel_right_current;
	s16 neck_yaw_pwm;
	s16 neck_pitch_pwm;
	s16 wheel_left_pwm;
	s16 wheel_right_pwm;
	s32 timestamp;
};

static s16 neck_yaw_current_sys;
static s16 neck_pitch_current_sys;
static s16 wheel_left_current_sys;
static s16 wheel_right_current_sys;
static s16 neck_yaw_pwm_sys;
static s16 neck_pitch_pwm_sys;
static s16 wheel_left_pwm_sys;
static s16 wheel_right_pwm_sys;

static s16 neck_yaw_current_before;
static s16 neck_pitch_current_before;
static s16 wheel_left_current_before;
static s16 wheel_right_current_before;
static s16 neck_yaw_pwm_before;
static s16 neck_pitch_pwm_before;
static s16 wheel_left_pwm_before;
static s16 wheel_right_pwm_before;

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
	return sprintf(buf, "%hd %hd %hd %hd %hd %hd %hd %hd\n", neck_yaw_current_sys,
			neck_pitch_current_sys, wheel_left_current_sys,
			wheel_right_current_sys, neck_yaw_pwm_sys,
			neck_pitch_pwm_sys, wheel_left_pwm_sys,
			wheel_right_pwm_sys);
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

static struct attribute *motor_attrs[] = {
	&dev_attr_timestamp_info.attr,
	&dev_attr_data_info.attr,
	&dev_attr_mode.attr,
	NULL
};

static void motor_work_function(struct work_struct *work)
{
	struct motor_dev *motor_dev =
	    container_of((struct delayed_work *)work, struct motor_dev,
			 work);
	neck_yaw_current_sys = motor_dev->neck_yaw_current;
	neck_pitch_current_sys = motor_dev->neck_pitch_current;
	wheel_left_current_sys = motor_dev->wheel_left_current;
	wheel_right_current_sys = motor_dev->wheel_right_current;

	neck_yaw_pwm_sys = motor_dev->neck_yaw_pwm;
	neck_pitch_pwm_sys = motor_dev->neck_pitch_pwm;
	wheel_left_pwm_sys = motor_dev->wheel_left_pwm;
	wheel_right_pwm_sys = motor_dev->wheel_right_pwm;

	if (!set_mode_first_time &&
			neck_yaw_current_sys == neck_yaw_current_before &&
			neck_pitch_current_sys == neck_pitch_current_before &&
			wheel_left_current_sys == wheel_left_current_before &&
			wheel_right_current_sys == wheel_right_current_before &&
			neck_yaw_pwm_sys == neck_yaw_pwm_before &&
			neck_pitch_pwm_sys == neck_pitch_pwm_before &&
			wheel_left_pwm_sys == wheel_left_pwm_before &&
			wheel_right_pwm_sys == wheel_right_pwm_before) {
		return;
	} else {
		neck_yaw_current_before = neck_yaw_current_sys;
		neck_pitch_current_before = neck_pitch_current_sys;
		wheel_left_current_before = wheel_left_current_sys;
		wheel_right_current_before = wheel_right_current_sys;

		neck_yaw_pwm_before = neck_yaw_pwm_sys;
		neck_pitch_pwm_before = neck_pitch_pwm_sys;
		wheel_left_pwm_before = wheel_left_pwm_sys;
		wheel_right_pwm_before = wheel_right_pwm_sys;
	}

	input_report_abs(motor_dev->dev, 1,
			 motor_dev->neck_yaw_current);
	input_report_abs(motor_dev->dev, 2,
			 motor_dev->neck_pitch_current);
	input_report_abs(motor_dev->dev, 3,
			 motor_dev->wheel_left_current);
	input_report_abs(motor_dev->dev, 4,
			 motor_dev->wheel_right_current);
	input_report_abs(motor_dev->dev, 5,
			 motor_dev->neck_yaw_pwm);
	input_report_abs(motor_dev->dev, 6,
			 motor_dev->neck_pitch_pwm);
	input_report_abs(motor_dev->dev, 7,
			 motor_dev->wheel_left_pwm);
	input_report_abs(motor_dev->dev, 8,
			 motor_dev->wheel_right_pwm);
	wheel_right_pwm_sys = motor_dev->wheel_right_pwm;
	input_report_abs(motor_dev->dev, ABS_TIMESTAMP,
			 motor_dev->timestamp);
	timestamp_sys = motor_dev->timestamp;
	input_sync(motor_dev->dev);
	if (set_mode_first_time)
		set_mode_first_time = 0;
}

static int mcu_feedback_handler(struct notifier_block *self, u8 header,
				struct payload *sub_payload)
{
	struct motor_dev *motor_dev =
	    container_of(self, struct motor_dev, nb);

	if (sensor_mode == 0)
		return 0;

	if (header == 1) {
		motor_dev->timestamp = (s32)sub_payload->timestamp;
	} else if (header == 0xE && sub_payload->dataLength == 8) {
		motor_dev->neck_yaw_current = (sub_payload->data[0] + (sub_payload->data[1] << 8));
		motor_dev->neck_pitch_current = (sub_payload->data[2] + (sub_payload->data[3] << 8));
		motor_dev->wheel_left_current = (sub_payload->data[4] + (sub_payload->data[5] << 8));
		motor_dev->wheel_right_current = (sub_payload->data[6] + (sub_payload->data[7] << 8));
	} else if (header == 0x32 && sub_payload->dataLength == 8) {
		motor_dev->neck_yaw_pwm = (sub_payload->data[0] + (sub_payload->data[1] << 8));
		motor_dev->neck_pitch_pwm = (sub_payload->data[2] + (sub_payload->data[3] << 8));
		motor_dev->wheel_left_pwm = (sub_payload->data[4] + (sub_payload->data[5] << 8));
		motor_dev->wheel_right_pwm = (sub_payload->data[6] + (sub_payload->data[7] << 8));
	} else if (header == 0xFF && sub_payload->dataLength == 0)
		queue_delayed_work(motor_dev->motor_wq, &motor_dev->work, 0);

	return 0;
}

static int motor_init(void)
{
	int ret;
	struct motor_dev *motor_dev;
	sensor_mode = 0;
	set_mode_first_time = 0;

	pr_info("=== motor_init ===\n");
	motor_dev = kzalloc(sizeof(*motor_dev), GFP_KERNEL);
	if (!motor_dev) {
		pr_err("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_kzalloc_failed;
	}

	motor_dev->motor_wq =
	    create_singlethread_workqueue("motor_wq");
	if (!motor_dev->motor_wq) {
		pr_err("create_singlethread_workqueue failed!\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}

	INIT_DELAYED_WORK(&motor_dev->work, motor_work_function);
	motor_dev->dev = input_allocate_device();

	if (!motor_dev->dev) {
		ret = -ENOMEM;
		goto err_allocate_input_device;
	}

	motor_dev->dev->name = "motor";
	motor_dev->dev->id.bustype = BUS_I2C;
	input_set_capability(motor_dev->dev, EV_ABS, ABS_MISC);
	input_set_abs_params(motor_dev->dev, 1, -32768, 32767, 0, 0);
	input_set_abs_params(motor_dev->dev, 2, -32768, 32767, 0, 0);
	input_set_abs_params(motor_dev->dev, 3, -32768, 32767, 0, 0);
	input_set_abs_params(motor_dev->dev, 4, -32768, 32767, 0, 0);
	input_set_abs_params(motor_dev->dev, 5, -3600, 3600, 0, 0);
	input_set_abs_params(motor_dev->dev, 6, -3600, 3600, 0, 0);
	input_set_abs_params(motor_dev->dev, 7, -3600, 3600, 0, 0);
	input_set_abs_params(motor_dev->dev, 8, -3600, 3600, 0, 0);
	input_set_abs_params(motor_dev->dev, ABS_TIMESTAMP, -2147483648, 2147483647, 0, 0);
	ret = input_register_device(motor_dev->dev);
	if (ret < 0) {
		input_free_device(motor_dev->dev);
		goto err_allocate_input_device;
	}

	motor_dev->neck_yaw_current = 0;
	motor_dev->neck_pitch_current = 0;
	motor_dev->wheel_left_current = 0;
	motor_dev->wheel_right_current = 0;
	motor_dev->neck_yaw_pwm = 0;
	motor_dev->neck_pitch_pwm = 0;
	motor_dev->wheel_left_pwm = 0;
	motor_dev->wheel_right_pwm = 0;
	motor_dev->timestamp = 0;

	neck_yaw_current_sys = 0;
	neck_pitch_current_sys = 0;
	wheel_left_current_sys = 0;
	wheel_right_current_sys = 0;
	neck_yaw_pwm_sys = 0;
	neck_pitch_pwm_sys = 0;
	wheel_left_pwm_sys = 0;
	wheel_right_pwm_sys = 0;
	timestamp_sys = 0;

        neck_yaw_current_before = 0;
	neck_pitch_current_before = 0;
	wheel_left_current_before = 0;
	wheel_right_current_before = 0;
	neck_yaw_pwm_before = 0;
	neck_pitch_pwm_before = 0;
	wheel_left_pwm_before = 0;
	wheel_right_pwm_before = 0;

	motor_dev->nb.notifier_call = mcu_feedback_handler;
	mcu_feedback_register_client(&motor_dev->nb);

	motor_dev->motor_control_kobj =
	    kobject_create_and_add("motor", NULL);
	motor_dev->attrs.attrs = motor_attrs;
	if (motor_dev->motor_control_kobj == NULL) {
		pr_err("%s: subsystem_register failed\n", __func__);
		goto err_allocate_input_device;
	}

	ret =
	    sysfs_create_group(motor_dev->motor_control_kobj,
			       &motor_dev->attrs);

	if (ret) {
		pr_err("%s: sysfs_create_group: wheel_attrs failed\n",
		       __func__);
		goto err_allocate_input_device;
	}

	return 0;
err_allocate_input_device:
	destroy_workqueue(motor_dev->motor_wq);
err_create_singlethread_workqueue_failed:
	kfree(motor_dev);
err_kzalloc_failed:
	return ret;
}

static void motor_exit(void)
{
	pr_info("=== motor_exit ===\n");
}

module_init(motor_init);
module_exit(motor_exit);
