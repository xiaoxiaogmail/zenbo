/*
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/board_asustek.h>

#define V_SENSOR_3V3	387
#define V_SENSOR_1V8	394
#define GPIO_SENSOR_NAME "gpio_sensor_device"

static int gpio_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	pr_info("gpio sensor hardware reset\n");
	ret = gpio_request(V_SENSOR_1V8, "V_SENSOR_1V8");
	if (ret) {
		pr_err("gpio %d request failed\n", V_SENSOR_1V8);
		return ret;
	}

	ret = gpio_direction_output(V_SENSOR_1V8, 0);
	if (ret) {
		pr_err("gpio %d unavaliable for output\n",
				V_SENSOR_1V8);
		goto err_free_gpio1;
	}

	msleep(20);

	ret = gpio_request(V_SENSOR_3V3, "V_SENSOR_3V3");
	if (ret) {
		pr_err("gpio %d request failed\n", V_SENSOR_3V3);
		goto err_free_gpio1;
	}

	ret = gpio_direction_output(V_SENSOR_3V3, 0);
	if (ret) {
		pr_err("gpio %d unavaliable for output\n",
				V_SENSOR_3V3);
		goto err_free_gpio2;
	}

	msleep(340);

	ret = gpio_direction_output(V_SENSOR_3V3, 1);
	if (ret) {
		pr_err("gpio %d unavaliable for output\n",
				V_SENSOR_3V3);
		goto err_free_gpio2;
	}

	msleep(20);

	ret = gpio_direction_output(V_SENSOR_1V8, 1);
	if (ret) {
		pr_err("gpio %d unavaliable for output\n",
				V_SENSOR_1V8);
		goto err_free_gpio2;
	}

	msleep(20);

	return 0;
err_free_gpio2:
	if (gpio_is_valid(V_SENSOR_3V3))
		gpio_free(V_SENSOR_3V3);
err_free_gpio1:
	if (gpio_is_valid(V_SENSOR_1V8))
		gpio_free(V_SENSOR_1V8);
	return ret;
}

static void gpio_sensor_shutdown(struct platform_device *pdev)
{
	int ret = 0;

	ret = gpio_direction_output(V_SENSOR_1V8, 0);
	if (ret) {
		pr_err("gpio %d unavaliable for output\n",
				V_SENSOR_1V8);
		goto err_free_gpio;
	}

	msleep(20);

	ret = gpio_direction_output(V_SENSOR_3V3, 0);
	if (ret) {
		pr_err("gpio %d unavaliable for output\n",
				V_SENSOR_3V3);
		goto err_free_gpio;
	}

err_free_gpio:
	if (gpio_is_valid(V_SENSOR_3V3))
		gpio_free(V_SENSOR_3V3);
	if (gpio_is_valid(V_SENSOR_1V8))
		gpio_free(V_SENSOR_1V8);
}

static struct resource gpio_sensor_resource[] = {

};

static struct platform_device gpio_sensor_device = {
	.name = GPIO_SENSOR_NAME,
	.id = -1,
	.resource = gpio_sensor_resource,
	.num_resources = ARRAY_SIZE(gpio_sensor_resource),
};

static struct platform_driver gpio_sensor_driver = {
	.probe = gpio_sensor_probe,
	.shutdown = gpio_sensor_shutdown,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = GPIO_SENSOR_NAME,
	},
};

static int __init gpio_sensor_init(void)
{
	int hw_id = 0;

	hw_id = asustek_get_hw_rev();

	if (hw_id == 5) {
		platform_device_register(&gpio_sensor_device);

		return platform_driver_register(&gpio_sensor_driver);
	}

	return 0;
}
device_initcall(gpio_sensor_init);

static void __exit gpio_sensor_exit(void)
{
	int hw_id = 0;

	hw_id = asustek_get_hw_rev();

	if (hw_id == 5) {
		if (gpio_is_valid(V_SENSOR_1V8))
			gpio_free(V_SENSOR_1V8);
		if (gpio_is_valid(V_SENSOR_3V3))
			gpio_free(V_SENSOR_3V3);
		platform_driver_unregister(&gpio_sensor_driver);
	}
}
module_exit(gpio_sensor_exit);

MODULE_AUTHOR("ASUSTeK Computer Inc.");
MODULE_DESCRIPTION("GPIO driver");
MODULE_LICENSE("GPL");
