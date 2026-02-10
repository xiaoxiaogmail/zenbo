/*
 * drivers/hwmon/cy8c_capsense_core.c - capacitive touch sensor.
 *
 * Device Driver for cypress of PSoC family CY8C4014SXI-420
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c/cy8c_capsense.h>

#ifdef CONFIG_ACPI
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#endif
#include <linux/spinlock.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>

/*
 * Debug Utility
 */
#define CAP_DEBUG(fmt, arg...)  \
	pr_debug("CY8C4014SXI==: [%s] " fmt , __func__ , ##arg)
#define CAP_INFO(fmt, arg...)   \
	pr_info("CY8C4014SXI==: [%s] " fmt , __func__ , ##arg)
#define CAP_ERROR(fmt, arg...)  \
	pr_err("CY8C4014SXI==: [%s] " fmt , __func__ , ##arg)

#define MAX_BUFFER_SIZE         780
/*
 * Global Variable
 */
struct cy8c_capsense_dev {
	struct i2c_client *client;
	struct miscdevice cap_device;
};

static DEFINE_MUTEX(cap_mutex);

/*
 * Function Declaration
 */
static int cy8c_capsense_probe(struct i2c_client *client,
			       const struct i2c_device_id *id);
static int cy8c_capsense_remove(struct i2c_client *client);
static int cy8c_capsense_init(void);
static void cy8c_capsense_exit(void);

/*
 * I2C and ACPI Driver Structure
 */
static struct i2c_device_id cy8c_capsense_id[] = {
	{"cap", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, cy8c_capsense_id);

#ifdef CONFIG_ACPI
static struct acpi_device_id cy8c_capsense_acpi_match[] = {
	{"CYPS4421", 0},
	{"", 0},
};

MODULE_DEVICE_TABLE(acpi, cy8c_capsense_acpi_match);
#endif

static ssize_t cap_dev_read(struct file *filp, char __user *buf,
			    size_t count, loff_t *offset)
{
	struct cy8c_capsense_dev *cap_dev = filp->private_data;
	unsigned char tmp[MAX_BUFFER_SIZE];
	int size;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;
	mutex_lock(&cap_mutex);
	size = i2c_master_recv(cap_dev->client, tmp, count);
	mutex_unlock(&cap_mutex);
	if (copy_to_user(buf, tmp, size))
		CAP_ERROR("failed to copy to user space\n");
	return size;
}

static ssize_t cap_dev_write(struct file *filp, const char __user *buf,
			     size_t count, loff_t *offset)
{
	struct cy8c_capsense_dev *cap_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;
	if (count > MAX_BUFFER_SIZE) {
		CAP_ERROR("Out of memory!\n");
		return -ENOMEM;
	}

	if (copy_from_user(tmp, buf, count)) {
		CAP_ERROR("failed to copy from user space!\n");
		return -EFAULT;
	}
	mutex_lock(&cap_mutex);
	ret = i2c_master_send(cap_dev->client, tmp, count);
	if (ret != count) {
		ret = -EIO;
		CAP_ERROR("failed to write!\n");
	}
	mutex_unlock(&cap_mutex);
	return ret;
}

static int cap_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct cy8c_capsense_dev *cap_dev = container_of(filp->private_data,
							 struct
							 cy8c_capsense_dev,
							 cap_device);

	filp->private_data = cap_dev;
	CAP_INFO("%s : %d.%d\n", __func__, imajor(inode), iminor(inode));
	return ret;
}

static const struct file_operations cap_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = cap_dev_read,
	.write = cap_dev_write,
	.open = cap_dev_open,
};

static struct i2c_driver cy8c_capsense_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = cy8c_capsense_probe,
	.remove = cy8c_capsense_remove,
	.id_table = cy8c_capsense_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "cap",
#ifdef CONFIG_ACPI
		   .acpi_match_table = ACPI_PTR(cy8c_capsense_acpi_match),
#endif
		   },
};

static int cy8c_capsense_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct cy8c_capsense_dev *cy8c_capsense_dev;
	int ret;
	CAP_INFO("Start to probe!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		return -ENODEV;
	}

	cy8c_capsense_dev = kzalloc(sizeof(*cy8c_capsense_dev), GFP_KERNEL);
	if (!cy8c_capsense_dev) {
		CAP_ERROR("kzalloc failed!\n");
		return -ENOMEM;
	}

	/* Set cy8c_capsense_dev relative members */
	cy8c_capsense_dev->client = client;
	i2c_set_clientdata(client, cy8c_capsense_dev);
	cy8c_capsense_dev->client->flags = 0;

	cy8c_capsense_dev->cap_device.minor = MISC_DYNAMIC_MINOR;
	cy8c_capsense_dev->cap_device.name = "cap";
	cy8c_capsense_dev->cap_device.fops = &cap_dev_fops;
	ret = misc_register(&cy8c_capsense_dev->cap_device);
	if (ret) {
		CAP_ERROR("%s : misc_register failed\n", __FILE__);
		goto misc_register_failed;
	}
	return 0;

misc_register_failed:
	kfree(cy8c_capsense_dev);
	return ret;
}

static int cy8c_capsense_remove(struct i2c_client *client)
{
	struct cy8c_capsense_dev *cy8c_capsense_dev =
	    i2c_get_clientdata(client);

	kfree(cy8c_capsense_dev);
	return 0;
}

static int __init cy8c_capsense_init(void)
{
	return i2c_add_driver(&cy8c_capsense_driver);
}

static void __exit cy8c_capsense_exit(void)
{
	i2c_del_driver(&cy8c_capsense_driver);
}

module_init(cy8c_capsense_init);
module_exit(cy8c_capsense_exit);

MODULE_AUTHOR("ASUSTeK Computer Inc.");
MODULE_DESCRIPTION("Cypress capacitive  driver");
MODULE_LICENSE("GPL");
