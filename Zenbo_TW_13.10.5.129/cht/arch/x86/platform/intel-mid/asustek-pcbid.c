/*
 * arch/x86/platform/intel-mid/asustek-pcbid.c
 *
 * Copyright (C) 2014 ASUSTek Inc.
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
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/board_asustek.h>

#define PCBID_VALUE_INVALID 0x4E2F4100 /* N/A */

enum {
	DEBUG_STATE = 1U << 0,
	DEBUG_VERBOSE = 1U << 1,
};

static int debug_mask = DEBUG_VERBOSE;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static unsigned int asustek_pcbid = PCBID_VALUE_INVALID;
static const char *asustek_chipid;
static unsigned int hw_rev_pcbid[] = {0, 1, 2};
static unsigned int project_id_pcbid[] = {3, 4, 5};
static unsigned int ddr_sku_pcbid[] = {6};
//static unsigned int lcd_type_pcbid[] = {8};
//static unsigned int tp_type_pcbid[] = {9};
//static unsigned int cam_front_pcbid[] = {10};
//static unsigned int typec_sku_pcbid[] = {11};

struct pcbid_maps {
	unsigned char name[16];
	unsigned int *pcbid;
	unsigned int pcbid_num;
} asustek_pcbid_maps[] = {
	{"HW_REV", hw_rev_pcbid, ARRAY_SIZE(hw_rev_pcbid)},
	{"PROJECT_ID", project_id_pcbid, ARRAY_SIZE(project_id_pcbid)},
	{"DDR_SKU", ddr_sku_pcbid, ARRAY_SIZE(ddr_sku_pcbid)},
	//{"LCD_TYPE", lcd_type_pcbid, ARRAY_SIZE(lcd_type_pcbid)},
	//{"TP_TYPE", tp_type_pcbid, ARRAY_SIZE(tp_type_pcbid)},
	//{"CAM_FRONT", cam_front_pcbid, ARRAY_SIZE(cam_front_pcbid)},
	//{"TYPEC_SKU", typec_sku_pcbid, ARRAY_SIZE(typec_sku_pcbid)},
};

#define NUM_MAPS (sizeof(asustek_pcbid_maps) / sizeof(asustek_pcbid_maps[0]))

int get_pcbid_type(const char *func)
{
	int i = 0, ret = 0;
	struct pcbid_maps *map = NULL;

	if (asustek_pcbid == PCBID_VALUE_INVALID) {
		pr_err("ASUSTek PCBID was invalid\n");
		return -ENODEV;
	}

	for (i = 0; i < NUM_MAPS; i++) {
		if (!strcmp(func, asustek_pcbid_maps[i].name)) {
			//if (debug_mask & DEBUG_VERBOSE)
				//pr_info("%s was found\n", func);

			map = &asustek_pcbid_maps[i];
			break;
		}
	}

	if (map) {
		/* found */
		for (i = 0; i < map->pcbid_num; i++) {
				ret += asustek_pcbid & BIT(map->pcbid[i]);
		}
		ret = ret >> map->pcbid[0];
	} else
		ret = -ENODEV;

	return ret;
}

hw_rev asustek_get_hw_rev(void)
{
	hw_rev ret = HW_REV_INVALID;

	ret = get_pcbid_type("HW_REV");

	//if (debug_mask & DEBUG_VERBOSE)
		//pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= HW_REV_MAX))
		ret = HW_REV_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_hw_rev);

project_id asustek_get_project_id(void)
{
	project_id ret = PROJECT_ID_INVALID;

	ret = get_pcbid_type("PROJECT_ID");

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= PROJECT_ID_MAX))
		ret = PROJECT_ID_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_project_id);

ddr_sku asustek_get_ddr_sku(void)
{
	ddr_sku ret = DDR_SKU_INVALID;

	ret = get_pcbid_type("DDR_SKU");

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= DDR_SKU_MAX))
		ret = DDR_SKU_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_ddr_sku);

lcd_type asustek_get_lcd_type(void)
{
	lcd_type ret = LCD_TYPE_INVALID;

    /*
	ret = get_pcbid_type("LCD_TYPE");

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= LCD_TYPE_MAX))
		ret = LCD_TYPE_INVALID;
    */
	return ret;
}
EXPORT_SYMBOL(asustek_get_lcd_type);

tp_type asustek_get_tp_type(void)
{
	tp_type ret = TP_TYPE_INVALID;
    /*
	ret = get_pcbid_type("TP_TYPE");

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= TP_TYPE_MAX))
		ret = TP_TYPE_INVALID;
    */
	return ret;
}
EXPORT_SYMBOL(asustek_get_tp_type);

cam_front asustek_get_camera_front(void)
{
	cam_front ret = CAM_FRONT_INVALID;
    /*
	ret = get_pcbid_type("CAM_FRONT");

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= CAM_FRONT_MAX))
		ret = CAM_FRONT_INVALID;
    */
	return ret;
}
EXPORT_SYMBOL(asustek_get_camera_front);

typec_sku asustek_get_typec_sku(void)
{
	typec_sku ret = TYPEC_SKU_INVALID;
    /*
	ret = get_pcbid_type("TYPEC_SKU");

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= TYPEC_SKU_MAX))
		ret = TYPEC_SKU_INVALID;
    */
	return ret;
}
EXPORT_SYMBOL(asustek_get_typec_sku);

#define ASUSTEK_PCBID_ATTR(module) \
static struct kobj_attribute module##_attr = { \
	.attr = { \
		.name = __stringify(module), \
		.mode = 0444, \
	}, \
	.show = module##_show, \
}

static ssize_t asustek_pcbid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "%04x\n", asustek_pcbid);

	return s - buf;
}

static ssize_t asustek_projectid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "%02x\n", (asustek_pcbid >> 3) & 0x7);

	return s - buf;
}

static ssize_t asustek_hardwareid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "%x\n", asustek_pcbid & 0x7);

	return s - buf;
}

static ssize_t asustek_chipid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "%s\n", asustek_chipid);

	return s - buf;
}

static ssize_t asustek_ddrid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(s, "%x\n", asustek_get_ddr_sku());

	return s - buf;
}

ASUSTEK_PCBID_ATTR(asustek_pcbid);
ASUSTEK_PCBID_ATTR(asustek_projectid);
ASUSTEK_PCBID_ATTR(asustek_hardwareid);
ASUSTEK_PCBID_ATTR(asustek_chipid);
ASUSTEK_PCBID_ATTR(asustek_ddrid);

static struct attribute *attr_list[] = {
	&asustek_pcbid_attr.attr,
	&asustek_projectid_attr.attr,
	&asustek_hardwareid_attr.attr,
	&asustek_chipid_attr.attr,
	&asustek_ddrid_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attr_list,
};

static int __init pcbid_driver_probe(struct platform_device *pdev)
{
	int i, j, ret = 0;
	struct resource *res;
	struct asustek_pcbid_platform_data *pdata;
	unsigned int value;
	int gpio;
	//struct resource *pdata_res;
	int nr_pdata_res;

	printk("pcbid_driver_probe+\n");

	if (!pdev)
		return -EINVAL;

	pdata = pdev->dev.platform_data;

	if (pdata)
		asustek_chipid = kstrdup(pdata->UUID, GFP_KERNEL);

	asustek_pcbid = 0;

	//manipulate pcbid of project & revision
	for (i = 0; i < pdev->num_resources; i++) {
		res = platform_get_resource(pdev, IORESOURCE_IO, i);
		if (!res)
			return -ENODEV;

		gpio = res->start;

		if (debug_mask & DEBUG_VERBOSE)
			pr_info("ASUSTek: Requesting gpio%d\n", gpio);

		ret = gpio_request(gpio, res->name);
		if (ret) {
			/* indicate invalid pcbid value when error happens */
			pr_err("ASUSTek: Failed to request gpio%d\n", gpio);
			asustek_pcbid = PCBID_VALUE_INVALID;
			res = NULL;
			break;
		}

		ret = gpio_direction_input(gpio);
		if (ret) {
			/* indicate invalid pcbid value when error happens */
			pr_err("ASUSTek: Failed to configure direction for gpio%d\n",
					gpio);
			asustek_pcbid = PCBID_VALUE_INVALID;
			res = NULL;
			break;
		}

		/* read input value through gpio library directly */
		value = gpio_get_value(gpio) ? 1 : 0;
		if (debug_mask & DEBUG_VERBOSE)
			pr_info("ASUSTek: Input value of gpio%d is %s\n", gpio,
					value ? "high" : "low");

		asustek_pcbid |= value << i;
	}


    /*
	//manipulate rest of pcbid
	pdata_res = pdata->resource0;
	nr_pdata_res = pdata->nr_resource0;

	if (!pdata_res)
		return -ENODEV;

	for (j = 0; j < nr_pdata_res; j++) {
		gpio = (&pdata_res[j])->start;

		if (debug_mask & DEBUG_VERBOSE)
			pr_info("ASUSTek: Requesting gpio%d\n", gpio);

		ret = gpio_request(gpio, (&pdata_res[j])->name);
		if (ret) {
			//indicate invalid pcbid value when error happens
			pr_err("ASUSTek: Failed to request gpio%d\n", gpio);
			asustek_pcbid = PCBID_VALUE_INVALID;
			break;
		}

		ret = gpio_direction_input(gpio);
		if (ret) {
			//indicate invalid pcbid value when error happens
			pr_err("ASUSTek: Failed to configure direction for gpio%d\n",
					gpio);
			asustek_pcbid = PCBID_VALUE_INVALID;
			break;
		}

		// read input value through gpio library directly
		value = gpio_get_value(gpio) ? 1 : 0;
		if (debug_mask & DEBUG_VERBOSE)
			pr_info("ASUSTek: Input value of gpio%d is %s\n", gpio,
					value ? "high" : "low");

		asustek_pcbid |= value << (j + 7);
	}
    */

	if (asustek_pcbid == PCBID_VALUE_INVALID) {

		/* error handler to free allocated gpio resources */
		while (i >= 0) {
			res = platform_get_resource(pdev, IORESOURCE_IO, i);
			if (!res)
				return -ENODEV;

			if (debug_mask & DEBUG_VERBOSE)
				pr_info("ASUSTek: Freeing gpio%d\n", gpio);

			gpio_free(gpio);
			i--;
		}
        /*
		while (j >= 0) {
			gpio = (&pdata_res[j])->start;

			if (debug_mask & DEBUG_VERBOSE)
				pr_info("ASUSTek: Freeing gpio%d\n", gpio);

			gpio_free(gpio);
			j--;
		}
        */
	} else {
		/* report pcbid info to dmesg */
		if (debug_mask && DEBUG_STATE)
			pr_info("ASUSTek: PCBID=%04x\n", asustek_pcbid);

		/* create a sysfs interface */
		ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);

		if (ret)
			pr_err("ASUSTek: Failed to create sysfs group\n");
	}

	printk("pcbid_driver_probe-\n");

	return ret;
}

static int pcbid_driver_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver asustek_pcbid_driver __refdata = {
	.probe = pcbid_driver_probe,
	.remove = pcbid_driver_remove,
	.driver = {
		.name = "asustek_pcbid",
		.owner = THIS_MODULE,
	},
};

static int asustek_pcbid_init(void)
{
	printk("asustek_pcbid_init\n");
	return platform_driver_register(&asustek_pcbid_driver);
}

rootfs_initcall(asustek_pcbid_init);

MODULE_DESCRIPTION("ASUSTek PCBID driver");
MODULE_AUTHOR("Paris Yeh <paris_yeh@asus.com>");
MODULE_LICENSE("GPL");
