/* Copyright (c) 2014, ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/board_asustek.h>

#ifdef CONFIG_ASUSTEK_PCBID
static char serialno[32] = {0,};
int __init asustek_androidboot_serialno(char *s)
{
	int n;

	if (*s == '=')
		s++;
	n = snprintf(serialno, sizeof(serialno), "%s", s);
	serialno[n] = '\0';

	return 1;
}
__setup("androidboot.serialno", asustek_androidboot_serialno);

static struct resource resources_asustek_pcbid[] = {
	{
		.start	= 389,
		.end	= 389,
		.name	= "PCB_ID0",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= 341,
		.end	= 341,
		.name	= "PCB_ID1",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= 391,
		.end	= 391,
		.name	= "PCB_ID2",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= 491,
		.end	= 491,
		.name	= "PCB_ID3",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= 307,
		.end	= 307,
		.name	= "PCB_ID4",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= 492,
		.end	= 492,
		.name	= "PCB_ID5",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= 407,
		.end	= 407,
		.name	= "PCB_ID6",
		.flags	= IORESOURCE_IO,
	},
};

/*
static struct resource pdata_resources0[] = {
	{
		.start	= 407,
		.end	= 407,
		.name	= "PCB_ID7",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= 411,
		.end	= 411,
		.name	= "PCB_ID8",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= 505,
		.end	= 505,
		.name	= "PCB_ID9",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= 387,
		.end	= 387,
		.name	= "PCB_ID10",
		.flags	= IORESOURCE_IO,
	},
	{
		.start	= 394,
		.end	= 394,
		.name	= "PCB_ID11",
		.flags	= IORESOURCE_IO,
	},
};
*/
struct asustek_pcbid_platform_data asustek_pcbid_pdata = {
	.UUID = serialno,
	//.resource0 = pdata_resources0,
	//.nr_resource0 = ARRAY_SIZE(pdata_resources0),
};

static struct platform_device asustek_pcbid_device = {
	.name		= "asustek_pcbid",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_asustek_pcbid),
	.resource = resources_asustek_pcbid,
	.dev = {
		.platform_data = &asustek_pcbid_pdata,
	}
};

static void __init asustek_add_pcbid_devices(void)
{
	printk("asustek_add_pcbid_devices+\n");
	platform_device_register(&asustek_pcbid_device);
	printk("asustek_add_pcbid_devices-\n");
}

rootfs_initcall(asustek_add_pcbid_devices);
#endif
