/*
 *      uvc_debugfs.c --  USB Video Class driver - Debugging support
 *
 *      Copyright (C) 2011
 *          Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/usb.h>

#include "uvcvideo.h"

/* -----------------------------------------------------------------------------
 * Statistics
 */

#define UVC_DEBUGFS_BUF_SIZE	1024

static u32 g_uvc_device_num;

struct uvc_debugfs_buffer {
	size_t count;
	char data[UVC_DEBUGFS_BUF_SIZE];
};

static int uvc_debugfs_stats_open(struct inode *inode, struct file *file)
{
	struct uvc_streaming *stream = inode->i_private;
	struct uvc_debugfs_buffer *buf;

	buf = kmalloc(sizeof(*buf), GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	buf->count = uvc_video_stats_dump(stream, buf->data, sizeof(buf->data));

	file->private_data = buf;
	return 0;
}

static ssize_t uvc_debugfs_stats_read(struct file *file, char __user *user_buf,
				      size_t nbytes, loff_t *ppos)
{
	struct uvc_debugfs_buffer *buf = file->private_data;

	return simple_read_from_buffer(user_buf, nbytes, ppos, buf->data,
				       buf->count);
}

static int uvc_debugfs_stats_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	file->private_data = NULL;

	return 0;
}

static const struct file_operations uvc_debugfs_stats_fops = {
	.owner = THIS_MODULE,
	.open = uvc_debugfs_stats_open,
	.llseek = no_llseek,
	.read = uvc_debugfs_stats_read,
	.release = uvc_debugfs_stats_release,
};

/* -----------------------------------------------------------------------------
 * Global and stream initialization/cleanup
 */

static struct dentry *uvc_debugfs_root_dir;

int uvc_debugfs_init_stream(struct uvc_streaming *stream)
{
	struct usb_device *udev = stream->dev->udev;
	struct dentry *dent;
	char dir_name[32];

	if (uvc_debugfs_root_dir == NULL)
		return -ENODEV;

	sprintf(dir_name, "%u-%u", udev->bus->busnum, udev->devnum);

	dent = debugfs_create_dir(dir_name, uvc_debugfs_root_dir);
	if (IS_ERR_OR_NULL(dent)) {
		uvc_printk(KERN_INFO, "Unable to create debugfs %s "
			   "directory.\n", dir_name);
		return -ENODEV;
	}

	stream->debugfs_dir = dent;

	dent = debugfs_create_file("stats", 0444, stream->debugfs_dir,
				   stream, &uvc_debugfs_stats_fops);

	g_uvc_device_num++;

	if (IS_ERR_OR_NULL(dent)) {
		uvc_printk(KERN_INFO, "Unable to create debugfs stats file.\n");
		uvc_debugfs_cleanup_stream(stream);
		return -ENODEV;
	}

	return 0;
}

void uvc_debugfs_cleanup_stream(struct uvc_streaming *stream)
{
	if (stream->debugfs_dir == NULL)
		return;

	debugfs_remove_recursive(stream->debugfs_dir);
	stream->debugfs_dir = NULL;
	g_uvc_device_num--;
}

static int dbg_gpio_n7_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_gpio_n7_set_value(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	unsigned int gpio_value = 0;

	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%d", &gpio_value);

	gpio_value = (gpio_value == 0)?0:1;
	pr_info("%s: %d\n", __func__, gpio_value);
	gpiod_direction_output(gpio_to_desc(GPIO_R200_PWREN), gpio_value);

	return count;
}

static const struct file_operations dbg_gpio_n7_fops = {
	.open		= dbg_gpio_n7_open,
	.write		= dbg_gpio_n7_set_value,
};

int uvc_debugfs_init(void)
{
	struct dentry *dir;

	dir = debugfs_create_dir("uvcvideo", usb_debug_root);
	if (IS_ERR_OR_NULL(dir)) {
		uvc_printk(KERN_INFO, "Unable to create debugfs directory\n");
		return -ENODATA;
	}

	uvc_debugfs_root_dir = dir;

#if FACTORY_IMAGE
	(void) debugfs_create_file("gpio_n7", S_IRUGO  | S_IWUGO,
		uvc_debugfs_root_dir, NULL, &dbg_gpio_n7_fops);
#else
	(void) debugfs_create_file("gpio_n7", S_IRUGO,
		uvc_debugfs_root_dir, NULL, &dbg_gpio_n7_fops);
#endif

	debugfs_create_u32("device_num", 0666, uvc_debugfs_root_dir, &g_uvc_device_num);

	return 0;
}

void uvc_debugfs_cleanup(void)
{
	if (uvc_debugfs_root_dir != NULL)
		debugfs_remove_recursive(uvc_debugfs_root_dir);
}
