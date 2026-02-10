/* include/linux/board_asustek.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2012, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2012, LGE Inc.
 * Copyright (c) 2012-2014, ASUSTek Computer Inc.
 * Author: Paris Yeh <paris_yeh@asus.com>
 *	   Hank Lee  <hank_lee@asus.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_INTEL_MID_BOARD_ASUSTEK_H
#define __ASM_INTEL_MID_BOARD_ASUSTEK_H

#include <linux/types.h>

#ifndef CONFIG_SFI_PCB
#define CONFIG_SFI_PCB
#endif
typedef enum {
	HW_REV_INVALID = -1,
	HW_REV_MAX = 8
} hw_rev;

typedef enum {
	PROJECT_ID_INVALID = -1,
	PROJECT_ID_MAX = 16
} project_id;

typedef enum {
	LCD_TYPE_INVALID = -1,
	LCD_TYPE_MAX = 2
} lcd_type;

typedef enum {
	TP_TYPE_INVALID = -1,
	TP_TYPE_MAX = 2
} tp_type;

typedef enum {
	CAM_FRONT_INVALID = -1,
	CAM_FRONT_MAX = 2
} cam_front;

typedef enum {
	DDR_SKU_INVALID = -1,
	DDR_SKU_MAX = 2
} ddr_sku;

typedef enum {
	TYPEC_SKU_INVALID = -1,
	TYPEC_SKU_MAX = 2
} typec_sku;

struct asustek_pcbid_platform_data {
	const char *UUID;
	struct resource *resource0;
	u32 nr_resource0;
};

hw_rev asustek_get_hw_rev(void);

project_id asustek_get_project_id(void);

lcd_type asustek_get_lcd_type(void);

tp_type asustek_get_tp_type(void);

cam_front asustek_get_camera_front(void);

ddr_sku asustek_get_ddr_sku(void);

typec_sku asustek_get_typec_sku(void);

#endif /* __ASM_INTEL_MID_BOARD_ASUSTEK_H */
