/*
 * include/linux/cy8c_capsense.h - capacitive touch sensor.
 *
 * Device Driver for cypress of PSoC family CY8C4014SXI-420
 *
 * Copyright (C) 2015 ASUSTeK Computer Inc.
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

#ifndef _LINUX_I2C_CY8C_CAPSENSE_H
#define _LINUX_I2C_CY8C_CAPSENSE_H

#define GPIO_IRQ	290
#define GPIO_IRQ_NAME	"cap_sensor_irq"
#define CY8C4014SXI_DEVICE_ID	"cy8c4014sxi"
#define CY8C4014SXI_DRIVER_NAME	"cy8c-capsense-i2c"

/* cy8c4014sxi capsensor register definition */
#define CY8C4014SXI_REG_MAX	0x15
#define REG_KEYSTATUS		0x00
#define REG_FW_REVERSION	0x01
#define REG_ENTERBOOTLOADER	0x02	/* Default 0 (normal mode) */
#define REG_PRESSCOUNT		0x03	/* 0-255, default 0x96 (3-sec) */
#define REG_MID_PRESS_TIME	0x04	/* 0-255, default 0xC8 (1-sec) */
#define REG_IGNORE_TIME		0x05	/* 0-255, default 0x46 (0.35-sec) */
#define REG_INTERRUPT_MODE	0x06	/* 0: delay 10 ms, 1: wait I2C read complete */
#define REG_SENSOR_CP		0x07
#define REG_SENSOR1_CP		0x08	/* Supported range of Parasitic Capacitance (Cp) */
#define REG_SENSOR2_CP		0x09	/* Supported range of Parasitic Capacitance (Cp) */
#define REG_SENSOR3_CP		0x0A	/* Supported range of Parasitic Capacitance (Cp) */
#define REG_DEBUG_MODE		0x0B	/* 0x01: Diff Count, 0x02: Baseline, 0x03: Raw Data */
#define REG_BTN1_HB		0x0C
#define REG_BTN1_LB		0x0D
#define REG_BTN2_HB		0x0E
#define REG_BTN2_LB		0x0F
#define REG_BTN3_HB		0x10
#define REG_BTN3_LB		0x11
#define REG_PRESS_TIME_HB	0x12
#define REG_PRESS_TIME_LB	0x13
#define REG_EXTREME_TIME	0x14	/* 0-255, default 0x1 (0.005-sec) */

#endif	/* _LINUX_I2C_CY8C_CAPSENSE_H */
