/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __CHARGER_BQ24725A_H_
#define __CHARGER_BQ24725A_H_

#include <linux/types.h>
#include <linux/power_supply.h>

struct bq24725a {
	struct i2c_client	*client;
	char *name;
	uint32_t charge_current;
	uint32_t charge_voltage;
	uint32_t input_current;
};

int bq24725a_enable_charging(void);
int bq24725a_disable_charging(void);
int bq24725a_config_charger(
	uint32_t charge_current, uint32_t charge_voltage);
bool bq24725a_charger_is_present(void);
void bq24725a_charger_init(void);
void bq24725a_learn_enable(bool enable);

#endif /* __CHARGER_BQ24725A_H_ */
