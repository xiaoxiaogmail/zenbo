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

#ifndef __CHARGER_BQ30Z55_H_
#define __CHARGER_BQ30Z55_H_

#include <linux/types.h>
#include <linux/power_supply.h>
#include <linux/board_asustek.h>

struct bq30z55_device {
	int bat_limit_enable;
	int stop_wq;
	int upper_limit;
	int lower_limit;
	int ac_periodic;
	int bat_periodic;
	bool ship_mode;
	struct i2c_client	*client;
	struct workqueue_struct	*bq30z55_wq;
	struct delayed_work	periodic_update_work;
	struct notifier_block nb;
	struct dentry		*dent;
	struct power_supply	batt_psy;
	struct power_supply	ac_psy;
};

static enum power_supply_property pm_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property power_online_prop[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

struct debug_reg {
	char	*name;
	u8		reg;
	u16		subcmd;
};

bool check_battery_ship(void);
extern bool check_cos_mode(void);
extern int mcu_feedback_register_client(struct notifier_block *nb);
extern int mcu_feedback_unregister_client(struct notifier_block *nb);

#endif /* __CHARGER_BQ30Z55_H_ */
