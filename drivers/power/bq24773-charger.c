/*
 * bq2477x-charger.c -- BQ24775 Charger driver
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.	 All rights reserved.
 *
 * Author: Andy Park <andyp@nvidia.com>
 * Author: Syed Rafiuddin <srafiuddin@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power/bq24773-charger.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/alarmtimer.h>
#include <linux/sched/rt.h>
#include <linux/ina3221.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/power/battery-charger-gauge-comm.h>
#include <linux/power/power_supply_extcon.h>
#include <linux/extcon.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>

#include <linux/platform/tegra/common.h>
#include <../mach-tegra/gpio-names.h>

#define MAX_STR_PRINT 50

#define USB_PROFILE	0
#define DOCK_PROFILE	1

static char *chargername[2] = {"USB", "DOCK"};

#define driver_name "bq2477x"

#define BQ2477X_DEBUG_MESSAGES 1

struct power_supply_extcon {
	struct device				*dev;
	struct extcon_dev			*edev;
	struct power_supply			ac;
	struct power_supply			usb;
	uint8_t					dock_online;
	struct bq2477x_chip			*bq2477x;
};

struct power_supply_cables {
	const char *name;
	long int event;
	struct power_supply_extcon	*psy_extcon;
	struct notifier_block nb;
	struct extcon_specific_cable_nb *extcon_dev;
	struct delayed_work extcon_notifier_work;
};


static struct power_supply_cables dock_psy_cables[] = {
	{
		.name	= "Dock-12V",
	},
};

struct battery_status {
	int battery_charge_voltage_mV;
	int battery_charge_current_mA;
	int pre_thermal_zone;
};

struct bq2477x_chip {
	struct device	*dev;
	struct power_supply	ac;
	struct power_supply	usb;
	struct regmap	*regmap;
	struct regmap	*regmap_word;
	struct mutex	mutex;
	int	disable_vbus_12v_boost_gpio;
	int	dac_v;
	int	dac_minsv;
	int	suspended;
	int	wdt_refresh_timeout;
	struct battery_charger_dev	*bc_dev;
	struct wake_lock dock_wake_lock;
	struct wake_lock usb_wake_lock;
	int	force_no_wake_lock;
	int	chg_status;
	int charger_type_is_usb;
	int charger_type_is_dock;
	char *extcon_dock_name;
	struct power_supply_extcon *psy_dock_extcon;
	struct regulator *vbus_reg;	/* regulator for drawing VBUS */
	int max_charge_ua;
	int dock_max_ua;
	int usb_charge_ua;
	struct notifier_block vbus_nb;
	struct delayed_work usb_work;
	struct delayed_work battery_work;
	struct delayed_work acok_work;
	struct power_supply *pwr_battery;
	const struct bq2477x_charge_zone *charge_table;
	int manual_temp_scaling;
	int acok_gpio;
	struct battery_status batt_status;
};

/* Kthread scheduling parameters */
struct sched_param bq2477x_param = {
	.sched_priority = MAX_RT_PRIO - 1,
};

static const struct regmap_config bq2477x_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= BQ2477X_MAX_REGS,
};

static const struct regmap_config bq2477x_regmap_word_config = {
	.reg_bits	= 8,
	.val_bits	= 16,
	.max_register	= BQ2477X_MAX_REGS,
};

static void bq2477x_plug_in_charging_disable(struct bq2477x_chip *bq2477x);
static void bq2477x_plug_in_charging_enable_vbus(struct bq2477x_chip *bq2477x);
static int bq2477x_plug_in_charging_enable(struct bq2477x_chip *bq2477x,
		int chargertype);
static int bq2477x_stop_charging(struct bq2477x_chip *bq2477x);

#define CHARGER_DETECTION_DEFAULT_DEBOUNCE_TIME_MS		50
#define BATTERY_CHECK_TIME					(30*HZ)

static void bq2477x_usb_work(struct work_struct *w)
{
	struct bq2477x_chip *bq2477x = container_of(to_delayed_work(w),
			struct bq2477x_chip, usb_work);

	mutex_lock(&bq2477x->mutex);
	if (!bq2477x->usb_charge_ua) {
		dev_info(bq2477x->dev, "USB-2 cable removed\n");
		bq2477x->charger_type_is_usb = 0;
		if (!bq2477x->force_no_wake_lock)
			wake_unlock(&bq2477x->usb_wake_lock);
		bq2477x_plug_in_charging_disable(bq2477x);
	} else {
		dev_info(bq2477x->dev, "USB-2 cable connected: max_uA = %d\n",
			bq2477x->usb_charge_ua);
		bq2477x->charger_type_is_usb = 1;
		if (!bq2477x->force_no_wake_lock)
			wake_lock(&bq2477x->usb_wake_lock);
		bq2477x_plug_in_charging_enable_vbus(bq2477x);
	}
	mutex_unlock(&bq2477x->mutex);
}

static int bq2477x_vbus_changed(struct notifier_block *nb,
					unsigned long event, void *v)
{
	struct bq2477x_chip *bq2477x = container_of(nb,
				struct bq2477x_chip, vbus_nb);

	bq2477x->usb_charge_ua = (int) v;

	cancel_delayed_work(&bq2477x->usb_work);
	schedule_delayed_work(&bq2477x->usb_work, msecs_to_jiffies(50));
	return NOTIFY_DONE;
}

static int power_supply_extcon_remove_dock_cable(
		struct power_supply_extcon *psy_extcon,
		struct extcon_dev *edev)
{
	struct bq2477x_chip *bq2477x = psy_extcon->bq2477x;

	if (!extcon_get_cable_state(edev, "Dock-12V")) {
		dev_info(bq2477x->dev, "12V Charging Dock disconnected\n");
		psy_extcon->dock_online = 0;
		bq2477x->charger_type_is_dock = 0;
		if (!bq2477x->force_no_wake_lock)
			wake_unlock(&bq2477x->dock_wake_lock);
		if (bq2477x->charger_type_is_usb)
			bq2477x_plug_in_charging_enable(bq2477x, USB_PROFILE);
		else
			bq2477x_plug_in_charging_disable(bq2477x);
		/* Re-enable boost after charger has been reconfigured. */
		gpio_direction_output(bq2477x->disable_vbus_12v_boost_gpio, 0);
	} else {
		dev_info(psy_extcon->dev, "%s: Unknown cable detected\n",
			__func__);
	}
	return 0;
}

static int power_supply_extcon_attach_dock_cable(
		struct power_supply_extcon *psy_extcon,
		struct extcon_dev *edev)
{
	struct bq2477x_chip *bq2477x = psy_extcon->bq2477x;

	if (true == extcon_get_cable_state(edev, "Dock-12V")) {
		bq2477x_plug_in_charging_enable(bq2477x, DOCK_PROFILE);
		bq2477x->charger_type_is_dock = 1;
		psy_extcon->dock_online = 1;
		if (!bq2477x->force_no_wake_lock)
			wake_lock(&bq2477x->dock_wake_lock);
		dev_info(psy_extcon->dev,
			"12V Charging Dock connected\n");
	} else {
		dev_info(psy_extcon->dev, "%s: Unknown cable detected\n",
				__func__);
	}
	return 0;
}

static void bq2477x_extcon_dock_handle_notifier(struct work_struct *w)
{
	struct power_supply_cables *cable = container_of(to_delayed_work(w),
			struct power_supply_cables, extcon_notifier_work);
	struct power_supply_extcon *psy_extcon = cable->psy_extcon;
	struct extcon_dev *edev = cable->extcon_dev->edev;
	struct bq2477x_chip *bq2477x = psy_extcon->bq2477x;

	mutex_lock(&bq2477x->mutex);
	if (cable->event == 0)
		power_supply_extcon_remove_dock_cable(psy_extcon, edev);
	else if (cable->event == 1)
		power_supply_extcon_attach_dock_cable(psy_extcon, edev);
	mutex_unlock(&bq2477x->mutex);
}

static int bq2477x_extcon_dock_notifier(struct notifier_block *self,
		unsigned long event, void *ptr)
{
	struct power_supply_cables *cable = container_of(self,
		struct power_supply_cables, nb);

	cable->event = event;
	cancel_delayed_work(&cable->extcon_notifier_work);
	schedule_delayed_work(&cable->extcon_notifier_work,
		msecs_to_jiffies(CHARGER_DETECTION_DEFAULT_DEBOUNCE_TIME_MS));

	return NOTIFY_DONE;
}


static int bq22477x_dock_extcon_probe(struct bq2477x_chip *bq2477x)
{
	int ret = 0;
	uint8_t j;
	struct power_supply_extcon *psy_extcon;

	psy_extcon =
		devm_kzalloc(bq2477x->dev, sizeof(*psy_extcon), GFP_KERNEL);
	if (!psy_extcon) {
		dev_err(bq2477x->dev, "failed to allocate memory status\n");
		return -ENOMEM;
	}
	bq2477x->psy_dock_extcon = psy_extcon;
	psy_extcon->bq2477x = bq2477x;
	psy_extcon->dev = bq2477x->dev;

	for (j = 0; j < ARRAY_SIZE(dock_psy_cables); j++) {
		struct power_supply_cables *cable = &dock_psy_cables[j];

		cable->extcon_dev = devm_kzalloc(bq2477x->dev,
				sizeof(struct extcon_specific_cable_nb),
				GFP_KERNEL);
		if (!cable->extcon_dev) {
			dev_err(bq2477x->dev, "Malloc for extcon_dev failed\n");
			goto econ_err;
		}

		INIT_DELAYED_WORK(&cable->extcon_notifier_work,
					bq2477x_extcon_dock_handle_notifier);

		cable->psy_extcon = psy_extcon;
		cable->nb.notifier_call = bq2477x_extcon_dock_notifier;

		ret = extcon_register_interest(cable->extcon_dev,
				bq2477x->extcon_dock_name,
				cable->name, &cable->nb);
		if (ret < 0)
			dev_err(psy_extcon->dev, "Cannot register for cable: %s\n",
					cable->name);
	}

	psy_extcon->edev = extcon_get_extcon_dev(bq2477x->extcon_dock_name);
	if (!psy_extcon->edev) {
		dev_err(bq2477x->dev, "failed extcon_get_extcon_dev(%s)\n",
			bq2477x->extcon_dock_name);
		goto econ_err;
	}

	dev_info(bq2477x->dev, "%s() get success\n", __func__);
	return 0;

econ_err:
	return ret;
}

static int bq2477x_read(struct bq2477x_chip *bq2477x,
			unsigned int reg, unsigned int *val)
{
	return regmap_read(bq2477x->regmap, reg, val);
}

static int bq2477x_read_word(struct bq2477x_chip *bq2477x,
			     unsigned int reg, unsigned int *val)
{
	return regmap_read(bq2477x->regmap_word, reg, val);
}

static int bq2477x_write(struct bq2477x_chip *bq2477x,
			 unsigned int reg, unsigned int val)
{
	return regmap_write(bq2477x->regmap, reg, val);
}

static int bq2477x_write_word(struct bq2477x_chip *bq2477x,
			      unsigned int reg, unsigned int val)
{
	return regmap_write(bq2477x->regmap_word, reg, val);
}

static int bq2477x_charger_get_status(struct battery_charger_dev *bc_dev)
{
	struct bq2477x_chip *bq2477x = battery_charger_get_drvdata(bc_dev);

	return bq2477x->chg_status;
}

static int bq2477x_charging_restart(struct battery_charger_dev *bc_dev)
{
	struct bq2477x_chip *bq2477x = battery_charger_get_drvdata(bc_dev);

	cancel_delayed_work_sync(&bq2477x->battery_work);
	schedule_delayed_work(&bq2477x->battery_work, 10);
	return 0;
}


static struct battery_charging_ops bq2477x_charger_bci_ops = {
	.get_charging_status = bq2477x_charger_get_status,
	.restart_charging = bq2477x_charging_restart,
};

static struct battery_charger_info bq2477x_charger_bci = {
	.cell_id = 0,
	.bc_ops = &bq2477x_charger_bci_ops,
};

static int bq2477x_show_chip_revision(struct bq2477x_chip *bq2477x)
{
	int ret;
	unsigned int val;

	ret = bq2477x_read(bq2477x, BQ2477X_DEVICE_ADDR_REG, &val);
	if (ret < 0) {
		dev_err(bq2477x->dev, "BQ2477X_DEVICE_ADDR_REG read failed: %d\n",
			ret);
		return 0;
	}

	if (val == BQ24770_DEVICE_ID_PG_1_0)
		dev_info(bq2477x->dev, "chip type BQ24770 detected\n");
	else if (val == BQ24773_DEVICE_ID_PG_1_0)
		dev_info(bq2477x->dev, "chip type BQ24773 PG_1_0 detected\n");
	else if (val == BQ24773_DEVICE_ID_PG_1_1)
		dev_info(bq2477x->dev, "chip type BQ24773 PG_1_1 detected\n");
	else {
		dev_info(bq2477x->dev, "unrecognized chip type: 0x%04x\n",
			val);
		return -EINVAL;
	}
	return val;
}

/* factory test */
static int bq2477x_t2_power_path_switch(struct bq2477x_chip *bq2477x,
					int onoff)
{
	if (onoff) {
		dev_err(bq2477x->dev,
			"bq2477x_t2_power_path_switch ONOFF %d\n", onoff);
		schedule_delayed_work(&bq2477x->battery_work,
				msecs_to_jiffies(50));
	} else {
		dev_err(bq2477x->dev,
			"bq2477x_t2_power_path_switch ONOFF %d\n", onoff);
		cancel_delayed_work_sync(&bq2477x->battery_work);
		bq2477x_stop_charging(bq2477x);
		wake_unlock(&bq2477x->dock_wake_lock);
	}
	return 0;
}

static void bq2477x_get_battery_status(struct bq2477x_chip *bq2477x,
				struct battery_status *status)
{
	int i;
	int thermal_zone = 1; /* If all else fails, use normal temp zone. */
	int hysteresis_temp = 20;
	struct power_supply *psy;
	union power_supply_propval battery_health;
	union power_supply_propval battery_status;
	union power_supply_propval battery_charge_voltage;
	union power_supply_propval battery_charge_current;
	union power_supply_propval battery_temp;
	/* Check if the battery can handle this much voltage/current */
	psy = bq2477x->pwr_battery;
	if (psy == NULL) {
		/* No battery. */
		status->battery_charge_voltage_mV = 0;
		status->battery_charge_current_mA = 0;
		return;
	}

	/* Make sure the battery is in good health. */
	if (!psy->get_property(psy,
				POWER_SUPPLY_PROP_HEALTH, &battery_health)) {
		if (battery_health.intval != POWER_SUPPLY_HEALTH_GOOD) {
			/* Bad battery. */
			status->battery_charge_voltage_mV = 0;
			status->battery_charge_current_mA = 0;
			/* set overheat temp zone. */
			if (battery_health.intval ==
				POWER_SUPPLY_HEALTH_OVERHEAT &&
				bq2477x->manual_temp_scaling)
				bq2477x->batt_status.pre_thermal_zone = 2;
			return;
		}
	}

	/* Check if the battery is fully charged. */
	if (!psy->get_property(psy,
				POWER_SUPPLY_PROP_STATUS, &battery_status)) {
		if (battery_status.intval == POWER_SUPPLY_STATUS_FULL) {
			/* Fully charged. */
			bq2477x_stop_charging(bq2477x);
			status->battery_charge_voltage_mV = 0;
			status->battery_charge_current_mA = 0;
			return;
		}
	}

	/* Get the battery temperature. */
	if (psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &battery_temp)) {
		/* Failed to get battery temperature/ */
		goto default_thermal_zone;
	}

	if (!bq2477x->manual_temp_scaling || bq2477x->charge_table == NULL) {
		/* Use integrated Advanced Charge Algorithm */
		if (!psy->get_property(
				psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
				&battery_charge_voltage) &&
		    !psy->get_property(
				psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
				&battery_charge_current)) {
			/* Scale battery properties from micro-units
			   to milli-units. */
			status->battery_charge_voltage_mV =
				battery_charge_voltage.intval / 1000;
			status->battery_charge_current_mA =
				battery_charge_current.intval / 1000;
			return;
		}
	}
	/* Use software defined Advanced Charge Algorithm table */
	for (i = 0; i < 3; i++) {
		if (battery_temp.intval > bq2477x->charge_table[i].min_temp &&
		    battery_temp.intval <= bq2477x->charge_table[i].max_temp) {
			/* battery in thermal zone i */
			thermal_zone = i;
			break;
		}
	}

	/*follow the spec use the hysteresis_temp to
		adjust the temperature range.*/
	if (thermal_zone < bq2477x->batt_status.pre_thermal_zone) {
		if (battery_temp.intval >
			bq2477x->charge_table[thermal_zone].max_temp -
			hysteresis_temp)
			thermal_zone = bq2477x->batt_status.pre_thermal_zone;
	}

default_thermal_zone:
	status->battery_charge_voltage_mV =
		bq2477x->charge_table[thermal_zone].charge_voltage;
	status->battery_charge_current_mA =
		bq2477x->charge_table[thermal_zone].charge_current;
	bq2477x->batt_status.pre_thermal_zone = thermal_zone;

#if BQ2477X_DEBUG_MESSAGES
	dev_info(bq2477x->dev, "%s: temp %d\n",
		__func__, battery_temp.intval);
#endif

	return;
}

int bq2477x_hw_init(struct bq2477x_chip *bq2477x, int profile)
{
	int ret = 0;
	int curr;
	int dac_iin;
	int dac_ichg;
	int dac_v;
	int reg_dac_iin;
	const int charger_input_voltage = 12;
	/* To avoid floating point, we break the 90% into 9 / 10. */
	int dcdc_efficiency_numerator = 9;
	int dcdc_efficiency_denominator = 10;

#if BQ2477X_DEBUG_MESSAGES
	dev_info(bq2477x->dev, "%s: bq2477x->dac_v = %d\n",
				__func__, bq2477x->dac_v);
#endif

	/* Configure input current based on power supply source. */
	switch (profile) {
	default:
	case USB_PROFILE: {
		int usb_voltage = 5;
		/* USB profile is charging at 5V boosted to 12V via 90% efficient converter. */
		int usb_power_mW = usb_voltage * (bq2477x->usb_charge_ua / 1000);
		dac_iin = (usb_power_mW * dcdc_efficiency_numerator) / charger_input_voltage;
		dac_iin /= dcdc_efficiency_denominator;
		break;
	}
	case DOCK_PROFILE:
		/* Dock profile is charging at 12V so dac_iin = doc_max_ua. */
		dac_iin = bq2477x->dock_max_ua / 1000;
		/* Disable the boost converter */
		gpio_direction_output(bq2477x->disable_vbus_12v_boost_gpio, 1);
		break;
	}

	/* Convert to register units of 1/64 ma. */
	reg_dac_iin = dac_iin/64;
#if BQ2477X_DEBUG_MESSAGES
	dev_info(bq2477x->dev, "%s: PROFILE= %s, dac_iin = %d, reg_dac_iin = %d\n",
		__func__, chargername[profile], dac_iin, reg_dac_iin*64);
#endif
	/* Configure the chargers input current */
	ret = bq2477x_write(bq2477x, BQ2477X_INPUT_CURRENT, reg_dac_iin);
	if (ret < 0) {
		dev_err(bq2477x->dev, "INPUT_CURRENT write failed %d\n", ret);
		return ret;
	}

	bq2477x_read(bq2477x, BQ2477X_INPUT_CURRENT, &curr);
#if BQ2477X_DEBUG_MESSAGES
	dev_info(bq2477x->dev, "%s: BQ2477X_INPUT_CURRENT = %d mA\n",
		__func__, curr*64);
#endif

	/* Limit max charge mA based on board file */
	dac_ichg = bq2477x->max_charge_ua / 1000;
	/* Limit max charge voltage based on board file */
	dac_v = bq2477x->dac_v;

	/* Check if the battery can handle this much voltage/current */
	bq2477x_get_battery_status(bq2477x, &bq2477x->batt_status);

	/* Adjust the charger voltage */
	if (bq2477x->batt_status.battery_charge_voltage_mV < dac_v)
		dac_v = bq2477x->batt_status.battery_charge_voltage_mV;

	/* Adjust the charger current */
	if (bq2477x->batt_status.battery_charge_current_mA < dac_ichg)
		dac_ichg = bq2477x->batt_status.battery_charge_current_mA;

#if BQ2477X_DEBUG_MESSAGES
	/* Report the charger settings */
	dev_info(bq2477x->dev, "%s: battery charge %dmV @ %dmA\n",
		__func__, dac_v, dac_ichg);
#endif

	/* Configure the battery charge voltage */
	ret = bq2477x_write_word(bq2477x, BQ2477X_MAX_CHARGE_VOLTAGE_LSB,
			(dac_v >> 8) | ((0x00FF & dac_v) << 8));
	if (ret < 0) {
		dev_err(bq2477x->dev, "CHARGE_VOLTAGE write failed %d\n", ret);
		return ret;
	}

	/* Configure charger output current */
#if BQ2477X_DEBUG_MESSAGES
	dev_info(bq2477x->dev, "%s: dac_ichg = %d\n", __func__, dac_ichg);
#endif
	ret = bq2477x_write_word(bq2477x, BQ2477X_CHARGE_CURRENT_LSB,
		(dac_ichg >> 8) | ((0x00FF & dac_ichg) << 8));
	if (ret < 0) {
		dev_err(bq2477x->dev, "CHARGE_CURRENT write failed %d\n", ret);
		return ret;
	}

	/* Configure control and Enable charging */
	ret = bq2477x_write(bq2477x, BQ2477X_CHARGE_OPTION_0_MSB,
		BQ2477X_CHARGE_OPTION_POR_MSB);
	if (ret < 0) {
		dev_err(bq2477x->dev, "CHARGE_OPTION_0 MSB write failed %d\n", ret);
		return ret;
	}
	ret = bq2477x_write(bq2477x, BQ2477X_CHARGE_OPTION_0_LSB,
		BQ2477X_CHARGE_OPTION_POR_LSB);
	if (ret < 0) {
		dev_err(bq2477x->dev, "CHARGE_OPTION_0 LSB write failed %d\n", ret);
		return ret;
	}

	power_supply_changed(&bq2477x->usb);
	power_supply_changed(&bq2477x->ac);
	return ret;
}

static enum power_supply_property bq2477x_psy_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_POWER_PATH,
};

static enum power_supply_property bq2477x_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};


static int bq2477x_usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct bq2477x_chip *bq2477x = container_of(psy,
					struct bq2477x_chip, usb);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (bq2477x->charger_type_is_dock)
			val->intval = 0;
		else
			val->intval = bq2477x->charger_type_is_usb;
		return 0;
	default:
		return -EINVAL;
	}
	return 0;
}


static int bq2477x_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct bq2477x_chip *bq2477x = container_of(psy,
					struct bq2477x_chip, ac);
	struct power_supply_extcon *psy_extcon;
	int charge_current;
	int ret;

	psy_extcon = bq2477x->psy_dock_extcon;
	if (psy_extcon == NULL)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = psy_extcon->dock_online;
		return 0;

	case POWER_SUPPLY_PROP_POWER_PATH:
		ret = bq2477x_read_word(bq2477x, BQ2477X_CHARGE_CURRENT_LSB,
					&charge_current);
		if (ret < 0) {
			dev_err(bq2477x->dev,
				"BQ2477X_CHARGE_CURRENT_LSB read failed %d\n",
					ret);
			val->intval = 0;
			return 0;
		}

		if (charge_current > 0) {
			val->intval = 1;
			return 0;
		}

		val->intval = 0;
		return 0;
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq2477x_ac_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	struct bq2477x_chip *bq2477x;
	bq2477x = container_of(psy, struct bq2477x_chip, ac);

	switch (psp) {
	case POWER_SUPPLY_PROP_POWER_PATH:
		return bq2477x_t2_power_path_switch(bq2477x, val->intval);
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2477x_stop_charging(struct bq2477x_chip *bq2477x)
{
	int ret;

	dev_dbg(bq2477x->dev, "%s: ENTER\n", __func__);

	ret = bq2477x_write(bq2477x, BQ2477X_CHARGE_CURRENT_LSB, 0x00);
	if (ret < 0) {
		dev_err(bq2477x->dev, "INPUT_CURRENT write failed %d\n", ret);
		return ret;
	}
	ret = bq2477x_write(bq2477x, BQ2477X_CHARGE_CURRENT_MSB, 0x00);
	if (ret < 0) {
		dev_err(bq2477x->dev, "INPUT_CURRENT write failed %d\n", ret);
		return ret;
	}
	power_supply_changed(&bq2477x->usb);
	power_supply_changed(&bq2477x->ac);
	return ret;
}

static int bq2477x_plug_in_charging_enable(struct bq2477x_chip *bq2477x,
		int chargertype)
{
	int ret = 0;

#if BQ2477X_DEBUG_MESSAGES
	dev_info(bq2477x->dev, "%s chargertype=%s\n",
		__func__, chargername[chargertype]);
#endif

	switch (chargertype) {
	case USB_PROFILE:
		ret = bq2477x_hw_init(bq2477x, USB_PROFILE);
		break;
	case DOCK_PROFILE:
		ret = bq2477x_hw_init(bq2477x, DOCK_PROFILE);
		break;
	}

	if (!ret) {
		bq2477x->chg_status = BATTERY_CHARGING;
		battery_charging_status_update(bq2477x->bc_dev,
			BATTERY_CHARGING);
	}

	return ret;
}

static int bq2477x_plug_out_charging_disable(struct bq2477x_chip *bq2477x)
{
	bq2477x->chg_status = BATTERY_DISCHARGING;
	battery_charging_status_update(bq2477x->bc_dev, BATTERY_DISCHARGING);
	return 0;
}

static void bq2477x_plug_in_charging_enable_vbus(struct bq2477x_chip *bq2477x)
{
	if (!bq2477x->charger_type_is_dock)
		bq2477x_plug_in_charging_enable(bq2477x, USB_PROFILE);
}



static void bq2477x_plug_in_charging_disable(struct bq2477x_chip *bq2477x)
{
	if (!bq2477x->charger_type_is_usb &&
	    !bq2477x->charger_type_is_dock) {
		bq2477x_plug_out_charging_disable(bq2477x);
		bq2477x_stop_charging(bq2477x);
	}
}

static void bq2477x_battery_work(struct work_struct *w)
{
	struct bq2477x_chip *bq2477x = container_of(to_delayed_work(w),
			struct bq2477x_chip, battery_work);
	struct battery_status batt_status;

	bq2477x_get_battery_status(bq2477x, &batt_status);

	if (batt_status.battery_charge_voltage_mV ==
		bq2477x->batt_status.battery_charge_voltage_mV &&
		batt_status.battery_charge_current_mA ==
		bq2477x->batt_status.battery_charge_current_mA) {
		goto exit;
	}

	mutex_lock(&bq2477x->mutex);
	if (bq2477x->charger_type_is_dock)
		bq2477x_plug_in_charging_enable(bq2477x, DOCK_PROFILE);
	else if (bq2477x->charger_type_is_usb)
		bq2477x_plug_in_charging_enable(bq2477x, USB_PROFILE);
	mutex_unlock(&bq2477x->mutex);
exit:
	schedule_delayed_work(&bq2477x->battery_work, BATTERY_CHECK_TIME);
}

static int bq2477x_is_temp_scaled_manual(struct bq2477x_chip *bq2477x)
{
	union power_supply_propval battery_Manufacturer;
	char manufacturer_smp[] = "SMP";
	struct power_supply *psy;
	int ret;

	psy = bq2477x->pwr_battery;
	if (!psy)
		return 1;

	ret = psy->get_property(psy,
		POWER_SUPPLY_PROP_MANUFACTURER, &battery_Manufacturer);

	if (ret)
		return 1;

	ret = strcmp(battery_Manufacturer.strval, manufacturer_smp) != 0;

	return ret;
}

static void bq2477x_acok_work(struct work_struct *work)
{
	int state;
	int reg_dac_iin;
	int ret;

	struct bq2477x_chip *bq2477x = container_of(work,
						struct bq2477x_chip,
						acok_work.work);

	state = gpio_get_value(bq2477x->acok_gpio);

	if (!state) {
		ret = bq2477x_read(bq2477x, BQ2477X_INPUT_CURRENT,
						&reg_dac_iin);
		if (ret < 0) {
			dev_err(bq2477x->dev,
				"BQ2477X_INPUT_CURRENT read failed %d\n", ret);
		}
		dev_info(bq2477x->dev,
				"%s: (B) BQ2477X_INPUT_CURRENT = %d mA\n",
				__func__, reg_dac_iin*64);

		/* current should not go lowered by 128 mA */
		if (reg_dac_iin <= 2)
			return;

		reg_dac_iin = reg_dac_iin-1;
		ret = bq2477x_write(bq2477x, BQ2477X_INPUT_CURRENT,
						reg_dac_iin);
		if (ret < 0) {
			dev_err(bq2477x->dev,
				"INPUT_CURRENT write failed %d\n", ret);
		}

		ret = bq2477x_read(bq2477x, BQ2477X_INPUT_CURRENT,
						&reg_dac_iin);
		if (ret < 0) {
			dev_err(bq2477x->dev,
				"BQ2477X_INPUT_CURRENT read failed %d\n",
				ret);
		}
		dev_info(bq2477x->dev,
			"%s: (A) BQ2477X_INPUT_CURRENT = %d mA\n",
			__func__, reg_dac_iin*64);
	}
}

static irqreturn_t bq2477x_acok_irq(int irq, void *data)
{
	struct bq2477x_chip *bq2477x = data;

	schedule_delayed_work(&bq2477x->acok_work, msecs_to_jiffies(4));
	return IRQ_HANDLED;
}


static void initial_hw_settings(struct bq2477x_chip *bq2477x)
{
	int ret;
	int curr;

	bq2477x_show_chip_revision(bq2477x);

	dev_info(bq2477x->dev, "%s: bq2477x->dac_v = %d, bq2477x->dac_minsv = %d\n",
		__func__, bq2477x->dac_v, bq2477x->dac_minsv);

	/* Configure the minimum system voltage */
	ret = bq2477x_write(bq2477x, BQ2477X_MIN_SYS_VOLTAGE,
		bq2477x->dac_minsv >> BQ2477X_MIN_SYS_VOLTAGE_SHIFT);
	if (ret < 0)
		dev_err(bq2477x->dev, "MIN_SYS_VOLTAGE write failed %d\n", ret);

	bq2477x_read(bq2477x, BQ2477X_INPUT_CURRENT, &curr);
	dev_info(bq2477x->dev, "%s: BQ2477X_INPUT_CURRENT = %d mA\n",
		__func__, curr*64);
}

static ssize_t bq2477x_set_force_no_wake_lock(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2477x_chip *bq2477x = i2c_get_clientdata(client);
	char *p = (char *)buf;
	unsigned long long x;

	x = memparse(p, &p);
	bq2477x->force_no_wake_lock = x;

	if (!bq2477x->force_no_wake_lock) {
		dev_info(bq2477x->dev, "%s: reboot required\n", __func__);
		return -EINVAL;
	} else {
		if (bq2477x->charger_type_is_dock)
			wake_unlock(&bq2477x->dock_wake_lock);
		if (bq2477x->charger_type_is_usb)
			wake_unlock(&bq2477x->usb_wake_lock);
	}

	return count;
}

static ssize_t bq2477x_show_force_no_wake_lock(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2477x_chip *bq2477x = i2c_get_clientdata(client);

	if (bq2477x->force_no_wake_lock)
		return snprintf(buf, MAX_STR_PRINT, "enabled\n");
	else
		return snprintf(buf, MAX_STR_PRINT, "disabled\n");
}

static DEVICE_ATTR(force_no_wake_lock, (S_IRUGO | S_IWUSR),
		bq2477x_show_force_no_wake_lock, 
		bq2477x_set_force_no_wake_lock);

static struct attribute *bq2477x_attributes[] = {
	&dev_attr_force_no_wake_lock.attr,
	NULL
};

static const struct attribute_group bq2477x_attr_group = {
	.attrs = bq2477x_attributes,
};

static int bq2477x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct bq2477x_chip *bq2477x = NULL;
	struct bq2477x_platform_data *pdata;
	int ret;

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "No Platform data");
		return -EINVAL;
	}

	bq2477x = devm_kzalloc(&client->dev, sizeof(*bq2477x), GFP_KERNEL);
	if (!bq2477x) {
		dev_err(&client->dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	msleep(25);

	bq2477x->dev = &client->dev;
	bq2477x->dac_v = pdata->dac_v;
	bq2477x->dac_minsv = pdata->dac_minsv;
	bq2477x->wdt_refresh_timeout = pdata->wdt_refresh_timeout;
	bq2477x->disable_vbus_12v_boost_gpio =
				pdata->disable_vbus_12v_boost_gpio;
	bq2477x->charge_table = pdata->charge_table;

	dev_info(bq2477x->dev, "%s: dac_v = %04X, dac_minsv = %04X, disable_vbus_12v_boost_gpio = %d\n",
		__func__, bq2477x->dac_v,
		bq2477x->dac_minsv, bq2477x->disable_vbus_12v_boost_gpio);

	bq2477x->vbus_reg = regulator_get(bq2477x->dev, "usb_bat_chg");
	if (IS_ERR(bq2477x->vbus_reg)) {
		dev_info(bq2477x->dev,
			"%s: usb_bat_chg regulator not registered: err - %p\n",
				__func__, bq2477x->vbus_reg);
		bq2477x->vbus_reg = NULL;
		kfree(bq2477x);
		return -EPROBE_DEFER;
	}

	i2c_set_clientdata(client, bq2477x);
	bq2477x->extcon_dock_name = pdata->extcon_dock_name;

	ret = gpio_request(bq2477x->disable_vbus_12v_boost_gpio,
			"usb2_12v_boost_disabled");
	if (ret) {
		dev_err(&client->dev, "Failed to request disable_vbus_12v_boost_gpio pin: %d\n",
			ret);
	}

	ret = gpio_direction_output(bq2477x->disable_vbus_12v_boost_gpio, 0);

	if (ret) {
		dev_err(&client->dev,
			"Failed to set disable_vbus_12v_boost_gpio to output: %d\n",
			ret);
	}
	bq2477x->acok_gpio = pdata->acok_gpio;
	ret = gpio_request(bq2477x->acok_gpio, "acok_gpio");
	if (ret) {
		dev_err(&client->dev, "Failed to request acok_gpio pin: %d\n",
			ret);
		bq2477x->acok_gpio = 0;
	}

	if (bq2477x->acok_gpio)	 {
		ret = gpio_direction_input(bq2477x->acok_gpio);
		if (ret) {
			dev_err(&client->dev,
				"Failed to set acok_gpio to input: %d\n",
				ret);
			bq2477x->acok_gpio = 0;
		}
	}
	mutex_init(&bq2477x->mutex);

	bq2477x->regmap = devm_regmap_init_i2c(client, &bq2477x_regmap_config);
	if (IS_ERR(bq2477x->regmap)) {
		ret = PTR_ERR(bq2477x->regmap);
		dev_err(&client->dev, "regmap init failed with err %d\n", ret);
		goto gpio_err;
	}

	bq2477x->regmap_word = devm_regmap_init_i2c(client,
		&bq2477x_regmap_word_config);
	if (IS_ERR(bq2477x->regmap_word)) {
		ret = PTR_ERR(bq2477x->regmap_word);
		dev_err(&client->dev,
			"regmap_word init failed with err %d\n", ret);
		goto gpio_err;
	}

	INIT_DELAYED_WORK(&bq2477x->acok_work, bq2477x_acok_work);
	INIT_DELAYED_WORK(&bq2477x->usb_work, bq2477x_usb_work);
	INIT_DELAYED_WORK(&bq2477x->battery_work, bq2477x_battery_work);

	bq2477x->ac.name		= "dock-12v";
	bq2477x->ac.type		= POWER_SUPPLY_TYPE_MAINS;
	bq2477x->ac.get_property	= bq2477x_ac_get_property;
	/*factory test*/
	bq2477x->ac.set_property	= bq2477x_ac_set_property;
	bq2477x->ac.properties		= bq2477x_psy_props;
	bq2477x->ac.num_properties	= ARRAY_SIZE(bq2477x_psy_props);

	ret = power_supply_register(bq2477x->dev, &bq2477x->ac);
	if (ret < 0) {
		dev_err(bq2477x->dev,
			"AC power supply register failed %d\n", ret);
		goto gpio_err;
	}

	bq2477x->usb.name		= "usb-charger";
	bq2477x->usb.type		= POWER_SUPPLY_TYPE_USB;
	bq2477x->usb.get_property	= bq2477x_usb_get_property;
	bq2477x->usb.properties		= bq2477x_usb_props;
	bq2477x->usb.num_properties	= ARRAY_SIZE(bq2477x_usb_props);
	ret = power_supply_register(bq2477x->dev, &bq2477x->usb);
	if (ret) {
		dev_err(bq2477x->dev,
			"USB power supply register failed %d\n", ret);
		goto gpio_err;
	}

	bq2477x->max_charge_ua = pdata->max_charge_ua;
	bq2477x->dock_max_ua = pdata->dock_max_ua;

	bq2477x->bc_dev = battery_charger_register(bq2477x->dev,
		&bq2477x_charger_bci, bq2477x);
	if (IS_ERR(bq2477x->bc_dev)) {
		ret = PTR_ERR(bq2477x->bc_dev);
		dev_err(bq2477x->dev, "battery charger register failed: %d\n",
			ret);
		goto psy_err;
	}
	ret = sysfs_create_group(&client->dev.kobj, &bq2477x_attr_group);
	if (ret < 0) {
		dev_err(&client->dev, "sysfs create failed %d\n", ret);
		goto bc_err;
	}

	wake_lock_init(&bq2477x->dock_wake_lock, WAKE_LOCK_SUSPEND,
		"12v-dock-lock");

	bq2477x->vbus_nb.notifier_call = bq2477x_vbus_changed;
	ret = regulator_register_notifier(bq2477x->vbus_reg,
					  &bq2477x->vbus_nb);
	if (ret < 0) {
		dev_err(bq2477x->dev, "regulator_register_notifier %d\n", ret);
		goto bc_err;
	}

	/* do one pass in case dock was connected at boot time */
	if (bq22477x_dock_extcon_probe(bq2477x) < 0) {
		dev_err(bq2477x->dev, "bq22477x_dock_extcon_probe failed: %d\n",
			ret);
		goto bc_err;
	}

	bq2477x->pwr_battery = power_supply_get_by_name("battery");
	if (!bq2477x->pwr_battery)
		dev_err(bq2477x->dev, "Battery power supply is not available\n");

	bq2477x->manual_temp_scaling = bq2477x_is_temp_scaled_manual(bq2477x);
	wake_lock_init(&bq2477x->usb_wake_lock, WAKE_LOCK_SUSPEND,
		"usb2-lock");

	/* bootloader may leave usb current too high */
	/* when tegra_udc detects cable will be adjusted */
	bq2477x->usb_charge_ua = 0;
	bq2477x->batt_status.pre_thermal_zone = 1; /*normal temp zone. */
	initial_hw_settings(bq2477x);
	bq2477x_hw_init(bq2477x, USB_PROFILE);

	if (bq2477x->acok_gpio) {
		ret = request_irq(gpio_to_irq(bq2477x->acok_gpio),
				bq2477x_acok_irq,
				IRQF_SHARED | IRQF_TRIGGER_FALLING,
				"ACOK", bq2477x);
		if (ret < 0) {
			dev_warn(bq2477x->dev,
				"request IRQ %d fail, err = %d\n",
				gpio_to_irq(bq2477x->acok_gpio), ret);
			dev_info(bq2477x->dev,
				"Use bq driver without ACOK interrupt\n");
			ret = 0;
		}
	}
	schedule_delayed_work(&bq2477x->battery_work, 35*HZ);
	dev_info(bq2477x->dev, "%s: success\n", __func__);
	return ret;

bc_err:
	battery_charger_unregister(bq2477x->bc_dev);
psy_err:
	power_supply_unregister(&bq2477x->ac);
gpio_err:
	gpio_free(bq2477x->disable_vbus_12v_boost_gpio);
	return ret;
}

static int bq2477x_remove(struct i2c_client *client)
{
	struct bq2477x_chip *bq2477x = i2c_get_clientdata(client);

	bq2477x_stop_charging(bq2477x);
	if (bq2477x->acok_gpio) {
		free_irq(gpio_to_irq(bq2477x->acok_gpio), NULL);
		cancel_delayed_work_sync(&bq2477x->acok_work);
	}
	power_supply_unregister(&bq2477x->ac);
	gpio_free(bq2477x->disable_vbus_12v_boost_gpio);
	wake_lock_destroy(&bq2477x->dock_wake_lock);
	wake_lock_destroy(&bq2477x->usb_wake_lock);
	return 0;
}

static void bq2477x_shutdown(struct i2c_client *client)
{
	struct bq2477x_chip *bq2477x = i2c_get_clientdata(client);

	/* boot loader hangs if charging using wall charger.
	 * Turn off charging as work around.  boot loader should turn
	 * charging on.
	 */
	bq2477x_stop_charging(bq2477x);
}


#ifdef CONFIG_PM_SLEEP
static int bq2477x_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bq2477x_chip *bq2477x = platform_get_drvdata(pdev);

	if (bq2477x->charger_type_is_dock)
		battery_charging_wakeup(bq2477x->bc_dev, 300);
	return 0;
}

static int bq2477x_resume(struct device *dev)
{
	return 0;
}
#endif /* CONFIG_PM */

static SIMPLE_DEV_PM_OPS(bq2477x_pm_ops, bq2477x_suspend, bq2477x_resume);

static const struct i2c_device_id bq2477x_id[] = {
	{.name = driver_name,},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2477x_id);

static struct i2c_driver bq2477x_i2c_driver = {
	.driver = {
		.name	= driver_name,
		.owner	= THIS_MODULE,
		.pm = &bq2477x_pm_ops,
	},
	.probe		= bq2477x_probe,
	.remove		= bq2477x_remove,
	.shutdown	= bq2477x_shutdown,
	.id_table	= bq2477x_id,
};

static int __init bq2477x_module_init(void)
{
	return i2c_add_driver(&bq2477x_i2c_driver);
}
rootfs_initcall(bq2477x_module_init);

static void __exit bq2477x_cleanup(void)
{
	i2c_del_driver(&bq2477x_i2c_driver);
}
module_exit(bq2477x_cleanup);

MODULE_DESCRIPTION("BQ24775/BQ24777 battery charger driver");
MODULE_AUTHOR("Philip Rakity <prakity@nvidia.com>");
MODULE_AUTHOR("Andy Park <andyp@nvidia.com>");
MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com");
MODULE_LICENSE("GPL v2");

