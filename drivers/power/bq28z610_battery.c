/*
 *  bq28z610_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 * Copyright (c) 2012-2014, NVIDIA CORPORATION.	 All rights reserved.
 *  Chandler Zhang <chazhang@nvidia.com>
 *  Syed Rafiuddin <srafiuddin@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/unaligned.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/power/bq28z610_battery.h>
#include <linux/power/battery-charger-gauge-comm.h>
#include <linux/pm.h>
#include <linux/jiffies.h>
#include <linux/regmap.h>
#include <linux/ina3221.h>


#define BQ28Z610_DELAY		(30*HZ)

#define BQ28Z610_VERSION_NO	0x11
#define BQ28Z610_CONTROL_STATUS		0x0000
#define BQ28Z610_CONTROL_STATUS_AUTO_CAL	(1<<12)
#define BQ28Z610_DEVICE_TYPE		0x0001
#define BQ28Z610_FW_VERSION		0x0002
#define BQ28Z610_DM_CODE			0x0004
#define BQ28Z610_PREV_MACWRITE		0x0007
#define BQ28Z610_CHEM_ID			0x0008
#define BQ28Z610_BAT_INSERT		0x000C
#define BQ28Z610_BAT_REMOVE		0x000D
#define BQ28Z610_SET_HIBERNATE		0x0011
#define BQ28Z610_CLEAR_HIBERNATE		0x0012
#define BQ28Z610_SET_CFGUPDATE		0x0013
#define BQ28Z610_SHUTDOWN_ENABLE		0x001B
#define BQ28Z610_SHUTDOWN		0x001C
#define BQ28Z610_SEALED			0x0020
#define BQ28Z610_PULSE_SOC_INT		0x0023
#define BQ28Z610_RESET			0x0041
#define BQ28Z610_SOFT_RESET		0x0042
#define BQ28Z610_MANUFACTURER_NAME	0x004C
#define BQ28Z610_MANUFACTURER_DATE	0x004D
#define BQ28Z610_MANUFACTURER_SERIAL_NUM	0x004E

#define BQ28Z610_MANUFACTURERACCESS	0x00
#define BQ28Z610_MANUFACTURERDATA	0x40
#define BQ28Z610_CONTROL_1		0x00
#define BQ28Z610_CONTROL_2		0x01
#define BQ28Z610_TEMPERATURE		0x06
#define BQ28Z610_VOLTAGE		0x08
#define BQ28Z610_BATTERY_STATUS		0x0a
#define BQ28Z610_BATTERY_FULL_DISCHARGED		(1<<4)
#define BQ28Z610_BATTERY_FULL_CHARGED			(1<<5)
#define BQ28Z610_BATTERY_DISCHARGING			(1<<6)

#define BQ28Z610_BATTERY_TERMINATE_DISCHARGE_ALARM	(1<<11)
#define BQ28Z610_BATTERY_OVER_TEMP_ALARM		(1<<12)
#define BQ28Z610_BATTERY_TERMINATE_CHARGE_ALARM		(1<<14)
#define BQ28Z610_BATTERY_OVER_CHARGE_ALARM		(1<<15)

#define BQ28Z610_CURRENT_NOW		0x0c
#define BQ28Z610_REMAINING_CAPACITY	0x10
#define BQ28Z610_FULL_CHG_CAPACITY	0x12
#define BQ28Z610_AVG_CURRENT		0x14
#define BQ28Z610_AVG_TIME_TO_EMPTY	0x16
#define BQ28Z610_AVG_TIME_TO_FULL	0x18
#define BQ28Z610_STANDBY_CURRENT	0x1a
#define BQ28Z610_STANDBY_TIME_TO_EMPTY	0x1c
#define BQ28Z610_MAXLOAD_CURRENT	0x1e
#define BQ28Z610_MAXLOAD_TIME_TO_EMPTY	0x20
#define BQ28Z610_AVERAGE_POWER		0x24
#define BQ28Z610_INT_TEMPERATURE	0x28
#define BQ28Z610_CYCLE_COUNT		0x2a
#define BQ28Z610_RELATIVE_STATE_OF_CHARGE	0x2c
#define BQ28Z610_STATE_OF_HEALTH	0x2e
#define BQ28Z610_CHARGING_VOLTAGE	0x30
#define BQ28Z610_CHARGING_CURRENT	0x32
#define BQ28Z610_DESIGN_CAPACITY	0x30

#define BQ28Z610_BLOCK_DATA_CHECKSUM	0x60
#define BQ28Z610_BLOCK_DATA_CONTROL	0x61
#define BQ28Z610_DATA_BLOCK_CLASS	0x3E
#define BQ28Z610_DATA_BLOCK		0x3F

#define BQ28Z610_DESIGN_CAPACITY_1	0x4A
#define BQ28Z610_DESIGN_CAPACITY_2	0x4B

#define BQ28Z610_BATTERY_LOW		15
#define BQ28Z610_BATTERY_FULL		100
#define BQ28Z610_MAX_REGS		0x7F

#ifdef NOTUSED
static char *chargingstatus[] =
	{"UNKNOWN", "CHARGING", "DISCHARGING", "NOT_CHARGING", "FULL"};
#endif

struct bq28z610_chip {
	struct i2c_client		*client;
#ifdef BQ28Z610_DEBUG
	struct delayed_work		work;
#endif
	struct delayed_work		power_work;
	struct power_supply		battery;
	struct bq28z610_platform_data *pdata;
	struct battery_gauge_dev	*bg_dev;
	struct regmap			*regmap;


	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge Scale */
	int soc_scale;
	/* State Of Charge Shift */
	int soc_shift;
	int power_supply_status;
	int status;
	/* battery health */
	int health;
	/* battery capacity */
	int capacity_level;
	/* Amount of current in ma flowing into or out of the battery */
	int current_now;
	int temperature;
	/* battery present */
	int present;
	int manual_temp_alarm;
	int temp_alarm_degc;
	
	int shutdown_complete;
	struct mutex mutex;
};

static int bq28z610_write_word(struct i2c_client *client, int reg, u16 value)
{
	struct bq28z610_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mutex);
	if (chip && chip->shutdown_complete) {
		mutex_unlock(&chip->mutex);
		return -ENODEV;
	}


	ret = i2c_smbus_write_word_data(client, reg, swab16(value));

	if (ret < 0)
		dev_err(&client->dev, "%s(): Failed in writing register"
					"0x%02x err %d\n", __func__, reg, ret);

	mutex_unlock(&chip->mutex);
	return ret;
}

static int bq28z610_read_word(struct i2c_client *client, u8 reg, s16 *val)
{
	int ret;
	int try;
	int retry_count=2;

	struct bq28z610_chip *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->mutex);
	if (chip && chip->shutdown_complete) {
		mutex_unlock(&chip->mutex);
		return -ENODEV;
	}

	for (try = 0; try <= retry_count; try++) {
		ret = regmap_raw_read(chip->regmap, reg, (u16 *) val, sizeof(*val));
		if (ret != -EREMOTEIO){
			if (ret < 0) {
				mutex_unlock(&chip->mutex);
				return ret;
			}
			break;
		}
		msleep(40);

		if (try==2 && ret < 0) {
			dev_err(&client->dev, "error reading reg: 0x%x, ret=%d\n",
				reg, ret);
			mutex_unlock(&chip->mutex);
			return ret;
		}
	}

	mutex_unlock(&chip->mutex);
	return 0;
}

static int bq28z610_read_block(struct i2c_client *client, u8 reg, s32 *val)
{
	int ret;
	int try;
	int retry_count=2;

	struct bq28z610_chip *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->mutex);
	if (chip && chip->shutdown_complete) {
		mutex_unlock(&chip->mutex);
		return -ENODEV;
	}

	for (try = 0; try <= retry_count; try++) {
		ret = regmap_raw_read(chip->regmap, reg, (u32*) val, sizeof(*val));
		if (ret != -EREMOTEIO){
			if (ret < 0) {
				mutex_unlock(&chip->mutex);
				return ret;
			}
			break;
		}
		msleep(40);

		if (try==2 && ret < 0) {
			dev_err(&client->dev, "error reading reg: 0x%x, ret=%d\n",
				reg, ret);
			mutex_unlock(&chip->mutex);
			return ret;
		}
	}

	mutex_unlock(&chip->mutex);
	return 0;
}


#ifdef BQ28Z610_DEBUG
static int bq28z610_get_average_current(struct bq28z610_chip *chip)
{
	s16 curr;
	int ret;

	ret = bq28z610_read_word(chip->client, BQ28Z610_AVG_CURRENT, &curr);
	pr_info("%s: BQ28Z610_AVG_CURRENT = %d mA\n", __func__, curr);
	return curr;
}

static int bq28z610_get_time_to_full(struct bq28z610_chip *chip)
{
	u16 time;
	int ret;

	ret = bq28z610_read_word(chip->client,
			BQ28Z610_AVG_TIME_TO_FULL, &time);
	pr_info("%s: BQ28Z610_AVG_TIME_TO_FULL = %d minutes\n", __func__, time);
	return time;
}

static int bq28z610_get_time_to_empty(struct bq28z610_chip *chip)
{
	u16 time;
	int ret;

	ret = bq28z610_read_word(chip->client,
			BQ28Z610_AVG_TIME_TO_EMPTY, &time);
	pr_info("%s: BQ28Z610_AVG_TIME_TO_EMPTY = %d minutes\n",
			__func__, time);
	return time;
}

static int bq28z610_get_full_charge_capacity(struct bq28z610_chip *chip)
{
	int ret;
	u16 full_charge;

	ret = bq28z610_read_word(chip->client,
			BQ28Z610_FULL_CHG_CAPACITY, &full_charge);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
	pr_info("%s  BQ28Z610_FULL_CHG_CAPACITY = %d mAh\n",
			__func__, full_charge);
	return full_charge;
}

static int bq28z610_get_remaining_capacity(struct bq28z610_chip *chip)
{
	int ret;
	u16 capacity;

	ret = bq28z610_read_word(chip->client, BQ28Z610_REMAINING_CAPACITY,
			&capacity);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
	pr_info("%s: BQ28Z610_REMAINING_CAPACITY = %d mAh\n",
		__func__, capacity);
	return capacity;
}

static int bq28z610_get_max_load_current(struct bq28z610_chip *chip)
{
	s16 max_load_current;
	int ret;

	ret = bq28z610_read_word(chip->client, BQ28Z610_MAXLOAD_CURRENT,
		&max_load_current);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
	pr_info("%s: BQ28Z610_MAXLOAD_CURRENT = %d mA\n",
		__func__, max_load_current);
	return max_load_current;
}

static int bq28z610_get_standyby_current(struct bq28z610_chip *chip)
{
	int ret;
	s16 standby_curr;

	ret = bq28z610_read_word(chip->client, BQ28Z610_STANDBY_CURRENT,
			&standby_curr);
	pr_info("%s: BQ28Z610_STANDBY_CURRENT = %d\n", __func__, standby_curr);
	if (ret)
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
	return standby_curr;
}

static int bq28z610_get_Manufacturer_Date(struct bq28z610_chip *chip)
{
	u32 read_data;
	int ret;

	ret = bq28z610_write_word(chip->client, BQ28Z610_MANUFACTURERACCESS,
			(BQ28Z610_MANUFACTURER_DATE >> 8) |
			((0x00FF & BQ28Z610_MANUFACTURER_DATE) << 8));
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s: write ManufacturerAccess err %d\n"
				, __func__, ret);
		return ret;
	}

	ret = bq28z610_read_block(chip->client, BQ28Z610_MANUFACTURERDATA, &read_data);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	pr_info("%s: DATA = %x\n", __func__, read_data);


	return 0;
}

static int bq28z610_get_serial_num(struct bq28z610_chip *chip)
{
	u16 data;
	int ret;

	ret = bq28z610_write_word(chip->client,
				BQ28Z610_MANUFACTURERACCESS,
				(BQ28Z610_MANUFACTURER_SERIAL_NUM >> 8) |
				((0x00FF & BQ28Z610_MANUFACTURER_SERIAL_NUM) << 8));
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s: write ManufacturerAccess err %d\n"
				, __func__, ret);
		return ret;
	}

	ret = bq28z610_read_word(chip->client, BQ28Z610_MANUFACTURERDATA, &data);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	pr_info("%s: DATA = %x\n", __func__, data);


	return 0;
}
#endif


static int bq28z610_get_charging_voltage(struct bq28z610_chip *chip)
{
	int ret;
	u16 charge_voltage;

	ret = bq28z610_read_word(chip->client, BQ28Z610_CHARGING_VOLTAGE,
			&charge_voltage);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
	return charge_voltage;
}

static int bq28z610_get_current_now(struct bq28z610_chip *chip)
{
	s16 curr;
	int ret;

	ret = bq28z610_read_word(chip->client, BQ28Z610_CURRENT_NOW, &curr);
	chip->current_now = curr;
	return curr;
}

static int bq28z610_get_temperature(struct bq28z610_chip *chip)
{
	u16 val;
	s16 temp;
	int ret;

	ret = bq28z610_read_word(chip->client, BQ28Z610_TEMPERATURE, &val);
	if (ret	 < 0){
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
	}

	temp = (val - 2731) / 10;

	 /* TODO: this is a temporary solution to avoid abnormally large temperature value from bq28z610 */
	if (temp < - 100 || temp > 100) temp = chip->temperature;
	else chip->temperature = temp;

	return temp;
}

static int bq28z610_get_cycle_count(struct bq28z610_chip *chip)
{
	u16 count;
	int ret;

	ret = bq28z610_read_word(chip->client, BQ28Z610_CYCLE_COUNT, &count);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, count);
	return count;
}


static int bq28z610_get_charging_current(struct bq28z610_chip *chip)
{
	s16 chargingcurrent;
	int ret;

	ret = bq28z610_read_word(chip->client,
			BQ28Z610_CHARGING_CURRENT, &chargingcurrent);
	return chargingcurrent;
}

static int bq28z610_get_vcell(struct bq28z610_chip *chip)
{
	static int last_vcell=7400;
	u16 vcell;
	int ret;

	ret = bq28z610_read_word(chip->client, BQ28Z610_VOLTAGE, &vcell);

	if (ret < 0){
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
		chip->vcell=last_vcell;
	}
	else{
		chip->vcell = vcell;
		last_vcell=chip->vcell;
	}
	return vcell;
}


static int bq28z610_get_status_and_soc(struct bq28z610_chip *chip)
{
	u16 status;
	u16 soc;
	int ret;
	u16 reported_status;
	int temp;

	/* Get battery StateOfCharge */
	ret = bq28z610_read_word(chip->client,
			BQ28Z610_RELATIVE_STATE_OF_CHARGE, &soc);
	if (ret < 0) {
		soc = 50;
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, soc);
	}

	/* Get battery status */
	ret = bq28z610_read_word(chip->client,
			BQ28Z610_BATTERY_STATUS, &status);
	if (ret) {
		chip->soc = soc;
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
		return -1;
	}

	temp = bq28z610_get_temperature(chip);

	/* Parse battery status bits */
	if (status & BQ28Z610_BATTERY_FULL_CHARGED) {
		reported_status = POWER_SUPPLY_STATUS_FULL;
		chip->soc = 100;
		chip->soc_scale = soc;
		chip->health = POWER_SUPPLY_HEALTH_GOOD;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	} else if (status & BQ28Z610_BATTERY_FULL_DISCHARGED) {
		reported_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		chip->health = POWER_SUPPLY_HEALTH_DEAD;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		chip->soc = 0;
	} else if (status & BQ28Z610_BATTERY_OVER_TEMP_ALARM ||
			(chip->manual_temp_alarm &&
			temp >= chip->temp_alarm_degc)) {
		reported_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		chip->health = POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (status & BQ28Z610_BATTERY_OVER_CHARGE_ALARM) {
		reported_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		chip->health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	} else {
		chip->health = POWER_SUPPLY_HEALTH_GOOD;
		chip->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		if (chip->soc_scale == 0)
			chip->soc = bq28z610_do_shift_round(soc,
				chip->soc_shift);
		else if (soc > chip->soc_scale)
			chip->soc = BQ28Z610_BATTERY_FULL;
		else
			chip->soc = (100 * bq28z610_do_shift_round(soc,
				chip->soc_shift)) / chip->soc_scale;


		if (chip->soc >= 100) {
			reported_status = POWER_SUPPLY_STATUS_FULL;
		} else if (chip->power_supply_status != BATTERY_CHARGING) {
			reported_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else if (status & BQ28Z610_BATTERY_DISCHARGING) {
			reported_status = POWER_SUPPLY_STATUS_DISCHARGING;
		} else {
			reported_status = POWER_SUPPLY_STATUS_CHARGING;
		}
	}

	if (chip->status != reported_status) {
		chip->status = reported_status;
		power_supply_changed(&chip->battery);
	}

	return reported_status;
}

static char *bq28z610_get_Manufacturer_Name(struct bq28z610_chip *chip)
{
	u32 read_data;
	int ret;

	ret = bq28z610_write_word(chip->client, BQ28Z610_MANUFACTURERACCESS,
				(BQ28Z610_MANUFACTURER_NAME >> 8) |
				((0x00FF & BQ28Z610_MANUFACTURER_NAME) << 8));
	if (ret < 0) {
		dev_err(&chip->client->dev,
				"%s: write ManufacturerAccess err %d\n",
				__func__, ret);
		return "UNKNOWN";
	}

	ret = bq28z610_read_block(chip->client, BQ28Z610_MANUFACTURERDATA,
			&read_data);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
		return "UNKNOWN";
	}

	read_data &= 0xff;

	if (read_data == 'S') return "SMP";
	else if (read_data == 'L') return "LGC";

	return "UNKNOWN";
}


static int bq28z610_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct bq28z610_chip *chip = container_of(psy,
				struct bq28z610_chip, battery);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		bq28z610_get_status_and_soc(chip);
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		bq28z610_get_vcell(chip);
		val->intval = chip->vcell*1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (chip->present) val->intval = chip->soc;
		else val->intval = 50;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		bq28z610_get_status_and_soc(chip);
		val->intval = chip->health;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = chip->capacity_level;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = bq28z610_get_temperature(chip);
		if (chip->present) val->intval = ret*10;
		else val->intval = 250;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->present;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret=bq28z610_get_current_now(chip);
		val->intval = chip->current_now * 1000;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = bq28z610_get_cycle_count(chip);
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = bq28z610_get_charging_current(chip);
		val->intval = ret * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq28z610_get_charging_voltage(chip);
		val->intval = ret * 1000;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = bq28z610_get_Manufacturer_Name(chip);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}



static int bq28z610_set_current_available(struct i2c_client *client)
{
	u16 status;
	int ret;

	ret = bq28z610_read_word(client, BQ28Z610_CONTROL_STATUS, &status);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}
	status |= BQ28Z610_CONTROL_STATUS_AUTO_CAL;
	ret = bq28z610_write_word(client, BQ28Z610_CONTROL_STATUS, status);
	ret = bq28z610_read_word(client, BQ28Z610_CONTROL_STATUS, &status);
	return 0;
}

u16 bq28z610_do_shift_round(u16 soc, int shift)
{
	int tmp;

	/* only support positve 10% shift */
	if (shift >= BQ28Z610_BATTERY_FULL/10)
		shift = BQ28Z610_BATTERY_FULL/10;
	if (shift < 0)
		shift = 0;
	/* do shifting and round soc to the nearest whole number  */
	tmp = (int) soc;
	tmp = ((tmp-shift)*BQ28Z610_BATTERY_FULL+50)
		/(BQ28Z610_BATTERY_FULL-shift);
	return (u16)((tmp > 0) ? tmp : 0);
}

static int bq28z610_online(struct bq28z610_chip *chip)
{
	u16 status;
	int ret;

	ret = bq28z610_read_word(chip->client, BQ28Z610_BATTERY_STATUS, &status);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
	        chip->present = 0;
		return ret;
	}

	chip->present = 1;

	return 0;
}


#ifdef BQ28Z610_DEBUG
static void bq28z610_update(struct bq28z610_chip *chip)
{
        int last_soc;
	pr_info("\n%s ENTER soc=%d, status = %s\n",
		__func__, chip->soc, chargingstatus[chip->status]);
	last_soc = chip->soc;

	bq28z610_get_charging_current(chip);
	bq28z610_get_current_now(chip);
	bq28z610_get_charging_voltage(chip);
	bq28z610_get_temperature(chip);
	bq28z610_get_vcell(chip);
	bq28z610_get_average_current(chip);
	bq28z610_get_time_to_empty(chip);
	bq28z610_get_time_to_full(chip);
	bq28z610_get_full_charge_capacity(chip);
	bq28z610_get_remaining_capacity(chip);
	bq28z610_get_max_load_current(chip);
	bq28z610_get_standyby_current(chip);
	bq28z610_get_status_and_soc(chip);
	bq28z610_get_cycle_count(chip);

	pr_info("%s EXIT soc=%d previous_soc = %d status = %s\n",
		__func__, chip->soc, last_soc,
		chargingstatus[chip->status]);

	schedule_delayed_work(&chip->work, BQ28Z610_DELAY);
}

static void bq28z610_work(struct work_struct *work)
{
	struct bq28z610_chip *chip;

	chip = container_of(work, struct bq28z610_chip, work.work);

        bq28z610_update(chip);
}
#endif

static enum power_supply_property bq28z610_battery_props[] = {
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

#ifdef CONFIG_OF
static struct bq28z610_platform_data *bq28z610_parse_dt(struct device *dev)
{
	u32 tmp;
	struct bq28z610_platform_data *pdata;
	struct device_node *np = dev->of_node;
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);
		
	pdata->model_data = devm_kzalloc(dev,
		sizeof(*(pdata->model_data)), GFP_KERNEL);
	if (!pdata->model_data)
		return ERR_PTR(-ENOMEM);

	of_property_read_string(np, "tz-name", &pdata->tz_name);
	
	if (!of_property_read_u32(np, "soc_shift", &tmp))
		pdata->model_data->soc_shift = (unsigned short)tmp;

	return pdata;
}
#else
static struct bq28z610_platform_data *bq28z610_parse_dt(struct device *dev)
{
	return NULL;
}
#endif /* CONFIG_OF */


static void bq28z610_power_work(struct work_struct *work)
{
	struct bq28z610_chip *chip;
	int status;

	chip = container_of(work, struct bq28z610_chip, power_work.work);
	status = bq28z610_get_status_and_soc(chip);
}

static int bq28z610_update_battery_status(struct battery_gauge_dev *bg_dev,
		enum battery_charger_status status)
{
	struct bq28z610_chip *chip = battery_gauge_get_drvdata(bg_dev);

	chip->power_supply_status = status;
	switch (status) {
	case BATTERY_CHARGING:
		/* fuel gauge can more then 1 sec to show status */
		cancel_delayed_work(&chip->power_work);
		schedule_delayed_work(&chip->power_work, 2*HZ);
		break;
	case BATTERY_CHARGING_DONE:
		chip->status = POWER_SUPPLY_STATUS_FULL;
		break;
	case BATTERY_DISCHARGING:
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		dev_err(&chip->client->dev, "%s: unknown battery event = %d\n",
			 __func__, status);
		break;
	}
	power_supply_changed(&chip->battery);
	return 0;
}

static int bq28z610_is_temp_alarm_manual(struct bq28z610_chip *chip)
{
	char manufacturer_smp[] = "SMP";
	char *battery_manufacturer;
	int ret;

	battery_manufacturer = bq28z610_get_Manufacturer_Name(chip);

	ret = strcmp(battery_manufacturer, manufacturer_smp) != 0;

	return ret;
}

static const struct regmap_config bq28z610_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= BQ28Z610_MAX_REGS,
};

static struct battery_gauge_ops bq28z610_bg_ops = {
	.update_battery_status = bq28z610_update_battery_status,
};

static struct battery_gauge_info bq28z610_bgi = {
	.cell_id = 0,
	.bg_ops = &bq28z610_bg_ops,
};

static int bq28z610_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bq28z610_chip *chip;
	int ret;

	pr_info("%s: ENTER\n", __func__);
	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;

	if (client->dev.of_node) {
		chip->pdata = bq28z610_parse_dt(&client->dev);
		if (IS_ERR(chip->pdata)) {
			kfree(chip);
			return PTR_ERR(chip->pdata);
		}
	} else {
		chip->pdata = client->dev.platform_data;
		if (!chip->pdata) {
			kfree(chip);
			return -ENODATA;
		}
	}
	chip->regmap = devm_regmap_init_i2c(client, &bq28z610_regmap_config);
	if (IS_ERR(chip->regmap)) {
		kfree(chip);
		ret = PTR_ERR(chip->regmap);
		dev_err(&client->dev, "regmap init failed with err %d\n", ret);
		return ret;
	}

	chip->shutdown_complete = 0;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->mutex);

	bq28z610_online(chip);
	bq28z610_set_current_available(client);

	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= bq28z610_get_property;
	chip->battery.properties	= bq28z610_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(bq28z610_battery_props);
	chip->status			= POWER_SUPPLY_STATUS_DISCHARGING;
	chip->soc			= 0;
	chip->soc_scale			= 0;
	chip->soc_shift			= chip->pdata->model_data->soc_shift;
	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto error;
	}

	bq28z610_bgi.tz_name = chip->pdata->tz_name;
	chip->bg_dev = battery_gauge_register(&client->dev, &bq28z610_bgi,
				chip);
	if (IS_ERR(chip->bg_dev)) {
		ret = PTR_ERR(chip->bg_dev);
		dev_err(&client->dev, "battery gauge register failed: %d\n",
			ret);
		goto bg_err;
	}

	chip->manual_temp_alarm = bq28z610_is_temp_alarm_manual(chip);
	if (chip->manual_temp_alarm)
		chip->temp_alarm_degc = 45;
	INIT_DEFERRABLE_WORK(&chip->power_work, bq28z610_power_work);

#ifdef BQ28Z610_DEBUG
	INIT_DEFERRABLE_WORK(&chip->work, bq28z610_work);
	schedule_delayed_work(&chip->work, 2*HZ);
#endif
	device_set_wakeup_capable(&client->dev, 1);
	pr_info("%s: probe DONE\n", __func__);

	return 0;

bg_err:
	power_supply_unregister(&chip->battery);
error:
	mutex_destroy(&chip->mutex);
	dev_err(&client->dev, "%s: failed -- ret = %d\n", __func__, ret);
	return ret;
}

static int bq28z610_remove(struct i2c_client *client)
{
	struct bq28z610_chip *chip = i2c_get_clientdata(client);

	if (client->irq)
		free_irq(client->irq, chip);
	battery_gauge_unregister(chip->bg_dev);
	power_supply_unregister(&chip->battery);
	cancel_delayed_work_sync(&chip->power_work);
#ifdef BQ28Z610_DEBUG
	cancel_delayed_work_sync(&chip->work);
#endif
	mutex_destroy(&chip->mutex);

	return 0;
}

static void bq28z610_shutdown(struct i2c_client *client)
{
	struct bq28z610_chip *chip = i2c_get_clientdata(client);

	if (client->irq)
		disable_irq(client->irq);
#ifdef BQ28Z610_DEBUG
	cancel_delayed_work_sync(&chip->work);
#endif
	cancel_delayed_work_sync(&chip->power_work);
	mutex_lock(&chip->mutex);
	chip->shutdown_complete = 1;
	mutex_unlock(&chip->mutex);
}

#ifdef CONFIG_PM_SLEEP
static int bq28z610_suspend(struct device *dev)
{
#ifdef BQ28Z610_DEBUG
	struct platform_device *pdev = to_platform_device(dev);
	struct bq28z610_chip *chip = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&chip->work);
#endif
	return 0;
}

static int bq28z610_resume(struct device *dev)
{
#ifdef BQ28Z610_DEBUG
	struct platform_device *pdev = to_platform_device(dev);
	struct bq28z610_chip *chip = platform_get_drvdata(pdev);

        /* Update imediately upon resume. */
	bq28z610_update(chip);
#endif
	return 0;
}
#endif /* CONFIG_PM */

static SIMPLE_DEV_PM_OPS(bq28z610_pm_ops, bq28z610_suspend, bq28z610_resume);

#ifdef CONFIG_OF
static const struct of_device_id bq28z610_dt_match[] = {
	{ .compatible = "ti,bq28z610" },
	{ },
};
MODULE_DEVICE_TABLE(of, bq28z610_dt_match);
#endif

static const struct i2c_device_id bq28z610_id[] = {
	{ "bq28z610", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bq28z610_id);

static struct i2c_driver bq28z610_i2c_driver = {
	.driver	= {
		.name	= "bq28z610",
		.of_match_table = of_match_ptr(bq28z610_dt_match),
		.pm = &bq28z610_pm_ops,
	},
	.probe		= bq28z610_probe,
	.remove		= bq28z610_remove,
	.id_table	= bq28z610_id,
	.shutdown	= bq28z610_shutdown,
};

static int __init bq28z610_init(void)
{
	return i2c_add_driver(&bq28z610_i2c_driver);
}
fs_initcall_sync(bq28z610_init);

static void __exit bq28z610_exit(void)
{
	i2c_del_driver(&bq28z610_i2c_driver);
}
module_exit(bq28z610_exit);

MODULE_AUTHOR("Philip Rakity <prakity@nvidia.com>");
MODULE_AUTHOR("Chandler Zhang <chazhang@nvidia.com>");
MODULE_DESCRIPTION("BQ28Z610 Fuel Gauge");
MODULE_LICENSE("GPL");


