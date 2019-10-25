/*
 * bq2477x_extcon: Yellowstone Power supply detection through extcon.
 *
 * Copyright (c) 2012-2014, NVIDIA CORPORATION.  All rights reserved.
 * Philip Rakity <prakity@nvidia.com>
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/power/power_supply_extcon.h>
#include <linux/slab.h>
#include <linux/extcon.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/power/bq24773-charger.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#define DRIVER_NAME "power_bq2477x_extcon"

struct bq2477x_extcon {
	struct device *dev;
	struct device *parent;
	struct extcon_dev edev;
	int dock_usb3_gpio;
	int dock_usb3_active_high;
	int dock_12v_gpio;
	int dock_12v_gpio_active_high;
	struct delayed_work dock_work;
	struct regulator_dev *chg_rdev;
	struct regulator_desc chg_reg_desc;
	struct regulator_init_data chg_reg_init_data;
	struct regulator_consumer_supply chg_consumer_supply[4];
	int max_current_ua;
};

static const char * const tegra_bq2477x_extcon_cable[] = {
	"Dock-12V",
	NULL,
};

static void tegra_bq2477x_dock_helper(struct bq2477x_extcon *psy_extcon)
{
	int state;

	state = gpio_get_value(psy_extcon->dock_12v_gpio);

	pr_info("%s: dock_12v_gpio = %d\n", __func__, state);

	if (!psy_extcon->dock_12v_gpio_active_high)
		state = !state;

	if (state) {
		dev_info(psy_extcon->dev, "12V DOCK CONNECTED\n");
		extcon_set_cable_state(&psy_extcon->edev, "Dock-12V", true);
	} else {
		dev_info(psy_extcon->dev, "12V DOCK DISCONNECTED\n");
		extcon_set_cable_state(&psy_extcon->edev, "Dock-12V", false);
	}
}

static void tegra_bq2477x_set_dock_work(struct work_struct *work)
{
	struct bq2477x_extcon *psy_extcon = container_of(work,
						struct bq2477x_extcon,
						dock_work.work);

	tegra_bq2477x_dock_helper(psy_extcon);
}

static irqreturn_t bq2477x_dock_12v_irq(int irq, void *data)
{
	struct bq2477x_extcon *psy_extcon = data;

	cancel_delayed_work(&psy_extcon->dock_work);
	schedule_delayed_work(&psy_extcon->dock_work, msecs_to_jiffies(50));
	return IRQ_HANDLED;
}

static int bq2477x_set_charging_current(struct regulator_dev *rdev,
			int min_ua, int max_ua)
{
	struct bq2477x_extcon *psy_extcon = rdev_get_drvdata(rdev);

	if (max_ua > psy_extcon->chg_reg_init_data.constraints.max_uA)
		max_ua = psy_extcon->chg_reg_init_data.constraints.max_uA;
	psy_extcon->max_current_ua = max_ua;
	blocking_notifier_call_chain(&rdev->notifier, 1234, (void *) max_ua);
	return 0;
}

static int bq2477x_get_charging_current(struct regulator_dev *rdev)
{
	struct bq2477x_extcon *psy_extcon = rdev_get_drvdata(rdev);

	return psy_extcon->max_current_ua;
}


static struct regulator_ops bq2477x_tegra_regulator_ops = {
	.set_current_limit = bq2477x_set_charging_current,
	.get_current_limit = bq2477x_get_charging_current,
};


static int bq2477x_init_charger_regulator(struct bq2477x_extcon *psy_extcon,
		struct bq2477x_platform_data *pdata)
{
	int ret = 0;
	struct regulator_config rconfig = { };

	if (!pdata) {
		dev_err(psy_extcon->dev, "No charger platform data\n");
		return 0;
	}

	psy_extcon->chg_reg_desc.name  = "bq2477x-charger";
	psy_extcon->chg_reg_desc.ops   = &bq2477x_tegra_regulator_ops;
	psy_extcon->chg_reg_desc.type  = REGULATOR_CURRENT;
	psy_extcon->chg_reg_desc.owner = THIS_MODULE;

	psy_extcon->chg_reg_init_data.supply_regulator	= NULL;
	psy_extcon->chg_reg_init_data.regulator_init	= NULL;

	psy_extcon->chg_reg_init_data.num_consumer_supplies = 4;
	psy_extcon->chg_consumer_supply[0].supply = "usb_bat_chg";
	psy_extcon->chg_consumer_supply[0].dev_name = "tegra-udc.0";
	psy_extcon->chg_consumer_supply[1].supply = "usb_bat_chg";
	psy_extcon->chg_consumer_supply[1].dev_name = "1-006a";
	psy_extcon->chg_consumer_supply[2].supply = "dock-12v";
	psy_extcon->chg_consumer_supply[2].dev_name = "bq2477x";
	psy_extcon->chg_consumer_supply[3].supply = "usb";
	psy_extcon->chg_consumer_supply[3].dev_name = "bq2477x";
	psy_extcon->chg_reg_init_data.consumer_supplies	=
		(struct regulator_consumer_supply *)
				&psy_extcon->chg_consumer_supply;
	psy_extcon->chg_reg_init_data.driver_data	= psy_extcon;
	psy_extcon->chg_reg_init_data.constraints.name	= "bq2477x-charger";
	psy_extcon->chg_reg_init_data.constraints.min_uA	= 0;
	psy_extcon->chg_reg_init_data.constraints.max_uA	=
			pdata->dock_max_ua;
	psy_extcon->chg_reg_init_data.constraints.ignore_current_constraint_init =
							true;
	psy_extcon->chg_reg_init_data.constraints.valid_modes_mask =
						REGULATOR_MODE_NORMAL |
						REGULATOR_MODE_STANDBY;

	psy_extcon->chg_reg_init_data.constraints.valid_ops_mask =
						REGULATOR_CHANGE_MODE |
						REGULATOR_CHANGE_STATUS |
						REGULATOR_CHANGE_CURRENT;

	rconfig.dev = psy_extcon->dev;
	rconfig.of_node = NULL;
	rconfig.init_data = &psy_extcon->chg_reg_init_data;
	rconfig.driver_data = psy_extcon;
	psy_extcon->chg_rdev = devm_regulator_register(psy_extcon->dev,
				&psy_extcon->chg_reg_desc, &rconfig);
	if (IS_ERR(psy_extcon->chg_rdev)) {
		ret = PTR_ERR(psy_extcon->chg_rdev);
		dev_err(psy_extcon->dev,
			"usb_bat_chg regulator register failed %d\n", ret);
	}
	return ret;
}

static int bq2477x_extcon_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct bq2477x_extcon *psy_extcon;
	struct bq2477x_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data, exiting..\n");
		return -ENODEV;
	}

	psy_extcon = devm_kzalloc(&pdev->dev, sizeof(struct bq2477x_extcon), GFP_KERNEL);
	if (!psy_extcon) {
		dev_err(&pdev->dev, "failed to allocate memory status\n");
		return -ENOMEM;
	}

	psy_extcon->dev = &pdev->dev;
	psy_extcon->parent = pdev->dev.parent;
	dev_set_drvdata(&pdev->dev, psy_extcon);

	psy_extcon->dock_usb3_gpio = pdata->dock_usb3_gpio;
	psy_extcon->dock_usb3_active_high = pdata->dock_usb3_active_high;
	psy_extcon->dock_12v_gpio = pdata->dock_12v_gpio;
	psy_extcon->dock_12v_gpio_active_high =
			pdata->dock_12v_gpio_active_high;

	dev_info(&pdev->dev, "%s: dock_usb3_gpio = %d, dock_usb3_active_high = %d\n",
		__func__,
		psy_extcon->dock_usb3_gpio, psy_extcon->dock_usb3_active_high);
	
	dev_info(&pdev->dev, "%s: dock_12v_gpio = %d, dock_12v_gpio_active_high = %d\n",
		__func__, psy_extcon->dock_12v_gpio,
		psy_extcon->dock_12v_gpio_active_high);

	psy_extcon->edev.name = DRIVER_NAME;
	psy_extcon->edev.supported_cable =
			(const char **) tegra_bq2477x_extcon_cable;
	psy_extcon->edev.dev.parent = &pdev->dev;
	ret = extcon_dev_register(&psy_extcon->edev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register extcon device\n");
		return ret;
	}

	INIT_DELAYED_WORK(&psy_extcon->dock_work, tegra_bq2477x_set_dock_work);

	ret = gpio_request(psy_extcon->dock_12v_gpio, "Dock 12V Detect");
	if (ret) {
		/* not fatal as some units share gpio Dock-USB3 Detect */
		dev_err(&pdev->dev, "gpio_request fail for Dock 12V Detect: %d\n",
			ret);
	}
	ret = request_irq(gpio_to_irq(psy_extcon->dock_12v_gpio),
		bq2477x_dock_12v_irq,
		IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"Dock 12V Detect", psy_extcon);
	if (ret < 0) {
		dev_warn(psy_extcon->dev, "request IRQ %d fail, err = %d\n",
			gpio_to_irq(psy_extcon->dock_12v_gpio), ret);
		dev_info(psy_extcon->dev,
			"Supporting bq driver without 12v detect interrupt\n");
		ret = 0;
	}
	enable_irq_wake(gpio_to_irq(psy_extcon->dock_12v_gpio));

	ret = bq2477x_init_charger_regulator(psy_extcon, pdata);
	if (ret < 0) {
		dev_err(psy_extcon->dev,
			"Charger regualtor init failed %d\n", ret);
		goto econ_err;
	}

	cancel_delayed_work(&psy_extcon->dock_work);
	schedule_delayed_work(&psy_extcon->dock_work, msecs_to_jiffies(6000));
	dev_info(&pdev->dev, "%s: success\n", __func__);
	return 0;

econ_err:
	kfree(psy_extcon);
	return ret;
}

static int bq2477x_extcon_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver bq2477x_extcon_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = bq2477x_extcon_probe,
	.remove = bq2477x_extcon_remove,

};

static int __init bq2477x_extcon_init(void)
{
	return platform_driver_register(&bq2477x_extcon_driver);
}

static void __exit bq2477x_extcon_exit(void)
{
	platform_driver_unregister(&bq2477x_extcon_driver);
}

subsys_initcall(bq2477x_extcon_init);
module_exit(bq2477x_extcon_exit);

MODULE_DESCRIPTION("Yellowstone Power supply detection through extcon driver");
MODULE_AUTHOR("Philip Rakity <prakity@nvidia.com>");
MODULE_LICENSE("GPL v2");

