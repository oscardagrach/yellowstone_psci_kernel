/*
 * arch/arm/mach-tegra/board-yellowstone-sensors.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <linux/pid_thermal_gov.h>
#include <linux/tegra-fuse.h>
#include <linux/of_platform.h>
#include <mach/edp.h>
#include <mach/io_dpd.h>
#include <media/camera.h>

#include <media/ov4682.h>
#include <media/ov7251.h>
#include <media/ov9762.h>

#include <linux/nct1008.h>

#include <linux/platform_device.h>
#include <media/soc_camera.h>
#include <media/soc_camera_platform.h>
#include <media/tegra_v4l2_camera.h>
#include <linux/generic_adc_thermal.h>
#include <linux/cm3217.h>
#include <misc/sh-ldisc.h>

#include <linux/platform/tegra/cpu-tegra.h>
#include "devices.h"
#include "board.h"
#include "board-common.h"
#include "board-yellowstone.h"
#include "tegra-board-id.h"

static struct tegra_io_dpd csia_io = {
	.name			= "CSIA",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 0,
};

static struct tegra_io_dpd csib_io = {
	.name			= "CSIB",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 1,
};

static struct tegra_io_dpd dsic_io = {
	.name			= "DSIC",
	.io_dpd_reg_index	= 1,
	.io_dpd_bit		= 8,
};

static struct tegra_io_dpd dsid_io = {
	.name			= "DSID",
	.io_dpd_reg_index	= 1,
	.io_dpd_bit		= 9,
};

static struct tegra_io_dpd csic_io = {
	.name			= "CSIC",
	.io_dpd_reg_index	= 1,
	.io_dpd_bit		= 10,
};

static struct tegra_io_dpd csid_io = {
	.name			= "CSID",
	.io_dpd_reg_index	= 1,
	.io_dpd_bit		= 11,
};

static struct tegra_io_dpd csie_io = {
	.name			= "CSIE",
	.io_dpd_reg_index	= 1,
	.io_dpd_bit		= 12,
};

static int yellowstone_ov4682_power_on(struct ov4682_power_rail *pw, unsigned char sh_device)
{
	/* disable CSIA/B IOs DPD mode to turn on front camera for yellowstone */
	tegra_io_dpd_disable(&csia_io);
	tegra_io_dpd_disable(&csib_io);

	sh_camera_power(sh_device, 1);

	return 1;
}

static int yellowstone_ov4682_power_off(struct ov4682_power_rail *pw, unsigned char sh_device)
{
	sh_camera_power(sh_device, 0);

	/* enable CSIA/B IOs DPD mode to turn off front camera for yellowstone */
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);

	return 1;
}

struct ov4682_platform_data yellowstone_ov4682_pdata = {
	.power_on = yellowstone_ov4682_power_on,
	.power_off = yellowstone_ov4682_power_off,
	.sh_device = DEVICE_REAR_CAMERA,
};

static int yellowstone_ov7251_power_on(struct ov7251_power_rail *pw, unsigned char sh_device)
{
	/* disable DSIC IOs DPD mode to turn on front
	 * camera for yellowstone */
	tegra_io_dpd_disable(&dsic_io);

	sh_camera_power(sh_device, 1);

	return 0;
}

static int yellowstone_ov7251_power_off(struct ov7251_power_rail *pw, unsigned char sh_device)
{
	sh_camera_power(sh_device, 0);

	/* enable DSIC IOs DPD mode to turn off front
	 * camera for yellowstone */
	tegra_io_dpd_enable(&dsic_io);

	return 0;
}

struct ov7251_platform_data yellowstone_ov7251_pdata = {
	.power_on = yellowstone_ov7251_power_on,
	.power_off = yellowstone_ov7251_power_off,
	.sh_device = DEVICE_FISHEYE_CAMERA,
};

static int yellowstone_ov9762_power_on(struct ov9762_power_rail *pw, unsigned char sh_device)
{
	int err;
	if (unlikely(WARN_ON(!pw || !pw->avdd_hv
		|| !pw->vdd_lv || !pw->dvdd))) {
		return -EFAULT;
	}

	/* disable CSIE IOs DPD mode to turn on front camera for yellowstone */
	tegra_io_dpd_disable(&csie_io);

	gpio_set_value(FCAM_PWDN, 0); /* TEGRA_GPIO_PBB4 */
	usleep_range(10, 20);

	err = regulator_enable(pw->avdd_hv); /*AVDD*/
	if (unlikely(err))
		goto ov9762_avdd_fail;

	err = regulator_enable(pw->vdd_lv); /*VDIG*/
	if (unlikely(err))
		goto ov9762_vdd_fail;

	err = regulator_enable(pw->dvdd); /*DOVDD*/
	if (unlikely(err))
		goto ov9762_dvdd_fail;

	gpio_set_value(FCAM_PWDN, 1); /* TEGRA_GPIO_PBB4 */

	/* Clock to device is controlled by sensorhub, as is
	 * timestamp streaming */
	sh_camera_power(sh_device, 1);
	usleep_range(300, 310);

	printk("ov9762 power on done PWDN:%d!\n",gpio_get_value(FCAM_PWDN));
	return 0;

ov9762_dvdd_fail:
ov9762_vdd_fail:
	regulator_disable(pw->avdd_hv);

ov9762_avdd_fail:
	gpio_set_value(FCAM_PWDN, 0);
	printk("ov9762 power on failed!\n");
	return -ENODEV;
}

static int yellowstone_ov9762_power_off(struct ov9762_power_rail *pw, unsigned char sh_device)
{
	if (unlikely(WARN_ON(!pw || !pw->avdd_hv
		|| !pw->vdd_lv || !pw->dvdd)))
		return -EFAULT;

	/* Turn off clock to device and timestamp streaming */
	sh_camera_power(sh_device, 0);

	usleep_range(100, 120);
	gpio_set_value(FCAM_PWDN, 0); /*TEGRA_GPIO_PBB4*/

	regulator_disable(pw->dvdd);
	regulator_disable(pw->vdd_lv);
	regulator_disable(pw->avdd_hv);

	/* enable CSIE IOs DPD mode to turn off front camera for yellowstone */
	tegra_io_dpd_enable(&csie_io);

	return 0;
}

struct ov9762_platform_data yellowstone_ov9762_pdata = {
	.power_on = yellowstone_ov9762_power_on,
	.power_off = yellowstone_ov9762_power_off,
	.sh_device = DEVICE_FRONT_CAMERA,
};

static struct camera_data_blob yellowstone_camera_lut[] = {
	{"yellowstone_ov4682_pdata", &yellowstone_ov4682_pdata},
	{"yellowstone_ov7251_pdata", &yellowstone_ov7251_pdata},
	{"yellowstone_ov9762_pdata", &yellowstone_ov9762_pdata},
	{},
};

void __init yellowstone_camera_auxdata(void *data)
{
	struct of_dev_auxdata *aux_lut = data;
	while (aux_lut && aux_lut->compatible) {
		if (!strcmp(aux_lut->compatible, "nvidia,tegra124-camera")) {
			pr_info("%s: update camera lookup table.\n", __func__);
			aux_lut->platform_data = yellowstone_camera_lut;
		}
		aux_lut++;
	}
}

static int yellowstone_camera_init(void)
{
	pr_debug("%s: ++\n", __func__);

	/**
	 * MIPI on Yellowstone
	 *
	 * OV4682 - 4 lanes MIPI interface --> CSIA(2 of 4) and CSIB(2 of 4) IOs
	 * OV7251 - 2 lanes MIPI interface --> DSIC(2 of 2) IOs
	 * OV9762 - 1 lane  MIPI interface --> CSIE(1 of 1) IOs
	 *
	 * Display - 4 lanes MIPI interface --> DSIA(4 of 4) IOs
	 *
	 * CSIC and CSID IOs are defunct on T124
	 */
	/* put CSIA/B/C/D/E IOs into DPD mode to
	 * save additional power for yellowstone
	 */
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	tegra_io_dpd_enable(&csic_io);
	tegra_io_dpd_enable(&csid_io);
	tegra_io_dpd_enable(&csie_io);

	/* put DSIC/D into DPD */
	tegra_io_dpd_enable(&dsic_io);
	tegra_io_dpd_enable(&dsid_io);

	return 0;
}

static struct pid_thermal_gov_params cpu_pid_params = {
	.max_err_temp = 4000,
	.max_err_gain = 1000,

	.gain_p = 1000,
	.gain_d = 0,

	.up_compensation = 15,
	.down_compensation = 15,
};

static struct thermal_zone_params cpu_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &cpu_pid_params,
};

static struct thermal_zone_params board_tzp = {
	.governor_name = "step_wise"
};

static struct nct1008_platform_data yellowstone_nct72_pdata = {
	.loc_name = "tegra",
	.supported_hwrev = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.offset = 0,
	.extended_range = true,

	.sensors = {
		[LOC] = {
			.tzp = &board_tzp,
			.shutdown_limit = 120, /* C */
			.passive_delay = 1000,
			.num_trips = 1,
			.trips = {
				{
					.cdev_type = "therm_est_activ",
					.trip_temp = 40000,
					.trip_type = THERMAL_TRIP_ACTIVE,
					.hysteresis = 1000,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.mask = 1,
				},
			},
		},
		[EXT] = {
			.tzp = &cpu_tzp,
			.shutdown_limit = 95, /* C */
			.passive_delay = 1000,
			.num_trips = 2,
			.trips = {
				{
					.cdev_type = "shutdown_warning",
					.trip_temp = 93000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.mask = 0,
				},
				{
					.cdev_type = "cpu-balanced",
					.trip_temp = 83000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.hysteresis = 1000,
					.mask = 1,
				},
			}
		}
	}
};

static struct i2c_board_info yellowstone_i2c_nct72_board_info[] = {
	{
		I2C_BOARD_INFO("nct72", 0x4c),
		.platform_data = &yellowstone_nct72_pdata,
		.irq = -1,
	},
};

static int yellowstone_nct72_init(void)
{
	s32 base_cp, shft_cp;
	u32 base_ft, shft_ft;
	int nct72_port = TEGRA_GPIO_PH7;
	int ret = 0;
	int i;
	struct thermal_trip_info *trip_state;

	/* raise NCT's thresholds if soctherm CP,FT fuses are ok */
	if ((tegra_fuse_calib_base_get_cp(&base_cp, &shft_cp) >= 0) &&
	    (tegra_fuse_calib_base_get_ft(&base_ft, &shft_ft) >= 0)) {
		yellowstone_nct72_pdata.sensors[EXT].shutdown_limit += 20;
		for (i = 0; i < yellowstone_nct72_pdata.sensors[EXT].num_trips;
			 i++) {
			trip_state = &yellowstone_nct72_pdata.sensors[EXT].trips[i];
			if (!strncmp(trip_state->cdev_type, "cpu-balanced",
					THERMAL_NAME_LENGTH)) {
				trip_state->cdev_type = "_none_";
				break;
			}
		}
	} else {
		tegra_platform_edp_init(
			yellowstone_nct72_pdata.sensors[EXT].trips,
			&yellowstone_nct72_pdata.sensors[EXT].num_trips,
					12000); /* edp temperature margin */
		tegra_add_cpu_vmax_trips(
			yellowstone_nct72_pdata.sensors[EXT].trips,
			&yellowstone_nct72_pdata.sensors[EXT].num_trips);
		tegra_add_tgpu_trips(
			yellowstone_nct72_pdata.sensors[EXT].trips,
			&yellowstone_nct72_pdata.sensors[EXT].num_trips);
		tegra_add_vc_trips(
			yellowstone_nct72_pdata.sensors[EXT].trips,
			&yellowstone_nct72_pdata.sensors[EXT].num_trips);
		tegra_add_core_vmax_trips(
			yellowstone_nct72_pdata.sensors[EXT].trips,
			&yellowstone_nct72_pdata.sensors[EXT].num_trips);
	}

	tegra_add_all_vmin_trips(yellowstone_nct72_pdata.sensors[EXT].trips,
		&yellowstone_nct72_pdata.sensors[EXT].num_trips);

	yellowstone_i2c_nct72_board_info[0].irq = gpio_to_irq(nct72_port);

	ret = gpio_request(nct72_port, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(nct72_port);
	if (ret < 0) {
		pr_info("%s: calling gpio_free(nct72_port)", __func__);
		gpio_free(nct72_port);
	}

	i2c_register_board_info(0, yellowstone_i2c_nct72_board_info,
	ARRAY_SIZE(yellowstone_i2c_nct72_board_info));

	return ret;
}

int __init yellowstone_sensors_init(void)
{
	yellowstone_nct72_init();
	yellowstone_camera_init();

	return 0;
}
