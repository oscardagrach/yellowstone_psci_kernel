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
#include <mach/edp.h>
#include <mach/io_dpd.h>
#include <media/camera.h>

//#include <media/ov4682.h>
//#include <media/ov7251.h>
//#include <media/ov9762.h>

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

/** proximity sensor */
static struct cm3217_platform_data yellowstone_cm3217_pdata = {
	.levels = {10, 160, 225, 320, 640, 1280, 2600, 5800, 8000, 10240},
	.golden_adc = 0,
	.power = 0,
};

static struct i2c_board_info yellowstone_i2c_cm3217_board_info[] = {
	{
		I2C_BOARD_INFO("cm3217", 0x10),
		.platform_data = &yellowstone_cm3217_pdata,
	},
};

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
	/* yellowstone_camera_init(); */
	yellowstone_nct72_init();
	i2c_register_board_info
		(0, yellowstone_i2c_cm3217_board_info,
			ARRAY_SIZE(yellowstone_i2c_cm3217_board_info));

	return 0;
}
