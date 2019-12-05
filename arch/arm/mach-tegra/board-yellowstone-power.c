/*
 * arch/arm/mach-tegra/board-yellowstone-power.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/io.h>
#include <mach/edp.h>
#include <mach/irqs.h>
#include <linux/edp.h>
#include <linux/platform_data/tegra_edp.h>
#include <linux/pid_thermal_gov.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/tegra-dfll-bypass-regulator.h>
#include <linux/tegra-fuse.h>
#include <linux/tegra-pmc.h>
#include <linux/power/power_supply_extcon.h>
#include <linux/power_supply.h>
#include <linux/power/bq24773-charger.h>
#include <linux/delay.h>
#include <linux/pinctrl/pinconf-tegra.h>

#include <asm/mach-types.h>
#include <linux/tegra_soctherm.h>

#include "pm.h"
#include <linux/platform/tegra/dvfs.h>
#include "board.h"
#include <linux/platform/tegra/common.h>
#include "tegra-board-id.h"
#include "board-pmu-defines.h"
#include "board-common.h"
#include "board-yellowstone.h"
#include "board-pmu-defines.h"
#include "devices.h"
#include "iomap.h"
#include <linux/platform/tegra/tegra_cl_dvfs.h>

#define E1735_EMULATE_E1767_SKU	1001
static u32 tegra_chip_id;
static struct tegra_suspend_platform_data yellowstone_suspend_data = {
	.cpu_timer      = 500,
	.cpu_off_timer  = 300,
	.cpu_suspend_freq = 1044000,
	.suspend_mode   = TEGRA_SUSPEND_LP0,
	.core_timer     = 0x157e,
	.core_off_timer = 10,
	.corereq_high   = true,
	.sysclkreq_high = true,
	.cpu_lp2_min_residency = 1000,
	.min_residency_vmin_fmin = 1000,
	.min_residency_ncpu_fast = 8000,
	.min_residency_ncpu_slow = 5000,
	.min_residency_mclk_stop = 5000,
	.min_residency_crail = 20000,
#ifdef CONFIG_TEGRA_LP1_LOW_COREVOLTAGE
	.lp1_lowvolt_support = true,
	.i2c_base_addr = TEGRA_I2C5_BASE,
	.pmuslave_addr = 0xB0,
	.core_reg_addr = 0x33,
	.lp1_core_volt_low_cold = 0x33,
	.lp1_core_volt_low = 0x33,
	.lp1_core_volt_high = 0x42,
#endif
};

/* Working theory. 5V -> 12V converter operating at 92% efficiency.
** According to TI WEBENCH, TPS55330 boost 5V in -> 12V out.
** Assuming 90% efficency for the boost converter
** Assuming 90% efficency for the charger converting 12V to 8700
** (2 batteries rated at 4.35v each = 8.7volts)
** Empirical observations.
** iin > ichg or the charger won't limit system current properly NOT
** iin >= ~512ma as according to bs24770.pdf page 21 wrt Setting Input Current.
** When programming over I2C, current steps are by 64ma so we & with ~0x3F
*/
static struct bq2477x_charge_zone yellowstone_bq2477x_charge_zones[] = {
	/* Cold temperature charging. */
	{
		.min_temp = -150,	/* -15.0 C */
		.max_temp = 20,		/* 2.0 C */
		.charge_voltage = 0,	/* 0V */
		.charge_current = 0,	/* 0mA */
	},
	/* Nominal temperature charging. */
	{
		.min_temp = 20,		/* 2.0 C */
		.max_temp = 450,	/* 45.0 C */
		.charge_voltage = 8704, /* 8.704V */
		.charge_current = 2240, /* 2240mA */
	},
	/* Overtemp shutdown. */
	{
		.min_temp = 450,	/* 45.0 C */
		.max_temp = 999,	/* 99.9 C */
		.charge_voltage = 0,	/* 0V */
		.charge_current = 0,	/* 0mA */
	}
};

struct bq2477x_platform_data yellowstone_bq2477x_pdata = {
	.dac_v			= 8704,
	.dac_minsv		= 6144,
	.extcon_dock_name	= "power_bq2477x_extcon",
	.max_charge_ua		= 2440000,
	.dock_max_ua		= 2000000,
	.wdt_refresh_timeout	= 40,
	.disable_vbus_12v_boost_gpio = TEGRA_GPIO_PBB7,	/* 12v boost disable */
	.dock_12v_gpio = TEGRA_GPIO_PS0,
	.acok_gpio = TEGRA_GPIO_PJ0,
	.charge_table = yellowstone_bq2477x_charge_zones,
};

static struct platform_device yellowstone_bq2477x_extcon = {
	.name	= "power_bq2477x_extcon",
	.id	= -1,
	.dev	= {
		.platform_data = &yellowstone_bq2477x_pdata,
	},
};
static struct i2c_board_info __initdata bq2477x_boardinfo[] = {
	{
		I2C_BOARD_INFO("bq2477x", 0x6A),
		.platform_data	= &yellowstone_bq2477x_pdata,
	},
};

/************************ ARDBEG CL-DVFS DATA *********************/
#define E1735_CPU_VDD_MAP_SIZE		33
#define E1735_CPU_VDD_MIN_UV		752000
#define E1735_CPU_VDD_STEP_UV		16000
#define E1735_CPU_VDD_STEP_US		80
#define E1735_CPU_VDD_BOOT_UV		1248000
#define E1735_CPU_VDD_IDLE_MA		5000
#define YELLOWSTONE_DEFAULT_CVB_ALIGNMENT	10000

#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
/* E1735 board parameters for cpu dfll */
static struct tegra_cl_dvfs_cfg_param e1735_cl_dvfs_param = {
	.sample_rate = 50000,

	.force_mode = TEGRA_CL_DVFS_FORCE_FIXED,
	.cf = 10,
	.ci = 0,
	.cg = 2,

	.droop_cut_value = 0xF,
	.droop_restore_ramp = 0x0,
	.scale_out_ramp = 0x0,
};

/* E1735 dfll bypass device for legacy dvfs control */
static struct regulator_consumer_supply e1735_dfll_bypass_consumers[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};
DFLL_BYPASS(e1735,
	    E1735_CPU_VDD_MIN_UV, E1735_CPU_VDD_STEP_UV, E1735_CPU_VDD_BOOT_UV,
	    E1735_CPU_VDD_MAP_SIZE, E1735_CPU_VDD_STEP_US, TEGRA_GPIO_PX2);

static struct tegra_cl_dvfs_platform_data e1735_cl_dvfs_data = {
	.dfll_clk_name = "dfll_cpu",
	.pmu_if = TEGRA_CL_DVFS_PMU_PWM,
	.u.pmu_pwm = {
		.pwm_rate = 12750000,
		.min_uV = E1735_CPU_VDD_MIN_UV,
		.step_uV = E1735_CPU_VDD_STEP_UV,
		.out_gpio = TEGRA_GPIO_PS5,
		.out_enable_high = false,
#ifdef CONFIG_REGULATOR_TEGRA_DFLL_BYPASS
		.dfll_bypass_dev = &e1735_dfll_bypass_dev,
#endif
	},

	.cfg_param = &e1735_cl_dvfs_param,
};

#ifdef CONFIG_REGULATOR_TEGRA_DFLL_BYPASS
static void e1735_suspend_dfll_bypass(void)
{
	__gpio_set_value(TEGRA_GPIO_PS5, 1); /* tristate external PWM buffer */
}

static void e1735_resume_dfll_bypass(void)
{
	__gpio_set_value(TEGRA_GPIO_PS5, 0); /* enable PWM buffer operations */
}

static void e1767_configure_dvfs_pwm_tristate(const char *gname, int tristate)
{
	struct pinctrl_dev *pctl_dev = NULL;
	unsigned long config;
	int ret;

	pctl_dev = tegra_get_pinctrl_device_handle();
	if (!pctl_dev)
		return;

	config = TEGRA_PINCONF_PACK(TEGRA_PINCONF_PARAM_TRISTATE, tristate);
	ret = pinctrl_set_config_for_group_name(pctl_dev, gname, config);
	if (ret < 0)
		pr_err("%s(): ERROR: Not able to set %s to TRISTATE %d: %d\n",
			__func__, gname, tristate, ret);
}

static void e1767_suspend_dfll_bypass(void)
{
	e1767_configure_dvfs_pwm_tristate("dvfs_pwm_px0", TEGRA_PIN_ENABLE);
}

static void e1767_resume_dfll_bypass(void)
{
	e1767_configure_dvfs_pwm_tristate("dvfs_pwm_px0", TEGRA_PIN_DISABLE);
}
#endif

static int __init yellowstone_cl_dvfs_init(struct board_info *pmu_board_info)
{
	u16 pmu_board_id = pmu_board_info->board_id;
	struct tegra_cl_dvfs_platform_data *data = NULL;

	if (pmu_board_id == BOARD_E1735) {
		bool e1767 = pmu_board_info->sku == E1735_EMULATE_E1767_SKU;
		struct device_node *dn = of_find_compatible_node(
			NULL, NULL, "nvidia,tegra124-dfll");
		/*
		 * Ardbeg platforms with E1735 PMIC module maybe used with
		 * different DT variants. Some of them include CL-DVFS data
		 * in DT, some - not. Check DT here, and continue with platform
		 * device registration only if DT DFLL node is not present.
		 */
		if (dn) {
			bool available = of_device_is_available(dn);
			of_node_put(dn);

			if (available) {
				return 0;
			}
		}

		data = &e1735_cl_dvfs_data;

		data->u.pmu_pwm.pinctrl_dev = tegra_get_pinctrl_device_handle();
		if (!data->u.pmu_pwm.pinctrl_dev)
			return -EINVAL;

		data->u.pmu_pwm.pwm_pingroup =
				pinctrl_get_selector_from_group_name(
					data->u.pmu_pwm.pinctrl_dev,
					"dvfs_pwm_px0");
		if (data->u.pmu_pwm.pwm_pingroup < 0) {
			pr_err("%s: Tegra pin dvfs_pwm_px0 not found\n", __func__);
			return -EINVAL;
		}

		data->u.pmu_pwm.pwm_bus = e1767 ?
			TEGRA_CL_DVFS_PWM_1WIRE_DIRECT :
			TEGRA_CL_DVFS_PWM_1WIRE_BUFFER;

		if (data->u.pmu_pwm.dfll_bypass_dev) {
			platform_device_register(
				data->u.pmu_pwm.dfll_bypass_dev);
		} else {
			(void)e1735_dfll_bypass_dev;
		}
	}

	if (data) {
		data->flags = TEGRA_CL_DVFS_DYN_OUTPUT_CFG;
		tegra_cl_dvfs_device.dev.platform_data = data;
		platform_device_register(&tegra_cl_dvfs_device);
	}
	return 0;
}
#else
static inline int yellowstone_cl_dvfs_init(struct board_info *pmu_board_info)
{ return 0; }
#endif

static void yellowstone_charger_init(void)
{
	int ret = 0;

	ret = gpio_request(TEGRA_GPIO_PK5, "bq2477x-charger");
	if (ret < 0) {
		pr_err("%s: charger_enable TEGRA_GPIO_PK5 request failed\n",
			__func__);
	} else {
		ret = gpio_direction_output(TEGRA_GPIO_PK5, 1);
		if (ret < 0)
			pr_err("%s: TEGRA_GPIO_PK5 direction failed\n",
				__func__);
	}

	msleep(20);

	platform_device_register(&yellowstone_bq2477x_extcon);
	i2c_register_board_info(1, bq2477x_boardinfo,
		ARRAY_SIZE(bq2477x_boardinfo));
}

int __init yellowstone_rail_alignment_init(void)
{
	struct board_info pmu_board_info;

	tegra_get_pmu_board_info(&pmu_board_info);

	if (pmu_board_info.board_id == BOARD_E1735)
		tegra12x_vdd_cpu_align(E1735_CPU_VDD_STEP_UV,
				       E1735_CPU_VDD_MIN_UV);
	else
		tegra12x_vdd_cpu_align(YELLOWSTONE_DEFAULT_CVB_ALIGNMENT, 0);
	return 0;
}

int __init yellowstone_regulator_init(void)
{
	struct board_info pmu_board_info;
	tegra_get_pmu_board_info(&pmu_board_info);

	yellowstone_charger_init();

	yellowstone_cl_dvfs_init(&pmu_board_info);
	return 0;
}

int __init yellowstone_suspend_init(void)
{
	struct board_info pmu_board_info;

	tegra_get_pmu_board_info(&pmu_board_info);

	if (pmu_board_info.board_id == BOARD_E1735) {
		struct tegra_suspend_platform_data *data = &yellowstone_suspend_data;
		if (pmu_board_info.sku != E1735_EMULATE_E1767_SKU) {
			data->cpu_timer = 2000;
			data->crail_up_early = true;
#ifdef CONFIG_REGULATOR_TEGRA_DFLL_BYPASS
			data->suspend_dfll_bypass = e1735_suspend_dfll_bypass;
			data->resume_dfll_bypass = e1735_resume_dfll_bypass;
		} else {
			data->suspend_dfll_bypass = e1767_suspend_dfll_bypass;
			data->resume_dfll_bypass = e1767_resume_dfll_bypass;
#endif
		}
	}

	tegra_init_suspend(&yellowstone_suspend_data);
	return 0;
}

static struct pid_thermal_gov_params soctherm_pid_params = {
	.max_err_temp = 9000,
	.max_err_gain = 1000,

	.gain_p = 1000,
	.gain_d = 0,

	.up_compensation = 20,
	.down_compensation = 20,
};

static struct thermal_zone_params soctherm_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &soctherm_pid_params,
};

static struct tegra_thermtrip_pmic_data tpdata_palmas = {
	.reset_tegra = 1,
	.pmu_16bit_ops = 0,
	.controller_type = 0,
	.pmu_i2c_addr = 0x58,
	.i2c_controller_id = 4,
	.poweroff_reg_addr = 0xa0,
	.poweroff_reg_data = 0x0,
};

/*
 * @PSKIP_CONFIG_NOTE: For T132, throttling config of PSKIP is no longer
 * done in soctherm registers. These settings are now done via registers in
 * denver:ccroc module which are at a different register offset. More
 * importantly, there are _only_ three levels of throttling: 'low',
 * 'medium' and 'heavy' and are selected via the 'throttling_depth' field
 * in the throttle->devs[] section of the soctherm config. Since the depth
 * specification is per device, it is necessary to manually make sure the
 * depths specified alongwith a given level are the same across all devs,
 * otherwise it will overwrite a previously set depth with a different
 * depth. We will refer to this comment at each relevant location in the
 * config sections below.
 */
static struct soctherm_platform_data yellowstone_soctherm_data = {
	.oc_irq_base = TEGRA_SOC_OC_IRQ_BASE,
	.num_oc_irqs = TEGRA_SOC_OC_NUM_IRQ,
	.therm = {
		[THERM_CPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 101000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 99000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "cpu-balanced",
					.trip_temp = 90000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
		},
		[THERM_GPU] = {
			.zone_enable = true,
			.passive_delay = 1000,
			.hotspot_offset = 6000,
			.num_trips = 3,
			.trips = {
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 101000,
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "tegra-heavy",
					.trip_temp = 99000,
					.trip_type = THERMAL_TRIP_HOT,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
				{
					.cdev_type = "gpu-balanced",
					.trip_temp = 90000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
		},
		[THERM_MEM] = {
			.zone_enable = true,
			.num_trips = 1,
			.trips = {
				{
					.cdev_type = "tegra-shutdown",
					.trip_temp = 101000, /* = GPU shut */
					.trip_type = THERMAL_TRIP_CRITICAL,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
				},
			},
			.tzp = &soctherm_tzp,
		},
		[THERM_PLL] = {
			.zone_enable = true,
			.tzp = &soctherm_tzp,
		},
	},
	.throttle = {
		[THROTTLE_HEAVY] = {
			.priority = 100,
			.devs = {
				[THROTTLE_DEV_CPU] = {
					.enable = true,
					.depth = 80,
					/* see @PSKIP_CONFIG_NOTE */
					.throttling_depth = "heavy_throttling",
				},
				[THROTTLE_DEV_GPU] = {
					.enable = true,
					.throttling_depth = "heavy_throttling",
				},
			},
		},
	},
};

struct soctherm_throttle baseband_throttle = {
	.throt_mode = BRIEF,
	.polarity = SOCTHERM_ACTIVE_HIGH,
	.priority = 50,
	.devs = {
		[THROTTLE_DEV_CPU] = {
			.enable = true,
			.depth = 50,
		},
		[THROTTLE_DEV_GPU] = {
			.enable = true,
			.throttling_depth = "medium_throttling",
		},
	},
};

int __init yellowstone_soctherm_init(void)
{
	const int t12x_edp_temp_margin = 7000;
	int cpu_edp_temp_margin, gpu_edp_temp_margin;
	int cp_rev, ft_rev;

	struct board_info pmu_board_info = {0};
	struct board_info board_info = {0};
	enum soctherm_therm_id therm_cpu = THERM_CPU;

	tegra_get_board_info(&board_info);
	tegra_chip_id = tegra_get_chip_id();

	cp_rev = tegra_fuse_calib_base_get_cp(NULL, NULL);
	ft_rev = tegra_fuse_calib_base_get_ft(NULL, NULL);

	cpu_edp_temp_margin = t12x_edp_temp_margin;
	gpu_edp_temp_margin = t12x_edp_temp_margin;

	/* do this only for supported CP,FT fuses */
	if ((cp_rev >= 0) && (ft_rev >= 0)) {
		tegra_platform_edp_init(
			yellowstone_soctherm_data.therm[therm_cpu].trips,
			&yellowstone_soctherm_data.therm[therm_cpu].num_trips,
			cpu_edp_temp_margin);
		tegra_platform_gpu_edp_init(
			yellowstone_soctherm_data.therm[THERM_GPU].trips,
			&yellowstone_soctherm_data.therm[THERM_GPU].num_trips,
			gpu_edp_temp_margin);
		tegra_add_cpu_vmax_trips(
			yellowstone_soctherm_data.therm[therm_cpu].trips,
			&yellowstone_soctherm_data.therm[therm_cpu].num_trips);
		tegra_add_tgpu_trips(
			yellowstone_soctherm_data.therm[THERM_GPU].trips,
			&yellowstone_soctherm_data.therm[THERM_GPU].num_trips);
		tegra_add_vc_trips(
			yellowstone_soctherm_data.therm[therm_cpu].trips,
			&yellowstone_soctherm_data.therm[therm_cpu].num_trips);
		tegra_add_core_vmax_trips(
			yellowstone_soctherm_data.therm[THERM_PLL].trips,
			&yellowstone_soctherm_data.therm[THERM_PLL].num_trips);
	}

	yellowstone_soctherm_data.tshut_pmu_trip_data = &tpdata_palmas;

	/* enable baseband OC if Bruce modem is enabled */
	if (tegra_get_modem_id() == TEGRA_BB_BRUCE) {
		/* enable baseband OC unless board has voltage comparator */
		int board_has_vc;

		board_has_vc = (pmu_board_info.board_id == BOARD_P1761)
			&& (pmu_board_info.fab >= BOARD_FAB_A02);

		if (!board_has_vc)
			memcpy(&yellowstone_soctherm_data.throttle[THROTTLE_OC3],
			       &baseband_throttle,
			       sizeof(baseband_throttle));
	}

	return tegra_soctherm_init(&yellowstone_soctherm_data);
}
