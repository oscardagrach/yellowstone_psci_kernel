/*
 * Copyright (C) 2009 Samsung Electronics
 * Copyright (C) 2012-2013 NVIDIA CORPORATION
 * Minkyu Kang <mk7.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __BQ28Z610_BATTERY_H_
#define __BQ28Z610_BATTERY_H_
#include <linux/smb349-charger.h>

#define BQ28Z610_DATA_SIZE 64

struct bq28z610_battery_model {
	uint8_t rcomp;
	uint8_t soccheck_A;
	uint8_t soccheck_B;
	uint8_t bits;
	uint8_t alert_threshold;
	uint8_t one_percent_alerts;
	uint8_t alert_on_reset;
	uint16_t rcomp_seg;
	uint16_t hibernate;
	uint16_t vreset;
	uint16_t valert;
	uint16_t ocvtest;
	uint16_t soc_shift;
	uint8_t data_tbl[BQ28Z610_DATA_SIZE];
};

struct bq28z610_platform_data {
	struct bq28z610_battery_model *model_data;
	const char *tz_name;
};

u16 bq28z610_do_shift_round(u16 soc, int shift);

#if 0
#ifdef CONFIG_BATTERY_BQ28Z610
extern void bq28z610_battery_status(int status);
extern int bq28z610_check_battery(void);
#else
static inline void bq28z610_battery_status(int status) {}
static inline int bq28z610_check_battery(void) { return -ENODEV; }
#endif
#endif

#endif /* __MAX17048_BATTERY_H_ */


