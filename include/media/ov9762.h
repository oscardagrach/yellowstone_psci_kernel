/*
 * ov7251.h - ov7251 sensor driver
 *
 * Copyright (c) 2014 NVIDIA Corporation.  All rights reserved.
 *
 * Contributors:
 *  Jerry Chang <jerchang@nvidia.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef __OV9762_H__
#define __OV9762_H__

#include <linux/ioctl.h>
#include <misc/sensor-hub-msg.h>

#define OV9762_IOCTL_SET_MODE	_IOW('o', 1, struct ov9762_mode)
#define OV9762_IOCTL_SET_FRAME_LENGTH	_IOW('o', 2, __u32)
#define OV9762_IOCTL_SET_COARSE_TIME	_IOW('o', 3, __u32)
#define OV9762_IOCTL_SET_GAIN	_IOW('o', 4, __u16)
#define OV9762_IOCTL_GET_STATUS	_IOR('o', 5, __u8)
#define OV9762_IOCTL_SET_GROUP_HOLD	_IOW('o', 6, struct ov9762_grouphold)
#define OV9762_IOCTL_GET_FUSEID	_IOR('o', 7, struct ov9762_sensordata)
#define OV9762_IOCTL_START_STREAM _IO('o', 8)
#define OV9762_IOCTL_GET_FRAMEINFO _IOWR('o', 9, struct frameinfo)

struct ov9762_sensordata {
	__u32 fuse_id_size;
	__u8 fuse_id[16];
};

struct ov9762_mode {
	int xres;
	int yres;
	int fps;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
};

struct ov9762_grouphold {
	__u32	frame_length;
	__u8	frame_length_enable;
	__u32	coarse_time;
	__u8	coarse_time_enable;
	__s32	gain;
	__u8	gain_enable;
};

#ifdef __KERNEL__
struct ov9762_power_rail {
	struct regulator *avdd_hv;
	struct regulator *vdd_lv;
	struct regulator *dvdd;
};

struct ov9762_platform_data {
	int (*power_on)(struct ov9762_power_rail *pw, unsigned char sh_device);
	int (*power_off)(struct ov9762_power_rail *pw, unsigned char sh_device);
	const char *mclk_name;
	unsigned char sh_device;
};
#endif /* __KERNEL__ */

#endif /* __OV9762_H__ */
