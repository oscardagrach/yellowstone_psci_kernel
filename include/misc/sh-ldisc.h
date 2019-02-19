#ifndef SENSOR_HUB_SH_LDISC_H
#define SENSOR_HUB_SH_LDISC_H

#include <linux/ioctl.h>

#include "sensor-hub-msg.h"

#define SENSOR_HUB_IOCTL_GET_FRAMEINFO		_IOWR(SENSOR_HUB_MAGIC, 0, \
	struct frameinfo)

int sh_camera_power(int cam_id, int power);
int sh_get_frameinfo(unsigned char camera_id, struct frameinfo *info);

#if defined(__KERNEL__)

#define SH_SPI_NAME "sensor_hub_spi"

struct sensor_hub_platform_data {
	int gpio_reset_n;
	int gpio_pwr_en;
	int gpio_boot_cfg0;
	int gpio_boot_cfg1;

	void (*power_on)(const struct sensor_hub_platform_data *pdata);
	void (*power_off)(const struct sensor_hub_platform_data *pdata);
};
#endif /** __KERNEL__ */

#endif

