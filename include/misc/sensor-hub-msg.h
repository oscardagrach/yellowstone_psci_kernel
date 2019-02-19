/*
 * sensor-hub-msg.h - sensorhub message protocol definition
 *
 * Copyright (c) 2014, BSQUARE CORPORATION.  All rights reserved.
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

#ifndef SENSOR_HUB_SENSOR_HUB_MSG_H_
#define SENSOR_HUB_SENSOR_HUB_MSG_H_


/* Sensor Hub Magic Number */
#define SENSOR_HUB_MAGIC	'T'
#define SENSOR_HUB_MAGIC1	't'

/* sensor_msg_type_t */
#define SENSOR_HUB_START	0x01
#define SENSOR_HUB_MSG		0x01
#define SENSOR_HUB_ACK		0x02
#define SENSOR_HUB_NAK		0x03
#define SENSOR_HUB_WAIT		0x04
#define SENSOR_HUB_VERSION	0x05
#define SENSOR_HUB_GET_HW_VERSION	0x06	/* This is a request from Tegra
						 * to Sensorhub. The reply will
						 * use the same code. */
#define SENSOR_HUB_SET_HW_VERSION	0x07	/* This is an instruction from
						 * Tegra to Sensorhub. There is
						 * no reply. */
#define SENSOR_HUB_POWER	0x08
#define SENSOR_HUB_SET_VCSEL_CURRENTS   0x09
#define SENSOR_HUB_SET_TIME	0x0a
#define SENSOR_HUB_END		0x0a
/* Torch HAL messages */
#define TORCH_HAL_START		0x10
#define TORCH_MODE		0x10
#define TORCH_SRC		0x11
#define TORCH_STROBE		0x12
#define TORCH_PULSE_WIDTH	0x13
#define TORCH_PULSE_DELAY	0x14
#define TORCH_POWER		0x15
#define TORCH_ENABLE		0x16
#define TORCH_DISABLE		0x17
#define TORCH_CHOOSE_DEVICE_B	0x18
#define TORCH_HAL_END		0x18
/* Camera HAL messages */
#define CAMERA_HAL_START	0x20
#define CAMERA_ENABLE		0x20
#define CAMERA_DISABLE		0x21
#define CAMERA_FRAMEINFO	0x22
#define CAMERA_TIMESTAMP	0x22 /* duplicated for compat */
#define CAMERA_RESET_FRAMECOUNT	0x23
#define CAMERA_HAL_END		0x23
/* Sensor HAL messages */
#define SENSOR_HAL_START	0x30
#define SENSOR_ENABLE		0x30
#define SENSOR_DISABLE		0x31
#define SENSOR_RATE		0x32
#define SENSOR_RANGE		0x33
#define SENSOR_ONE_SHOT		0x34
#define SENSOR_STREAM_ON	0x35
#define SENSOR_STREAM_OFF	0x36
/* Torch HAL messages cont'd */
#define TORCH_ON		0x37
#define TORCH_OFF		0x38
/* and back to the sensor HAL messages */
#define SENSOR_SELF_TEST	0x39
#define SENSOR_HAL_END		0x39

/* sensor_hub_device_t */
#define DEVICE_NONE		0x01
/* Camera devices */
#define DEVICE_REAR_CAMERA	0x02
#define DEVICE_FRONT_CAMERA	0x03
#define DEVICE_FISHEYE_CAMERA	0x04
#define DEVICE_OTHER_CAMERA	0x05
/* Torch devices */
#define DEVICE_WHITE_FLASH	0x06
#define DEVICE_FRONT_IR		0x07
#define DEVICE_REAR_IR		0x08
#define DEVICE_VCSEL		0x09
/* Sensor devices */
#define DEVICE_ACCEL		0x0A
#define DEVICE_GYRO		0x0B
#define DEVICE_MAG		0x0C
#define DEVICE_BARO		0x0D
#define DEVICE_BARO_TEMP	0x0E
#define DEVICE_CPU_TEMP		0x0F
#define DEVICE_IMU_TEMP		0x10
#define DEVICE_END		0x10

struct __attribute__ ((__packed__)) sensor_hub_msg_header_t {
	unsigned char magic;		/* 8bits mangic Number 'T' for Tango! */
	unsigned char msg_type;		/* 8bits from sensor_msg_type_t */
	unsigned char device;		/* 8bits from sensor_hub_device_t */
	unsigned char length;		/* 8bits coding the length of message to
					 * follow */
};

struct __attribute__ ((__packed__)) sensor_hub_hal_msg_t {
	unsigned char power_state;
};

struct __attribute__ ((__packed__)) sensor_hub_torch_hal_msg_t {
	union {
		uint16_t mode;		/* Can be torch_mode_none,
					 * torch_mode_torch,  or
					 * torch_mode_flash */
		uint16_t src;		/* Can be device_none,
					 * device_rear_camera,
					 * device_front_camera, or
					 * device_fisheye_camera */
		uint16_t strobe;	/* Can be torch_strobe_off,
					 * torch_strobe_once,
					 * torch_strobe_repeat,
					 * torch_strobe_on */
		uint16_t high_time;	/* MicroSeconds the torch is high
					 * during torch_strobe_once, and
					 * torch_strobe_repeat */
		uint16_t low_time;	/* MicroSeconds the torch is low
					 * during torch_strobe_repeat */
		uint16_t power;		/* Output power in mA */
	} param;
};

struct __attribute__ ((__packed__)) frameinfo {
	uint64_t timestamp;
	uint32_t frame_counter;
};

struct __attribute__ ((__packed__)) sensor_hub_camera_hal_msg_t {
	union {
		uint16_t power;
		struct frameinfo timestamp;
	} param;
};

struct __attribute__ ((__packed__)) sensor_hub_sensor_hal_msg_t {
	uint64_t timestamp;
	union {
		uint16_t accel[3];
		uint16_t gyro[3];
		uint16_t mag[3];
		int32_t baro[1];
		int16_t temp[1];
		int8_t gyro_result[1];
		uint16_t accel_results[6];
	} measurement;
};

struct __attribute__ ((__packed__)) sensor_hub_sensor_msg_t {
	uint32_t delay;
};

struct __attribute__ ((__packed__)) sensor_hub_msg_t {
	struct sensor_hub_msg_header_t header;
	union {
		struct sensor_hub_hal_msg_t sensor_hub;
		struct sensor_hub_torch_hal_msg_t torch;
		struct sensor_hub_camera_hal_msg_t camera;
		struct sensor_hub_sensor_hal_msg_t sensor;
		struct sensor_hub_sensor_msg_t sensor_msg;
	} msg;
};

#endif  /* SENSOR_HUB_SENSOR_HUB_MSG_H_ */

