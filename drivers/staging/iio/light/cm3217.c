/* Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/light/ls_sysfs.h>
#include <linux/iio/light/ls_dt.h>
#ifdef CONFIG_MACH_YELLOWSTONE
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/rtc.h>
#endif

/* IT = Integration Time.  The amount of time the photons hit the sensor.
 * STEP = the value from HW which is the photon count during IT.
 * LUX = STEP * (CM3217_RESOLUTION_STEP / IT) / CM3217_RESOLUTION_DIVIDER
 * The above LUX reported as LUX * CM3217_INPUT_LUX_DIVISOR.
 * The final value is done in user space to get a float value of
 * LUX / CM3217_INPUT_LUX_DIVISOR.
 */
#define CM3217_NAME			"cm3217"
#define CM3217_I2C_ADDR_CMD1_WR		(0x10)
#define CM3217_I2C_ADDR_CMD2_WR		(0x11)
#define CM3217_I2C_ADDR_RD		(0x10)
#define CM3217_HW_CMD1_DFLT		(0x22)
#define CM3217_HW_CMD1_BIT_SD		(0)
#define CM3217_HW_CMD1_BIT_IT_T		(2)
#define CM3217_HW_CMD2_BIT_FD_IT	(5)
#define CM3217_HW_DELAY			(10)
#define CM3217_POWER_UA			(90)
#define CM3217_RESOLUTION		(1)
#define CM3217_RESOLUTION_STEP		(6000000L)
#define CM3217_RESOLUTION_DIVIDER	(10000L)
#define CM3217_POLL_DELAY_MS_DFLT	(1600)
#define CM3217_POLL_DELAY_MS_MIN	(33 + CM3217_HW_DELAY)
#define CM3217_INPUT_LUX_DIVISOR	(10)
#define CM3217_INPUT_LUX_MIN		(0)
#define CM3217_INPUT_LUX_MAX		(119156)
#define CM3217_INPUT_LUX_FUZZ		(0)
#define CM3217_INPUT_LUX_FLAT		(0)
#define CM3217_MAX_REGULATORS		(1)

#ifdef CONFIG_MACH_YELLOWSTONE
#define CM3217_RAW2LUX_500		(1628)
#define CCI_FILE_PATH	""
#define CCI_LIGHT_CALI_FILE_PATH	\
		"/persist/lightsensor/light_sensor_cal.bin"
#define CCI_LIGHT_TEST_FILE_PATH	\
		"/persist/lightsensor/light_sensor_test.txt"

#define CCI_DEBOUNCE			(10)
#define CCI_CAL_TIMEOUT			(100)

struct file *file;
struct inode *inode;
loff_t fsize;
mm_segment_t old_fs;
char read_buf[40];
#endif /* CONFIG_MACH_YELLOWSTONE */

enum als_state {
	CHIP_POWER_OFF,
	CHIP_POWER_ON_ALS_OFF,
	CHIP_POWER_ON_ALS_ON,
};

enum i2c_state {
	I2C_XFER_NOT_ENABLED,
	I2c_XFER_OK_REG_NOT_SYNC,
	I2c_XFER_OK_REG_SYNC,
};

struct cm3217_inf {
	struct i2c_client *i2c;
	struct workqueue_struct *wq;
	struct delayed_work dw;
	struct regulator_bulk_data vreg[CM3217_MAX_REGULATORS];
	int raw_illuminance_val;
	int als_state;
#ifdef CONFIG_MACH_YELLOWSTONE
	unsigned int cal_data;
#endif
};

static int cm3217_i2c_rd(struct cm3217_inf *inf)
{
	struct i2c_msg msg[2];
	__u8 buf[2];

	msg[0].addr = CM3217_I2C_ADDR_RD + 1;
	msg[0].flags = I2C_M_RD;
	msg[0].len = 1;
	msg[0].buf = &buf[0];
	msg[1].addr = CM3217_I2C_ADDR_RD;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &buf[1];
	if (i2c_transfer(inf->i2c->adapter, msg, 2) != 2)
		return -EIO;

	inf->raw_illuminance_val = (__u16)((buf[1] << 8) | buf[0]);
	return 0;
}

static int cm3217_i2c_wr(struct cm3217_inf *inf, __u8 cmd1, __u8 cmd2)
{
	struct i2c_msg msg[2];
	__u8 buf[2];

	buf[0] = cmd1;
	buf[1] = cmd2;
	msg[0].addr = CM3217_I2C_ADDR_CMD1_WR;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf[0];
	msg[1].addr = CM3217_I2C_ADDR_CMD2_WR;
	msg[1].flags = 0;
	msg[1].len = 1;
	msg[1].buf = &buf[1];
	if (i2c_transfer(inf->i2c->adapter, msg, 2) != 2)
		return -EIO;

	return 0;
}

static int cm3217_cmd_wr(struct cm3217_inf *inf, __u8 it_t,
		__u8 fd_it, int sd_mode)
{
	__u8 cmd1;
	__u8 cmd2;
	int err;

	cmd1 = (CM3217_HW_CMD1_DFLT);
	if (sd_mode)
		cmd1 |= (1 << CM3217_HW_CMD1_BIT_SD);
	cmd1 |= (it_t << CM3217_HW_CMD1_BIT_IT_T);
	cmd2 = fd_it << CM3217_HW_CMD2_BIT_FD_IT;
	err = cm3217_i2c_wr(inf, cmd1, cmd2);
	return err;
}
#ifndef CONFIG_MACH_YELLOWSTONE
static int cm3217_vreg_dis(struct cm3217_inf *inf, unsigned int i)
{
	int err = 0;

	if (inf->vreg[i].ret && (inf->vreg[i].consumer != NULL)) {
		err = regulator_disable(inf->vreg[i].consumer);
		if (!err)
			dev_dbg(&inf->i2c->dev, "%s %s\n",
				__func__, inf->vreg[i].supply);
		else
			dev_err(&inf->i2c->dev, "%s %s ERR\n",
				__func__, inf->vreg[i].supply);
	}
	inf->vreg[i].ret = 0;
	return err;
}

static int cm3217_vreg_dis_all(struct cm3217_inf *inf)
{
	unsigned int i;
	int err = 0;

	for (i = CM3217_MAX_REGULATORS; i > 0; i--)
		err |= cm3217_vreg_dis(inf, (i - 1));
	return err;
}

static int cm3217_vreg_en(struct cm3217_inf *inf, unsigned int i)
{
	int err = 0;

	if (!inf->vreg[i].ret && (inf->vreg[i].consumer != NULL)) {
		err = regulator_enable(inf->vreg[i].consumer);
		if (!err) {
			inf->vreg[i].ret = 1;
			dev_dbg(&inf->i2c->dev, "%s %s\n",
				__func__, inf->vreg[i].supply);
			err = 1; /* flag regulator state change */
		} else {
			dev_err(&inf->i2c->dev, "%s %s ERR\n",
				__func__, inf->vreg[i].supply);
		}
	}
	return err;
}

static int cm3217_vreg_en_all(struct cm3217_inf *inf)
{
	unsigned i;
	int err = 0;

	for (i = 0; i < CM3217_MAX_REGULATORS; i++)
		err |= cm3217_vreg_en(inf, i);
	return err;
}

static void cm3217_vreg_exit(struct cm3217_inf *inf)
{
	int i;

	for (i = 0; i < CM3217_MAX_REGULATORS; i++) {
		regulator_put(inf->vreg[i].consumer);
		inf->vreg[i].consumer = NULL;
	}
}

static int cm3217_vreg_init(struct cm3217_inf *inf)
{
	unsigned int i;
	int err = 0;

	/*
	 * regulator names in order of powering on.
	 * ARRAY_SIZE(cm3217_vregs) must be < CM3217_MAX_REGULATORS
	 */
	char *cm3217_vregs[] = {
		"vdd",
	};

	for (i = 0; i < ARRAY_SIZE(cm3217_vregs); i++) {
		inf->vreg[i].supply = cm3217_vregs[i];
		inf->vreg[i].ret = 0;
		inf->vreg[i].consumer = regulator_get(&inf->i2c->dev,
							inf->vreg[i].supply);
		if (IS_ERR(inf->vreg[i].consumer)) {
			err = PTR_ERR(inf->vreg[i].consumer);
			dev_err(&inf->i2c->dev, "%s err %d for %s\n",
				__func__, err, inf->vreg[i].supply);
			inf->vreg[i].consumer = NULL;
		}
	}
	for (; i < CM3217_MAX_REGULATORS; i++)
		inf->vreg[i].consumer = NULL;
	return err;
}
#endif
static ssize_t cm3217_chan_regulator_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cm3217_inf *inf = iio_priv(indio_dev);
	unsigned int enable = 0;

	if (inf->als_state != CHIP_POWER_OFF)
		enable = 1;
	return sprintf(buf, "%d\n", inf->als_state);
}

static ssize_t cm3217_chan_regulator_enable(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	u8 enable;
	int ret = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cm3217_inf *inf = iio_priv(indio_dev);

	if (kstrtou8(buf, 10, &enable))
		return -EINVAL;

	if ((enable != 0) && (enable != 1))
		return -EINVAL;

	if (enable == (inf->als_state != CHIP_POWER_OFF))
		return 1;
#ifdef CONFIG_MACH_YELLOWSTONE
	inf->als_state = enable;

	return ret ? ret : 1;
#endif
#ifndef CONFIG_MACH_YELLOWSTONE
	if (!inf->vreg)
		goto success;

	if (enable)
		ret = cm3217_vreg_en_all(inf);
	else
		ret = cm3217_vreg_dis_all(inf);

	if (ret != enable) {
		dev_err(&inf->i2c->dev,
				"func:%s line:%d err:%d fails\n",
				__func__, __LINE__, ret);
		goto fail;
	}

success:
	inf->als_state = enable;
	if (!enable && regulator_is_enabled(inf->vreg[0].consumer))
		cm3217_cmd_wr(inf, 0, 0, 0);
fail:
	return ret ? ret : 1;
#endif
}

static void cm3217_work(struct work_struct *ws)
{
	struct cm3217_inf *inf;
	struct iio_dev *indio_dev;

	inf = container_of(ws, struct cm3217_inf, dw.work);
	indio_dev = iio_priv_to_dev(inf);
	mutex_lock(&indio_dev->mlock);
	cm3217_i2c_rd(inf);
	mutex_unlock(&indio_dev->mlock);
}

static ssize_t cm3217_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cm3217_inf *inf = iio_priv(indio_dev);
	unsigned int enable = 0;

	if (inf->als_state == CHIP_POWER_ON_ALS_ON)
		enable = 1;
	return sprintf(buf, "%u\n", enable);
}

static ssize_t cm3217_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cm3217_inf *inf = iio_priv(indio_dev);
	u8 enable;
	int err = 0;

	if (kstrtou8(buf, 10, &enable))
		return -EINVAL;

	if ((enable != 0) && (enable != 1))
		return -EINVAL;

	if (enable == (inf->als_state - 1))
		goto success;

	mutex_lock(&indio_dev->mlock);
	if (enable) {
		err = cm3217_cmd_wr(inf, 0, 0, 0);
		inf->raw_illuminance_val = -EINVAL;
		queue_delayed_work(inf->wq, &inf->dw, CM3217_HW_DELAY);
	} else {
		cancel_delayed_work_sync(&inf->dw);
		err = cm3217_cmd_wr(inf, 0, 0, 1);
	}
	mutex_unlock(&indio_dev->mlock);
	if (err)
		return err;

success:
	inf->als_state = enable + 1;
	return count;
}

static ssize_t cm3217_raw_illuminance_val_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cm3217_inf *inf = iio_priv(indio_dev);
#ifdef CONFIG_MACH_YELLOWSTONE
	unsigned int cal_raw = -1;
#endif

	if (inf->als_state != CHIP_POWER_ON_ALS_ON)
		return sprintf(buf, "-1\n");
	queue_delayed_work(inf->wq, &inf->dw, 0);
#ifdef CONFIG_MACH_YELLOWSTONE
	if (inf->cal_data <= 0) {
		return sprintf(buf, "%d\n", inf->raw_illuminance_val);
	} else {
		cal_raw = (inf->raw_illuminance_val *
				CM3217_RAW2LUX_500) / inf->cal_data;
		return sprintf(buf, "%u\n", cal_raw);
	}
#else
	if (inf->raw_illuminance_val == -EINVAL)
		return sprintf(buf, "-1\n");
	return sprintf(buf, "%d\n", inf->raw_illuminance_val);
#endif
}

#ifdef CONFIG_MACH_YELLOWSTONE
static int kernel_file_open(char *file_path, mode_t mode)
{
	file = filp_open(file_path, mode, 0777);
	if (IS_ERR(file))
		return -1;
	return 0;
}

static loff_t kernel_file_size(struct file *file)
{
	if (file == NULL)
		return 0;
	inode = file->f_dentry->d_inode;
	fsize = inode->i_size;
	return fsize;
}
static int kernel_addr_limit_expend(void)
{
	old_fs = get_fs();
	set_fs(get_ds());
	return 0;
}

static int kernel_addr_limit_resume(void)
{
	set_fs(old_fs);
	return 0;
}

void *kernel_file_read(struct file *file, loff_t fsize)
{
	loff_t pos = 0;
	vfs_read(file, read_buf, sizeof(unsigned int), &pos);
	return read_buf;
}

static ssize_t kernel_file_write(struct file *file, char *buf,
		loff_t fsize, loff_t *pos)
{
	ssize_t err;
	if (file == NULL)
		return -1;
	err = vfs_write(file, buf, fsize, pos);
	return err;
}

static ssize_t cm3217_cci_cali_init_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cm3217_inf *inf = iio_priv(indio_dev);
	ssize_t err;
	unsigned int *buf1;
	err = kernel_file_open(CCI_LIGHT_CALI_FILE_PATH, O_RDONLY);
	if (err == -1) {
		dev_err(&inf->i2c->dev, "[#23]%s cannot find calibration data(%s)\n",
			__func__, CCI_LIGHT_CALI_FILE_PATH);
		goto err;
	}
	kernel_file_size(file);
	kernel_addr_limit_expend();
	buf1 = kernel_file_read(file, fsize);
	inf->cal_data = *buf1;
	filp_close(file, NULL);
	kernel_addr_limit_resume();

	if (inf->cal_data <= 0) {
		dev_err(&inf->i2c->dev, "[#23]%s calibration data read error.\n",
			__func__);
		goto err;
	} else {
		dev_info(&inf->i2c->dev, "[#23] %s calibration data, value is %u.\n",
			__func__, inf->cal_data);
	}
	return sprintf(buf, "0\n");
err:
	return sprintf(buf, "1\n");
}

static ssize_t cm3217_cci_cali_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cm3217_inf *inf = iio_priv(indio_dev);
	unsigned int i, j;
	unsigned int raw_avg = 0;
	char buf1[10];
	char buf2[40] = {'\0'};
	ssize_t err;
	ssize_t length;
	loff_t *cali_pos;
	loff_t log_pos;
	struct timeval tv;
	struct rtc_time tm;
	int count = 0;
	unsigned int raw_tmp[11] = {0};
	bool success = true;

	if (inf->als_state != CHIP_POWER_ON_ALS_ON) {
		dev_err(&inf->i2c->dev, "[#23]%s ALS not enable, state is %d.\n",
			__func__, inf->als_state);
		return sprintf(buf, "0\n");
	}

	err = kernel_file_open(CCI_LIGHT_TEST_FILE_PATH, O_WRONLY|O_CREAT);
	if (err == -1) {
		dev_err(&inf->i2c->dev, "[#23]%s file open error. %s\n",
			__func__, CCI_LIGHT_TEST_FILE_PATH);
		return sprintf(buf, "0\n");
	}
	kernel_addr_limit_expend();

	for (i = 0; i < 11; i++) {
		count++;
		for (j = i; j > 0; j--)
			raw_tmp[j] = raw_tmp[j-1];

		queue_delayed_work(inf->wq, &inf->dw, 0);
		msleep(200);
		raw_tmp[0] = inf->raw_illuminance_val;

		do_gettimeofday(&tv);
		rtc_time_to_tm(tv.tv_sec, &tm);
		sprintf(buf2, "[%d-%d-%d %d:%d:%d] Cali(%d): %u\n",
			tm.tm_year + 1900, tm.tm_mon, tm.tm_mday, tm.tm_hour,
			tm.tm_min, tm.tm_sec, i, inf->raw_illuminance_val);
		log_pos = kernel_file_size(file);
		length = strlen(buf2);
		err = kernel_file_write(file, buf2, length, &log_pos);
		if (err < 0)
			dev_info(&inf->i2c->dev, "[#23]%s write calibration log fail.\n",
				__func__);

		dev_info(&inf->i2c->dev, "[#23]%s raw_tmp[0]: %u(i = %d)\n",
			__func__, raw_tmp[0], i);
		if ((i == 0) || (raw_tmp[1] == 0)
			|| (abs(raw_tmp[1] - raw_tmp[0]) <= CCI_DEBOUNCE)) {
			break;
		} else {
			dev_info(&inf->i2c->dev, "[#23]%s calibration data drop!\n",
				__func__);
			memset(raw_tmp, 0, sizeof(raw_tmp));
			i = 0;
		}

		if (count >= CCI_CAL_TIMEOUT) {
			success = false;
			dev_info(&inf->i2c->dev, "[#23]%s Calibration timeout!!!\n",
				__func__);
			break;
		}
	}
	filp_close(file, NULL);
	kernel_addr_limit_resume();
	if (!success)
		return sprintf(buf, "0\n");

	for (i = 0; i < sizeof(raw_tmp)/sizeof(unsigned int) - 1; i++) {
		dev_info(&inf->i2c->dev, "[#23]%s raw_tmp[%d]: %u\n",
			__func__, i, raw_tmp[i]);
		raw_avg += raw_tmp[i];
	}

	raw_avg /= (sizeof(raw_tmp)/sizeof(unsigned int) - 1);
	inf->cal_data = raw_avg;
	memcpy(buf1, &raw_avg, sizeof(unsigned int));
	err = kernel_file_open(CCI_LIGHT_CALI_FILE_PATH, O_WRONLY|O_CREAT);
	if (err == -1) {
		dev_err(&inf->i2c->dev, "[#23]%s file open error. %s\n",
			__func__, CCI_LIGHT_CALI_FILE_PATH);
		return sprintf(buf, "0\n");
	}
	kernel_addr_limit_expend();
	cali_pos = &(file->f_pos);
	err = kernel_file_write(file, buf1, sizeof(raw_avg), cali_pos);
	filp_close(file, NULL);
	kernel_addr_limit_resume();
	dev_info(&inf->i2c->dev, "[#23]%s raw_avg: %u, cal_data: %u\n",
		    __func__, raw_avg, inf->cal_data);
	if (err < 0)
		return sprintf(buf, "0\n");
	return sprintf(buf, "1\n");
}

static ssize_t cm3217_cci_test_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cm3217_inf *inf = iio_priv(indio_dev);
	unsigned int cal_raw = -1;
	char buf1[40] = {'\0'};
	ssize_t err;
	ssize_t length;
	loff_t log_pos;
	struct timeval tv;
	struct rtc_time tm;

	if (inf->als_state != CHIP_POWER_ON_ALS_ON) {
		dev_err(&inf->i2c->dev, "[#23]%s ALS not enable, state is %d.\n",
			__func__, inf->als_state);
		return sprintf(buf, "-1\n");
	}

	if (inf->cal_data <= 0) {
		dev_err(&inf->i2c->dev, "[#23]%s no calibration data.\n",
			__func__);
		return sprintf(buf, "%d\n", cal_raw);
	}

	queue_delayed_work(inf->wq, &inf->dw, 0);
	cal_raw = (inf->raw_illuminance_val * CM3217_RAW2LUX_500)
		/ inf->cal_data;
	dev_info(&inf->i2c->dev, "[#23]%s cal_raw: %u, gain:%u\n",
		    __func__, cal_raw, inf->cal_data);
	/* Store test result */
	err = kernel_file_open(CCI_LIGHT_TEST_FILE_PATH, O_WRONLY|O_CREAT);
	if (err == -1) {
		dev_err(&inf->i2c->dev, "[#23]%s file open error. %s\n",
			__func__, CCI_LIGHT_TEST_FILE_PATH);
		goto finished;
	}
	kernel_addr_limit_expend();
	do_gettimeofday(&tv);
	rtc_time_to_tm(tv.tv_sec, &tm);
	sprintf(buf1, "[%d-%d-%d %d:%d:%d] Test: %u\n", tm.tm_year + 1900,
			tm.tm_mon, tm.tm_mday, tm.tm_hour,
			tm.tm_min, tm.tm_sec, cal_raw);
	log_pos = kernel_file_size(file);
	length = strlen(buf1);
	err = kernel_file_write(file, buf1, length, &log_pos);
	if (err < 0)
		dev_info(&inf->i2c->dev, "[#23]%s write test result fail.\n",
			__func__);
	filp_close(file, NULL);
	kernel_addr_limit_resume();
finished:
	return sprintf(buf, "%d\n", cal_raw);
}
#endif /* CONFIG_MACH_YELLOWSTONE */

static IIO_DEVICE_ATTR(in_illuminance_regulator_enable,
			S_IRUGO | S_IWUSR | S_IWOTH,
			cm3217_chan_regulator_enable_show,
			cm3217_chan_regulator_enable, 0);
static IIO_DEVICE_ATTR(in_illuminance_enable,
			S_IRUGO | S_IWUSR | S_IWOTH,
			cm3217_enable_show, cm3217_enable_store, 0);
static IIO_DEVICE_ATTR(in_illuminance_raw, S_IRUGO,
		   cm3217_raw_illuminance_val_show, NULL, 0);
#ifdef CONFIG_MACH_YELLOWSTONE
static IIO_DEVICE_ATTR(cci_cali_init, S_IRUGO | S_IWUSR,
		   cm3217_cci_cali_init_show, NULL, 0);
static IIO_DEVICE_ATTR(cci_cali_enable, S_IRUGO | S_IWUSR,
		   cm3217_cci_cali_enable_show, NULL, 0);
static IIO_DEVICE_ATTR(cci_test_enable, S_IRUGO | S_IWUSR,
		   cm3217_cci_test_enable_show, NULL, 0);
#endif
static IIO_CONST_ATTR(vendor, "Capella");
/* FD_IT = 000b, IT_TIMES = 1/2T i.e., 00b nano secs */
static IIO_CONST_ATTR(in_illuminance_integration_time, "480000");
/* WDM = 0b, IT_TIMES = 1/2T i.e., 00b raw_illuminance_val */
static IIO_CONST_ATTR(in_illuminance_max_range, "78643.2");
/* WDM = 0b, IT_TIMES = 1/2T i.e., 00b  mLux */
static IIO_CONST_ATTR(in_illuminance_resolution, "307");
static IIO_CONST_ATTR(in_illuminance_power_consumed, "1670"); /* milli Watt */

static struct attribute *cm3217_attrs[] = {
	&iio_dev_attr_in_illuminance_enable.dev_attr.attr,
	&iio_dev_attr_in_illuminance_regulator_enable.dev_attr.attr,
	&iio_dev_attr_in_illuminance_raw.dev_attr.attr,
#ifdef CONFIG_MACH_YELLOWSTONE
	&iio_dev_attr_cci_cali_init.dev_attr.attr,
	&iio_dev_attr_cci_cali_enable.dev_attr.attr,
	&iio_dev_attr_cci_test_enable.dev_attr.attr,
#endif
	&iio_const_attr_vendor.dev_attr.attr,
	&iio_const_attr_in_illuminance_integration_time.dev_attr.attr,
	&iio_const_attr_in_illuminance_max_range.dev_attr.attr,
	&iio_const_attr_in_illuminance_resolution.dev_attr.attr,
	&iio_const_attr_in_illuminance_power_consumed.dev_attr.attr,
	NULL
};

static struct attribute_group cm3217_attr_group = {
	.name = CM3217_NAME,
	.attrs = cm3217_attrs
};

static const struct iio_info cm3217_iio_info = {
	.attrs = &cm3217_attr_group,
	.driver_module = THIS_MODULE
};

#ifdef CONFIG_PM_SLEEP
static int cm3217_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct cm3217_inf *inf = iio_priv(indio_dev);
	int ret = 0;

	if (inf->als_state == CHIP_POWER_ON_ALS_ON)
		ret = cm3217_cmd_wr(inf, 0, 0, 1);
	if (ret)
		dev_err(&client->adapter->dev,
				"%s err in cm3217 write\n", __func__);

	return ret;
}

static int cm3217_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct cm3217_inf *inf = iio_priv(indio_dev);
	int ret = 0;

	if (inf->als_state == CHIP_POWER_ON_ALS_ON)
		ret = cm3217_cmd_wr(inf, 0, 0, 0);
	if (ret)
		dev_err(&client->adapter->dev,
				"%s err in cm3217 write\n", __func__);

	return ret;
}

static SIMPLE_DEV_PM_OPS(cm3217_pm_ops, cm3217_suspend, cm3217_resume);
#define CM3217_PM_OPS (&cm3217_pm_ops)
#else
#define CM3217_PM_OPS NULL
#endif

static int cm3217_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct cm3217_inf *inf = iio_priv(indio_dev);
#ifndef CONFIG_MACH_YELLOWSTONE
	iio_device_unregister(indio_dev);
#endif
	destroy_workqueue(inf->wq);
#ifndef CONFIG_MACH_YELLOWSTONE
	cm3217_vreg_exit(inf);
#endif
	iio_device_free(indio_dev);
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	return 0;
}

static int cm3217_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct cm3217_inf *inf;
	struct iio_dev *indio_dev;
	struct lightsensor_spec *ls_spec;
	int err;

	indio_dev = iio_device_alloc(sizeof(*inf));
	if (indio_dev == NULL) {
		dev_err(&client->dev, "%s iio_device_alloc err\n", __func__);
		return -ENOMEM;
	}

	inf = iio_priv(indio_dev);

	ls_spec = of_get_ls_spec(&client->dev);
	if (!ls_spec) {
		dev_warn(&client->dev,
			"devname:%s func:%s line:%d invalid meta data, use default\n",
			id->name, __func__, __LINE__);
	} else {
		fill_ls_attrs(ls_spec, cm3217_attrs);
	}

	inf->wq = create_singlethread_workqueue(CM3217_NAME);
	if (!inf->wq) {
		dev_err(&client->dev, "%s workqueue err\n", __func__);
		err = -ENOMEM;
		goto err_wq;
	}

	inf->i2c = client;
#ifndef CONFIG_MACH_YELLOWSTONE
	err = cm3217_vreg_init(inf);
	if (err) {
		dev_info(&client->dev,
			"%s regulator init failed, assume always on", __func__);
		goto err_vreg_init;
	}
#endif
	INIT_DELAYED_WORK(&inf->dw, cm3217_work);
#ifdef CONFIG_MACH_YELLOWSTONE
	inf->als_state = CHIP_POWER_ON_ALS_OFF;
#else
	inf->als_state = 0;
#endif

	i2c_set_clientdata(client, indio_dev);
	indio_dev->info = &cm3217_iio_info;
	indio_dev->name = id->name;
	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	err = iio_device_register(indio_dev);
	if (err) {
		dev_err(&client->dev, "%s iio_device_register err\n", __func__);
#ifdef CONFIG_MACH_YELLOWSTONE
		goto err_vreg_init;
#else
		goto err_iio_register;
#endif
	}

	inf->raw_illuminance_val = -EINVAL;
	dev_info(&client->dev, "%s success\n", __func__);
	return 0;
#ifndef CONFIG_MACH_YELLOWSTONE
err_iio_register:
	cm3217_vreg_exit(inf);
#endif
err_vreg_init:
	destroy_workqueue(inf->wq);
err_wq:
	iio_device_free(indio_dev);
	dev_err(&client->dev, "%s err=%d\n", __func__, err);
	return err;
}

static void cm3217_shutdown(struct i2c_client *client)
{
#ifdef CONFIG_MACH_YELLOWSTONE
	cm3217_remove(client);
#else
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct cm3217_inf *inf = iio_priv(indio_dev);

	inf->als_state = CHIP_POWER_OFF;
	smp_wmb();
	cancel_delayed_work_sync(&inf->dw);
	cm3217_vreg_exit(inf);
#endif
}

static const struct i2c_device_id cm3217_i2c_device_id[] = {
	{"cm3217-siio", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cm3217_i2c_device_id);

#ifdef CONFIG_OF
static const struct of_device_id cm3217_of_match[] = {
	{ .compatible = "capella,cm3217-siio", },
	{ },
};
MODULE_DEVICE_TABLE(of, cm3217_of_match);
#endif

static struct i2c_driver cm3217_driver = {
	.probe		= cm3217_probe,
	.remove		= cm3217_remove,
	.id_table	= cm3217_i2c_device_id,
	.driver = {
		.name	= "cm3217",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(cm3217_of_match),
#ifdef CONFIG_MACH_YELLOWSTONE
		.pm = CM3217_PM_OPS,
#endif
	},
	.shutdown = cm3217_shutdown,
};
module_i2c_driver(cm3217_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM3217 driver");
MODULE_AUTHOR("NVIDIA Corp");
