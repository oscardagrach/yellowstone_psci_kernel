/*
 * sh-ldisc.c - sensorhub line discipline
 *
 * Copyright (c) 2014, BSQUARE CORPORATION.  All rights reserved.

 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.

 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/kref.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/crc8.h>

#include <linux/gpio.h>
#include <linux/platform/tegra/common.h>
#include <../mach-tegra/gpio-names.h>

#include <misc/sh-ldisc.h>


DECLARE_CRC8_TABLE(sh_crc_tab);

/* number of characters left in xmit buffer before select has we have
 * room */
#define WAKEUP_CHARS 256

/*
 * This defines the low- and high-watermarks for throttling and
 * unthrottling the TTY driver.  These watermarks are used for
 * controlling the space in the read buffer.
 */
#define TTY_THRESHOLD_THROTTLE      128 /* now based on remaining room */
#define TTY_THRESHOLD_UNTHROTTLE    128


/* the following enum and names array needs to match the order of cameras and
 * sensors as in sensor-hub-msg.h  */
enum sh_devices {
	SH_CAM_TS0 = 0,     /* rear camera */
	SH_CAM_TS1,         /* front camera */
	SH_CAM_TS2,         /* fisheye camera */
	SH_CAM_END,
	SH_ACCEL = SH_CAM_END,
	SH_GYRO,
	SH_MAG,
	SH_BARO,
	SH_TEMP,
	SH_CPU_TEMP,
	SH_IMU_TEMP,
	SH_SENSOR_END,
	SH_MISC = SH_SENSOR_END,
	SH_NUM_DEVICES
};

#define NUM_CAM_DEVICES         (SH_CAM_END - SH_CAM_TS0)
#define NUM_SENSOR_DEVICES      (SH_SENSOR_END - SH_ACCEL)


static char *sh_dev_names[] = {
	"cam_ts0",
	"cam_ts1",
	"cam_ts2",
	"sh_accel",
	"sh_gyro",
	"sh_mag",
	"sh_baro",
	"sh_temp",
	"sh_cpu_temp",
	"sh_imu_temp",
	"sh_misc",
};

struct sh_dev_device {
	atomic_t instance_nr;
	struct sh_ldisc_data *sdev;
	struct miscdevice mdev;

	struct mutex read_lock;
	struct mutex frameinfo_lock;

	char *read_buf;
	int read_head;
	int read_tail;
	wait_queue_head_t inq;

	struct frameinfo *frameinfo_buf;
	int frameinfo_head;
	wait_queue_head_t frameinfo_wait;
};



struct sh_ldisc_data {
	struct kref refcount;
	int removed;
	char *read_buf;
	int read_head;
	int read_tail;
	int read_cnt;
	int need_time_sync;

	union {
		struct sensor_hub_msg_t sh_msg;
		unsigned char sh_buf[sizeof(struct sensor_hub_msg_header_t) +
				     256 + 1];
	};
	int sh_msg_idx;

	unsigned char send_buf[sizeof(struct sensor_hub_msg_header_t) +
			       256 + 1];

	struct mutex atomic_read_lock;
	struct mutex output_lock;
	raw_spinlock_t read_lock;
	struct tty_struct *tty;
	struct sh_dev_device sh_dev_dev[SH_NUM_DEVICES];
	struct regulator *reg;
};


static ssize_t sh_ldisc_chars_in_buffer(struct tty_struct *tty);
static void sh_ldisc_set_termios(struct tty_struct *tty, struct ktermios *old);
static void ldata_release(struct kref *ref);

static int sensor_hub_dest_dev(unsigned char device_id);
static int sh_dev_get_frameinfo_blocking(struct sh_dev_device *shdev,
	struct frameinfo *info);
static void sh_clear_frameinfo(unsigned char camera_id);

static struct sh_ldisc_data *sh_ldisc_data;
static struct sensor_hub_platform_data *sh_platform_data;


/* GPIOs used to control sensorhub */
#define SH_PWREN        TEGRA_GPIO_PQ1
#define SH_CONFIG0      TEGRA_GPIO_PQ2
#define SH_CONFIG1      TEGRA_GPIO_PQ3
#define SH_RESET_N      TEGRA_GPIO_PQ4
#define SH_PWREN_DVT3   TEGRA_GPIO_PS6


#define SH_STATUS_0     TEGRA_GPIO_PS2
#define SH_STATUS_1     TEGRA_GPIO_PS1
#define SH_NEW_DATA_1   TEGRA_GPIO_PO4
#define SH_NEW_DATA_2   TEGRA_GPIO_PI3

#define SH_FRAMEINFO_SIZE 32
#define SH_FRAMEINFO_NEXT(x) ((x + 1) & (SH_FRAMEINFO_SIZE - 1))
#define SH_FRAMEINFO_PREV(x) (x == 0 ? SH_FRAMEINFO_SIZE - 1 : x - 1)


static void sh_flush(void)
{
	if (sh_ldisc_data->tty->ops->flush_chars)
		sh_ldisc_data->tty->ops->flush_chars(sh_ldisc_data->tty);

	if (sh_ldisc_data->tty->ops->chars_in_buffer) {
		while (sh_ldisc_data->tty->
			ops->chars_in_buffer(sh_ldisc_data->tty) > 0)
			mdelay(1);
	}

	if (sh_ldisc_data->tty->ops->wait_until_sent)
		sh_ldisc_data->tty->ops->wait_until_sent(
		    sh_ldisc_data->tty, 0);
}

/* array of camera power states for use in suspend/resume */
static int camera_power_state[4] = { 1, 1, 1, 1 };
static DEFINE_MUTEX(camera_power_lock);
static DECLARE_WAIT_QUEUE_HEAD(camera_power_wait);
static int camera_power_ack;

int sh_camera_power(int cam_id, int power)
{
	struct __attribute__ ((__packed__)) {
		struct sensor_hub_msg_header_t header;
		unsigned char magic1;
	} msg;
	int c;

	mutex_lock(&camera_power_lock);

	pr_info("%s: E %d %d\n", __func__, cam_id, power);

	if (NULL == sh_ldisc_data) {
		pr_info("%s: line discipline disabled\n", __func__);
		mutex_unlock(&camera_power_lock);
		return 1;
	}

	msg.header.magic = SENSOR_HUB_MAGIC;
	msg.header.msg_type = power ? CAMERA_ENABLE : CAMERA_DISABLE;
	switch (cam_id) {
	case DEVICE_REAR_CAMERA:
	case DEVICE_FISHEYE_CAMERA:
	case DEVICE_FRONT_CAMERA:
	case DEVICE_OTHER_CAMERA:
		break;
	default:
		mutex_unlock(&camera_power_lock);
		return 1;
		break;
	}
	msg.header.device = cam_id;
	msg.header.length = 0;
	msg.magic1 = 0xff ^ crc8(sh_crc_tab, (void *) &msg,
				 sizeof(msg) - 1,
				 CRC8_INIT_VALUE);

	camera_power_ack = 0;

	mutex_lock(&sh_ldisc_data->output_lock);

	c = sh_ldisc_data->tty->ops->write(sh_ldisc_data->tty,
			(unsigned char *) &msg, sizeof(msg));

	sh_flush();

	mutex_unlock(&sh_ldisc_data->output_lock);

	if (c != sizeof(msg)) {
		pr_info("%s: problem sending message\n", __func__);
		mutex_unlock(&camera_power_lock);
		return 1;
	}

	c = wait_event_timeout(camera_power_wait, camera_power_ack != 0,
			       msecs_to_jiffies(200));
	if (c == 0)
		pr_info("%s: camera_power_ack timeout\n", __func__);

	sh_clear_frameinfo(cam_id);

	camera_power_state[cam_id - DEVICE_REAR_CAMERA] = power;

	mutex_unlock(&camera_power_lock);

	return 0;
}

static void sh_clear_frameinfo(unsigned char camera_id)
{
	int dst_dev;
	struct sh_dev_device *shdev;

	dst_dev = sensor_hub_dest_dev(camera_id);
	shdev =	&sh_ldisc_data->sh_dev_dev[dst_dev];

	mutex_lock(&shdev->frameinfo_lock);

	memset(shdev->frameinfo_buf, 0, sizeof(shdev->frameinfo_buf));
	shdev->frameinfo_head = 0;

	mutex_unlock(&shdev->frameinfo_lock);
}

int sh_get_frameinfo(unsigned char camera_id, struct frameinfo *info)
{
	int dst_dev;

	dst_dev = sensor_hub_dest_dev(camera_id);

	return sh_dev_get_frameinfo_blocking(
		&sh_ldisc_data->sh_dev_dev[dst_dev], info);
}

static int sh_set_power_state(int power)
{
	struct __attribute__ ((__packed__)) {
		struct sensor_hub_msg_header_t header;
		unsigned char power;
		unsigned char magic1;
	} msg;
	int c;

	pr_info("sh_set_power_state: %d\n", power);

	if (NULL == sh_ldisc_data) {
		pr_info("sh_set_power_state: line discipline disabled\n");
		return 1;
	}

	msg.header.magic = SENSOR_HUB_MAGIC;
	msg.header.msg_type = SENSOR_HUB_POWER;
	msg.header.device = DEVICE_NONE;
	msg.header.length = 1;
	msg.power = power;
	msg.magic1 = 0xff ^ crc8(sh_crc_tab, (void *) &msg,
				 sizeof(msg) - 1,
				 CRC8_INIT_VALUE);

	mutex_lock(&sh_ldisc_data->output_lock);

	c = sh_ldisc_data->tty->ops->write(sh_ldisc_data->tty,
			(unsigned char *) &msg, sizeof(msg));

	sh_flush();

	mutex_unlock(&sh_ldisc_data->output_lock);

	if (c != sizeof(msg)) {
		pr_info("sh_set_power_state: problem sending message\n");
		return 1;
	}

	return 0;
}

static int sh_set_time(void)
{
	struct __attribute__ ((__packed__)) {
		struct sensor_hub_msg_header_t header;
		uint64_t time;
		unsigned char magic1;
	} msg;
	int c;
	struct timespec ts;

#define ENABLE_SH_TIME_SYNC 1
#ifdef ENABLE_SH_TIME_SYNC
	pr_info("%s E\n", __func__);

	if (NULL == sh_ldisc_data) {
		pr_info("%s: line discipline disabled\n", __func__);
		return 1;
	}

	mutex_lock(&sh_ldisc_data->output_lock);

	getrawmonotonic(&ts);

	msg.header.magic = SENSOR_HUB_MAGIC;
	msg.header.msg_type = SENSOR_HUB_SET_TIME;
	msg.header.device = DEVICE_NONE;
	msg.header.length = 8;
	msg.time = (uint64_t) ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
	msg.magic1 = 0xff ^ crc8(sh_crc_tab, (void *) &msg,
				 sizeof(msg) - 1,
				 CRC8_INIT_VALUE);

	c = sh_ldisc_data->tty->ops->write(sh_ldisc_data->tty,
			(unsigned char *) &msg, sizeof(msg));

	sh_flush();

	mutex_unlock(&sh_ldisc_data->output_lock);

	if (c != sizeof(msg)) {
		pr_info("%s: problem sending message\n", __func__);
		return 1;
	}

	pr_info("%s X\n", __func__);
#endif

	sh_ldisc_data->need_time_sync = 0;

	return 0;
}


/*********************************************************/


static ssize_t sh_dev_write(struct file *file, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct sh_dev_device *shdev = file->private_data;
	struct sh_ldisc_data *ldata = shdev->sdev;
	struct tty_struct *tty = ldata->tty;
	unsigned char *b;
	int c;
	int retval = 0;

	if (ldata->removed)
		return -EINVAL;

	if (count == 0)
		return 0;

	if (ldata->need_time_sync && count > 1 && buffer[1] == SENSOR_ENABLE)
		sh_set_time();

	if (mutex_lock_interruptible(&ldata->output_lock))
		return -EINTR;

	if (tty->ops->write_room && tty->ops->write_room(tty) < count) {
		mutex_unlock(&ldata->output_lock);
		return 0;
	}

	memcpy(ldata->send_buf, buffer, count);
	b = ldata->send_buf;

	/* compute CRC of message */
	if (b[0] == SENSOR_HUB_MAGIC &&
	    count >=  sizeof(struct sensor_hub_msg_header_t) &&
	    count == (b[3] + sizeof(struct sensor_hub_msg_header_t) + 1)) {
		b[count - 1] = 0xff ^ crc8(sh_crc_tab, b, count - 1,
					   CRC8_INIT_VALUE);
	}

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			break;
		}
		while (count > 0) {
			c = tty->ops->write(tty, b, count);
			if (c < 0) {
				retval = c;
				goto break_out;
			}
			if (!c)
				break;
			b += c;
			count -= c;
		}
		if (!count)
			break;
		if (file->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			break;
		}
		schedule();
	}
break_out:

	if (tty->ops->flush_chars)
		tty->ops->flush_chars(tty);

	__set_current_state(TASK_RUNNING);
	if (b - ldata->send_buf != count && tty->fasync)
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

	mutex_unlock(&ldata->output_lock);

	return (b - ldata->send_buf) ? b - ldata->send_buf : retval;
}

static ssize_t sh_dev_read(struct file *file, char __user *buffer,
			   size_t count, loff_t *ppos)
{
	struct sh_dev_device *shdev = file->private_data;
	ssize_t size;
	ssize_t msg_size;
	ssize_t retval = 0;
	char __user *b = buffer;

	if (shdev->sdev->removed)
		return -EINVAL;

	if (mutex_lock_interruptible(&shdev->read_lock))
		return -ERESTARTSYS;

	while (CIRC_CNT(shdev->read_head,
			shdev->read_tail, N_TTY_BUF_SIZE) == 0) {
		/* nothing to read */
		mutex_unlock(&shdev->read_lock); /* release the lock */
		if (file->f_flags & O_NONBLOCK)
				return -EAGAIN;

		if (wait_event_interruptible(shdev->inq,
					     (CIRC_CNT(shdev->read_head,
						       shdev->read_tail,
						       N_TTY_BUF_SIZE) != 0 ||
					      shdev->sdev->removed)))
			return -ERESTARTSYS; /* signal: tell fs layer to handle
					      * it otherwise loop, but first
					      * reacquire the lock */

		if (shdev->sdev->removed)
			return -EINVAL;

		if (mutex_lock_interruptible(&shdev->read_lock))
				return -ERESTARTSYS;
	}

	/* get the message size */
	msg_size = shdev->read_buf[(shdev->read_tail + 3) & (N_TTY_BUF_SIZE-1)]
		   + 4 + 1;

	if (msg_size > count)
		retval = -EINVAL;
	else {
		while (msg_size) {
			int c;

			c = shdev->read_buf[shdev->read_tail];
			shdev->read_tail = (shdev->read_tail+1) &
					   (N_TTY_BUF_SIZE-1);
			msg_size--;

			if (put_user(c, b++)) {
				b--;
				retval = -EFAULT;
				break;
			}
		}
	}

	mutex_unlock(&shdev->read_lock); /* release the lock */

	size = b - buffer;
	if (size)
		retval = size;

	return retval;
}

/* unsafe, call with shdev->frameinfo_lock held */
static int sh_dev_get_frameinfo(struct sh_dev_device *shdev,
	struct frameinfo *info)
{
	int i;
	int curr_req_frame_count;
	int oldest_frame_count;
	struct frameinfo *curr_info;

	curr_req_frame_count = info->frame_counter;
	/* frameinfo_head is the oldest element. */
	i = shdev->frameinfo_head;
	i = SH_FRAMEINFO_PREV(i);
	/* curr_info is the newest element. */
	curr_info = &shdev->frameinfo_buf[i];
	oldest_frame_count = curr_info->frame_counter;
	oldest_frame_count -= SH_FRAMEINFO_SIZE;
	if (curr_req_frame_count <= oldest_frame_count ||
	    curr_req_frame_count > curr_info->frame_counter) {
		int rval = -ENOENT;
		/* Requested frame_counter out of bounds.
		 * Return most recent info along with error. */
		if (curr_req_frame_count > curr_info->frame_counter) {
			if (curr_req_frame_count <=
				curr_info->frame_counter + 2)
				rval = -EAGAIN;
			else
				pr_err("%s:%d %d > %d\n", __func__, __LINE__,
					curr_req_frame_count,
					curr_info->frame_counter);
		} else {
			pr_err("%s:%d %d <= %d\n", __func__, __LINE__,
				curr_req_frame_count, oldest_frame_count);
		}
		info->timestamp = curr_info->timestamp;
		return rval;
	}

	i = shdev->frameinfo_head;
	do {
		i = SH_FRAMEINFO_PREV(i);
		curr_info = &shdev->frameinfo_buf[i];

		if (curr_info->frame_counter == curr_req_frame_count) {
			info->timestamp = curr_info->timestamp;
			if (curr_info->timestamp > 0)
				return 0;
			else
				return -ENOENT;
		}
	} while (i != shdev->frameinfo_head);

	return -ENOENT;
}

static int sh_dev_get_frameinfo_blocking(struct sh_dev_device *shdev,
	struct frameinfo *info)
{
	int ret;
	int retries;

	for (retries = 0; retries < SH_FRAMEINFO_SIZE; ++retries) {
		int current_index;

		ret = mutex_lock_interruptible(&shdev->frameinfo_lock);

		if (ret)
			break;

		ret = sh_dev_get_frameinfo(shdev, info);

		current_index = shdev->frameinfo_head;

		mutex_unlock(&shdev->frameinfo_lock);

		if (ret != -EAGAIN)
			break;

		ret = wait_event_interruptible(shdev->frameinfo_wait,
			((shdev->frameinfo_head != current_index) ||
				shdev->sdev->removed));

		if (ret)
			break;

		/* set ret to error in case it is end of loop */
		ret = -ENOENT;
	}

	return ret;
}

static int sh_dev_add_frameinfo(struct sh_dev_device *shdev,
	const unsigned char *buf,
	int count)
{
	struct sensor_hub_msg_t *msg = (struct sensor_hub_msg_t *) buf;

	if (count < sizeof(struct sensor_hub_camera_hal_msg_t) +
		sizeof(struct sensor_hub_msg_header_t))
		return -EFAULT;

	mutex_lock(&shdev->frameinfo_lock);

	shdev->frameinfo_buf[shdev->frameinfo_head] =
		msg->msg.camera.param.timestamp;

	shdev->frameinfo_head = SH_FRAMEINFO_NEXT(shdev->frameinfo_head);

	wake_up_interruptible(&shdev->frameinfo_wait);

	mutex_unlock(&shdev->frameinfo_lock);

	return 0;
}

static void sh_dev_add_line(struct sh_dev_device *shdev, unsigned char *buf,
			    int count)
{
	int buf_space_available;

	mutex_lock(&shdev->read_lock);

	buf_space_available = CIRC_SPACE(shdev->read_head, shdev->read_tail,
					 N_TTY_BUF_SIZE);

	if (buf_space_available > count) {
		int space = CIRC_SPACE_TO_END(shdev->read_head,
			shdev->read_tail, N_TTY_BUF_SIZE);
		if (space > count) {
				memcpy(&shdev->read_buf[shdev->read_head], buf,
				       count);
			} else {
				memcpy(&shdev->read_buf[shdev->read_head], buf,
				       space);
				memcpy(&shdev->read_buf[0], buf + space,
				       count - space);
			}
		shdev->read_head += count;
		shdev->read_head &= (N_TTY_BUF_SIZE - 1);
		wake_up_interruptible(&shdev->inq);
	} else {
		static int limit = 20;
		if (limit) {
			pr_info("no space in buffer - discarding camts line\n");
			limit--;
		}
	}

	mutex_unlock(&shdev->read_lock);
}


static unsigned int sh_dev_poll(struct file *file, poll_table *wait)
{
	struct sh_dev_device *shdev = file->private_data;
	unsigned int retval = 0;

	poll_wait(file, &shdev->inq, wait);

	mutex_lock(&shdev->read_lock);
	if (CIRC_CNT(shdev->read_head, shdev->read_tail, N_TTY_BUF_SIZE))
		retval |= POLLIN | POLLRDNORM;
	mutex_unlock(&shdev->read_lock);

	retval |= POLLOUT | POLLWRNORM;

	if (shdev->sdev->removed)
		retval |= POLLERR | POLLHUP;

	return retval;
}

static void sh_dev_flush_buffer(struct sh_dev_device *shdev)
{
	mutex_lock(&shdev->read_lock);
	shdev->read_head = shdev->read_tail = 0;
	mutex_unlock(&shdev->read_lock);
}

static int sh_dev_open(struct inode *inode, struct file *file)
{
	struct miscdevice *md = file->private_data;
	struct sh_dev_device *shdev;

	shdev = container_of(md, struct sh_dev_device, mdev);

	/* the device could be opened once */
	if (!atomic_dec_and_test(&shdev->instance_nr)) {
		atomic_inc(&shdev->instance_nr);
		pr_err("sh-ldisc: cannot open device more than once.\n");
		return -EBUSY;
	}


	kref_get(&shdev->sdev->refcount);

	sh_dev_flush_buffer(shdev);

	file->private_data = shdev;
	nonseekable_open(inode, file);

	return 0;
}

static int sh_dev_release(struct inode *inode, struct file *file)
{
	struct sh_dev_device *shdev = file->private_data;

	kref_put(&shdev->sdev->refcount, ldata_release);
	atomic_inc(&shdev->instance_nr);

	return 0;
}

static long sh_dev_compat_ioctl(struct file *file,
				unsigned int cmd,
				unsigned long arg)
{
	int ret = 0;
	struct sh_dev_device *shdev = file->private_data;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(SENSOR_HUB_IOCTL_GET_FRAMEINFO):
	{
		struct frameinfo info;

		if (copy_from_user(&info, (const void __user *) arg,
				   sizeof(info))) {
			pr_err("cannot get frameinfo argument from userland");
			return -EFAULT;
		}

		ret = sh_dev_get_frameinfo_blocking(shdev, &info);

		/* copy the frameinfo back to userland */
		if (copy_to_user((void __user *) arg, &info, sizeof(info))) {
			pr_err("cannot copy frameinfo back to userspace");
			return -EFAULT;
		}

		break;
	}
	default:
		ret = -EINVAL;
	}

	return ret;
}


static long sh_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return sh_dev_compat_ioctl(file, cmd, arg);
}

static int sh_suspend(struct device *dev)
{
	pr_info("%s\n", __func__);
	sh_set_power_state(0);

	mdelay(40);

	return 0;
}

static int sh_suspend_late(struct device *dev)
{
	int ret = 0;
	const struct sensor_hub_platform_data *pdata =
		dev->platform_data;

	pr_info("%s\n", __func__);

	pdata->power_off(pdata);

	if (sh_ldisc_data->reg) {
		ret = regulator_disable(sh_ldisc_data->reg);
		if (ret)
			pr_err("%s: disable sh regulator failed.\n", __func__);
	}

	return ret;
}

static int sh_resume_early(struct device *dev)
{
	int ret = 0;
	const struct sensor_hub_platform_data *pdata =
		dev->platform_data;

	if (sh_ldisc_data->reg) {
		ret = regulator_enable(sh_ldisc_data->reg);
		if (ret) {
			pr_err("%s: enable sh regulator failed.\n", __func__);
			goto fail;
		}
	}

	pdata->power_on(pdata);
fail:
	return ret;
}

static int sh_resume(struct device *dev)
{
	int i;

	pr_info("%s\n", __func__);

	/* delay to allow sensor hub to initialise */
	mdelay(100);

	sh_set_power_state(1);

	if (sh_ldisc_data)
		sh_ldisc_data->need_time_sync = 1;

	/* turn off cameras if they weren't on before suspend */
	for (i = DEVICE_REAR_CAMERA; i <= DEVICE_OTHER_CAMERA; i++)
		if (!camera_power_state[i - DEVICE_REAR_CAMERA])
			sh_camera_power(i, 0);

	return 0;
}

static const struct file_operations sh_dev_fops = {
	.owner		= THIS_MODULE,
	.open		= sh_dev_open,
	.release	= sh_dev_release,
	.read		= sh_dev_read,
	.write		= sh_dev_write,
	.poll		= sh_dev_poll,
	.compat_ioctl	= sh_dev_compat_ioctl,
	.unlocked_ioctl	= sh_dev_ioctl,
	.llseek		= no_llseek,
};


static int create_sh_dev_dev(struct sh_ldisc_data *ldata)
{
	int i;

	for (i = 0; i < SH_NUM_DEVICES; i++) {
		struct sh_dev_device *shdev = &ldata->sh_dev_dev[i];
		struct miscdevice *md = &shdev->mdev;

		memset(shdev, 0, sizeof(struct sh_dev_device));

		atomic_set(&shdev->instance_nr, 1);
		shdev->read_buf = kzalloc(N_TTY_BUF_SIZE, GFP_KERNEL);
		shdev->frameinfo_buf = kzalloc(
			SH_FRAMEINFO_SIZE * sizeof(struct frameinfo),
			GFP_KERNEL);
		if (!shdev->read_buf || !shdev->frameinfo_buf) {
			pr_info("create_sh_dev_dev: out of memory\n");
			goto err_exit;
		} else {
			shdev->sdev = ldata;
			mutex_init(&shdev->read_lock);
			mutex_init(&shdev->frameinfo_lock);
			init_waitqueue_head(&shdev->inq);
			init_waitqueue_head(&shdev->frameinfo_wait);

			md->fops = &sh_dev_fops;
			md->minor = MISC_DYNAMIC_MINOR;
			md->name = sh_dev_names[i];

			if (misc_register(md) != 0) {
				pr_info("create_sh_dev_dev: misc_register failed\n");
				kfree(shdev->read_buf);
				kfree(shdev->frameinfo_buf);
				goto err_exit;
			}
		}
	}

	return 1;

err_exit:
	while (--i >= 0) {
		struct sh_dev_device *shdev = &ldata->sh_dev_dev[i];
		struct miscdevice *md = &shdev->mdev;

		if (misc_deregister(md) != 0)
			pr_info("create_sh_dev_dev: misc_deregister failed\n");

		kfree(shdev->read_buf);
		kfree(shdev->frameinfo_buf);
	}

	return 0;
}

static void delete_sh_dev_dev(struct sh_ldisc_data *ldata)
{
	int i;

	for (i = 0; i < SH_NUM_DEVICES; i++) {
		struct sh_dev_device *shdev = &ldata->sh_dev_dev[i];
		struct miscdevice *md = &shdev->mdev;

		wake_up_interruptible(&shdev->inq);
		wake_up_interruptible(&shdev->frameinfo_wait);

		if (misc_deregister(md) != 0)
			pr_info("misc_register failed in delete_sh_dev_dev\n");

		kfree(shdev->read_buf);
		kfree(shdev->frameinfo_buf);
	}
}


/*********************************************************/

static inline int tty_put_user(struct tty_struct *tty, unsigned char x,
				   unsigned char __user *ptr)
{
	tty_audit_add_data(tty, &x, 1, 1);
	return put_user(x, ptr);
}


static void reset_buffer_flags(struct sh_ldisc_data *ldata)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ldata->read_lock, flags);
	ldata->read_head = ldata->read_tail = ldata->read_cnt = 0;
	ldata->sh_msg_idx = 0;
	raw_spin_unlock_irqrestore(&ldata->read_lock, flags);
}

static inline int input_available_p(struct tty_struct *tty)
{
	struct sh_ldisc_data *ldata = tty->disc_data;

	tty_flush_to_ldisc(tty);
	if (ldata->read_cnt)
		return 1;

	return 0;
}

static void sh_ldisc_set_room(struct tty_struct *tty)
{
	struct sh_ldisc_data *ldata = tty->disc_data;
	int left;
	int old_left;

	/* ldata->read_cnt is not read locked ? */
	left = N_TTY_BUF_SIZE - ldata->read_cnt - 1;

	old_left = tty->receive_room;
	tty->receive_room = left;

	/* Did this open up the receive buffer? We may need to flip */
	if (left && !old_left)
		schedule_work(&tty->port->buf.work);
}


static void ldata_release(struct kref *ref)
{
	struct sh_ldisc_data *ldata;
	ldata = container_of(ref, struct sh_ldisc_data, refcount);

	if (sh_platform_data)
		sh_platform_data->power_off(sh_platform_data);

	if (ldata->reg) {
		int ret = regulator_disable(ldata->reg);
		if (ret)
			pr_err("%s: disable sh regulator failed.\n", __func__);

		regulator_put(ldata->reg);
		ldata->reg = NULL;
	}

	kfree(ldata->read_buf);
	kfree(ldata);
}


/*
 * Called when a tty is put into sensorhub line discipline. Called in process
 * context.
 */
static int
sh_ldisc_open(struct tty_struct *tty)
{
	struct sh_ldisc_data *ldata;
	int ret = 0;

	ldata = kzalloc(sizeof(*ldata), GFP_KERNEL);
	if (!ldata) {
		ret = -ENOMEM;
		goto err;
	}

	mutex_init(&ldata->atomic_read_lock);
	mutex_init(&ldata->output_lock);
	raw_spin_lock_init(&ldata->read_lock);
	kref_init(&ldata->refcount);

	/* These are ugly. Currently a malloc failure here can panic */
	ldata->read_buf = kzalloc(N_TTY_BUF_SIZE, GFP_KERNEL);
	if (!ldata->read_buf) {
		ret = -ENOMEM;
		goto err_free_bufs;
	}

	ldata->tty = tty;

	tty->disc_data = ldata;
	reset_buffer_flags(tty->disc_data);
	/* indicate buffer work may resume */
	clear_bit(TTY_LDISC_HALTED, &tty->flags);
	sh_ldisc_set_termios(tty, NULL);

	ldata->reg = regulator_get(ldata->tty->dev, "vdd_sensorhub");
	if (IS_ERR(ldata->reg)) {
		ret = PTR_ERR(ldata->reg);
		if (ldata->reg == ERR_PTR(-ENODEV))
			pr_info("%s: no regulator device\n", __func__);
		else
			pr_err("%s: couldn't get regulator\n", __func__);

		ldata->reg = NULL;
		goto err_free_bufs;
	}

	ret = regulator_enable(ldata->reg);
	if (ret) {
		pr_err("%s: enable sh regulator failed.\n", __func__);
		goto err_free_bufs;
	}

	if (sh_platform_data)
		sh_platform_data->power_on(sh_platform_data);

	if (!create_sh_dev_dev(ldata)) {
		ret = -ENOMEM;
		goto err_free_bufs;
	}

	tty_unthrottle(tty);

	pr_info("sh_ldisc_open\n");
	sh_ldisc_data = ldata;

	ldata->need_time_sync = 1;

	return 0;

err_free_bufs:
	kref_put(&ldata->refcount, ldata_release);
err:
	return ret;
}

/*
 * Called when the tty is put into another line discipline
 * or it hangs up.
 * This routine must be called from process context, not
 * interrupt or softirq context.
 */
static void
sh_ldisc_close(struct tty_struct *tty)
{
	struct sh_ldisc_data *ldata = tty->disc_data;

	pr_info("sh_ldisc_close\n");
	ldata->removed = 1;
	sh_ldisc_data = NULL;

	delete_sh_dev_dev(ldata);

	kref_put(&ldata->refcount, ldata_release);
	tty->disc_data = NULL;
}

/*
 * Called on tty hangup in process context.
 */
static int sh_ldisc_hangup(struct tty_struct *tty)
{
	sh_ldisc_close(tty);
	return 0;
}

/*
 * Read
 */
static ssize_t
sh_ldisc_read(struct tty_struct *tty, struct file *file,
		  unsigned char __user *buf, size_t nr)
{
	struct sh_ldisc_data *ldata = tty->disc_data;
	unsigned char __user *b = buf;
	DECLARE_WAITQUEUE(wait, current);
	int c;
	ssize_t retval = 0;
	ssize_t size;
	long timeout;
	unsigned long flags;

do_it_again:
	timeout = MAX_SCHEDULE_TIMEOUT;
	/*
	 *  Internal serialization of reads.
	 */
	if (file->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&ldata->atomic_read_lock))
			return -EAGAIN;
	} else {
		if (mutex_lock_interruptible(&ldata->atomic_read_lock))
			return -ERESTARTSYS;
	}

	add_wait_queue(&tty->read_wait, &wait);
	while (nr) {
		/* This statement must be first before checking for input
		   so that any interrupt will set the state back to
		   TASK_RUNNING. */
		set_current_state(TASK_INTERRUPTIBLE);

		if (!input_available_p(tty)) {
			if (test_bit(TTY_OTHER_CLOSED, &tty->flags)) {
				retval = -EIO;
				break;
			}
			if (tty_hung_up_p(file))
				break;
			if (!timeout)
				break;
			if (file->f_flags & O_NONBLOCK) {
				retval = -EAGAIN;
				break;
			}
			if (signal_pending(current)) {
				retval = -ERESTARTSYS;
				break;
			}
			/* FIXME: does sh_ldisc_set_room need locking ? */
			sh_ldisc_set_room(tty);
			timeout = schedule_timeout(timeout);
			continue;
		}
		__set_current_state(TASK_RUNNING);

		/* N.B. avoid overrun if nr == 0 */
		raw_spin_lock_irqsave(&ldata->read_lock, flags);
		while (nr && ldata->read_cnt) {
			c = ldata->read_buf[ldata->read_tail];
			ldata->read_tail = ((ldata->read_tail+1) &
						(N_TTY_BUF_SIZE-1));
			ldata->read_cnt--;
			raw_spin_unlock_irqrestore(&ldata->read_lock, flags);

			if (tty_put_user(tty, c, b++)) {
				retval = -EFAULT;
				b--;
				raw_spin_lock_irqsave(&ldata->read_lock, flags);
				break;
			}
			nr--;

			raw_spin_lock_irqsave(&ldata->read_lock, flags);
		}
		raw_spin_unlock_irqrestore(&ldata->read_lock, flags);
		if (retval)
			break;

		/* If there is enough space in the read buffer now, let the
		 * low-level driver know. We use sh_ldisc_chars_in_buffer() to
		 * check the buffer, as it now knows about canonical mode.
		 * Otherwise, if the driver is throttled and the line is
		 * longer than TTY_THRESHOLD_UNTHROTTLE in canonical mode,
		 * we won't get any more characters.
		 */
		while (1) {
			tty_set_flow_change(tty, TTY_UNTHROTTLE_SAFE);
			if (sh_ldisc_chars_in_buffer(tty) >
			    TTY_THRESHOLD_UNTHROTTLE)
				break;
			if (!tty->count)
				break;
			sh_ldisc_set_room(tty);
			if (!tty_unthrottle_safe(tty))
				break;
		}
		__tty_set_flow_change(tty, 0);
		if (b - buf >= 1)
			break;
	}
	mutex_unlock(&ldata->atomic_read_lock);
	remove_wait_queue(&tty->read_wait, &wait);

	__set_current_state(TASK_RUNNING);
	size = b - buf;
	if (size) {
		retval = size;
		if (nr)
			clear_bit(TTY_PUSH, &tty->flags);
	} else if (test_and_clear_bit(TTY_PUSH, &tty->flags))
		goto do_it_again;

	sh_ldisc_set_room(tty);
	return retval;
}

/*
 * Write
 */
static ssize_t sh_ldisc_write(struct tty_struct *tty, struct file *file,
			      const unsigned char *buf, size_t nr)
{
	struct sh_ldisc_data *ldata = tty->disc_data;
	const unsigned char *b = buf;
	DECLARE_WAITQUEUE(wait, current);
	int c;
	ssize_t retval = 0;

	if (mutex_lock_interruptible(&ldata->output_lock))
		return -EINTR;

	add_wait_queue(&tty->write_wait, &wait);
	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			break;
		}
		if (tty_hung_up_p(file) || (tty->link && !tty->link->count)) {
			retval = -EIO;
			break;
		}
		while (nr > 0) {
			c = tty->ops->write(tty, b, nr);
			if (c < 0) {
				retval = c;
				goto break_out;
			}
			if (!c)
				break;
			b += c;
			nr -= c;
		}
		if (!nr)
			break;
		if (file->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			break;
		}
		schedule();
	}
break_out:

	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&tty->write_wait, &wait);
	if (b - buf != nr && tty->fasync)
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	mutex_unlock(&ldata->output_lock);
	return (b - buf) ? b - buf : retval;
}


/*
 * Called in process context only. May be re-entered by multiple
 * ioctl calling threads.
 */

static int sh_ldisc_ioctl(struct tty_struct *tty, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	int err, val;
	int __user *p = (int __user *)arg;

	err = -EFAULT;

	switch (cmd) {
	case TCFLSH:
		/* flush our buffers and the serial port's buffer */
		if (arg == TCIOFLUSH || arg == TCOFLUSH)
			;
		err = n_tty_ioctl_helper(tty, file, cmd, arg);
		break;

	case FIONREAD:
		val = 0;
		if (put_user(val, p))
			break;
		err = 0;
		break;

	default:
		/* Try the various mode ioctls */
		err = tty_mode_ioctl(tty, file, cmd, arg);
	}

	return err;
}

static unsigned int sh_ldisc_poll(struct tty_struct *tty, struct file *file,
				  poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &tty->read_wait, wait);
	poll_wait(file, &tty->write_wait, wait);
	if (input_available_p(tty))
		mask |= POLLIN | POLLRDNORM;
	if (test_bit(TTY_OTHER_CLOSED, &tty->flags))
		mask |= POLLHUP;
	if (tty_hung_up_p(file))
		mask |= POLLHUP;
	if (!(mask & (POLLHUP | POLLIN | POLLRDNORM)))
		tty->minimum_to_wake = 1;
	if (tty->ops->write && !tty_is_writelocked(tty) &&
	    tty_chars_in_buffer(tty) < WAKEUP_CHARS &&
	    tty_write_room(tty) > 0)
		mask |= POLLOUT | POLLWRNORM;
	return mask;
}


static void sh_ldisc_set_termios(struct tty_struct *tty, struct ktermios *old)
{
	sh_ldisc_set_room(tty);
	/*
	 * Fix tty hang when I_IXON(tty) is cleared, but the tty
	 * been stopped by STOP_CHAR(tty) before it.
	 */
	if (!I_IXON(tty) && old && (old->c_iflag & IXON) && !tty->flow_stopped)
		start_tty(tty);

	/* The termios change make the tty ready for I/O */
	wake_up_interruptible(&tty->write_wait);
	wake_up_interruptible(&tty->read_wait);
}


/*
 *  Flush the input buffer. Called when the tty layer wants the
 *  buffer flushed (eg at hangup)
 */

static void sh_ldisc_flush_buffer(struct tty_struct *tty)
{
	reset_buffer_flags(tty->disc_data);
	sh_ldisc_set_room(tty);
}

/*
 *  Report the number of characters buffered to be delivered to user
 *  at this instant in time.
 */
static ssize_t sh_ldisc_chars_in_buffer(struct tty_struct *tty)
{
	struct sh_ldisc_data *ldata = tty->disc_data;
	unsigned long flags;
	ssize_t n = 0;

	raw_spin_lock_irqsave(&ldata->read_lock, flags);
	n = ldata->read_cnt;
	raw_spin_unlock_irqrestore(&ldata->read_lock, flags);
	return n;
}


static void put_tty_queue_nolock(unsigned char c, struct sh_ldisc_data *ldata)
{
	if (ldata->read_cnt < N_TTY_BUF_SIZE) {
		ldata->read_buf[ldata->read_head] = c;
		ldata->read_head = (ldata->read_head + 1) & (N_TTY_BUF_SIZE-1);
		ldata->read_cnt++;
	}
}

/**
 *  put_tty_queue       -   add character to tty
 *  @c: character
 *  @ldata: n_tty data
 *
 *  Add a character to the tty read_buf queue. This is done under the
 *  read_lock to serialize character addition and also to protect us
 *  against parallel reads or flushes
 */

static void put_tty_queue(unsigned char c, struct sh_ldisc_data *ldata)
{
	unsigned long flags;
	/*
	 *  The problem of stomping on the buffers ends here.
	 *  Why didn't anyone see this one coming? --AJK
	*/
	raw_spin_lock_irqsave(&ldata->read_lock, flags);
	put_tty_queue_nolock(c, ldata);
	raw_spin_unlock_irqrestore(&ldata->read_lock, flags);
}




static inline void sh_ldisc_receive_char(struct tty_struct *tty,
					 unsigned char c)
{
	struct sh_ldisc_data *ldata = tty->disc_data;

	if (tty->closing)
		return;

	if (ldata->read_cnt >= (N_TTY_BUF_SIZE - 1))
		return;

	put_tty_queue(c, ldata);
}

static int sensor_hub_msg_type_valid(unsigned char byte)
{
	if ((byte >= SENSOR_HUB_START && byte <= SENSOR_HUB_END) ||
	    (byte >= TORCH_HAL_START  && byte <= TORCH_HAL_END) ||
	    (byte >= CAMERA_HAL_START && byte <= CAMERA_HAL_END) ||
	    (byte >= SENSOR_HAL_START && byte <= SENSOR_HAL_END))
		return 1;
	return 0;
}

static bool sensor_hub_msg_type_is_timestamp(const unsigned char byte)
{
	if (sensor_hub_msg_type_valid(byte))
		return (byte == CAMERA_FRAMEINFO);
	else
		return false;
}

static int sensor_hub_device_valid(unsigned char byte)
{
	if (byte >= DEVICE_NONE && byte <= DEVICE_END)
		return 1;
	return 0;
}

static int sensor_hub_dest_dev(unsigned char device_id)
{
	if (device_id >= DEVICE_ACCEL && device_id <= DEVICE_END)
		return SH_ACCEL + (device_id - DEVICE_ACCEL);

	if (device_id >= DEVICE_REAR_CAMERA && device_id <= DEVICE_OTHER_CAMERA)
		return SH_CAM_TS0 + (device_id - DEVICE_REAR_CAMERA);

	return SH_MISC;
}

static void process_message(struct sh_ldisc_data *ldata)
{
	int dst_dev;

	switch (ldata->sh_msg.header.msg_type) {
	case CAMERA_ENABLE:
	case CAMERA_DISABLE:
		/* camera enable/disable acknowledgment */
		camera_power_ack = 1;
		wake_up(&camera_power_wait);
		break;


	case SENSOR_HUB_SET_TIME:
		/* request for current time */
		sh_set_time();
		break;

	default:
		dst_dev = sensor_hub_dest_dev(ldata->sh_msg.header.device);

		sh_dev_add_line(&ldata->sh_dev_dev[dst_dev],
				ldata->sh_buf,
				ldata->sh_msg_idx);

		if (sensor_hub_msg_type_is_timestamp(
			ldata->sh_msg.header.msg_type)) {
			sh_dev_add_frameinfo
				(&ldata->sh_dev_dev[dst_dev],
					ldata->sh_buf,
					ldata->sh_msg_idx);
		}
	}
}

static inline void sh_ldisc_receive_msg(struct tty_struct *tty, unsigned char c)
{
	struct sh_ldisc_data *ldata = tty->disc_data;

	/* sanity check index */
	if (ldata->sh_msg_idx >= sizeof(ldata->sh_buf))
		ldata->sh_msg_idx = 0;

	ldata->sh_buf[ldata->sh_msg_idx] = c;

	switch (ldata->sh_msg_idx++) {
	case 0:
		/* expecting Magic value */
		if (c != SENSOR_HUB_MAGIC)
			ldata->sh_msg_idx = 0;
		break;

	case 1:
		/* Expecting message type */
		if (!sensor_hub_msg_type_valid(c))
			ldata->sh_msg_idx = 0;
		break;

	case 2:
		/* expecting device */
		if (!sensor_hub_device_valid(c))
			ldata->sh_msg_idx = 0;
		break;

	case 3:
		/* expecting length */
		break;

	default:
		/* accumulate rest of message */
		if (ldata->sh_msg_idx == ldata->sh_msg.header.length +
		    sizeof(struct sensor_hub_msg_header_t) + 1) {
			u8 crc;
			crc = crc8(sh_crc_tab, ldata->sh_buf, ldata->sh_msg_idx,
				   CRC8_INIT_VALUE);
			if (crc == CRC8_GOOD_VALUE(sh_crc_tab))
				/* got a complete message */
				process_message(ldata);
			else {
				pr_err("%s: crc error, discarding packet\n",
				       __func__);
				print_hex_dump(KERN_ERR, "rx buf: ",
					       DUMP_PREFIX_NONE, 16, 1,
					       ldata->sh_buf, ldata->sh_msg_idx,
					       false);
			}

			ldata->sh_msg_idx = 0;
		}
		break;
	}
}


/* May sleep, don't call from interrupt level or with interrupts disabled */
static void sh_ldisc_receive(struct tty_struct *tty, const unsigned char *cp,
			     char *fp, int count)
{
	const unsigned char *p;
	char *f, flags = TTY_NORMAL;
	int i;
	char    buf[64];

	for (i = count, p = cp, f = fp; i; i--, p++) {
		if (f)
			flags = *f++;
		switch (flags) {
		case TTY_NORMAL:
			sh_ldisc_receive_msg(tty, *p);
			break;
		case TTY_BREAK:
		case TTY_PARITY:
		case TTY_FRAME:
		case TTY_OVERRUN:
			break;
		default:
			pr_err("%s: unknown flag %d\n",
					tty_name(tty, buf), flags);
			break;
		}
	}
	if (tty->ops->flush_chars)
		tty->ops->flush_chars(tty);

	sh_ldisc_set_room(tty);

	/*
	 * Check the remaining room for the input canonicalization
	 * mode.  We don't want to throttle the driver if we're in
	 * canonical mode and don't have a newline yet!
	 */
	while (1) {
		tty_set_flow_change(tty, TTY_THROTTLE_SAFE);
		if (tty->receive_room >= TTY_THRESHOLD_THROTTLE)
			break;
		if (!tty_throttle_safe(tty))
			break;
	}
	__tty_set_flow_change(tty, 0);
}

static void sh_ldisc_wakeup(struct tty_struct *tty)
{
	if (tty->fasync && test_and_clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags))
		kill_fasync(&tty->fasync, SIGIO, POLL_OUT);
}


static struct tty_ldisc_ops sh_ldisc = {
	.owner  = THIS_MODULE,
	.magic  = TTY_LDISC_MAGIC,
	.name   = "shub",
	.open   = sh_ldisc_open,
	.close  = sh_ldisc_close,
	.hangup = sh_ldisc_hangup,
	.read   = sh_ldisc_read,
	.write  = sh_ldisc_write,
	.ioctl  = sh_ldisc_ioctl,
	.poll   = sh_ldisc_poll,
	.set_termios     = sh_ldisc_set_termios,


	.flush_buffer    = sh_ldisc_flush_buffer,
	.chars_in_buffer = sh_ldisc_chars_in_buffer,

	.receive_buf = sh_ldisc_receive,
	.write_wakeup = sh_ldisc_wakeup,
};


static int sh_spi_probe(struct spi_device *spi)
{
	int ret = 0;
	struct sensor_hub_platform_data *pdata = spi->dev.platform_data;

	dev_info(&spi->dev, "++ %s ++\n", __func__);

	/* validate platform data */
	if (pdata == NULL ||
	    pdata->power_on == NULL || pdata->power_off == NULL)
		return -EINVAL;

	sh_platform_data = pdata;

	ret = tty_register_ldisc(N_SHUB, &sh_ldisc);
	if (ret != 0)
		dev_err(&spi->dev, "error %d registering line disc\n", ret);

	crc8_populate_msb(sh_crc_tab, 0x4d);

	dev_info(&spi->dev, "-- %s --\n", __func__);

	return ret;
}

static int sh_spi_remove(struct spi_device *spi)
{
	int ret = 0;
	struct sensor_hub_platform_data *pdata = spi->dev.platform_data;

	dev_info(&spi->dev, "++ %s ++\n", __func__);

	ret = tty_unregister_ldisc(N_SHUB);
	if (ret != 0)
		dev_err(&spi->dev, "failed to unregister sensor hub line discipline\n");

	pdata->power_off(pdata);

	sh_platform_data = NULL;

	dev_info(&spi->dev, "-- %s --\n", __func__);

	return ret;
}

static void sh_spi_shutdown(struct spi_device *spi)
{
	struct sensor_hub_platform_data *pdata = spi->dev.platform_data;
	pdata->power_off(pdata);
	return;
}

static const struct dev_pm_ops sh_pm_ops = {
	.suspend = sh_suspend,
	.suspend_late = sh_suspend_late,
	.resume_early = sh_resume_early,
	.resume = sh_resume,
};

static const struct spi_device_id sh_spi_id[] = {
	{ SH_SPI_NAME, 0 },
	{ }
};

static struct spi_driver sh_spi_driver = {
	.probe = sh_spi_probe,
	.remove = sh_spi_remove,
	.shutdown = sh_spi_shutdown,
	.id_table = sh_spi_id,
	.driver = {
		.name = SH_SPI_NAME,
		.owner = THIS_MODULE,
#if defined(CONFIG_PM)
		.pm = &sh_pm_ops,
#endif
	},
};

static int __init sh_spi_init(void)
{
	return spi_register_driver(&sh_spi_driver);
}

static void __exit sh_spi_exit(void)
{
	spi_unregister_driver(&sh_spi_driver);
}

module_init(sh_spi_init);
module_exit(sh_spi_exit);

MODULE_DESCRIPTION("sensorhub control driver");
MODULE_AUTHOR("Simon Munton <simonm@bsquare.com>");
MODULE_LICENSE("GPL");

