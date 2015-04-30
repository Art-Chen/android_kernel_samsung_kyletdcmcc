/*
 * Copyright (c) 2010-2011 Yamaha Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/yas.h>

#define __LINUX_KERNEL_DRIVER__

//DUAL
#if 0 //ndef CONFIG_YAS_ACC_MULTI_SUPPORT
#ifdef CONFIG_YAS_ACC_DRIVER_LIS3DH
#include "yas_acc_driver-lis3dh.c"
#elif CONFIG_YAS_ACC_DRIVER_BMA250
#include "yas_acc_driver-bma25x.c"
#elif CONFIG_YAS_ACC_DRIVER_BMA222E
#include "yas_acc_driver-bma222e.c"
#endif
//#else
#include <linux/yas_accel.h>
#endif
#include "yas_acc_driver-bma222e.c"
#include "yas_acc_driver-kxtj2.c"
#include <linux/yas_accel.h>

#include <linux/sensors_core.h>

#define YAS_ACC_KERNEL_VERSION                                       "4.4.702a"
#define YAS_ACC_KERNEL_NAME                                     "accelerometer"

#ifdef CONFIG_YAS_ACC_MULTI_SUPPORT

char vendor_name[MAX_CHIP_NUM][MAX_LEN_OF_NAME] = {
#ifdef CONFIG_YAS_ACC_DRIVER_LIS3DH
	{"STM"},
#endif
#ifdef (CONFIG_YAS_ACC_DRIVER_BMA250) || (CONFIG_YAS_ACC_DRIVER_BMA222E)
	{"BOSCH"},
#endif
#ifdef CONFIG_YAS_ACC_DRIVER_KXTJ2
	{"KIONIX"},
#endif
};

char chip_name[MAX_CHIP_NUM][MAX_LEN_OF_NAME] = {
#ifdef CONFIG_YAS_ACC_DRIVER_LIS3DH
	{"K3DH"},
#endif
#ifdef CONFIG_YAS_ACC_DRIVER_BMA250
	{"BMA25X"},
#endif
#ifdef CONFIG_YAS_ACC_DRIVER_BMA222E
	{"BMA222E"},
#endif
#ifdef CONFIG_YAS_ACC_DRIVER_KXTJ2
	{"KXTJ2"},
#endif
};
#endif

/* ABS axes parameter range [um/s^2] (for input event) */
#define GRAVITY_EARTH                                                   9806550
#define ABSMAX_2G                                           (GRAVITY_EARTH * 2)
#define ABSMIN_2G                                          (-GRAVITY_EARTH * 2)

#define delay_to_jiffies(d)                     ((d) ? msecs_to_jiffies(d) : 1)
#define actual_delay(d)                 (jiffies_to_msecs(delay_to_jiffies(d)))

/* -------------------------------------------------------------------------- *
 *  Function prototype declaration
 * -------------------------------------------------------------------------- */
static struct yas_acc_private_data *yas_acc_get_data(void);
static void yas_acc_set_data(struct yas_acc_private_data *);

static int yas_acc_lock(void);
static int yas_acc_unlock(void);
static int yas_acc_i2c_open(void);
static int yas_acc_i2c_close(void);
static int yas_acc_i2c_write(uint8_t, const uint8_t *, int);
static int yas_acc_i2c_read(uint8_t, uint8_t *, int);
static void yas_acc_msleep(int);

static int yas_acc_core_driver_init(struct yas_acc_private_data *);
static void yas_acc_core_driver_fini(struct yas_acc_private_data *);
static int yas_acc_get_enable(struct yas_acc_driver *);
static int yas_acc_set_enable(struct yas_acc_driver *, int);
static int yas_acc_get_delay(struct yas_acc_driver *);
static int yas_acc_set_delay(struct yas_acc_driver *, int);
static int yas_acc_get_position(struct yas_acc_driver *);
static int yas_acc_set_position(struct yas_acc_driver *, int);
static int yas_acc_get_threshold(struct yas_acc_driver *);
static int yas_acc_set_threshold(struct yas_acc_driver *, int);
static int yas_acc_get_filter_enable(struct yas_acc_driver *);
static int yas_acc_set_filter_enable(struct yas_acc_driver *, int);
static int yas_acc_measure(struct yas_acc_driver *, struct yas_acc_data *);
static int yas_acc_input_init(struct yas_acc_private_data *);
static void yas_acc_input_fini(struct yas_acc_private_data *);

static ssize_t yas_acc_enable_show(struct device *, struct device_attribute *
				   , char *);
static ssize_t yas_acc_enable_store(struct device *, struct device_attribute *
				    , const char *, size_t);
static ssize_t yas_acc_delay_show(struct device *, struct device_attribute *
				  , char *);
static ssize_t yas_acc_delay_store(struct device *, struct device_attribute *
				   , const char *, size_t);
static ssize_t yas_acc_position_show(struct device *, struct device_attribute *
				     , char *);
static ssize_t yas_acc_position_store(struct device *
				      , struct device_attribute *
				      , const char *, size_t);
static ssize_t yas_acc_threshold_show(struct device *
				      , struct device_attribute *, char *);
static ssize_t yas_acc_threshold_store(struct device *
				       , struct device_attribute *
				       , const char *, size_t);
static ssize_t yas_acc_filter_enable_show(struct device *
					  , struct device_attribute *, char *);
static ssize_t yas_acc_filter_enable_store(struct device *
					   , struct device_attribute *
					   , const char *, size_t);
static ssize_t yas_acc_wake_store(struct device * , struct device_attribute *
				  , const char *, size_t);
static ssize_t yas_acc_private_data_show(struct device *
					 , struct device_attribute *, char *);
#if DEBUG
static ssize_t yas_acc_debug_reg_show(struct device *
				      , struct device_attribute *, char *);
static int yas_acc_suspend(struct i2c_client *, pm_message_t);
static int yas_acc_resume(struct i2c_client *);
#endif

static void yas_acc_work_func(struct work_struct *);
static int yas_acc_probe(struct i2c_client *, const struct i2c_device_id *);
static int yas_acc_remove(struct i2c_client *);
static int yas_acc_suspend(struct i2c_client *, pm_message_t);
static int yas_acc_resume(struct i2c_client *);
static int yas_acc_set_enable_factory_test(struct yas_acc_driver *, int);
/* -------------------------------------------------------------------------- *
 *  Driver private data
 * -------------------------------------------------------------------------- */
struct yas_acc_private_data {
	struct mutex driver_mutex;
	struct mutex data_mutex;
	struct mutex enable_mutex;
	struct i2c_client *client;
	struct input_dev *input;
	struct yas_acc_driver *driver;
	struct delayed_work work;
	struct yas_acc_data last;
	int suspend_enable;
#if DEBUG
	struct mutex suspend_mutex;
	int suspend;
#endif
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct list_head devfile_list;
#endif
	struct device *accel_sensor_device;
	struct yas_vector cal_data;
	int enabled;
// DUAL
//#ifdef CONFIG_YAS_ACC_MULTI_SUPPORT
	int used_chip;
	int position;
	char vendor_name[MAX_LEN_OF_NAME];
	char chip_name[MAX_LEN_OF_NAME];
//#endif
};

/* BMA222E calibration */
#define CALIBRATION_FILE_PATH	"/efs/calibration_data"

static int iscalibrated=0;
static u8 calRestored=0; // Cal file should be read at least one time after boot up
static int foundAccel=0;


static struct yas_acc_private_data *yas_acc_private_data;
static struct yas_acc_private_data *yas_acc_get_data(void)
{
	return yas_acc_private_data;
}
static void yas_acc_set_data(struct yas_acc_private_data *data)
{
	yas_acc_private_data = data;
}

#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE

#include <linux/miscdevice.h>
#define SENSOR_NAME "accelerometer"
#define MAX_COUNT (64)

struct sensor_device {
	struct list_head list;
	struct mutex lock;
	wait_queue_head_t waitq;
	struct input_event events[MAX_COUNT];
	int head, num_event;
};

static void
get_time_stamp(struct timeval *tv)
{
	struct timespec ts;

	ktime_get_ts(&ts);
	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = ts.tv_nsec / 1000;
}

static void
make_event(struct input_event *ev, int type, int code, int value)
{
	struct timeval tv;

	get_time_stamp(&tv);
	ev->type = type;
	ev->code = code;
	ev->value = value;
	ev->time = tv;
}

static void
make_event_w_time(struct input_event *ev, int type, int code, int value
		  , struct timeval *tv)
{
	ev->type = type;
	ev->code = code;
	ev->value = value;
	ev->time = *tv;
}

static void
sensor_enq(struct sensor_device *kdev, struct input_event *ev)
{
	int idx;

	idx = kdev->head + kdev->num_event;
	if (MAX_COUNT <= idx)
		idx -= MAX_COUNT;

	kdev->events[idx] = *ev;
	kdev->num_event++;
	if (MAX_COUNT < kdev->num_event) {
		kdev->num_event = MAX_COUNT;
		kdev->head++;
		if (MAX_COUNT <= kdev->head)
			kdev->head -= MAX_COUNT;
	}
}

static int
sensor_deq(struct sensor_device *kdev, struct input_event *ev)
{
	if (kdev->num_event == 0)
		return 0;

	*ev = kdev->events[kdev->head];
	kdev->num_event--;
	kdev->head++;
	if (MAX_COUNT <= kdev->head)
		kdev->head -= MAX_COUNT;

	return 1;
}

static void
sensor_event(struct list_head *devlist, struct input_event *ev, int num)
{
	struct sensor_device *kdev;
	int i;

	list_for_each_entry(kdev, devlist, list) {
		mutex_lock(&kdev->lock);
		for (i = 0; i < num; i++)
			sensor_enq(kdev, &ev[i]);
		mutex_unlock(&kdev->lock);
		wake_up_interruptible(&kdev->waitq);
	}
}

static ssize_t
sensor_write(struct file *f, const char __user *buf, size_t count, loff_t *pos)
{
	struct yas_acc_private_data *data = yas_acc_get_data();
	struct sensor_device *kdev;
	struct input_event ev[MAX_COUNT];
	int num, i;

	if (count < sizeof(struct input_event))
		return -EINVAL;

	num = count / sizeof(struct input_event);
	if (MAX_COUNT < num)
		num = MAX_COUNT;

	if (copy_from_user(ev, buf, num * sizeof(struct input_event)))
		return -EFAULT;

	list_for_each_entry(kdev, &data->devfile_list, list) {
		mutex_lock(&kdev->lock);
		for (i = 0; i < num; i++)
			sensor_enq(kdev, &ev[i]);
		mutex_unlock(&kdev->lock);
		wake_up_interruptible(&kdev->waitq);
	}

	return count;
}

static ssize_t
sensor_read(struct file *f, char __user *buf, size_t count, loff_t *pos)
{
	struct sensor_device *kdev = f->private_data;
	int rt, num;
	struct input_event ev[MAX_COUNT];

	if (count < sizeof(struct input_event))
		return -EINVAL;

	rt = wait_event_interruptible(kdev->waitq, kdev->num_event != 0);
	if (rt)
		return rt;

	mutex_lock(&kdev->lock);
	for (num = 0; num < count / sizeof(struct input_event); num++) {
		if (!sensor_deq(kdev, &ev[num]))
			break;
	}
	mutex_unlock(&kdev->lock);

	if (copy_to_user(buf, ev, num * sizeof(struct input_event)))
		return -EFAULT;

	return num * sizeof(struct input_event);
}

static unsigned int
sensor_poll(struct file *f, struct poll_table_struct *wait)
{
	struct sensor_device *kdev = f->private_data;

	poll_wait(f, &kdev->waitq, wait);
	if (kdev->num_event != 0)
		return POLLIN | POLLRDNORM;

	return 0;
}

static int sensor_open(struct inode *inode, struct file *f)
{
	struct yas_acc_private_data *data = yas_acc_get_data();
	struct sensor_device *kdev;

	kdev = kzalloc(sizeof(struct sensor_device), GFP_KERNEL);
	if (!kdev)
		return -ENOMEM;

	mutex_init(&kdev->lock);
	init_waitqueue_head(&kdev->waitq);
	f->private_data = kdev;
	kdev->head = 0;
	kdev->num_event = 0;
	list_add(&kdev->list, &data->devfile_list);

	return 0;
}

static int sensor_release(struct inode *inode, struct file *f)
{
	struct sensor_device *kdev = f->private_data;

	list_del(&kdev->list);
	kfree(kdev);

	return 0;
}

static const struct file_operations sensor_fops = {
	.owner = THIS_MODULE,
	.open = sensor_open,
	.release = sensor_release,
	.write = sensor_write,
	.read = sensor_read,
	.poll = sensor_poll,
};

static struct miscdevice sensor_devfile = {
	.name = SENSOR_NAME,
	.fops = &sensor_fops,
	.minor = MISC_DYNAMIC_MINOR,
};

#endif


int read_accel_raw_xyz(struct yas_vector *acc)
{
	struct yas_acc_data accel;
	struct yas_acc_private_data *data = yas_acc_get_data();

	mutex_lock(&data->data_mutex);
	yas_acc_measure(data->driver, &accel);
	mutex_unlock(&data->data_mutex);

	pr_debug("acc_data_read x: %d y: %d z: %d\n",
		accel.xyz.v[0], accel.xyz.v[1], accel.xyz.v[2]);

	acc->v[0] = accel.xyz.v[0];

	acc->v[1] = accel.xyz.v[1];

	acc->v[2] = accel.xyz.v[2];

	return 0;
}


static int accel_open_calibration(void)
{
	struct file *cal_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;
	struct yas_acc_private_data *data = yas_acc_get_data();

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open("/efs/calibration_data", O_RDONLY, 0666);
	if (IS_ERR(cal_filp)) {
		pr_err("%s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}

	err = cal_filp->f_op->read(cal_filp,
		(char *)&data->cal_data, 3 * sizeof(s32), &cal_filp->f_pos);
	if (err != 3 * sizeof(s32)) {
		pr_err("%s: Can't read the cal data from file\n", __func__);
		err = -EIO;
	}

	pr_info("%s: (%d,%d,%d)\n", __func__,
		data->cal_data.v[0], data->cal_data.v[1], data->cal_data.v[2]);

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return err;
}

/* BMA222E calibration */
static int bma222_open_calibration(struct yas_acc_private_data *data)
{
	struct file *cal_filp = NULL;
	struct acc_cal_data cal_data;
	struct yas_vector offset;
	char buf[3] = {0, };
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp)) 
	{
		pr_err("%s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		iscalibrated=0;
		calRestored = 1;
		return err;
        	}

	err = cal_filp->f_op->read(cal_filp, (char *)&cal_data, 3 * sizeof(s16), &cal_filp->f_pos);
	if (err != 3 * sizeof(s16)) 
	{
		pr_err("%s: is not yet made or Cannot open.\n", __func__);
		err = -EIO;
	}

	if( (cal_data.x==0)&&(cal_data.x==0)&&(cal_data.x==0) )
	{
		iscalibrated=0;
	}
	else
	{
		iscalibrated=1;
	}
	calRestored = 1;

	printk(KERN_INFO "%s: cal data (%d,%d,%d)\n", __func__, cal_data.x, cal_data.y, cal_data.z);
	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	offset.v[0] = cal_data.x;
	offset.v[1] = cal_data.y;
	offset.v[2] = cal_data.z;

	data->driver->set_offset(&offset);

	return YAS_NO_ERROR;
}

static int accel_do_calibrate(int enable)
{
	struct file *cal_filp;
	int sum[3] = { 0, };
	int err;
	int i;
	mm_segment_t old_fs;
	struct yas_acc_private_data *data = yas_acc_get_data();

	mutex_lock(&data->data_mutex);
	yas_acc_set_enable_factory_test(data->driver, 1);
	mutex_unlock(&data->data_mutex);

	for (i = 0; i < 100; i++) {
		err = read_accel_raw_xyz(&data->cal_data);
		if (err < 0) {
			pr_err("%s: accel_read_accel_raw_xyz() "
				"failed in the %dth loop\n", __func__, i);
			return err;
		}

		sum[0] += data->cal_data.v[0];
		sum[1] += data->cal_data.v[1];
		sum[2] += data->cal_data.v[2];
	}

	mutex_lock(&data->data_mutex);
	yas_acc_set_enable_factory_test(data->driver, 0);
	mutex_unlock(&data->data_mutex);

	if (enable) {
		data->cal_data.v[0] =
			(sum[0] / 100);
		data->cal_data.v[1] =
			(sum[1] / 100);
		data->cal_data.v[2] =
			((sum[2] / 100) - 1024);
	} else {
		data->cal_data.v[0] = 0;
		data->cal_data.v[1] = 0;
		data->cal_data.v[2] = 0;
	}

	pr_info("%s: cal data (%d,%d,%d)\n", __func__,
			data->cal_data.v[0], data->cal_data.v[1],
				data->cal_data.v[2]);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open("/efs/calibration_data",
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp)) {
		pr_err("%s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}

	err = cal_filp->f_op->write(cal_filp,
		(char *)&data->cal_data, 3 * sizeof(s32), &cal_filp->f_pos);
	if (err != 3 * sizeof(s32)) {
		pr_err("%s: Can't write the cal data to file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return err;
}


/* -------------------------------------------------------------------------- *
 *  Accelerlomete core driver callback function
 * -------------------------------------------------------------------------- */
static int yas_acc_lock(void)
{
	struct yas_acc_private_data *data = yas_acc_get_data();

	mutex_lock(&data->driver_mutex);

	return 0;
}

static int yas_acc_unlock(void)
{
	struct yas_acc_private_data *data = yas_acc_get_data();

	mutex_unlock(&data->driver_mutex);

	return 0;
}

static int yas_acc_i2c_open(void)
{
	return 0;
}

static int yas_acc_i2c_close(void)
{
	return 0;
}

static int yas_acc_i2c_write(uint8_t adr, const uint8_t *buf, int len)
{
	struct yas_acc_private_data *data = yas_acc_get_data();
	struct i2c_msg msg[2];
	char buffer[16];
	uint8_t reg;
	int err;
	int i;

	if (len > 15)
		return -1;

	reg = adr;
	buffer[0] = reg;
	for (i = 0; i < len; i++)
		buffer[i+1] = buf[i];

	msg[0].addr = data->client->addr;
	msg[0].flags = 0;
	msg[0].len = len + 1;
	msg[0].buf = buffer;
	err = i2c_transfer(data->client->adapter, msg, 1);
	if (err != 1) {
		dev_err(&data->client->dev,
		"i2c_transfer() write error: s_addr=%02x, r_addr=%02x, err=%d\n"
			, data->client->addr, adr, err);
		return err;
	}

	return 0;
}

static int yas_acc_i2c_read(uint8_t adr, uint8_t *buf, int len)
{
	struct yas_acc_private_data *data = yas_acc_get_data();
	struct i2c_msg msg[2];
	uint8_t reg;
	int err;

	reg = adr;
	msg[0].addr = data->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = data->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;

	err = i2c_transfer(data->client->adapter, msg, 2);
	if (err != 2) {
		dev_err(&data->client->dev,
		"i2c_transfer() read error: s_addr=%02x, r_addr=%02x, err=%d\n"
			, data->client->addr, adr, err);
		return err;
	}

	return 0;
}

static void yas_acc_msleep(int msec)
{
	msleep(msec);
}

/* -------------------------------------------------------------------------- *
 *  Accelerometer core driver access function
 * -------------------------------------------------------------------------- */
static int yas_acc_core_driver_init(struct yas_acc_private_data *data)
{
	struct yas_acc_driver_callback *cbk;
	struct yas_acc_driver *driver;
	int err;

	data->driver = driver =
		kzalloc(sizeof(struct yas_acc_driver), GFP_KERNEL);

	if (!driver) {
		err = -ENOMEM;
		return err;
	}

	cbk = &driver->callback;
	cbk->lock = yas_acc_lock;
	cbk->unlock = yas_acc_unlock;
	cbk->device_open = yas_acc_i2c_open;
	cbk->device_close = yas_acc_i2c_close;
	cbk->device_write = yas_acc_i2c_write;
	cbk->device_read = yas_acc_i2c_read;
	cbk->msleep = yas_acc_msleep;

// DUAL
#if 0
#ifdef CONFIG_YAS_ACC_MULTI_SUPPORT
	#ifdef CONFIG_YAS_ACC_DRIVER_LIS3DH
		if (data->used_chip == K3DH_ENABLED)
			err = yas_acc_driver_lis3dh_init(driver);
	#endif
	#ifdef CONFIG_YAS_ACC_DRIVER_BMA250
		if (data->used_chip == BMA25X_ENABLED)
			err = yas_acc_driver_BMA25X_init(driver);
	#endif
	#ifdef CONFIG_YAS_ACC_DRIVER_BMA222E
		if (data->used_chip == BMA222E_ENABLED)
			err = yas_acc_driver_BMA222E_init(driver);
	#endif
	#ifdef CONFIG_YAS_ACC_DRIVER_KXTJ2
		if (data->used_chip == KXTJ2_ENABLED)
			err = yas_acc_driver_kxtj2_init(driver);
	#endif
#else
	err = yas_acc_driver_init(driver);
#endif
#else // else of DUAL
	// Dual init
	if(data->used_chip == BMA222E_ENABLED)
		err = yas_acc_driver_BMA222E_init(driver);
	else if (data->used_chip == KXTJ2_ENABLED)
		err = yas_acc_driver_kxtj2_init(driver);
	else {
		printk("##%s : No defined function \n", __func__);
		return -1;
	}
#endif
	if (err != YAS_NO_ERROR) {
		kfree(driver);
		return err;
	}

	err = driver->init();
	if (err != YAS_NO_ERROR) {
		kfree(driver);
		return err;
	}
	else
		foundAccel  = 1;

#ifdef CONFIG_YAS_ACC_MULTI_SUPPORT
	err = driver->set_position(data->position);
	if (err != YAS_NO_ERROR) {
		kfree(driver);
		return err;
	}
#else
	err = driver->set_position(CONFIG_INPUT_YAS_ACCELEROMETER_POSITION);
	if (err != YAS_NO_ERROR) {
		kfree(driver);
		return err;
	}
#endif
	return 0;
}

static void yas_acc_core_driver_fini(struct yas_acc_private_data *data)
{
	struct yas_acc_driver *driver = data->driver;

	driver->term();
	kfree(driver);
}

static int yas_acc_get_enable(struct yas_acc_driver *driver)
{
	int enable;

	enable = driver->get_enable();

	return enable;
}

static int yas_acc_set_enable(struct yas_acc_driver *driver, int enable)
{
	struct yas_acc_private_data *data = yas_acc_get_data();
	int delay = driver->get_delay();

	if (yas_acc_get_enable(data->driver) != enable) {
		if (enable) 
		{
			driver->set_enable(enable);
			schedule_delayed_work(&data->work
					      , delay_to_jiffies(delay) + 1);
			/* BMA222E calibration */
	printk(KERN_INFO "%s: calRestored(%d), iscalibrated(%d)\n", __func__, calRestored, iscalibrated);			
			if(!calRestored)
				bma222_open_calibration(data);	// necessary cause file-system usually not yet ready at probe 
		} 
		else {
			cancel_delayed_work_sync(&data->work);
			driver->set_enable(enable);
		}
	}

	return 0;
}

static int yas_acc_set_enable_factory_test(struct yas_acc_driver *driver,
	int enable)
{
	struct yas_acc_private_data *data = yas_acc_get_data();

	if (data->enabled != enable) {
		if (enable)
			driver->set_enable(enable);
	} else {
		if (!enable)
			driver->set_enable(enable);
	}
	return 0;
}

static int yas_acc_get_delay(struct yas_acc_driver *driver)
{
	return driver->get_delay();
}

static int yas_acc_set_delay(struct yas_acc_driver *driver, int delay)
{
	struct yas_acc_private_data *data = yas_acc_get_data();

	mutex_lock(&data->enable_mutex);

	if (yas_acc_get_enable(data->driver)) {
		cancel_delayed_work_sync(&data->work);
		driver->set_delay(actual_delay(delay));
		schedule_delayed_work(&data->work, delay_to_jiffies(delay) + 1);
	} else {
		driver->set_delay(actual_delay(delay));
	}

	mutex_unlock(&data->enable_mutex);

	return 0;
}

static int yas_acc_get_offset(struct yas_acc_driver *driver,
			      struct yas_vector *offset)
{
	return driver->get_offset(offset);
}

static int yas_acc_set_offset(struct yas_acc_driver *driver,
			      struct yas_vector *offset)
{
	return driver->set_offset(offset);
}

static int yas_acc_get_position(struct yas_acc_driver *driver)
{
	return driver->get_position();
}

static int yas_acc_set_position(struct yas_acc_driver *driver, int position)
{
	return driver->set_position(position);
}

static int yas_acc_get_threshold(struct yas_acc_driver *driver)
{
	struct yas_acc_filter filter;

	driver->get_filter(&filter);

	return filter.threshold;
}

static int yas_acc_set_threshold(struct yas_acc_driver *driver, int threshold)
{
	struct yas_acc_filter filter;

	filter.threshold = threshold;

	return driver->set_filter(&filter);
}

static int yas_acc_get_filter_enable(struct yas_acc_driver *driver)
{
	return driver->get_filter_enable();
}

static int yas_acc_set_filter_enable(struct yas_acc_driver *driver, int enable)
{
	return driver->set_filter_enable(enable);
}

static int yas_acc_measure(struct yas_acc_driver *driver,
			   struct yas_acc_data *accel)
{
	int err;

	err = driver->measure(accel);

	return err;
}

/* -------------------------------------------------------------------------- *
 *  Input device interface
 * -------------------------------------------------------------------------- */
static int yas_acc_input_init(struct yas_acc_private_data *data)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = "accelerometer";
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_capability(dev, EV_ABS, ABS_RUDDER);
	input_set_abs_params(dev, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	data->input = dev;

	return 0;
}

static void yas_acc_input_fini(struct yas_acc_private_data *data)
{
	struct input_dev *dev = data->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static ssize_t yas_acc_enable_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	int enable = 0;

	mutex_lock(&data->enable_mutex);

	enable = yas_acc_get_enable(data->driver);

	mutex_unlock(&data->enable_mutex);

	return sprintf(buf, "%d\n", enable);
}

static ssize_t yas_acc_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	unsigned long enable;
	int ret;

	ret = strict_strtoul(buf, 10, &enable);
	if (ret < 0)
		return count;
	
	printk(KERN_INFO "%s: enable:%d  delay:%d\n", __func__, enable,acc_data.delay);

	mutex_lock(&data->enable_mutex);

	yas_acc_set_enable(data->driver, enable);

	mutex_unlock(&data->enable_mutex);

	data->enabled = enable;

	return count;
}

static ssize_t yas_acc_delay_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", yas_acc_get_delay(data->driver));
}

static ssize_t yas_acc_delay_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf,
				   size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	unsigned long delay;
	int ret;

	ret = strict_strtoul(buf, 10, &delay);
	if (ret < 0)
		return count;

	yas_acc_set_delay(data->driver, delay);

	return count;
}

static ssize_t yas_acc_offset_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	struct yas_vector offset;

	yas_acc_get_offset(data->driver, &offset);

	return sprintf(buf, "%d %d %d\n"
		       , offset.v[0], offset.v[1], offset.v[2]);
}

static ssize_t yas_acc_offset_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	struct yas_vector offset;

	sscanf(buf, "%d %d %d", &offset.v[0], &offset.v[1], &offset.v[2]);

	yas_acc_set_offset(data->driver, &offset);

	return count;
}

static ssize_t yas_acc_position_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", yas_acc_get_position(data->driver));
}

static ssize_t yas_acc_position_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	unsigned long position;
	int ret;

	ret = strict_strtoul(buf, 10, &position);
	if (ret < 0)
		return count;

	yas_acc_set_position(data->driver, position);

	return count;
}

static ssize_t yas_acc_threshold_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", yas_acc_get_threshold(data->driver));
}

static ssize_t yas_acc_threshold_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	unsigned long threshold;
	int ret;

	ret = strict_strtoul(buf, 10, &threshold);
	if (ret < 0)
		return count;

	yas_acc_set_threshold(data->driver, threshold);

	return count;
}

static ssize_t yas_acc_filter_enable_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", yas_acc_get_filter_enable(data->driver));
}

static ssize_t yas_acc_filter_enable_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	unsigned long enable;
	int ret;

	ret = strict_strtoul(buf, 10, &enable);
	if (ret < 0)
		return count;

	yas_acc_set_filter_enable(data->driver, enable);

	return count;
}

static ssize_t yas_acc_wake_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	static atomic_t serial = ATOMIC_INIT(0);
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct yas_acc_private_data *data = input_get_drvdata(input);
	struct input_event ev[1];
	make_event(ev, EV_ABS, ABS_MISC, atomic_inc_return(&serial));
	sensor_event(&data->devfile_list, ev, 1);
#else
	input_report_abs(input, ABS_MISC, atomic_inc_return(&serial));
	input_sync(input);
#endif

	return count;
}

static ssize_t yas_acc_private_data_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	struct yas_acc_data accel;

	mutex_lock(&data->data_mutex);
	accel = data->last;
	mutex_unlock(&data->data_mutex);

	return sprintf(buf, "%d %d %d\n"
		       , accel.xyz.v[0], accel.xyz.v[1], accel.xyz.v[2]);
}

#if DEBUG
#if YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA150
#define ADR_MAX (0x16)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA222 || \
	YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA222E || \
	YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA250 ||	\
	YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA254
#define ADR_MAX (0x40)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXSD9
#define ADR_MAX (0x0f)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTE9
#define ADR_MAX (0x5c)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTF9
#define ADR_MAX (0x60)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DL  || \
	YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLH ||	\
	YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLM
#define ADR_MAX (0x40)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS3DH
#define ADR_MAX (0x3e)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL345
#define ADR_MAX (0x3a)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL346
#define ADR_MAX (0x3d)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8452Q || \
	YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8453Q
#define ADR_MAX (0x32)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTJ2 || \
#define ADR_MAX (0x6B)
#else
#define ADR_MAX (0x16)
#endif
static uint8_t reg[ADR_MAX];

static ssize_t yas_acc_debug_reg_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	struct i2c_client *client = data->client;
	ssize_t count = 0;
	int ret;
	int i;

	memset(reg, -1, ADR_MAX);
	for (i = 0; i < ADR_MAX; i++) {
		ret = data->driver->get_register(i, &reg[i]);
		if (ret != 0)
			dev_err(&client->dev, "get_register() erorr %d (%d)\n"
				, ret, i);
		else
#if 0
			count += sprintf(&buf[count], "%02x: %3d [0x%02x]\n"
					 , i, reg[i], reg[i]);
#else
			count += sprintf(&buf[count], "%02x: %d\n", i, reg[i]);
#endif
	}

	return count;
}

static ssize_t yas_acc_debug_suspend_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	int suspend = 0;

	mutex_lock(&data->suspend_mutex);

	suspend = sprintf(buf, "%d\n", data->suspend);

	mutex_unlock(&data->suspend_mutex);

	return suspend;
}

static ssize_t yas_acc_debug_suspend_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct yas_acc_private_data *data = input_get_drvdata(input);
	struct i2c_client *client = data->client;
	unsigned long suspend;
	pm_message_t msg;
	int ret;

	ret = strict_strtoul(buf, 10, &suspend);
	if (ret < 0)
		return count;

	memset(&msg, 0, sizeof(pm_message_t));

	mutex_lock(&data->suspend_mutex);

	if (suspend) {
		yas_acc_suspend(client, msg);
		data->suspend = 1;
	} else {
		yas_acc_resume(client);
		data->suspend = 0;
	}

	mutex_unlock(&data->suspend_mutex);

	return count;
}
#endif /* DEBUG */

static DEVICE_ATTR(enable,
		   S_IRUGO|S_IWUSR|S_IWGRP,
		   yas_acc_enable_show,
		   yas_acc_enable_store
		   );
static DEVICE_ATTR(delay,
		   S_IRUGO|S_IWUSR|S_IWGRP,
		   yas_acc_delay_show,
		   yas_acc_delay_store
		   );
static DEVICE_ATTR(offset,
		   S_IRUGO|S_IWUSR,
		   yas_acc_offset_show,
		   yas_acc_offset_store
		   );
static DEVICE_ATTR(position,
		   S_IRUGO|S_IWUSR,
		   yas_acc_position_show,
		   yas_acc_position_store
		   );
static DEVICE_ATTR(threshold,
		   S_IRUGO|S_IWUSR,
		   yas_acc_threshold_show,
		   yas_acc_threshold_store
		   );
static DEVICE_ATTR(filter_enable,
		   S_IRUGO|S_IWUSR,
		   yas_acc_filter_enable_show,
		   yas_acc_filter_enable_store
		   );
static DEVICE_ATTR(wake,
		   S_IWUSR|S_IWGRP,
		   NULL,
		   yas_acc_wake_store);
static DEVICE_ATTR(data,
		   S_IRUGO,
		   yas_acc_private_data_show,
		   NULL);
#if DEBUG
static DEVICE_ATTR(debug_reg,
		   S_IRUGO,
		   yas_acc_debug_reg_show,
		   NULL
		   );
static DEVICE_ATTR(debug_suspend,
		   S_IRUGO|S_IWUSR,
		   yas_acc_debug_suspend_show,
		   yas_acc_debug_suspend_store
		   );
#endif /* DEBUG */

static struct attribute *yas_acc_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_offset.attr,
	&dev_attr_position.attr,
	&dev_attr_threshold.attr,
	&dev_attr_filter_enable.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
#if DEBUG
	&dev_attr_debug_reg.attr,
	&dev_attr_debug_suspend.attr,
#endif /* DEBUG */
	NULL
};

static struct attribute_group yas_acc_attribute_group = {
	.attrs = yas_acc_attributes
};

static void yas_acc_work_func(struct work_struct *work)
{
	struct yas_acc_private_data *data \
		= container_of((struct delayed_work *)work,
			       struct yas_acc_private_data,
			       work);
	struct yas_acc_data accel, last;
	unsigned long delay = delay_to_jiffies(yas_acc_get_delay(data->driver));
	static int cnt;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct input_event ev[4];
	struct timeval tv;
#endif

	accel.xyz.v[0] = accel.xyz.v[1] = accel.xyz.v[2] = 0;
	yas_acc_measure(data->driver, &accel);

	mutex_lock(&data->data_mutex);
	last = data->last;
	mutex_unlock(&data->data_mutex);

	//accel.xyz.v[0] = accel.xyz.v[0] - data->cal_data.v[0];
	//accel.xyz.v[1] = accel.xyz.v[1] - data->cal_data.v[1];
	//accel.xyz.v[2] = accel.xyz.v[2] - data->cal_data.v[2];

#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	get_time_stamp(&tv);
	make_event_w_time(&ev[0], EV_ABS, ABS_X, accel.xyz.v[0], &tv);
	make_event_w_time(&ev[1], EV_ABS, ABS_Y, accel.xyz.v[1], &tv);
	make_event_w_time(&ev[2], EV_ABS, ABS_Z, accel.xyz.v[2], &tv);
	make_event_w_time(&ev[3], EV_SYN, 0, 0, &tv);
	sensor_event(&data->devfile_list, ev, 4);
#else
	input_report_abs(data->input, ABS_X, accel.xyz.v[0]);
	input_report_abs(data->input, ABS_Y, accel.xyz.v[1]);
	input_report_abs(data->input, ABS_Z, accel.xyz.v[2]);
	if (last.xyz.v[0] == accel.xyz.v[0] &&
	    last.xyz.v[1] == accel.xyz.v[1] &&
	    last.xyz.v[2] == accel.xyz.v[2])
		input_report_abs(data->input, ABS_RUDDER, cnt++);
	input_sync(data->input);
#endif

	mutex_lock(&data->data_mutex);
	data->last = accel;
	mutex_unlock(&data->data_mutex);

	schedule_delayed_work(&data->work, delay);
}


static ssize_t acc_data_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_acc_data accel;
	struct yas_acc_private_data *data = yas_acc_get_data();
	s32 x;
	s32 y;
	s32 z;

	mutex_lock(&data->data_mutex);
	yas_acc_set_enable_factory_test(data->driver, 1);
	usleep_range(1000, 2000);
	yas_acc_measure(data->driver, &accel);
	yas_acc_set_enable_factory_test(data->driver, 0);
	mutex_unlock(&data->data_mutex);

	x = accel.xyz.v[0];
	y = accel.xyz.v[1];
	z = accel.xyz.v[2];

	return sprintf(buf, "%d, %d, %d\n", x, y, z);
}

/* BMA222E calibration */
static int bma222_fast_calibration(int is_cal_erase)
{
#if 0 // FOR BMA222E
	struct file *cal_filp = NULL;
	struct acc_cal_data cal_data;
	struct yas_vector offset;
	struct yas_acc_private_data *data =yas_acc_get_data();
	int err = 0;
	int i = 0;
	unsigned char buf[6] = {0, };
	int raw[3] = {0, };
	mm_segment_t old_fs;
        char tmp = 1;	// select x axis in cal_trigger by default
	int power_off_after_calibration = 0;
	int layout[3] = {0, 0, 1};
	
	printk(KERN_WARNING "YAS BMA] %s \n", __func__);
	
	//if (yas_bma2x2_sort == YAS_BMA222E)
	{
		layout[2] = YAS_BMA222E_RESOLUTION*yas_bma222e_position_map[CONFIG_INPUT_YAS_ACCELEROMETER_POSITION][2][2];		

		if(is_cal_erase) 
		{
			cal_data.x = 0;
			cal_data.y = 0;
			cal_data.z = 0;
			iscalibrated=0;
		}
		else 
		{
			 /* Read acceleration data */
		 	if (yas_bma222e_read_reg(YAS_BMA222E_ACC_REG, buf, 6) != 0) 
		 	{
        			cal_data.x = 0;
				cal_data.y = 0;
				cal_data.z = 0;
    		 	} 
		 	else 
		 	{
        			for (i = 0; i < 3; i++) 
					raw[i] = *(int8_t *)&buf[i*2+1];
					
		        	cal_data.x = raw[0] - layout[0];
		        	cal_data.y = raw[1] - layout[1];
		        	cal_data.z = raw[2] - layout[2];
	         	}
			iscalibrated=1;
		}

		old_fs = get_fs();
		set_fs(KERNEL_DS);
		cal_filp = filp_open(CALIBRATION_FILE_PATH, O_CREAT | O_TRUNC | O_WRONLY, 0666);
		if (IS_ERR(cal_filp)) 
		{
			pr_err("%s: Can't open calibration file\n", __func__);
			set_fs(old_fs);
			err = PTR_ERR(cal_filp);
			return err;
		}

		err = cal_filp->f_op->write(cal_filp, (char *)&cal_data, 3 * sizeof(s16), &cal_filp->f_pos);
		if (err != 3 * sizeof(s16)) 
		{
			pr_err("%s: Can't write the cal data to file\n", __func__);
			err = -EIO;
		}

		filp_close(cal_filp, current->files);
		set_fs(old_fs);
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		offset.v[0] = cal_data.x;
		offset.v[1] = cal_data.y;
		offset.v[2] = cal_data.z;

		printk(KERN_INFO "%s: cal data (%d,%d,%d)\n", __func__, cal_data.x, cal_data.y, cal_data.z);

		data->driver->set_offset(&offset);	// set offset		

		return err;
	}
	#if 0
	else
	{
	    if(!yas_bma222_get_enable())
	    {
	        yas_bma222_power_up();
	        power_off_after_calibration = 1;
	    }
		
	    yas_bma222_update_bits(YAS_BMA222_COMP_TARGET_OFFSET_X, layout[0]);

	    yas_bma222_update_bits(YAS_BMA222_EN_FAST_COMP, tmp);
	    do
	    {
	        mdelay(2);
	        tmp = yas_bma222_read_bits(YAS_BMA222_FAST_COMP_RDY_S);
	    } while(tmp == 0);

	    yas_bma222_update_bits(YAS_BMA222_COMP_TARGET_OFFSET_Y, layout[1]);

	    tmp = 2;	//selet y axis in cal_trigger
	    yas_bma222_update_bits(YAS_BMA222_EN_FAST_COMP, tmp);
	    do
	    {
	        mdelay(2); 
	        tmp = yas_bma222_read_bits( YAS_BMA222_FAST_COMP_RDY_S);
	    } while(tmp == 0);

	    yas_bma222_update_bits(YAS_BMA222_COMP_TARGET_OFFSET_Z, layout[2]);
	    tmp = 3;	//selet z axis in cal_trigger
	    yas_bma222_update_bits(YAS_BMA222_EN_FAST_COMP, tmp);
	    do
	    {
	        mdelay(2); 
	        tmp = yas_bma222_read_bits(YAS_BMA222_FAST_COMP_RDY_S);
	    } while(tmp == 0);

	    tmp = 1;	//unlock eeprom
	    yas_bma222_update_bits(YAS_BMA222_UNLOCK_EE_WRITE_SETTING, tmp);
	    yas_bma222_update_bits(YAS_BMA222_START_EE_WRITE_SETTING, 0x01);

	    do
	    {
	        mdelay(2); 
	        tmp = yas_bma222_read_bits(YAS_BMA222_EE_WRITE_SETTING_S);
	    } while(tmp==0);

	    tmp = 0; 	//lock eemprom	
	    yas_bma222_update_bits(YAS_BMA222_UNLOCK_EE_WRITE_SETTING, tmp);

	    if(power_off_after_calibration)
	    {
	        yas_bma222_power_down();
	    }
		iscalibrated=1;
	    return 0;
	}
	#endif
//#else  /*KXTJ2 has same position with BMA222E and modified resoultion to 8bit for KyleTD*/

	struct yas_acc_private_data *data = yas_acc_get_data();
	int err = 0;	
	#if 0
	struct yas_acc_data accel;
	struct yas_acc_private_data *data = yas_acc_get_data();
	struct acc_cal_data cal_data;
	struct yas_vector offset;	
	mm_segment_t old_fs;
	int layout[3] = {0, 0, 1};
	
	printk(KERN_WARNING "KXTJ2 %s ", __func__);

	layout[2] = YAS_BMA222E_RESOLUTION*yas_kxtj2_position_map[CONFIG_INPUT_YAS_ACCELEROMETER_POSITION][2][2];		

	if(is_cal_erase) 
	{
		cal_data.x = 0;
		cal_data.y = 0;
		cal_data.z = 0;
		iscalibrated=0;
		printk(KERN_WARNING "KXTJ2 ACC CAL ERASED !!! \n");
	}
	else 
	{
		 /* Read acceleration data */
		accel.raw.v[0] = accel.raw.v[1] = accel.raw.v[2] = 0;
		yas_acc_measure(data->driver, &accel);
	
		
	    	cal_data.x = accel.raw.v[0] - layout[0];
	    	cal_data.y = accel.raw.v[1] - layout[1];
	    	cal_data.z = accel.raw.v[2] - layout[2];
		iscalibrated=1;
		printk(KERN_WARNING "KXTJ2 ACC CAL SAVED !!! (x,y,z)=(%d,%d,%d) \n",cal_data.x,cal_data.y,cal_data.z);

	}
		
	offset.v[0] = cal_data.x;
	offset.v[1] = cal_data.y;
	offset.v[2] = cal_data.z;
	data->driver->set_offset(&offset);	// set offset		
	#endif

 err=data->driver->acc_calibration(1);
 
#endif
}

static ssize_t accel_calibration_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
    printk("### %s  iscalibrated %d\n",__func__, iscalibrated);

    return sprintf(buf, "%d\n", iscalibrated);

#if 0
	int count = 0;
	s32 x;
	s32 y;
	s32 z;
	struct yas_acc_private_data *data = yas_acc_get_data();

	x = data->cal_data.v[0];
	y = data->cal_data.v[1];
	z = data->cal_data.v[2];
	pr_info("accel_calibration_show %d %d %d\n", x, y, z);

	count = sprintf(buf, "%d %d %d\n", x, y, z);
	return count;
#endif	
}

static ssize_t accel_calibration_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
    int is_cal_erase = buf[0];
	struct yas_acc_private_data *data = yas_acc_get_data();
	struct yas_vector offset;
	struct acc_cal_data cal_data;	
	mm_segment_t old_fs;	
	struct file *cal_filp = NULL;
	int err=0;

	printk("### accel_calibration_store start cal_erase : %d\n", is_cal_erase);
	
	/*Update offset*/
#if 0 /*OLD*/
    if (is_cal_erase == 48) /* 0 */
		bma222_fast_calibration(1);	
    else /* 1 */
		bma222_fast_calibration(0);	
#endif
	/*NEW*/
	if (is_cal_erase == 48) /* 0 */
	{
		offset.v[0] = 0;
		offset.v[1] = 0;
		offset.v[2] = 0;
		err=yas_acc_set_offset(data->driver, &offset);
		//err=data->driver->acc_calibration(0); //No need to call function, Just save 0,0,0
		printk("### accel_calibration ERASE : offset is saved as 0 \n");
		iscalibrated=0;
	}
	else /* 1 */
	{
		err=data->driver->acc_calibration(1);
		printk("### accel_calibration EXECUTED \n");
		iscalibrated=1;
	}
	
	//if(err) /*0 means NoError*/

	printk("### accel_calibration  result:%d, iscal:%d \n",err,iscalibrated);

	/*Save offset at FILE*/
	yas_acc_get_offset(data->driver, &offset);
	printk("### GET offset =>x,y,z=%d,%d,%d \n",offset.v[0],offset.v[1],offset.v[2]);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp)) 
	{
		pr_err("%s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}

	cal_data.x=(s16)offset.v[0];
	cal_data.y=(s16)offset.v[1];
	cal_data.z=(s16)offset.v[2];	

	err = cal_filp->f_op->write(cal_filp, (char *)&cal_data, 3 * sizeof(s16), &cal_filp->f_pos);
	if (err != 3 * sizeof(s16)) 
	{
		pr_err("%s: Can't write the cal data to file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	printk(KERN_INFO "### %s finished with cal data (%d,%d,%d)\n", __func__, cal_data.x, cal_data.y, cal_data.z);		
	//sprintf(buf, "%d %d %d\n"  , offset.v[0], offset.v[1], offset.v[2]);
		
	return size;
}

static ssize_t get_vendor_name(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_YAS_ACC_MULTI_SUPPORT
	struct yas_acc_private_data *data = yas_acc_get_data();
	return sprintf(buf, "%s\n", vendor_name[data->used_chip]);
#else
// DUAL
//	return sprintf(buf, "%s\n", VENDOR_NAME);
	struct yas_acc_private_data *data = yas_acc_get_data();
	return sprintf(buf, "%s\n", data->vendor_name[data->used_chip]);
#endif
}

static ssize_t get_chip_name(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_YAS_ACC_MULTI_SUPPORT
	struct yas_acc_private_data *data = yas_acc_get_data();
	return sprintf(buf, "%s\n", chip_name[data->used_chip]);
#else
// DUAL
//	return sprintf(buf, "%s\n", CHIP_NAME);
	struct yas_acc_private_data *data = yas_acc_get_data();
	return sprintf(buf, "%s\n", data->chip_name[data->used_chip]);
#endif
}


static DEVICE_ATTR(calibration, 0664, accel_calibration_show,
	accel_calibration_store);
static DEVICE_ATTR(raw_data, S_IRUGO, acc_data_read,
	NULL);
static DEVICE_ATTR(vendor, S_IRUGO, get_vendor_name, NULL);
static DEVICE_ATTR(name, S_IRUGO, get_chip_name, NULL);

static struct device_attribute *accel_sensor_attrs[] = {
	&dev_attr_raw_data,
	&dev_attr_calibration,
	&dev_attr_vendor,
	&dev_attr_name,
	NULL,
};

static int yas_acc_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct yas_acc_private_data *data;
	int err;

	calRestored = 0;	
	if(foundAccel != 0 ) 
	{
		printk("accelerometer_sensor ALREADY PROBED.... so exit probe now\n");
		return -1;
	}

// DUAL
//#ifdef CONFIG_YAS_ACC_MULTI_SUPPORT
	struct accel_platform_data *platform_data;

	if (client->dev.platform_data != NULL)
		platform_data = client->dev.platform_data;

//#endif

	/* Setup private data */
	data = kzalloc(sizeof(struct yas_acc_private_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto ERR1;
	}
	yas_acc_set_data(data);

// Dual	
//#ifdef CONFIG_YAS_ACC_MULTI_SUPPORT
	data->used_chip = platform_data->used_chip;
	data->position = platform_data->position;
	memcpy(data->vendor_name, platform_data->vendor_name, MAX_LEN_OF_NAME);
	memcpy(data->chip_name, platform_data->chip_name, MAX_LEN_OF_NAME);
//#endif
	mutex_init(&data->driver_mutex);
	mutex_init(&data->data_mutex);
	mutex_init(&data->enable_mutex);
#if DEBUG
	mutex_init(&data->suspend_mutex);
#endif

	/* Setup i2c client */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto ERR2;
	}
	i2c_set_clientdata(client, data);
	data->client = client;

	/* Setup accelerometer core driver */
	err = yas_acc_core_driver_init(data);
	if (err < 0)
		goto ERR2;

	/* Setup driver interface */
	INIT_DELAYED_WORK(&data->work, yas_acc_work_func);

	/* Setup input device interface */
	err = yas_acc_input_init(data);
	if (err < 0)
		goto ERR3;

	/* Setup sysfs */
	err = sysfs_create_group(&data->input->dev.kobj,
				 &yas_acc_attribute_group);
	if (err < 0)
		goto ERR4;

#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	INIT_LIST_HEAD(&data->devfile_list);
	if (misc_register(&sensor_devfile) < 0)
		goto ERR4;
#endif
	err = sensors_register(data->accel_sensor_device,
		NULL, accel_sensor_attrs,
			"accelerometer_sensor");
	if (err) {
		pr_err("%s: cound not register accelerometer sensor device(%d).\n",
			__func__, err);
		goto out_sensor_register_failed;
	}
	printk("accelerometer_sensor probe finished\n");
	
	return 0;

out_sensor_register_failed:
ERR4:
	yas_acc_input_fini(data);
ERR3:
	yas_acc_core_driver_fini(data);
ERR2:
	kfree(data);
ERR1:
	return err;
}

static int yas_acc_remove(struct i2c_client *client)
{
	struct yas_acc_private_data *data = i2c_get_clientdata(client);
	struct yas_acc_driver *driver = data->driver;

#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	misc_deregister(&sensor_devfile);
#endif
	yas_acc_set_enable(driver, 0);
	sysfs_remove_group(&data->input->dev.kobj, &yas_acc_attribute_group);
	yas_acc_input_fini(data);
	yas_acc_core_driver_fini(data);
	kfree(data);

	return 0;
}

static int yas_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct yas_acc_private_data *data = i2c_get_clientdata(client);
	struct yas_acc_driver *driver = data->driver;

	mutex_lock(&data->enable_mutex);

	data->suspend_enable = yas_acc_get_enable(driver);
	if (data->suspend_enable)
		yas_acc_set_enable(driver, 0);

	mutex_unlock(&data->enable_mutex);

	return 0;
}

static int yas_acc_resume(struct i2c_client *client)
{
	struct yas_acc_private_data *data = i2c_get_clientdata(client);
	struct yas_acc_driver *driver = data->driver;

	mutex_lock(&data->enable_mutex);

	if (data->suspend_enable)
		yas_acc_set_enable(driver, 1);

	mutex_unlock(&data->enable_mutex);

	return 0;
}

static const struct i2c_device_id yas_acc_id[] = {
	{YAS_ACC_KERNEL_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, yas_acc_id);

struct i2c_driver yas_acc_driver = {
	.driver = {
		.name = "accelerometer",
		.owner = THIS_MODULE,
	},
	.probe = yas_acc_probe,
	.remove = yas_acc_remove,
	.suspend = yas_acc_suspend,
	.resume = yas_acc_resume,
	.id_table = yas_acc_id,
};

/* -------------------------------------------------------------------------- *
 *  Module init and exit
 * -------------------------------------------------------------------------- */
static int __init yas_acc_init(void)
{
	return i2c_add_driver(&yas_acc_driver);
}
module_init(yas_acc_init);

static void __exit yas_acc_exit(void)
{
	i2c_del_driver(&yas_acc_driver);
}
module_exit(yas_acc_exit);

MODULE_DESCRIPTION("accelerometer kernel driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(YAS_ACC_KERNEL_VERSION);