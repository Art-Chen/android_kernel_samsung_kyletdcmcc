/*
 * Copyright (c) 2011 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include <linux/yas.h>

#ifdef CONFIG_YAS_ACC_MULTI_SUPPORT
#include <linux/yas_accel.h>
#else
// DUAL
//#define CHIP_NAME		"Kionix"
//#define VENDOR_NAME		"KXTJ2"
#endif

/* Axes data range  [um/s^2] */
#define YAS_KXTJ2_GRAVITY_EARTH                                         9806550
#define YAS_KXTJ2_ABSMIN_2G                     (-YAS_KXTJ2_GRAVITY_EARTH * 2)
#define YAS_KXTJ2_ABSMAX_2G                      (YAS_KXTJ2_GRAVITY_EARTH * 2)


/* Default parameters */
#define YAS_KXTJ2_DEFAULT_DELAY                                            100
#define YAS_KXTJ2_DEFAULT_POSITION                                            5

#define YAS_KXTJ2_MAX_DELAY                                             200
#define YAS_KXTJ2_MIN_DELAY                                               10

/* Registers */
#define YAS_KXTJ2_WHO_AM_I_REG						0x0f
#define YAS_KXTJ2_WHO_AM_I                                                 0x09

#define YAS_KXTJ2_CTRL_REG1                                                0x1B
#define YAS_KXTJ2_CTRL_REG2							0x1D
#define YAS_KXTJ2_INT_CTRL_REG1						0x1E
#define YAS_KXTJ2_INT_CTRL_REG2						0x1F
#define YAS_KXTJ2_DATA_CTRL_REG						0x21

/* CONTROL REGISTER 1 BITS */
#define PC1_OFF					0x7F
#define PC1_ON					(1 << 7)
#define DRDYE					(1 << 5)
#define RES_12BIT				(1 << 6)
#define RES_8BIT				(0 << 6)
#define YAS_KXTJ2_FS_2G		(0 << 3)
#define YAS_KXTJ2_FS_4G		(1 << 3)
#define YAS_KXTJ2_FS_8G		(2 << 3)
#define YAS_KXTJ2_FS_8G_14bit	(3<<3)
#define YAS_KXTJ2_FS_MASK		(0x18)

#define YAS_KXTJ2_ODR_12_5HZ							0
#define YAS_KXTJ2_ODR_25HZ								1
#define YAS_KXTJ2_ODR_50HZ								2
#define YAS_KXTJ2_ODR_100HZ							3
#define YAS_KXTJ2_ODR_200HZ							4
#define YAS_KXTJ2_ODR_400HZ							5
#define YAS_KXTJ2_ODR_800HZ							6
#define YAS_KXTJ2_ODR_1600HZ							7

#define YAS_KXTJ2_ACC_REG                                                  0x06

#define YAS_KXTJ2_RESOLUTION_HERE                                                 64
//#define DEBUG	1
/* -------------------------------------------------------------------------- */
/*  Structure definition                                                      */
/* -------------------------------------------------------------------------- */
#if 0 // DUAL
/* Output data rate */
struct yas_kxtj2_odr {
	unsigned long delay;          /* min delay (msec) in the range of ODR */
	unsigned char odr;            /* bandwidth register value             */
};

/* Axes data */
struct yas_kxtj2_acceleration {
	int x;
	int y;
	int z;
	int x_raw;
	int y_raw;
	int z_raw;
};

/* Driver private data */
struct yas_kxtj2_data {
	uint8_t chip_id;
	int initialize;
	int i2c_open;
	int enable;
	int delay;
	int position;
	int threshold;
	int filter_enable;
	uint8_t odr;
	struct yas_vector offset;
	struct yas_kxtj2_acceleration last;
};
#endif

/* -------------------------------------------------------------------------- */
/*  Data                                                                      */
/* -------------------------------------------------------------------------- */
/* Control block */
static struct yas_acc_driver  cb;
static struct yas_acc_driver *pcb;
static struct yas_sensor_data kxtj2_acc_data;  // DUAL

/* Output data rate */
static const struct yas_sensor_odr yas_kxtj2_odr_tbl[] = {
	{3,   YAS_KXTJ2_ODR_800HZ},
	{5,   YAS_KXTJ2_ODR_400HZ},
	{10,   YAS_KXTJ2_ODR_200HZ},
	{20,  YAS_KXTJ2_ODR_100HZ},
	{40,  YAS_KXTJ2_ODR_50HZ},
	{80,  YAS_KXTJ2_ODR_25HZ},
	{0, YAS_KXTJ2_ODR_12_5HZ},
};

/* Transformation matrix for chip mounting position */
static const int yas_kxtj2_position_map[][3][3] = {
	{ { 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1} },   /* top/upper-left     */
	{ { 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1} },   /* top/upper-right    */
	{ { 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1} },   /* top/lower-right    */
	{ {-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1} },   /* top/lower-left     */
	{ { 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1} },   /* bottom/upper-right */
	{ {-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1} },   /* bottom/upper-left  */
	{ { 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1} },   /* bottom/lower-left  */
	{ { 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1} },   /* bottom/lower-right */
};

/* -------------------------------------------------------------------------- */
/*  Prototype declaration                                                     */
/* -------------------------------------------------------------------------- */
static void yas_kxtj2_init_data(void);
static int yas_kxtj2_ischg_enable(int);
static int yas_kxtj2_read_reg(unsigned char, unsigned char *, unsigned char);
static int yas_kxtj2_write_reg(unsigned char, unsigned char *, unsigned char);
static int yas_kxtj2_read_reg_byte(unsigned char);
static int yas_kxtj2_write_reg_byte(int, int);
static int yas_kxtj2_lock(void);
static int yas_kxtj2_unlock(void);
static int yas_kxtj2_i2c_open(void);
static int yas_kxtj2_i2c_close(void);
static int yas_kxtj2_msleep(int);
static int yas_kxtj2_power_up(void);
static int yas_kxtj2_power_down(void);
static int yas_kxtj2_init(void);
static int yas_kxtj2_term(void);
static int yas_kxtj2_get_delay(void);
static int yas_kxtj2_set_delay(int);
static int yas_kxtj2_get_offset(struct yas_vector *);
static int yas_kxtj2_set_offset(struct yas_vector *);
static int yas_kxtj2_get_enable(void);
static int yas_kxtj2_set_enable(int);
static int yas_kxtj2_get_filter(struct yas_acc_filter *);
static int yas_kxtj2_set_filter(struct yas_acc_filter *);
static int yas_kxtj2_get_filter_enable(void);
static int yas_kxtj2_set_filter_enable(int);
static int yas_kxtj2_get_position(void);
static int yas_kxtj2_set_position(int);
static int yas_kxtj2_measure(int *, int *);
static int kxtj2_acc_calibration(int);
#if DEBUG
static int yas_get_register(uint8_t, uint8_t *);
#endif

/* -------------------------------------------------------------------------- */
/*  Local functions                                                           */
/* -------------------------------------------------------------------------- */

static void yas_kxtj2_init_data(void)
{
	kxtj2_acc_data.chip_id = 0;
	kxtj2_acc_data.initialize = 0;
	kxtj2_acc_data.enable = 0;
	kxtj2_acc_data.delay = YAS_KXTJ2_DEFAULT_DELAY;
	kxtj2_acc_data.offset.v[0] = 0;
	kxtj2_acc_data.offset.v[1] = 0;
	kxtj2_acc_data.offset.v[2] = 0;
	kxtj2_acc_data.position = YAS_KXTJ2_DEFAULT_POSITION;
	kxtj2_acc_data.threshold = YAS_ACC_DEFAULT_FILTER_THRESH;
	kxtj2_acc_data.filter_enable = 0;
	kxtj2_acc_data.last.x = 0;
	kxtj2_acc_data.last.y = 0;
	kxtj2_acc_data.last.z = 0;
	kxtj2_acc_data.last.x_raw = 0;
	kxtj2_acc_data.last.y_raw = 0;
	kxtj2_acc_data.last.z_raw = 0;
}

static int yas_kxtj2_ischg_enable(int enable)
{
	if (kxtj2_acc_data.enable == enable)
		return 0;

	return 1;
}

/* register access functions */
static int yas_kxtj2_read_reg(unsigned char adr, unsigned char *buf,
			       unsigned char len)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;
	int err;

	if (kxtj2_acc_data.i2c_open) {
		err = cbk->device_read(adr, buf, len);
		if (err != 0)
			return err;

		return err;
	}

	return YAS_NO_ERROR;
}

static int yas_kxtj2_write_reg(unsigned char adr, unsigned char *buf,
				unsigned char len)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;
	int err;

	if (kxtj2_acc_data.i2c_open) {
		err = cbk->device_write(adr, buf, len);
		if (err != 0)
			return err;

		return err;
	}

	return YAS_NO_ERROR;
}

static int yas_kxtj2_read_reg_byte(unsigned char adr)
{
	unsigned char buf = 0xff;
	int err;

	err = yas_kxtj2_read_reg(adr, &buf, 1);
	if (err == 0)
		return buf;

	return 0;
}

static int yas_kxtj2_write_reg_byte(int adr, int val)
{
	return yas_kxtj2_write_reg((unsigned char)adr,
				    (unsigned char *)&val, 1);
}

#define yas_kxtj2_read_bits(r) \
	((yas_kxtj2_read_reg_byte(r##_REG) & r##_MASK) >> r##_SHIFT)

#define yas_kxtj2_update_bits(r, v) \
	yas_kxtj2_write_reg_byte(r##_REG, \
			   ((yas_kxtj2_read_reg_byte(r##_REG) & ~r##_MASK) | \
			    ((v) << r##_SHIFT)))

static int yas_kxtj2_lock(void)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;
	int err;

	if (cbk->lock != NULL && cbk->unlock != NULL)
		err = cbk->lock();
	else
		err = YAS_NO_ERROR;

	return err;
}

static int yas_kxtj2_unlock(void)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;
	int err;

	if (cbk->lock != NULL && cbk->unlock != NULL)
		err = cbk->unlock();
	else
		err = YAS_NO_ERROR;

	return err;
}

static int yas_kxtj2_i2c_open(void)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;
	int err;

	if (kxtj2_acc_data.i2c_open == 0) {
		err = cbk->device_open();
		if (err != YAS_NO_ERROR)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		kxtj2_acc_data.i2c_open = 1;
	}

	return YAS_NO_ERROR;
}

static int yas_kxtj2_i2c_close(void)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;
	int err;

	if (kxtj2_acc_data.i2c_open != 0) {
		err = cbk->device_close();
		if (err != YAS_NO_ERROR)
			return YAS_ERROR_DEVICE_COMMUNICATION;
		kxtj2_acc_data.i2c_open = 0;
	}
	return YAS_NO_ERROR;
}
#if 1
static int yas_kxtj2_msleep(int msec)
{
	struct yas_acc_driver_callback *cbk = &pcb->callback;

	if (msec <= 0)
		return YAS_ERROR_ARG;

	cbk->msleep(msec);

	return YAS_NO_ERROR;
}
#endif
static int yas_kxtj2_power_up(void)
{
	int err, ctrl_reg1 = 0;

	/* turn on outputs */
	ctrl_reg1 = yas_kxtj2_read_reg_byte(YAS_KXTJ2_CTRL_REG1);
	ctrl_reg1 |= PC1_ON;
	err = yas_kxtj2_write_reg_byte(YAS_KXTJ2_CTRL_REG1, ctrl_reg1);
	if (err != 0)
		return err;
	mdelay(81); // consider to min. 12.5Hz - Start up time 80ms

	return YAS_NO_ERROR;
}

static int yas_kxtj2_power_down(void)
{
	int err, ctrl_reg1 = 0;
	ctrl_reg1 = yas_kxtj2_read_reg_byte(YAS_KXTJ2_CTRL_REG1);
	ctrl_reg1 &= PC1_OFF;
	err = yas_kxtj2_write_reg_byte(YAS_KXTJ2_CTRL_REG1, ctrl_reg1);
	if (err != 0)
		return err;

	return YAS_NO_ERROR;
}

static int yas_kxtj2_init(void)
{
	struct yas_acc_filter filter;
	int err;
	int id, ctrl_reg1;

	/* Check intialize */
	if (kxtj2_acc_data.initialize == 1)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Init data */
	yas_kxtj2_init_data();

	/* Open i2c */
	err = yas_kxtj2_i2c_open();
	if (err != YAS_NO_ERROR)
		return err;

	/* Check id */
	id = yas_kxtj2_read_reg_byte(YAS_KXTJ2_WHO_AM_I_REG);
	if (id != YAS_KXTJ2_WHO_AM_I) {
		printk("-ERR- KXTJ2 : WHO_AM_I = 0x%02x\n", id);
		yas_kxtj2_i2c_close();
		return YAS_ERROR_CHIP_ID;
	}
	printk("KXTJ2 : WHO_AM_I = 0x%02x\n", id);

	/* Softreset */
	yas_kxtj2_write_reg_byte(YAS_KXTJ2_CTRL_REG2, (1<<7));
	yas_kxtj2_msleep(80);

	/* Set G-range/Resolution to 2g@12-bits */
	ctrl_reg1 = yas_kxtj2_read_reg_byte(YAS_KXTJ2_CTRL_REG1);
	//printk("-- KXTJ2 : YAS_KXTJ2_CTRL_REG1 = 0x%02x\n", ctrl_reg1);
	ctrl_reg1 |= (RES_12BIT | YAS_KXTJ2_FS_2G);
	//ctrl_reg1 |= (RES_8BIT | YAS_KXTJ2_FS_2G);
	yas_kxtj2_write_reg_byte(YAS_KXTJ2_CTRL_REG1, ctrl_reg1);

	kxtj2_acc_data.chip_id = (uint8_t)id;
	kxtj2_acc_data.initialize = 1;

	yas_kxtj2_set_delay(YAS_KXTJ2_DEFAULT_DELAY);
	yas_kxtj2_set_position(YAS_KXTJ2_DEFAULT_POSITION);
	filter.threshold = YAS_ACC_DEFAULT_FILTER_THRESH;
	yas_kxtj2_set_filter(&filter);
	yas_kxtj2_power_down();

	return YAS_NO_ERROR;
}

static int yas_kxtj2_term(void)
{
	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_kxtj2_set_enable(0);

	/* Close I2C */
	yas_kxtj2_i2c_close();

	kxtj2_acc_data.initialize = 0;

	return YAS_NO_ERROR;
}

static int yas_kxtj2_get_delay(void)
{
	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return kxtj2_acc_data.delay;
}

static int yas_kxtj2_set_delay(int delay)
{
	unsigned char buf;
	int i;

	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	kxtj2_acc_data.delay = delay;
	printk("KXTJ2 %s: acc_data.delay %d msec \n", __func__, kxtj2_acc_data.delay);
	/* Use the lowest ODR that can support the requested poll interval */
	for (i=0; i < ARRAY_SIZE(yas_kxtj2_odr_tbl); i++) {
		kxtj2_acc_data.odr = yas_kxtj2_odr_tbl[i].odr;
		if(kxtj2_acc_data.delay < yas_kxtj2_odr_tbl[i].delay)
			break;
	}
	printk(" KXTJ2%s: [%d] ODR = %d, delay = %d msec \n", __func__, i, kxtj2_acc_data.odr, kxtj2_acc_data.delay);


	if (yas_kxtj2_get_enable()) {
		printk("-I- KXTJ2: kxtj2 is already enabled and so, disable PC1 bit to configure the ODR\n");
		yas_kxtj2_power_down();
		//yas_kxtj2_write_reg_byte(YAS_KXTJ2_DATA_CTRL_REG, acc_data.odr);
		yas_kxtj2_power_up();
	} else {
		//yas_kxtj2_power_up();
		//yas_kxtj2_write_reg_byte(YAS_KXTJ2_DATA_CTRL_REG, kxtj2_acc_data.odr);
		//yas_kxtj2_power_down();
	}
	return YAS_NO_ERROR;
}

static int yas_kxtj2_get_offset(struct yas_vector *offset)
{
	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;
	
	*offset = kxtj2_acc_data.offset;

	return YAS_NO_ERROR;
}

static int yas_kxtj2_set_offset(struct yas_vector *offset)
{
	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	kxtj2_acc_data.offset = *offset;
	return YAS_NO_ERROR;
}

static int yas_kxtj2_get_enable(void)
{
	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return kxtj2_acc_data.enable;
}

static int yas_kxtj2_set_enable(int enable)
{
	int err, ctrl_reg1 = 0;

	printk("KXTJ2 : yas_kxtj2_set_enable  enable= %d,=======\n", enable);
	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	if (yas_kxtj2_ischg_enable(enable)) {
		if (enable) {
			/* Open i2c */
			err = yas_kxtj2_i2c_open();
			if (err != YAS_NO_ERROR)
				return err;
			/* Reset chip */
			yas_kxtj2_write_reg_byte(YAS_KXTJ2_CTRL_REG2, (1<<7));
			yas_kxtj2_msleep(80);
			kxtj2_acc_data.enable = enable;
				/* Set G-range */
			ctrl_reg1 = yas_kxtj2_read_reg_byte(YAS_KXTJ2_CTRL_REG1);
			ctrl_reg1 |= (RES_12BIT | YAS_KXTJ2_FS_2G);
			//ctrl_reg1 |= (RES_8BIT | YAS_KXTJ2_FS_2G);
			yas_kxtj2_write_reg_byte(YAS_KXTJ2_CTRL_REG1, ctrl_reg1);
			yas_kxtj2_set_delay(kxtj2_acc_data.delay);
			yas_kxtj2_power_up();
		} else {
			yas_kxtj2_power_down();
			err = yas_kxtj2_i2c_close();
			if (err != YAS_NO_ERROR)
				return err;
		}
	}

	kxtj2_acc_data.enable = enable;

	return YAS_NO_ERROR;
}

static int yas_kxtj2_get_filter(struct yas_acc_filter *filter)
{
	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	filter->threshold = kxtj2_acc_data.threshold;

	return YAS_NO_ERROR;
}

static int yas_kxtj2_set_filter(struct yas_acc_filter *filter)
{
	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	kxtj2_acc_data.threshold = filter->threshold;

	return YAS_NO_ERROR;
}

static int yas_kxtj2_get_filter_enable(void)
{
	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	return kxtj2_acc_data.filter_enable;
}

static int yas_kxtj2_set_filter_enable(int enable)
{
	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	kxtj2_acc_data.filter_enable = enable;

	return YAS_NO_ERROR;
}

static int yas_kxtj2_get_position(void)
{
	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	printk("KXTJ2 : yas_kxtj2_get_position  kxtj2_acc_data.position= %d,=======\n", kxtj2_acc_data.position);
	return kxtj2_acc_data.position;
}

static int yas_kxtj2_set_position(int position)
{
	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	kxtj2_acc_data.position = position;
	printk(KERN_WARNING "KXTJ2 yas_kxtj2_set_position position=(%d) \n",position);

	return YAS_NO_ERROR;
}

static int yas_kxtj2_data_filter(int data[], int raw[],
				  struct yas_sensor_acceleration *accel)
{
	int filter_enable = kxtj2_acc_data.filter_enable;
	int threshold = kxtj2_acc_data.threshold;

	if (filter_enable) {
		if ((ABS(kxtj2_acc_data.last.x - data[0]) > threshold) ||
		    (ABS(kxtj2_acc_data.last.y - data[1]) > threshold) ||
		    (ABS(kxtj2_acc_data.last.z - data[2]) > threshold)) {
			accel->x = data[0];
			accel->y = data[1];
			accel->z = data[2];
			accel->x_raw = raw[0];
			accel->y_raw = raw[1];
			accel->z_raw = raw[2];
		} else {
			*accel = kxtj2_acc_data.last;
		}
	} else {
		accel->x = data[0];
		accel->y = data[1];
		accel->z = data[2];
		accel->x_raw = raw[0];
		accel->y_raw = raw[1];
		accel->z_raw = raw[2];
	}

	return YAS_NO_ERROR;
}

#if 0
static void yas_kxtj2_current_time(int32_t *sec, int32_t *msec)
{
	struct timeval tv;
	do_gettimeofday(&tv);
	*sec = tv.tv_sec;
	*msec = (tv.tv_usec % 1000 == 0) ? \
		tv.tv_usec / 1000 : tv.tv_usec / 1000 + 1;
}
#endif

static int yas_kxtj2_measure(int *out_data, int *out_raw)
{
	struct yas_sensor_acceleration accel;
	unsigned char buf[6];
	int32_t raw[3], data[3];
	int pos = kxtj2_acc_data.position;
	int i, j;
#if 0
	int32_t sec;
	int32_t msec;
#endif

	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	// 12-BIT
	/* Read acceleration data */
	if (yas_kxtj2_read_reg(YAS_KXTJ2_ACC_REG, buf, 6) != 0) {
		for (i = 0; i < 3; i++)
			raw[i] = 0;
	} else {
		for (i = 0; i < 3; i++) {
			raw[i] = (int16_t)((buf[i*2+1] << 8) | buf[i*2]) >> 4;
			raw[i] = raw[i]/16;
		}
	}

	raw[0] = raw[0] - kxtj2_acc_data.offset.v[0];
	raw[1] = raw[1] - kxtj2_acc_data.offset.v[1];
	raw[2] = raw[2] - kxtj2_acc_data.offset.v[2];
	
	/* for X, Y, Z axis */
	for (i = 0; i < 3; i++) {
		/* coordinate transformation */
		data[i] = 0;
		for (j = 0; j < 3; j++)
			data[i] += raw[j] * yas_kxtj2_position_map[pos][i][j];
	}
	
	yas_kxtj2_data_filter(data, raw, &accel);
	//printk("KXTJ2 : yas_kxtj2_measure  X,Y,Z= %d,%d,%d======= after filter\n", accel.x_raw,accel.y_raw,accel.z_raw);

	out_data[0] = accel.x;
	out_data[1] = accel.y;
	out_data[2] = accel.z;
	out_raw[0] = accel.x_raw;
	out_raw[1] = accel.y_raw;
	out_raw[2] = accel.z_raw;

	kxtj2_acc_data.last = accel;

	return YAS_NO_ERROR;
}

/* -------------------------------------------------------------------------- */
static int yas_cbk_kxtj2_init(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_kxtj2_lock();
	err = yas_kxtj2_init();
	yas_kxtj2_unlock();

	return err;
}

static int yas_cbk_kxtj2_term(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_kxtj2_lock();
	err = yas_kxtj2_term();
	yas_kxtj2_unlock();

	return err;
}

static int yas_cbk_kxtj2_get_delay(void)
{
	int ret;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_kxtj2_lock();
	ret = yas_kxtj2_get_delay();
	yas_kxtj2_unlock();

	return ret;
}

static int yas_cbk_kxtj2_set_delay(int delay)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (delay < 0 || delay > YAS_KXTJ2_MAX_DELAY)
		return YAS_ERROR_ARG;
	else if (delay < YAS_KXTJ2_MIN_DELAY)
		delay = YAS_KXTJ2_MIN_DELAY;

	yas_kxtj2_lock();
	err = yas_kxtj2_set_delay(delay);
	yas_kxtj2_unlock();

	return err;
}

static int yas_cbk_kxtj2_get_offset(struct yas_vector *offset)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (offset == NULL)
		return YAS_ERROR_ARG;

	yas_kxtj2_lock();
	err = yas_kxtj2_get_offset(offset);
	yas_kxtj2_unlock();

	return err;
}

static int yas_cbk_kxtj2_set_offset(struct yas_vector *offset)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (offset == NULL ||
	    offset->v[0] < YAS_KXTJ2_ABSMIN_2G ||
	    YAS_KXTJ2_ABSMAX_2G < offset->v[0] ||
	    offset->v[1] < YAS_KXTJ2_ABSMIN_2G ||
	    YAS_KXTJ2_ABSMAX_2G < offset->v[1] ||
	    offset->v[2] < YAS_KXTJ2_ABSMIN_2G ||
	    YAS_KXTJ2_ABSMAX_2G < offset->v[2])
		return YAS_ERROR_ARG;

	yas_kxtj2_lock();
	err = yas_kxtj2_set_offset(offset);
	yas_kxtj2_unlock();

	return err;
}

static int yas_cbk_kxtj2_get_enable(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_kxtj2_lock();
	err = yas_kxtj2_get_enable();
	yas_kxtj2_unlock();

	return err;
}

static int yas_cbk_kxtj2_set_enable(int enable)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (enable != 0)
		enable = 1;

	yas_kxtj2_lock();
	err = yas_kxtj2_set_enable(enable);
	yas_kxtj2_unlock();

	return err;
}

static int yas_cbk_kxtj2_get_filter(struct yas_acc_filter *filter)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (filter == NULL)
		return YAS_ERROR_ARG;

	yas_kxtj2_lock();
	err = yas_kxtj2_get_filter(filter);
	yas_kxtj2_unlock();

	return err;
}

static int yas_cbk_kxtj2_set_filter(struct yas_acc_filter *filter)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (filter == NULL ||
	    filter->threshold < 0 ||
	    filter->threshold > YAS_KXTJ2_ABSMAX_2G)
		return YAS_ERROR_ARG;

	yas_kxtj2_lock();
	err = yas_kxtj2_set_filter(filter);
	yas_kxtj2_unlock();

	return err;
}

static int yas_cbk_kxtj2_get_filter_enable(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_kxtj2_lock();
	err = yas_kxtj2_get_filter_enable();
	yas_kxtj2_unlock();

	return err;
}

static int yas_cbk_kxtj2_set_filter_enable(int enable)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (enable != 0)
		enable = 1;

	yas_kxtj2_lock();
	err = yas_kxtj2_set_filter_enable(enable);
	yas_kxtj2_unlock();

	return err;
}

static int yas_cbk_kxtj2_get_position(void)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	yas_kxtj2_lock();
	err = yas_kxtj2_get_position();
	yas_kxtj2_unlock();

	return err;
}

static int yas_cbk_kxtj2_set_position(int position)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (!((position >= 0) && (position <= 7)))
		return YAS_ERROR_ARG;

	yas_kxtj2_lock();
	err = yas_kxtj2_set_position(position);
	yas_kxtj2_unlock();

	return err;
}

static int yas_cbk_kxtj2_measure(struct yas_acc_data *data)
{
	int err;

	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (data == NULL)
		return YAS_ERROR_ARG;

	yas_kxtj2_lock();
	err = yas_kxtj2_measure(data->xyz.v, data->raw.v);
	yas_kxtj2_unlock();

	return err;
}

static int kxtj2_acc_calibration(int onoff)
{
	int err=0;
	struct acc_cal_data cal_data;
	unsigned char buf[6];
	int32_t raw[3], data[3];
	struct yas_vector offset;	
	int pos = kxtj2_acc_data.position;
	int i, j;	
	int layout[3] = {0, 0, 1};	

	printk(KERN_WARNING "KXTJ2 kxtj2_acc_calibration onoff=(%d) \n",onoff);

	if (onoff != 0)
		onoff = 1;
	
	/* Check intialize */
	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	if (onoff == NULL)
		return YAS_ERROR_ARG;

	yas_kxtj2_lock();

	layout[2] = YAS_KXTJ2_RESOLUTION_HERE*yas_kxtj2_position_map[CONFIG_INPUT_YAS_ACCELEROMETER_POSITION][2][2];		
	if(onoff)/*calibrate*/
	{
		/* Read acceleration data */
		if (yas_kxtj2_read_reg(YAS_KXTJ2_ACC_REG, buf, 6) != 0) {
			cal_data.x = 0;
			cal_data.y = 0;
			cal_data.z = 0;		
		} else {
			for (i = 0; i < 3; i++) {
				raw[i] = (int16_t)((buf[i*2+1] << 8) | buf[i*2]) >> 4;
				/* normalization FOR KYLETD*/
				raw[i] = raw[i]/16; 			
			}
			cal_data.x = raw[0] - layout[0];
			cal_data.y = raw[1] - layout[1];
			cal_data.z = raw[2] - layout[2];					
		}
		printk("KXTJ2 ACC CAL SAVED !!! (x,y,z)=(%d,%d,%d) \n",cal_data.x,cal_data.y,cal_data.z);
		
	}
	else
	{
		cal_data.x = 0;
		cal_data.y = 0;
		cal_data.z = 0;
		printk("KXTJ2 ACC CAL ERASED !!! \n");
		//err=1; //means erased
	}

	offset.v[0] = cal_data.x;
	offset.v[1] = cal_data.y;
	offset.v[2] = cal_data.z;
	err=yas_kxtj2_set_offset(&offset);	// set offset		

	yas_kxtj2_unlock();

	return err; /* 0 means NoError */
}

#if DEBUG
static int yas_cbk_kxtj2_get_register(uint8_t adr, uint8_t *val)
{
	int open;

	if (pcb == NULL)
		return YAS_ERROR_NOT_INITIALIZED;

	/* Check initialize */
	if (kxtj2_acc_data.initialize == 0)
		return YAS_ERROR_NOT_INITIALIZED;

	open = kxtj2_acc_data.i2c_open;

	yas_kxtj2_i2c_open();

	*val = yas_kxtj2_read_reg_byte(adr);

	if (open == 0)
		yas_kxtj2_i2c_close();

	return YAS_NO_ERROR;
}
#endif
/* -------------------------------------------------------------------------- */
/*  Global function                                                           */
/* -------------------------------------------------------------------------- */
//Dual
//#ifdef CONFIG_YAS_ACC_MULTI_SUPPORT
int yas_acc_driver_kxtj2_init(struct yas_acc_driver *f)
//#else
//int yas_acc_driver_init(struct yas_acc_driver *f)
//#endif
{
	struct yas_acc_driver_callback *cbk;

	/* Check parameter */
	if (f == NULL)
		return YAS_ERROR_ARG;

	cbk = &f->callback;
	if (cbk->device_open == NULL ||
	    cbk->device_close == NULL ||
	    cbk->device_write == NULL ||
	    cbk->device_read == NULL ||
	    cbk->msleep == NULL)
		return YAS_ERROR_ARG;

	/* Clear intialize */
	yas_kxtj2_term();

	/* Set callback interface */
	cb.callback = *cbk;

	/* Set driver interface */
	f->init = yas_cbk_kxtj2_init;
	f->term = yas_cbk_kxtj2_term;
	f->get_delay = yas_cbk_kxtj2_get_delay;
	f->set_delay = yas_cbk_kxtj2_set_delay;
	f->get_offset = yas_cbk_kxtj2_get_offset;
	f->set_offset = yas_cbk_kxtj2_set_offset;
	f->get_enable = yas_cbk_kxtj2_get_enable;
	f->set_enable = yas_cbk_kxtj2_set_enable;
	f->get_filter = yas_cbk_kxtj2_get_filter;
	f->set_filter = yas_cbk_kxtj2_set_filter;
	f->get_filter_enable = yas_cbk_kxtj2_get_filter_enable;
	f->set_filter_enable = yas_cbk_kxtj2_set_filter_enable;
	f->get_position = yas_cbk_kxtj2_get_position;
	f->set_position = yas_cbk_kxtj2_set_position;
	f->measure = yas_cbk_kxtj2_measure;
	f->acc_calibration = kxtj2_acc_calibration;
#if DEBUG
	f->get_register = yas_cbk_kxtj2_get_register;
#endif
	pcb = &cb;

	return YAS_NO_ERROR;
}
