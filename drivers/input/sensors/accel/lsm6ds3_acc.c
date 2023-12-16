/* drivers/input/sensors/access/mpu6880_acc.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: Bin Yang<yangbin@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>


#define LSM6DS3_ACCEL_FS_ADDR		0x10
#define LSM6DS3_GYRO_FS_ADDR		0x11
#define LSM6DS3_RESET_ADDR		0x12
#define LSM6DS3_INT2_ON_INT1_ADDR	0x13

#define LSM6DS3_LIR_ADDR		0x58
#define LSM6DS3_FIFO_ODR_ADDR		0x0a
#define LSM6DS3_ROUNDING_ADDR		0x16

#define LSM6DS3_ACCEL_FS_2G_VAL		0x00
#define LSM6DS3_ACCEL_FS_4G_VAL		0x02
#define LSM6DS3_ACCEL_FS_8G_VAL		0x03
#define LSM6DS3_ACCEL_FS_16G_VAL	0x01
#define LSM6DS3_ACCEL_FS_2G_GAIN	61
#define LSM6DS3_ACCEL_FS_4G_GAIN	122
#define LSM6DS3_ACCEL_FS_8G_GAIN	244
#define LSM6DS3_ACCEL_FS_16G_GAIN	488

#define LSM6DS3_ACCEL_XOUT_H		0x28
#define LSM6DS3_WHOAMI				0x0F
#define LSM6DS3_DEVICE_ID			0x69
#define LSM6DS3_PRECISION			16
#define LSM6DS3_PWR_ACCEL			0x10
#define LSM6DS3_PWR_GYRO			0x11
#define LSM6DS3_INT1_CTRL			0x0D

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);
	int result = 0;

	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	if (!enable)
		sensor->ops->ctrl_data &= 0x0F;	/*power down*/
	else
		sensor->ops->ctrl_data |= 0x40;	/*104HZ*/
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if (result)
		printk("%s:fail to active sensor\n", __func__);

	return result;
}

static int sensor_init(struct i2c_client *client)
{
	int res = 0;
	unsigned char acc_reg_data = 0;
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);

	res = sensor->ops->active(client, 0, 0);
	if (res) {
		printk("%s:line=%d,error\n", __func__, __LINE__);
		return res;
	}
	sensor->status_cur = SENSOR_OFF;

	acc_reg_data = sensor_read_reg(client, LSM6DS3_ACCEL_FS_ADDR);
	acc_reg_data &= 0xF3;   /*2g 50Hz*/
	/*acc_reg_data |= 0x40; */  /*104HZ*/
	DBG("acc_reg_data[0x10] = 0x%2x\n", acc_reg_data);
	res = sensor_write_reg(client, LSM6DS3_ACCEL_FS_ADDR, acc_reg_data);
	if (res)
		return res;

	acc_reg_data = sensor_read_reg(client, LSM6DS3_PWR_GYRO);
	DBG("acc_reg_data[0x11] = 0x%2x\n", acc_reg_data);
	if ((acc_reg_data&0xF0) == 0) {
		acc_reg_data = sensor_read_reg(client, LSM6DS3_RESET_ADDR);
		acc_reg_data |= 0x41;
		DBG("acc_reg_data1 = 0x%2x\n", acc_reg_data);
	} else {
		DBG("acc_reg_data3 = 0x%2x\n", acc_reg_data);
		acc_reg_data = sensor_read_reg(client, LSM6DS3_RESET_ADDR);
		acc_reg_data |= 0x40;
		DBG("acc_reg_data2 = 0x%2x\n", acc_reg_data);
	}
	res = sensor_write_reg(client, LSM6DS3_RESET_ADDR, acc_reg_data);
	if (res)
		return res;

	res = sensor_write_reg(client, LSM6DS3_LIR_ADDR, 0x81);
	if (res)
		return res;

	res = sensor_write_reg(client, LSM6DS3_INT2_ON_INT1_ADDR, 0x20);
	if (res)
		return res;

	if (sensor->pdata->irq_enable) {
		acc_reg_data = sensor_read_reg(client, LSM6DS3_INT1_CTRL);
		acc_reg_data |= 0x01;
		res = sensor_write_reg(client, LSM6DS3_INT1_CTRL, acc_reg_data);
		if (res)
			return res;
	}

	return res;
}

static int gsensor_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);

	if (sensor->status_cur == SENSOR_ON) {
		/* Report acceleration sensor information */
		input_report_abs(sensor->input_dev, ABS_X, axis->x);
		input_report_abs(sensor->input_dev, ABS_Y, axis->y);
		input_report_abs(sensor->input_dev, ABS_Z, axis->z);
		input_sync(sensor->input_dev);
		DBG("Gsensor x==%d  y==%d z==%d\n", axis->x, axis->y, axis->z);
	}
	return 0;
}

#define GSENSOR_MIN 10
static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *)i2c_get_clientdata(client);
	struct sensor_platform_data *pdata = sensor->pdata;
	char buffer[6];
	int ret = 0;
	char value = 0;
	struct sensor_axis axis;
	short x, y, z;
	unsigned char reg_buf = 0;
        unsigned char i = 0;

	/*sensor->ops->read_len = 6*/
	if (sensor->ops->read_len < 6) {
		printk("%s:len is error,len=%d\n", __func__, sensor->ops->read_len);
		return -1;
	}
	memset(buffer, 0, 6);

	reg_buf = sensor->ops->read_reg;
        for (i = 0; i < sensor->ops->read_len; i++) {
                buffer[i] = sensor_read_reg(client, reg_buf);
                reg_buf++;
        }

	x = ((buffer[1] << 8) & 0xff00) + (buffer[0] & 0xFF);
	y = ((buffer[3] << 8) & 0xff00) + (buffer[2] & 0xFF);
	z = ((buffer[5] << 8) & 0xff00) + (buffer[4] & 0xFF);

	axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
	axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z;
	axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;

	//axis.x = 61*axis.x;
	//axis.y = 61*axis.y;
	//axis.z = 61*axis.z;

	if ((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) || (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) || (abs(sensor->axis.z - axis.z) > GSENSOR_MIN)) {
		gsensor_report_value(client, &axis);
		mutex_lock(&sensor->data_mutex);
		sensor->axis = axis;
		mutex_unlock(&sensor->data_mutex);
	}

    	/*read sensor intterupt status register*/
	if ((sensor->pdata->irq_enable) && (sensor->ops->int_status_reg >= 0)) {
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		DBG("%s:gsensor int status :0x%x\n", __func__, value);
	}

	return ret;
}

struct sensor_operate gsensor_lsm6ds3_ops = {
	.name				= "lsm6ds3_acc",
	.type				= SENSOR_TYPE_ACCEL,			/*sensor type and it should be correct*/
	.id_i2c				= ACCEL_ID_LSM6DS3,				/*i2c id number*/
	.read_reg			= LSM6DS3_ACCEL_XOUT_H,			/*read data*/
	.read_len			= 6,							/*data length*/
	.id_reg				= LSM6DS3_WHOAMI,				/*read device id from this register*/
	.id_data			= LSM6DS3_DEVICE_ID,			/*device id*/
	.precision			= LSM6DS3_PRECISION,			/*16 bit*/
	.ctrl_reg			= LSM6DS3_PWR_ACCEL,				/*enable or disable*/
	.int_status_reg			= SENSOR_UNKNOW_DATA,			/*intterupt status register*/
	//.range				= {-2000000, 2000000},			/*range value*61*/
	.range				= {-32768, 32768},			/*range*/
	.trig				= IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
	.active				= sensor_active,
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/

#if 0
/*function name should not be changed*/
static struct sensor_operate *gsensor_get_ops(void)
{
	return &gsensor_lsm6ds3_ops;
}

static int __init gsensor_lsm6ds3_init(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int result = 0;
	int type = ops->type;

	result = sensor_register_slave(type, NULL, NULL, gsensor_get_ops);
	return result;
}

static void __exit gsensor_lsm6ds3_exit(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int type = ops->type;

	sensor_unregister_slave(type, NULL, NULL, gsensor_get_ops);
}

module_init(gsensor_lsm6ds3_init);
module_exit(gsensor_lsm6ds3_exit);
#endif


static int gsensor_lsm6ds3_probe(struct i2c_client *client,
                                const struct i2c_device_id *devid)
{
        return sensor_register_device(client, NULL, devid, &gsensor_lsm6ds3_ops);
}

static int gsensor_lsm6ds3_remove(struct i2c_client *client)
{
        return sensor_unregister_device(client, NULL, &gsensor_lsm6ds3_ops);
}

static const struct i2c_device_id gsensor_lsm6ds3_id[] = {
        {"lsm6ds3_acc", ACCEL_ID_LSM6DS3},
        {}
};

static struct i2c_driver gsensor_lsm6ds3_driver = {
        .probe = gsensor_lsm6ds3_probe,
        .remove = gsensor_lsm6ds3_remove,
        .shutdown = sensor_shutdown,
        .id_table = gsensor_lsm6ds3_id,
        .driver = {
                .name = "gsensor_lsm6ds3",
        #ifdef CONFIG_PM
                .pm = &sensor_pm_ops,
        #endif
        },
};

module_i2c_driver(gsensor_lsm6ds3_driver);

MODULE_AUTHOR("lxq <lxq@chipsee.com>");
MODULE_DESCRIPTION("lsm6ds3 3-Axis accelerometer driver");
MODULE_LICENSE("GPL");
