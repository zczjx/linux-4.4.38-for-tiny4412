/*
 * linux/drivers/hwmon/mma7660.c
 *
 * 3-Axis Orientation/Motion Detection Sensor support
 *
 * Copyright (C) 2009-2010 Freescale Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
 
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input-polldev.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define MMA7660_NAME		"mma7660"

/* MMA7760 Registers */
#define MMA7660_XOUT			0x00	// 6-bit output value X
#define MMA7660_YOUT			0x01	// 6-bit output value Y
#define MMA7660_ZOUT			0x02	// 6-bit output value Z
#define MMA7660_TILT			0x03	// Tilt status
#define MMA7660_SRST			0x04	// Sampling Rate Status
#define MMA7660_SPCNT			0x05	// Sleep Count
#define MMA7660_INTSU			0x06	// Interrupt Setup
#define MMA7660_MODE			0x07	// Mode
#define MMA7660_SR				0x08	// Auto-Wake/Sleep and Debounce Filter
#define MMA7660_PDET			0x09	// Tap Detection
#define MMA7660_PD				0x0a	// Tap Debounce Count

struct mma7660_platform_data {
	/* parameters for input device */
	int poll_interval;
	int input_fuzz;
	int input_flat;
};

#define POLL_INTERVAL		100
#define INPUT_FUZZ			4
#define INPUT_FLAT			4

static struct i2c_client	*mma7660_client;

static struct device		*hwmon_dev;
static struct input_polled_dev	*mma7660_idev;
static struct mma7660_platform_data	*plat_data;

static int					last_tilt = 0;
static int					oper_mode;


/*-----------------------------------------------------------------------------
 * MMA7660 operations
 */

#define __need_retry(__v)	(__v & (1 << 6))
#define __is_negative(__v)	(__v & (1 << 5))

static const char *mma7660_bafro[] = {
	"Unknown", "Front", "Back"
};
static const char *mma7660_pola[] = {
	"Unknown",
	"Left", "Right",
	"Rsvd", "Rsvd",
	"Down", "Up",
	"Rsvd",
};

static int mma7660_read_xyz(struct i2c_client *client, int idx, int *xyz)
{
	int val;

	do {
		val = i2c_smbus_read_byte_data(client, idx + MMA7660_XOUT);
		if (val < 0) {
			dev_err(&client->dev, "Read register %02x failed, %d\n",
					idx + MMA7660_XOUT, val);
			return -EIO;
		}
	} while (__need_retry(val));

	*xyz = __is_negative(val) ? (val | ~0x3f) : (val & 0x3f);

	return 0;
}

static int mma7660_read_tilt(struct i2c_client *client, int *tilt)
{
	int val;

	do {
		val = i2c_smbus_read_byte_data(client, MMA7660_TILT);
		if (val < 0) {
			dev_err(&client->dev, "Read register %02x failed, %d\n",
					MMA7660_TILT, val);
			return -EIO;
		}
	} while (__need_retry(val));

	*tilt = (val & 0xff);

	return 0;
}

static int mma7660_initialize(struct i2c_client *client)
{
	int val;

	/* Using test mode to probe chip */
	i2c_smbus_write_byte_data(client, MMA7660_MODE, 0x00);
	mdelay(10);
	i2c_smbus_write_byte_data(client, MMA7660_MODE, 0x04);
	mdelay(10);
	i2c_smbus_write_byte_data(client, MMA7660_XOUT, 0x3f);
	i2c_smbus_write_byte_data(client, MMA7660_YOUT, 0x01);
	i2c_smbus_write_byte_data(client, MMA7660_ZOUT, 0x15);
	val = i2c_smbus_read_byte_data(client, MMA7660_ZOUT);
	if (val != 0x15) {
		dev_err(&client->dev, "no device\n");
		return -ENODEV;
	}

	/* Goto standby mode for configuration */
	i2c_smbus_write_byte_data(client, MMA7660_MODE, 0x00);
	mdelay(10);

	/* Sample rate: 64Hz / 16Hz; Filt: 3 samples  */
	i2c_smbus_write_byte_data(client, MMA7660_SR, ((2<<5) | (1<<3) | 1));

	/* Sleep count */
	i2c_smbus_write_byte_data(client, MMA7660_SPCNT, 0xA0);

	/* Tap detect and debounce ~4ms */
	i2c_smbus_write_byte_data(client, MMA7660_PDET, 4);
	i2c_smbus_write_byte_data(client, MMA7660_PD, 15);

	/* Enable interrupt except exiting Auto-Sleep */
	i2c_smbus_write_byte_data(client, MMA7660_INTSU, 0xe7);

	/* IPP, Auto-wake, auto-sleep and standby */
	i2c_smbus_write_byte_data(client, MMA7660_MODE, 0x59);
	mdelay(10);

	/* Save current tilt status */
	mma7660_read_tilt(client, &last_tilt);

	mma7660_client = client;
	return 0;
}


/*-----------------------------------------------------------------------------
 * sysfs group support
 */

static ssize_t mma7660_show_regs(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int reg, val;
	int i, len = 0;

	for (reg = 0; reg < 0x0b; reg++) {
		val = i2c_smbus_read_byte_data(mma7660_client, reg);
		len += sprintf(buf + len, "REG: 0x%02x = 0x%02x ...... [ ", reg, val);
		for (i = 7; i >= 0; i--) {
			len += sprintf(buf + len, "%d", (val >> i) & 1);
			if ((i % 4) == 0) {
				len += sprintf(buf + len, " ");
			}
		}
		len += sprintf(buf + len, "]\n");
	}

	return len;
}

static ssize_t mma7660_write_reg(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int reg, val;
	int ret;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2) {
		if (reg >= 0 && reg <= 0x0a) {
			i2c_smbus_write_byte_data(mma7660_client, reg, val);
			val = i2c_smbus_read_byte_data(mma7660_client, reg);
			printk("REG: 0x%02x = 0x%02x\n", reg, val);
		}
	}

	return count;
}

static ssize_t mma7660_show_xyz_g(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int axis[3];
	int i;

	for (i = 0; i < 3; i++) {
		mma7660_read_xyz(mma7660_client, i, &axis[i]);
	}

	return sprintf(buf, "%3d, %3d, %3d\n", axis[0], axis[1], axis[2]);
}

static ssize_t mma7660_show_axis_g(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int n = to_sensor_dev_attr(attr)->index;
	int val;

	mma7660_read_xyz(mma7660_client, n, &val);

	return sprintf(buf, "%3d\n", val);
}

static ssize_t mma7660_show_tilt(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int val = 0, len = 0;

	mma7660_read_tilt(mma7660_client, &val);

	len += sprintf(buf + len, "%s", mma7660_bafro[val & 0x03]);
	len += sprintf(buf + len, ", %s", mma7660_pola[(val >> 2) & 0x07]);
	if (val & (1 << 5)) {
		len += sprintf(buf + len, ", Tap");
	}
	if (val & (1 << 7)) {
		len += sprintf(buf + len, ", Shake");
	}
	len += sprintf(buf + len, "\n");

	return len;
}

static SENSOR_DEVICE_ATTR(registers, 0660,
		mma7660_show_regs, mma7660_write_reg, 0);
static SENSOR_DEVICE_ATTR(x_axis_g, S_IRUGO, mma7660_show_axis_g, NULL, 0);
static SENSOR_DEVICE_ATTR(y_axis_g, S_IRUGO, mma7660_show_axis_g, NULL, 1);
static SENSOR_DEVICE_ATTR(z_axis_g, S_IRUGO, mma7660_show_axis_g, NULL, 2);
static SENSOR_DEVICE_ATTR(all_axis_g, S_IRUGO, mma7660_show_xyz_g, NULL, 0);
static SENSOR_DEVICE_ATTR(tilt_status, S_IRUGO, mma7660_show_tilt, NULL, 0);

static struct attribute* mma7660_attrs[] = {
	&sensor_dev_attr_registers.dev_attr.attr,
	&sensor_dev_attr_x_axis_g.dev_attr.attr,
	&sensor_dev_attr_y_axis_g.dev_attr.attr,
	&sensor_dev_attr_z_axis_g.dev_attr.attr,
	&sensor_dev_attr_all_axis_g.dev_attr.attr,
	&sensor_dev_attr_tilt_status.dev_attr.attr,
	NULL
};

static const struct attribute_group mma7660_group = {
	.attrs		= mma7660_attrs,
};


/*-----------------------------------------------------------------------------
 * Input interfaces
 */
static void mma7660_report_abs(void)
{
	int axis[3];
	int i;

	for (i = 0; i < 3; i++) {
		mma7660_read_xyz(mma7660_client, i, &axis[i]);
	}

	input_report_abs(mma7660_idev->input, ABS_X, axis[0]);
	input_report_abs(mma7660_idev->input, ABS_Y, axis[1]);
	input_report_abs(mma7660_idev->input, ABS_Z, axis[2]);
	input_sync(mma7660_idev->input);

	//printk("3-Axis ... %3d, %3d, %3d\n", axis[0], axis[1], axis[2]);
}

static void mma7660_dev_poll(struct input_polled_dev *dev)
{
	mma7660_report_abs();
}


/*-----------------------------------------------------------------------------
 * Interrupt handler
 */

static void mma7660_worker(struct work_struct *work)
{
	int bafro, pola, shake, tap;
	int val = 0;

	mma7660_read_tilt(mma7660_client, &val);

	/* TODO: report it ? */

	bafro = val & 0x03;
	if (bafro != (last_tilt & 0x03)) {
		printk("%s\n", mma7660_bafro[bafro]);
	}

	pola = (val >> 2) & 0x07;
	if (pola != ((last_tilt >> 2) & 0x07)) {
		printk("%s\n", mma7660_pola[pola]);
	}

	shake = (val >> 5) & 0x01;
	if (shake && shake != ((last_tilt >> 5) & 0x01)) {
		printk("Shake\n");
	}

	tap = (val >> 7) & 0x01;
	if (tap && tap != ((last_tilt >> 7) & 0x01)) {
		printk("Tap\n");
	}

	/* Save current status */
	last_tilt = val;
}

DECLARE_WORK(mma7660_work, mma7660_worker);

static irqreturn_t mma7660_interrupt(int irq, void *dev_id)
{
	schedule_work(&mma7660_work);

	return IRQ_HANDLED;
}

#ifdef CONFIG_OF

static const struct of_device_id mma7660_dt_ids[] = {
	{ .compatible = "fsl,mma7660", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mma7660_dt_ids);
#endif

/*-----------------------------------------------------------------------------
 * I2C client driver interfaces
 */

static int mma7660_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct input_dev *idev;
	int poll_interval = POLL_INTERVAL;
	int input_fuzz = INPUT_FUZZ;
	int input_flat = INPUT_FLAT;
	int ret;
	struct of_device_id *match;
	struct device_node *np = NULL;

	ret = i2c_check_functionality(adapter,
			I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA);
	if (!ret) {
		dev_err(&client->dev, "I2C check functionality failed\n");
		return -ENXIO;
	}

	match = of_match_device(mma7660_dt_ids, &client->dev);
	
	if (NULL != match) {
		np = client->dev.of_node;
		plat_data = devm_kzalloc(&client->dev, sizeof(*plat_data), GFP_KERNEL);
		
		if (!plat_data) {
			dev_err(&client->dev, "failed to allocate platform data\n");
			goto error_init_client;
		}
		
		if (of_property_read_u32(np, "poll_interval", &plat_data->poll_interval)) {
			dev_err(&client->dev, "failed to get poll_interval property\n");
			return -EINVAL;
		}

		if (of_property_read_u32(np, "input_fuzz", &plat_data->poll_interval)) {
			dev_err(&client->dev, "failed to get input_fuzz property\n");
			return -EINVAL;
		}

		if (of_property_read_u32(np, "input_flat", &plat_data->poll_interval)) {
			dev_err(&client->dev, "failed to get input_flat property\n");
			return -EINVAL;
		}
		
	}
	else
		plat_data = (struct mma7660_platform_data *)client->dev.platform_data;	

	
	if (plat_data == NULL) {
		dev_err(&client->dev, "lack of platform data!\n");
		return -ENODEV;
	}

	/* Get parameters from platfrom data */
	if (plat_data->poll_interval > 0)
		poll_interval = plat_data->poll_interval;
	if (plat_data->input_fuzz > 0)
		input_fuzz = plat_data->input_fuzz;
	if (plat_data->input_flat > 0)
		input_flat = plat_data->input_flat;

	if (mma7660_initialize(client) < 0) {
		goto error_init_client;
	}

	ret = sysfs_create_group(&client->dev.kobj, &mma7660_group);
	if (ret) {
		dev_err(&client->dev, "create sysfs group failed!\n");
		goto error_init_client;
	}

	/* register to hwmon device */
	hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(hwmon_dev)) {
		dev_err(&client->dev, "hwmon register failed!\n");
		ret = PTR_ERR(hwmon_dev);
		goto error_rm_dev_file;
	}

	/* input poll device register */
	mma7660_idev = input_allocate_polled_device();
	if (!mma7660_idev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		ret = -ENOMEM;
		goto error_rm_hwmon_dev;
	}

	mma7660_idev->poll = mma7660_dev_poll;
	mma7660_idev->poll_interval = plat_data->poll_interval;

	idev = mma7660_idev->input;
	idev->name = MMA7660_NAME;
	idev->id.bustype = BUS_I2C;
	idev->id.vendor = 0x12FA;
	idev->id.product = 0x7660;
	idev->id.version = 0x0100;
	idev->dev.parent = &client->dev;

	set_bit(EV_ABS, idev->evbit);

	set_bit(ABS_X, idev->absbit);
	set_bit(ABS_Y, idev->absbit);
	set_bit(ABS_Z, idev->absbit);

	input_set_abs_params(idev, ABS_X, -512, 512, input_fuzz, input_flat);
	input_set_abs_params(idev, ABS_Y, -512, 512, input_fuzz, input_flat);
	input_set_abs_params(idev, ABS_Z, -512, 512, input_fuzz, input_flat);
	ret = input_register_polled_device(mma7660_idev);
	if (ret) {
		dev_err(&client->dev, "register poll device failed!\n");
		goto error_free_poll_dev;
	}

	/* register interrupt handle */
	ret = devm_request_irq(&client->dev, client->irq, mma7660_interrupt,
			IRQF_TRIGGER_FALLING, MMA7660_NAME, idev);
	
	if (ret) {
		dev_err(&client->dev, "request irq (%d) failed %d\n", client->irq, ret);
		goto error_rm_poll_dev;
	}

	dev_info(&client->dev, "MMA7660 device is probed successfully.\n");

#if	0
	set_mod(1);
#endif

	return 0;

error_rm_poll_dev:
	input_unregister_polled_device(mma7660_idev);
error_free_poll_dev:
	input_free_polled_device(mma7660_idev);
error_rm_hwmon_dev:
	hwmon_device_unregister(hwmon_dev);
error_rm_dev_file:
	sysfs_remove_group(&client->dev.kobj, &mma7660_group);
error_init_client:
	mma7660_client = NULL;

	return 0;
}

static int mma7660_remove(struct i2c_client *client)
{
	free_irq(client->irq, mma7660_idev->input);

	input_unregister_polled_device(mma7660_idev);
	input_free_polled_device(mma7660_idev);
	hwmon_device_unregister(hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &mma7660_group);
	mma7660_client = NULL;

	return 0;
}

static int mma7660_suspend(struct device *dev, pm_message_t state)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	oper_mode = i2c_smbus_read_byte_data(client, MMA7660_MODE);

	ret = i2c_smbus_write_byte_data(client, MMA7660_MODE, 0);
	if (ret) {
		printk("%s: set mode (0) for suspend failed, ret = %d\n",
				MMA7660_NAME, ret);
	}

	return 0;
}

static int mma7660_resume(struct device *dev)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);

	ret = i2c_smbus_write_byte_data(client, MMA7660_MODE, oper_mode);
	if (ret) {
		printk("%s: set mode (%d) for resume failed, ret = %d\n",
				MMA7660_NAME, oper_mode, ret);
	}

	return 0;
}

static const struct i2c_device_id mma7660_ids[] = {
	{ "mma7660", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mma7660_ids);

static struct i2c_driver i2c_mma7660_driver = {
	.driver		= {
		.name	= MMA7660_NAME,
		.suspend	= mma7660_suspend,
		.resume		= mma7660_resume,
#ifdef CONFIG_OF
		.of_match_table = mma7660_dt_ids,
#endif
	},

	.probe		= mma7660_probe,
	.remove		= mma7660_remove,
	.id_table	= mma7660_ids,
};

static int __init init_mma7660(void)
{
	int ret;

	ret = i2c_add_driver(&i2c_mma7660_driver);
	printk(KERN_INFO "MMA7660 sensor driver registered.\n");

	return ret;
}

static void __exit exit_mma7660(void)
{
	i2c_del_driver(&i2c_mma7660_driver);
	printk(KERN_INFO "MMA7660 sensor driver removed.\n");
}

module_init(init_mma7660);
module_exit(exit_mma7660);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MMA7660 sensor driver");
MODULE_LICENSE("GPL");

