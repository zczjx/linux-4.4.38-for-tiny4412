/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/atomic.h>
#include <asm/uaccess.h>

#define HC_DEV_NAME "hc-sr04"
#define IOC_GET_US_TIME_GAP (0x01)

typedef struct ultrasonic_dev {
	int	irq_no;
	int trigger_gpio;
	int echo_gpio;
	struct timer_list tigger_cycle_timer;
	unsigned long  tigger_cycle;
	unsigned long time_gap_us;
	wait_queue_head_t wq;
	atomic_t data_update_flag;
} ultrasonic_dev;

static struct ultrasonic_dev hc_sr04_dev;

static void tigger_cycle_func(unsigned long data)
{
	gpio_set_value(hc_sr04_dev.trigger_gpio, 1);
	udelay(40);
	gpio_set_value(hc_sr04_dev.trigger_gpio, 0);
	mod_timer(&hc_sr04_dev.tigger_cycle_timer, 
			jiffies+hc_sr04_dev.tigger_cycle);

}


static int hc_sr04_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	gpio_set_value(hc_sr04_dev.trigger_gpio, 0);
	gpio_set_value(hc_sr04_dev.echo_gpio, 0);
	init_timer(&hc_sr04_dev.tigger_cycle_timer);
	hc_sr04_dev.tigger_cycle_timer.function = tigger_cycle_func;
	hc_sr04_dev.tigger_cycle = HZ >> 2;
	hc_sr04_dev.time_gap_us = 0;
	mod_timer(&hc_sr04_dev.tigger_cycle_timer, 
			jiffies+hc_sr04_dev.tigger_cycle);

	return ret;
}

static int hc_sr04_release(struct inode *inode, struct file *filp)
{
	int ret = 0;

	atomic_set(&hc_sr04_dev.data_update_flag, 1);
	wake_up_all(&hc_sr04_dev.wq);
	gpio_set_value(hc_sr04_dev.trigger_gpio, 0);
	gpio_set_value(hc_sr04_dev.echo_gpio, 0);
	del_timer_sync(&hc_sr04_dev.tigger_cycle_timer);
	
	return ret;
}

static long hc_sr04_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	long ret = -ENOIOCTLCMD;
	
	switch(cmd)
	{
		case IOC_GET_US_TIME_GAP:
			atomic_set(&hc_sr04_dev.data_update_flag, 0);
			ret = wait_event_interruptible(hc_sr04_dev.wq,
						atomic_read(&hc_sr04_dev.data_update_flag));
		
			if (ret < 0) {
				ret = -EHOSTUNREACH;
				goto end;
			}

			copy_to_user((void __user *) arg, &hc_sr04_dev.time_gap_us, 
						sizeof(hc_sr04_dev.time_gap_us));
			break;
			
		default :
			ret = -ENOIOCTLCMD;
			break;

	}

end:
	return ret;
}

static irqreturn_t hc_sr04_isr(int irq, void *dev_id)
{
	int val;
	unsigned long pre_us, cur_us;
	struct timeval tv;

	do_gettimeofday(&tv);
	pre_us = tv.tv_sec * 1000 + tv.tv_usec;
	val = gpio_get_value(hc_sr04_dev.echo_gpio);
	
	while (1 == val) {
		val = gpio_get_value(hc_sr04_dev.echo_gpio);
	}

	do_gettimeofday(&tv);
	cur_us = tv.tv_sec * 1000 + tv.tv_usec;
	hc_sr04_dev.time_gap_us = cur_us - pre_us;
	atomic_set(&hc_sr04_dev.data_update_flag, 1);
	wake_up_all(&hc_sr04_dev.wq);
	return IRQ_HANDLED;
}

static int setup_gpios(struct device_node *np, struct device *dev)
{
	enum of_gpio_flags gpio_flags;
	int error = 0;

	hc_sr04_dev.trigger_gpio = of_get_named_gpio_flags(np, "trigger-gpio", 0,
					      							&gpio_flags);
	if (-EPROBE_DEFER == hc_sr04_dev.trigger_gpio) {
		dev_err(dev, "failed to get trigger-gpio\n");
		return -EPROBE_DEFER;
	}
	
	if (gpio_is_valid(hc_sr04_dev.trigger_gpio)) {
		error = devm_gpio_request_one(dev, hc_sr04_dev.trigger_gpio,
									GPIOF_OUT_INIT_LOW, "trigger-gpio");
		
		if (error) {
			dev_err(dev, "unable to request GPIO %d as reset pin (%d)\n",
				hc_sr04_dev.trigger_gpio, error);
			return error;
		}
	}
	
	gpio_direction_output(hc_sr04_dev.trigger_gpio, 0);

	hc_sr04_dev.echo_gpio = of_get_named_gpio_flags(np, "echo-gpio", 0,
					      							&gpio_flags);
	if (-EPROBE_DEFER == hc_sr04_dev.echo_gpio) {
		dev_err(dev, "failed to get echo-gpio\n");
		return -EPROBE_DEFER;
	}
	
	if (gpio_is_valid(hc_sr04_dev.echo_gpio)) {
		error = devm_gpio_request_one(dev, hc_sr04_dev.echo_gpio,
									gpio_flags, "echo-gpio");
		
		if (error) {
			dev_err(dev, "unable to request GPIO %d as reset pin (%d)\n",
				hc_sr04_dev.echo_gpio, error);
			return error;
		}
	}
	
	gpio_direction_input(hc_sr04_dev.echo_gpio);
	gpio_set_value(hc_sr04_dev.trigger_gpio, 0);
	gpio_set_value(hc_sr04_dev.echo_gpio, 0);
	udelay(5);

	return 0;

}

static const struct file_operations hc_sr04_ops = {
	.owner		= THIS_MODULE,
	.open		= hc_sr04_open,
	.release	= hc_sr04_release,
	.unlocked_ioctl = hc_sr04_ioctl,
};

static struct miscdevice hc_sr04_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= HC_DEV_NAME,
	.fops		= &hc_sr04_ops
};


static const struct of_device_id hc_sr04_match_table[] = {
	{ .compatible = "hc-sr04", },
	{ .compatible = "hc-sr04p", },
	{}
};

static int hc_sr04_probe(struct platform_device *pdev)
{
	int ret = -EINVAL;
	struct of_device_id *match = NULL;
	unsigned long irq_flags;
	int error;
	struct device_node *np = NULL;
	struct device *dev = NULL;

	match = of_match_device(hc_sr04_match_table, &pdev->dev);

	if (NULL == match) {
		ret = -ENODEV;
		goto out;
	}

	dev = &pdev->dev;
	np = dev->of_node;
	error = setup_gpios(np, dev);

	if (error) {
		dev_err(dev, "Unable to setup %s gpio\n", HC_DEV_NAME);
		ret = error;
		goto out;
	}
	
	init_waitqueue_head(&hc_sr04_dev.wq);
	atomic_set(&hc_sr04_dev.data_update_flag, 0);
	
	hc_sr04_dev.irq_no = platform_get_irq_byname(pdev, "echo");

	if (hc_sr04_dev.irq_no < 0) {
		dev_err(dev, "failed to acquire irq resource\n");
		ret = -ENOENT;
		goto out;
	}

	irq_flags = irq_get_trigger_type(hc_sr04_dev.irq_no);
	if (irq_flags == IRQF_TRIGGER_NONE)
		irq_flags = IRQF_TRIGGER_RISING;
	irq_flags |= IRQF_ONESHOT;

	error = devm_request_threaded_irq(dev, hc_sr04_dev.irq_no,
					NULL, hc_sr04_isr, irq_flags,
					HC_DEV_NAME, NULL);
	if (error) {
		dev_err(dev, "Unable to request %s echo IRQ.\n", HC_DEV_NAME);
		ret = -ENOENT;
		goto out;
	}
	
	ret = misc_register(&hc_sr04_miscdev);
	
	if (ret) {
		dev_err(dev, "%s: misc_register failed\n",
		       HC_DEV_NAME);
		goto out;
	}

out:
	return ret;
}

static int hc_sr04_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	devm_free_irq(dev, hc_sr04_dev.irq_no, NULL);
	misc_deregister(&hc_sr04_miscdev);

	return 0;
}

MODULE_DEVICE_TABLE(of, hc_sr04_match_table);

static struct platform_driver hc_sr04_driver = {
	.driver	= {
		.name		= "hc-sr04",
		.of_match_table	= hc_sr04_match_table,
	},
	.probe		= hc_sr04_probe,
	.remove		= hc_sr04_remove,
};

module_platform_driver(hc_sr04_driver);

MODULE_AUTHOR("Clarence.Zhou<zhou_chenz@163.com>");
MODULE_DESCRIPTION("hc_sr04 Ultrasonic Distance Measurement Device");
MODULE_LICENSE("GPL v2");

