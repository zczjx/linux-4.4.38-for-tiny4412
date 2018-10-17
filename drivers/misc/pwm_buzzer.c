/*******************************************************************************
* Copyright (C), 2000-2018,  Electronic Technology Co., Ltd.
*                
* @filename: pwm_buzzer.c 
*                
* @author: Clarence.Zhou <zhou_chenz@163.com> 
*                
* @version:
*                
* @date: 2018-10-16    
*                
* @brief:          
*                  
*                  
* @details:        
*                 
*    
*    
* @comment           
*******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/of_device.h>

#define DEVICE_NAME				"pwm-buzzer"

#define PWM_IOCTL_SET_FREQ		1
#define PWM_IOCTL_STOP			0

#define NS_IN_1HZ				(1000000000UL)


static struct pwm_device *pwm4buzzer = NULL;

static struct semaphore lock;


static void pwm_set_freq(unsigned long freq) {
	int period_ns = NS_IN_1HZ / freq;

	pwm_config(pwm4buzzer, period_ns / 2, period_ns);
	pwm_enable(pwm4buzzer);
}

static int pwm_buzzer_open(struct inode *inode, struct file *file) {
	if (!down_trylock(&lock))
		return 0;
	else
		return -EBUSY;
}

static int pwm_buzzer_close(struct inode *inode, struct file *file) {
	up(&lock);
	return 0;
}

static long pwm_buzzer_ioctl(struct file *filep, unsigned int cmd,
		unsigned long arg)
{
	switch (cmd) {
		case PWM_IOCTL_SET_FREQ:
			if (arg == 0)
				return -EINVAL;
			pwm_set_freq(arg);
			break;

		case PWM_IOCTL_STOP:
		default:
			pwm_config(pwm4buzzer, 0, NS_IN_1HZ / 100);
			pwm_disable(pwm4buzzer);
			break;
	}

	return 0;
}


static struct file_operations pwm_buzzer_ops = {
	.owner			= THIS_MODULE,
	.open			= pwm_buzzer_open,
	.release		= pwm_buzzer_close, 
	.unlocked_ioctl	= pwm_buzzer_ioctl,
};

static struct miscdevice pwm_buzzer_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &pwm_buzzer_ops,
};

static const struct of_device_id pwm_buzzer_match_table[] = {
	{ .compatible = "pwm-buzzer", },
	{}
};

static int pwm_buzzer_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct of_device_id *match = NULL;
	struct device_node *np = NULL;
	struct device *dev = NULL;

	match = of_match_device(pwm_buzzer_match_table, &pdev->dev);

	if (NULL == match) {
		ret = -ENODEV;
		goto out;
	}

	dev = &pdev->dev;
	np = dev->of_node;
	pwm4buzzer = devm_pwm_get(dev, "buzzer-pwm");

	if (IS_ERR(pwm4buzzer)) {
		printk("request pwm %s failed\n", DEVICE_NAME);
		return -ENODEV;
	}

	pwm_config(pwm4buzzer, 0, NS_IN_1HZ / 100);
	pwm_disable(pwm4buzzer);
	sema_init(&lock, 1);
	ret = misc_register(&pwm_buzzer_dev);

	printk(DEVICE_NAME " probe OK\n");
	
out:
	return ret;
	
}

static int pwm_buzzer_remove(struct platform_device *pdev)
{
	pwm_config(pwm4buzzer, 0, NS_IN_1HZ / 100);
	pwm_disable(pwm4buzzer);

	misc_deregister(&pwm_buzzer_dev);

	return 0;
}

MODULE_DEVICE_TABLE(of, pwm_buzzer_match_table);

static struct platform_driver pwm_buzzer_driver = {
	.driver	= {
		.name		= DEVICE_NAME,
		.of_match_table	= pwm_buzzer_match_table,
	},
	.probe		= pwm_buzzer_probe,
	.remove		= pwm_buzzer_remove,
};

module_platform_driver(pwm_buzzer_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Clarence.Zhou<zhou_chenz@163.com>");
MODULE_DESCRIPTION("Tiny4412 PWM Buzzer Driver");

