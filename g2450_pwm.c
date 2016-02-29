/*
*	G2450_PWM.C - The s3c2450 PWM module.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#include <asm/delay.h>
#include <asm/uaccess.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <plat/devs.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <plat/gpio-cfg.h>
#include <plat/s3c2416.h> 
#include <mach/regs-gpio.h>

#include "g2450_pwm.h"


struct pwm_device {
	struct list_head     list;
	struct platform_device  *pdev;

	struct clk      *clk_div;
	struct clk      *clk;
	const char      *label;

	unsigned int         period_ns;
	unsigned int         duty_ns;

	unsigned char        tcon_base;
	unsigned char        running;
	unsigned char        use_count;
	unsigned char        pwm_id;
};

static struct pwm_device *bz_pwm;

static long g2450_pwm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct g2450_pwm_duty_t pwm_duty;

	switch(cmd) {
		case DEV_PWM_RUN: 
			pwm_enable(bz_pwm);
			break;

		case DEV_PWM_STOP: 
			pwm_disable(bz_pwm);
			break;

		case DEV_PWM_DUTYRATE:
			copy_from_user((void *)&pwm_duty, (const void *)arg, sizeof( struct g2450_pwm_duty_t )); 
			pwm_config(bz_pwm, pwm_duty.pulse_width, pwm_duty.period);
			break;

		default:
			return 0;
	}
	return 0;
}

static int g2450_pwm_open(struct inode *inode, struct file *filp)
{
	bz_pwm = pwm_request(2, "bz_pwm");
	if( NULL == bz_pwm )
	{
		printk("Fail!!\n");
		return -1;
	}

    return 0;
}

static int g2450_pwm_release(struct inode *inode, struct file *filp)
{
	pwm_free( bz_pwm );
	printk("pwm_release!\n");
    return 0;
}

struct file_operations g2450_pwm_fops = { 
    .open			= g2450_pwm_open,
    .release  		= g2450_pwm_release,
	.unlocked_ioctl	= g2450_pwm_ioctl,
};


static int __init g2450_pwm_init(void)
{

	s3c_gpio_cfgpin(S3C2410_GPB(2), S3C_GPIO_SFN(2));
	s3c_gpio_setpull(S3C2410_GPB(2), S3C_GPIO_PULL_UP);

	printk("Insert pwm module!\n");

	register_chrdev( DEV_PWM_MAJOR, DEV_PWM_NAME, &g2450_pwm_fops );

	return 0;
}

static void __exit g2450_pwm_exit(void)
{
	printk("pwm_exit! (unregister chardev) \n");


	unregister_chrdev( DEV_PWM_MAJOR, DEV_PWM_NAME );
}

module_init(g2450_pwm_init);
module_exit(g2450_pwm_exit);
MODULE_LICENSE("GPL");
