/* drivers/android/msm_vibrator.c
 *
 * (C) 2009 LGE, Inc.
 *
 * msm vibrator driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <mach/gpio.h>
#include <mach/msm_iomap.h>
#include <mach/vreg.h>
#include <mach/mpp.h>

#include <../../../drivers/staging/android/timed_output.h>

#define GPIO_VIB_PWM		28

#define GPIO_LDO_EN			109

/* addresses of gp_mn register */
#define GP_MN_CLK_MDIV              0x004C
#define GP_MN_CLK_NDIV              0x0050
#define GP_MN_CLK_DUTY              0x0054

#define GPMN_M_DEFAULT              21
#define GPMN_N_DEFAULT              4500

#define GPMN_D_DEFAULT              3200  /* 2250 */
#define PWM_MULTIPLIER              2560  /* 4394 */

/* The maximum value of M is 511 and N is 8191 */
#define GPMN_M_MASK                 0x01FF
#define GPMN_N_MASK                 0x1FFF
#define GPMN_D_MASK                 0x1FFF

/* Qcoin motor state */
#define PWM_DUTY_MAX		1
#define PWM_DUTY_SET		2
#define PWM_DUTY_MIN		3
#define VIB_DISABLE			4

#define DUTY_MAX_TIME		20
#define DUTY_MIN_TIME		20

#define MSM_WEB_BASE          IOMEM(0xE100C000)
#define MSM_WEB_PHYS          0xA9D00040 //0xA9D00000 in code.
#define MSM_WEB_SIZE          SZ_4K
#if 0 
#define REG_WRITEL(value, reg)      writel(value, (MSM_WEB_BASE + reg))
#else 
#define REG_WRITEL(value, reg)  
#endif

struct vibrator_device {
	struct timed_output_dev timed_vibrator;
	struct hrtimer vib_timer;
	int state;
	int value;
	int pwm_max_time;
	int pwm_min_time;
	spinlock_t	lock;
};

static atomic_t s_vibrator = ATOMIC_INIT(0);
static atomic_t s_amp = ATOMIC_INIT(50);

/* set amp level, default amp 50
 * level 1: 100, 3.1V
 * level 2:  75, 2.8V
 * level 3:  50, 2.4V
 * level 4:  25, 2.0V
 * level 5:   0, 1.6V */

static uint32_t debug_mask;
module_param_named(debug, debug_mask, uint, 0664);

static ssize_t vibrator_amp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int amp = atomic_read(&s_amp);
    return sprintf(buf, "%d\n", amp);
}

static ssize_t vibrator_amp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int value;

    sscanf(buf, "%d", &value);
    atomic_set(&s_amp, value);

	if (debug_mask)
		printk(KERN_INFO "%s %d\n", __func__, value);

    return size;
}

static DEVICE_ATTR(amp, S_IRUGO | S_IWUSR, vibrator_amp_show, vibrator_amp_store);

int is_vib_state(void)
{
    int vib_state;
    vib_state = atomic_read(&s_vibrator);
    return vib_state;
}

void vibrator_disable(void)
{
	if (debug_mask)
		printk(KERN_INFO "%s\n", __func__);

	gpio_set_value(GPIO_VIB_PWM, 0);
	gpio_set_value(GPIO_LDO_EN, 0);
}

void vibrator_enable(void)
{
	int amp, gain;
	amp = atomic_read(&s_amp);

	gain = ((PWM_MULTIPLIER * amp) >> 8) + GPMN_D_DEFAULT;
	REG_WRITEL((gain & GPMN_D_MASK), GP_MN_CLK_DUTY);
	//
	//
	//
	if (debug_mask)
		printk(KERN_INFO "%s\n", __func__);

	gpio_set_value(GPIO_VIB_PWM, 1);
	gpio_set_value(GPIO_LDO_EN, 1);

}

static void vibrator_set(struct timed_output_dev *tdev, int value)
{
	struct vibrator_device *vib_dev;
	vib_dev = container_of(tdev, struct vibrator_device, timed_vibrator);

	if (vib_dev->state > 0) {
		if (debug_mask)
			printk(KERN_INFO "vibrator is working now\n");
	} else {
		vib_dev->value = (value > 15000 ? 15000 : value);

		/* set over_drive time */
		if (value <= DUTY_MAX_TIME) {
			vib_dev->pwm_max_time = value;
			vib_dev->value = 0;
			vib_dev->pwm_min_time = 0;
		} else if (value > DUTY_MAX_TIME && value <= DUTY_MAX_TIME + DUTY_MIN_TIME) {
			vib_dev->pwm_max_time = DUTY_MAX_TIME;
			vib_dev->value = 0;
			vib_dev->pwm_min_time = value - DUTY_MAX_TIME;
		} else if (value > DUTY_MAX_TIME + DUTY_MIN_TIME && value <= 100) {
			vib_dev->pwm_max_time = DUTY_MAX_TIME;
			vib_dev->value = value - DUTY_MAX_TIME - DUTY_MIN_TIME;
			vib_dev->pwm_min_time = DUTY_MIN_TIME;
		} else {
			vib_dev->pwm_max_time = DUTY_MAX_TIME;
			vib_dev->value = value - DUTY_MAX_TIME;
			vib_dev->pwm_min_time = 0;
		}

		vib_dev->state = PWM_DUTY_MAX;
		atomic_set(&s_vibrator, 1);
		vibrator_enable();
		hrtimer_start(&vib_dev->vib_timer, ktime_set(0, 0), HRTIMER_MODE_REL);
	}
}

static void timed_vibrator_enable(struct timed_output_dev *tdev, int value)
{
	struct vibrator_device *vib_dev;
	vib_dev = container_of(tdev, struct vibrator_device, timed_vibrator);

	if (debug_mask)
		printk(KERN_INFO "\n%s %d %d\n", __func__, vib_dev->state, value);

	spin_lock(&vib_dev->lock);

	if (value == 0) {
		hrtimer_cancel(&vib_dev->vib_timer);
		vibrator_disable();
		vib_dev->state = 0;
	} else
		vibrator_set(tdev, value);

	spin_unlock(&vib_dev->lock);
}

static void pwm_gen(struct vibrator_device *vib_dev)
{
	int amp, gain;
	amp = atomic_read(&s_amp);

	switch (vib_dev->state) {
	default:
		printk(KERN_INFO "pwm_timer error\n");
		vibrator_disable();
		vib_dev->state = 0;
		atomic_set(&s_vibrator, 0);
		break;
	case PWM_DUTY_MAX:
		gain = 4200;  /* 3.1V */
		REG_WRITEL((gain & GPMN_D_MASK), GP_MN_CLK_DUTY);

		if (debug_mask)
			printk(KERN_INFO "PWM max duty:%d\n", gain);
		
		hrtimer_start(&vib_dev->vib_timer,
			      ktime_set(vib_dev->pwm_max_time  / 1000,
				  (vib_dev->pwm_max_time % 1000) * 1000000),
			      HRTIMER_MODE_REL);
		vib_dev->state = PWM_DUTY_SET;
		break;
	case PWM_DUTY_SET:
		gain = ((PWM_MULTIPLIER * amp) >> 8) + GPMN_D_DEFAULT;
		REG_WRITEL((gain & GPMN_D_MASK), GP_MN_CLK_DUTY);

		if (debug_mask)
			printk(KERN_INFO "PWM set duty:%d\n", gain);

		hrtimer_start(&vib_dev->vib_timer,
			      ktime_set(vib_dev->value / 1000,
				  (vib_dev->value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
		vib_dev->state = PWM_DUTY_MIN;
		break;
	case PWM_DUTY_MIN:
		gain = 0;  /* -3.1V */
		REG_WRITEL((gain & GPMN_D_MASK), GP_MN_CLK_DUTY);

		if (debug_mask)
			printk(KERN_INFO "PWM min duty:%d\n", gain);

		hrtimer_start(&vib_dev->vib_timer,
			      ktime_set(vib_dev->pwm_min_time / 1000,
				  (vib_dev->pwm_min_time % 1000) * 1000000),
			      HRTIMER_MODE_REL);
		vib_dev->state = VIB_DISABLE;
		break;
	case VIB_DISABLE:
		vibrator_disable();
		vib_dev->state = 0;
		atomic_set(&s_vibrator, 0);
		break;
	}
}

static enum hrtimer_restart timed_vibrator_timer_func(struct hrtimer *vib_timer)
{
	struct vibrator_device *vib_dev;
	vib_dev = container_of(vib_timer, struct vibrator_device, vib_timer);

	pwm_gen(vib_dev);
	return HRTIMER_NORESTART;
}

static int timed_vibrator_get_time(struct timed_output_dev *tdev)
{
	struct vibrator_device *vib_dev;
	vib_dev = container_of(tdev, struct vibrator_device, timed_vibrator);

	if (hrtimer_active(&vib_dev->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib_dev->vib_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

void __init swift_init_timed_vibrator(void)
{
	int ret;
	struct vibrator_device *vib_dev;

	/* Set PWM frequency */
	//Prevent boot time hung
	//REG_WRITEL((GPMN_M_DEFAULT & GPMN_M_MASK), GP_MN_CLK_MDIV);
	//REG_WRITEL((~(GPMN_N_DEFAULT - GPMN_M_DEFAULT) & GPMN_N_MASK), GP_MN_CLK_NDIV);

	/* Off Enable */
	gpio_set_value(GPIO_VIB_PWM, 0);
	gpio_set_value(GPIO_LDO_EN, 0);

	vib_dev = kmalloc(sizeof(*vib_dev), GFP_KERNEL);
	if (vib_dev == NULL)
	{
		printk(KERN_ERR "[Swift-vibrator]Failed to allocate private data!\n");
		return;
	}
	vib_dev->timed_vibrator.name = "vibrator";
	vib_dev->timed_vibrator.get_time = timed_vibrator_get_time;
	vib_dev->timed_vibrator.enable = timed_vibrator_enable;
	vib_dev->state = 0;

	hrtimer_init(&vib_dev->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib_dev->vib_timer.function = timed_vibrator_timer_func;

	ret = timed_output_dev_register(&vib_dev->timed_vibrator);
	if (ret < 0) {
		printk(KERN_ERR "timed_output_dev_register error!\n");
		return;
	}

	ret = device_create_file(vib_dev->timed_vibrator.dev, &dev_attr_amp);
	if (ret < 0) {
		printk(KERN_ERR "device_create_file error!\n");
		timed_output_dev_unregister(&vib_dev->timed_vibrator);
		return;
	}

	gpio_request(GPIO_LDO_EN, "gpio_qcoin_motor_en");
	gpio_request(GPIO_VIB_PWM, "gpio_qcoin_motor_pwm");


	spin_lock_init(&vib_dev->lock);

	printk(KERN_INFO "timed_vibrator probed\n");
}

MODULE_AUTHOR("Taehyun Kim <kimth@lge.com>");
MODULE_DESCRIPTION("Swift vibrator driver for Android");
MODULE_LICENSE("GPL");
