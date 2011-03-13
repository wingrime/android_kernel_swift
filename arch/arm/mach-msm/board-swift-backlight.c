/* linux/arch/arm/mach-msm/swift/board-swift-bl-rt9393.c
 *
 * Copyright (C) 2009-2011 LGE Inc. , wingrime 2011 (c)
 * Author : MoonCheol Kang <knight0708@lge.com>
 *
 * This software is licensed under the term of the GNU General Public
 * License version 2, as published by the Free Sofrware Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANABILITY or FITNESS FOR A PARTICLULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
//#include <linux/fs.h>

#if defined(CONFIG_RT9393_MSG_DEBUG)
enum {
	BL_DEBUG_DATA = 1U << 6,
	BL_DEBUG_FUNC = 1U << 7, /* function debug*/
};

static unsigned int lge_bl_debug_mask;

module_param_named(debug_mask, lge_bl_debug_mask, int,
		S_IRUGO | S_IWUSR | S_IWGRP);

#define KNIGHT_DBG(mask, fmt, args...) \
	do {\
		if ((mask) & lge_bl_debug_mask) \
				printk(KERN_INFO "[MC] [%-18s:%5d] " \
					fmt, __func__, __LINE__, ## args);\
		} while (0)
#else
#define KNIGHT_DBG(mask, fmt, args...)
#endif

#define GPIO_BL_EN	84
#define MAX_BRIGHTNESS 32
#define VMAX_BRIGHTNESS 16
#define MIN_BRIGHTNESS 0

wait_queue_head_t backlight_wait_q;
atomic_t backlight_event_handle;
atomic_t lcd_event_handled;

enum power_status{
	POWER_DOWN = 0,
	POWER_ON
} power_state;

static struct backlight_device *bd;
static unsigned short current_intensity = 0;
//static unsigned short power_state = 0;
static DEFINE_SPINLOCK(rt9393_lock);
struct timer_list timerblbl;

//void swift_turn_off_led(void);

static int rt9393_power_down(void)
{
	gpio_set_value(GPIO_BL_EN, 0);
	mdelay(4); // 3ms < tSHDN
	power_state = POWER_DOWN;
	current_intensity = 0;
	KNIGHT_DBG(lge_bl_debug_mask, "\n");
	atomic_set(&backlight_event_handle, 0);

	return 0;
}

static int rt9393_power_on(void)
{
	gpio_set_value(GPIO_BL_EN, 1);
	udelay(40); // 30us < tIH.INIT
	power_state = POWER_ON;
	current_intensity = MAX_BRIGHTNESS;
	KNIGHT_DBG(lge_bl_debug_mask, "\n");

	return 0;
}

static void bl_timer(unsigned long arg)
{
	if(atomic_read(&backlight_event_handle) == 0) {
		atomic_set(&lcd_event_handled, 1);
		wake_up(&backlight_wait_q);
		mod_timer(&timerblbl, jiffies + msecs_to_jiffies(20));
	}
}

static int rt9393_send_intensity(struct backlight_device *bd)
{
	int i, circular;
	int next_intensity = bd->props.brightness;

	KNIGHT_DBG(lge_bl_debug_mask, "set value = %d, current value = %d \n",
			next_intensity, current_intensity);


	if ((atomic_read(&backlight_event_handle) == 0) && (next_intensity != 0)) {
		mod_timer(&timerblbl, jiffies + msecs_to_jiffies(600));
		wait_event_interruptible_timeout(backlight_wait_q,
				atomic_read(&lcd_event_handled), HZ/2);
		atomic_set(&backlight_event_handle, 1);
		msleep(50);
	}

	spin_lock(&rt9393_lock);
	if (next_intensity == current_intensity) {
		KNIGHT_DBG(lge_bl_debug_mask, "next == current, %d \n", next_intensity);
		spin_unlock(&rt9393_lock);
		return 0;
	}

	if (next_intensity > VMAX_BRIGHTNESS) {
		next_intensity = VMAX_BRIGHTNESS;
		KNIGHT_DBG(lge_bl_debug_mask, "the value is over virtual maximum(%d)\n",
				VMAX_BRIGHTNESS);
	}
	else if (next_intensity < MIN_BRIGHTNESS) {
		next_intensity = MIN_BRIGHTNESS;
		KNIGHT_DBG(lge_bl_debug_mask, "the value is under minimum(%d),\
				we will turn off the BL\n", MIN_BRIGHTNESS);
	}

	// calculate the circular...
	if (next_intensity < current_intensity) {
		circular = current_intensity - next_intensity;
		KNIGHT_DBG(lge_bl_debug_mask, "circular = %d\n", circular);
	} else {
		circular = 32 - (next_intensity - current_intensity);
		KNIGHT_DBG(lge_bl_debug_mask, "circular = %d\n", circular);
	}
	
	// real control RT9393
	if (next_intensity == 0) { // shutdown
		rt9393_power_down();
	} else {
		if (power_state == POWER_DOWN) {
			rt9393_power_on();
			circular = 32 - next_intensity;
		}
		for (i=0; i < circular; i++) {
			gpio_set_value(GPIO_BL_EN, 0);
			udelay(2); // 0.5 us < tIL < 500us
			gpio_set_value(GPIO_BL_EN, 1);
			udelay(2); // 0.5us < tIH 
		}
	}
	current_intensity = next_intensity;
	spin_unlock(&rt9393_lock);

	//if (next_intensity == 0) 
	 // swift_turn_off_led();

	return 0;
}

static int rt9393_get_intensity(struct backlight_device *bd)
{
	return current_intensity;
}

static int rt9393_set_intensity(struct backlight_device *bd)
{
	rt9393_send_intensity(bd);
	return 0;
}

static struct backlight_ops rt9393_ops = {
	.get_brightness = rt9393_get_intensity,
	.update_status = rt9393_set_intensity,
};

#if defined(CONFIG_RT9393_SYS_DEBUG)
ssize_t brightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", current_intensity);
}

ssize_t brightness_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value=0;

	sscanf(buf, "%d\n", &value);
	KNIGHT_DBG(lge_bl_debug_mask, "set value from sysfile system %d\n", value);
	bd->props.brightness = value;
	rt9393_send_intensity(bd);

	return count;
}

ssize_t circular_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	gpio_set_value(GPIO_BL_EN, 0);
	udelay(2); // 0.5 us < tIL < 500us
	gpio_set_value(GPIO_BL_EN, 1);
	udelay(2); // 0.5us < tIH 
	current_intensity = current_intensity - 1;
	if (current_intensity == 0)
		current_intensity = VMAX_BRIGHTNESS;

	return count;
}

ssize_t gpio_no_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", GPIO_BL_EN);
}

DEVICE_ATTR(bri, 0666, brightness_show, brightness_store);
DEVICE_ATTR(cir, 0664, NULL, circular_store);
DEVICE_ATTR(gpio, 0664, gpio_no_show, NULL);

#endif // CONFIG_RT9393_SYS_DEBUG

static int rt9393_probe(struct platform_device *pdev)
{
  printk("PROBING BACKLIGHT\n");
	int err;
	struct backlight_properties props;
	err = gpio_request(GPIO_BL_EN, 0);

	if (err < 0 ) {
		printk("Cannot get the gpio pin : %d\n", GPIO_BL_EN);
		return err;
	}	
	
	memset(&props, 0, sizeof(struct backlight_properties));
	
	bd = backlight_device_register("rt9393",&pdev->dev, NULL , &rt9393_ops,&props);
	if (IS_ERR(bd)) {
		printk("failed to register backlight device\n");
		return PTR_ERR(bd);
	}

	if (system_state == SYSTEM_BOOTING) {
		current_intensity = 16;
		power_state = POWER_ON;
	} else {
		rt9393_power_down();
	}

	init_waitqueue_head(&backlight_wait_q);
	atomic_set(&backlight_event_handle, 1);
	atomic_set(&lcd_event_handled, 0);

	bd->props.max_brightness = VMAX_BRIGHTNESS;
	bd->props.power = 0;//FB_BLANK_UNBLANK;
	//power_state = POWER_DOWN; 

	setup_timer(&timerblbl, bl_timer, (unsigned long)pdev);
#if 1
	bd->props.brightness = MAX_BRIGHTNESS/2;
	rt9393_set_intensity(bd);
#endif

#if defined(CONFIG_RT9393_SYS_DEBUG)
	err = device_create_file(&bd->dev, &dev_attr_bri);
	err = device_create_file(&bd->dev, &dev_attr_cir);
	err = device_create_file(&bd->dev, &dev_attr_gpio);
#endif

	return 0;
}

static int rt9393_remove(struct platform_device *pdev)
{
	KNIGHT_DBG(lge_bl_debug_mask, "\n");
	backlight_device_unregister(bd);
	return 0;
}

static int rt9393_suspend(struct platform_device *pdev, pm_message_t state)
{
	KNIGHT_DBG(lge_bl_debug_mask, "\n");
	return 0;
}

static int rt9393_resume(struct platform_device *pdev)
{
	KNIGHT_DBG(lge_bl_debug_mask, "\n");
	return 0;
}

static struct platform_driver this_driver = {
	.probe 		= rt9393_probe,
	.remove		= rt9393_remove,
	.suspend	= rt9393_suspend,
	.resume		= rt9393_resume,
	.driver		= {
		.name	= "swift_backlight",
	},
};

static int __init rt9393_init(void)
{
	int ret;

	ret = platform_driver_register(&this_driver);
	if (ret) {
		printk("platform driver cannot register\n");
		return ret;
	}
	return 0;
}

static void __exit rt9393_exit(void)
{
	platform_driver_unregister(&this_driver);
}

module_init(rt9393_init);
module_exit(rt9393_exit);
MODULE_DESCRIPTION("RT9393 driver for LCD Backlight");
MODULE_LICENSE("GPL");

