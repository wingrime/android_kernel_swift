/*
 * kernel/arch/arm/mach-msm/swift/board-swift-leds.c - MSM PMIC LEDs driver.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>

#include <mach/pmic.h>

/*****************************
keypad LED
******************************/
#define MAX_BACKLIGHT_LEVEL	16

void swift_turn_off_led(void)
{
	int ret;

	ret = pmic_set_led_intensity(LED_KEYPAD, 0);
	if (ret)
		printk(KERN_INFO "Can't turn of led\n");	
}

static void swift_keypad_bl_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	int ret;

	ret = pmic_set_led_intensity(LED_KEYPAD, value / MAX_BACKLIGHT_LEVEL);
	if (ret)
		dev_err(led_cdev->dev, "can't set keypad backlight\n");
}

static struct led_classdev swift_kp_bl_led = {
	.name			= "button-backlight",
	.brightness_set		= swift_keypad_bl_led_set,
	.brightness		= LED_OFF,
};



static int swift_led_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &swift_kp_bl_led);
	if (rc) {
		dev_err(&pdev->dev, "unable to register led class driver\n");
		return rc;
	}
	swift_keypad_bl_led_set(&swift_kp_bl_led, LED_OFF);
	return rc;
}

static int __devexit swift_led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&swift_kp_bl_led);

	return 0;
}

#ifdef CONFIG_PM
static int swift_led_suspend(struct platform_device *dev,
		pm_message_t state)
{
	led_classdev_suspend(&swift_kp_bl_led);

	return 0;
}

static int swift_led_resume(struct platform_device *dev)
{
	led_classdev_resume(&swift_kp_bl_led);

	return 0;
}
#else
#define swift_led_suspend NULL
#define swift_led_resume NULL
#endif

static struct platform_driver swift_led_driver = {
	.probe		= swift_led_probe,
	.remove		= __devexit_p(swift_led_remove),
	.suspend	= swift_led_suspend,
	.resume		= swift_led_resume,
	.driver		= {
		.name	= "swift-keyled",
		.owner	= THIS_MODULE,
	},
};

static int __init swift_led_init(void)
{
	return platform_driver_register(&swift_led_driver);
}
module_init(swift_led_init);

static void __exit swift_led_exit(void)
{
	platform_driver_unregister(&swift_led_driver);
}
module_exit(swift_led_exit);




MODULE_DESCRIPTION("Swift LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pmic-leds");
