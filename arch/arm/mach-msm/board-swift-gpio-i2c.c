/* linux/arch/arm/mach-msm/board-swift-gpio-i2c.c
 *
 * Copyright (C) 2009 LGE, Inc.
 * Author: SungEun Kim <cleaneye@lge.com>
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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/delay.h>
#define GPIOF_DRIVE_OUTPUT      0x00040000
#define LGE_GPIO_I2C_DBG
#define LGE_GPIO_I2C_ERR
//#undef  LGE_GPIO_I2C_DBG
//#undef  LGE_GPIO_I2C_ERR

#if defined(LGE_GPIO_I2C_DBG)
#define GI2C_D(fmt, args...)  printk(KERN_INFO "swift i2c[%-18s:%5d]" fmt, __func__, __LINE__, ## args)
#else
#define GI2C_D(fmt, args...)  do {} while(0)
#endif

#if defined(LGE_GPIO_I2C_ERR)
#define GI2C_E(fmt, args...)  printk(KERN_INFO "swift i2c [%-18s:%5d]" fmt, __func__, __LINE__, ## args)
#else
#define GI2C_E(fmt, args...)  do {} while(0)
#endif

/* GPIO I2C BUS Number Lists */
#define I2C_BUS_NUM_ECOMPASS 		2 
#define I2C_BUS_NUM_MOTION			3 
#define I2C_BUS_NUM_AMP				5

typedef void (gpio_i2c_init_func_t)(void);



/* E-COMPASS : AKM8973 */
#define GPIO_ECOMPASS_INT			   (18)	
#define GPIO_ECOMPASS_I2C_SDA 		(31)
#define GPIO_ECOMPASS_I2C_SCL 		(30)
#define ECOMPASS_I2C_ADDR           (0x1C)

static struct i2c_gpio_platform_data ecompass_i2c_gpio_pdata = {
	.sda_pin = GPIO_ECOMPASS_I2C_SDA,
	.scl_pin = GPIO_ECOMPASS_I2C_SCL,
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device ecompass_pdevice = {
	.name = "i2c-gpio", 
	.id = I2C_BUS_NUM_ECOMPASS,
	.dev.platform_data = &ecompass_i2c_gpio_pdata,
};

static struct i2c_board_info ecompass_i2c_bdinfo[] = {
	{
		I2C_BOARD_INFO("akm8973-swift", ECOMPASS_I2C_ADDR),
		.irq = MSM_GPIO_TO_INT(GPIO_ECOMPASS_INT),
	},
};

void __init init_i2c_gpio_compass(void)
{
	int rc = 0;
	/*
	gpio_configure(GPIO_ECOMPASS_I2C_SDA, GPIOF_DRIVE_OUTPUT);
	gpio_configure(GPIO_ECOMPASS_I2C_SCL, GPIOF_DRIVE_OUTPUT);
	*/
	GPIO_CFG(GPIO_ECOMPASS_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), 
	GPIO_CFG(GPIO_ECOMPASS_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), 
	rc = gpio_request(GPIO_ECOMPASS_INT, "compass_int");
   if (rc < 0) {
      GI2C_E("failed to gpio %d request(ret=%d)\n", GPIO_ECOMPASS_INT, rc);
      return;
   }
	gpio_direction_input(GPIO_ECOMPASS_INT);

	rc = i2c_register_board_info(I2C_BUS_NUM_ECOMPASS, ecompass_i2c_bdinfo, 1);
   if (rc < 0) {
      GI2C_E("failed to i2c register board info(busnum=%d, ret=%d)\n", I2C_BUS_NUM_ECOMPASS, rc);
      return;
   }

	rc = platform_device_register(&ecompass_pdevice);
   if (rc != 0) {
      GI2C_E("failed to register motion platform device(ret=%d)\n", rc);
   }
}

/* MOTION : BMA150  */
#define GPIO_MOTION_INT             (33)	
#define GPIO_MOTION_I2C_SDA			(41)
#define GPIO_MOTION_I2C_SCL			(40)
#define MOTION_I2C_ADDR             (0x38)

static struct i2c_gpio_platform_data motion_i2c_gpio_pdata = {
	.sda_pin = GPIO_MOTION_I2C_SDA,
	.scl_pin = GPIO_MOTION_I2C_SCL,
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device motion_pdevice = {
	.name = "i2c-gpio", 
	.id = I2C_BUS_NUM_MOTION,
	.dev.platform_data = &motion_i2c_gpio_pdata,
};

static struct i2c_board_info motion_i2c_bdinfo[] = {
	{
		I2C_BOARD_INFO("bma150", MOTION_I2C_ADDR), 
		.irq = MSM_GPIO_TO_INT(GPIO_MOTION_INT),
	},
};

void __init init_i2c_gpio_motion(void)
{
	int rc = 0;
	/*
	gpio_configure(GPIO_MOTION_I2C_SDA, GPIOF_DRIVE_OUTPUT);
	gpio_configure(GPIO_MOTION_I2C_SCL, GPIOF_DRIVE_OUTPUT);
	*/
	GPIO_CFG(GPIO_MOTION_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), 
	GPIO_CFG(GPIO_MOTION_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), 
   rc = gpio_request(GPIO_MOTION_INT, "motion_int");
   if (rc < 0) {
      GI2C_E("failed to gpio %d request(ret=%d)\n", GPIO_MOTION_INT, rc);
      return;
   }
	gpio_direction_input(GPIO_MOTION_INT);

	rc = i2c_register_board_info(I2C_BUS_NUM_MOTION, motion_i2c_bdinfo, 1);
   if (rc < 0) {
      GI2C_E("failed to i2c register board info(busnum=%d, ret=%d)\n", I2C_BUS_NUM_MOTION, rc);
      return;
   }

	rc = platform_device_register(&motion_pdevice);
   if (rc != 0) {
      GI2C_E("failed to register motion platform device(ret=%d)\n", rc);
   }
}

/* AUDIO SUBSYSTEM : WOLFSON9093 */
#define GPIO_AMP_I2C_SCL		(27)		
#define GPIO_AMP_I2C_SDA		(17)	
#define AMP_I2C_ADDR          (0x6E)

static struct i2c_gpio_platform_data audiosub_i2c_gpio_pdata = {
	.sda_pin = GPIO_AMP_I2C_SDA,
	.scl_pin = GPIO_AMP_I2C_SCL,
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2
};

static struct platform_device audiosub_pdevice = {
	.name = "i2c-gpio",
	.id = I2C_BUS_NUM_AMP,
	.dev.platform_data = &audiosub_i2c_gpio_pdata,
};

static struct i2c_board_info audiosub_i2c_bdinfo[] = {
   {
	   I2C_BOARD_INFO("amp_wm9093", AMP_I2C_ADDR), 
   },
};

void __init init_i2c_gpio_amp(void)
{
   int rc = 0;
   /*
	gpio_configure(GPIO_AMP_I2C_SDA, GPIOF_DRIVE_OUTPUT);
	gpio_configure(GPIO_AMP_I2C_SCL, GPIOF_DRIVE_OUTPUT);
   */
	GPIO_CFG(GPIO_AMP_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), 
	GPIO_CFG(GPIO_AMP_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), 
	rc = i2c_register_board_info(I2C_BUS_NUM_AMP, audiosub_i2c_bdinfo, 1);
   if (rc < 0) {
      GI2C_E("failed to i2c register board info(busnum=%d, ret=%d)\n", I2C_BUS_NUM_AMP, rc);
      return;
   }

	rc = platform_device_register(&audiosub_pdevice);
   if (rc != 0) {
      GI2C_E("failed to register motion platform device(ret=%d)\n", rc);
   }
}




void __init swift_init_gpio_i2c_devices(void)
{
  // disable not using drivers --wingrime
     init_i2c_gpio_compass();
    init_i2c_gpio_motion();
    //  init_i2c_gpio_amp();

	return;
}
