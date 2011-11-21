/* linux/arch/arm/mach-msm/swift/board-swift-ss-driveric-panel.c
 *
 * Copyright (C) 2009-2011 LGE Inc.
 * Author : MoonCheol Kang <knight0708@lge.com>
 *
 * This software is licensed under the term of GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <mach/vreg.h>

#include <linux/backlight.h>

#include "msm_fb.h"
//#include "../../../../drivers/video/msm/mddihost_eve.h"
#include "mddihosti.h"

#define TM_GET_DID(id)	((id) & 0xff)
#define TM_GET_PID(id)	(((id) & 0xff00)>>8)
#define SWIFT_DEBUG_LCD 0 

#define LCD_CONTROL_BLOCK_BASE	0x110000
#define INTFLG		LCD_CONTROL_BLOCK_BASE|(0x18)
#define INTMSK		LCD_CONTROL_BLOCK_BASE|(0x1c)
#define VPOS		LCD_CONTROL_BLOCK_BASE|(0xc0)

#define GPIO_LCD_RESET_N		101

typedef enum {
	SS_POWER_OFF,
	SS_POWER_ON,
	SS_DISPLAY_ON,
	SS_DISPLAY_OFF
} mddi_ss_driveric_state_t;

typedef enum {
	INNOTEK,
} mddi_ss_panel_t;

static uint32 mddi_ss_driveric_curr_vpos;
static boolean mddi_ss_driveric_monitor_refresh_value = FALSE;
static boolean mddi_ss_driveric_report_refresh_measurements = FALSE;

/* These value is taken from EVE source */
/* Dot clock (10MHz) / pixels per row (320) = rows_per_second 
 * Rows Per second, this number arrived upon empirically
 * after observing the timing of Vsync pulses
 */ 
static uint32 mddi_ss_driveric_rows_per_second = 31250;
static uint32 mddi_ss_driveric_rows_per_refresh = 480;
/* rows_per_refresh / rows_per_second */
static uint32 mddi_ss_driveric_usecs_per_refresh = 15360;
extern boolean mddi_vsync_detect_enabled;

static msm_fb_vsync_handler_type mddi_ss_driveric_vsync_handler;
static void *mddi_ss_driveric_vsync_handler_arg;
static uint16 mddi_ss_driveric_vsync_attempts;

static mddi_ss_driveric_state_t ss_driveric_state = SS_POWER_OFF;
static mddi_ss_panel_t ss_panel = INNOTEK;
static struct msm_panel_common_pdata *mddi_ss_driveric_pdata;

static int mddi_ss_driveric_on(struct platform_device *pdev);
static int mddi_ss_driveric_off(struct platform_device *pdev);

/* START table for innotek panel from EVE source */
#define REGFLAG_DELAY	      0xFFFE
#define REGFLAG_END_OF_TABLE	0xFFFF

struct display_table {
	unsigned reg;
	unsigned char count;
	unsigned int val_list[5];
};

static struct display_table mddi_innotek_display_on[] = {
#if 1
	{0x29, 1, {0x00000000}},
#else
	// Display on sequence
	{0xEF, 1, {0x00000006}},
	{REGFLAG_DELAY, 20, {}},
	{0xEF, 1, {0x00000007}},
//	{REGFLAG_END_OF_TABLE, 0x00, {}}
#endif
};

static struct display_table mddi_innotek_display_off[] = {
#if 1
	{0x28, 1, {0x00000000}},
#else
	// Display off sequence
	{0xEF, 1, {0x00000006}},
	{REGFLAG_DELAY, 20, {}},
	{0xEF, 1, {0x00000000}},
	{REGFLAG_DELAY, 10, {}},
//	{REGFLAG_END_OF_TABLE, 0x00, {}}
#endif
};

static struct display_table mddi_innotek_power_on[] = {
#if 1
	/* Power on Sequence */
	{0x11, 1, {0x00000000}},
    {REGFLAG_DELAY, 120,{}},
	{0xF3, 3, {0x00000101, 0x7F2C4407, 0x00000000}},
	{0xF4, 2, {0x5527230D, 0x00000011}},
	{0xF5, 2, {0xF0080010, 0x00001F05}},
	/* Normal Mode */
	{0x13, 1, {0x00000000}},
	/* Column address */
	{0x2A, 1, {0x3F010000}},
	/* Page address */
	{0x2B, 1, {0xDF010000}},
	/* Memory data address Control */
	{0x36, 1, {0x00000048}},
	/* Interface pixel format */
	{0x3A, 1, {0x00000005}},
	/* Display Control Set */
	{0xF2, 3, {0x03031414, 0x03080803, 0x00151500}},
	/* Tearing Effect Lion ON */
	{0x35, 1, {0x00000000}},
	/* CABC Control */
	{0x51, 1, {0x000000FF}},
	{0x53, 1, {0x0000002C}},
	{0x55, 1, {0x00000003}},
	{0x5E, 1, {0x00000000}},
	{0xCA, 1, {0x003F8080}},
	{0xCB, 1, {0x00000003}},
	{0xCC, 2, {0x008F0120, 0x000000EF}},
	{0xCD, 1, {0x00009704}},
	/* Positive Gamma RED */
	{0xF7, 4, {0x11000000, 0x35322C17, 0x00000105, 0x00222200}},
	/* Negative Gamma RED */
	{0xF8, 4, {0x11000000, 0x35322C17, 0x00000105, 0x00222200}},
	/* Positive Gamma GREEN */
	{0xF9, 4, {0x11000000, 0x35322C17, 0x00000105, 0x00222200}},
	/* Negative Gamma GREEN */
	{0xFA, 4, {0x11000000, 0x35322C17, 0x00000105, 0x00222200}},
	/* Positive Gamma BLUE */
	{0xFB, 4, {0x11000000, 0x35322C17, 0x00000105, 0x00222200}},
	/* Negative Gamma BLUE */
	{0xFC, 4, {0x11000000, 0x35322C17, 0x00000105, 0x00222200}},
	/* Gate Control Register */
	{0xFD, 1, {0x00003B11}},
	/* GRAM Write */
	{0x2C, 1, {0x00000000}},
	/* Display On */
	{0x29, 1, {0x00000000}},
#else
	/* Power ON Sequence */
    {0xF3, 1, {0x00000000}},
    {0x11, 1, {0x00000000}},
    {REGFLAG_DELAY, 10, {}},
    {0xF3, 3, {0x00000100, 0x7F204407, 0x00000000}},
    {0xF4, 2, {0x5531230D, 0x00000011}},
    {0xF5, 2, {0xF0080010, 0x00001F35}},
    {REGFLAG_DELAY, 10, {}},
    {0xF3, 1, {0x00000300}},    
    {REGFLAG_DELAY, 10, {}},
    {0xF3, 1, {0x00000700}},    
    {REGFLAG_DELAY, 10, {}},
    {0xF3, 1, {0x00000F00}},    
    {REGFLAG_DELAY, 10, {}},
    {0xF3, 1, {0x00001F00}},    
    {REGFLAG_DELAY, 10, {}},
    {0xF3, 1, {0x00003F00}},    
    {REGFLAG_DELAY, 20, {}},
    {0xF3, 1, {0x00007F00}},
    {REGFLAG_DELAY, 30, {}},

    // Normal Mode
    {0x13, 1, {0x00000000}},

    // Column Address
    {0x2A, 1, {0x3F010000}},

    // Page Address
    {0x2B, 1, {0xDF010000}},

    // Memory Data Address Control
    {0x36, 1, {0x00000048}},

    // Interface Pixel Format
    {0x3A, 1, {0x00000005}},

	// Display Control Set
    {0xF2, 3, {0x03031416, 0x03080803, 0x00151500}},

	// Tearing Effect Lion On
    {0x35, 1, {0x00000000}},

	// CABC Control
    {0x51, 1, {0x000000FF}},
    {0x53, 1, {0x0000002C}},
    {0x55, 1, {0x00000003}},
    {0x5E, 1, {0x00000000}},
    {0xCA, 1, {0x003F8080}},
    {0xCB, 1, {0x00000003}},
    {0xCC, 2, {0x008F0120, 0x000000EF}},
    {0xCD, 1, {0x00009704}},

    // Positive Gamma Red
    {0xF7, 4, {0x11000000, 0x292A2318, 0x00050812, 0x00222200}},

    // Negative Gamma Red
    {0xF8, 4, {0x11000000, 0x292A2318, 0x00050812, 0x00222200}},

    // Positive Gamma Green
    {0xF9, 4, {0x11000000, 0x292A2318, 0x00050812, 0x00222200}},

    // Negative Gamma Green
    {0xFA, 4, {0x11000000, 0x292A2318, 0x00050812, 0x00222200}},

    // Positive Gamma Blue
    {0xFB, 4, {0x11000000, 0x292A2318, 0x00050812, 0x00222200}},

    // Negative Gamma Blue
    {0xFC, 4, {0x11000000, 0x292A2318, 0x00050812, 0x00222200}},

    // Gate Control Register
    {0xFD, 1, {0x00003B11}},

    // GRAM Write
    {0x2C, 1, {0x00000000}},

    // Dispaly On Sequence
    {0xEF, 1, {0x00000006}},
    {REGFLAG_DELAY, 55, {}},
    {0xEF, 1, {0x00000007}},

 //   {REGFLAG_END_OF_TABLE, 0x00, {}}
#endif
};

static struct display_table mddi_innotek_power_off[] = {
#if 1
	{0x28, 1, {0x00000000}},
	{0x10, 1, {0x00000000}},
	{REGFLAG_DELAY, 120,{}},
	{0xE0, 1, {0x00000002}},
#else
	// Display off sequence
	{0xEF, 1, {0x00000006}},
	{REGFLAG_DELAY, 90, {}},
	{0xEF, 1, {0x00000000}},
	{REGFLAG_DELAY, 10, {}},

	// Sleep In
	{0xF3, 1, {0x00000000}},
	{0x10, 1, {0x00000000}},

//   {REGFLAG_END_OF_TABLE, 0x00, {}}
#endif
};

static struct display_table mddi_innotek_position[] = {
   // Column Address
    {0x2A, 1, {0x3F010000}},
    // Page Address
    {0x2B, 1, {0xDF010000}},
};



static void display_table(struct display_table *table, unsigned int count)
{
	unsigned int i;
	for(i = 0; i < count; i++) {
        unsigned reg;
        reg = table[i].reg;
		
        switch (reg) {
		case REGFLAG_DELAY :
			//msleep(table[i].count);
			mdelay(table[i].count - 2);
			break;
		//case REGFLAG_END_OF_TABLE :
			//break;
		default:
			mddi_host_register_cmd_write(reg, table[i].count, table[i].val_list,
					0, 0, 0);
#if SWIFT_DEBUG_LCD
		       printk(KERN_INFO, "reg : %x, val : 0x%X.\n",
					reg, table[i].val_list[0]);
#endif
               break;
       	}
    }
}
/* END table for innotek panel from EVE source */

static void mddi_ss_driveric_vsync_set_handler(msm_fb_vsync_handler_type handler,	/*ISR to be excuted */
					void *arg)
{
	boolean error = FALSE;
	unsigned long flags;

	printk( KERN_INFO "%s : handler = %x\n", 
			__func__, (unsigned int)handler);

	/* Disable interrupts */
	spin_lock_irqsave(&mddi_host_spin_lock, flags);
	/* INTLOCK(); */

	if (mddi_ss_driveric_vsync_handler != NULL) {
		error = TRUE;
	} else {
		/* Register the handler for this particular GROUP interrupt source */
		mddi_ss_driveric_vsync_handler = handler;
		mddi_ss_driveric_vsync_handler_arg = arg;
	}
	
	/* Restore interrupts */
	spin_unlock_irqrestore(&mddi_host_spin_lock, flags);
	/* MDDI_INTFREE(); */
	if (error) {
		MDDI_MSG_ERR("MDDI: Previous Vsync handler never called\n");
	} else {
		/* Enable the vsync wakeup */
		mddi_queue_register_write(INTMSK, 0x0000, FALSE, 0);

		mddi_ss_driveric_vsync_attempts = 1;
		mddi_vsync_detect_enabled = TRUE;
	}
}

static void mddi_ss_driveric_vsync_detected(boolean detected)
{
	/* static timetick_type start_time = 0; */
	static struct timeval start_time;
	static boolean first_time = TRUE;
	/* unit32 mdp_cnt_val = 0; */
	/* timetick_type elapsed_us; */
	struct timeval now;
	uint32 elapsed_us;
	uint32 num_vsyncs;
#if SWIFT_DEBUG_LCD
	printk(KERN_INFO "%s : detected = %d\n", __func__, detected);
#endif
	if ((detected) || (mddi_ss_driveric_vsync_attempts > 5)) {
		if ((detected) || (mddi_ss_driveric_monitor_refresh_value)) {
			/* if (start_time != 0) */
			if (!first_time) {
				jiffies_to_timeval(jiffies, &now);
				elapsed_us =
					(now.tv_sec - start_time.tv_sec) * 1000000 +
					now.tv_usec - start_time.tv_usec;
				/*
				 * LCD is configured for a refresh every usecs,
				 *  so to determine the number of vsyncs that
				 *  have occurred since the last measurement
				 *  add half that to the time difference and
				 *  divide by the refresh rate.
				 */
				num_vsyncs = (elapsed_us +
						(mddi_ss_driveric_usecs_per_refresh >>
						 1))/
					mddi_ss_driveric_usecs_per_refresh;
				/*
				 * LCD is configured for * hsyncs (rows) per
				 * refresh cycle. Calculate new rows_per_second
				 * value based upon these new measuerments.
				 * MDP can update with this new value.
				 */
				mddi_ss_driveric_rows_per_second =
					(mddi_ss_driveric_rows_per_refresh * 1000 *
					 num_vsyncs) / (elapsed_us / 1000);
			}
			/* start_time = timetick_get();*/
			first_time = FALSE;
			jiffies_to_timeval(jiffies, &start_time);
			if (mddi_ss_driveric_report_refresh_measurements) {
				(void)mddi_queue_register_read_int(VPOS,
									&mddi_ss_driveric_curr_vpos);
				/* mdp_cnt_val = MDP_LINE_COUNT; */
			}
		}
		/* if detected = TRUE, client initiated wakeup was detected */
		if (mddi_ss_driveric_vsync_handler != NULL) {
			(*mddi_ss_driveric_vsync_handler)
				(mddi_ss_driveric_vsync_handler_arg);
			mddi_ss_driveric_vsync_handler = NULL;
		}
		mddi_vsync_detect_enabled = FALSE;
		mddi_ss_driveric_vsync_attempts = 0;
		/* need to disable the interrupt wakeup */
		if (!mddi_queue_register_write_int(INTMSK, 0x0001))
			MDDI_MSG_ERR("Vsync interrupt disable failed!\n");
		if (!detected) {
			/* give up after 5 failed attempts but show error */
			MDDI_MSG_NOTICE("Vsync detection failed!\n");
		} else if ((mddi_ss_driveric_monitor_refresh_value) &&
				(mddi_ss_driveric_report_refresh_measurements)) {
			MDDI_MSG_NOTICE("  Last Line Counter=%d!\n",
					mddi_ss_driveric_curr_vpos);
			/* MDDI_MSG_NOTICE("  MDP Line Counter=%d!\n",mdp_cnt_val); */
			MDDI_MSG_NOTICE("  Lines Per Second=%d!\n",
					mddi_ss_driveric_rows_per_second);
		}
		/* clear the interrupt */
		if (!mddi_queue_register_write_int(INTFLG, 0x0001))
			MDDI_MSG_ERR("Vsync interrupt clear failed!\n");
	} else {
		/* if detected = FALSE, we woke up from hibernation, but did not
		 * detect client initiated wakeup.
		 */
		mddi_ss_driveric_vsync_attempts++;
	}
}

static int innotek_panel_hw_reset(void)
{
#if SWIFT_DEBUG_LCD
        printk(KERN_INFO  "%s : Reset pin = %d\n",__func__, GPIO_LCD_RESET_N);
#endif
	gpio_set_value(GPIO_LCD_RESET_N, 1);
	mdelay(9);
	gpio_set_value(GPIO_LCD_RESET_N, 0);
	mdelay(9);
	gpio_set_value(GPIO_LCD_RESET_N, 1);

	return 0;
}

static int mddi_ss_driveric_display_on(struct msm_fb_data_type *mfd)
{
	ss_driveric_state = SS_POWER_ON;
	display_table(mddi_innotek_display_on, ARRAY_SIZE(mddi_innotek_display_on));

	return 0;
}

static int mddi_ss_driveric_display_off(struct msm_fb_data_type *mfd)
{
	ss_driveric_state = SS_POWER_OFF;
	display_table(mddi_innotek_display_off, ARRAY_SIZE(mddi_innotek_display_off));

	return 0;
}

static int mddi_ss_driveric_innotek_powerdown(struct msm_fb_data_type *mfd)
{

	ss_driveric_state = SS_POWER_OFF;
	display_table(mddi_innotek_power_off, ARRAY_SIZE(mddi_innotek_power_off));
	gpio_set_value(GPIO_LCD_RESET_N, 0);

	return 0;
}

static int mddi_ss_driveric_innotek_poweron(struct msm_fb_data_type *mfd)
{
	ss_driveric_state = SS_POWER_ON;
	display_table(mddi_innotek_power_on, ARRAY_SIZE(mddi_innotek_power_on));

	return 0;
}

int mddi_ss_driveric_innotek_position(void)
{
	display_table(mddi_innotek_position, ARRAY_SIZE(mddi_innotek_position));
	return 0;
}
EXPORT_SYMBOL(mddi_ss_driveric_innotek_position);

static int mddi_ss_driveric_powerdown(struct msm_fb_data_type *mfd)
{
#if SWIFT_DEBUG_LCD
        printk( KERN_INFO "%s : state = %d\n", __func__, ss_panel);
#endif
	mddi_ss_driveric_innotek_powerdown(mfd);

	return 0;
}

static int mddi_ss_driveric_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct vreg *vreg;
	unsigned int ret=0;
	
   if (system_state != SYSTEM_BOOTING) {
   	vreg = vreg_get(NULL, "gp1");
	   if(IS_ERR(vreg)) {
		   printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
			   	__func__, "gp1", PTR_ERR(vreg));
	   }
	   ret = vreg_enable(vreg);
	   if (ret) {
		   printk(KERN_ERR "%s: vreg enabled failed!\n", __func__);
	   }

	   vreg = vreg_get(NULL, "gp2");
	   if (IS_ERR(vreg)) {
		   printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
			   	__func__, "gp2", PTR_ERR(vreg));
	   }
	   ret = vreg_enable(vreg);
	   if (ret) {
		   printk(KERN_ERR "%s: vreg enabled failed!\n", __func__);
	   }

	   mdelay(1);
   }

#if SWIFT_DEBUG_LCD	
	printk(KERN_INFO  "%s\n", __func__);
#endif
	mfd = platform_get_drvdata(pdev);

	if (system_state != SYSTEM_BOOTING) {
		innotek_panel_hw_reset();
		mddi_ss_driveric_innotek_poweron(mfd);
	}

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

#if 0
		mddi_ss_driveric_innotek_init(mfd);
#endif

	return 0;
}

static int mddi_ss_driveric_off(struct platform_device *pdev)
{
	struct vreg *vreg;
	unsigned int ret=0;

	printk(KERN_INFO  "%s\n", __func__);
#if SWIFT_DEBUG_LCD
	mddi_ss_driveric_powerdown(platform_get_drvdata(pdev));
#endif
	//	vreg = vreg_get(NULL, "gp1");
	//	if(IS_ERR(vreg)) {
	//	printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
	//				__func__, "gp1", PTR_ERR(vreg));
//	}
	//	ret = vreg_must_disable(vreg);
	//	if(ret) {
	//		printk(KERN_ERR "%s: vreg disabled failed!\n", __func__);//
	//	}

	//	vreg = vreg_get(NULL, "gp2");
	//	if(IS_ERR(vreg)) {
	//		printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
		       //			__func__, "gp2", PTR_ERR(vreg));
	//	}
//	ret = vreg_must_disable(vreg);
//	if(ret) {
	  //	printk(KERN_ERR "%s: vreg disabled failed!\n", __func__);
		//	}

	return 0;
}

#ifdef CONFIG_FB_BACKLIGHT
int rt9393_set_intensity(struct backlight_device * bd);

static int
mddi_ss_driveric_set_backlight(struct msm_fb_data_type * mfd)
{
        struct backlight_device *bd;
        bd->props.brightness = mfd->bl_level;
#if SWIFT_DEBUG_LCD
        printk(KERN_INFO  "[sungwoo] bl_level : %d \n", mfd->bl_level);
#endif
        rt9393_set_intensity(bd);
        
        return 0;
}
#endif

static int __devinit mddi_swift_probe(struct platform_device *pdev)
{
	int err;

	if (pdev->id == 0) {
	 	printk(KERN_INFO  "%s : pdev->id = 0\n", __func__);
		mddi_ss_driveric_pdata = pdev->dev.platform_data;
		return 0;
	}

	
		err = gpio_request(GPIO_LCD_RESET_N, 0);
		if (err < 0 ) {
			printk(KERN_ERR "Cannot get the gpio pin : %d\n", GPIO_LCD_RESET_N);
			return err;
		}
	
	msm_fb_add_device(pdev);

	return 0;
}

static struct  platform_driver this_driver = {
  	.probe	= mddi_swift_probe,
	.driver	= {
		.name	= "mddi_swift",
	},
};

static struct msm_fb_panel_data ss_driveric_panel_data = {
	.on	= mddi_ss_driveric_on,
	.off	= mddi_ss_driveric_off,
};

static int ch_used[3];

int mddi_ss_driveric_register(struct msm_panel_info *pinfo,
		u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	printk(KERN_INFO  "%s\n", __func__);


	if ((channel > 2) || ch_used[channel])
		return -ENODEV;

	if ((channel != INNOTEK) &&
		mddi_ss_driveric_pdata && mddi_ss_driveric_pdata->panel_num)
		if (mddi_ss_driveric_pdata->panel_num() < 2)
			return -ENODEV;

	ch_used[channel] = TRUE;
	
	pdev = platform_device_alloc("mddi_swift", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	if (channel == INNOTEK) {

#ifdef CONFIG_FB_BACKLIGHT
		ss_driveric_panel_data.set_backlight =
			mddi_ss_driveric_set_backlight;
#endif

		if (pinfo->lcd.vsync_enable) {
			ss_driveric_panel_data.set_vsync_notifier =
				mddi_ss_driveric_vsync_set_handler;
			mddi_lcd.vsync_detected =
				mddi_ss_driveric_vsync_detected;
		}
	} else {
		ss_driveric_panel_data.set_backlight = NULL;
		ss_driveric_panel_data.set_vsync_notifier = NULL;
	}

	ss_driveric_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &ss_driveric_panel_data,
			sizeof(ss_driveric_panel_data));
	if (ret) {
		printk(KERN_ERR
			"%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
			"%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mddi_ss_driveric_init(void)
{
	return platform_driver_register(&this_driver);
}

module_init(mddi_ss_driveric_init);
MODULE_DESCRIPTION("Mddi Driver Swift(MSM)");
MODULE_LICENSE("GPL");

