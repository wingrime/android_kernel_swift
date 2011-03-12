/* kernel/arch/arm/mach-msm/swift/board-swift-msm-touch.c
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <asm/mach-types.h>

#include <mach/msm_touch.h>

#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>

/* HW register map */
#define TSSC_CTL_REG      0x100
#define TSSC_SI_REG       0x108
#define TSSC_OPN_REG      0x104
#define TSSC_STATUS_REG   0x10C
#define TSSC_AVG12_REG    0x110

/* status bits */
#define TSSC_STS_OPN_SHIFT 0x6
#define TSSC_STS_OPN_BMSK  0x1C0
#define TSSC_STS_NUMSAMP_SHFT 0x1
#define TSSC_STS_NUMSAMP_BMSK 0x3E

/* CTL bits */
#define TSSC_CTL_EN		(0x1 << 0)
#define TSSC_CTL_SW_RESET	(0x1 << 2)
#define TSSC_CTL_MASTER_MODE	(0x3 << 3)
#define TSSC_CTL_AVG_EN		(0x1 << 5)
#define TSSC_CTL_DEB_EN		(0x1 << 6)
#define TSSC_CTL_DEB_12_MS	(0x2 << 7)	/* 1.2 ms */
#define TSSC_CTL_DEB_16_MS	(0x3 << 7)	/* 1.6 ms */
#define TSSC_CTL_DEB_2_MS	(0x4 << 7)	/* 2 ms */
#define TSSC_CTL_DEB_3_MS	(0x5 << 7)	/* 3 ms */
#define TSSC_CTL_DEB_4_MS	(0x6 << 7)	/* 4 ms */
#define TSSC_CTL_DEB_6_MS	(0x7 << 7)	/* 6 ms */
#define TSSC_CTL_INTR_FLAG1	(0x1 << 10)
#define TSSC_CTL_DATA		(0x1 << 11)
#define TSSC_CTL_SSBI_CTRL_EN	(0x1 << 13)

#define TSSC_CTL_STATE	  ( \
		TSSC_CTL_DEB_6_MS | \
		TSSC_CTL_DEB_EN | \
		TSSC_CTL_AVG_EN | \
		TSSC_CTL_MASTER_MODE | \
		TSSC_CTL_EN)

#define TSSC_NUMBER_OF_OPERATIONS 2
#define TSSC_SI_STATE 8     

#if defined(CONFIG_MACH_MSM7X27_SWIFT)
#define TS_PENUP_TIMEOUT_MS 15 /* 100 */
#else	/* by qualcomm */
#define TS_PENUP_TIMEOUT_MS 20
#endif

#define TS_DRIVER_NAME "swift_touchscreen"

#define TS_KEY_CALMODE

#define X_MAX 		2783 /* Rev.C: 2890 Rev.B: 2800 */
#define Y_MAX 		2555 /* Rev.C: 2550 Rev.B: 2480 */
#define X_MIN 		1267 /* Rev.C: 1195 Rev.B: 1290 */
#define Y_MIN   	   0 /* Rev.C:   15 Rev.B:   70 */
#define MENU_KEY_X  1220 /* Rev1: 1180 Rev.C: 1150 Rev.B: 1200 */
#define BACK_KEY_X  2840 /* Rev1: 2880 Rev.C: 2965 Rev.B: 2880 */
#define TS_KEY_Y	2600 

#define P_MAX	256

static int PreRejectTouchCount = 0;
static int preRejectValue = 2;

static uint32_t msm_tsdebug;
module_param_named(debug_mask, msm_tsdebug, uint, 0664);

struct ts {
	struct input_dev *input;
	struct timer_list timer;
	int irq;
	unsigned int x_max;
	unsigned int y_max;
	
	unsigned int count;
	int x_lastpt;
	int y_lastpt;    
	
	u8 keypad;
};

static int TouchWindowPress = 1; 
//extern int ts_key_event;
int ts_key_event;
static void __iomem *virt;
#define TSSC_REG(reg) (virt + TSSC_##reg##_REG)

static int menu_x = MENU_KEY_X;
static int menu_y = TS_KEY_Y; 
static int back_x = BACK_KEY_X;
static int back_y = TS_KEY_Y; 

#if defined(TS_KEY_CALMODE)

#define TOUCH_KEY_FILENAME "/data/nv/tskey_cal"

int ts_calibration_for_touch_key_region(char *filename, int *cal_data)
{
	int fd, err = 0;
	int count, i,j,ii, count1 = 0;
	int cal_data_count = 0;

	char data1[50]= {0,};
	char data2[10]={0,};
	int  value = 0, value1 = 0;
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	printk("[SWIFT]TS_CAL. sys_open-- for READ........\n");
	fd = sys_open(filename, O_RDWR, 0);
	if (fd < 0) {
		printk(KERN_WARNING "%s: Can not open %s\n",
				__func__, filename);
		return -ENOENT;
	}

	printk("[SWIFT]TS_CAL. sys_lseek........\n");
	count = sys_lseek(fd, (off_t)0, 2);
	if (count == 0) {
		printk("[SWIFT]TS_CAL. sys_lseek ERROR count == 0........\n");
		err = -EIO;
		goto err_close_file;
	}

	sys_lseek(fd, (off_t)0, 0);

	printk("[SWIFT]TS_CAL. kmalloc......count[%d]..\n",count);
	printk("[SWIFT] sys_read........\n");

	(unsigned)sys_read(fd, (char *)data1, count);

	count1 = 0;
	for(i =0 ; i < count ; i++){
		//printk("data[%d]=0x%x\n",i,data1[i]);
		if((data1[i]== 0x2C)||(data1[i]== 0x0D)||data1[i]== 0x0A){
			ii = 0;
			value = 0;

			while(count1 > 0){
				count1-=1;

				for(j=0, value1=1; j < count1 ; j++)
					value1*= 10;

				value +=(data2[ii]-0x30)*value1;
				//printk("[SWIFT]ii[%d] FIND value[%d]..data2[%d]... count1[%d]\n",ii,value,data2[ii],count1);

				if (count1 == 0) {
					cal_data[cal_data_count] = value;
					cal_data_count++;
				}

				ii++;
			} 

			memset(data2,0x00,sizeof(data2));
			count1 = 0;
		}
		else{
			data2[count1]= data1[i];
			//printk("[SWIFT] count1[%d]..data2[0x%x]....data1[0x%x]..i[%d]...\n",count1,data2[count1],data1[i],i);
			count1+=1;     
		} 
	}

	printk("[SWIFT]TS_CAL. sys_close........\n");
	sys_close(fd);
	set_fs(old_fs);

	return 0;

err_close_file:
	printk("[SWIFT]err_close_file!!........\n");
	sys_close(fd);
	set_fs(old_fs);
	return err;

}

/*  ---------------------------------------------------------------------------*/	 
/*	 						 ioctl command API                                 */
/*  ---------------------------------------------------------------------------*/	 
#define TOUCH_CAL_IOC_MAGIC	  0xA1
	 
#define TOUCH_CAL_SET_DATA   	   _IOWR(TOUCH_CAL_IOC_MAGIC, 0x01, int[8])
#define TOUCH_CAL_TOUCH_KEY_MODE   _IOWR(TOUCH_CAL_IOC_MAGIC, 0x02, int)

static int touch_cal_open(struct inode *inode, struct file *file)
{	   
	 int status = 0;

	 printk("touch_cal_open\n"); 	 
	 return status;
}

static int touch_cal_release(struct inode *inode, struct file *file)
{
	 printk("touch_release\n"); 	 
	 return 0;
}

static int touch_cal_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int cal_data[8] = {0,};
	int current_cal_mode = 0;
	int i;
	int x1 = 0, y1 = 0, x2 = 0, y2 = 0, x3 = 0, y3 = 0, x4 = 0, y4 = 0;

	printk("[touch_cal_ioctl]cmd[%d]\n", cmd); 

	switch (cmd) {
	case TOUCH_CAL_SET_DATA:

		printk("[touch_cal_ioctl]TOUCH_CAL_SET_DATA\n"); 

		ts_calibration_for_touch_key_region(TOUCH_KEY_FILENAME, cal_data);

		/*
		if (copy_from_user(&cal_data[0], argp, sizeof(cal_data))){
			return -ENODEV;
		}
		*/

		for(i=0 ; i < 8 ; i++)
			printk("[SWIFT TOUCH CAL](%d) cal_data:%d\n", i, cal_data[i]);  

		if (cal_data[0]) {
			x1 = (cal_data[0] * (X_MAX - X_MIN) / 320) + X_MIN;
		}

		if (cal_data[1]) {
			y1 = ((cal_data[1] + 20) * (Y_MAX - Y_MIN) / 450) + Y_MIN;
		}

		if (cal_data[2]) {
			x2 = (cal_data[2] * (X_MAX - X_MIN) / 320) + X_MIN;
		}

		if (cal_data[3]) {
			y2 = (cal_data[3] * (Y_MAX - Y_MIN) / 450) + Y_MIN;
		}

		if (cal_data[4]) {
			x3 = (cal_data[4] * (X_MAX - X_MIN) / 320) + X_MIN;
		}

		if (cal_data[5]) {
			y3 = (cal_data[5] * (Y_MAX - Y_MIN) / 450) + Y_MIN;
		}

		if (cal_data[6]) {
			x4 = (cal_data[6] * (X_MAX - X_MIN) / 320) + X_MIN;
		}

		if (cal_data[7]) {
			y4 = ((cal_data[7] + 20 )* (Y_MAX - Y_MIN) / 450) + Y_MIN;
		}

		menu_x = x1 - ((x4 - x1) * 100 / 541);
		menu_y = y2 + ((y2 - y1) * 100 / 841); /* 567 */
		back_x = x4 + ((x4 - x1) * 100 / 541);
		back_y = y3 + ((y3 - y4) * 100 / 841); /* 567 */
	
		if (!menu_x || !menu_y || !back_x || !back_y) {
			menu_x = MENU_KEY_X;
			menu_y = TS_KEY_Y; 
			back_x = BACK_KEY_X;
			back_y = TS_KEY_Y; 
		}

		printk("[SWIFT TOUCH CAL] X1:%d, Y1:%d\n", x1, y1);  
		printk("[SWIFT TOUCH CAL] X2:%d, Y2:%d\n", x2, y2);  
		printk("[SWIFT TOUCH CAL] X3:%d, Y3:%d\n", x3, y3);  
		printk("[SWIFT TOUCH CAL] X4:%d, Y4:%d\n", x4, y4);  
		printk("[SWIFT TOUCH CAL] MENU X:%d, MENU Y:%d\n", menu_x, menu_y);  
		printk("[SWIFT TOUCH CAL] BACK X:%d, BACk Y:%d\n", back_x, back_y);  

		break;
	}

	return 0;
}

static struct file_operations touch_cal_fops = {
	.owner	  = THIS_MODULE,
	.open	  = touch_cal_open,
	.release  = touch_cal_release,
	.ioctl	  = touch_cal_ioctl,
};

static struct miscdevice touch_cal_misc_device = {
	.minor	  = MISC_DYNAMIC_MINOR,
	.name	  = "swift_tssc_cal",
	.fops	  = &touch_cal_fops,
};
#endif

static int ts_check_region(struct ts *ts, int x, int y, int pressure)
{
    int update_event = false;
	int x_axis, y_axis;
	int x_diff, y_diff;
	x_axis = x;
	y_axis = y;

	if (ts->count == 0){
		ts->x_lastpt = x_axis;
		ts->y_lastpt = y_axis;

        update_event = true;
		TouchWindowPress = 1;
		ts->count = ts->count + 1;
	}	
	
	x_diff = x_axis - ts->x_lastpt;
	if (x_diff < 0)
		x_diff = x_diff * -1;
	y_diff = y_axis - ts->y_lastpt;
    if (y_diff < 0)
        y_diff = y_diff * -1;


    if ((x_diff < 40) && (y_diff < 40)){
        x_axis = ts->x_lastpt;
        y_axis = ts->y_lastpt;
    } else {
        ts->x_lastpt = x_axis;
        ts->y_lastpt = y_axis;
	
        x = x_axis;
        y = y_axis;

        update_event = true;
		TouchWindowPress = 1;
	}	

    return update_event;
}

static void ts_update_pen_state(struct ts *ts, int x, int y, int pressure)
{
	if (pressure) {
		if (ts_check_region(ts, x, y, pressure) == false)
				  return;

		input_report_abs(ts->input, ABS_X, x);
		input_report_abs(ts->input, ABS_Y, y);
		input_report_abs(ts->input, ABS_PRESSURE, pressure);
		input_report_key(ts->input, BTN_TOUCH, !!pressure);
		input_sync(ts->input);

	} else {
		input_report_abs(ts->input, ABS_PRESSURE, 0);
		input_report_key(ts->input, BTN_TOUCH, 0);
		input_sync(ts->input);
		ts->count=0;
		TouchWindowPress = 0;

	}
}

static void ts_timer(unsigned long arg)
{
	struct ts *ts = (struct ts *)arg;

	ts->count = 0;
//#if defined(CONFIG_MACH_MSM7X27_SWIFT)
	input_report_abs(ts->input, ABS_PRESSURE, 0);
	input_report_key(ts->input, BTN_TOUCH, 0);
    input_sync(ts->input);

	TouchWindowPress = 0;
	PreRejectTouchCount =0; 
//#if defined(CONFIG_MACH_MSM7X27_SWIFT_REV_1)
	if (ts->keypad == KEY_MENU) {
		input_report_key(ts->input, KEY_MENU, 0);
		ts->keypad = 0;
	}
//#else
//	if (ts->keypad == KEY_HOME) {
//		input_report_key(ts->input, KEY_HOME, 0);
//		ts->keypad = 0;
//	}
//#endif
	if (ts->keypad == KEY_BACK) {
		input_report_key(ts->input, KEY_BACK, 0);
		ts->keypad = 0;
		ts_key_event = 0;
	}

//#else /* by qualcomm */
//	ts_update_pen_state(ts, 0, 0, 0);
//#endif
}

static irqreturn_t ts_interrupt(int irq, void *dev_id)
{
	u32 avgs, x, y, lx, ly, x_prime, y_prime;
	u32 num_op, num_samp;
	u32 status;

	struct ts *ts = dev_id;
	status = readl(TSSC_REG(STATUS));
	avgs = readl(TSSC_REG(AVG12));
	x = avgs & 0xFFFF;
	y = avgs >> 16;

	/* For pen down make sure that the data just read is still valid.
	 * The DATA bit will still be set if the ARM9 hasn't clobbered
	 * the TSSC. If it's not set, then it doesn't need to be cleared
	 * here, so just return.
	 */
	if (!(readl(TSSC_REG(CTL)) & TSSC_CTL_DATA))
		goto out;

	/* Data has been read, OK to clear the data flag */
	writel(TSSC_CTL_STATE, TSSC_REG(CTL));
    writel(TSSC_SI_STATE, TSSC_REG(SI));

	/* Valid samples are indicated by the sample number in the status
	 * register being the number of expected samples and the number of
	 * samples collected being zero (this check is due to ADC contention).
	 */
	num_op = (status & TSSC_STS_OPN_BMSK) >> TSSC_STS_OPN_SHIFT;
	num_samp = (status & TSSC_STS_NUMSAMP_BMSK) >> TSSC_STS_NUMSAMP_SHFT;

	if ((num_op == TSSC_NUMBER_OF_OPERATIONS) && (num_samp == 0)) {
		/* TSSC can do Z axis measurment, but driver doesn't support
		 * this yet.
		 */

		/*
		 * REMOVE THIS:
		 * These x, y co-ordinates adjustments will be removed once
		 * Android framework adds calibration framework.
		 */
		
		x_prime = y;
		y_prime = x;

//#if defined(CONFIG_MACH_MSM7X27_SWIFT)
		lx = x_prime;
		ly = 3300 - y_prime;  /* 10bit : 827  12bit : 3300 */
//#else
//#ifdef CONFIG_ANDROID_TOUCHSCREEN_MSM_HACKS
//		lx = ts->x_max + 25 - x;
//		ly = ts->y_max + 25 - y;
//		if (machine_is_msm7201a_surf()) {
//			if (lx > 435) {
//				/* Max out x for points lying outside hvga display */
//				lx = X_MAX;
//			} else {
//				/* Scale x for hvga display */
//				if (lx < 250)
//					lx = lx * 2 - 55;
//				else if (lx > 250 && lx < 260)
//					lx = lx * 2;
//				else
//					lx = lx * 2 + 70;
//			}
//		} else {
//			/* manipulate x,y co-ordinates for ffa */
//			if (lx > 700 || ly > 820) {
//				/* Max out x for points lying outside hvga display */
//				lx = X_MAX;
//				ly = Y_MAX;
//			} else {
//				if (ly < 700 && ly > 280) {
//					lx = lx * 2 - 250 ;
//					ly = ly  + 160;
//				} else if (lx > 530)
//					lx = lx * 2 + 30;
//				else if (ly < 280)
//					ly = ly - 50;
//				else
//					ly = ly + 250;
//			}
//		}
//#else
//		lx = x;
//		ly = y;
//#endif
//#endif

		if (msm_tsdebug & 1)
			printk("++++++++x=%d, y=%d++++++++\n", lx, ly);

		if ((lx  <  menu_x) && (ly > menu_y)) {
//#if defined(CONFIG_MACH_MSM7X27_SWIFT_REV_1)
			if (msm_tsdebug & 1)
				printk("Menu key : x=%d, y=%d\n", lx, ly);
			
			if (ts->keypad == 0) {
				printk("input report MENU key\n");
				input_report_key(ts->input, KEY_MENU, 1);
				ts->keypad = KEY_MENU;
			}
//#else
//			if (msm_tsdebug & 1)
//				printk("Home key : x=%d, y=%d\n", lx, ly);
//			
//			if (ts->keypad == 0) {
//				printk("input report HOME key\n");
//				input_report_key(ts->input, KEY_HOME, 1);
//				ts->keypad = KEY_HOME;
//			}
//#endif 
		} else if ((lx > back_x) && (ly > back_y)) {
			if (msm_tsdebug & 1)
				printk("Back key : x=%d, y=%d\n", lx, ly);

			if (ts->keypad == 0) {
				printk("input report BACK key\n");
				input_report_key(ts->input, KEY_BACK, 1);
				ts->keypad = KEY_BACK;
				ts_key_event = 1;
			}	
		
		} else {
			if (PreRejectTouchCount > preRejectValue) {
				ts_update_pen_state(ts, lx, ly, 255);
			} else {
				PreRejectTouchCount++;
			}
		}
		
		/* kick pen up timer - to make sure it expires again(!) */
		mod_timer(&ts->timer, jiffies + msecs_to_jiffies(TS_PENUP_TIMEOUT_MS));

	} else
		printk(KERN_INFO "Ignored interrupt: {%3d, %3d},"
				" op = %3d samp = %3d\n",
				x, y, num_op, num_samp);

out:
		return IRQ_HANDLED;
}

static int __devinit ts_probe(struct platform_device *pdev)
{
	int result;
	struct input_dev *input_dev;
	struct resource *res, *ioarea;
	struct ts *ts;
	unsigned int x_max, y_max, pressure_max;
	struct msm_ts_platform_data *pdata = pdev->dev.platform_data;

	/* The primary initialization of the TS Hardware
	 * is taken care of by the ADC code on the modem side
	 */

	ts = kzalloc(sizeof(struct ts), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!input_dev || !ts) {
		result = -ENOMEM;
		goto fail_alloc_mem;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		result = -ENOENT;
		goto fail_alloc_mem;
	}

	ts->irq = platform_get_irq(pdev, 0);
	if (!ts->irq) {
		dev_err(&pdev->dev, "Could not get IORESOURCE_IRQ\n");
		result = -ENODEV;
		goto fail_alloc_mem;
	}

	ioarea = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "Could not allocate io region\n");
		result = -EBUSY;
		goto fail_alloc_mem;
	}

	virt = ioremap(res->start, resource_size(res));
	if (!virt) {
		dev_err(&pdev->dev, "Could not ioremap region\n");
		result = -ENOMEM;
		goto fail_ioremap;
	}

	input_dev->name = TS_DRIVER_NAME;
	input_dev->phys = "msm_touch/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	input_dev->absbit[BIT_WORD(ABS_MISC)] = BIT_MASK(ABS_MISC);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

//#if defined(CONFIG_MACH_MSM7X27_SWIFT_REV_1)
    set_bit(KEY_MENU, input_dev->keybit); 
//#else
  //  set_bit(KEY_HOME, input_dev->keybit); 
//#endif
	set_bit(KEY_BACK, input_dev->keybit);

    if (pdata) {
		x_max = pdata->x_max ? : X_MAX;
		y_max = pdata->y_max ? : Y_MAX;
		pressure_max = pdata->pressure_max ? : P_MAX;
	} else {
		x_max = X_MAX;
		y_max = Y_MAX;
		pressure_max = P_MAX;
	}

	ts->x_max = x_max;
	ts->y_max = y_max;
	ts->keypad = 0;

	input_set_abs_params(input_dev, ABS_X, X_MIN, X_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, Y_MIN, Y_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, pressure_max, 0, 0);

	result = input_register_device(input_dev);
	if (result)
		goto fail_ip_reg;

	ts->input = input_dev;
	ts->count = 0;

	setup_timer(&ts->timer, ts_timer, (unsigned long)ts);
	result = request_irq(ts->irq, ts_interrupt, IRQF_TRIGGER_RISING,
				 "touchscreen", ts);
	if (result)
		goto fail_req_irq;

	platform_set_drvdata(pdev, ts);

//#if defined(TS_KEY_CALMODE)
//	res = misc_register(&touch_cal_misc_device);
//	if (res) {
//		printk(KERN_ERR"heaven_motion_misc_device register failed\n");
//		goto fail_misc_device_register_failed;
//	}  
//
//	printk("[SWIFT TOUCH CAL-PROBE] MenuKey X: %d, MENUKey y: %d\n", menu_x, menu_y); 
//	printk("[SWIFT TOUCH CAL-PROBE] BACKKey X: %d, BACKKey y: %d\n", back_x, back_y); 
//#endif

	return 0;

fail_req_irq:
	input_unregister_device(input_dev);
	input_dev = NULL;
fail_ip_reg:
	iounmap(virt);
fail_ioremap:
	release_mem_region(res->start, resource_size(res));
fail_alloc_mem:
#if defined(TS_KEY_CALMODE)
fail_misc_device_register_failed:
#endif
	input_free_device(input_dev);
	kfree(ts);
	return result;
}

static int __devexit ts_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ts *ts = platform_get_drvdata(pdev);

	free_irq(ts->irq, ts);
	del_timer_sync(&ts->timer);

	input_unregister_device(ts->input);
	iounmap(virt);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));
	platform_set_drvdata(pdev, NULL);
	kfree(ts);

	return 0;
}

static struct platform_driver ts_driver = {
	.probe		= ts_probe,
	.remove		= __devexit_p(ts_remove),
	.driver		= {
		.name = TS_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};
 
static int __init ts_init(void)
{
	return platform_driver_register(&ts_driver);
}
module_init(ts_init);

static void __exit ts_exit(void)
{
	platform_driver_unregister(&ts_driver);
}
module_exit(ts_exit);

MODULE_DESCRIPTION("Swift Screen driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msm_touch");
