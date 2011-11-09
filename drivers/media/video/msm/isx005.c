/* drivers/media/video/msm/isx005.c 
*
* This software is for SONY 3M sensor 
*  
* Copyright (C) 2009-2011 LGE Inc.  
* Author: Hyungtae Lee <leehyungtae@lge.com>  
* (GPL License)  
* This software is licensed under the terms of the GNU General Public  
* License version 2, as published by the Free Software Foundation, and  
* may be copied, distributed, and modified under those terms.  
*  
* This program is distributed in the hope that it will be useful,  
* but WITHOUT ANY WARRANTY; without even the implied warranty of  
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the  
* GNU General Public License for more details. 
*/
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include "isx005.h"
#include <linux/byteorder/little_endian.h>
/* Sensor Core Registers */
#define  REG_ISX005_MODEL_ID 0x0000
#define  ISX005_MODEL_ID     0x0520

#define PREVIEW_MODE 0
#define CAPTURE_MODE 1

#define NOT_NIGHT_MODE 0
#define NIGHT_MODE 1

int current_scene = 0;
int previous_mode = 0;

int focus_mode = 0;
static int debug_mask = 1; 

module_param_named(debug_mask, debug_mask, int, S_IRUGO|S_IWUSR|S_IWGRP);
struct isx005_work {
	struct work_struct work;
};

static struct  isx005_work *isx005_sensorw;
static struct  i2c_client *isx005_client;

struct isx005_ctrl {
	const struct msm_camera_sensor_info *sensordata;
	int8_t sensormode;
	unsigned char qfactor;

   /* for Video Camera */
	int8_t effect;
	int8_t wb;
	int8_t scene;
	unsigned char brightness;
	int8_t af;
	int8_t zoom;
   /* to write register */
	int16_t write_byte;
	int16_t write_word;
};


static struct isx005_ctrl *isx005_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(isx005_wait_queue);
DECLARE_MUTEX(isx005_sem);


/*=============================================================
	EXTERNAL DECLARATIONS
==============================================================*/
extern struct isx005_reg isx005_regs;


/*=============================================================*/

static int32_t isx005_i2c_txdata(unsigned short saddr,
	unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	if (i2c_transfer(isx005_client->adapter, msg, 1) < 0 ) {
		printk("isx005_i2c_txdata failed\n");
		return -EIO;
	}

	return 0;
}

static int32_t isx005_i2c_write(unsigned short saddr,
	unsigned short waddr, unsigned short wdata, enum isx005_width width)
{
	int32_t rc = -EIO;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	switch (width) {
	case WORD_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = (wdata & 0xFF00)>>8;
		buf[3] = (wdata & 0x00FF);

		rc = isx005_i2c_txdata(saddr, buf, 4);
	}
		break;

	case BYTE_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = wdata;
		rc = isx005_i2c_txdata(saddr, buf, 3);
	}
		break;
	}

	if (rc < 0){
		printk("[isx005.c]%s: fail in i2c writing, addr = 0x%x, val = 0x%x!\n",__func__,waddr, wdata);
	}
	return rc;	
}

static int32_t isx005_i2c_write_table(
	struct isx005_i2c_reg_conf const *reg_conf_tbl,
	int num_of_items_in_table)
{
	int i = 0;
	int retry, f_count=0;
	int32_t rc = -EIO;
	
	for (i = 0; i < num_of_items_in_table; i++) {
		rc = isx005_i2c_write(isx005_client->addr,
		reg_conf_tbl->waddr, reg_conf_tbl->wdata,
		reg_conf_tbl->width);
		
		if (reg_conf_tbl->mdelay_time != 0)
			mdelay(reg_conf_tbl->mdelay_time);
     
		if(rc < 0){
    		for(retry = 0; retry < 3; retry++){
   				rc = isx005_i2c_write(isx005_client->addr,
		    			reg_conf_tbl->waddr, reg_conf_tbl->wdata,
		    			reg_conf_tbl->width);
           
            	if(rc >= 0)
               	retry = 3;
            
            	f_count = f_count + 1;
				if(debug_mask)  
					printk("[isx005.c]%s: the number of i2c failure = #%d\n",__func__,f_count);
	
         	}
         	reg_conf_tbl++;
		}else
         	reg_conf_tbl++;
	}
	return rc;
}

static int isx005_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
		.addr   = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(isx005_client->adapter, msgs, 2) < 0) {
		printk("[isx005.c]%s: fail in i2c transfer!\n", __func__);
		return -EIO;
	}

	return 0;
}

static int32_t isx005_i2c_read(unsigned short   saddr,
	unsigned short raddr, unsigned short *rdata, enum isx005_width width)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	switch (width) {
	case WORD_LEN: {
		buf[0] = (raddr & 0xFF00)>>8;
		buf[1] = (raddr & 0x00FF);

		rc = isx005_i2c_rxdata(saddr, buf, 2);
		if (rc < 0)
			return rc;

		*rdata = buf[0] << 8 | buf[1];
		}
		break;
   	
   case BYTE_LEN:{
		buf[0] = (raddr & 0xFF00)>>8;
		buf[1] = (raddr & 0x00FF);

		rc = isx005_i2c_rxdata(saddr, buf, 2);
		if (rc < 0)
			return rc;
		
		*rdata = buf[0];
		}
		break;           
	}

	if (rc < 0)
		printk("[isx005.c]%s: fail in i2c reading!\n",__func__);
   
	return rc;
}

void isx005_sensor_power_disable(void)
{
	int32_t rc = 0;
	struct vreg* vreg_rftx;
	struct vreg* vreg_rfrx2;
	struct vreg* vreg_msme2;
	struct vreg* vreg_wlan;

	printk("%s \n",__func__);

	vreg_wlan = vreg_get(NULL,"wlan");
    rc= vreg_disable(vreg_wlan);
	if(rc < 0)
		printk("isx005: VREG_CAM_AF: vreg_disabel() fail\n");
    mdelay(5);

	vreg_rftx = vreg_get(NULL,"rftx");
    rc= vreg_disable(vreg_rftx);
	if(rc < 0)
		printk("isx005: VREG_CAM_AVDD: vreg_disabel() fail\n");
	mdelay(5);

	vreg_rfrx2 = vreg_get(NULL,"rfrx2");
	rc= vreg_disable(vreg_rfrx2);
	if(rc < 0)
		printk("isx005: VREG_CAM_IOVDD: vreg_disabel() fail\n");
	mdelay(5);
	
	vreg_msme2 = vreg_get(NULL,"msme2");
	rc= vreg_disable(vreg_msme2);
	if(rc < 0)
		printk("isx005: VREG_CAM_DVDD: vreg_disabel() fail\n");
}

void isx005_sensor_power_enable(void)
{
	int32_t rc = 0;
	struct vreg* vreg_rftx;
	struct vreg* vreg_rfrx2;
	struct vreg* vreg_msme2;
	struct vreg* vreg_wlan;
	printk("%s \n",__func__);

	vreg_msme2 = vreg_get(NULL,"msme2");
	vreg_enable(vreg_msme2);
	rc = vreg_set_level(vreg_msme2,1200);
	if (rc < 0) {
		printk("isx005: camera power enable fail : msme2\n");
	}
	mdelay(5);

	vreg_rfrx2 = vreg_get(NULL,"rfrx2");
	vreg_enable(vreg_rfrx2);
	rc = vreg_set_level(vreg_rfrx2,2600);
	if (rc < 0) {
		printk("isx005: camera power enable fail : rfrx2\n");		
	}
	mdelay(5);

	vreg_rftx = vreg_get(NULL,"rftx");
	vreg_enable(vreg_rftx);
	rc = vreg_set_level(vreg_rftx,2700);
	if (rc < 0) {
		printk("isx005: camera power enable fail : rftx\n");
	}
	mdelay(5);

	vreg_wlan = vreg_get(NULL,"wlan");
	vreg_enable(vreg_wlan);
	  rc = vreg_set_level(vreg_wlan,2800);
	if (rc < 0) {
		printk("isx005: camera power enable fail : wlan\n");
	}

}
static int isx005_reset(const struct msm_camera_sensor_info *dev, int value)
{
	int rc = 0;
	
	rc = gpio_request(dev->sensor_reset, "isx005");
	if (!rc) 
		rc = gpio_direction_output(dev->sensor_reset, value);
	else{
		printk("[isx005.c]%s: fail in gpio_direction_output\n",__func__);
		return rc;
	}

	gpio_free(dev->sensor_reset);
	return rc;
}

static int isx005_pwdn(const struct msm_camera_sensor_info *dev, int value)
{
	int rc = 0;
	
	rc = gpio_request(dev->sensor_pwd, "isx005");
	if (!rc) 
		rc = gpio_direction_output(dev->sensor_pwd, value);
	else{
		printk("[isx005.c]%s: fail in gpio_direction_output\n",__func__);
		return rc;
	}

	gpio_free(dev->sensor_pwd);
	return rc;
}

static long isx005_snapshot_config(int mode, int width, int height)
{
	int32_t rc;
	unsigned short picture_width, picture_height;

		if(debug_mask)
	  printk("[isx005.c]%s: Input resolution[%d * %d]\n",__func__,width,height);

	picture_width = cpu_to_be16(width);
	picture_height = cpu_to_be16(height);

	rc = isx005_i2c_write(isx005_client->addr,
			0x0024, picture_width, WORD_LEN);
	if (rc < 0)
		return rc;
	
	rc = isx005_i2c_write(isx005_client->addr,
			0x002A, picture_height, WORD_LEN);
	if (rc < 0)
		return rc;
	
	rc = isx005_i2c_write(isx005_client->addr,
			0x00FC, 0xFF, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	rc = isx005_i2c_write(isx005_client->addr,
			0x0011, 0x02, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	isx005_ctrl->sensormode = mode;
	return 0;
}

static int32_t isx005_cancel_focus(int mode)
{
	int32_t rc;
	unsigned short return_pos=0;
	
	if(debug_mask)
		printk("[isx005.c]%s: mode = %d\n",__func__,mode);

	switch(mode){
	case AUTO_FOCUS:
		if(debug_mask)
	   		printk("[isx005.c] return to the infinity\n");

		return_pos = 0x3200;
		break;
	
	case MACRO_FOCUS:
		if(debug_mask)
	   		printk("[isx005.c] return to the macro\n");

		return_pos = 0x4402;
		break;
	}

	rc = isx005_i2c_write(isx005_client->addr,
			0x002E, 0x02, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	rc = isx005_i2c_write(isx005_client->addr,
			0x4852, return_pos, WORD_LEN);
	if (rc < 0)
		return rc;
	
	rc = isx005_i2c_write(isx005_client->addr,
			0x0012, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	rc = isx005_i2c_write(isx005_client->addr,
			0x4850, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	mdelay(60); // 1 frame delay to get lense move
	
	rc = isx005_i2c_write(isx005_client->addr,
			0x00FC, 0x1F, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	return rc;
}

static long isx005_set_sensor_mode(int mode,int width,int height)
{
	int32_t rc = 0;
		
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		if(debug_mask)
			printk("[isx005.c] sensor preview mode\n");
		
		mdelay(60); // 1 frame skip ==> total 2 frames skip
		if(previous_mode == CAPTURE_MODE){
			rc = isx005_i2c_write_table(&isx005_regs.prev_reg_settings[0],
				isx005_regs.prev_reg_settings_size);
			if(rc<0){
				printk("[isx005.c]%s: fail in writing for preview mode!\n",__func__);
				return rc;
			}
			rc = isx005_cancel_focus(focus_mode);
			if(rc<0)
				return rc;
			
			mdelay(100);  // 2 frames skip

			if(current_scene == NIGHT_MODE){
				if(debug_mask)
					printk("[isx005.c] current scene is NIGHT\n");
				
				mdelay(800);
		   }
			previous_mode = PREVIEW_MODE;
		}
	    break;

	case SENSOR_SNAPSHOT_MODE:
		if(debug_mask)
			printk("[isx005.c] sensor snapshot mode\n");
	
		rc = isx005_snapshot_config(mode,width,height);
		
		//mdelay(80);    // 1 frame skip
		
		if(current_scene == NIGHT_MODE){
			printk("[isx005.c] current scene is NIGHT\n");
			mdelay(500);
		}
		if(rc <0)
			return rc;
		previous_mode = CAPTURE_MODE;
		break;

	default:
		return -EINVAL;
	}
   
	return 0;
}

static int32_t isx005_check_af_lock_and_clear(void)
{
	int32_t rc;
	int i;
	unsigned short af_lock;
	
	/* check AF lock status */
	for(i=0; i<5; i++){
		rc = isx005_i2c_read(isx005_client->addr,
				0x00F8, &af_lock, BYTE_LEN);
		if (rc < 0){
			printk("[isx005.c]%s: fail in reading af_lock\n",__func__);
			return rc;
		}
	
		if((af_lock & 0x10)==0x10){
            if(debug_mask)
			    printk("[isx005.c] af_lock is ready to focus\n");
			break;
		}
		if(debug_mask)
			printk("[isx005.c] af_lock( 0x%x ) is not ready to focus yet\n", af_lock);
		mdelay(70);
	}	
	
	rc = isx005_i2c_write(isx005_client->addr,
			0x00FC, 0x10, BYTE_LEN);  // clear register
	if (rc < 0)
		return rc;
	
	/* check AF lock status */
	for(i=0; i<5; i++){
		rc = isx005_i2c_read(isx005_client->addr,
				0x00F8, &af_lock, BYTE_LEN);
		if (rc < 0){
			printk("[isx005.c]%s: fail in reading af_lock\n",__func__);
			return rc;
		}
	
		if((af_lock & 0x10)==0x00){
			printk("[isx005.c] af_lock has been cleared\n");
			break;
		}
		if(debug_mask)
			printk("[isx005.c] af_lock( 0x%x ) is not cleard state\n", af_lock);
		mdelay(70);
	}	

	return rc;
}

static int32_t isx005_check_focus(void)
{
	int32_t rc;
	int num = 0;
	unsigned short af_status, af_result_1st, af_result_2nd;
	
	while(num < 200){
		rc = isx005_i2c_read(isx005_client->addr,
        		0x6D76, &af_status, BYTE_LEN);
		if (rc < 0){
			printk("[isx005.c]%s: fail in reading af_status\n",__func__);
			return rc;
		}

		if(af_status == 8){
			if(debug_mask)
				printk("[isx005] af_status is lock done\n");
			break;
		}
		
		if(af_status != 8){
			if(debug_mask)
				printk("[isx005.c] af_status = %d, waiting untill af_status changes to be lock done \n", af_status);
		}
		mdelay(30);
		num = num + 1;
	}
	isx005_check_af_lock_and_clear();
	
	rc = isx005_i2c_read(isx005_client->addr,
        	0x6D3A, &af_result_1st, BYTE_LEN);
	if (rc < 0){
		printk("[isx005.c]%s: fai; in reading af_result_1st\n",__func__);
		return rc;
	}
	
	rc = isx005_i2c_read(isx005_client->addr,
        	0x6D52, &af_result_2nd, BYTE_LEN);
	if (rc < 0){
		printk("[isx005.c]%s: fail in reading af_result_2nd\n",__func__);
		return rc;
	}
	
	rc = isx005_i2c_write(isx005_client->addr,
			0x002E, 0x03, BYTE_LEN);
	if (rc < 0)
		return rc;

	rc = isx005_i2c_write(isx005_client->addr,
			0x0012, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	if(af_result_1st == 1 && af_result_2nd == 1){
		if(debug_mask)
			printk("[isx005.c] Good Focus !!\n");
		return 0;
	}
	
	if(debug_mask)
		printk("[isx005.c] time is over or af_result is bad\n");

	return -ETIME;
}

static int32_t isx005_set_focus(void)
{
	int32_t rc;
	
	rc = isx005_cancel_focus(focus_mode);
	if(rc<0){
		if(debug_mask)
			printk("[isx005.c]%s: fail in cancel_focus\n",__func__);
		return rc;
	}
		
	rc = isx005_i2c_write_table(&isx005_regs.AF_reg_settings[0],
		isx005_regs.AF_reg_settings_size);
	if(rc<0){
		printk("[isx005.c]%s: fail in writing for focus\n",__func__);
		return rc;
	}
	
	msleep(60); //1 frame skip
	
	rc = isx005_check_focus();

	return rc;
}

static int32_t isx005_set_auto_focus(void)
{
	int32_t rc;
	
	if(debug_mask)
		printk("[isx005.c] auto focus\n");

	rc = isx005_i2c_write_table(&isx005_regs.AF_nomal_reg_settings[0],
			isx005_regs.AF_nomal_reg_settings_size);
	if(rc<0){
		printk("[isx005.c]%s: fail in writing for AF\n",__func__);
		return rc;
	}
	
	return rc;
}

static int32_t isx005_set_macro_focus(void)
{
	int32_t rc;
	
	if(debug_mask)
		printk("[isx005.c] macro focus\n");

	rc = isx005_i2c_write_table(&isx005_regs.AF_macro_reg_settings[0],
			isx005_regs.AF_macro_reg_settings_size);
	if(rc<0){
		printk("[isx005.c]%s: fail in writing for macro\n",__func__);
		return rc;
	}
	
	return rc;
}

static int32_t isx005_focus_config(int mode)
{
	int32_t rc;
	int i;
	unsigned short af_driver_check, af_driver_status;
	
	/* check af driver */
	for(i=0; i<10; i++){
		rc = isx005_i2c_read(isx005_client->addr,
			0x000A, &af_driver_check, BYTE_LEN);
		if(debug_mask)
			printk(" 0x000A = %d\n", af_driver_check);
		
		rc = isx005_i2c_read(isx005_client->addr,
			0x6D76, &af_driver_status, BYTE_LEN);
		if(debug_mask)
			printk(" 0x6D76 = %d\n", af_driver_status);
    	if(af_driver_check == 2 && af_driver_status == 3){
			if(debug_mask)
				printk("[isx005] af_driver is already loaded\n");
			break;
		}else{
			if(af_driver_check != 2){
				
				rc = isx005_i2c_write(isx005_client->addr,
						0x000A, 0x00, BYTE_LEN);
				if (rc < 0){
					printk("[isx005.c]%s: fail in setting 0x000A\n",__func__);
					return rc;
				}
				rc = isx005_i2c_write_table(&isx005_regs.AF_driver_reg_settings[0],
						isx005_regs.AF_driver_reg_settings_size);
				if(rc<0){
					printk("[isx005.c]%s: fail in writing for AF driver\n",__func__);
					return rc;
				}
				
				rc = isx005_i2c_write(isx005_client->addr,
						0x000A, 0x01, BYTE_LEN);
				if (rc < 0){
					printk("[isx005.c]%s: fail in setting 0x000A\n",__func__);
					return rc;
				}
			}else{
				if(af_driver_status != 255){				

					/*
					rc = isx005_i2c_write(isx005_client->addr,
							0x000A, 0x01, BYTE_LEN);
					if (rc < 0){
						printk("[isx005.c]%s: fail in writing for AF driver\n",__func__);
						return rc;
					}
					*/
					mdelay(70); // 1 frame skip
				}else{
					
					rc = isx005_i2c_write(isx005_client->addr,
							0x000A, 0x00, BYTE_LEN);
					if (rc < 0){
						printk("[isx005.c]%s: fail in setting 0x000A\n",__func__);
						return rc;
					}
					rc = isx005_i2c_write_table(&isx005_regs.AF_driver_reg_settings[0],
							isx005_regs.AF_driver_reg_settings_size);
					if(rc<0){
						printk("[isx005.c]%s: fail in writing for AF driver\n",__func__);
						return rc;
					}
				
					rc = isx005_i2c_write(isx005_client->addr,
							0x000A, 0x01, BYTE_LEN);
					if (rc < 0){
						printk("[isx005.c]%s: fail in setting 0x000A\n",__func__);
						return rc;
					}	

				}	
			}	

		}				
		printk("[isx005.c] AF driver is not ready yet\n");
		mdelay(70); // 1 frame skip
	}
	switch(mode){
	case AUTO_FOCUS:
		rc = isx005_set_auto_focus();
		break;
	
	case MACRO_FOCUS:
		rc = isx005_set_macro_focus();
		break;
	
	default:
		return -EINVAL;
	}
	
	return rc;		
}

static int32_t isx005_move_focus(int32_t steps)
{
	int32_t rc;
	unsigned short cm_changed_sts, cm_changed_clr, af_pos, manual_pos;
	int i;
	
	if(debug_mask)
		printk("[isx005.c] move focus: step is %d\n",steps);
	
	/* MF ON */

	rc = isx005_i2c_write_table(&isx005_regs.manual_focus_reg_settings[0],
			isx005_regs.manual_focus_reg_settings_size);
	if(rc<0){
		printk("[isx005.c]%s: fail in writing for move focus\n",__func__);
		return rc;
	}

	/* check cm_changed_sts */
	for(i=0; i<4; i++){
		rc = isx005_i2c_read(isx005_client->addr,
				0x00F8, &cm_changed_sts, BYTE_LEN);
		if (rc < 0){
			printk("[isx005.c]%s; fail in reading cm_changed_sts\n",__func__);
			return rc;
		}
	
		if((cm_changed_sts & 0x02)==0x02){
			if(debug_mask)
				printk("[isx005.c] cm_changed_sts is done\n");
			break;
		}
		if(debug_mask)
			printk("[isx005.c] cm_changed_sts( 0x%x ) for MF is not ready yet\n", cm_changed_sts);
		
		mdelay(60); // 1 frame skip
	}	
	
	/* clear the interrupt register */
	rc = isx005_i2c_write(isx005_client->addr,
			0x00FC, 0x02, BYTE_LEN);
	if (rc < 0)
		return rc;

	/* check cm_changed_clr */
	for(i=0; i<4; i++){
		rc = isx005_i2c_read(isx005_client->addr,
				0x00FC, &cm_changed_sts, BYTE_LEN);
		if (rc < 0){
			printk("[isx005.c]%s: fail in reading cm_changed_clr\n",__func__);
			return rc;
		}
	
		if((cm_changed_clr & 0x00)==0x00){
			if(debug_mask)
				printk("[isx005.c] cm_changed_clr is cleared\n");
			break;
		}
		if(debug_mask)
			printk("isx005: cm_changed_clr( 0x%x ) for MF is not ready yet\n", cm_changed_clr);
		
		mdelay(60); // 1 frame skip
	}
		
	manual_pos = cpu_to_be16( 50 + (26 * steps) );
	if(debug_mask)
		printk("[isx005.c] manual position is 0x%x\n", manual_pos);

	rc = isx005_i2c_write(isx005_client->addr,
			0x4852, manual_pos, WORD_LEN);
	if (rc < 0)
		return rc;

	rc = isx005_i2c_write(isx005_client->addr,
			0x4850, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	rc = isx005_i2c_write(isx005_client->addr,
			0x0015, 0x01, BYTE_LEN);
	if (rc < 0)
		return rc;
	
	isx005_check_af_lock_and_clear();
	
	/* check lens position */
	for(i=0; i<4; i++){
		rc = isx005_i2c_read(isx005_client->addr,
				0x6D7A, &af_pos, WORD_LEN);
		if (rc < 0){
			printk("[isx005.c]%s: fail in reading af_lenspos\n",__func__);
		}
	
		if(af_pos == manual_pos){
            if(debug_mask)
                printk("[isx005.c] af_pos is equal with manual_pos\n");
			break;
		}
		
		if(debug_mask)
			printk("[isx005.c] lens position(0x%x) is not ready yet\n",af_pos);
		
		mdelay(60); // 1 frame skip
	}	
	return rc;
}

static int32_t isx005_set_effect(int effect)
{
	int32_t rc;

	switch (effect) {
	case CAMERA_EFFECT_OFF: 
		if(debug_mask)
			printk("[isx005.c] effect is OFF\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x005F, 0x00, BYTE_LEN);
		if (rc < 0)
			return rc;
	
		rc = isx005_i2c_write(isx005_client->addr,
				0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;
	
      break;

	case CAMERA_EFFECT_MONO: 
		if(debug_mask)
			printk("[isx005.c] effect is MONO\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x005F, 0x04, BYTE_LEN);
		if (rc < 0)
			return rc;
		rc = isx005_i2c_write(isx005_client->addr,
				0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;
		break;

   case CAMERA_EFFECT_SEPIA: 
		if(debug_mask)
			printk("[isx005.c] effect is SEPIA\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x005F, 0x03, BYTE_LEN);
		if (rc < 0)			
			return rc;
		rc = isx005_i2c_write(isx005_client->addr,
				0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;
		break;

	case CAMERA_EFFECT_NEGATIVE: 
		if(debug_mask)
			printk("[isx005.c] effect is NAGATIVE\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x005F, 0x02, BYTE_LEN);
		if (rc < 0)
			return rc;
		rc = isx005_i2c_write(isx005_client->addr,
				0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;
		break;

   case CAMERA_EFFECT_NEGATIVE_SEPIA:
		if(debug_mask)
			printk("[isx005.c] effect is NAGATIVE_SEPIA\n");

		rc = isx005_i2c_write(isx005_client->addr,
				0x005F, 0x02, BYTE_LEN);
		if (rc < 0)
			return rc;
		rc = isx005_i2c_write(isx005_client->addr,
				0x038A, 0x6D11, WORD_LEN);
		if (rc < 0)
			return rc;
		break;

   case CAMERA_EFFECT_BLUE:
		if(debug_mask)
			printk("[isx005.c] effect is BLUE\n");

		rc = isx005_i2c_write(isx005_client->addr,
			0x005F, 0x03, BYTE_LEN);
		if (rc < 0)
			return rc;
		rc = isx005_i2c_write(isx005_client->addr,
				0x038A, 0x6D11, WORD_LEN);
		if (rc < 0)
			return rc;
		break;

   case CAMERA_EFFECT_SOLARIZE: 
		if(debug_mask)
			printk("[isx005.c] effect is SOLARIZE\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x005F, 0x01, BYTE_LEN);
		if (rc < 0)
			return rc;
		rc = isx005_i2c_write(isx005_client->addr,
				0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;
		break;

   case CAMERA_EFFECT_PASTEL: 
		if(debug_mask)
			printk("[isx005.c] effect is PASTEL\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x005F, 0x05, BYTE_LEN);
		if (rc < 0)
			return rc;
		rc = isx005_i2c_write(isx005_client->addr,
				0x038A, 0x6911, WORD_LEN);
		if (rc < 0)
			return rc;
		break;

   default: 
		printk("[isx005.c] incorrect effect mode\n");
		return -EINVAL;	
	}
	
	return 0;
}

static int32_t isx005_set_zoom(int8_t zoom)
{
	int32_t rc;
		
	switch (zoom){
	case 0:
		if(debug_mask)
			printk("[isx005.c] zoom is X1.0\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0x0001, WORD_LEN);
		break;

	case 1:
		if(debug_mask)
			printk("[isx005.c] zoom is X1.15\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0x2601, WORD_LEN);
		break;		
	
	case 2:
		if(debug_mask)
			printk("[isx005.c] zoom is X1.3\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0x4C01, WORD_LEN);
		break;	
		
	case 3:
		if(debug_mask)
			printk("[isx005.c] zoom is X1.45\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0x7301, WORD_LEN);
		break;
		
	case 4:
		if(debug_mask)
			printk("[isx005.c] zoom is X1.6\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0x9901, WORD_LEN);
		break;
		
	case 5:
		if(debug_mask)
			printk("[isx005.c] zoom is X1.75\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0xC001, WORD_LEN);
		break;
		
	case 6:
		if(debug_mask)
			printk("[isx005.c] zoom is X1.9\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0xE601, WORD_LEN);
		break;
		
	case 7:
		if(debug_mask)
			printk("[isx005.c] zoom is X2.05\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0x0C02, WORD_LEN);
		break;
		
	case 8:
		if(debug_mask)
			printk("[isx005.c] zoom is X2.2\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0x3302, WORD_LEN);
		break;
		
	case 9:
		if(debug_mask)
			printk("[isx005.c]  zoom is X2.35\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0x5902, WORD_LEN);
		break;
		
	case 10:
		if(debug_mask)
			printk("[isx005.c] zoom is X2.5\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0x8002, WORD_LEN);
		break;
		
	case 11:
		if(debug_mask)
			printk("[isx005.c] zoom is X2.65\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0xA602, WORD_LEN);
		break;
		
	case 12:
		if(debug_mask)
			printk("[isx005.c] zoom is X2.8\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0xCC02, WORD_LEN);
		break;
		
	case 13:
		if(debug_mask)
			printk("[isx005.c] zoom is X2.95\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0xF302, WORD_LEN);
		break;
	
	case 14:
		if(debug_mask)
			printk("[isx005.c] zoom is X3.1\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0x1903, WORD_LEN);
		break;	
		
	case 15:
		if(debug_mask)
			printk("[isx005.c] zoom is X3.2\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0032, 0x3303, WORD_LEN);
		break;		
	
	default:
		printk("[isx005.c] incorrect zoom value\n");
		return -EFAULT;
	}
	
	isx005_ctrl->zoom = zoom;
	return rc;
}

static int32_t isx005_set_wb(int8_t wb)
{
	int32_t rc;
   
	switch (wb){
	case CAMERA_WB_AUTO:
		if(debug_mask)
			printk("[isx005.c] wb is AUTO\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x4453, 0x7B, BYTE_LEN);
		if (rc < 0)
			return rc;
		rc = isx005_i2c_write(isx005_client->addr,
				0x0102, 0x20, BYTE_LEN);
		if (rc < 0)
			return rc;
		break;

	case CAMERA_WB_INCANDESCENT:
		if(debug_mask)
			printk("[isx005.c] wb is INCANDESCENT\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x4453,0x3B, BYTE_LEN);
		if (rc < 0)
			return rc;
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0102,0x08, BYTE_LEN);
		if (rc < 0)
			return rc;
		break;
	
	case CAMERA_WB_SUNNY:
		if(debug_mask)
			printk("[isx005.c] wb is SUNNY\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x4453,0x3B, BYTE_LEN);
		if (rc < 0)
			return rc;
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0102,0x04, BYTE_LEN);
		if (rc < 0)
			return rc;
		break;

	case CAMERA_WB_FLUORESCENT:
		if(debug_mask)
			printk("[isx005.c] wb is FLUORESCENT\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x4453,0x3B, BYTE_LEN);
		if (rc < 0)
			return rc;
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0102,0x07, BYTE_LEN);
		if (rc < 0)
			return rc;
		break;

	case CAMERA_WB_CLOUDY:
		if(debug_mask)
			printk("[isx005.c] wb is CLOUDY\n");
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x4453, 0x3B, BYTE_LEN);
		if (rc < 0)
			return rc;

		rc = isx005_i2c_write(isx005_client->addr,
				0x0102,0x06, BYTE_LEN);
		if (rc < 0)
			return rc;
		break;

	default:
		printk("[isx005.c] incorrect white balance value\n");
		return -EFAULT;
	}
	isx005_ctrl->wb = wb;

	return 0;
}

static int32_t isx005_set_iso_indoor_50hz(int8_t iso){
	
	int32_t rc;
	
	if(debug_mask)
			printk("[isx005.c] here is 50 Hz indoor\n");
			
	switch (iso){
	case CAMERA_ISO_AUTO:
		if(debug_mask)
				printk("[isx005.c] iso is AUTO\n");
		
		rc = isx005_i2c_write_table(&isx005_regs.iso_auto_indoor_reg_settings[0],
					isx005_regs.iso_auto_indoor_reg_settings_size);

		break;                                                    
                                                            
	case CAMERA_ISO_100:
		if(debug_mask)
			printk("[isx005.c] iso is 100\n");
			
		rc = isx005_i2c_write_table(&isx005_regs.iso_100_indoor_reg_settings[0],
          isx005_regs.iso_100_indoor_reg_settings_size);

		break;

	case CAMERA_ISO_200:
		if(debug_mask)
				printk("[isx005.c] iso is 200\n");
				
		rc = isx005_i2c_write_table(&isx005_regs.iso_200_indoor_reg_settings[0],
                isx005_regs.iso_200_indoor_reg_settings_size);

		break;

	case CAMERA_ISO_400:
		if(debug_mask)
				printk("[isx005.c] iso is 400\n");
			
		rc = isx005_i2c_write_table(&isx005_regs.iso_400_indoor_reg_settings[0],
          isx005_regs.iso_400_indoor_reg_settings_size);
	
		break;
   
	default:
		printk("[isx005.c] incorrect iso value\n");
		rc = -EINVAL;
	}
	return rc;
}

static int32_t isx005_set_iso_indoor_60hz(int8_t iso){
	
	int32_t rc;
	
	if(debug_mask)
			printk("[isx005.c] here is 60 Hz indoor\n");
			
	switch (iso){
	case CAMERA_ISO_AUTO:
		if(debug_mask)
				printk("[isx005.c] iso is AUTO\n");
		
		rc = isx005_i2c_write_table(&isx005_regs.iso_auto_indoor_60hz_reg_settings[0],
					isx005_regs.iso_auto_indoor_60hz_reg_settings_size);

		break;                                                    
                                                            
	case CAMERA_ISO_100:
		if(debug_mask)
			printk("[isx005.c] iso is 100\n");
			
		rc = isx005_i2c_write_table(&isx005_regs.iso_100_indoor_60hz_reg_settings[0],
          isx005_regs.iso_100_indoor_60hz_reg_settings_size);

		break;

	case CAMERA_ISO_200:
		if(debug_mask)
				printk("[isx005.c] iso is 200\n");
				
		rc = isx005_i2c_write_table(&isx005_regs.iso_200_indoor_60hz_reg_settings[0],
                isx005_regs.iso_200_indoor_60hz_reg_settings_size);

		break;

	case CAMERA_ISO_400:
		if(debug_mask)
				printk("[isx005.c] iso is 400\n");
			
		rc = isx005_i2c_write_table(&isx005_regs.iso_400_indoor_60hz_reg_settings[0],
          isx005_regs.iso_400_indoor_60hz_reg_settings_size);
	
		break;
   
	default:
		printk("[isx005.c] incorrect iso value\n");
		rc = -EINVAL;
	}
	return rc;
}

static int32_t isx005_set_iso(int8_t iso)
{
	int32_t rc;
	
	unsigned short outdoor_f, d_flicker, d_hz;
	
	rc = isx005_i2c_read(isx005_client->addr,
        	0x6C21, &outdoor_f, BYTE_LEN);
	if (rc < 0){
		printk("[isx005.c]%s: fail in reading outdoor_f\n",__func__);
		return rc;
	}
	
	if(outdoor_f == 0){  // indoor
		
		if(debug_mask)
			printk("[isx005.c] here is indoor\n");
			
		rc = isx005_i2c_read(isx005_client->addr,
        	0x6C1B, &d_flicker, BYTE_LEN);
		if (rc < 0){
			printk("[isx005.c]%s: fail in reading d_flicker\n",__func__);
			return rc;
		}
		if (d_flicker == 0){
			
			rc = isx005_set_iso_indoor_50hz(iso);
			if (rc < 0){
				printk("[isx005.c]%s: fail in setting 50Hz\n",__func__);
				return rc;
			}	
		
		}else{
			
			rc = isx005_i2c_read(isx005_client->addr,
        	0x6C1C, &d_hz, BYTE_LEN);
			if (rc < 0){
				printk("[isx005.c]%s: fail in reading d_hz\n",__func__);
				return rc;
			}
			
			if(d_hz == 0){
				rc = isx005_set_iso_indoor_60hz(iso);
				if (rc < 0){
					printk("[isx005.c]%s: fail in setting 60Hz\n",__func__);
					return rc;
				}
			}else{
				rc = isx005_set_iso_indoor_50hz(iso);
				if (rc < 0){
					printk("[isx005.c]%s: fail in setting 50Hz\n",__func__);
					return rc;
				}
			}
		}
	}else{  // outdoor
		
		if(debug_mask)
			printk("[isx005.c] outdoor\n");
		
		switch (iso){
		case CAMERA_ISO_AUTO:
			if(debug_mask)
				printk("[isx005.c] iso is AUTO\n");
		
			rc = isx005_i2c_write_table(&isx005_regs.iso_auto_outdoor_reg_settings[0],
                isx005_regs.iso_auto_outdoor_reg_settings_size);

			break;                                                    
                                                            
		case CAMERA_ISO_100:
			if(debug_mask)
				printk("[isx005.c] iso is 100\n");
			
			rc = isx005_i2c_write_table(&isx005_regs.iso_100_outdoor_reg_settings[0],
                isx005_regs.iso_100_outdoor_reg_settings_size);

			break;

		case CAMERA_ISO_200:
			if(debug_mask)
				printk("[isx005.c] iso is 200\n");
			
			rc = isx005_i2c_write_table(&isx005_regs.iso_200_outdoor_reg_settings[0],
                isx005_regs.iso_200_outdoor_reg_settings_size);

			break;

		case CAMERA_ISO_400:
			if(debug_mask)
				printk("[isx005.c] iso is 400\n");
			
			rc = isx005_i2c_write_table(&isx005_regs.iso_400_outdoor_reg_settings[0],
                isx005_regs.iso_400_outdoor_reg_settings_size);
	
			break;
   
		default:
			printk("[isx005.c] incorrect iso value\n");
			rc = -EINVAL;
		}
	}
		
	return rc;
}

static int32_t isx005_set_scene_mode(int8_t mode)
{
	int32_t rc = 0;

	current_scene = NOT_NIGHT_MODE;
	switch (mode) {
	case CAMERA_SCENE_NORMAL:
		if(debug_mask)
			printk("[isx005.c] scene mode is normal\n");
			
		rc = isx005_i2c_write_table(&isx005_regs.scene_normal_reg_settings[0],
                isx005_regs.scene_normal_reg_settings_size);
		break;
	
	case CAMERA_SCENE_PORTRAIT:
		if(debug_mask)
			printk("[isx005.c] scene mode is portrait\n");
		
		rc = isx005_i2c_write_table(&isx005_regs.scene_portrait_reg_settings[0],
                isx005_regs.scene_portrait_reg_settings_size);
     	break;
	
	case CAMERA_SCENE_LANDSCAPE:
		if(debug_mask)
			printk("[isx005.c] scene mode is landscape\n");
	
		rc = isx005_i2c_write_table(&isx005_regs.scene_landscape_reg_settings[0],
                isx005_regs.scene_landscape_reg_settings_size);
		break;
	
	case CAMERA_SCENE_SPORT:
		if(debug_mask)
			printk("[isx005.c] scene mode is sport\n");
	
		rc = isx005_i2c_write_table(&isx005_regs.scene_sport_reg_settings[0],
                isx005_regs.scene_sport_reg_settings_size);
		break;
	
	case CAMERA_SCENE_SUNSET:
		if(debug_mask)
			printk("[isx005.c] scene mode is sunset\n");
	
		rc = isx005_i2c_write_table(&isx005_regs.scene_sunset_reg_settings[0],
                isx005_regs.scene_sunset_reg_settings_size);
		break;
	
	case CAMERA_SCENE_NIGHT:
		if(debug_mask)
			printk("[isx005.c] scene mode is night\n");
	
		rc = isx005_i2c_write_table(&isx005_regs.scene_night_reg_settings[0],
                isx005_regs.scene_night_reg_settings_size);
		
		current_scene = NIGHT_MODE;
		break;
	
	default:
		printk("[isx005.c] incorrect  scene mode value\n");
	}   

	isx005_ctrl->scene = mode;
	return rc;
   
}

static int32_t isx005_set_brightness(int8_t ev)
{
	int32_t rc=0;
	
	switch (ev) {
	case 1:
		rc = isx005_i2c_write(isx005_client->addr,
				0x0060, 0x80, BYTE_LEN);
		if(rc<0)
			return rc;
		
		rc = isx005_i2c_write(isx005_client->addr,
				0x0061, 0x50, BYTE_LEN);
		if(rc<0)
			return rc;
		
		break;

	case 2:
		rc = isx005_i2c_write(isx005_client->addr,
				0x0060, 0x80, BYTE_LEN);
		if(rc<0)
			return rc;	
	
		rc = isx005_i2c_write(isx005_client->addr,
				0x0061, 0x60, BYTE_LEN);
		if(rc<0)
			return rc;	

		break;

	case 3:
		rc = isx005_i2c_write(isx005_client->addr,
				0x0060, 0x80, BYTE_LEN);
		if(rc<0)
			return rc;	
	
		rc = isx005_i2c_write(isx005_client->addr,
				0x0061, 0x70, BYTE_LEN);
		if(rc<0)
			return rc;	

		break;

	case 4:
		rc = isx005_i2c_write(isx005_client->addr,
				0x0060, 0xCD, BYTE_LEN);
		if(rc<0)
			return rc;	

		rc = isx005_i2c_write(isx005_client->addr,
				0x0061, 0x80, BYTE_LEN);
		if(rc<0)
		return rc;	

		break;

	case 5:
		rc = isx005_i2c_write(isx005_client->addr,
				0x0060, 0xEF, BYTE_LEN);
		if(rc<0)
			return rc;	

		rc = isx005_i2c_write(isx005_client->addr,
				0x0061, 0x80, BYTE_LEN);
		if(rc<0)
			return rc;	

		break;

	case 6:
		rc = isx005_i2c_write(isx005_client->addr,
				0x0060, 0x00, BYTE_LEN);
		if(rc<0)
			return rc;	
	
		rc = isx005_i2c_write(isx005_client->addr,
				0x0061, 0x80, BYTE_LEN);
		if(rc<0)
			return rc;	

		break;

	case 7:
		rc = isx005_i2c_write(isx005_client->addr,
				0x0060, 0x18, BYTE_LEN);
		if(rc<0)
			return rc;	

		rc = isx005_i2c_write(isx005_client->addr,
				0x0061, 0x80, BYTE_LEN);
		if(rc<0)
			return rc;	

		break;

	case 8:
		rc = isx005_i2c_write(isx005_client->addr,
				0x0060, 0x7F, BYTE_LEN);
		if(rc<0)
			return rc;	

		rc = isx005_i2c_write(isx005_client->addr,
				0x0061, 0x8A, BYTE_LEN);
		if(rc<0)
			return rc;	

		break;

	case 9:
		rc = isx005_i2c_write(isx005_client->addr,
				0x0060, 0x7F, BYTE_LEN);
		if(rc<0)
			return rc;	

		rc = isx005_i2c_write(isx005_client->addr,
				0x0061, 0x9C, BYTE_LEN);
		if(rc<0)
			return rc;	

		break;

	case 10:
		rc = isx005_i2c_write(isx005_client->addr,
				0x0060, 0x7F, BYTE_LEN);
		if(rc<0)
			return rc;	

		rc = isx005_i2c_write(isx005_client->addr,
				0x0061, 0xAA, BYTE_LEN);
		if(rc<0)
			return rc;	

		break;

	case 11:
		rc = isx005_i2c_write(isx005_client->addr,
				0x0060, 0x7F, BYTE_LEN);
		if(rc<0)
			return rc;	

		rc = isx005_i2c_write(isx005_client->addr,
				0x0061, 0xC8, BYTE_LEN);
		if(rc<0)
			return rc;	

		break;
	
	default:
		printk("[isx005.c] incoreect brightness value\n");
	}
	
	isx005_ctrl->brightness = ev;
	return rc;
}

static int32_t isx005_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	int32_t  rc;
	unsigned short model_id;

	/* Read the Model ID of the sensor */
	rc = isx005_i2c_read(isx005_client->addr,
		REG_ISX005_MODEL_ID, &model_id, WORD_LEN);
	if (rc < 0)
		goto init_probe_fail;

	printk("[isx005.c] model_id = 0x%x\n", model_id);
	/* Check if it matches it with the value in Datasheet */
	if (model_id != ISX005_MODEL_ID) {
		rc = -EINVAL;
		goto init_probe_fail;
	}

	return rc;
   
init_probe_fail:
    return rc;
}

int32_t isx005_sensor_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc;
	isx005_sensor_power_enable();
	isx005_ctrl = kzalloc(sizeof(struct isx005_ctrl), GFP_KERNEL);
	if (!isx005_ctrl) {
		printk("[isx005.c] fail in sensor initialization\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		isx005_ctrl->sensordata = data;
		
	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	msm_camio_camif_pad_reg_reset();
	mdelay(5);
	

	rc = isx005_reset(data,1);
	if (rc < 0) {
		printk("[isx005.c]%s: reset fail\n",__func__);
		goto init_fail;
	}
    mdelay(5);
    rc = isx005_pwdn(data,1);
	if (rc < 0) {
		printk("[isx005.c]%s: pwdn fail\n",__func__);
		goto init_fail;
    }

    mdelay(8);  // T2
	
	rc = isx005_i2c_write_table(&isx005_regs.pll[0],
			isx005_regs.pll_size);
	if(rc<0){
	   printk("[isx005.c]%s: fail in writing pll\n",__func__);
	   goto init_fail; 
	}
	mdelay(16);  // T3+T4
   
	rc = isx005_i2c_write_table(&isx005_regs.init[0],
            isx005_regs.init_size);
	if(rc<0){
		printk("[isx005.c]%s: fail in writing init_table\n",__func__);
		goto init_fail; 
	}

init_done:
	return rc;

init_fail:
	printk("[isx005.c]%s: init fail\n",__func__);
	kfree(isx005_ctrl);
	return rc;
}

static int isx005_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&isx005_wait_queue);
	return 0;
}

int isx005_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	int32_t  rc=0;
	 
	if (copy_from_user(&cfg_data,(void *)argp,sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	if(debug_mask)
		printk("[isx005.c] isx005_ioctl, cfgtype = %d, mode = %d width= height %d \n",
		       cfg_data.cfgtype, cfg_data.mode,cfg_data.width,cfg_data.height);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		if(debug_mask)
			printk("[isx005.c]%s: command is CFG_SET_MODE\n",__func__);
			
		rc = isx005_set_sensor_mode(cfg_data.mode,cfg_data.width,cfg_data.height);
		break;

	case CFG_SET_EFFECT:
		if(debug_mask)
			printk("stud [isx005.c]%s: command is CFG_SET_EFFECT\n",__func__);
			
		rc = isx005_set_effect(cfg_data.cfg.effect);

		break;
  
  case CFG_SET_ZOOM_VIDEO:
		if(debug_mask)
			printk("stud [isx005.c]%s: command is CFG_SET_ZOOM\n",__func__);
			
		rc = isx005_set_zoom(cfg_data.cfg.zoom);

		break;   
	
	case CFG_SET_PARM_AF_MODE:
		if(debug_mask)
			printk(" stud [isx005.c]%s: command is CFG_SET_PARM_AF_MODE\n",__func__);
		focus_mode = cfg_data.cfg.focus.mode;
		rc = isx005_focus_config(cfg_data.cfg.focus.mode);
		
		break;

	case CFG_SET_DEFAULT_FOCUS:
		if(debug_mask)
			printk("[isx005.c]%s: command is CFG_SET_DEFAULT_FOCUS\n",__func__);
		   
		rc = isx005_set_focus();
		break;

	case CFG_MOVE_FOCUS:
		
		if(debug_mask)
			printk("[isx005.c]%s: command is CFG_MOVE_FOCUS: steps=%d\n",
		    			__func__,cfg_data.cfg.focus.steps);
		rc = isx005_move_focus(cfg_data.cfg.focus.steps);
		break;	

	case CFG_SET_CANCEL_FOCUS:
		if(debug_mask)
			printk("[isx005.c]%s: command is CFG_SET_CANCEL_FOCUS\n",__func__);
		   
		break;
		 	
	case CFG_GET_AF_MAX_STEPS:
		cfg_data.max_steps = 20;
		if (copy_to_user((void *)argp,&cfg_data, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
		
	case CFG_SET_WB:
		if(debug_mask)
			printk("[isx005.c]%s: command is CFG_SET_WB\n",__func__);
		
			
		rc = isx005_set_wb(cfg_data.cfg.wb);
		break;

	case CFG_SET_ISO:
		if(debug_mask)
			printk("[isx005.c]%s: command is CFG_SET_ISO\n",__func__);
		
		rc = isx005_set_iso(cfg_data.cfg.iso);
		break;

	case CFG_SET_SCENE_MODE:
		if(debug_mask)
			printk("[isx005.c]%s: command is CFG_SET_SCENE_MODE\n",__func__);
	       
		rc = isx005_set_scene_mode(cfg_data.cfg.scene_mode);
		break;
	case CFG_SET_BRIGHTNESS:
		if(debug_mask)
			printk("stud - [isx005.c]%s: command is CFG_SET_BRIGHTNESS\n",__func__);
 
		rc = isx005_set_brightness(cfg_data.cfg.ev);
		break;
	default:
		rc = -EFAULT;
	}
	
	if (rc < 0)
		printk("[isx005.c] ERROR in sensor_config, %d\n", rc);
	
	return rc; 	
}

/* =====================================================================================*/
/* isx005 sysf                                                                          */
/* =====================================================================================*/

static ssize_t isx005_write_byte_store(struct device* dev, struct device_attribute* attr,const char* buf, size_t n)
{
	unsigned int val;
	unsigned short waddr, wdata;
	long rc;

	if (isx005_ctrl == NULL)
		return 0;

	sscanf(buf,"%x",&val);
	waddr=(val & 0xffff00)>>8;
	wdata=(val & 0x0000ff);

	rc = isx005_i2c_write(isx005_client->addr, waddr, wdata, BYTE_LEN);
	if (rc < 0)
		printk("[isx005.c]%s: fail in writing byte register\n",__func__);

	return n;
}

static DEVICE_ATTR(write_byte, S_IRUGO|S_IWUGO, NULL, isx005_write_byte_store);

static ssize_t isx005_write_word_store(struct device* dev, struct device_attribute* attr,const char* buf, size_t n)
{
	unsigned int val;
	unsigned short waddr, wdata;
	long rc;

	if (isx005_ctrl == NULL)
		return 0;

	sscanf(buf,"%x",&val);
	waddr=(val & 0xffff0000)>>16;
	wdata=(val & 0x0000ffff);

	rc = isx005_i2c_write(isx005_client->addr, waddr, wdata, WORD_LEN);
	if (rc < 0)
		printk("[isx005.c]%s: fail in writing word register\n",__func__);

	return n;
}

static DEVICE_ATTR(write_word, S_IRUGO|S_IWUGO, NULL, isx005_write_word_store);

static ssize_t isx005_af_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
	int val;
	long rc;

	if (isx005_ctrl == NULL)
		return 0;

	sscanf(buf,"%d",&val);

	rc = isx005_focus_config(val);
	if (rc < 0)
		printk("[isx005.c]%s: fail in setting autofocus\n",__func__);

	return n;
}

static DEVICE_ATTR(af, S_IRUGO|S_IWUGO, NULL, isx005_af_store);

static ssize_t isx005_move_focus_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
	int val;
	long rc;

	if (isx005_ctrl == NULL)
		return 0;

	sscanf(buf,"%d",&val);

	rc = isx005_move_focus(val);
	if (rc < 0)
		printk("[isx005.c]%s: fail in setting move focus\n",__func__);

	return n;
}

static DEVICE_ATTR(mf, S_IRUGO|S_IWUGO, NULL, isx005_move_focus_store);

static ssize_t isx005_cancel_focus_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
	int val;
	long rc;

	if (isx005_ctrl == NULL)
		return 0;

	sscanf(buf,"%d",&val);

	rc = isx005_cancel_focus(val);
	if (rc < 0)
		printk("[isx005.c]%s: fail in setting cancelfocus\n",__func__);

	return n;
}

static DEVICE_ATTR(cf, S_IRUGO|S_IWUGO, NULL, isx005_cancel_focus_store);

static ssize_t isx005_zoom_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
	int val;
	long rc;

	if (isx005_ctrl == NULL)
		return 0;

	sscanf(buf,"%d",&val);

	rc = isx005_set_zoom(val);
	if (rc < 0)
		printk("[isx005.c]%s: fail in setting zoom\n",__func__);

	return n;
}

static DEVICE_ATTR(zoom, S_IRUGO|S_IWUGO, NULL, isx005_zoom_store);

static ssize_t isx005_brightness_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
	int val;
	long rc;

	if (isx005_ctrl == NULL)
		return 0;

	sscanf(buf,"%d",&val);

	rc = isx005_set_brightness(val);
	if (rc < 0)
		printk("[isx005.c]%s: fail in setting brightness\n",__func__);

	return n;
}

static DEVICE_ATTR(brightness, S_IRUGO|S_IWUGO, NULL, isx005_brightness_store);

static ssize_t isx005_scene_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
	int val;
	long rc;

	if (isx005_ctrl == NULL)
		return 0;

	sscanf(buf,"%x",&val);

	if(val < CAMERA_SCENE_NORMAL || val > CAMERA_SCENE_MAX) {
		printk("[isx005.c] invalid scene mode input\n");
		return 0;
	}

	rc = isx005_set_scene_mode(val);
	if (rc < 0)
		printk("[isx005]%s: fail in setting scene mode\n",__func__);

	return n;
}
static DEVICE_ATTR(scene, S_IRUGO|S_IWUGO, NULL, isx005_scene_store);

static ssize_t isx005_wb_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
	int val;
	long rc;

	if (isx005_ctrl == NULL)
		return 0;

	sscanf(buf,"%x",&val);

	if(val < CAMERA_WB_MIN || val > CAMERA_WB_MAX) {
		printk("[isx005.c] invalid white balance input\n");
		return 0;
	}

	rc = isx005_set_wb(val);
	if (rc < 0)
		printk("[isx005.c]%s: fail in setting white balance\n",__func__);

	return n;
}
static DEVICE_ATTR(wb, S_IRUGO|S_IWUGO, NULL, isx005_wb_store);

static ssize_t isx005_effect_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
	int val;
	long rc;

	if (isx005_ctrl == NULL)
		return 0;

	sscanf(buf,"%x",&val);

	if(val < CAMERA_EFFECT_OFF || val > CAMERA_EFFECT_MAX) {
		printk("[isx005.c] invalid effect input\n");
		return 0;
	}

	rc = isx005_set_effect(val);
	if (rc < 0)
		printk("[isx005.c]%s: fail in setting effect\n",__func__);

	return n;
}
static DEVICE_ATTR(effect, S_IRUGO|S_IWUGO, NULL, isx005_effect_store);

static struct attribute* isx005_sysfs_attrs[] = {
	&dev_attr_write_byte.attr,
	&dev_attr_write_word.attr,
	&dev_attr_af.attr,
	&dev_attr_mf.attr,
	&dev_attr_cf.attr,
	&dev_attr_effect.attr,
	&dev_attr_wb.attr,
	&dev_attr_scene.attr,
	&dev_attr_zoom.attr,
	&dev_attr_brightness.attr,  
	NULL
};

static void isx005_sysfs_add(struct kobject* kobj)
{
	int i, n, ret;
	n = ARRAY_SIZE(isx005_sysfs_attrs);
	for(i = 0; i < n; i++){
		if(isx005_sysfs_attrs[i]){
			ret = sysfs_create_file(kobj, isx005_sysfs_attrs[i]);
			if(ret < 0)
				printk("[isx005.c]%s: sysfs is not created\n",__func__);
		}
	}
}

/*======================================================================================*/
/*  end :  sysf                                                                         */
/*======================================================================================*/

int isx005_sensor_release(const struct msm_camera_sensor_info *info)
{
	if(isx005_ctrl)
		kfree(isx005_ctrl);
	isx005_ctrl=NULL;
	
	return 0;
}

static int isx005_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	isx005_sensorw =
		kzalloc(sizeof(struct isx005_work), GFP_KERNEL);

	if (!isx005_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, isx005_sensorw);
	isx005_init_client(client);
	isx005_client = client;
	
	isx005_sysfs_add(&client->dev.kobj);

	if(debug_mask)
		printk("[isx005.c] succeed in probing i2c\n");

	return 0;
	
probe_failure:
	kfree(isx005_sensorw);
	isx005_sensorw = NULL;
	printk("[isx005.c]%s: fail in probing i2c\n",__func__);
	return rc;
}

static const struct i2c_device_id isx005_i2c_id[] = {
	{ "isx005", 0},
	{ },
};

static struct i2c_driver isx005_i2c_driver = {
	.id_table = isx005_i2c_id,
	.probe  = isx005_i2c_probe,
	.remove = __exit_p(isx005_i2c_remove),
	.driver = {
		.name = "isx005",
	},
};
MODULE_DEVICE_TABLE(i2c, isx005_i2c_id);

static int isx005_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	isx005_sensor_power_enable();
	int rc = i2c_add_driver(&isx005_i2c_driver);
	
	if (rc < 0 || isx005_client == NULL) {
		rc = -ENOTSUPP;
		printk("[isx005.c]%s: fail in i2c add driver \n",__func__);
		goto probe_done;
	}
		
  	 /* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	rc = isx005_reset(info,1);
	if (rc < 0) {
		printk("[isx005.c]%s: fail in reset\n",__func__);
	}
	mdelay(5);

	rc = isx005_pwdn(info,1);
	if (rc < 0) {
		printk("[isx005.c]%s: fail in pwdn\n",__func__);
	}

	mdelay(10);

	rc = isx005_sensor_init_probe(info);
	if (rc < 0)
	{	
		printk("[isx005.c]%s: fail in sensor init porobe\n",__func__);
		goto probe_done;
	}
	mdelay(10);
		
	rc = isx005_pwdn(info,0);
	if (rc < 0) {
		printk("isx005: pwdn failed!\n");
	}
	mdelay(5);
	
	rc = isx005_reset(info,0);
	if (rc < 0) {
		printk("isx005: reset failed!\n");
	}

	s->s_init = isx005_sensor_init;
	s->s_release = isx005_sensor_release;
	s->s_config  = isx005_sensor_config;
probe_done:
	printk("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __isx005_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, isx005_sensor_probe);	
}

static struct platform_driver msm_camera_driver = {
	.probe = __isx005_probe,
	.driver = {
		.name = "msm_camera_isx005",
		.owner = THIS_MODULE,
	},
};
static int __init isx005_init(void)
{
	return platform_driver_register(&msm_camera_driver);  
}

module_init(isx005_init);
