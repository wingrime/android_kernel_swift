// SUNGWOO_EDBAE_CAMERA ADDED [
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


#ifndef ISX005_H
#define ISX005_H

#include <linux/types.h>
#include <mach/camera.h>

extern struct isx005_reg isx005_regs;

enum isx005_width {
	WORD_LEN,
	BYTE_LEN
};

struct isx005_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
	enum isx005_width width;
	unsigned short mdelay_time;
};

struct isx005_reg {
	const struct isx005_i2c_reg_conf *pll;
	uint16_t pll_size;
	const struct isx005_i2c_reg_conf *init;
	uint16_t init_size;
	
	const struct isx005_i2c_reg_conf *AF_driver_reg_settings;
	uint16_t AF_driver_reg_settings_size;
	
	/*register for return to preview mode from snapshot mode*/
	const struct isx005_i2c_reg_conf *prev_reg_settings;
	uint16_t prev_reg_settings_size;
	/*register for scene*/
	const struct isx005_i2c_reg_conf *scene_normal_reg_settings;
	uint16_t scene_normal_reg_settings_size;	
	const struct isx005_i2c_reg_conf *scene_portrait_reg_settings;
	uint16_t scene_portrait_reg_settings_size;	
	const struct isx005_i2c_reg_conf *scene_landscape_reg_settings;
	uint16_t scene_landscape_reg_settings_size;
	const struct isx005_i2c_reg_conf *scene_sport_reg_settings;
	uint16_t scene_sport_reg_settings_size;
	const struct isx005_i2c_reg_conf *scene_sunset_reg_settings;
	uint16_t scene_sunset_reg_settings_size;
	const struct isx005_i2c_reg_conf *scene_night_reg_settings;
	uint16_t scene_night_reg_settings_size;
	/*register for AF*/
	const struct isx005_i2c_reg_conf *AF_reg_settings;
	uint16_t AF_reg_settings_size;
	const struct isx005_i2c_reg_conf *AF_nomal_reg_settings;
	uint16_t AF_nomal_reg_settings_size;
	const struct isx005_i2c_reg_conf *AF_macro_reg_settings;
	uint16_t AF_macro_reg_settings_size;

	const struct isx005_i2c_reg_conf *manual_focus_reg_settings;
	uint16_t manual_focus_reg_settings_size;

	/*register for iso indoor 50Hz*/
	const struct isx005_i2c_reg_conf *iso_auto_indoor_reg_settings;
	uint16_t iso_auto_indoor_reg_settings_size;
	const struct isx005_i2c_reg_conf *iso_100_indoor_reg_settings;
	uint16_t iso_100_indoor_reg_settings_size;
	const struct isx005_i2c_reg_conf *iso_200_indoor_reg_settings;
	uint16_t iso_200_indoor_reg_settings_size;
	const struct isx005_i2c_reg_conf *iso_400_indoor_reg_settings;
	uint16_t iso_400_indoor_reg_settings_size;
	/*register for iso indoor 60Hz*/
	const struct isx005_i2c_reg_conf *iso_auto_indoor_60hz_reg_settings;
	uint16_t iso_auto_indoor_60hz_reg_settings_size;
	const struct isx005_i2c_reg_conf *iso_100_indoor_60hz_reg_settings;
	uint16_t iso_100_indoor_60hz_reg_settings_size;
	const struct isx005_i2c_reg_conf *iso_200_indoor_60hz_reg_settings;
	uint16_t iso_200_indoor_60hz_reg_settings_size;
	const struct isx005_i2c_reg_conf *iso_400_indoor_60hz_reg_settings;
	uint16_t iso_400_indoor_60hz_reg_settings_size;
	/*register for iso outdoor*/
	const struct isx005_i2c_reg_conf *iso_auto_outdoor_reg_settings;
	uint16_t iso_auto_outdoor_reg_settings_size;
	const struct isx005_i2c_reg_conf *iso_100_outdoor_reg_settings;
	uint16_t iso_100_outdoor_reg_settings_size;
	const struct isx005_i2c_reg_conf *iso_200_outdoor_reg_settings;
	uint16_t iso_200_outdoor_reg_settings_size;
	const struct isx005_i2c_reg_conf *iso_400_outdoor_reg_settings;
	uint16_t iso_400_outdoor_reg_settings_size;

/*Special modes */
/*Auto focus mode */
#define AUTO_FOCUS  0
#define MACRO_FOCUS 1
/*Special effects */
#define	 CAMERA_EFFECT_PASTEL 10
#define	 CAMERA_EFFECT_NEGATIVE_SEPIA 11
#define	 CAMERA_EFFECT_BLUE 12
/*white balance */
#define	 CAMERA_WB_MIN 0 
#define	 CAMERA_WB_AUTO 1
#define	 CAMERA_WB_INCANDESCENT 2
#define	 CAMERA_WB_SUNNY 3
#define	 CAMERA_WB_FLUORESCENT 4 
#define	 CAMERA_WB_CLOUDY 5
#define	 CAMERA_WB_CUSTOM 6
#define	 CAMERA_WB_DAYLIGHT 7
#define	 CAMERA_WB_CLOUDY_DAYLIGHT 8 
#define	 CAMERA_WB_TWILIGHT 9
#define	 CAMERA_WB_SHADE 10
#define	 CAMERA_WB_MAX 11
/*ISO MODES*/
#define	CAMERA_ISO_AUTO 0 
#define	CAMERA_ISO_100 1 
#define	CAMERA_ISO_200 2
#define	CAMERA_ISO_400 3
#define	CAMERA_ISO_800 4
#define	CAMERA_ISO_DEBLUR 5
#define	CAMERA_ISO_MAX 6
/*Scene modes */
#define	CAMERA_SCENE_NORMAL 0
#define	CAMERA_SCENE_PORTRAIT 1
#define	CAMERA_SCENE_LANDSCAPE 2
#define	CAMERA_SCENE_SPORT 3
#define	CAMERA_SCENE_SUNSET 4
#define	CAMERA_SCENE_NIGHT 5
#define	CAMERA_SCENE_BACKLIGHT 6
#define	CAMERA_SCENE_NIGHT_PORTRAIT 7
#define	CAMERA_SCENE_BEACH 8
#define	CAMERA_SCENE_PARTY 9
#define	CAMERA_SCENE_MAX 10
/*cfg*/
#define CFG_SET_AF_PARAM_INIT	128
#define CFG_SET_AF_START		129
#define CFG_GET_AF_STATUS		130
#define CFG_SET_ZOOM_VIDEO 38
#define CFG_SET_PARM_AF_MODE 36
#define CFG_SET_ISO 28
#define CFG_SET_SCENE_MODE 29
#define CFG_SET_CANCEL_FOCUS 35

};
#endif 