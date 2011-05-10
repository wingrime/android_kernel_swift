/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef __PMIC8058_MISC_H__
#define __PMIC8058_MISC_H__

enum pm8058_vib_en_mode {
	PM8058_VIB_MANUAL,
	PM8058_VIB_DTEST1,
	PM8058_VIB_DTEST2,
	PM8058_VIB_DTEST3
};

struct pm8058_vib_config {
	u16 drive_mV;
	u8 active_low;
	enum pm8058_vib_en_mode enable_mode;
};

int pm8058_vibrator_config(struct pm8058_vib_config *vib_config);

#endif /* __PMIC8058_MISC_H__ */
