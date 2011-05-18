/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __QCOM_WLAN_PWRIF_H__
#define __QCOM_WLAN_PWRIF_H__

/*
 * Headers for WLAN Power Interface Functions
 */
#include <linux/err.h>
#include <mach/mpp.h>
#include <linux/device.h>
#include <mach/vreg.h>
#include <linux/delay.h>
#include <linux/mfd/pmic8058.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <mach/msm_xo.h>
#include <asm/mach-types.h>
#include <mach/rpm-regulator.h>

#define CHIP_POWER_ON         1
#define CHIP_POWER_OFF        0

int vos_chip_power_qrf8615(int on);
int qcomwlan_pmic_xo_core_force_enable(int on);
int qcomwlan_freq_change_1p3v_supply(enum rpm_vreg_freq freq);

#endif /* __QCOM_WLAN_PWRIF_H__ */
