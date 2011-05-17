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

#ifndef __ARCH_ARM_MACH_MSM_MPM_H
#define __ARCH_ARM_MACH_MSM_MPM_H

#include <linux/types.h>
#include <linux/list.h>

enum msm_mpm_pin {
	MSM_MPM_PIN_SDC3_DAT1 = 21,
	MSM_MPM_PIN_SDC3_DAT3 = 22,
	MSM_MPM_PIN_SDC4_DAT1 = 23,
	MSM_MPM_PIN_SDC4_DAT3 = 24,
};

#define MSM_MPM_NR_MPM_IRQS  64

struct msm_mpm_device_data {
	uint16_t *irqs_m2a;
	unsigned int irqs_m2a_size;
	uint16_t *bypassed_apps_irqs;
	unsigned int bypassed_apps_irqs_size;
	void __iomem *mpm_request_reg_base;
	void __iomem *mpm_status_reg_base;
	void __iomem *mpm_apps_ipc_reg;
	unsigned int mpm_apps_ipc_val;
	unsigned int mpm_ipc_irq;
};

#ifdef CONFIG_MSM_MPM
extern struct msm_mpm_device_data msm_mpm_dev_data;

int msm_mpm_enable_irq(unsigned int irq, unsigned int enable);
int msm_mpm_set_irq_wake(unsigned int irq, unsigned int on);
int msm_mpm_set_irq_type(unsigned int irq, unsigned int flow_type);
int msm_mpm_enable_pin(enum msm_mpm_pin pin, unsigned int enable);
int msm_mpm_set_pin_wake(enum msm_mpm_pin pin, unsigned int on);
int msm_mpm_set_pin_type(enum msm_mpm_pin pin, unsigned int flow_type);
bool msm_mpm_irqs_detectable(bool from_idle);
bool msm_mpm_gpio_irqs_detectable(bool from_idle);
void msm_mpm_enter_sleep(bool from_idle);
void msm_mpm_exit_sleep(bool from_idle);
#else

int msm_mpm_enable_irq(unsigned int irq, unsigned int enable)
{ return -ENODEV; }
int msm_mpm_set_irq_wake(unsigned int irq, unsigned int on)
{ return -ENODEV; }
int msm_mpm_set_irq_type(unsigned int irq, unsigned int flow_type)
{ return -ENODEV; }
int msm_mpm_enable_pin(enum msm_mpm_pin pin, unsigned int enable)
{ return -ENODEV; }
int msm_mpm_set_pin_wake(enum msm_mpm_pin pin, unsigned int on)
{ return -ENODEV; }
int msm_mpm_set_pin_type(enum msm_mpm_pin pin, unsigned int flow_type)
{ return -ENODEV; }
bool msm_mpm_irqs_detectable(bool from_idle)
{ return false; }
bool msm_mpm_gpio_irqs_detectable(bool from_idle)
{ return false; }
void msm_mpm_enter_sleep(bool from_idle) {}
void msm_mpm_exit_sleep(bool from_idle) {}
#endif



#endif /* __ARCH_ARM_MACH_MSM_MPM_H */
