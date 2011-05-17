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

#ifndef __ARCH_ARM_MACH_MSM_RPM_H
#define __ARCH_ARM_MACH_MSM_RPM_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/semaphore.h>

#if defined(CONFIG_ARCH_MSM8X60)
#include <mach/rpm-8660.h>
#elif defined(CONFIG_ARCH_MSM8960)
#include <mach/rpm-8960.h>
#endif


enum {
	MSM_RPM_PAGE_STATUS,
	MSM_RPM_PAGE_CTRL,
	MSM_RPM_PAGE_REQ,
	MSM_RPM_PAGE_ACK,
	MSM_RPM_PAGE_COUNT
};

enum {
	MSM_RPM_CTX_SET_0,
	MSM_RPM_CTX_SET_SLEEP,
	MSM_RPM_CTX_SET_COUNT,

	MSM_RPM_CTX_NOTIFICATION = 30,
	MSM_RPM_CTX_REJECTED = 31,
};

static inline uint32_t msm_rpm_get_ctx_mask(unsigned int ctx)
{
	return 1UL << ctx;
}

#define MSM_RPM_SEL_MASK_SIZE  (MSM_RPM_SEL_LAST / 32 + 1)

static inline unsigned int msm_rpm_get_sel_mask_reg(unsigned int sel)
{
	return sel / 32;
}

static inline uint32_t msm_rpm_get_sel_mask(unsigned int sel)
{
	return 1UL << (sel % 32);
}

struct msm_rpm_iv_pair {
	uint32_t id;
	uint32_t value;
};

struct msm_rpm_notification {
	struct list_head list;  /* reserved for RPM use */
	struct semaphore sem;
	uint32_t sel_masks[MSM_RPM_SEL_MASK_SIZE];  /* reserved for RPM use */
};

struct msm_rpm_map_data {
	uint32_t id;
	uint32_t sel;
	uint32_t count;
};

#define MSM_RPM_MAP(i, s, c) { \
	.id = MSM_RPM_ID_##i, .sel = MSM_RPM_SEL_##s, .count = c }


struct msm_rpm_platform_data {
	void __iomem *reg_base_addrs[MSM_RPM_PAGE_COUNT];

	unsigned int irq_ack;
	unsigned int irq_err;
	unsigned int irq_vmpm;
	void *msm_apps_ipc_rpm_reg;
	unsigned int msm_apps_ipc_rpm_val;
};

extern struct msm_rpm_map_data rpm_map_data[];
extern unsigned int rpm_map_data_size;

int msm_rpm_local_request_is_outstanding(void);
int msm_rpm_get_status(struct msm_rpm_iv_pair *status, int count);
int msm_rpm_set(int ctx, struct msm_rpm_iv_pair *req, int count);
int msm_rpm_set_noirq(int ctx, struct msm_rpm_iv_pair *req, int count);

static inline int msm_rpm_set_nosleep(
	int ctx, struct msm_rpm_iv_pair *req, int count)
{
	unsigned long flags;
	int rc;

	local_irq_save(flags);
	rc = msm_rpm_set_noirq(ctx, req, count);
	local_irq_restore(flags);

	return rc;
}

int msm_rpm_clear(int ctx, struct msm_rpm_iv_pair *req, int count);
int msm_rpm_clear_noirq(int ctx, struct msm_rpm_iv_pair *req, int count);

static inline int msm_rpm_clear_nosleep(
	int ctx, struct msm_rpm_iv_pair *req, int count)
{
	unsigned long flags;
	int rc;

	local_irq_save(flags);
	rc = msm_rpm_clear_noirq(ctx, req, count);
	local_irq_restore(flags);

	return rc;
}

int msm_rpm_register_notification(struct msm_rpm_notification *n,
	struct msm_rpm_iv_pair *req, int count);
int msm_rpm_unregister_notification(struct msm_rpm_notification *n);
int msm_rpm_init(struct msm_rpm_platform_data *data);

#endif /* __ARCH_ARM_MACH_MSM_RPM_H */
