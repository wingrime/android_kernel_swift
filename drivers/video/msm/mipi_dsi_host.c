
/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <asm/system.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/clk.h>
#include <mach/dma.h>

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mdp.h"
#include "mdp4.h"

static struct completion dsi_dma_comp;
static struct dsi_buf dsi_tx_buf;
static int dsi_irq_enabled;
static spinlock_t dsi_lock;

static struct list_head pre_kickoff_list;
static struct list_head post_kickoff_list;

void mipi_dsi_init(void)
{
	init_completion(&dsi_dma_comp);
	mipi_dsi_buf_alloc(&dsi_tx_buf, DSI_BUF_SIZE);
	spin_lock_init(&dsi_lock);

	INIT_LIST_HEAD(&pre_kickoff_list);
	INIT_LIST_HEAD(&post_kickoff_list);
}

void mipi_dsi_enable_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&dsi_lock, flags);
	if (dsi_irq_enabled) {
		pr_debug("%s: IRQ aleady enabled\n", __func__);
		spin_unlock_irqrestore(&dsi_lock, flags);
		return;
	}
	dsi_irq_enabled = 1;
	enable_irq(DSI_IRQ);
	spin_unlock_irqrestore(&dsi_lock, flags);
}

void mipi_dsi_disable_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&dsi_lock, flags);
	if (dsi_irq_enabled == 0) {
		pr_debug("%s: IRQ already disabled\n", __func__);
		spin_unlock_irqrestore(&dsi_lock, flags);
		return;
	}

	dsi_irq_enabled = 0;
	disable_irq(DSI_IRQ);
	spin_unlock_irqrestore(&dsi_lock, flags);
}

/*
 * mipi_dsi_disale_irq_nosync() should be called
 * from interrupt context
 */
 void mipi_dsi_disable_irq_nosync(void)
{
	spin_lock(&dsi_lock);
	if (dsi_irq_enabled == 0) {
		pr_debug("%s: IRQ cannot be disabled\n", __func__);
		return;
	}

	dsi_irq_enabled = 0;
	disable_irq_nosync(DSI_IRQ);
	spin_unlock(&dsi_lock);
}

static void mipi_dsi_action(struct list_head *act_list)
{
	struct list_head *lp;
	struct dsi_kickoff_action *act;

	list_for_each(lp, act_list) {
		act = list_entry(lp, struct dsi_kickoff_action, act_entry);
		if (act && act->action)
			act->action(act->data);
	}
}

void mipi_dsi_pre_kickoff_action(void)
{
	mipi_dsi_action(&pre_kickoff_list);
}

void mipi_dsi_post_kickoff_action(void)
{
	mipi_dsi_action(&post_kickoff_list);
}

/*
 * mipi_dsi_pre_kickoff_add:
 * ov_mutex need to be acquired before call this function.
 */
void mipi_dsi_pre_kickoff_add(struct dsi_kickoff_action *act)
{
	if (act)
		list_add_tail(&act->act_entry, &pre_kickoff_list);
}

/*
 * mipi_dsi_pre_kickoff_add:
 * ov_mutex need to be acquired before call this function.
 */
void mipi_dsi_post_kickoff_add(struct dsi_kickoff_action *act)
{
	if (act)
		list_add_tail(&act->act_entry, &post_kickoff_list);
}

/*
 * mipi_dsi_pre_kickoff_add:
 * ov_mutex need to be acquired before call this function.
 */
void mipi_dsi_pre_kickoff_del(struct dsi_kickoff_action *act)
{
	if (!list_empty(&pre_kickoff_list) && act)
		list_del(&act->act_entry);
}

/*
 * mipi_dsi_pre_kickoff_add:
 * ov_mutex need to be acquired before call this function.
 */
void mipi_dsi_post_kickoff_del(struct dsi_kickoff_action *act)
{
	if (!list_empty(&post_kickoff_list) && act)
		list_del(&act->act_entry);
}

/*
 * mipi dsi buf mechanism
 */
char *mipi_dsi_buf_reserve(struct dsi_buf *dp, int len)
{
	dp->data += len;
	return dp->data;
}

char *mipi_dsi_buf_unreserve(struct dsi_buf *dp, int len)
{
	dp->data -= len;
	return dp->data;
}

char *mipi_dsi_buf_push(struct dsi_buf *dp, int len)
{
	dp->data -= len;
	dp->len += len;
	return dp->data;
}

char *mipi_dsi_buf_reserve_hdr(struct dsi_buf *dp, int hlen)
{
	dp->hdr = (uint32 *)dp->data;
	return mipi_dsi_buf_reserve(dp, hlen);
}

char *mipi_dsi_buf_init(struct dsi_buf *dp)
{
	int off;

	dp->data = dp->start;
	off = (int)dp->data;
	/* 8 byte align */
	off &= 0x07;
	if (off)
		off = 8 - off;
	dp->data += off;
	dp->len = 0;
	return dp->data;
}

int mipi_dsi_buf_alloc(struct dsi_buf *dp, int size)
{

	dp->start = kmalloc(size, GFP_KERNEL);
	if (dp->start == NULL) {
		pr_err("%s:%u\n", __func__, __LINE__);
		return -ENOMEM;
	}

	dp->end = dp->start + size;
	dp->size = size;

	if ((int)dp->start & 0x07)
		pr_err("%s: buf NOT 8 bytes aligned\n", __func__);

	dp->data = dp->start;
	dp->len = 0;
	return size;
}

/*
 * mipi dsi gerneric long write
 */
static int mipi_dsi_generic_lwrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	char *bp;
	uint32 *hp;
	int i, len;

	bp = mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);

	/* fill up payload */
	if (cm->payload) {
		len = cm->dlen;
		len += 3;
		len &= ~0x03;	/* multipled by 4 */
		for (i = 0; i < cm->dlen; i++)
			*bp++ = cm->payload[i];

		/* append 0xff to the end */
		for (; i < len; i++)
			*bp++ = 0xff;

		dp->len += len;
	}

	/* fill up header */
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(cm->dlen);
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_DTYPE(DTYPE_GEN_LWRITE);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;
}

/*
 * mipi dsi gerneric short write with 0, 1 2 parameters
 */
static int mipi_dsi_generic_swrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;
	int len;

	if (cm->dlen && cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return 0;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	if (cm->last)
		*hp |= DSI_HDR_LAST;


	len = (cm->dlen > 2) ? 2 : cm->dlen;

	if (len == 1) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_WRITE1);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(0);
	} else if (len == 2) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_WRITE2);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(cm->payload[1]);
	} else {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_WRITE);
		*hp |= DSI_HDR_DATA1(0);
		*hp |= DSI_HDR_DATA2(0);
	}

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

/*
 * mipi dsi gerneric read with 0, 1 2 parameters
 */
static int mipi_dsi_generic_read(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;
	int len;

	if (cm->dlen && cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return 0;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_BTA;
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	len = (cm->dlen > 2) ? 2 : cm->dlen;

	if (len == 1) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_READ1);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(0);
	} else if (len == 2) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_READ2);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(cm->payload[1]);
	} else {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_READ);
		*hp |= DSI_HDR_DATA1(0);
		*hp |= DSI_HDR_DATA2(0);
	}

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return dp->len;	/* 4 bytes */
}

/*
 * mipi dsi dcs long write
 */
static int mipi_dsi_dcs_lwrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	char *bp;
	uint32 *hp;
	int i, len;

	bp = mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);

	/*
	 * fill up payload
	 * dcs command byte (first byte) followed by payload
	 */
	if (cm->payload) {
		len = cm->dlen;
		len += 3;
		len &= ~0x03;	/* multipled by 4 */
		for (i = 0; i < cm->dlen; i++)
			*bp++ = cm->payload[i];

		/* append 0xff to the end */
		for (; i < len; i++)
			*bp++ = 0xff;

		dp->len += len;
	}

	/* fill up header */
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(cm->dlen);
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_LWRITE);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;
}

/*
 * mipi dsi dcs short write with 0 parameters
 */
static int mipi_dsi_dcs_swrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;
	int len;

	if (cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return -EINVAL;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	if (cm->ack)		/* ask ACK trigger msg from peripeheral */
		*hp |= DSI_HDR_BTA;
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	len = (cm->dlen > 1) ? 1 : cm->dlen;

	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_WRITE);
	*hp |= DSI_HDR_DATA1(cm->payload[0]);	/* dcs command byte */
	*hp |= DSI_HDR_DATA2(0);

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return dp->len;
}

/*
 * mipi dsi dcs short write with 1 parameters
 */
static int mipi_dsi_dcs_swrite1(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	if (cm->dlen < 2 || cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return -EINVAL;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	if (cm->ack)		/* ask ACK trigger msg from peripeheral */
		*hp |= DSI_HDR_BTA;
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_WRITE1);
	*hp |= DSI_HDR_DATA1(cm->payload[0]);	/* dcs comamnd byte */
	*hp |= DSI_HDR_DATA2(cm->payload[1]);	/* parameter */

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;
}
/*
 * mipi dsi dcs read with 0 parameters
 */

static int mipi_dsi_dcs_read(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	if (cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return -EINVAL;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_BTA;
	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_READ);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	*hp |= DSI_HDR_DATA1(cm->payload[0]);	/* dcs command byte */
	*hp |= DSI_HDR_DATA2(0);

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_cm_on(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_CM_ON);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_cm_off(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_CM_OFF);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_peripheral_on(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_PERIPHERAL_ON);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_peripheral_off(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_PERIPHERAL_OFF);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_set_max_pktsize(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	if (cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return 0;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_MAX_PKTSIZE);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	*hp |= DSI_HDR_DATA1(cm->payload[0]);
	*hp |= DSI_HDR_DATA2(cm->payload[1]);

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_null_pkt(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(cm->dlen);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_NULL_PKT);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

static int mipi_dsi_blank_pkt(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(cm->dlen);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_BLANK_PKT);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	/* 4 bytes */
}

/*
 * prepare cmd buffer to be txed
 */
int mipi_dsi_cmd_dma_add(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	int len = 0;

	switch (cm->dtype) {
	case DTYPE_GEN_WRITE:
	case DTYPE_GEN_WRITE1:
	case DTYPE_GEN_WRITE2:
		len = mipi_dsi_generic_swrite(dp, cm);
		break;
	case DTYPE_GEN_LWRITE:
		len = mipi_dsi_generic_lwrite(dp, cm);
		break;
	case DTYPE_GEN_READ:
	case DTYPE_GEN_READ1:
	case DTYPE_GEN_READ2:
		len = mipi_dsi_generic_read(dp, cm);
		break;
	case DTYPE_DCS_LWRITE:
		len = mipi_dsi_dcs_lwrite(dp, cm);
		break;
	case DTYPE_DCS_WRITE:
		len = mipi_dsi_dcs_swrite(dp, cm);
		break;
	case DTYPE_DCS_WRITE1:
		len = mipi_dsi_dcs_swrite1(dp, cm);
		break;
	case DTYPE_DCS_READ:
		len = mipi_dsi_dcs_read(dp, cm);
		break;
	case DTYPE_MAX_PKTSIZE:
		len = mipi_dsi_set_max_pktsize(dp, cm);
		break;
	case DTYPE_NULL_PKT:
		len = mipi_dsi_null_pkt(dp, cm);
		break;
	case DTYPE_BLANK_PKT:
		len = mipi_dsi_blank_pkt(dp, cm);
		break;
	case DTYPE_CM_ON:
		len = mipi_dsi_cm_on(dp, cm);
		break;
	case DTYPE_CM_OFF:
		len = mipi_dsi_cm_off(dp, cm);
		break;
	case DTYPE_PERIPHERAL_ON:
		len = mipi_dsi_peripheral_on(dp, cm);
		break;
	case DTYPE_PERIPHERAL_OFF:
		len = mipi_dsi_peripheral_off(dp, cm);
		break;
	default:
		pr_debug("%s: dtype=%x NOT supported\n",
					__func__, cm->dtype);
		break;

	}

	return len;
}

void mipi_dsi_host_init(struct mipi_panel_info *pinfo)
{
	uint32 dsi_ctrl, intr_ctrl;
	uint32 data;

	if (pinfo->mode == DSI_VIDEO_MODE) {
		data = 0;
		if (pinfo->pulse_mode_hsa_he)
			data |= BIT(28);
		if (pinfo->hfp_power_stop)
			data |= BIT(24);
		if (pinfo->hbp_power_stop)
			data |= BIT(20);
		if (pinfo->hsa_power_stop)
			data |= BIT(16);
		if (pinfo->eof_bllp_power_stop)
			data |= BIT(15);
		if (pinfo->bllp_power_stop)
			data |= BIT(12);
		data |= ((pinfo->traffic_mode & 0x03) << 8);
		data |= ((pinfo->dst_format & 0x03) << 4); /* 2 bits */
		data |= (pinfo->vc & 0x03);
		MIPI_OUTP(MIPI_DSI_BASE + 0x000c, data);

		data = 0;
		data |= ((pinfo->rgb_swap & 0x07) << 12);
		if (pinfo->b_sel)
			data |= BIT(8);
		if (pinfo->g_sel)
			data |= BIT(4);
		if (pinfo->r_sel)
			data |= BIT(0);
		MIPI_OUTP(MIPI_DSI_BASE + 0x001c, data);
	} else if (pinfo->mode == DSI_CMD_MODE) {
		data = 0;
		data |= ((pinfo->interleave_max & 0x0f) << 20);
		data |= ((pinfo->rgb_swap & 0x07) << 16);
		if (pinfo->b_sel)
			data |= BIT(12);
		if (pinfo->g_sel)
			data |= BIT(8);
		if (pinfo->r_sel)
			data |= BIT(4);
		data |= (pinfo->dst_format & 0x0f);	/* 4 bits */
		MIPI_OUTP(MIPI_DSI_BASE + 0x003c, data);

		/* DSI_COMMAND_MODE_MDP_DCS_CMD_CTRL */
		data = pinfo->wr_mem_continue & 0x0ff;
		data <<= 8;
		data |= (pinfo->wr_mem_start & 0x0ff);
		if (pinfo->insert_dcs_cmd)
			data |= BIT(16);
		MIPI_OUTP(MIPI_DSI_BASE + 0x0040, data);
	} else
		pr_err("%s: Unknown DSI mode=%d\n", __func__, pinfo->mode);

	dsi_ctrl = BIT(8) | BIT(2);	/* clock enable & cmd mode */
	intr_ctrl = 0;
	intr_ctrl = (DSI_INTR_CMD_DMA_DONE_MASK | DSI_INTR_CMD_MDP_DONE_MASK);

	if (pinfo->crc_check)
		dsi_ctrl |= BIT(24);
	if (pinfo->ecc_check)
		dsi_ctrl |= BIT(20);
	if (pinfo->data_lane3)
		dsi_ctrl |= BIT(7);
	if (pinfo->data_lane2)
		dsi_ctrl |= BIT(6);
	if (pinfo->data_lane1)
		dsi_ctrl |= BIT(5);
	if (pinfo->data_lane0)
		dsi_ctrl |= BIT(4);

#ifdef RGB_SWAP
	/* there has hardware problem
	 * the color channel between dsi and mdp are swapped
	 */
	MIPI_OUTP(MIPI_DSI_BASE + 0x1c, 0x2000); /* rGB --> BGR */
#endif

	/* from frame buffer, low power mode */
	/* DSI_COMMAND_MODE_DMA_CTRL */
	MIPI_OUTP(MIPI_DSI_BASE + 0x38, 0x14000000);

	data = 0;
	if (pinfo->te_sel)
		data |= BIT(31);
	data |= pinfo->mdp_trigger << 4;/* cmd mdp trigger */
	data |= pinfo->dma_trigger;	/* cmd dma trigger */
	data |= (pinfo->stream & 0x01) << 8;
	MIPI_OUTP(MIPI_DSI_BASE + 0x0080, data); /* DSI_TRIG_CTRL */

	/* DSI_LAN_SWAP_CTRL */
	MIPI_OUTP(MIPI_DSI_BASE + 0x00ac, pinfo->dlane_swap);

	/* clock out ctrl */
	data = pinfo->t_clk_post & 0x3f;	/* 6 bits */
	data <<= 8;
	data |= pinfo->t_clk_pre & 0x3f;	/*  6 bits */
	MIPI_OUTP(MIPI_DSI_BASE + 0xc0, data);	/* DSI_CLKOUT_TIMING_CTRL */

	data = 0;
	if (pinfo->rx_eot_ignore)
		data |= BIT(4);
	if (pinfo->tx_eot_append)
		data |= BIT(0);
	MIPI_OUTP(MIPI_DSI_BASE + 0x00c8, data); /* DSI_EOT_PACKET_CTRL */


	/* allow only ack-err-status  to generate interrupt */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0108, 0x13ff3fe0); /* DSI_ERR_INT_MASK0 */

	intr_ctrl |= DSI_INTR_ERROR_MASK;
	MIPI_OUTP(MIPI_DSI_BASE + 0x010c, intr_ctrl); /* DSI_INTL_CTRL */

	/* turn esc, byte, dsi, pclk, sclk, hclk on */
	MIPI_OUTP(MIPI_DSI_BASE + 0x118, 0x23f); /* DSI_CLK_CTRL */

	dsi_ctrl |= BIT(0);	/* enable dsi */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0000, dsi_ctrl);

	wmb();
}

void mipi_dsi_op_mode_config(int mode)
{

	uint32 dsi_ctrl, intr_ctrl;

	dsi_ctrl = MIPI_INP(MIPI_DSI_BASE + 0x0000);
	dsi_ctrl &= ~0x07;
	if (mode == DSI_VIDEO_MODE) {
		dsi_ctrl |= 0x03;
		intr_ctrl = DSI_INTR_CMD_DMA_DONE_MASK;
	} else {		/* command mode */
		dsi_ctrl |= 0x05;
		intr_ctrl = DSI_INTR_CMD_DMA_DONE_MASK | DSI_INTR_ERROR_MASK |
				DSI_INTR_CMD_MDP_DONE_MASK;
	}

	pr_debug("%s: dsi_ctrl=%x intr=%x\n", __func__, dsi_ctrl, intr_ctrl);

	MIPI_OUTP(MIPI_DSI_BASE + 0x010c, intr_ctrl); /* DSI_INTL_CTRL */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0000, dsi_ctrl);
	wmb();
}

void mipi_dsi_cmd_mdp_sw_trigger(void)
{
	mipi_dsi_pre_kickoff_action();
	mipi_dsi_enable_irq();
	MIPI_OUTP(MIPI_DSI_BASE + 0x090, 0x01);	/* trigger */
	wmb();
}


void mipi_dsi_cmd_bta_sw_trigger(void)
{
	uint32 data;
	int cnt = 0;

	MIPI_OUTP(MIPI_DSI_BASE + 0x094, 0x01);	/* trigger */
	wmb();

	while (cnt < 10000) {
		data = MIPI_INP(MIPI_DSI_BASE + 0x0004);/* DSI_STATUS */
		if ((data & 0x0010) == 0)
			break;
		cnt++;
	}

	mipi_dsi_ack_err_status();

	pr_debug("%s: BTA done, cnt=%d\n", __func__, cnt);
}

static char set_tear_on[2] = {0x35, 0x00};
static struct dsi_cmd_desc dsi_tear_on_cmd = {
	DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(set_tear_on), set_tear_on};

static char set_tear_off[2] = {0x34, 0x00};
static struct dsi_cmd_desc dsi_tear_off_cmd = {
	DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(set_tear_off), set_tear_off};

#ifdef SET_TEAR_SCANLINE
static char set_tear_scanline[3] = {0x44, 0x02, 0xcf};	/* line 719 */

static struct dsi_cmd_desc dsi_tear_scanline_cmd = {
	DTYPE_DCS_LWRITE, 1, 0, 0, 0,
			sizeof(set_tear_scanline), set_tear_scanline};
#endif

void mipi_dsi_set_tear_on(struct msm_fb_data_type *mfd)
{
	mipi_dsi_buf_init(&dsi_tx_buf);
	mipi_dsi_cmds_tx(mfd, &dsi_tx_buf, &dsi_tear_on_cmd, 1);
}

void mipi_dsi_set_tear_off(struct msm_fb_data_type *mfd)
{
	mipi_dsi_buf_init(&dsi_tx_buf);
	mipi_dsi_cmds_tx(mfd, &dsi_tx_buf, &dsi_tear_off_cmd, 1);
}

int mipi_dsi_cmd_reg_tx(uint32 data)
{
#ifdef DSI_HOST_DEBUG
	int i;
	char *bp;

	bp = (char *)&data;
	pr_debug("%s: ", __func__);
	for (i = 0; i < 4; i++)
		pr_debug("%x ", *bp++);

	pr_debug("\n");
#endif

	MIPI_OUTP(MIPI_DSI_BASE + 0x0080, 0x04);/* sw trigger */
	MIPI_OUTP(MIPI_DSI_BASE + 0x0, 0x135);

	wmb();

	MIPI_OUTP(MIPI_DSI_BASE + 0x038, data);
	wmb();
	MIPI_OUTP(MIPI_DSI_BASE + 0x08c, 0x01);	/* trigger */
	wmb();

	udelay(300);

	return 4;
}

/*
 * mipi_dsi_cmds_tx:
 * ov_mutex need to be acquired before call this function.
 */
int mipi_dsi_cmds_tx(struct msm_fb_data_type *mfd,
		struct dsi_buf *tp, struct dsi_cmd_desc *cmds, int cnt)
{
	struct dsi_cmd_desc *cm;
	uint32 dsi_ctrl, ctrl;
	int i, video_mode;

	/* turn on cmd mode
	* for video mode, do not send cmds more than
	* one pixel line, since it only transmit it
	* during BLLP.
	*/
	dsi_ctrl = MIPI_INP(MIPI_DSI_BASE + 0x0000);
	video_mode = dsi_ctrl & 0x02; /* VIDEO_MODE_EN */
	if (video_mode) {
		ctrl = dsi_ctrl | 0x04; /* CMD_MODE_EN */
		MIPI_OUTP(MIPI_DSI_BASE + 0x0000, ctrl);
	} else { /* cmd mode */
		/*
		 * during boot up, cmd mode is configured
		 * even it is video mode panel.
		 */
		/* make sure mdp dma is not txing pixel data */
		if (mfd->panel_info.type == MIPI_CMD_PANEL)
			mdp4_dsi_cmd_dma_busy_wait(mfd);
	}

	mipi_dsi_enable_irq();
	cm = cmds;
	mipi_dsi_buf_init(tp);
	for (i = 0; i < cnt; i++) {
		mipi_dsi_buf_init(tp);
		mipi_dsi_cmd_dma_add(tp, cm);
		mipi_dsi_cmd_dma_tx(tp);
		if (cm->wait)
			msleep(cm->wait);
		cm++;
	}
	mipi_dsi_disable_irq();

	if (video_mode)
		MIPI_OUTP(MIPI_DSI_BASE + 0x0000, dsi_ctrl); /* restore */

	return cnt;
}

/* MIPI_DSI_MRPS, Maximum Return Packet Size */
static char max_pktsize[2] = {0x00, 0x00}; /* LSB tx first, 10 bytes */

static struct dsi_cmd_desc pkt_size_cmd[] = {
	{DTYPE_MAX_PKTSIZE, 1, 0, 0, 0,
		sizeof(max_pktsize), max_pktsize}
};

/*
 * DSI panel reply with  MAX_RETURN_PACKET_SIZE bytes of data
 * plus DCS header, ECC and CRC for DCS long read response
 * mipi_dsi_controller only have 4x32 bits register ( 16 bytes) to
 * hold data per transaction.
 * MIPI_DSI_LEN equal to 8
 * len should be either 4 or 8
 * any return data more than MIPI_DSI_LEN need to be break down
 * to multiple transactions.
 *
 * ov_mutex need to be acquired before call this function.
 */
int mipi_dsi_cmds_rx(struct msm_fb_data_type *mfd,
			struct dsi_buf *tp, struct dsi_buf *rp,
			struct dsi_cmd_desc *cmds, int len)
{
	int cnt, res;
	static int pkt_size;

	if (len <= 2)
		cnt = 4;	/* short read */
	else {
		if (len > MIPI_DSI_LEN)
			len = MIPI_DSI_LEN;	/* 8 bytes at most */

		res = len & 0x03;
		len += (4 - res); /* 4 bytes align */
		/*
		 * add extra 2 bytes to len to have overall
		 * packet size is multipe by 4. This also make
		 * sure 4 bytes dcs headerlocates within a
		 * 32 bits register after shift in.
		 * after all, len should be either 6 or 10.
		 */
		len += 2;
		cnt = len + 6; /* 4 bytes header + 2 bytes crc */
	}


	if (mfd->panel_info.type == MIPI_CMD_PANEL) {
		/* make sure mdp dma is not txing pixel data */
		mdp4_dsi_cmd_dma_busy_wait(mfd);
	}

	mipi_dsi_enable_irq();
	if (pkt_size != len) {
		/* set new max pkt size */
		pkt_size = len;
		max_pktsize[0] = pkt_size;
		mipi_dsi_buf_init(tp);
		mipi_dsi_cmd_dma_add(tp, pkt_size_cmd);
		mipi_dsi_cmd_dma_tx(tp);
	}

	mipi_dsi_buf_init(tp);
	mipi_dsi_cmd_dma_add(tp, cmds);

	/* transmit read comamnd to client */
	mipi_dsi_cmd_dma_tx(tp);
	/*
	 * once cmd_dma_done interrupt received,
	 * return data from client is ready and stored
	 * at RDBK_DATA register already
	 */
	mipi_dsi_cmd_dma_rx(rp, cnt);

	mipi_dsi_disable_irq();

	/* strip off dcs header & crc */
	if (cnt > 4) { /* long response */
		rp->data += 4; /* skip dcs header */
		rp->len -= 6; /* deduct 4 bytes header + 2 bytes crc */
		rp->len -= 2; /* extra 2 bytes added */
	} else {
		rp->data += 1; /* skip dcs short header */
		rp->len -= 2; /* deduct 1 byte header + 1 byte ecc */
	}

	return rp->len;
}

int mipi_dsi_cmd_dma_tx(struct dsi_buf *tp)
{
	int len;

#ifdef DSI_HOST_DEBUG
	int i;
	char *bp;

	bp = tp->data;

	pr_debug("%s: ", __func__);
	for (i = 0; i < tp->len; i++)
		pr_debug("%x ", *bp++);

	pr_debug("\n");
#endif

	len = tp->len;
	len += 3;
	len &= ~0x03;	/* multipled by 4 */

	tp->dmap = dma_map_single(&dsi_dev, tp->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(&dsi_dev, tp->dmap))
		pr_err("%s: dmap mapp failed\n", __func__);

	INIT_COMPLETION(dsi_dma_comp);

	MIPI_OUTP(MIPI_DSI_BASE + 0x044, tp->dmap);
	MIPI_OUTP(MIPI_DSI_BASE + 0x048, len);
	wmb();
	MIPI_OUTP(MIPI_DSI_BASE + 0x08c, 0x01);	/* trigger */
	wmb();

	wait_for_completion(&dsi_dma_comp);

	dma_unmap_single(&dsi_dev, tp->dmap, len, DMA_TO_DEVICE);
	tp->dmap = 0;
	return tp->len;
}

int mipi_dsi_cmd_dma_rx(struct dsi_buf *rp, int rlen)
{
	uint32 *lp, data;
	int i, off, cnt;

	lp = (uint32 *)rp->data;
	cnt = rlen;
	cnt += 3;
	cnt >>= 2;

	if (cnt > 4)
		cnt = 4; /* 4 x 32 bits registers only */

	off = 0x068;	/* DSI_RDBK_DATA0 */
	off += ((cnt - 1) * 4);


	for (i = 0; i < cnt; i++) {
		data = (uint32)MIPI_INP(MIPI_DSI_BASE + off);
		*lp++ = ntohl(data);	/* to network byte order */
		off -= 4;
		rp->len += sizeof(*lp);
	}

	return rlen;
}

void mipi_dsi_irq_set(uint32 mask, uint32 irq)
{
	uint32 data;

	data = MIPI_INP(MIPI_DSI_BASE + 0x010c);/* DSI_INTR_CTRL */
	data &= ~mask;
	data |= irq;
	MIPI_OUTP(MIPI_DSI_BASE + 0x010c, data);
}


void mipi_dsi_ack_err_status(void)
{
	uint32 status;

	status = MIPI_INP(MIPI_DSI_BASE + 0x0064);/* DSI_ACK_ERR_STATUS */

	if (status) {
		MIPI_OUTP(MIPI_DSI_BASE + 0x0064, status);
		pr_debug("%s: status=%x\n", __func__, status);
	}
}

void mipi_dsi_timeout_status(void)
{
	uint32 status;

	status = MIPI_INP(MIPI_DSI_BASE + 0x00bc);/* DSI_TIMEOUT_STATUS */
	if (status & 0x0111) {
		MIPI_OUTP(MIPI_DSI_BASE + 0x00bc, status);
		pr_debug("%s: status=%x\n", __func__, status);
	}
}

void mipi_dsi_dln0_phy_err(void)
{
	uint32 status;

	status = MIPI_INP(MIPI_DSI_BASE + 0x00b0);/* DSI_DLN0_PHY_ERR */

	if (status & 0x011111) {
		MIPI_OUTP(MIPI_DSI_BASE + 0x00b0, status);
		pr_debug("%s: status=%x\n", __func__, status);
	}
}

void mipi_dsi_fifo_status(void)
{
	uint32 status;

	status = MIPI_INP(MIPI_DSI_BASE + 0x0008);/* DSI_FIFO_STATUS */

	if (status & 0x44444489) {
		MIPI_OUTP(MIPI_DSI_BASE + 0x0008, status);
		pr_debug("%s: status=%x\n", __func__, status);
	}
}

void mipi_dsi_status(void)
{
	uint32 status;

	status = MIPI_INP(MIPI_DSI_BASE + 0x0004);/* DSI_STATUS */

	if (status & 0x80000000) {
		MIPI_OUTP(MIPI_DSI_BASE + 0x0004, status);
		pr_debug("%s: status=%x\n", __func__, status);
	}
}

void mipi_dsi_error(void)
{
	/* DSI_ERR_INT_MASK0 */
	mipi_dsi_ack_err_status();	/* mask0, 0x01f */
	mipi_dsi_timeout_status();	/* mask0, 0x0e0 */
	mipi_dsi_fifo_status();		/* mask0, 0x133d00 */
	mipi_dsi_status();		/* mask0, 0xc0100 */
	mipi_dsi_dln0_phy_err();	/* mask0, 0x3e00000 */
}


irqreturn_t mipi_dsi_isr(int irq, void *ptr)
{
	uint32 isr;

	isr = MIPI_INP(MIPI_DSI_BASE + 0x010c);/* DSI_INTR_CTRL */
	MIPI_OUTP(MIPI_DSI_BASE + 0x010c, isr);

#ifdef CONFIG_FB_MSM_MDP40
	mdp4_stat.intr_dsi++;
#endif

	if (isr & DSI_INTR_ERROR) {
		mipi_dsi_error();
	}

	if (isr & DSI_INTR_VIDEO_DONE) {
		/*
		* do something  here
		*/
	}

	if (isr & DSI_INTR_CMD_DMA_DONE) {
		complete(&dsi_dma_comp);
	}

	if (isr & DSI_INTR_CMD_MDP_DONE) {
		mipi_dsi_disable_irq_nosync();
		mipi_dsi_post_kickoff_action();
	}


	return IRQ_HANDLED;
}
