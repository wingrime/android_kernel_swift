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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/msm_audio.h>
#include <mach/msm_hdmi_audio.h>
#include <mach/audio_dma_msm8k.h>
#include <sound/dai.h>
#include "q6core.h"

#define NUM_FRA_IN_BLOCK		192
#define NUM_SAMP_PER_CH_PER_AC3_FRAME	1536
#define AC3_REP_PER			1536	/* num of 60958 Frames */
#define FRAME_60958_SZ			8	/* bytes */
#define PREABLE_61937_SZ_16_BIT		4	/* in 16 bit words */


#define MAX_AC3_FRA_SZ_16_BIT		1920  /* in 16 bit words */
#define DMA_PERIOD_SZ			(AC3_REP_PER * FRAME_60958_SZ)
#define DMA_BUF_SZ			(DMA_PERIOD_SZ * 2)
#define USER_BUF_SZ			DMA_PERIOD_SZ
#define DMA_ALLOC_BUF_SZ		(SZ_4K * 6)

#define HDMI_AUDIO_FIFO_WATER_MARK	4

struct audio_buffer {
	dma_addr_t phys;
	void *data;
	uint32_t size;
	uint32_t used;	/* 1 = CPU is waiting for DMA to consume this buf */
	uint32_t actual_size;	/* actual number of bytes read by DMA */
};

struct lpa_if {
	struct mutex lock;
	struct msm_audio_config cfg;
	struct audio_buffer audio_buf[2];
	int cpu_buf;		/* next buffer the CPU will touch */
	int dma_buf;		/* next buffer the DMA will touch */
	u8 *buffer;
	dma_addr_t buffer_phys;
	u32 dma_ch;
	wait_queue_head_t wait;
	u32 config;
};

static struct lpa_if  *lpa_if_ptr;

static unsigned int dma_buf_index;

static irqreturn_t lpa_if_irq(int intrsrc, void *data)
{
	struct lpa_if *lpa_if = data;
	int dma_ch = 0;
	unsigned int pending;

	if (lpa_if)
		dma_ch = lpa_if->dma_ch;
	else {
		pr_err("invalid lpa_if\n");
		return IRQ_NONE;
	}

	pending = (intrsrc
		   & (UNDER_CH(dma_ch) | PER_CH(dma_ch) | ERR_CH(dma_ch)));

	pr_debug("pending = 0x%08x\n", pending);

	if (pending & UNDER_CH(dma_ch))
		pr_err("under run\n");
	if (pending & ERR_CH(dma_ch))
		pr_err("DMA %x Master Error\n", dma_ch);

	if (pending & PER_CH(dma_ch)) {

		lpa_if->audio_buf[lpa_if->dma_buf].used = 0;

		pr_debug("dma_buf %d  used %d\n", lpa_if->dma_buf,
			lpa_if->audio_buf[lpa_if->dma_buf].used);

		lpa_if->dma_buf ^= 1;

		wake_up(&lpa_if->wait);
	}
	return IRQ_HANDLED;
}


int lpa_if_start(struct lpa_if *lpa_if)
{
	pr_debug("buf1 0x%x, buf2 0x%x dma_ch %d\n",
		(unsigned int)lpa_if->audio_buf[0].data,
		(unsigned int)lpa_if->audio_buf[1].data, lpa_if->dma_ch);

	dai_start_hdmi(lpa_if->dma_ch);

	dsb();

	hdmi_audio_enable(1, HDMI_AUDIO_FIFO_WATER_MARK);
	dsb();
	return 0;
}

int lpa_if_config(struct lpa_if *lpa_if)
{
	struct dai_dma_params dma_params;

	dma_params.src_start = lpa_if->buffer_phys;
	dma_params.buffer = lpa_if->buffer;
	dma_params.buffer_size = DMA_BUF_SZ;
	dma_params.period_size = DMA_PERIOD_SZ;
	dma_params.channels = 2;

	lpa_if->dma_ch = 0;

	dai_set_params(lpa_if->dma_ch, &dma_params);

	register_dma_irq_handler(lpa_if->dma_ch, lpa_if_irq, (void *)lpa_if);

	dsb();
	pr_debug("lpa_if 0x%08x  buf_vir 0x%08x   buf_phys 0x%08x  "
		"config %u\n", (u32)lpa_if, (u32) (lpa_if->buffer),
		lpa_if->buffer_phys, lpa_if->config);

	pr_debug("user_buf_cnt %u user_buf_size %u\n",
			lpa_if->cfg.buffer_count, lpa_if->cfg.buffer_size);

	lpa_if->config = 1;

	lpa_if_start(lpa_if);

	return 0;
}


static long lpa_if_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct lpa_if *lpa_if = file->private_data;
	int rc = 0;

	pr_debug("cmd %u\n", cmd);

	mutex_lock(&lpa_if->lock);

	switch (cmd) {
	case AUDIO_START:
		pr_info("AUDIO_START\n");

		if (dma_buf_index == 2) {
			if (!lpa_if->config) {
				rc = lpa_if_config(lpa_if);
				if (rc)
					pr_err("lpa_if_config failed\n");
			}
		} else {
			pr_err("did not receved two buffer for "
				"AUDIO_STAR\n");
			rc =  -EPERM;
		}
		break;

	case AUDIO_STOP:
		pr_debug("AUDIO_STOP\n");
		break;

	case AUDIO_FLUSH:
		pr_debug("AUDIO_FLUSH\n");
		break;


	case AUDIO_GET_CONFIG:
		pr_debug("AUDIO_GET_CONFIG\n");
		if (copy_to_user((void *)arg, &lpa_if->cfg,
				 sizeof(struct msm_audio_config))) {
			rc = -EFAULT;
		}
		break;

	default:
		pr_err("UnKnown Ioctl\n");
		rc = -EINVAL;
	}

	mutex_unlock(&lpa_if->lock);

	return rc;
}


static int lpa_if_open(struct inode *inode, struct file *file)
{
	struct lpa_if *lpa_if;

	pr_debug("\n");

	file->private_data = lpa_if_ptr;
	lpa_if = lpa_if_ptr;

	lpa_if->cfg.buffer_count = 2;
	lpa_if->cfg.buffer_size = USER_BUF_SZ;

	lpa_if->audio_buf[0].phys = lpa_if->buffer_phys;
	lpa_if->audio_buf[0].data = lpa_if->buffer;
	lpa_if->audio_buf[0].size = DMA_PERIOD_SZ;
	lpa_if->audio_buf[0].used = 0;

	lpa_if->audio_buf[1].phys = lpa_if->buffer_phys + DMA_PERIOD_SZ;
	lpa_if->audio_buf[1].data = lpa_if->buffer + DMA_PERIOD_SZ;
	lpa_if->audio_buf[1].size = DMA_PERIOD_SZ;
	lpa_if->audio_buf[1].used = 0;

	dma_buf_index = 0;

	core_req_bus_bandwith(AUDIO_IF_BUS_ID, 100000, 0);

	return 0;
}

static ssize_t lpa_if_write(struct file *file, const char __user *buf,
		size_t count, loff_t *pos)
{
	struct lpa_if *lpa_if = file->private_data;
	struct audio_buffer *ab;
	const char __user *start = buf;
	int xfer, rc;

	pr_debug("count %u cpu_buf %d dma_buf %d\n",
		(unsigned int)count, lpa_if->cpu_buf, lpa_if->dma_buf);

	mutex_lock(&lpa_if->lock);

	if (dma_buf_index < 2) {

		ab = lpa_if->audio_buf + dma_buf_index;

		if (copy_from_user(ab->data, buf, count)) {
			pr_err("copy from user failed\n");
			rc = 0;
			goto end;

		}
		pr_debug("prefill: count %u  audio_buf[%u].size %u\n",
			 count, dma_buf_index, ab->size);

		ab->used = 1;
		dma_buf_index++;
		rc =  count;
		goto end;
	}

	if (lpa_if->config != 1) {
		pr_err("AUDIO_START did not happen\n");
		rc = 0;
		goto end;
	}

	while (count > 0) {

		ab = lpa_if->audio_buf + lpa_if->cpu_buf;

		rc = wait_event_timeout(lpa_if->wait, (ab->used == 0), 10 * HZ);
		if (!rc) {
			pr_err("wait_event_timeout failed\n");
			rc =  buf - start;
			goto end;
		}

		xfer = count;

		if (xfer > USER_BUF_SZ)
			xfer = USER_BUF_SZ;

		if (copy_from_user(ab->data, buf, xfer)) {
			pr_err("copy from user failed\n");
			rc = buf - start;
			goto end;
		}

		dsb();
		buf += xfer;
		count -= xfer;
		ab->used = 1;

		pr_debug("xfer %d, size %d, used %d cpu_buf %d\n",
			xfer, ab->size, ab->used, lpa_if->cpu_buf);

		lpa_if->cpu_buf ^= 1;
	}
	rc = buf - start;
end:
	mutex_unlock(&lpa_if->lock);
	return rc;
}

static int lpa_if_release(struct inode *inode, struct file *file)
{
	struct lpa_if *lpa_if = file->private_data;
	hdmi_audio_enable(0, HDMI_AUDIO_FIFO_WATER_MARK);

	smp_mb();

	if (lpa_if->config) {
		unregister_dma_irq_handler(lpa_if->dma_ch);
		dai_stop_hdmi(lpa_if->dma_ch);
		lpa_if->config = 0;
	}
	return 0;
}

static const struct file_operations lpa_if_fops = {
	.owner = THIS_MODULE,
	.open = lpa_if_open,
	.write = lpa_if_write,
	.release = lpa_if_release,
	.unlocked_ioctl = lpa_if_ioctl,
};

struct miscdevice lpa_if_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msm_lpa_if_out",
	.fops = &lpa_if_fops,
};

static int __init lpa_if_init(void)
{
	int rc;

	lpa_if_ptr = kzalloc(sizeof(struct lpa_if), GFP_KERNEL);
	if (!lpa_if_ptr) {
		pr_info("No mem for lpa-if\n");
		return -ENOMEM;
	}

	mutex_init(&lpa_if_ptr->lock);
	init_waitqueue_head(&lpa_if_ptr->wait);

	lpa_if_ptr->buffer = dma_alloc_coherent(NULL, DMA_ALLOC_BUF_SZ,
				    &(lpa_if_ptr->buffer_phys), GFP_KERNEL);
	if (!lpa_if_ptr->buffer) {
		pr_err("dma_alloc_coherent failed\n");
		kfree(lpa_if_ptr);
		return -ENOMEM;
	}

	pr_info("lpa_if_ptr 0x%08x   buf_vir 0x%08x   buf_phy 0x%08x "
		" buf_zise %u\n", (u32)lpa_if_ptr,
		(u32)(lpa_if_ptr->buffer), lpa_if_ptr->buffer_phys,
		DMA_ALLOC_BUF_SZ);

	rc =  misc_register(&lpa_if_misc);
	if (rc < 0) {
		pr_err("misc_register failed\n");

		dma_free_coherent(NULL, DMA_ALLOC_BUF_SZ, lpa_if_ptr->buffer,
				lpa_if_ptr->buffer_phys);
		kfree(lpa_if_ptr);
	}
	return rc;
}

device_initcall(lpa_if_init);
