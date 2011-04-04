/*
 *  linux/drivers/mmc/host/msm_sdcc.c - Qualcomm MSM 7X00A SDCC Driver
 *
 *  Copyright (C) 2007 Google Inc,
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
 *  Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Based on mmci.c
 *
 * Author: San Mehat (san@android.com)
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/clk.h>
#include <linux/scatterlist.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/pm_runtime.h>
#include <linux/wakelock.h>

#include <asm/cacheflush.h>
#include <asm/div64.h>
#include <asm/sizes.h>

#include <asm/mach/mmc.h>
#include <mach/msm_iomap.h>
#include <mach/clk.h>
#include <mach/dma.h>
#include <mach/htc_pwrsink.h>

#include "msm_sdcc.h"
#include "msm_sdcc_dml.h"

#define DRIVER_NAME "msm-sdcc"

#define DBG(host, fmt, args...)	\
	pr_debug("%s: %s: " fmt "\n", mmc_hostname(host->mmc), __func__ , args)

#define IRQ_DEBUG 0
#define SPS_SDCC_PRODUCER_PIPE_INDEX	1
#define SPS_SDCC_CONSUMER_PIPE_INDEX	2
#define SPS_CONS_PERIPHERAL		0
#define SPS_PROD_PERIPHERAL		1
/* 16 KB */
#define SPS_MAX_DESC_SIZE		(16 * 1024)

#if defined(CONFIG_DEBUG_FS)
static void msmsdcc_dbg_createhost(struct msmsdcc_host *);
static struct dentry *debugfs_dir;
static struct dentry *debugfs_file;
static int  msmsdcc_dbg_init(void);
#endif

static unsigned int msmsdcc_pwrsave = 1;

#define DUMMY_52_STATE_NONE		0
#define DUMMY_52_STATE_SENT		1

static struct mmc_command dummy52cmd;
static struct mmc_request dummy52mrq = {
	.cmd = &dummy52cmd,
	.data = NULL,
	.stop = NULL,
};
static struct mmc_command dummy52cmd = {
	.opcode = SD_IO_RW_DIRECT,
	.flags = MMC_RSP_PRESENT,
	.data = NULL,
	.mrq = &dummy52mrq,
};

#define VERBOSE_COMMAND_TIMEOUTS	0

#if IRQ_DEBUG == 1
static char *irq_status_bits[] = { "cmdcrcfail", "datcrcfail", "cmdtimeout",
				   "dattimeout", "txunderrun", "rxoverrun",
				   "cmdrespend", "cmdsent", "dataend", NULL,
				   "datablkend", "cmdactive", "txactive",
				   "rxactive", "txhalfempty", "rxhalffull",
				   "txfifofull", "rxfifofull", "txfifoempty",
				   "rxfifoempty", "txdataavlbl", "rxdataavlbl",
				   "sdiointr", "progdone", "atacmdcompl",
				   "sdiointrope", "ccstimeout", NULL, NULL,
				   NULL, NULL, NULL };

static void
msmsdcc_print_status(struct msmsdcc_host *host, char *hdr, uint32_t status)
{
	int i;

	pr_debug("%s-%s ", mmc_hostname(host->mmc), hdr);
	for (i = 0; i < 32; i++) {
		if (status & (1 << i))
			pr_debug("%s ", irq_status_bits[i]);
	}
	pr_debug("\n");
}
#endif

static void
msmsdcc_start_command(struct msmsdcc_host *host, struct mmc_command *cmd,
		      u32 c);

static int msmsdcc_sps_reset_ep(struct msmsdcc_host *host,
				struct msmsdcc_sps_ep_conn_data *ep);
static int msmsdcc_sps_restore_ep(struct msmsdcc_host *host,
				struct msmsdcc_sps_ep_conn_data *ep);
/**
 * Apply soft reset
 *
 * This function applies soft reset to SDCC core and
 * BAM, DML core.
 *
 * This function should be called to recover from error
 * conditions encountered with CMD/DATA tranfsers with card.
 *
 * Soft reset should only be used with SDCC controller v4.
 *
 * @host - Pointer to driver's host structure
 *
 */
static void msmsdcc_soft_reset_and_restore(struct msmsdcc_host *host)
{
	int rc;

	if (host->is_sps_mode) {
		/* Reset DML first */
		msmsdcc_dml_reset(host);
		/* Now reset all BAM pipes connections */
		rc = msmsdcc_sps_reset_ep(host, &host->sps.prod);
		if (rc)
			pr_err("%s:msmsdcc_sps_reset_ep() error=%d\n",
					mmc_hostname(host->mmc), rc);
		rc = msmsdcc_sps_reset_ep(host, &host->sps.cons);
		if (rc)
			pr_err("%s:msmsdcc_sps_reset_ep() error=%d\n",
					mmc_hostname(host->mmc), rc);
	}
	/*
	 * Reset SDCC controller's DPSM (data path state machine
	 * and CPSM (command path state machine).
	 */
	writel(0, host->base + MMCICOMMAND);
	writel(0, host->base + MMCIDATACTRL);

	pr_debug("%s: Applied soft reset to Controller\n",
			mmc_hostname(host->mmc));

	if (host->is_sps_mode) {
		/* Restore all BAM pipes connections */
		rc = msmsdcc_sps_restore_ep(host, &host->sps.prod);
		if (rc)
			pr_err("%s:msmsdcc_sps_restore_ep() error=%d\n",
					mmc_hostname(host->mmc), rc);
		rc = msmsdcc_sps_restore_ep(host, &host->sps.cons);
		if (rc)
			pr_err("%s:msmsdcc_sps_restore_ep() error=%d\n",
					mmc_hostname(host->mmc), rc);
		msmsdcc_dml_init(host);
	}
}

static void msmsdcc_reset_and_restore(struct msmsdcc_host *host)
{
	if (host->plat->sdcc_v4_sup) {
		msmsdcc_soft_reset_and_restore(host);
	} else {
		/* Give Clock reset (hard reset) to controller */
		u32	mci_clk = 0;
		u32	mci_mask0 = 0;
		int ret;

		/* Save the controller state */
		mci_clk = readl(host->base + MMCICLOCK);
		mci_mask0 = readl(host->base + MMCIMASK0);

		/* Reset the controller */
		ret = clk_reset(host->clk, CLK_RESET_ASSERT);
		if (ret)
			pr_err("%s: Clock assert failed at %u Hz"
				" with err %d\n", mmc_hostname(host->mmc),
					host->clk_rate, ret);

		ret = clk_reset(host->clk, CLK_RESET_DEASSERT);
		if (ret)
			pr_err("%s: Clock deassert failed at %u Hz"
				" with err %d\n", mmc_hostname(host->mmc),
					host->clk_rate, ret);

		pr_debug("%s: Controller has been reinitialized\n",
				mmc_hostname(host->mmc));

		/* Restore the contoller state */
		writel(host->pwr, host->base + MMCIPOWER);
		writel(mci_clk, host->base + MMCICLOCK);
		writel(mci_mask0, host->base + MMCIMASK0);
		ret = clk_set_rate(host->clk, host->clk_rate);
		if (ret)
			pr_err("%s: Failed to set clk rate %u Hz. err %d\n",
					mmc_hostname(host->mmc),
					host->clk_rate, ret);
	}
}

static int
msmsdcc_request_end(struct msmsdcc_host *host, struct mmc_request *mrq)
{
	int retval = 0;

	BUG_ON(host->curr.data);

	host->curr.mrq = NULL;
	host->curr.cmd = NULL;

	del_timer(&host->req_tout_timer);

	if (mrq->data)
		mrq->data->bytes_xfered = host->curr.data_xfered;
	if (mrq->cmd->error == -ETIMEDOUT)
		mdelay(5);

	/*
	 * Need to drop the host lock here; mmc_request_done may call
	 * back into the driver...
	 */
	spin_unlock(&host->lock);
	mmc_request_done(host->mmc, mrq);
	spin_lock(&host->lock);

	return retval;
}

static void
msmsdcc_stop_data(struct msmsdcc_host *host)
{
	host->curr.data = NULL;
	host->curr.got_dataend = 0;
}

static inline uint32_t msmsdcc_fifo_addr(struct msmsdcc_host *host)
{
	return host->core_memres->start + MMCIFIFO;
}

static inline void msmsdcc_delay(struct msmsdcc_host *host)
{
	udelay(1 + ((3 * USEC_PER_SEC) /
		(host->clk_rate ? host->clk_rate : host->plat->msmsdcc_fmin)));
}

static inline void
msmsdcc_start_command_exec(struct msmsdcc_host *host, u32 arg, u32 c)
{
	writel(arg, host->base + MMCIARGUMENT);
	msmsdcc_delay(host);
	writel(c, host->base + MMCICOMMAND);
}

static void
msmsdcc_dma_exec_func(struct msm_dmov_cmd *cmd)
{
	struct msmsdcc_host *host = (struct msmsdcc_host *)cmd->user;

	writel(host->cmd_timeout, host->base + MMCIDATATIMER);
	writel((unsigned int)host->curr.xfer_size, host->base + MMCIDATALENGTH);
	msmsdcc_delay(host);	/* Allow data parms to be applied */
	writel(host->cmd_datactrl, host->base + MMCIDATACTRL);
	msmsdcc_delay(host);	/* Force delay prior to ADM or command */

	if (host->cmd_cmd) {
		msmsdcc_start_command_exec(host,
			(u32)host->cmd_cmd->arg, (u32)host->cmd_c);
	}
}

static void
msmsdcc_dma_complete_tlet(unsigned long data)
{
	struct msmsdcc_host *host = (struct msmsdcc_host *)data;
	unsigned long		flags;
	struct mmc_request	*mrq;

	spin_lock_irqsave(&host->lock, flags);
	mrq = host->curr.mrq;
	BUG_ON(!mrq);

	if (!(host->dma.result & DMOV_RSLT_VALID)) {
		pr_err("msmsdcc: Invalid DataMover result\n");
		goto out;
	}

	if (host->dma.result & DMOV_RSLT_DONE) {
		host->curr.data_xfered = host->curr.xfer_size;
		host->curr.xfer_remain -= host->curr.xfer_size;
	} else {
		/* Error or flush  */
		if (host->dma.result & DMOV_RSLT_ERROR)
			pr_err("%s: DMA error (0x%.8x)\n",
			       mmc_hostname(host->mmc), host->dma.result);
		if (host->dma.result & DMOV_RSLT_FLUSH)
			pr_err("%s: DMA channel flushed (0x%.8x)\n",
			       mmc_hostname(host->mmc), host->dma.result);
		pr_err("Flush data: %.8x %.8x %.8x %.8x %.8x %.8x\n",
		       host->dma.err.flush[0], host->dma.err.flush[1],
		       host->dma.err.flush[2], host->dma.err.flush[3],
		       host->dma.err.flush[4],
		       host->dma.err.flush[5]);
		msmsdcc_reset_and_restore(host);
		if (!mrq->data->error)
			mrq->data->error = -EIO;
	}
	dma_unmap_sg(mmc_dev(host->mmc), host->dma.sg, host->dma.num_ents,
		     host->dma.dir);

	if (host->curr.user_pages) {
		struct scatterlist *sg = host->dma.sg;
		int i;

		for (i = 0; i < host->dma.num_ents; i++, sg++)
			flush_dcache_page(sg_page(sg));
	}

	host->dma.sg = NULL;
	host->dma.busy = 0;

	if (host->curr.got_dataend || mrq->data->error) {

		/*
		 * If we've already gotten our DATAEND / DATABLKEND
		 * for this request, then complete it through here.
		 */
		msmsdcc_stop_data(host);

		if (!mrq->data->error) {
			host->curr.data_xfered = host->curr.xfer_size;
			host->curr.xfer_remain -= host->curr.xfer_size;
		}
		if (!mrq->data->stop || mrq->cmd->error) {
			host->curr.mrq = NULL;
			host->curr.cmd = NULL;
			mrq->data->bytes_xfered = host->curr.data_xfered;
			del_timer(&host->req_tout_timer);
			spin_unlock_irqrestore(&host->lock, flags);

			mmc_request_done(host->mmc, mrq);
			return;
		} else
			msmsdcc_start_command(host, mrq->data->stop, 0);
	}

out:
	spin_unlock_irqrestore(&host->lock, flags);
	return;
}

#ifdef CONFIG_MMC_MSM_SPS_SUPPORT
/**
 * Callback notification from SPS driver
 *
 * This callback function gets triggered called from
 * SPS driver when requested SPS data transfer is
 * completed.
 *
 * SPS driver invokes this callback in BAM irq context so
 * SDCC driver schedule a tasklet for further processing
 * this callback notification at later point of time in
 * tasklet context and immediately returns control back
 * to SPS driver.
 *
 * @nofity - Pointer to sps event notify sturcture
 *
 */
static void
msmsdcc_sps_complete_cb(struct sps_event_notify *notify)
{
	struct msmsdcc_host *host =
		(struct msmsdcc_host *)
		((struct sps_event_notify *)notify)->user;

	host->sps.notify = *notify;
	pr_debug("%s: %s: sps ev_id=%d, addr=0x%x, size=0x%x, flags=0x%x\n",
		mmc_hostname(host->mmc), __func__, notify->event_id,
		notify->data.transfer.iovec.addr,
		notify->data.transfer.iovec.size,
		notify->data.transfer.iovec.flags);
	/* Schedule a tasklet for completing data transfer */
	tasklet_schedule(&host->sps.tlet);
}

/**
 * Tasklet handler for processing SPS callback event
 *
 * This function processing SPS event notification and
 * checks if the SPS transfer is completed or not and
 * then accordingly notifies status to MMC core layer.
 *
 * This function is called in tasklet context.
 *
 * @data - Pointer to sdcc driver data
 *
 */
static void msmsdcc_sps_complete_tlet(unsigned long data)
{
	unsigned long flags;
	int i, rc;
	u32 data_xfered = 0;
	struct mmc_request *mrq;
	struct sps_iovec iovec;
	struct sps_pipe *sps_pipe_handle;
	struct msmsdcc_host *host = (struct msmsdcc_host *)data;
	struct sps_event_notify *notify = &host->sps.notify;

	spin_lock_irqsave(&host->lock, flags);
	if (host->sps.dir == DMA_FROM_DEVICE)
		sps_pipe_handle = host->sps.prod.pipe_handle;
	else
		sps_pipe_handle = host->sps.cons.pipe_handle;
	mrq = host->curr.mrq;
	BUG_ON(!mrq);
	pr_debug("%s: %s: sps event_id=%d\n",
		mmc_hostname(host->mmc), __func__,
		notify->event_id);

	if (msmsdcc_is_dml_busy(host)) {
		/* oops !!! this should never happen. */
		pr_err("%s: %s: Received SPS EOT event"
			" but DML HW is still busy !!!\n",
			mmc_hostname(host->mmc), __func__);
	}
	/*
	 * Got End of transfer event!!! Check if all of the data
	 * has been transferred?
	 */
	for (i = 0; i < host->sps.xfer_req_cnt; i++) {
		rc = sps_get_iovec(sps_pipe_handle, &iovec);
		if (rc) {
			pr_err("%s: %s: sps_get_iovec() failed rc=%d, i=%d",
				mmc_hostname(host->mmc), __func__, rc, i);
			break;
		}
		data_xfered += iovec.size;
	}

	if (data_xfered == host->curr.xfer_size) {
		host->curr.data_xfered = host->curr.xfer_size;
		host->curr.xfer_remain -= host->curr.xfer_size;
		pr_debug("%s: Data xfer success. data_xfered=0x%x",
			mmc_hostname(host->mmc),
			host->curr.xfer_size);
	} else {
		pr_err("%s: Data xfer failed. data_xfered=0x%x,"
			" xfer_size=%d", mmc_hostname(host->mmc),
			data_xfered, host->curr.xfer_size);
		msmsdcc_reset_and_restore(host);
		if (!mrq->data->error)
			mrq->data->error = -EIO;
	}

	/* Unmap sg buffers */
	dma_unmap_sg(mmc_dev(host->mmc), host->sps.sg, host->sps.num_ents,
			 host->sps.dir);

	host->sps.sg = NULL;
	host->sps.busy = 0;

	if (host->curr.got_dataend || mrq->data->error) {
		/*
		 * If we've already gotten our DATAEND / DATABLKEND
		 * for this request, then complete it through here.
		 */
		msmsdcc_stop_data(host);

		if (!mrq->data->error) {
			host->curr.data_xfered = host->curr.xfer_size;
			host->curr.xfer_remain -= host->curr.xfer_size;
		}
		if (!mrq->data->stop || mrq->cmd->error) {
			host->curr.mrq = NULL;
			host->curr.cmd = NULL;
			mrq->data->bytes_xfered = host->curr.data_xfered;

			spin_unlock_irqrestore(&host->lock, flags);

#ifdef CONFIG_MMC_MSM_PROG_DONE_SCAN
			if ((mrq->cmd->opcode == SD_IO_RW_EXTENDED)
				&& (mrq->cmd->arg & 0x80000000)) {
				/* Set the prog_scan in a cmd53.*/
				host->prog_scan = 1;
				/* Send STOP to let the SDCC know to stop. */
				writel(MCI_CSPM_MCIABORT,
						host->base + MMCICOMMAND);
			}
#endif /* CONFIG_MMC_MSM_PROG_DONE_SCAN */
			mmc_request_done(host->mmc, mrq);
			return;
		} else {
			msmsdcc_start_command(host, mrq->data->stop, 0);
	}
	}
	spin_unlock_irqrestore(&host->lock, flags);
}

/**
 * Exit from current SPS data transfer
 *
 * This function exits from current SPS data transfer.
 *
 * This function should be called when error condition
 * is encountered during data transfer.
 *
 * @host - Pointer to sdcc host structure
 *
 */
static void msmsdcc_sps_exit_curr_xfer(struct msmsdcc_host *host)
{
	struct mmc_request *mrq;

	mrq = host->curr.mrq;
	BUG_ON(!mrq);

	msmsdcc_reset_and_restore(host);
	if (!mrq->data->error)
		mrq->data->error = -EIO;

	/* Unmap sg buffers */
	dma_unmap_sg(mmc_dev(host->mmc), host->sps.sg, host->sps.num_ents,
			 host->sps.dir);

	host->sps.sg = NULL;
	host->sps.busy = 0;
	msmsdcc_stop_data(host);
	msmsdcc_request_end(host, mrq);

}
#else
static inline void msmsdcc_sps_complete_cb(struct sps_event_notify *notify) { }
static inline void msmsdcc_sps_complete_tlet(unsigned long data) { }
static inline void msmsdcc_sps_exit_curr_xfer(struct msmsdcc_host *host) { }
#endif /* CONFIG_MMC_MSM_SPS_SUPPORT */

static void
msmsdcc_dma_complete_func(struct msm_dmov_cmd *cmd,
			  unsigned int result,
			  struct msm_dmov_errdata *err)
{
	struct msmsdcc_dma_data	*dma_data =
		container_of(cmd, struct msmsdcc_dma_data, hdr);
	struct msmsdcc_host *host = dma_data->host;

	dma_data->result = result;
	if (err)
		memcpy(&dma_data->err, err, sizeof(struct msm_dmov_errdata));

	tasklet_schedule(&host->dma_tlet);
}

static int msmsdcc_check_dma_op_req(struct mmc_data *data)
{
	if (((data->blksz * data->blocks) < MCI_FIFOSIZE) ||
	     ((data->blksz * data->blocks) % MCI_FIFOSIZE))
		return -EINVAL;
	else
		return 0;
}

static int msmsdcc_config_dma(struct msmsdcc_host *host, struct mmc_data *data)
{
	struct msmsdcc_nc_dmadata *nc;
	dmov_box *box;
	uint32_t rows;
	uint32_t crci;
	unsigned int n;
	int i;
	struct scatterlist *sg = data->sg;

	if (host->dma.channel == -1)
		return -ENOENT;

	host->dma.sg = data->sg;
	host->dma.num_ents = data->sg_len;

	BUG_ON(host->dma.num_ents > NR_SG); /* Prevent memory corruption */

	nc = host->dma.nc;

	if (host->pdev_id == 1)
		crci = DMOV_SDC1_CRCI;
	else if (host->pdev_id == 2)
		crci = DMOV_SDC2_CRCI;
	else if (host->pdev_id == 3)
		crci = DMOV_SDC3_CRCI;
	else if (host->pdev_id == 4)
		crci = DMOV_SDC4_CRCI;
#ifdef DMOV_SDC5_CRCI
	else if (host->pdev_id == 5)
		crci = DMOV_SDC5_CRCI;
#endif
	else {
		host->dma.sg = NULL;
		host->dma.num_ents = 0;
		return -ENOENT;
	}

	if (data->flags & MMC_DATA_READ)
		host->dma.dir = DMA_FROM_DEVICE;
	else
		host->dma.dir = DMA_TO_DEVICE;

	/* host->curr.user_pages = (data->flags & MMC_DATA_USERPAGE); */
	host->curr.user_pages = 0;
	box = &nc->cmd[0];
	for (i = 0; i < host->dma.num_ents; i++) {
		box->cmd = CMD_MODE_BOX;

		/* Initialize sg dma address */
		sg->dma_address = page_to_dma(mmc_dev(host->mmc), sg_page(sg))
					+ sg->offset;

		if (i == (host->dma.num_ents - 1))
			box->cmd |= CMD_LC;
		rows = (sg_dma_len(sg) % MCI_FIFOSIZE) ?
			(sg_dma_len(sg) / MCI_FIFOSIZE) + 1 :
			(sg_dma_len(sg) / MCI_FIFOSIZE) ;

		if (data->flags & MMC_DATA_READ) {
			box->src_row_addr = msmsdcc_fifo_addr(host);
			box->dst_row_addr = sg_dma_address(sg);

			box->src_dst_len = (MCI_FIFOSIZE << 16) |
					   (MCI_FIFOSIZE);
			box->row_offset = MCI_FIFOSIZE;

			box->num_rows = rows * ((1 << 16) + 1);
			box->cmd |= CMD_SRC_CRCI(crci);
		} else {
			box->src_row_addr = sg_dma_address(sg);
			box->dst_row_addr = msmsdcc_fifo_addr(host);

			box->src_dst_len = (MCI_FIFOSIZE << 16) |
					   (MCI_FIFOSIZE);
			box->row_offset = (MCI_FIFOSIZE << 16);

			box->num_rows = rows * ((1 << 16) + 1);
			box->cmd |= CMD_DST_CRCI(crci);
		}
		box++;
		sg++;
	}

	/* location of command block must be 64 bit aligned */
	BUG_ON(host->dma.cmd_busaddr & 0x07);

	nc->cmdptr = (host->dma.cmd_busaddr >> 3) | CMD_PTR_LP;
	host->dma.hdr.cmdptr = DMOV_CMD_PTR_LIST |
			       DMOV_CMD_ADDR(host->dma.cmdptr_busaddr);
	host->dma.hdr.complete_func = msmsdcc_dma_complete_func;
	host->dma.hdr.crci_mask = msm_dmov_build_crci_mask(1, crci);

	n = dma_map_sg(mmc_dev(host->mmc), host->dma.sg,
			host->dma.num_ents, host->dma.dir);
	/* dsb inside dma_map_sg will write nc out to mem as well */

	if (n != host->dma.num_ents) {
		pr_err("%s: Unable to map in all sg elements\n",
		       mmc_hostname(host->mmc));
		host->dma.sg = NULL;
		host->dma.num_ents = 0;
		return -ENOMEM;
	}

	return 0;
}

#ifdef CONFIG_MMC_MSM_SPS_SUPPORT
/**
 * Submits data transfer request to SPS driver
 *
 * This function make sg (scatter gather) data buffers
 * DMA ready and then submits them to SPS driver for
 * transfer.
 *
 * @host - Pointer to sdcc host structure
 * @data - Pointer to mmc_data structure
 *
 * @return 0 if success else negative value
 */
static int msmsdcc_sps_start_xfer(struct msmsdcc_host *host,
				struct mmc_data *data)
{
	int rc = 0;
	u32 flags;
	int i;
	u32 addr, len, data_cnt;
	struct scatterlist *sg = data->sg;
	struct sps_pipe *sps_pipe_handle;

	BUG_ON(data->sg_len > NR_SG); /* Prevent memory corruption */

	host->sps.sg = data->sg;
	host->sps.num_ents = data->sg_len;
	host->sps.xfer_req_cnt = 0;
	if (data->flags & MMC_DATA_READ) {
		host->sps.dir = DMA_FROM_DEVICE;
		sps_pipe_handle = host->sps.prod.pipe_handle;
	} else {
		host->sps.dir = DMA_TO_DEVICE;
		sps_pipe_handle = host->sps.cons.pipe_handle;
	}

	/* Make sg buffers DMA ready */
	rc = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
			host->sps.dir);

	if (rc != data->sg_len) {
		pr_err("%s: Unable to map in all sg elements, rc=%d\n",
		       mmc_hostname(host->mmc), rc);
		host->sps.sg = NULL;
		host->sps.num_ents = 0;
		rc = -ENOMEM;
		goto dma_map_err;
	}

	pr_debug("%s: %s: %s: pipe=0x%x, total_xfer=0x%x, sg_len=%d\n",
		mmc_hostname(host->mmc), __func__,
		host->sps.dir == DMA_FROM_DEVICE ? "READ" : "WRITE",
		(u32)sps_pipe_handle, host->curr.xfer_size, data->sg_len);

	for (i = 0; i < data->sg_len; i++) {
		/*
		 * Check if this is the last buffer to transfer?
		 * If yes then set the INT and EOT flags.
		 */
		len = sg_dma_len(sg);
		addr = sg_dma_address(sg);
		flags = 0;
		while (len > 0) {
			if (len > SPS_MAX_DESC_SIZE) {
				data_cnt = SPS_MAX_DESC_SIZE;
			} else {
				data_cnt = len;
				if (i == data->sg_len - 1)
					flags = SPS_IOVEC_FLAG_INT |
						SPS_IOVEC_FLAG_EOT;
			}
			rc = sps_transfer_one(sps_pipe_handle, addr,
						data_cnt, host, flags);
			if (rc) {
				pr_err("%s: sps_transfer_one() error! rc=%d,"
					" pipe=0x%x, sg=0x%x, sg_buf_no=%d\n",
					mmc_hostname(host->mmc), rc,
					(u32)sps_pipe_handle, (u32)sg, i);
				goto dma_map_err;
			}
			addr += data_cnt;
			len -= data_cnt;
			host->sps.xfer_req_cnt++;
		}
		sg++;
	}
	goto out;

dma_map_err:
	/* unmap sg buffers */
	dma_unmap_sg(mmc_dev(host->mmc), host->sps.sg, host->sps.num_ents,
			host->sps.dir);
out:
	return rc;
}
#else
static int msmsdcc_sps_start_xfer(struct msmsdcc_host *host,
				struct mmc_data *data) { return 0; }
#endif /* CONFIG_MMC_MSM_SPS_SUPPORT */

static void
msmsdcc_start_command_deferred(struct msmsdcc_host *host,
				struct mmc_command *cmd, u32 *c)
{
	DBG(host, "op %02x arg %08x flags %08x\n",
	    cmd->opcode, cmd->arg, cmd->flags);

	*c |= (cmd->opcode | MCI_CPSM_ENABLE);

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			*c |= MCI_CPSM_LONGRSP;
		*c |= MCI_CPSM_RESPONSE;
	}

	if (/*interrupt*/0)
		*c |= MCI_CPSM_INTERRUPT;

	if ((((cmd->opcode == 17) || (cmd->opcode == 18))  ||
	     ((cmd->opcode == 24) || (cmd->opcode == 25))) ||
	      (cmd->opcode == 53))
		*c |= MCI_CSPM_DATCMD;

	if (host->prog_scan && (cmd->opcode == 12)) {
		*c |= MCI_CPSM_PROGENA;
		host->prog_enable = 1;
	}

	if (cmd == cmd->mrq->stop)
		*c |= MCI_CSPM_MCIABORT;

	if (host->curr.cmd != NULL) {
		pr_err("%s: Overlapping command requests\n",
		       mmc_hostname(host->mmc));
	}
	host->curr.cmd = cmd;

	/*
	 * Kick the software command timeout timer here.
	 * Timer expires in 10 secs.
	 */
	mod_timer(&host->req_tout_timer,
			(jiffies + msecs_to_jiffies(MSM_MMC_REQ_TIMEOUT)));
}

static void
msmsdcc_start_data(struct msmsdcc_host *host, struct mmc_data *data,
			struct mmc_command *cmd, u32 c)
{
	unsigned int datactrl, timeout;
	unsigned long long clks;
	void __iomem *base = host->base;
	unsigned int pio_irqmask = 0;

	host->curr.data = data;
	host->curr.xfer_size = data->blksz * data->blocks;
	host->curr.xfer_remain = host->curr.xfer_size;
	host->curr.data_xfered = 0;
	host->curr.got_dataend = 0;

	memset(&host->pio, 0, sizeof(host->pio));

	datactrl = MCI_DPSM_ENABLE | (data->blksz << 4);

	if (!msmsdcc_check_dma_op_req(data)) {
		if (host->is_dma_mode && !msmsdcc_config_dma(host, data)) {
			datactrl |= MCI_DPSM_DMAENABLE;
		} else if (host->is_sps_mode) {
			if (!msmsdcc_is_dml_busy(host)) {
				if (!msmsdcc_sps_start_xfer(host, data)) {
					/* Now kick start DML transfer */
					msmsdcc_dml_start_xfer(host, data);
					datactrl |= MCI_DPSM_DMAENABLE;
					host->sps.busy = 1;
				}
			} else {
				/*
				 * Can't proceed with new transfer as
				 * previous trasnfer is already in progress.
				 * There is no point of going into PIO mode
				 * as well. Is this a time to do kernel panic?
				 */
				pr_err("%s: %s: DML HW is busy!!!"
					" Can't perform new SPS transfers"
					" now\n", mmc_hostname(host->mmc),
					__func__);
			}
		}
	}

	/* Is data transfer in PIO mode required? */
	if (!(datactrl & MCI_DPSM_DMAENABLE)) {
		host->pio.sg = data->sg;
		host->pio.sg_len = data->sg_len;
		host->pio.sg_off = 0;

		if (data->flags & MMC_DATA_READ) {
			pio_irqmask = MCI_RXFIFOHALFFULLMASK;
			if (host->curr.xfer_remain < MCI_FIFOSIZE)
				pio_irqmask |= MCI_RXDATAAVLBLMASK;
		} else
			pio_irqmask = MCI_TXFIFOHALFEMPTYMASK |
					MCI_TXFIFOEMPTYMASK;
	}

	if (data->flags & MMC_DATA_READ)
		datactrl |= MCI_DPSM_DIRECTION;

	clks = (unsigned long long)data->timeout_ns * host->clk_rate;
	do_div(clks, 1000000000UL);
	timeout = data->timeout_clks + (unsigned int)clks*2 ;

	if (host->is_dma_mode && (datactrl & MCI_DPSM_DMAENABLE)) {
		/* Use ADM (Application Data Mover) HW for Data transfer */
		/* Save parameters for the dma exec function */
		host->cmd_timeout = timeout;
		host->cmd_pio_irqmask = pio_irqmask;
		host->cmd_datactrl = datactrl;
		host->cmd_cmd = cmd;

		host->dma.hdr.exec_func = msmsdcc_dma_exec_func;
		host->dma.hdr.user = (void *)host;
		host->dma.busy = 1;
		if (data->flags & MMC_DATA_WRITE)
			host->prog_scan = 1;

		if (cmd) {
			msmsdcc_start_command_deferred(host, cmd, &c);
			host->cmd_c = c;
		}
		writel((readl(host->base + MMCIMASK0) & (~(MCI_IRQ_PIO))) |
				host->cmd_pio_irqmask, host->base + MMCIMASK0);
		dsb();
		msm_dmov_enqueue_cmd_ext(host->dma.channel, &host->dma.hdr);
	} else {
		/* SPS-BAM mode or PIO mode */
		if (data->flags & MMC_DATA_WRITE)
			host->prog_scan = 1;
		writel(timeout, base + MMCIDATATIMER);

		writel(host->curr.xfer_size, base + MMCIDATALENGTH);

		writel((readl(host->base + MMCIMASK0) & (~(MCI_IRQ_PIO))) |
				pio_irqmask, host->base + MMCIMASK0);
		msmsdcc_delay(host);	/* Allow parms to be applied */
		writel(datactrl, base + MMCIDATACTRL);

		if (cmd) {
			msmsdcc_delay(host); /* Delay between data/command */
			/* Daisy-chain the command if requested */
			msmsdcc_start_command(host, cmd, c);
		}
	}
}

static void
msmsdcc_start_command(struct msmsdcc_host *host, struct mmc_command *cmd, u32 c)
{
	msmsdcc_start_command_deferred(host, cmd, &c);
	msmsdcc_start_command_exec(host, cmd->arg, c);
}

static void
msmsdcc_data_err(struct msmsdcc_host *host, struct mmc_data *data,
		 unsigned int status)
{
	if (status & MCI_DATACRCFAIL) {
		if (!(data->mrq->cmd->opcode == MMC_BUSTEST_W
			|| data->mrq->cmd->opcode == MMC_BUSTEST_R)) {
			pr_err("%s: Data CRC error\n",
			       mmc_hostname(host->mmc));
			pr_err("%s: opcode 0x%.8x\n", __func__,
			       data->mrq->cmd->opcode);
			pr_err("%s: blksz %d, blocks %d\n", __func__,
			       data->blksz, data->blocks);
			data->error = -EILSEQ;
		}
	} else if (status & MCI_DATATIMEOUT) {
		/* CRC is optional for the bus test commands, not all
		 * cards respond back with CRC. However controller
		 * waits for the CRC and times out. Hence ignore the
		 * data timeouts during the Bustest.
		 */
		if (!(data->mrq->cmd->opcode == MMC_BUSTEST_W
			|| data->mrq->cmd->opcode == MMC_BUSTEST_R)) {
			pr_err("%s: Data timeout\n",
				 mmc_hostname(host->mmc));
			data->error = -ETIMEDOUT;
		}
	} else if (status & MCI_RXOVERRUN) {
		pr_err("%s: RX overrun\n", mmc_hostname(host->mmc));
		data->error = -EIO;
	} else if (status & MCI_TXUNDERRUN) {
		pr_err("%s: TX underrun\n", mmc_hostname(host->mmc));
		data->error = -EIO;
	} else {
		pr_err("%s: Unknown error (0x%.8x)\n",
		      mmc_hostname(host->mmc), status);
		data->error = -EIO;
	}

	/* Dummy CMD52 is not needed when CMD53 has errors */
	if (host->plat->dummy52_required && host->dummy_52_needed)
		host->dummy_52_needed = 0;
}

static int
msmsdcc_pio_read(struct msmsdcc_host *host, char *buffer, unsigned int remain)
{
	void __iomem	*base = host->base;
	uint32_t	*ptr = (uint32_t *) buffer;
	int		count = 0;

	if (remain % 4)
		remain = ((remain >> 2) + 1) << 2;

	while (readl(base + MMCISTATUS) & MCI_RXDATAAVLBL) {

		*ptr = readl(base + MMCIFIFO + (count % MCI_FIFOSIZE));
		ptr++;
		count += sizeof(uint32_t);

		remain -=  sizeof(uint32_t);
		if (remain == 0)
			break;
	}
	return count;
}

static int
msmsdcc_pio_write(struct msmsdcc_host *host, char *buffer,
		  unsigned int remain)
{
	void __iomem *base = host->base;
	char *ptr = buffer;
	unsigned int maxcnt = MCI_FIFOHALFSIZE;

	while (readl(base + MMCISTATUS) &
		(MCI_TXFIFOEMPTY | MCI_TXFIFOHALFEMPTY)) {
		unsigned int count, sz;

		count = min(remain, maxcnt);

		sz = count % 4 ? (count >> 2) + 1 : (count >> 2);
		writesl(base + MMCIFIFO, ptr, sz);
		ptr += count;
		remain -= count;

		if (remain == 0)
			break;
	}

	return ptr - buffer;
}

static irqreturn_t
msmsdcc_pio_irq(int irq, void *dev_id)
{
	struct msmsdcc_host	*host = dev_id;
	void __iomem		*base = host->base;
	uint32_t		status;

	status = readl(base + MMCISTATUS);
	if (((readl(host->base + MMCIMASK0) & status) & (MCI_IRQ_PIO)) == 0)
		return IRQ_NONE;

#if IRQ_DEBUG
	msmsdcc_print_status(host, "irq1-r", status);
#endif

	spin_lock(&host->lock);

	do {
		unsigned long flags;
		unsigned int remain, len;
		char *buffer;

		if (!(status & (MCI_TXFIFOHALFEMPTY | MCI_TXFIFOEMPTY
				| MCI_RXDATAAVLBL)))
			break;

		/* Map the current scatter buffer */
		local_irq_save(flags);
		buffer = kmap_atomic(sg_page(host->pio.sg),
				     KM_BIO_SRC_IRQ) + host->pio.sg->offset;
		buffer += host->pio.sg_off;
		remain = host->pio.sg->length - host->pio.sg_off;

		len = 0;
		if (status & MCI_RXACTIVE)
			len = msmsdcc_pio_read(host, buffer, remain);
		if (status & MCI_TXACTIVE)
			len = msmsdcc_pio_write(host, buffer, remain);

		/* Unmap the buffer */
		kunmap_atomic(buffer, KM_BIO_SRC_IRQ);
		local_irq_restore(flags);

		host->pio.sg_off += len;
		host->curr.xfer_remain -= len;
		host->curr.data_xfered += len;
		remain -= len;

		if (remain) /* Done with this page? */
			break; /* Nope */

		if (status & MCI_RXACTIVE && host->curr.user_pages)
			flush_dcache_page(sg_page(host->pio.sg));

		if (!--host->pio.sg_len) {
			memset(&host->pio, 0, sizeof(host->pio));
			break;
		}

		/* Advance to next sg */
		host->pio.sg++;
		host->pio.sg_off = 0;

		status = readl(base + MMCISTATUS);
	} while (1);

	if (status & MCI_RXACTIVE && host->curr.xfer_remain < MCI_FIFOSIZE) {
		writel((readl(host->base + MMCIMASK0) & (~(MCI_IRQ_PIO))) |
				MCI_RXDATAAVLBLMASK, host->base + MMCIMASK0);
		if (!host->curr.xfer_remain) {
			/* Delay needed (same port was just written) */
			msmsdcc_delay(host);
			writel((readl(host->base + MMCIMASK0) &
				(~(MCI_IRQ_PIO))) | 0, host->base + MMCIMASK0);
		}
	} else if (!host->curr.xfer_remain)
		writel((readl(host->base + MMCIMASK0) & (~(MCI_IRQ_PIO))) | 0,
				host->base + MMCIMASK0);

	spin_unlock(&host->lock);

	return IRQ_HANDLED;
}

static void
msmsdcc_request_start(struct msmsdcc_host *host, struct mmc_request *mrq);

static void msmsdcc_wait_for_rxdata(struct msmsdcc_host *host,
					struct mmc_data *data)
{
	u32 loop_cnt = 0;

	/*
	 * For read commands with data less than fifo size, it is possible to
	 * get DATAEND first and RXDATA_AVAIL might be set later because of
	 * synchronization delay through the asynchronous RX FIFO. Thus, for
	 * such cases, even after DATAEND interrupt is received software
	 * should poll for RXDATA_AVAIL until the requested data is read out
	 * of FIFO. This change is needed to get around this abnormal but
	 * sometimes expected behavior of SDCC3 controller.
	 *
	 * We can expect RXDATAAVAIL bit to be set after 6HCLK clock cycles
	 * after the data is loaded into RX FIFO. This would amount to less
	 * than a microsecond and thus looping for 1000 times is good enough
	 * for that delay.
	 */
	while (((int)host->curr.xfer_remain > 0) && (++loop_cnt < 1000)) {
		if (readl(host->base + MMCISTATUS) & MCI_RXDATAAVLBL) {
			spin_unlock(&host->lock);
			msmsdcc_pio_irq(1, host);
			spin_lock(&host->lock);
		}
	}
	if (loop_cnt == 1000) {
		pr_info("%s: Timed out while polling for Rx Data\n",
				mmc_hostname(host->mmc));
		data->error = -ETIMEDOUT;
		msmsdcc_reset_and_restore(host);
	}
}

static void msmsdcc_do_cmdirq(struct msmsdcc_host *host, uint32_t status)
{
	struct mmc_command *cmd = host->curr.cmd;

	host->curr.cmd = NULL;
	cmd->resp[0] = readl(host->base + MMCIRESPONSE0);
	cmd->resp[1] = readl(host->base + MMCIRESPONSE1);
	cmd->resp[2] = readl(host->base + MMCIRESPONSE2);
	cmd->resp[3] = readl(host->base + MMCIRESPONSE3);

	if (status & MCI_CMDTIMEOUT) {
#if VERBOSE_COMMAND_TIMEOUTS
		pr_err("%s: Command timeout\n", mmc_hostname(host->mmc));
#endif
		cmd->error = -ETIMEDOUT;
	} else if (status & MCI_CMDCRCFAIL && cmd->flags & MMC_RSP_CRC) {
		pr_err("%s: Command CRC error\n", mmc_hostname(host->mmc));
		cmd->error = -EILSEQ;
	}

	if (!cmd->data || cmd->error) {
		if (host->curr.data && host->dma.sg &&
			host->is_dma_mode)
			msm_dmov_stop_cmd(host->dma.channel,
					  &host->dma.hdr, 0);
		else if (host->curr.data && host->sps.sg &&
			host->is_sps_mode){
			/* Stop current SPS transfer */
			msmsdcc_sps_exit_curr_xfer(host);
		}
		else if (host->curr.data) { /* Non DMA */
			msmsdcc_reset_and_restore(host);
			msmsdcc_stop_data(host);
			msmsdcc_request_end(host, cmd->mrq);
		} else { /* host->data == NULL */
			if (!cmd->error && host->prog_enable) {
				if (status & MCI_PROGDONE) {
					host->prog_scan = 0;
					host->prog_enable = 0;
					 msmsdcc_request_end(host, cmd->mrq);
				} else
					host->curr.cmd = cmd;
			} else {
				if (host->prog_enable) {
					host->prog_scan = 0;
					host->prog_enable = 0;
				}
				if (cmd->data && cmd->error) {
					msmsdcc_reset_and_restore(host);
					if (host->plat->dummy52_required &&
							host->dummy_52_needed)
						host->dummy_52_needed = 0;
				}
				msmsdcc_request_end(host, cmd->mrq);
			}
		}
	} else if (cmd->data) {
		if (!(cmd->data->flags & MMC_DATA_READ))
			msmsdcc_start_data(host, cmd->data, NULL, 0);
	}
}

static irqreturn_t
msmsdcc_irq(int irq, void *dev_id)
{
	struct msmsdcc_host	*host = dev_id;
	u32			status;
	int			ret = 0;
	int			timer = 0;

	spin_lock(&host->lock);

	do {
		struct mmc_command *cmd;
		struct mmc_data *data;

		if (timer) {
			timer = 0;
			msmsdcc_delay(host);
		}

		if (!host->clks_on) {
			pr_debug("%s: %s: SDIO async irq received\n",
					mmc_hostname(host->mmc), __func__);
			host->mmc->ios.clock = host->clk_rate;
			spin_unlock(&host->lock);
			host->mmc->ops->set_ios(host->mmc, &host->mmc->ios);
			spin_lock(&host->lock);
			if (host->plat->cfg_mpm_sdiowakeup &&
				(host->mmc->pm_flags & MMC_PM_WAKE_SDIO_IRQ) &&
				!host->sdio_irq_disabled) {
				host->sdio_irq_disabled = 1;
				wake_lock(&host->sdio_wlock);
			}
			/* only ansyc interrupt can come when clocks are off */
			writel(MCI_SDIOINTMASK, host->base + MMCICLEAR);
		}

		status = readl(host->base + MMCISTATUS);

		if (((readl(host->base + MMCIMASK0) & status) &
						(~(MCI_IRQ_PIO))) == 0)
			break;

#if IRQ_DEBUG
		msmsdcc_print_status(host, "irq0-r", status);
#endif
		status &= readl(host->base + MMCIMASK0);
		writel(status, host->base + MMCICLEAR);
#if IRQ_DEBUG
		msmsdcc_print_status(host, "irq0-p", status);
#endif

		if ((host->plat->dummy52_required) &&
		    (host->dummy_52_state == DUMMY_52_STATE_SENT)) {
			if (status & (MCI_PROGDONE | MCI_CMDCRCFAIL |
					  MCI_CMDTIMEOUT)) {
				if (status & MCI_CMDTIMEOUT)
					pr_debug("%s: dummy CMD52 timeout\n",
						mmc_hostname(host->mmc));
				if (status & MCI_CMDCRCFAIL)
					pr_debug("%s: dummy CMD52 CRC failed\n",
						mmc_hostname(host->mmc));
				host->dummy_52_state = DUMMY_52_STATE_NONE;
				host->curr.cmd = NULL;
				msmsdcc_request_start(host, host->curr.mrq);
				spin_unlock(&host->lock);
				return IRQ_HANDLED;
			}
			break;
		}

		data = host->curr.data;
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
		if (status & MCI_SDIOINTROPE) {
			if (host->sdcc_suspending)
				wake_lock(&host->sdio_suspend_wlock);
			mmc_signal_sdio_irq(host->mmc);
		}
#endif
		/*
		 * Check for proper command response
		 */
		cmd = host->curr.cmd;
		if ((status & (MCI_CMDSENT | MCI_CMDRESPEND | MCI_CMDCRCFAIL |
			      MCI_CMDTIMEOUT | MCI_PROGDONE)) && cmd) {
			msmsdcc_do_cmdirq(host, status);
		}

		if (data) {
			/* Check for data errors */
			if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT|
				      MCI_TXUNDERRUN|MCI_RXOVERRUN)) {
				msmsdcc_data_err(host, data, status);
				host->curr.data_xfered = 0;
				if (host->dma.sg && host->is_dma_mode)
					msm_dmov_stop_cmd(host->dma.channel,
							  &host->dma.hdr, 0);
				else if (host->sps.sg && host->is_sps_mode) {
					/* Stop current SPS transfer */
					msmsdcc_sps_exit_curr_xfer(host);
				}
				else {
					msmsdcc_reset_and_restore(host);
					if (host->curr.data)
						msmsdcc_stop_data(host);
					if (!data->stop)
						timer |=
						 msmsdcc_request_end(host,
								    data->mrq);
					else {
						msmsdcc_start_command(host,
								     data->stop,
								     0);
						timer = 1;
					}
				}
			}

			/* Check for data done */
			if (!host->curr.got_dataend && (status & MCI_DATAEND))
				host->curr.got_dataend = 1;

			if (host->curr.got_dataend) {
				/*
				 * If DMA is still in progress, we complete
				 * via the completion handler
				 */
				if (!host->dma.busy && !host->sps.busy) {
					/*
					 * There appears to be an issue in the
					 * controller where if you request a
					 * small block transfer (< fifo size),
					 * you may get your DATAEND/DATABLKEND
					 * irq without the PIO data irq.
					 *
					 * Check to see if theres still data
					 * to be read, and simulate a PIO irq.
					 */
					if (data->flags & MMC_DATA_READ)
						msmsdcc_wait_for_rxdata(host,
								data);
					msmsdcc_stop_data(host);
					if (!data->error) {
						host->curr.data_xfered =
							host->curr.xfer_size;
						host->curr.xfer_remain -=
							host->curr.xfer_size;
					}

					if (!data->stop)
						timer |= msmsdcc_request_end(
							  host, data->mrq);
					else {
						msmsdcc_start_command(host,
							      data->stop, 0);
						timer = 1;
					}
				}
			}
		}

		ret = 1;
	} while (status);

	spin_unlock(&host->lock);

	return IRQ_RETVAL(ret);
}

static void
msmsdcc_request_start(struct msmsdcc_host *host, struct mmc_request *mrq)
{
	if (mrq->data && mrq->data->flags & MMC_DATA_READ) {
		/* Queue/read data, daisy-chain command when data starts */
		msmsdcc_start_data(host, mrq->data, mrq->cmd, 0);
	} else {
		msmsdcc_start_command(host, mrq->cmd, 0);
	}
}

static void
msmsdcc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msmsdcc_host *host = mmc_priv(mmc);
	unsigned long		flags;

	WARN_ON(host->curr.mrq != NULL);

        WARN_ON(host->pwr == 0);

	spin_lock_irqsave(&host->lock, flags);
	/*
	 * Enable clocks for SDIO clients if they are already turned off
	 * as part of their low-power management.
	 */
	if (mmc->card && (mmc->card->type == MMC_TYPE_SDIO) && !host->clks_on) {
		mmc->ios.clock = host->clk_rate;
		spin_unlock(&host->lock);
		mmc->ops->set_ios(host->mmc, &host->mmc->ios);
		spin_lock(&host->lock);
	}

	if (host->eject) {
		if (mrq->data && !(mrq->data->flags & MMC_DATA_READ)) {
			mrq->cmd->error = 0;
			mrq->data->bytes_xfered = mrq->data->blksz *
						  mrq->data->blocks;
		} else
			mrq->cmd->error = -ENOMEDIUM;

		spin_unlock_irqrestore(&host->lock, flags);
		mmc_request_done(mmc, mrq);
		return;
	}

	host->curr.mrq = mrq;

	if (host->plat->dummy52_required) {
		if (host->dummy_52_needed) {
			host->dummy_52_state = DUMMY_52_STATE_SENT;
			msmsdcc_start_command(host, &dummy52cmd,
					      MCI_CPSM_PROGENA);
			spin_unlock_irqrestore(&host->lock, flags);
			if (mrq->data && mrq->data->flags == MMC_DATA_WRITE) {
				if (mrq->cmd->opcode == SD_IO_RW_EXTENDED ||
					mrq->cmd->opcode == 54)
					host->dummy_52_needed = 1;
			} else {
				host->dummy_52_needed = 0;
			}
			return;
		}
		if (mrq->data && mrq->data->flags == MMC_DATA_WRITE) {
			if (mrq->cmd->opcode == SD_IO_RW_EXTENDED ||
				mrq->cmd->opcode == 54)
				host->dummy_52_needed = 1;
		}
	}
	msmsdcc_request_start(host, mrq);
	spin_unlock_irqrestore(&host->lock, flags);
}

static inline int msmsdcc_is_pwrsave(struct msmsdcc_host *host)
{
	if (host->clk_rate > 400000 && msmsdcc_pwrsave)
		return 1;
	return 0;
}

static void
msmsdcc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msmsdcc_host *host = mmc_priv(mmc);
	u32 clk = 0, pwr = 0;
	int rc;
	unsigned long flags;

	DBG(host, "ios->clock = %u\n", ios->clock);

	if (ios->clock) {

		spin_lock_irqsave(&host->lock, flags);
		if (!host->clks_on) {
			if (!IS_ERR_OR_NULL(host->dfab_pclk))
				clk_enable(host->dfab_pclk);
			if (!IS_ERR(host->pclk))
				clk_enable(host->pclk);
			clk_enable(host->clk);
			host->clks_on = 1;
			if (mmc->card && mmc->card->type == MMC_TYPE_SDIO) {
				if (!host->plat->sdiowakeup_irq) {
					writel(host->mci_irqenable,
							host->base + MMCIMASK0);
					if (host->plat->cfg_mpm_sdiowakeup &&
					(mmc->pm_flags & MMC_PM_WAKE_SDIO_IRQ))
						host->plat->cfg_mpm_sdiowakeup(
							mmc_dev(mmc), 0);
					disable_irq_wake(
						host->core_irqres->start);
				} else if (!(mmc->pm_flags &
							MMC_PM_WAKE_SDIO_IRQ)) {
					writel(host->mci_irqenable,
							host->base + MMCIMASK0);
				}
			}
		}
		spin_unlock_irqrestore(&host->lock, flags);

		if ((ios->clock < host->plat->msmsdcc_fmax) &&
				(ios->clock > host->plat->msmsdcc_fmid))
			ios->clock = host->plat->msmsdcc_fmid;

		if (ios->clock != host->clk_rate) {
			rc = clk_set_rate(host->clk, ios->clock);
			WARN_ON(rc < 0);
			host->clk_rate = ios->clock;
		}
		/*
		 * give atleast 2 MCLK cycles delay for clocks
		 * and SDCC core to stabilize
		 */
		msmsdcc_delay(host);
		clk |= MCI_CLK_ENABLE;
	}

	if (ios->bus_width == MMC_BUS_WIDTH_8)
		clk |= MCI_CLK_WIDEBUS_8;
	else if (ios->bus_width == MMC_BUS_WIDTH_4)
		clk |= MCI_CLK_WIDEBUS_4;
	else
		clk |= MCI_CLK_WIDEBUS_1;

	if (msmsdcc_is_pwrsave(host))
		clk |= MCI_CLK_PWRSAVE;

	clk |= MCI_CLK_FLOWENA;
	clk |= MCI_CLK_SELECTIN; /* feedback clock */

	if (host->plat->translate_vdd)
		pwr |= host->plat->translate_vdd(mmc_dev(mmc), ios->vdd);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		htc_pwrsink_set(PWRSINK_SDCARD, 0);
		if (!host->sdcc_irq_disabled) {
			disable_irq(host->core_irqres->start);
			host->sdcc_irq_disabled = 1;
		}
		break;
	case MMC_POWER_UP:
		pwr |= MCI_PWR_UP;
		if (host->sdcc_irq_disabled) {
			enable_irq(host->core_irqres->start);
			host->sdcc_irq_disabled = 0;
		}
		break;
	case MMC_POWER_ON:
		htc_pwrsink_set(PWRSINK_SDCARD, 100);
		pwr |= MCI_PWR_ON;
		break;
	}

	writel(clk, host->base + MMCICLOCK);

	udelay(50);

	if (host->pwr != pwr) {
		host->pwr = pwr;
		writel(pwr, host->base + MMCIPOWER);
	}

	spin_lock_irqsave(&host->lock, flags);
	if (!(clk & MCI_CLK_ENABLE) && host->clks_on) {
		if (mmc->card && mmc->card->type == MMC_TYPE_SDIO) {
			if (!host->plat->sdiowakeup_irq) {
				writel(MCI_SDIOINTMASK, host->base + MMCIMASK0);
				WARN_ON(host->sdcc_irq_disabled);
				if (host->plat->cfg_mpm_sdiowakeup &&
					(mmc->pm_flags & MMC_PM_WAKE_SDIO_IRQ))
					host->plat->cfg_mpm_sdiowakeup(
							mmc_dev(mmc), 1);
				enable_irq_wake(host->core_irqres->start);
			} else if (mmc->pm_flags & MMC_PM_WAKE_SDIO_IRQ) {
				writel(0, host->base + MMCIMASK0);
			} else {
				writel(MCI_SDIOINTMASK, host->base + MMCIMASK0);
			}
			msmsdcc_delay(host);
		}
		clk_disable(host->clk);
		if (!IS_ERR(host->pclk))
			clk_disable(host->pclk);
		if (!IS_ERR_OR_NULL(host->dfab_pclk))
			clk_disable(host->dfab_pclk);
		host->clks_on = 0;
	}
	spin_unlock_irqrestore(&host->lock, flags);
}

int msmsdcc_set_pwrsave(struct mmc_host *mmc, int pwrsave)
{
	struct msmsdcc_host *host = mmc_priv(mmc);
	u32 clk;

	clk = readl(host->base + MMCICLOCK);
	pr_debug("Changing to pwr_save=%d", pwrsave);
	if (pwrsave && msmsdcc_is_pwrsave(host))
		clk |= MCI_CLK_PWRSAVE;
	else
		clk &= ~MCI_CLK_PWRSAVE;
	writel(clk, host->base + MMCICLOCK);

	return 0;
}

static int msmsdcc_get_ro(struct mmc_host *mmc)
{
	int wpswitch_status = -ENOSYS;
	struct msmsdcc_host *host = mmc_priv(mmc);

	if (host->plat->wpswitch) {
		wpswitch_status = host->plat->wpswitch(mmc_dev(mmc));
		if (wpswitch_status < 0)
			wpswitch_status = -ENOSYS;
	}
	pr_debug("%s: Card read-only status %d\n", __func__, wpswitch_status);
	return wpswitch_status;
}

#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
static void msmsdcc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct msmsdcc_host *host = mmc_priv(mmc);
	unsigned long flags;

	if (enable) {
		spin_lock_irqsave(&host->lock, flags);
		host->mci_irqenable |= MCI_SDIOINTOPERMASK;
		writel(readl(host->base + MMCIMASK0) | MCI_SDIOINTOPERMASK,
			       host->base + MMCIMASK0);
		spin_unlock_irqrestore(&host->lock, flags);
	} else {
		host->mci_irqenable &= ~MCI_SDIOINTOPERMASK;
		writel(readl(host->base + MMCIMASK0) & ~MCI_SDIOINTOPERMASK,
		       host->base + MMCIMASK0);
	}
}
#endif /* CONFIG_MMC_MSM_SDIO_SUPPORT */

#ifdef CONFIG_PM_RUNTIME
static int msmsdcc_enable(struct mmc_host *mmc)
{
	int rc;
	struct device *dev = mmc->parent;

	if (atomic_read(&dev->power.usage_count) > 0) {
		pm_runtime_get_noresume(dev);
		goto out;
	}

	rc = pm_runtime_get_sync(dev);

	if (rc < 0) {
		pr_info("%s: %s: failed with error %d", mmc_hostname(mmc),
				__func__, rc);
		return rc;
	}
out:
	return 0;
}

static int msmsdcc_disable(struct mmc_host *mmc, int lazy)
{
	int rc;

	if (mmc->card && mmc->card->type == MMC_TYPE_SDIO)
		return -ENOTSUPP;

	rc = pm_runtime_put_sync(mmc->parent);

	if (rc < 0)
		pr_info("%s: %s: failed with error %d", mmc_hostname(mmc),
				__func__, rc);
	return rc;
}
#else
#define msmsdcc_enable NULL
#define msmsdcc_disable NULL
#endif

static const struct mmc_host_ops msmsdcc_ops = {
	.enable		= msmsdcc_enable,
	.disable	= msmsdcc_disable,
	.request	= msmsdcc_request,
	.set_ios	= msmsdcc_set_ios,
	.get_ro		= msmsdcc_get_ro,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	.enable_sdio_irq = msmsdcc_enable_sdio_irq,
#endif
};

static void
msmsdcc_check_status(unsigned long data)
{
	struct msmsdcc_host *host = (struct msmsdcc_host *)data;
	unsigned int status;

	if (!host->plat->status) {
		mmc_detect_change(host->mmc, 0);
	} else {
		status = host->plat->status(mmc_dev(host->mmc));
		host->eject = !status;
		if (status ^ host->oldstat) {
			pr_info("%s: Slot status change detected (%d -> %d)\n",
			       mmc_hostname(host->mmc), host->oldstat, status);
			mmc_detect_change(host->mmc, 0);
		}
		host->oldstat = status;
	}
}

static irqreturn_t
msmsdcc_platform_status_irq(int irq, void *dev_id)
{
	struct msmsdcc_host *host = dev_id;

	pr_debug("%s: %d\n", __func__, irq);
	msmsdcc_check_status((unsigned long) host);
	return IRQ_HANDLED;
}

static irqreturn_t
msmsdcc_platform_sdiowakeup_irq(int irq, void *dev_id)
{
	struct msmsdcc_host	*host = dev_id;

	pr_info("%s: SDIO Wake up IRQ : %d\n", __func__, irq);
	spin_lock(&host->lock);
	if (!host->sdio_irq_disabled) {
		disable_irq_nosync(irq);
		if (host->mmc->pm_flags & MMC_PM_WAKE_SDIO_IRQ) {
			wake_lock(&host->sdio_wlock);
			disable_irq_wake(irq);
		}
		host->sdio_irq_disabled = 1;
	}
	spin_unlock(&host->lock);

	return IRQ_HANDLED;
}

static void
msmsdcc_status_notify_cb(int card_present, void *dev_id)
{
	struct msmsdcc_host *host = dev_id;

	pr_debug("%s: card_present %d\n", mmc_hostname(host->mmc),
	       card_present);
	msmsdcc_check_status((unsigned long) host);
}

static int
msmsdcc_init_dma(struct msmsdcc_host *host)
{
	memset(&host->dma, 0, sizeof(struct msmsdcc_dma_data));
	host->dma.host = host;
	host->dma.channel = -1;

	if (!host->dmares)
		return -ENODEV;

	host->dma.nc = dma_alloc_coherent(NULL,
					  sizeof(struct msmsdcc_nc_dmadata),
					  &host->dma.nc_busaddr,
					  GFP_KERNEL);
	if (host->dma.nc == NULL) {
		pr_err("Unable to allocate DMA buffer\n");
		return -ENOMEM;
	}
	memset(host->dma.nc, 0x00, sizeof(struct msmsdcc_nc_dmadata));
	host->dma.cmd_busaddr = host->dma.nc_busaddr;
	host->dma.cmdptr_busaddr = host->dma.nc_busaddr +
				offsetof(struct msmsdcc_nc_dmadata, cmdptr);
	host->dma.channel = host->dmares->start;

	return 0;
}

#ifdef CONFIG_MMC_MSM_SPS_SUPPORT
/**
 * Allocate and Connect a SDCC peripheral's SPS endpoint
 *
 * This function allocates endpoint context and
 * connect it with memory endpoint by calling
 * appropriate SPS driver APIs.
 *
 * Also registers a SPS callback function with
 * SPS driver
 *
 * This function should only be called once typically
 * during driver probe.
 *
 * @host - Pointer to sdcc host structure
 * @ep   - Pointer to sps endpoint data structure
 * @is_produce - 1 means Producer endpoint
 *		 0 means Consumer endpoint
 *
 * @return - 0 if successful else negative value.
 *
 */
static int msmsdcc_sps_init_ep_conn(struct msmsdcc_host *host,
				struct msmsdcc_sps_ep_conn_data *ep,
				bool is_producer)
{
	int rc = 0;
	struct sps_pipe *sps_pipe_handle;
	struct sps_connect *sps_config = &ep->config;
	struct sps_register_event *sps_event = &ep->event;

	/* Allocate endpoint context */
	sps_pipe_handle = sps_alloc_endpoint();
	if (!sps_pipe_handle) {
		pr_err("%s: sps_alloc_endpoint() failed!!! is_producer=%d",
			   mmc_hostname(host->mmc), is_producer);
		rc = -ENOMEM;
		goto out;
	}

	/* Get default connection configuration for an endpoint */
	rc = sps_get_config(sps_pipe_handle, sps_config);
	if (rc) {
		pr_err("%s: sps_get_config() failed!!! pipe_handle=0x%x,"
			" rc=%d", mmc_hostname(host->mmc),
			(u32)sps_pipe_handle, rc);
		goto get_config_err;
	}

	/* Modify the default connection configuration */
	if (is_producer) {
		/*
		 * For SDCC producer transfer, source should be
		 * SDCC peripheral where as destination should
		 * be system memory.
		 */
		sps_config->source = host->sps.bam_handle;
		sps_config->destination = SPS_DEV_HANDLE_MEM;
		/* Producer pipe will handle this connection */
		sps_config->mode = SPS_MODE_SRC;
		sps_config->options =
			SPS_O_AUTO_ENABLE | SPS_O_EOT | SPS_O_ACK_TRANSFERS;
	} else {
		/*
		 * For SDCC consumer transfer, source should be
		 * system memory where as destination should
		 * SDCC peripheral
		 */
		sps_config->source = SPS_DEV_HANDLE_MEM;
		sps_config->destination = host->sps.bam_handle;
		sps_config->mode = SPS_MODE_DEST;
		sps_config->options =
			SPS_O_AUTO_ENABLE | SPS_O_EOT | SPS_O_ACK_TRANSFERS;
	}

	/* Producer pipe index */
	sps_config->src_pipe_index = host->sps.src_pipe_index;
	/* Consumer pipe index */
	sps_config->dest_pipe_index = host->sps.dest_pipe_index;
	/*
	 * This event thresold value is only significant for BAM-to-BAM
	 * transfer. It's ignored for BAM-to-System mode transfer.
	 */
	sps_config->event_thresh = 0x10;
	/*
	 * Max. no of scatter/gather buffers that can
	 * be passed by block layer = 32 (NR_SG).
	 * Each BAM descritor needs 64 bits (8 bytes).
	 * One BAM descriptor is required per buffer transfer.
	 * So we would require total 256 (32 * 8) bytes of descriptor FIFO.
	 * But due to HW limitation we need to allocate atleast one extra
	 * descriptor memory (256 bytes + 8 bytes). But in order to be
	 * in power of 2, we are allocating 512 bytes of memory.
	 */
	sps_config->desc.size = 512;
	sps_config->desc.base = dma_alloc_coherent(mmc_dev(host->mmc),
						sps_config->desc.size,
						&sps_config->desc.phys_base,
						GFP_KERNEL);

	memset(sps_config->desc.base, 0x00, sps_config->desc.size);

	/* Establish connection between peripheral and memory endpoint */
	rc = sps_connect(sps_pipe_handle, sps_config);
	if (rc) {
		pr_err("%s: sps_connect() failed!!! pipe_handle=0x%x,"
			" rc=%d", mmc_hostname(host->mmc),
			(u32)sps_pipe_handle, rc);
		goto sps_connect_err;
	}

	sps_event->mode = SPS_TRIGGER_CALLBACK;
	sps_event->options = SPS_O_EOT;
	sps_event->callback = msmsdcc_sps_complete_cb;
	sps_event->xfer_done = NULL;
	sps_event->user = (void *)host;

	/* Register callback event for EOT (End of transfer) event. */
	rc = sps_register_event(sps_pipe_handle, sps_event);
	if (rc) {
		pr_err("%s: sps_connect() failed!!! pipe_handle=0x%x,"
			" rc=%d", mmc_hostname(host->mmc),
			(u32)sps_pipe_handle, rc);
		goto reg_event_err;
	}
	/* Now save the sps pipe handle */
	ep->pipe_handle = sps_pipe_handle;
	pr_debug("%s: %s, success !!! %s: pipe_handle=0x%x,"
		" desc_fifo.phys_base=0x%x\n", mmc_hostname(host->mmc),
		__func__, is_producer ? "READ" : "WRITE",
		(u32)sps_pipe_handle, sps_config->desc.phys_base);
	goto out;

reg_event_err:
	sps_disconnect(sps_pipe_handle);
sps_connect_err:
	dma_free_coherent(mmc_dev(host->mmc),
			sps_config->desc.size,
			sps_config->desc.base,
			sps_config->desc.phys_base);
get_config_err:
	sps_free_endpoint(sps_pipe_handle);
out:
	return rc;
}

/**
 * Disconnect and Deallocate a SDCC peripheral's SPS endpoint
 *
 * This function disconnect endpoint and deallocates
 * endpoint context.
 *
 * This function should only be called once typically
 * during driver remove.
 *
 * @host - Pointer to sdcc host structure
 * @ep   - Pointer to sps endpoint data structure
 *
 */
static void msmsdcc_sps_exit_ep_conn(struct msmsdcc_host *host,
				struct msmsdcc_sps_ep_conn_data *ep)
{
	struct sps_pipe *sps_pipe_handle = ep->pipe_handle;
	struct sps_connect *sps_config = &ep->config;
	struct sps_register_event *sps_event = &ep->event;

	sps_event->xfer_done = NULL;
	sps_event->callback = NULL;
	sps_register_event(sps_pipe_handle, sps_event);
	sps_disconnect(sps_pipe_handle);
	dma_free_coherent(mmc_dev(host->mmc),
			sps_config->desc.size,
			sps_config->desc.base,
			sps_config->desc.phys_base);
	sps_free_endpoint(sps_pipe_handle);
}

/**
 * Reset SDCC peripheral's SPS endpoint
 *
 * This function disconnects an endpoint.
 *
 * This function should be called for reseting
 * SPS endpoint when data transfer error is
 * encountered during data transfer. This
 * can be considered as soft reset to endpoint.
 *
 * This function should only be called if
 * msmsdcc_sps_init() is already called.
 *
 * @host - Pointer to sdcc host structure
 * @ep   - Pointer to sps endpoint data structure
 *
 * @return - 0 if successful else negative value.
 */
static int msmsdcc_sps_reset_ep(struct msmsdcc_host *host,
				struct msmsdcc_sps_ep_conn_data *ep)
{
	int rc = 0;
	struct sps_pipe *sps_pipe_handle = ep->pipe_handle;

	rc = sps_disconnect(sps_pipe_handle);
	if (rc) {
		pr_err("%s: %s: sps_disconnect() failed!!! pipe_handle=0x%x,"
			" rc=%d", mmc_hostname(host->mmc), __func__,
			(u32)sps_pipe_handle, rc);
		goto out;
	}
 out:
	return rc;
}

/**
 * Restore SDCC peripheral's SPS endpoint
 *
 * This function connects an endpoint.
 *
 * This function should be called for restoring
 * SPS endpoint after data transfer error is
 * encountered during data transfer. This
 * can be considered as soft reset to endpoint.
 *
 * This function should only be called if
 * msmsdcc_sps_reset_ep() is called before.
 *
 * @host - Pointer to sdcc host structure
 * @ep   - Pointer to sps endpoint data structure
 *
 * @return - 0 if successful else negative value.
 */
static int msmsdcc_sps_restore_ep(struct msmsdcc_host *host,
				struct msmsdcc_sps_ep_conn_data *ep)
{
	int rc = 0;
	struct sps_pipe *sps_pipe_handle = ep->pipe_handle;
	struct sps_connect *sps_config = &ep->config;
	struct sps_register_event *sps_event = &ep->event;

	/* Establish connection between peripheral and memory endpoint */
	rc = sps_connect(sps_pipe_handle, sps_config);
	if (rc) {
		pr_err("%s: %s: sps_connect() failed!!! pipe_handle=0x%x,"
			" rc=%d", mmc_hostname(host->mmc), __func__,
			(u32)sps_pipe_handle, rc);
		goto out;
	}

	/* Register callback event for EOT (End of transfer) event. */
	rc = sps_register_event(sps_pipe_handle, sps_event);
	if (rc) {
		pr_err("%s: %s: sps_register_event() failed!!!"
			" pipe_handle=0x%x, rc=%d",
			mmc_hostname(host->mmc), __func__,
			(u32)sps_pipe_handle, rc);
		goto reg_event_err;
	}
	goto out;

reg_event_err:
	sps_disconnect(sps_pipe_handle);
out:
	return rc;
}

/**
 * Initialize SPS HW connected with SDCC core
 *
 * This function register BAM HW resources with
 * SPS driver and then initialize 2 SPS endpoints
 *
 * This function should only be called once typically
 * during driver probe.
 *
 * @host - Pointer to sdcc host structure
 *
 * @return - 0 if successful else negative value.
 *
 */
static int msmsdcc_sps_init(struct msmsdcc_host *host)
{
	int rc = 0;
	struct sps_bam_props bam = {0};

	host->bam_base = ioremap(host->bam_memres->start,
				resource_size(host->bam_memres));
	if (!host->bam_base) {
		pr_err("%s: BAM ioremap() failed!!! phys_addr=0x%x,"
			" size=0x%x", mmc_hostname(host->mmc),
			host->bam_memres->start,
			(host->bam_memres->end -
			host->bam_memres->start));
		rc = -ENOMEM;
		goto out;
	}

	bam.phys_addr = host->bam_memres->start;
	bam.virt_addr = host->bam_base;
	/*
	 * This event thresold value is only significant for BAM-to-BAM
	 * transfer. It's ignored for BAM-to-System mode transfer.
	 */
	bam.event_threshold = 0x10;	/* Pipe event threshold */
	/*
	 * This threshold controls when the BAM publish
	 * the descriptor size on the sideband interface.
	 * SPS HW will only be used when
	 * data transfer size >  MCI_FIFOSIZE (64 bytes).
	 * PIO mode will be used when
	 * data transfer size < MCI_FIFOSIZE (64 bytes).
	 * So set this thresold value to 64 bytes.
	 */
	bam.summing_threshold = 64;
	/* SPS driver wll handle the SDCC BAM IRQ */
	bam.irq = (u32)host->bam_irqres->start;
	bam.manage = SPS_BAM_MGR_LOCAL;

	pr_info("%s: bam physical base=0x%x\n", mmc_hostname(host->mmc),
			(u32)bam.phys_addr);
	pr_info("%s: bam virtual base=0x%x\n", mmc_hostname(host->mmc),
			(u32)bam.virt_addr);

	/* Register SDCC Peripheral BAM device to SPS driver */
	rc = sps_register_bam_device(&bam, &host->sps.bam_handle);
	if (rc) {
		pr_err("%s: sps_register_bam_device() failed!!! err=%d",
			   mmc_hostname(host->mmc), rc);
		goto reg_bam_err;
	}
	pr_info("%s: BAM device registered. bam_handle=0x%x",
		mmc_hostname(host->mmc), host->sps.bam_handle);

	host->sps.src_pipe_index = SPS_SDCC_PRODUCER_PIPE_INDEX;
	host->sps.dest_pipe_index = SPS_SDCC_CONSUMER_PIPE_INDEX;

	rc = msmsdcc_sps_init_ep_conn(host, &host->sps.prod,
					SPS_PROD_PERIPHERAL);
	if (rc)
		goto sps_reset_err;
	rc = msmsdcc_sps_init_ep_conn(host, &host->sps.cons,
					SPS_CONS_PERIPHERAL);
	if (rc)
		goto cons_conn_err;

	pr_info("%s: Qualcomm MSM SDCC-BAM at 0x%016llx irq %d\n",
		mmc_hostname(host->mmc),
		(unsigned long long)host->bam_memres->start,
		(unsigned int)host->bam_irqres->start);
	goto out;

cons_conn_err:
	msmsdcc_sps_exit_ep_conn(host, &host->sps.prod);
sps_reset_err:
	sps_deregister_bam_device(host->sps.bam_handle);
reg_bam_err:
	iounmap(host->bam_base);
out:
	return rc;
}

/**
 * De-initialize SPS HW connected with SDCC core
 *
 * This function deinitialize SPS endpoints and then
 * deregisters BAM resources from SPS driver.
 *
 * This function should only be called once typically
 * during driver remove.
 *
 * @host - Pointer to sdcc host structure
 *
 */
static void msmsdcc_sps_exit(struct msmsdcc_host *host)
{
	msmsdcc_sps_exit_ep_conn(host, &host->sps.cons);
	msmsdcc_sps_exit_ep_conn(host, &host->sps.prod);
	sps_deregister_bam_device(host->sps.bam_handle);
	iounmap(host->bam_base);
}
#else
static inline int msmsdcc_sps_init_ep_conn(struct msmsdcc_host *host,
				struct msmsdcc_sps_ep_conn_data *ep,
				bool is_producer) { return 0; }
static inline void msmsdcc_sps_exit_ep_conn(struct msmsdcc_host *host,
				struct msmsdcc_sps_ep_conn_data *ep) { }
static inline int msmsdcc_sps_reset_ep(struct msmsdcc_host *host,
				struct msmsdcc_sps_ep_conn_data *ep)
{
	return 0;
}
static inline int msmsdcc_sps_restore_ep(struct msmsdcc_host *host,
				struct msmsdcc_sps_ep_conn_data *ep)
{
	return 0;
}
static inline int msmsdcc_sps_init(struct msmsdcc_host *host) { return 0; }
static inline void msmsdcc_sps_exit(struct msmsdcc_host *host) {}
#endif /* CONFIG_MMC_MSM_SPS_SUPPORT */

static ssize_t
show_polling(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msmsdcc_host *host = mmc_priv(mmc);
	int poll;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	poll = !!(mmc->caps & MMC_CAP_NEEDS_POLL);
	spin_unlock_irqrestore(&host->lock, flags);

	return snprintf(buf, PAGE_SIZE, "%d\n", poll);
}

static ssize_t
set_polling(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msmsdcc_host *host = mmc_priv(mmc);
	int value;
	unsigned long flags;

	sscanf(buf, "%d", &value);

	spin_lock_irqsave(&host->lock, flags);
	if (value) {
		mmc->caps |= MMC_CAP_NEEDS_POLL;
		mmc_detect_change(host->mmc, 0);
	} else {
		mmc->caps &= ~MMC_CAP_NEEDS_POLL;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	host->polling_enabled = mmc->caps & MMC_CAP_NEEDS_POLL;
#endif
	spin_unlock_irqrestore(&host->lock, flags);
	return count;
}

static DEVICE_ATTR(polling, S_IRUGO | S_IWUSR,
		show_polling, set_polling);
static struct attribute *dev_attrs[] = {
	&dev_attr_polling.attr,
	NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void msmsdcc_early_suspend(struct early_suspend *h)
{
	struct msmsdcc_host *host =
		container_of(h, struct msmsdcc_host, early_suspend);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->polling_enabled = host->mmc->caps & MMC_CAP_NEEDS_POLL;
	host->mmc->caps &= ~MMC_CAP_NEEDS_POLL;
	spin_unlock_irqrestore(&host->lock, flags);
};
static void msmsdcc_late_resume(struct early_suspend *h)
{
	struct msmsdcc_host *host =
		container_of(h, struct msmsdcc_host, early_suspend);
	unsigned long flags;

	if (host->polling_enabled) {
		spin_lock_irqsave(&host->lock, flags);
		host->mmc->caps |= MMC_CAP_NEEDS_POLL;
		mmc_detect_change(host->mmc, 0);
		spin_unlock_irqrestore(&host->lock, flags);
	}
};
#endif

static void msmsdcc_req_tout_timer_hdlr(unsigned long data)
{
	struct msmsdcc_host *host = (struct msmsdcc_host *)data;
	struct mmc_request *mrq;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	if ((host->plat->dummy52_required) &&
		(host->dummy_52_state == DUMMY_52_STATE_SENT)) {
		pr_info("%s: %s: dummy CMD52 timeout\n",
				mmc_hostname(host->mmc), __func__);
		host->dummy_52_state = DUMMY_52_STATE_NONE;
	}

	mrq = host->curr.mrq;

	if (mrq && mrq->cmd) {
		pr_info("%s: %s CMD%d\n", mmc_hostname(host->mmc),
				__func__, mrq->cmd->opcode);
		if (!mrq->cmd->error)
			mrq->cmd->error = -ETIMEDOUT;
		if (host->plat->dummy52_required && host->dummy_52_needed)
			host->dummy_52_needed = 0;
		if (host->curr.data) {
			pr_info("%s: %s Request timeout\n",
					mmc_hostname(host->mmc), __func__);
			if (mrq->data && !mrq->data->error)
				mrq->data->error = -ETIMEDOUT;
			host->curr.data_xfered = 0;
			if (host->dma.sg && host->is_dma_mode) {
				msm_dmov_stop_cmd(host->dma.channel,
						&host->dma.hdr, 0);
			} else if (host->sps.sg && host->is_sps_mode) {
				/* Stop current SPS transfer */
				msmsdcc_sps_exit_curr_xfer(host);
			} else {
				msmsdcc_reset_and_restore(host);
				msmsdcc_stop_data(host);
				if (mrq->data && mrq->data->stop)
					msmsdcc_start_command(host,
							mrq->data->stop, 0);
				else
					msmsdcc_request_end(host, mrq);
			}
		} else {
			if (host->prog_enable) {
				host->prog_scan = 0;
				host->prog_enable = 0;
			}
			msmsdcc_reset_and_restore(host);
			msmsdcc_request_end(host, mrq);
		}
	}
	spin_unlock_irqrestore(&host->lock, flags);
}

static int
msmsdcc_probe(struct platform_device *pdev)
{
	struct mmc_platform_data *plat = pdev->dev.platform_data;
	struct msmsdcc_host *host;
	struct mmc_host *mmc;
	unsigned long flags;
	struct resource *core_irqres = NULL;
	struct resource *bam_irqres = NULL;
	struct resource *core_memres = NULL;
	struct resource *dml_memres = NULL;
	struct resource *bam_memres = NULL;
	struct resource *dmares = NULL;
	int ret;
	int i;

	/* must have platform data */
	if (!plat) {
		pr_err("%s: Platform data not available\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (pdev->id < 1 || pdev->id > 5)
		return -EINVAL;

	if (pdev->resource == NULL || pdev->num_resources < 2) {
		pr_err("%s: Invalid resource\n", __func__);
		return -ENXIO;
	}

	for (i = 0; i < pdev->num_resources; i++) {
		if (pdev->resource[i].flags & IORESOURCE_MEM) {
			if (!strcmp(pdev->resource[i].name,
					"sdcc_dml_addr"))
				dml_memres = &pdev->resource[i];
			else if (!strcmp(pdev->resource[i].name,
					"sdcc_bam_addr"))
				bam_memres = &pdev->resource[i];
			else
				core_memres = &pdev->resource[i];

		}
		if (pdev->resource[i].flags & IORESOURCE_IRQ) {
			if (!strcmp(pdev->resource[i].name,
					"sdcc_bam_irq"))
				bam_irqres = &pdev->resource[i];
			else
				core_irqres = &pdev->resource[i];
		}
		if (pdev->resource[i].flags & IORESOURCE_DMA)
			dmares = &pdev->resource[i];
	}

	if (!core_irqres || !core_memres) {
		pr_err("%s: Invalid sdcc core resource\n", __func__);
		return -ENXIO;
	}

	/*
	 * Both BAM and DML memory resource should be preset.
	 * BAM IRQ resource should also be present.
	 */
	if ((bam_memres && !dml_memres) ||
		(!bam_memres && dml_memres) ||
		((bam_memres && dml_memres) && !bam_irqres)) {
		pr_err("%s: Invalid sdcc BAM/DML resource\n", __func__);
		return -ENXIO;
	}

	/*
	 * Setup our host structure
	 */
	mmc = mmc_alloc_host(sizeof(struct msmsdcc_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	host = mmc_priv(mmc);
	host->pdev_id = pdev->id;
	host->plat = plat;
	host->mmc = mmc;
	host->curr.cmd = NULL;
	if (bam_memres && dml_memres && bam_irqres)
		host->is_sps_mode = 1;
	else if (dmares)
		host->is_dma_mode = 1;

	host->base = ioremap(core_memres->start,
			resource_size(core_memres));
	if (!host->base) {
		ret = -ENOMEM;
		goto host_free;
	}

	host->core_irqres = core_irqres;
	host->bam_irqres = bam_irqres;
	host->core_memres = core_memres;
	host->dml_memres = dml_memres;
	host->bam_memres = bam_memres;
	host->dmares = dmares;
	spin_lock_init(&host->lock);

#ifdef CONFIG_MMC_EMBEDDED_SDIO
	if (plat->embedded_sdio)
		mmc_set_embedded_sdio_data(mmc,
					   &plat->embedded_sdio->cis,
					   &plat->embedded_sdio->cccr,
					   plat->embedded_sdio->funcs,
					   plat->embedded_sdio->num_funcs);
#endif

	tasklet_init(&host->dma_tlet, msmsdcc_dma_complete_tlet,
			(unsigned long)host);

	tasklet_init(&host->sps.tlet, msmsdcc_sps_complete_tlet,
			(unsigned long)host);
	if (host->is_dma_mode) {
		/* Setup DMA */
		ret = msmsdcc_init_dma(host);
		if (ret)
			goto ioremap_free;
	} else {
		host->dma.channel = -1;
	}

	/*
	 * Setup SDCC clock if derived from Dayatona
	 * fabric core clock.
	 */
	if (plat->pclk_src_dfab) {
		host->dfab_pclk = clk_get(&pdev->dev, "dfab_sdc_clk");
		if (!IS_ERR(host->dfab_pclk)) {
			/* Set the clock rate to 64MHz for max. performance */
			ret = clk_set_rate(host->dfab_pclk, 64000000);
			if (ret)
				goto dfab_pclk_put;
			ret = clk_enable(host->dfab_pclk);
			if (ret)
				goto dfab_pclk_put;
		} else
			goto dma_free;
	}

	/*
	 * Setup main peripheral bus clock
	 */
	host->pclk = clk_get(&pdev->dev, "sdc_pclk");
	if (!IS_ERR(host->pclk)) {
		ret = clk_enable(host->pclk);
		if (ret)
			goto pclk_put;

		host->pclk_rate = clk_get_rate(host->pclk);
	}

	/*
	 * Setup SDC MMC clock
	 */
	host->clk = clk_get(&pdev->dev, "sdc_clk");
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		goto pclk_disable;
	}

	ret = clk_set_rate(host->clk, plat->msmsdcc_fmin);
	if (ret) {
		pr_err("%s: Clock rate set failed (%d)\n", __func__, ret);
		goto clk_put;
	}

	ret = clk_enable(host->clk);
	if (ret)
		goto clk_put;

	host->clk_rate = clk_get_rate(host->clk);

	host->clks_on = 1;

	/* Clocks has to be running before accessing SPS/DML HW blocks */
	if (host->is_sps_mode) {
		/* Initialize SPS */
		ret = msmsdcc_sps_init(host);
		if (ret)
			goto clk_disable;
		/* Initialize DML */
		ret = msmsdcc_dml_init(host);
		if (ret)
			goto sps_exit;
	}

	/*
	 * Setup MMC host structure
	 */
	mmc->ops = &msmsdcc_ops;
	mmc->f_min = plat->msmsdcc_fmin;
	mmc->f_max = plat->msmsdcc_fmax;
	mmc->ocr_avail = plat->ocr_mask;
	mmc->pm_caps |= MMC_PM_KEEP_POWER | MMC_PM_WAKE_SDIO_IRQ;
	mmc->caps |= plat->mmc_bus_width;

	mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;

	if (plat->nonremovable)
		mmc->caps |= MMC_CAP_NONREMOVABLE;
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	mmc->caps |= MMC_CAP_SDIO_IRQ;
#endif

	mmc->max_phys_segs = NR_SG;
	mmc->max_hw_segs = NR_SG;
	mmc->max_blk_size = 4096;	/* MCI_DATA_CTL BLOCKSIZE up to 4096 */
	mmc->max_blk_count = 65535;

	mmc->max_req_size = 33554432;	/* MCI_DATA_LENGTH is 25 bits */
	mmc->max_seg_size = mmc->max_req_size;

	writel(0, host->base + MMCIMASK0);
	writel(MCI_CLEAR_STATIC_MASK, host->base + MMCICLEAR);

	/* Delay needed (MMCIMASK0 was just written above) */
	msmsdcc_delay(host);
	writel(MCI_IRQENABLE, host->base + MMCIMASK0);
	host->mci_irqenable = MCI_IRQENABLE;

	ret = request_irq(core_irqres->start, msmsdcc_irq, IRQF_SHARED,
			  DRIVER_NAME " (cmd)", host);
	if (ret) {
		if (host->is_sps_mode)
			goto dml_exit;
		else
			goto clk_disable;
	}

	ret = request_irq(core_irqres->start, msmsdcc_pio_irq, IRQF_SHARED,
			  DRIVER_NAME " (pio)", host);
	if (ret)
		goto irq_free;

	/*
	 * Enable SDCC IRQ only when host is powered on. Otherwise, this
	 * IRQ is un-necessarily being monitored by MPM (Modem power
	 * management block) during idle-power collapse.  The MPM will be
	 * configured to monitor the DATA1 GPIO line with level-low trigger
	 * and thus depending on the GPIO status, it prevents TCXO shutdown
	 * during idle-power collapse.
	 */
	disable_irq(core_irqres->start);
	host->sdcc_irq_disabled = 1;

	if (plat->sdiowakeup_irq) {
		wake_lock_init(&host->sdio_wlock, WAKE_LOCK_SUSPEND,
				mmc_hostname(mmc));
		ret = request_irq(plat->sdiowakeup_irq,
			msmsdcc_platform_sdiowakeup_irq,
			IRQF_SHARED | IRQF_TRIGGER_LOW,
			DRIVER_NAME "sdiowakeup", host);
		if (ret) {
			pr_err("Unable to get sdio wakeup IRQ %d (%d)\n",
				plat->sdiowakeup_irq, ret);
			goto pio_irq_free;
		} else {
			spin_lock_irqsave(&host->lock, flags);
			if (!host->sdio_irq_disabled) {
				disable_irq_nosync(plat->sdiowakeup_irq);
				host->sdio_irq_disabled = 1;
			}
			spin_unlock_irqrestore(&host->lock, flags);
		}
	}

	if (plat->cfg_mpm_sdiowakeup) {
		wake_lock_init(&host->sdio_wlock, WAKE_LOCK_SUSPEND,
				mmc_hostname(mmc));
	}

	wake_lock_init(&host->sdio_suspend_wlock, WAKE_LOCK_SUSPEND,
			mmc_hostname(mmc));
	/*
	 * Setup card detect change
	 */

	if (plat->status) {
		host->oldstat = plat->status(mmc_dev(host->mmc));
		host->eject = !host->oldstat;
	}

	if (plat->status_irq) {
		ret = request_threaded_irq(plat->status_irq, NULL,
				  msmsdcc_platform_status_irq,
				  plat->irq_flags,
				  DRIVER_NAME " (slot)",
				  host);
		if (ret) {
			pr_err("Unable to get slot IRQ %d (%d)\n",
			       plat->status_irq, ret);
			goto sdiowakeup_irq_free;
		}
	} else if (plat->register_status_notify) {
		plat->register_status_notify(msmsdcc_status_notify_cb, host);
	} else if (!plat->status)
		pr_err("%s: No card detect facilities available\n",
		       mmc_hostname(mmc));

	mmc_set_drvdata(pdev, mmc);

	ret = pm_runtime_set_active(&(pdev)->dev);
	if (ret < 0)
		pr_info("%s: %s: failed with error %d", mmc_hostname(mmc),
				__func__, ret);
	/*
	 * There is no notion of suspend/resume for SD/MMC/SDIO
	 * cards. So host can be suspended/resumed with out
	 * worrying about its children.
	 */
	pm_suspend_ignore_children(&(pdev)->dev, true);

	/*
	 * MMC/SD/SDIO bus suspend/resume operations are defined
	 * only for the slots that will be used for non-removable
	 * media or for all slots when CONFIG_MMC_UNSAFE_RESUME is
	 * defined. Otherwise, they simply become card removal and
	 * insertion events during suspend and resume respectively.
	 * Hence, enable run-time PM only for slots for which bus
	 * suspend/resume operations are defined.
	 */
#ifdef CONFIG_MMC_UNSAFE_RESUME
	/*
	 * If this capability is set, MMC core will enable/disable host
	 * for every claim/release operation on a host. We use this
	 * notification to increment/decrement runtime pm usage count.
	 */
	mmc->caps |= MMC_CAP_DISABLE;
	pm_runtime_enable(&(pdev)->dev);
#else
	if (mmc->caps & MMC_CAP_NONREMOVABLE) {
		mmc->caps |= MMC_CAP_DISABLE;
		pm_runtime_enable(&(pdev)->dev);
	}
#endif
	setup_timer(&host->req_tout_timer, msmsdcc_req_tout_timer_hdlr,
			(unsigned long)host);

	mmc_add_host(mmc);

#ifdef CONFIG_HAS_EARLYSUSPEND
	host->early_suspend.suspend = msmsdcc_early_suspend;
	host->early_suspend.resume  = msmsdcc_late_resume;
	host->early_suspend.level   = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&host->early_suspend);
#endif

	pr_info("%s: Qualcomm MSM SDCC-core at 0x%016llx irq %d,%d dma %d\n",
	       mmc_hostname(mmc), (unsigned long long)core_memres->start,
	       (unsigned int) core_irqres->start,
	       (unsigned int) plat->status_irq, host->dma.channel);

	pr_info("%s: 8 bit data mode %s\n", mmc_hostname(mmc),
		(mmc->caps & MMC_CAP_8_BIT_DATA ? "enabled" : "disabled"));
	pr_info("%s: 4 bit data mode %s\n", mmc_hostname(mmc),
	       (mmc->caps & MMC_CAP_4_BIT_DATA ? "enabled" : "disabled"));
	pr_info("%s: polling status mode %s\n", mmc_hostname(mmc),
	       (mmc->caps & MMC_CAP_NEEDS_POLL ? "enabled" : "disabled"));
	pr_info("%s: MMC clock %u -> %u Hz, PCLK %u Hz\n",
	       mmc_hostname(mmc), plat->msmsdcc_fmin, plat->msmsdcc_fmax,
							host->pclk_rate);
	pr_info("%s: Slot eject status = %d\n", mmc_hostname(mmc),
	       host->eject);
	pr_info("%s: Power save feature enable = %d\n",
	       mmc_hostname(mmc), msmsdcc_pwrsave);

	if (host->is_dma_mode && host->dma.channel != -1) {
		pr_info("%s: DM non-cached buffer at %p, dma_addr 0x%.8x\n",
		       mmc_hostname(mmc), host->dma.nc, host->dma.nc_busaddr);
		pr_info("%s: DM cmd busaddr 0x%.8x, cmdptr busaddr 0x%.8x\n",
		       mmc_hostname(mmc), host->dma.cmd_busaddr,
		       host->dma.cmdptr_busaddr);
	} else if (host->is_sps_mode) {
		pr_info("%s: SPS-BAM data transfer mode available\n",
			mmc_hostname(mmc));
	} else
		pr_info("%s: PIO transfer enabled\n", mmc_hostname(mmc));

#if defined(CONFIG_DEBUG_FS)
	msmsdcc_dbg_createhost(host);
#endif
	if (!plat->status_irq) {
		ret = sysfs_create_group(&pdev->dev.kobj, &dev_attr_grp);
		if (ret)
			goto platform_irq_free;
	}
	return 0;

 platform_irq_free:
	del_timer_sync(&host->req_tout_timer);
	pm_runtime_disable(&(pdev)->dev);
	pm_runtime_set_suspended(&(pdev)->dev);

	if (plat->status_irq)
		free_irq(plat->status_irq, host);
 sdiowakeup_irq_free:
	wake_lock_destroy(&host->sdio_suspend_wlock);
	if (plat->sdiowakeup_irq)
		free_irq(plat->sdiowakeup_irq, host);
 pio_irq_free:
	if (plat->sdiowakeup_irq)
		wake_lock_destroy(&host->sdio_wlock);
	free_irq(core_irqres->start, host);
 irq_free:
	free_irq(core_irqres->start, host);
 dml_exit:
	if (host->is_sps_mode)
		msmsdcc_dml_exit(host);
 sps_exit:
	if (host->is_sps_mode)
		msmsdcc_sps_exit(host);
 clk_disable:
	clk_disable(host->clk);
 clk_put:
	clk_put(host->clk);
 pclk_disable:
	if (!IS_ERR(host->pclk))
		clk_disable(host->pclk);
 pclk_put:
	if (!IS_ERR(host->pclk))
		clk_put(host->pclk);
	if (!IS_ERR_OR_NULL(host->dfab_pclk))
		clk_disable(host->dfab_pclk);
 dfab_pclk_put:
	if (!IS_ERR_OR_NULL(host->dfab_pclk))
		clk_put(host->dfab_pclk);
 dma_free:
	if (host->is_dma_mode) {
		if (host->dmares)
			dma_free_coherent(NULL,
				sizeof(struct msmsdcc_nc_dmadata),
				host->dma.nc, host->dma.nc_busaddr);
	}
 ioremap_free:
	iounmap(host->base);
 host_free:
	mmc_free_host(mmc);
 out:
	return ret;
}

static int msmsdcc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = mmc_get_drvdata(pdev);
	struct mmc_platform_data *plat;
	struct msmsdcc_host *host;

	if (!mmc)
		return -ENXIO;

	if (pm_runtime_suspended(&(pdev)->dev))
		pm_runtime_resume(&(pdev)->dev);

	host = mmc_priv(mmc);

	DBG(host, "Removing SDCC device = %d\n", pdev->id);
	plat = host->plat;

	if (!plat->status_irq)
		sysfs_remove_group(&pdev->dev.kobj, &dev_attr_grp);

	del_timer_sync(&host->req_tout_timer);
	tasklet_kill(&host->dma_tlet);
	tasklet_kill(&host->sps.tlet);
	mmc_remove_host(mmc);

	if (plat->status_irq)
		free_irq(plat->status_irq, host);

	wake_lock_destroy(&host->sdio_suspend_wlock);
	if (plat->sdiowakeup_irq) {
		wake_lock_destroy(&host->sdio_wlock);
		set_irq_wake(plat->sdiowakeup_irq, 0);
		free_irq(plat->sdiowakeup_irq, host);
	}

	free_irq(host->core_irqres->start, host);
	free_irq(host->core_irqres->start, host);

	clk_put(host->clk);
	if (!IS_ERR(host->pclk))
		clk_put(host->pclk);
	if (!IS_ERR_OR_NULL(host->dfab_pclk))
		clk_put(host->dfab_pclk);

	if (host->is_dma_mode) {
		if (host->dmares)
			dma_free_coherent(NULL,
					sizeof(struct msmsdcc_nc_dmadata),
					host->dma.nc, host->dma.nc_busaddr);
	}

	if (host->is_sps_mode) {
		msmsdcc_dml_exit(host);
		msmsdcc_sps_exit(host);
	}

	iounmap(host->base);
	mmc_free_host(mmc);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&host->early_suspend);
#endif
	pm_runtime_disable(&(pdev)->dev);
	pm_runtime_set_suspended(&(pdev)->dev);

	return 0;
}

#ifdef CONFIG_PM
static int
msmsdcc_runtime_suspend(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msmsdcc_host *host = mmc_priv(mmc);
	int rc = 0;

	if (mmc) {
		host->sdcc_suspending = 1;
		mmc->suspend_task = current;

		/*
		 * If the clocks are already turned off by SDIO clients (as
		 * part of LPM), then clocks should be turned on before
		 * calling mmc_suspend_host() because mmc_suspend_host might
		 * send some commands to the card. The clocks will be turned
		 * off again after mmc_suspend_host. Thus for SD/MMC/SDIO
		 * cards, clocks will be turned on before mmc_suspend_host
		 * and turned off after mmc_suspend_host.
		 */
		mmc->ios.clock = host->clk_rate;
		mmc->ops->set_ios(host->mmc, &host->mmc->ios);

		/*
		 * MMC core thinks that host is disabled by now since
		 * runtime suspend is scheduled after msmsdcc_disable()
		 * is called. Thus, MMC core will try to enable the host
		 * while suspending it. This results in a synchronous
		 * runtime resume request while in runtime suspending
		 * context and hence inorder to complete this resume
		 * requet, it will wait for suspend to be complete,
		 * but runtime suspend also can not proceed further
		 * until the host is resumed. Thus, it leads to a hang.
		 * Hence, increase the pm usage count before suspending
		 * the host so that any resume requests after this will
		 * simple become pm usage counter increment operations.
		 */
		pm_runtime_get_noresume(dev);
		rc = mmc_suspend_host(mmc);
		pm_runtime_put_noidle(dev);

		if (!rc) {
			/*
			 * If MMC core level suspend is not supported, turn
			 * off clocks to allow deep sleep (TCXO shutdown).
			 */
			mmc->ios.clock = 0;
			mmc->ops->set_ios(host->mmc, &host->mmc->ios);

			if (mmc->card && (mmc->card->type == MMC_TYPE_SDIO) &&
				(mmc->pm_flags & MMC_PM_WAKE_SDIO_IRQ)) {
				host->sdio_irq_disabled = 0;
				if (host->plat->sdiowakeup_irq) {
					enable_irq_wake(
						host->plat->sdiowakeup_irq);
					enable_irq(host->plat->sdiowakeup_irq);
				}
			}
		}
		host->sdcc_suspending = 0;
		mmc->suspend_task = NULL;
	}
	return rc;
}

static int
msmsdcc_runtime_resume(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msmsdcc_host *host = mmc_priv(mmc);
	unsigned long flags;
	int release_lock = 0;

	if (mmc) {
		mmc->ios.clock = host->clk_rate;
		mmc->ops->set_ios(host->mmc, &host->mmc->ios);

		spin_lock_irqsave(&host->lock, flags);
		writel(host->mci_irqenable, host->base + MMCIMASK0);

		if (mmc->card && (mmc->card->type == MMC_TYPE_SDIO) &&
				(mmc->pm_flags & MMC_PM_WAKE_SDIO_IRQ) &&
				!host->sdio_irq_disabled) {
				if (host->plat->sdiowakeup_irq) {
					disable_irq_nosync(
						host->plat->sdiowakeup_irq);
					disable_irq_wake(
						host->plat->sdiowakeup_irq);
				}
				host->sdio_irq_disabled = 1;
		} else {
			release_lock = 1;
		}

		spin_unlock_irqrestore(&host->lock, flags);

		mmc_resume_host(mmc);

		/*
		 * FIXME: Clearing of flags must be handled in clients
		 * resume handler.
		 */
		spin_lock_irqsave(&host->lock, flags);
		mmc->pm_flags = 0;
		spin_unlock_irqrestore(&host->lock, flags);

		/*
		 * After resuming the host wait for sometime so that
		 * the SDIO work will be processed.
		 */
		if (mmc->card && (mmc->card->type == MMC_TYPE_SDIO) &&
					release_lock)
			wake_lock_timeout(&host->sdio_wlock, 1);

		wake_unlock(&host->sdio_suspend_wlock);
	}
	return 0;
}

static int msmsdcc_runtime_idle(struct device *dev)
{
	/* Idle timeout is not configurable for now */
	pm_schedule_suspend(dev, MSM_MMC_IDLE_TIMEOUT);

	return -EAGAIN;
}

static int msmsdcc_pm_suspend(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msmsdcc_host *host = mmc_priv(mmc);
	int rc = 0;

	if (host->plat->status_irq)
		disable_irq(host->plat->status_irq);

	if (!pm_runtime_suspended(dev))
		rc = msmsdcc_runtime_suspend(dev);

	return rc;
}

static int msmsdcc_pm_resume(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msmsdcc_host *host = mmc_priv(mmc);
	int rc = 0;

	rc = msmsdcc_runtime_resume(dev);
	if (host->plat->status_irq) {
		msmsdcc_check_status((unsigned long)host);
		enable_irq(host->plat->status_irq);
	}

	/* Update the run-time PM status */
	pm_runtime_disable(dev);
	rc = pm_runtime_set_active(dev);
	if (rc < 0)
		pr_info("%s: %s: failed with error %d", mmc_hostname(mmc),
				__func__, rc);
	pm_runtime_enable(dev);

	return rc;
}

#else
#define msmsdcc_runtime_suspend NULL
#define msmsdcc_runtime_resume NULL
#define msmsdcc_runtime_idle NULL
#define msmsdcc_pm_suspend NULL
#define msmsdcc_pm_resume NULL
#endif

static const struct dev_pm_ops msmsdcc_dev_pm_ops = {
	.runtime_suspend = msmsdcc_runtime_suspend,
	.runtime_resume  = msmsdcc_runtime_resume,
	.runtime_idle    = msmsdcc_runtime_idle,
	.suspend 	 = msmsdcc_pm_suspend,
	.resume		 = msmsdcc_pm_resume,
};

static struct platform_driver msmsdcc_driver = {
	.probe		= msmsdcc_probe,
	.remove		= msmsdcc_remove,
	.driver		= {
		.name	= "msm_sdcc",
		.pm	= &msmsdcc_dev_pm_ops,
	},
};

static int __init msmsdcc_init(void)
{
#if defined(CONFIG_DEBUG_FS)
	int ret = 0;
	ret = msmsdcc_dbg_init();
	if (ret) {
		pr_err("Failed to create debug fs dir \n");
		return ret;
	}
#endif
	return platform_driver_register(&msmsdcc_driver);
}

static void __exit msmsdcc_exit(void)
{
	platform_driver_unregister(&msmsdcc_driver);

#if defined(CONFIG_DEBUG_FS)
	debugfs_remove(debugfs_file);
	debugfs_remove(debugfs_dir);
#endif
}

module_init(msmsdcc_init);
module_exit(msmsdcc_exit);

MODULE_DESCRIPTION("Qualcomm Multimedia Card Interface driver");
MODULE_LICENSE("GPL");

#if defined(CONFIG_DEBUG_FS)

static int
msmsdcc_dbg_state_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t
msmsdcc_dbg_state_read(struct file *file, char __user *ubuf,
		       size_t count, loff_t *ppos)
{
	struct msmsdcc_host *host = (struct msmsdcc_host *) file->private_data;
	char buf[1024];
	int max, i;

	i = 0;
	max = sizeof(buf) - 1;

	i += scnprintf(buf + i, max - i, "STAT: %p %p %p\n", host->curr.mrq,
		       host->curr.cmd, host->curr.data);
	if (host->curr.cmd) {
		struct mmc_command *cmd = host->curr.cmd;

		i += scnprintf(buf + i, max - i, "CMD : %.8x %.8x %.8x\n",
			      cmd->opcode, cmd->arg, cmd->flags);
	}
	if (host->curr.data) {
		struct mmc_data *data = host->curr.data;
		i += scnprintf(buf + i, max - i,
			      "DAT0: %.8x %.8x %.8x %.8x %.8x %.8x\n",
			      data->timeout_ns, data->timeout_clks,
			      data->blksz, data->blocks, data->error,
			      data->flags);
		i += scnprintf(buf + i, max - i, "DAT1: %.8x %.8x %.8x %p\n",
			      host->curr.xfer_size, host->curr.xfer_remain,
			      host->curr.data_xfered, host->dma.sg);
	}

	return simple_read_from_buffer(ubuf, count, ppos, buf, i);
}

static const struct file_operations msmsdcc_dbg_state_ops = {
	.read	= msmsdcc_dbg_state_read,
	.open	= msmsdcc_dbg_state_open,
};

static void msmsdcc_dbg_createhost(struct msmsdcc_host *host)
{
	if (debugfs_dir) {
		debugfs_file = debugfs_create_file(mmc_hostname(host->mmc),
							0644, debugfs_dir, host,
							&msmsdcc_dbg_state_ops);
	}
}

static int __init msmsdcc_dbg_init(void)
{
	int err;

	debugfs_dir = debugfs_create_dir("msmsdcc", 0);
	if (IS_ERR(debugfs_dir)) {
		err = PTR_ERR(debugfs_dir);
		debugfs_dir = NULL;
		return err;
	}

	return 0;
}
#endif
