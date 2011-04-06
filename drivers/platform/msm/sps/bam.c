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

/* Bus-Access-Manager (BAM) Hardware manager. */

#include <linux/types.h>	/* u32 */
#include <linux/kernel.h>	/* pr_info() */
#include <linux/io.h>		/* ioread32() */
#include <linux/bitops.h>	/* find_first_bit() */
#include <linux/errno.h>	/* ENODEV */

#include "bam.h"

/**
 *  Valid BAM Hardware version.
 *
 */
#define BAM_MIN_VERSION 2
#define BAM_MAX_VERSION 2

/**
 *  BAM Hardware registers.
 *
 */
#define CTRL                        (0xf80)
#define REVISION                    (0xf84)
#define NUM_PIPES                   (0xfbc)
#define DESC_CNT_TRSHLD             (0xf88)
#define IRQ_SRCS                    (0xf8c)
#define IRQ_SRCS_MSK                (0xf90)
#define IRQ_SRCS_UNMASKED           (0xfb0)
#define IRQ_STTS                    (0xf94)
#define IRQ_CLR                     (0xf98)
#define IRQ_EN                      (0xf9c)
#define IRQ_SIC_SEL                 (0xfa0)
#define AHB_MASTER_ERR_CTRLS        (0xfa4)
#define AHB_MASTER_ERR_ADDR         (0xfa8)
#define AHB_MASTER_ERR_DATA         (0xfac)
#define IRQ_DEST                    (0xfb4)
#define PERIPH_IRQ_DEST             (0xfb8)
#define TEST_BUS_REG                (0xff8)
#define CNFG_BITS                   (0xffc)

#define P_CTRL(n)                  (0x0000 + 128 * (n))
#define P_RST(n)                   (0x0004 + 128 * (n))
#define P_HALT(n)                  (0x0008 + 128 * (n))
#define P_IRQ_STTS(n)              (0x0010 + 128 * (n))
#define P_IRQ_CLR(n)               (0x0014 + 128 * (n))
#define P_IRQ_EN(n)                (0x0018 + 128 * (n))
#define P_TIMER(n)                 (0x001c + 128 * (n))
#define P_TIMER_CTRL(n)            (0x0020 + 128 * (n))
#define P_EVNT_DEST_ADDR(n)        (0x102c + 64 * (n))
#define P_EVNT_REG(n)              (0x1018 + 64 * (n))
#define P_SW_OFSTS(n)              (0x1000 + 64 * (n))
#define P_DATA_FIFO_ADDR(n)        (0x1024 + 64 * (n))
#define P_DESC_FIFO_ADDR(n)        (0x101c + 64 * (n))
#define P_EVNT_GEN_TRSHLD(n)       (0x1028 + 64 * (n))
#define P_FIFO_SIZES(n)            (0x1020 + 64 * (n))
#define P_IRQ_DEST_ADDR(n)         (0x103c + 64 * (n))
#define P_RETR_CNTXT(n)           (0x1034 + 64 * (n))
#define P_SI_CNTXT(n)             (0x1038 + 64 * (n))
#define P_AU_PSM_CNTXT_1(n)       (0x1004 + 64 * (n))
#define P_PSM_CNTXT_2(n)          (0x1008 + 64 * (n))
#define P_PSM_CNTXT_3(n)          (0x100c + 64 * (n))
#define P_PSM_CNTXT_4(n)          (0x1010 + 64 * (n))
#define P_PSM_CNTXT_5(n)          (0x1014 + 64 * (n))

/**
 *  BAM Hardware registers bitmask.
 *  format: <register>_<field>
 *
 */
/* CTRL */
#define BAM_CACHED_DESC_STORE                   0x8000
#define BAM_DESC_CACHE_SEL                      0x6000
#define BAM_PERIPH_IRQ_SIC_SEL                  0x1000
#define BAM_TESTBUS_SEL                          0xfe0
#define BAM_EN_ACCUM                              0x10
#define BAM_EN                                     0x2
#define BAM_SW_RST                                 0x1

/* IRQ_SRCS */
#define BAM_IRQ                         0x80000000
#define P_IRQ                           0x7fffffff

#define IRQ_STTS_BAM_ERROR_IRQ                          0x4
#define IRQ_STTS_BAM_HRESP_ERR_IRQ                      0x2
#define IRQ_CLR_BAM_ERROR_CLR                           0x4
#define IRQ_CLR_BAM_HRESP_ERR_CLR                       0x2
#define IRQ_EN_BAM_ERROR_EN                             0x4
#define IRQ_EN_BAM_HRESP_ERR_EN                         0x2
#define IRQ_SIC_SEL_BAM_IRQ_SIC_SEL              0x80000000
#define IRQ_SIC_SEL_P_IRQ_SIC_SEL                0x7fffffff
#define AHB_MASTER_ERR_CTRLS_BAM_ERR_DIRECT_MODE    0x10000
#define AHB_MASTER_ERR_CTRLS_BAM_ERR_HCID            0xf000
#define AHB_MASTER_ERR_CTRLS_BAM_ERR_HPROT            0xf00
#define AHB_MASTER_ERR_CTRLS_BAM_ERR_HBURST            0xe0
#define AHB_MASTER_ERR_CTRLS_BAM_ERR_HSIZE             0x18
#define AHB_MASTER_ERR_CTRLS_BAM_ERR_HWRITE             0x4
#define AHB_MASTER_ERR_CTRLS_BAM_ERR_HTRANS             0x3
#define CNFG_BITS_BAM_FULL_PIPE                       0x800
#define CNFG_BITS_BAM_PIPE_CNFG                         0x4

/* P_ctrln */
#define P_SYS_MODE                             0x20
#define P_SYS_STRM                             0x10
#define P_DIRECTION                             0x8
#define P_EN                                    0x2

#define P_RST_P_SW_RST                                 0x1

#define P_HALT_P_PROD_HALTED                           0x2
#define P_HALT_P_HALT                                  0x1

#define P_IRQ_STTS_P_TRNSFR_END_IRQ                   0x20
#define P_IRQ_STTS_P_ERR_IRQ                          0x10
#define P_IRQ_STTS_P_OUT_OF_DESC_IRQ                   0x8
#define P_IRQ_STTS_P_WAKE_IRQ                          0x4
#define P_IRQ_STTS_P_TIMER_IRQ                         0x2
#define P_IRQ_STTS_P_PRCSD_DESC_IRQ                    0x1

#define P_IRQ_CLR_P_TRNSFR_END_CLR                    0x20
#define P_IRQ_CLR_P_ERR_CLR                           0x10
#define P_IRQ_CLR_P_OUT_OF_DESC_CLR                    0x8
#define P_IRQ_CLR_P_WAKE_CLR                           0x4
#define P_IRQ_CLR_P_TIMER_CLR                          0x2
#define P_IRQ_CLR_P_PRCSD_DESC_CLR                     0x1

#define P_IRQ_EN_P_TRNSFR_END_EN                      0x20
#define P_IRQ_EN_P_ERR_EN                             0x10
#define P_IRQ_EN_P_OUT_OF_DESC_EN                      0x8
#define P_IRQ_EN_P_WAKE_EN                             0x4
#define P_IRQ_EN_P_TIMER_EN                            0x2
#define P_IRQ_EN_P_PRCSD_DESC_EN                       0x1

#define P_TIMER_P_TIMER                             0xffff

/* P_TIMER_ctrln */
#define P_TIMER_RST                0x80000000
#define P_TIMER_RUN                0x40000000
#define P_TIMER_MODE               0x20000000
#define P_TIMER_TRSHLD                 0xffff

/* P_EVNT_regn */
#define P_BYTES_CONSUMED             0xffff0000
#define P_DESC_FIFO_PEER_OFST            0xffff

/* P_SW_ofstsn */
#define SW_OFST_IN_DESC              0xffff0000
#define SW_DESC_OFST                     0xffff

#define P_EVNT_GEN_TRSHLD_P_TRSHLD                  0xffff

/* P_FIFO_sizesn */
#define P_DATA_FIFO_SIZE           0xffff0000
#define P_DESC_FIFO_SIZE               0xffff

#define P_RETR_CNTXT_RETR_DESC_OFST            0xffff0000
#define P_RETR_CNTXT_RETR_OFST_IN_DESC             0xffff
#define P_SI_CNTXT_SI_DESC_OFST                    0xffff
#define P_AU_PSM_CNTXT_1_AU_PSM_ACCUMED        0xffff0000
#define P_AU_PSM_CNTXT_1_AU_ACKED                  0xffff
#define P_PSM_CNTXT_2_PSM_DESC_VALID           0x80000000
#define P_PSM_CNTXT_2_PSM_DESC_IRQ             0x40000000
#define P_PSM_CNTXT_2_PSM_DESC_IRQ_DONE        0x20000000
#define P_PSM_CNTXT_2_PSM_GENERAL_BITS         0x1e000000
#define P_PSM_CNTXT_2_PSM_CONS_STATE            0x1c00000
#define P_PSM_CNTXT_2_PSM_PROD_SYS_STATE         0x380000
#define P_PSM_CNTXT_2_PSM_PROD_B2B_STATE          0x70000
#define P_PSM_CNTXT_2_PSM_DESC_SIZE                0xffff
#define P_PSM_CNTXT_4_PSM_DESC_OFST            0xffff0000
#define P_PSM_CNTXT_4_PSM_SAVED_ACCUMED_SIZE       0xffff
#define P_PSM_CNTXT_5_PSM_BLOCK_BYTE_CNT       0xffff0000
#define P_PSM_CNTXT_5_PSM_OFST_IN_DESC             0xffff

#define BAM_ERROR   (-1)

/**
 *
 * Read register with debug info.
 *
 * @base - bam base virtual address.
 * @offset - register offset.
 *
 * @return u32
 */
static inline u32 bam_read_reg(void *base, u32 offset)
{
	u32 val = ioread32(base + offset);
	pr_debug("bam: read reg 0x%x r_val 0x%x.\n", offset, val);
	return val;
}

/**
 * Read register masked field with debug info.
 *
 * @base - bam base virtual address.
 * @offset - register offset.
 * @mask - register bitmask.
 *
 * @return u32
 */
static inline u32 bam_read_reg_field(void *base, u32 offset, const u32 mask)
{
	u32 shift = find_first_bit((void *)&mask, 32);
	u32 val = ioread32(base + offset);
	val &= mask;		/* clear other bits */
	val >>= shift;
	pr_debug("bam: read reg 0x%x mask 0x%x r_val 0x%x.\n",
		offset, mask, val);
	return val;
}

/**
 *
 * Write register with debug info.
 *
 * @base - bam base virtual address.
 * @offset - register offset.
 * @val - value to write.
 *
 */
static inline void bam_write_reg(void *base, u32 offset, u32 val)
{
	iowrite32(val, base + offset);
	pr_debug("bam: write reg 0x%x w_val 0x%x.\n", offset, val);
}

/**
 * Write register masked field with debug info.
 *
 * @base - bam base virtual address.
 * @offset - register offset.
 * @mask - register bitmask.
 * @val - value to write.
 *
 */
static inline void bam_write_reg_field(void *base, u32 offset,
				       const u32 mask, u32 val)
{
	u32 shift = find_first_bit((void *)&mask, 32);
	u32 tmp = ioread32(base + offset);

	tmp &= ~mask;		/* clear written bits */
	val = tmp | (val << shift);
	iowrite32(val, base + offset);
	pr_debug("bam: write reg 0x%x w_val 0x%x.\n", offset, val);
}

/**
 * Initialize a BAM device
 *
 */
int bam_init(void *base, u32 ee,
		u16 summing_threshold,
		u32 irq_mask, u32 *version, u32 *num_pipes)
{
	/* disable bit#11 because of HW bug */
	u32 cfg_bits = 0xffffffff & ~(1 << 11);
	u32 ver = 0;

	ver = bam_read_reg(base, REVISION);

	if ((ver < BAM_MIN_VERSION) || (ver > BAM_MAX_VERSION)) {
		pr_err("bam:Invalid BAM version 0x%x.\n", ver);
		return -ENODEV;
	}

	if (summing_threshold == 0) {
		summing_threshold = 4;
		pr_err("bam:summing_threshold is zero , use default 4.\n");
	}

	bam_write_reg_field(base, CTRL, BAM_SW_RST, 1);
	/* No delay needed */
	bam_write_reg_field(base, CTRL, BAM_SW_RST, 0);

	bam_write_reg_field(base, CTRL, BAM_EN, 1);

	bam_write_reg(base, DESC_CNT_TRSHLD, summing_threshold);

	bam_write_reg(base, CNFG_BITS, cfg_bits);

	/*
	 *  Enable Global BAM Interrupt - for error reasons ,
	 *  filter with mask.
	 *  Note: Pipes interrupts are disabled until BAM_P_IRQ_enn is set
	 */
	bam_write_reg_field(base, IRQ_SRCS_MSK, BAM_IRQ, 1);

	bam_write_reg(base, IRQ_EN, irq_mask);

	*num_pipes = bam_read_reg(base, NUM_PIPES);
	*version = ver;

	return 0;
}

/**
 * Verify that a BAM device is enabled and gathers the hardware
 * configuration.
 *
 */
int bam_check(void *base, u32 *version, u32 *num_pipes)
{
	u32 ver = 0;

	if (!bam_read_reg_field(base, CTRL, BAM_EN))
		return -ENODEV;

	ver = bam_read_reg(base, REVISION);

	/*
	 *  Discover the hardware version number and the number of pipes
	 *  supported by this BAM
	 */
	*num_pipes = bam_read_reg(base, NUM_PIPES);
	*version = ver;

	/* Check BAM version */
	if ((ver < BAM_MIN_VERSION) || (ver > BAM_MAX_VERSION)) {
		pr_err("bam:Invalid BAM version 0x%x.\n", ver);
		return -ENODEV;
	}

	return 0;
}

/**
 * Disable a BAM device
 *
 */
void bam_exit(void *base, u32 ee)
{
	bam_write_reg_field(base, IRQ_SRCS_MSK, BAM_IRQ, 0);

	bam_write_reg(base, IRQ_EN, 0);

	/* Disable the BAM */
	bam_write_reg_field(base, CTRL, BAM_EN, 0);
}

/**
 * Get and Clear BAM global IRQ status
 *
 * note: clear status only for pipes controlled by this
 * processor
 */
u32 bam_get_and_clear_irq_status(void *base, u32 ee, u32 mask)
{
	u32 status = bam_read_reg(base, IRQ_SRCS);
	u32 clr = status &= mask;

	bam_write_reg(base, IRQ_CLR, clr);

	return status;
}

/**
 * Initialize a BAM pipe
 */
int bam_pipe_init(void *base, u32 pipe,	struct bam_pipe_parameters *param)
{
	/* Reset the BAM pipe */
	bam_write_reg(base, P_RST(pipe), 1);
	/* No delay needed */
	bam_write_reg(base, P_RST(pipe), 0);

	/* Enable the Pipe Interrupt at the BAM level */
	bam_write_reg_field(base, IRQ_SRCS_MSK, (1 << pipe), 1);

	bam_write_reg(base, P_IRQ_EN(pipe), param->pipe_irq_mask);

	bam_write_reg_field(base, P_CTRL(pipe), P_DIRECTION, param->dir);
	bam_write_reg_field(base, P_CTRL(pipe), P_SYS_MODE, param->mode);

	bam_write_reg(base, P_EVNT_GEN_TRSHLD(pipe), param->event_threshold);

	bam_write_reg(base, P_DESC_FIFO_ADDR(pipe), param->desc_base);
	bam_write_reg_field(base, P_FIFO_SIZES(pipe), P_DESC_FIFO_SIZE,
			    param->desc_size);

	bam_write_reg_field(base, P_CTRL(pipe), P_SYS_STRM,
			    param->stream_mode);

	if (param->mode == BAM_PIPE_MODE_BAM2BAM) {
		u32 peer_dest_addr = param->peer_phys_addr +
				      P_EVNT_REG(param->peer_pipe);

		bam_write_reg(base, P_DATA_FIFO_ADDR(pipe),
			      param->data_base);
		bam_write_reg_field(base, P_FIFO_SIZES(pipe),
				    P_DATA_FIFO_SIZE, param->data_size);

		bam_write_reg(base, P_EVNT_DEST_ADDR(pipe), peer_dest_addr);

		pr_debug("bam:Bam=0x%x.Pipe=%d.peer_bam=0x%x.peer_pipe=%d.\n",
			(u32) base, pipe,
			(u32) param->peer_phys_addr,
			param->peer_pipe);
	}

	/* Pipe Enable - at last */
	bam_write_reg_field(base, P_CTRL(pipe), P_EN, 1);

	return 0;
}

/**
 * Reset the BAM pipe
 *
 */
void bam_pipe_exit(void *base, u32 pipe, u32 ee)
{
	bam_write_reg(base, P_IRQ_EN(pipe), 0);

	/* Disable the Pipe Interrupt at the BAM level */
	bam_write_reg_field(base, IRQ_SRCS_MSK, (1 << pipe), 0);

	/* Pipe Disable */
	bam_write_reg_field(base, P_CTRL(pipe), P_EN, 0);
}

/**
 * Enable a BAM pipe
 *
 */
void bam_pipe_enable(void *base, u32 pipe)
{
	bam_write_reg_field(base, P_CTRL(pipe), P_EN, 1);
}

/**
 * Diasble a BAM pipe
 *
 */
void bam_pipe_disable(void *base, u32 pipe)
{
	bam_write_reg_field(base, P_CTRL(pipe), P_EN, 0);
}

/**
 * Check if a BAM pipe is enabled.
 *
 */
int bam_pipe_is_enabled(void *base, u32 pipe)
{
	return bam_read_reg_field(base, P_CTRL(pipe), P_EN);
}

/**
 * Configure interrupt for a BAM pipe
 *
 */
void bam_pipe_set_irq(void *base, u32 pipe, enum bam_enable irq_en,
		      u32 src_mask, u32 ee)
{
	bam_write_reg(base, P_IRQ_EN(pipe), src_mask);
	bam_write_reg_field(base, IRQ_SRCS_MSK, (1 << pipe), irq_en);
}

/**
 * Configure a BAM pipe for satellite MTI use
 *
 */
void bam_pipe_satellite_mti(void *base, u32 pipe, u32 irq_gen_addr, u32 ee)
{
	bam_write_reg(base, P_IRQ_EN(pipe), 0);
	bam_write_reg(base, P_IRQ_DEST_ADDR(pipe), irq_gen_addr);

	bam_write_reg_field(base, IRQ_SIC_SEL, (1 << pipe), 1);
	bam_write_reg_field(base, IRQ_SRCS_MSK, (1 << pipe), 1);
}

/**
 * Configure MTI for a BAM pipe
 *
 */
void bam_pipe_set_mti(void *base, u32 pipe, enum bam_enable irq_en,
		      u32 src_mask, u32 irq_gen_addr)
{
	/*
	 * MTI use is only supported on BAMs when global config is controlled
	 * by a remote processor.
	 * Consequently, the global configuration register to enable SIC (MTI)
	 * support cannot be accessed.
	 * The remote processor must be relied upon to enable the SIC and the
	 * interrupt. Since the remote processor enable both SIC and interrupt,
	 * the interrupt enable mask must be set to zero for polling mode.
	 */

	bam_write_reg(base, P_IRQ_DEST_ADDR(pipe), irq_gen_addr);

	if (!irq_en)
		src_mask = 0;

	bam_write_reg(base, P_IRQ_EN(pipe), src_mask);
}

/**
 * Get and Clear BAM pipe IRQ status
 *
 */
u32 bam_pipe_get_and_clear_irq_status(void *base, u32 pipe)
{
	u32 status = 0;

	status = bam_read_reg(base, P_IRQ_STTS(pipe));
	bam_write_reg(base, P_IRQ_CLR(pipe), status);

	return status;
}

/**
 * Set write offset for a BAM pipe
 *
 */
void bam_pipe_set_desc_write_offset(void *base, u32 pipe, u32 next_write)
{
	/*
	 * It is not necessary to perform a read-modify-write masking to write
	 * the P_DESC_FIFO_PEER_OFST value, since the other field in the
	 * register (P_BYTES_CONSUMED) is read-only.
	 */
	bam_write_reg_field(base, P_EVNT_REG(pipe), P_DESC_FIFO_PEER_OFST,
			    next_write);
}

/**
 * Get write offset for a BAM pipe
 *
 */
u32 bam_pipe_get_desc_write_offset(void *base, u32 pipe)
{
	return bam_read_reg_field(base, P_EVNT_REG(pipe),
				  P_DESC_FIFO_PEER_OFST);
}

/**
 * Get read offset for a BAM pipe
 *
 */
u32 bam_pipe_get_desc_read_offset(void *base, u32 pipe)
{
	return bam_read_reg_field(base, P_SW_OFSTS(pipe), SW_DESC_OFST);
}

/**
 * Configure inactivity timer count for a BAM pipe
 *
 */
void bam_pipe_timer_config(void *base, u32 pipe, enum bam_pipe_timer_mode mode,
			 u32 timeout_count)
{
	bam_write_reg_field(base, P_TIMER_CTRL(pipe), P_TIMER_MODE, mode);
	bam_write_reg_field(base, P_TIMER_CTRL(pipe), P_TIMER_TRSHLD,
			    timeout_count);
}

/**
 * Reset inactivity timer for a BAM pipe
 *
 */
void bam_pipe_timer_reset(void *base, u32 pipe)
{
	/* reset */
	bam_write_reg_field(base, P_TIMER_CTRL(pipe), P_TIMER_RST, 0);
	/* active */
	bam_write_reg_field(base, P_TIMER_CTRL(pipe), P_TIMER_RST, 1);
}

/**
 * Get inactivity timer count for a BAM pipe
 *
 */
u32 bam_pipe_timer_get_count(void *base, u32 pipe)
{
	return bam_read_reg(base, P_TIMER(pipe));
}
