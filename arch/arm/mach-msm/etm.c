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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/percpu.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include <asm/atomic.h>

#include "cp14.h"

#define LOG_BUF_LEN    32768
#define ETM_NUM_REGS   128
#define ETB_NUM_REGS   9
#define ETB_RAM_SLOTS  2048 /* each slot is 4 bytes, 8kb total */

#define DATALOG_SYNC                 0xB5C7
#define ETM_DUMP_MSG_ID              0x000A6960
#define ETB_DUMP_MSG_ID              0x000A6961

/* ETM Registers */
#define ETM_REG_CONTROL              0x00
#define ETM_REG_STATUS               0x04
#define ETB_REG_CONTROL              0x71
#define ETB_REG_STATUS               0x72
#define ETB_REG_COUNT                0x73
#define ETB_REG_ADDRESS              0x74
#define ETB_REG_DATA                 0x75

/* Bitmasks for the ETM control register */
#define ETM_CONTROL_POWERDOWN        0x00000001
#define ETM_CONTROL_PROGRAM          0x00000400

/* Bitmasks for the ETM status register */
#define ETM_STATUS_PROGRAMMING       0x00000002

/* ETB Status Register bit definitions */
#define OV                           0x00200000

/* ETB Control Register bit definitions */
#define AIR                          0x00000008
#define AIW                          0x00000004
#define CPTM                         0x00000002
#define CPTEN                        0x00000001

/* Bitmasks for the swconfig field of ETM_CONFIG */
#define TRIGGER_ALL      0x00000002 /* ETM trigger propagated to ETM
				     * instances on all cores */

static int trace_enabled;
static int trace_on_boot;
module_param_named(
	trace_on_boot, trace_on_boot, int, S_IRUGO
);

struct b {
	uint8_t etm_log_buf[LOG_BUF_LEN];
	uint32_t log_end;
};

static struct b b0;
static struct b b1;
static struct b __percpu * *alloc_b;
static atomic_t etm_dev_in_use;

/* These default settings will be used to configure the ETM/ETB
 * when the driver loads. */
struct etm_config_struct {
	uint32_t etm_00_control;
	uint32_t etm_02_trigger_event;
	uint32_t etm_06_te_start_stop;
	uint32_t etm_07_te_single_addr_comp;
	uint32_t etm_08_te_event;
	uint32_t etm_09_te_control;
	uint32_t etm_0a_fifofull_region;
	uint32_t etm_0b_fifofull_level;
	uint32_t etm_0c_vd_event;
	uint32_t etm_0d_vd_single_addr_comp;
	uint32_t etm_0e_vd_mmd;
	uint32_t etm_0f_vd_control;
	uint32_t etm_10_addr_comp_value_1;
	uint32_t etm_11_addr_comp_value_2;
	uint32_t etm_12_addr_comp_value_3;
	uint32_t etm_13_addr_comp_value_4;
	uint32_t etm_14_addr_comp_value_5;
	uint32_t etm_15_addr_comp_value_6;
	uint32_t etm_16_addr_comp_value_7;
	uint32_t etm_17_addr_comp_value_8;
	uint32_t etm_20_addr_access_type_1;
	uint32_t etm_21_addr_access_type_2;
	uint32_t etm_22_addr_access_type_3;
	uint32_t etm_23_addr_access_type_4;
	uint32_t etm_24_addr_access_type_5;
	uint32_t etm_25_addr_access_type_6;
	uint32_t etm_26_addr_access_type_7;
	uint32_t etm_27_addr_access_type_8;
	uint32_t etm_30_data_comp_value_1;
	uint32_t etm_32_data_comp_value_2;
	uint32_t etm_40_data_comp_mask_1;
	uint32_t etm_42_data_comp_mask_2;
	uint32_t etm_50_counter_reload_value_1;
	uint32_t etm_51_counter_reload_value_2;
	uint32_t etm_54_counter_enable_1;
	uint32_t etm_55_counter_enable_2;
	uint32_t etm_58_counter_reload_event_1;
	uint32_t etm_59_counter_reload_event_2;
	uint32_t etm_60_seq_event_1_to_2;
	uint32_t etm_61_seq_event_2_to_1;
	uint32_t etm_62_seq_event_2_to_3;
	uint32_t etm_63_seq_event_3_to_1;
	uint32_t etm_64_seq_event_3_to_2;
	uint32_t etm_65_seq_event_1_to_3;
	uint32_t etm_6c_cid_comp_value_1;
	uint32_t etm_6f_cid_comp_mask;
	uint32_t etm_78_sync_freq;
	uint32_t swconfig;
	uint32_t etb_trig_cnt;
	uint32_t etb_init_ptr;
};

static struct etm_config_struct etm_config = {
	/* etm_00_control 0x0000D84E: 32-bit CID, cycle-accurate,
	 * monitorCPRT */
	.etm_00_control			= 0x0000D84E,
	/* etm_02_trigger_event 0x00000000: address comparator 0 matches */
	.etm_02_trigger_event		= 0x00000000,
	.etm_06_te_start_stop		= 0x00000000,
	.etm_07_te_single_addr_comp	= 0x00000000,
	/* etm_08_te_event 0x0000006F: always true */
	.etm_08_te_event		= 0x0000006F,
	/* etm_09_te_control 0x01000000: exclude none */
	.etm_09_te_control		= 0x01000000,
	.etm_0a_fifofull_region		= 0x00000000,
	.etm_0b_fifofull_level		= 0x00000000,
	/* etm_0c_vd_event 0x0000006F: always true */
	.etm_0c_vd_event                = 0x0000006F,
	.etm_0d_vd_single_addr_comp     = 0x00000000,
	.etm_0e_vd_mmd                  = 0x00000000,
	/* etm_0f_vd_control 0x00010000: exclude none */
	.etm_0f_vd_control              = 0x00010000,
	.etm_10_addr_comp_value_1       = 0x50337038,
	.etm_11_addr_comp_value_2       = 0x00000000,
	.etm_12_addr_comp_value_3       = 0x00000000,
	.etm_13_addr_comp_value_4       = 0x00000000,
	.etm_14_addr_comp_value_5       = 0x00000000,
	.etm_15_addr_comp_value_6       = 0x00000000,
	.etm_16_addr_comp_value_7       = 0x00000000,
	.etm_17_addr_comp_value_8       = 0x00000000,
	/* etm_20_addr_access_type_1 0x0000001E: word, data store */
	.etm_20_addr_access_type_1      = 0x0000001E,
	.etm_21_addr_access_type_2      = 0x00000000,
	.etm_22_addr_access_type_3      = 0x00000000,
	.etm_23_addr_access_type_4      = 0x00000000,
	.etm_24_addr_access_type_5      = 0x00000000,
	.etm_25_addr_access_type_6      = 0x00000000,
	.etm_26_addr_access_type_7      = 0x00000000,
	.etm_27_addr_access_type_8      = 0x00000000,
	.etm_30_data_comp_value_1       = 0x00000000,
	.etm_32_data_comp_value_2       = 0x00000000,
	.etm_40_data_comp_mask_1        = 0x00000000,
	.etm_42_data_comp_mask_2        = 0x00000000,
	.etm_50_counter_reload_value_1  = 0x00000000,
	.etm_51_counter_reload_value_2  = 0x00000000,
	.etm_54_counter_enable_1        = 0x0002406F,
	.etm_55_counter_enable_2        = 0x0002406F,
	.etm_58_counter_reload_event_1  = 0x0000406F,
	.etm_59_counter_reload_event_2  = 0x0000406F,
	.etm_60_seq_event_1_to_2        = 0x0000406F,
	.etm_61_seq_event_2_to_1        = 0x0000406F,
	.etm_62_seq_event_2_to_3        = 0x0000406F,
	.etm_63_seq_event_3_to_1        = 0x0000406F,
	.etm_64_seq_event_3_to_2        = 0x0000406F,
	.etm_65_seq_event_1_to_3        = 0x0000406F,
	.etm_6c_cid_comp_value_1        = 0x00000000,
	.etm_6f_cid_comp_mask           = 0x00000000,
	.etm_78_sync_freq               = 0x00000400,
	.swconfig                       = 0x00000002,
	/* etb_trig_cnt 0x00000020: record 32 bytes after trigger */
	.etb_trig_cnt                   = 0x00000020,
	/* etb_init_ptr 0x00000010: 16 marker bytes */
	.etb_init_ptr                   = 0x00000010,
};

static void emit_log_char(uint8_t c)
{
	int this_cpu = get_cpu();
	struct b *myb = *per_cpu_ptr(alloc_b, this_cpu);
	char *log_buf = myb->etm_log_buf;
	int index = (myb->log_end)++ & (LOG_BUF_LEN - 1);
	log_buf[index] = c;
	put_cpu();
}

static void emit_log_word(uint32_t word)
{
	emit_log_char(word >> 24);
	emit_log_char(word >> 16);
	emit_log_char(word >> 8);
	emit_log_char(word >> 0);
}

static void __cpu_enable_etb(void)
{
	uint32_t etb_control;
	uint32_t i;

	/* enable auto-increment on reads and writes */
	etb_control = AIR | AIW;
	etm_write_reg(ETB_REG_CONTROL, etb_control);

	/* write tags to the slots before the write pointer so we can
	 * detect overflow */
	etm_write_reg(ETB_REG_ADDRESS, 0x00000000);
	for (i = 0; i < (etm_config.etb_init_ptr >> 2); i++)
		etm_write_reg(ETB_REG_DATA, 0xDEADBEEF);

	etm_write_reg(ETB_REG_STATUS, 0x00000000);

	/* initialize write pointer */
	etm_write_reg(ETB_REG_ADDRESS, etm_config.etb_init_ptr);

	/* multiple of 16 */
	etm_write_reg(ETB_REG_COUNT, etm_config.etb_trig_cnt & 0xFFFFFFF0);

	/* Enable ETB and enable the trigger counter as appropriate. A
	 * trigger count of 0 will be used to signify that the user wants to
	 * ignore the trigger (just keep writing to the ETB and overwriting
	 * the oldest data).  For "trace before trigger" captures the user
	 * should set the trigger count to a small number. */

	etb_control |= CPTEN;
	if (etm_config.etb_trig_cnt)
		etb_control |= CPTM;
	etm_write_reg(ETB_REG_CONTROL, etb_control);
}

static void __cpu_disable_etb(void)
{
	uint32_t etb_control;
	etb_control = etm_read_reg(ETB_REG_CONTROL);
	etb_control &= ~CPTEN;
	etm_write_reg(ETB_REG_CONTROL, etb_control);
}

static void __cpu_enable_etm(void)
{
	uint32_t etm_control;
	uint32_t i;

	etm_control = etm_read_reg(ETM_REG_CONTROL);
	etm_control &= ~ETM_CONTROL_PROGRAM;

	etm_write_reg(ETM_REG_CONTROL, etm_control);

	/* wait for prog bit to take effect (poll 250 times maximum) */
	for (i = 0; i < 250; i++)
		if (etm_read_reg(ETM_REG_STATUS) & ETM_STATUS_PROGRAMMING)
			break;
}

static void __cpu_disable_etm(void)
{
	uint32_t etm_control;
	uint32_t i;

	etm_control = etm_read_reg(ETM_REG_CONTROL);
	etm_control |= ETM_CONTROL_PROGRAM;

	etm_write_reg(ETM_REG_CONTROL, etm_control);

	/* wait for prog bit to take effect (poll 250 times maximum) */
	for (i = 0; i < 250; i++)
		if (etm_read_reg(ETM_REG_STATUS) & ETM_STATUS_PROGRAMMING)
			break;
}

static void __cpu_enable_trace(void *unused)
{
	uint32_t i;
	uint32_t etm_control;
	uint32_t etm_trigger;
	uint32_t etm_external_output;

	get_cpu();

	etm_read_reg(0xC5); /* clear sticky bit in PDSR */

	__cpu_disable_etb();

	etm_control = (etm_config.etm_00_control & ~ETM_CONTROL_POWERDOWN) |
		ETM_CONTROL_PROGRAM;
	etm_write_reg(0x00, etm_control);

	/* wait for the prog bit to take effect (poll 250 times maximum) */
	for (i = 0; i < 250; i++)
		if (etm_read_reg(ETM_REG_STATUS) & ETM_STATUS_PROGRAMMING)
			break;

	etm_trigger = etm_config.etm_02_trigger_event;
	etm_external_output = 0x406F; /* always FALSE */

	if (etm_config.swconfig & TRIGGER_ALL) {
		uint32_t function = 0x5; /*  A OR B */
		uint32_t resource_b = 0x60; /* external input 1 */

		etm_trigger &= 0x7F; /* keep resource A, clear function and
				      * resource B */
		etm_trigger |= (function << 14);
		etm_trigger |= (resource_b << 7);
		etm_external_output = etm_trigger;
	}

	etm_write_reg(0x02, etm_trigger);
	etm_write_reg(0x06, etm_config.etm_06_te_start_stop);
	etm_write_reg(0x07, etm_config.etm_07_te_single_addr_comp);
	etm_write_reg(0x08, etm_config.etm_08_te_event);
	etm_write_reg(0x09, etm_config.etm_09_te_control);
	etm_write_reg(0x0a, etm_config.etm_0a_fifofull_region);
	etm_write_reg(0x0b, etm_config.etm_0b_fifofull_level);
	etm_write_reg(0x0c, etm_config.etm_0c_vd_event);
	etm_write_reg(0x0d, etm_config.etm_0d_vd_single_addr_comp);
	etm_write_reg(0x0e, etm_config.etm_0e_vd_mmd);
	etm_write_reg(0x0f, etm_config.etm_0f_vd_control);
	etm_write_reg(0x10, etm_config.etm_10_addr_comp_value_1);
	etm_write_reg(0x11, etm_config.etm_11_addr_comp_value_2);
	etm_write_reg(0x12, etm_config.etm_12_addr_comp_value_3);
	etm_write_reg(0x13, etm_config.etm_13_addr_comp_value_4);
	etm_write_reg(0x14, etm_config.etm_14_addr_comp_value_5);
	etm_write_reg(0x15, etm_config.etm_15_addr_comp_value_6);
	etm_write_reg(0x16, etm_config.etm_16_addr_comp_value_7);
	etm_write_reg(0x17, etm_config.etm_17_addr_comp_value_8);
	etm_write_reg(0x20, etm_config.etm_20_addr_access_type_1);
	etm_write_reg(0x21, etm_config.etm_21_addr_access_type_2);
	etm_write_reg(0x22, etm_config.etm_22_addr_access_type_3);
	etm_write_reg(0x23, etm_config.etm_23_addr_access_type_4);
	etm_write_reg(0x24, etm_config.etm_24_addr_access_type_5);
	etm_write_reg(0x25, etm_config.etm_25_addr_access_type_6);
	etm_write_reg(0x26, etm_config.etm_26_addr_access_type_7);
	etm_write_reg(0x27, etm_config.etm_27_addr_access_type_8);
	etm_write_reg(0x30, etm_config.etm_30_data_comp_value_1);
	etm_write_reg(0x32, etm_config.etm_32_data_comp_value_2);
	etm_write_reg(0x40, etm_config.etm_40_data_comp_mask_1);
	etm_write_reg(0x42, etm_config.etm_42_data_comp_mask_2);
	etm_write_reg(0x50, etm_config.etm_50_counter_reload_value_1);
	etm_write_reg(0x51, etm_config.etm_51_counter_reload_value_2);
	etm_write_reg(0x54, etm_config.etm_54_counter_enable_1);
	etm_write_reg(0x55, etm_config.etm_55_counter_enable_2);
	etm_write_reg(0x58, etm_config.etm_58_counter_reload_event_1);
	etm_write_reg(0x59, etm_config.etm_59_counter_reload_event_2);
	etm_write_reg(0x60, etm_config.etm_60_seq_event_1_to_2);
	etm_write_reg(0x61, etm_config.etm_61_seq_event_2_to_1);
	etm_write_reg(0x62, etm_config.etm_62_seq_event_2_to_3);
	etm_write_reg(0x63, etm_config.etm_63_seq_event_3_to_1);
	etm_write_reg(0x64, etm_config.etm_64_seq_event_3_to_2);
	etm_write_reg(0x65, etm_config.etm_65_seq_event_1_to_3);
	etm_write_reg(0x68, etm_external_output);
	etm_write_reg(0x6c, etm_config.etm_6c_cid_comp_value_1);
	etm_write_reg(0x6f, etm_config.etm_6f_cid_comp_mask);
	etm_write_reg(0x78, etm_config.etm_78_sync_freq);

	/* Note that we must enable the ETB before we enable the ETM if we
	 * want to capture the "always true" trigger event. */

	__cpu_enable_etb();
	__cpu_enable_etm();

	put_cpu();
}

static void __cpu_disable_trace(void *unused)
{
	__cpu_disable_etm();
	__cpu_disable_etb();
}

static void enable_trace(void)
{
	if (etm_config.swconfig & TRIGGER_ALL) {
		/* This register is accessible from either core.
		 * CPU1_extout[0] -> CPU0_extin[0]
		 * CPU_extout[0] -> CPU1_extin[0] */
		l2tevselr0_write(0x00000001);
	}

	get_cpu();
	__cpu_enable_trace(NULL);
	smp_call_function(__cpu_enable_trace, NULL, 1);
	put_cpu();
	trace_enabled = 1;
}

static void disable_trace(void)
{
	get_cpu();
	__cpu_disable_trace(NULL);
	smp_call_function(__cpu_disable_trace, NULL, 1);
	put_cpu();
	trace_enabled = 0;
}

static void generate_etb_dump(void)
{
	uint32_t i;
	uint32_t full_slots;
	uint32_t etb_control;
	uint32_t prim_len;
	uint32_t uptime = 0;

	etb_control = etm_read_reg(ETB_REG_CONTROL);
	etb_control |= AIR;
	etm_write_reg(ETB_REG_CONTROL, etb_control);

	if (etm_read_reg(ETB_REG_STATUS) & OV)
		full_slots = ETB_RAM_SLOTS;
	else
		full_slots = etm_read_reg(ETB_REG_ADDRESS) >> 2;

	prim_len = 28 + (full_slots * 4);

	emit_log_char((DATALOG_SYNC >> 8) & 0xFF);
	emit_log_char((DATALOG_SYNC >> 0) & 0xFF);
	emit_log_char((prim_len >> 8) & 0xFF);
	emit_log_char((prim_len >> 0) & 0xFF);
	emit_log_word(uptime);
	emit_log_word(ETB_DUMP_MSG_ID);
	emit_log_word(etm_read_reg(ETM_REG_CONTROL));
	emit_log_word(etm_config.etb_init_ptr >> 2);
	emit_log_word(etm_read_reg(ETB_REG_ADDRESS) >> 2);
	emit_log_word((etm_read_reg(ETB_REG_STATUS) & OV) >> 21);

	etm_write_reg(ETB_REG_ADDRESS, 0x00000000);
	for (i = 0; i < full_slots; i++)
		emit_log_word(etm_read_reg(ETB_REG_DATA));
}

static void generate_etm_dump(void)
{
	uint32_t i;
	uint32_t prim_len;
	uint32_t uptime = 0;

	prim_len = 12 + (4 * ETM_NUM_REGS);

	emit_log_char((DATALOG_SYNC >> 8) & 0xFF);
	emit_log_char((DATALOG_SYNC >> 0) & 0xFF);
	emit_log_char((prim_len >> 8) & 0xFF);
	emit_log_char((prim_len >> 0) & 0xFF);
	emit_log_word(uptime);
	emit_log_word(ETM_DUMP_MSG_ID);

	/* do not disturb ETB_REG_ADDRESS by reading ETB_REG_DATA */
	for (i = 0; i < ETM_NUM_REGS; i++)
		if (i == ETB_REG_DATA)
			emit_log_word(0);
		else
			emit_log_word(etm_read_reg(i));
}

static void dump_all(void *unused)
{
	get_cpu();
	etm_read_reg(0xC5); /* clear sticky bit in PDSR in case
			     * trace hasn't been enabled yet. */
	__cpu_disable_etb();
	generate_etm_dump();
	generate_etb_dump();
	if (trace_enabled)
		__cpu_enable_etb();
	put_cpu();
}

static int next_cpu_to_dump = -1;
static int bytes_to_dump;
static uint8_t *etm_buf_ptr;

static int etm_dev_open(struct inode *inode, struct file *file)
{
	if (atomic_cmpxchg(&etm_dev_in_use, 0, 1))
		return -EBUSY;

	pr_debug("%s: successfully opened\n", __func__);
	return 0;
}

static ssize_t etm_dev_read(struct file *file, char __user *data,
			    size_t len, loff_t *ppos)
{
	if (next_cpu_to_dump == -1) {
		get_cpu();
		dump_all(NULL);
		smp_call_function(dump_all, NULL, 1);
		put_cpu();
		bytes_to_dump = b0.log_end;
		b0.log_end = 0;
		etm_buf_ptr = b0.etm_log_buf;
		next_cpu_to_dump = 0;
	}

	if (len > bytes_to_dump)
		len = bytes_to_dump;

	if (copy_to_user(data, etm_buf_ptr, len)) {
		pr_debug("%s: copy_to_user failed\n", __func__);
		return -EFAULT;
	}

	bytes_to_dump -= len;
	etm_buf_ptr += len;

	pr_debug("%s: %d bytes copied, %d bytes left (cpu %d)\n",
		 __func__, len, bytes_to_dump, next_cpu_to_dump);
	return len;
}

static ssize_t etm_dev_write(struct file *file, const char __user *data,
			     size_t len, loff_t *ppos)
{
	char command;

	if (copy_from_user(&command, data, 1))
		return -EFAULT;

	if (command != '0' && command != '1')
		return -EINVAL;

	switch (command) {
	case '0':
		if (trace_enabled) {
			disable_trace();
			pr_info("etm: tracing disabled\n");
		}
		break;
	case '1':
		if (!trace_enabled) {
			enable_trace();
			pr_info("etm: tracing enabled\n");
		}
		break;
	}
	return len;
}

static int etm_dev_release(struct inode *inode, struct file *file)
{
	if (next_cpu_to_dump == 0) {
		bytes_to_dump = b1.log_end;
		b1.log_end = 0;
		etm_buf_ptr = b1.etm_log_buf;
		next_cpu_to_dump = 1;
	} else
		next_cpu_to_dump = -1;
	atomic_set(&etm_dev_in_use, 0);
	pr_debug("%s: released\n", __func__);
	return 0;
}

static const struct file_operations etm_dev_fops = {
	.owner = THIS_MODULE,
	.open = etm_dev_open,
	.read = etm_dev_read,
	.write = etm_dev_write,
	.release = etm_dev_release,
};

static struct miscdevice etm_dev = {
	.name = "etm",
	.minor = 0,
	.fops = &etm_dev_fops,
};

static int __init etm_init(void)
{
	int rv;
	void __iomem *etm_buf_addr_base;
	void *etm_dump_buf;

	rv = misc_register(&etm_dev);
	if (rv)
		return -ENODEV;

	alloc_b = alloc_percpu(typeof(*alloc_b));
	if (!alloc_b)
		return -ENOMEM;

	*per_cpu_ptr(alloc_b, 0) = &b0;
	*per_cpu_ptr(alloc_b, 1) = &b1;

	pr_info("ETM/ETB intialized.\n");

	if (trace_on_boot)
		enable_trace();

	/* Allocate memory for ETB dump on wdog reset. */
	etm_dump_buf = kmalloc(SZ_16K, GFP_KERNEL);
	if (!etm_dump_buf) {
		pr_err("Could not allocate etm dump buffer.\n");
		return -ENOMEM;
	}
	etm_buf_addr_base = ioremap(0x2a05f000, SZ_4K);
	if (!etm_buf_addr_base) {
		pr_err("Could not map etm buffer dump address.\n");
		kfree(etm_dump_buf);
		return -ENOMEM;
	}
	writel(__pa(etm_dump_buf), etm_buf_addr_base + 0x10);
	iounmap(etm_buf_addr_base);

	return 0;
}

static void __exit etm_exit(void)
{
	disable_trace();
}

module_init(etm_init);
module_exit(etm_exit);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("embedded trace driver");
