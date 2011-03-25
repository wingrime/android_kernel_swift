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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "qs_s5k4e1.h"
/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define REG_GROUPED_PARAMETER_HOLD		0x0104
#define GROUPED_PARAMETER_HOLD_OFF		0x00
#define GROUPED_PARAMETER_HOLD			0x01
/* Integration Time */
#define REG_COARSE_INTEGRATION_TIME		0x0202
/* Gain */
#define REG_GLOBAL_GAIN					0x0204
#define REG_GR_GAIN					0x0206
#define REG_R_GAIN					0x0208
#define REG_B_GAIN					0x020A
#define REG_GB_GAIN					0x020C
/* PLL registers */
#define REG_FRAME_LENGTH_LINES			0x0340
#define REG_LINE_LENGTH_PCK				0x0342
/* Test Pattern */
#define REG_TEST_PATTERN_MODE			0x0601
#define REG_VCM_NEW_CODE				0x30F2
#define AF_ADDR							0x18
#define BRIDGE_ADDR						0x80
/*============================================================================
			 TYPE DECLARATIONS
============================================================================*/

/* 16bit address - 8 bit context register structure */
#define Q8  0x00000100
#define Q10 0x00000400
#define QS_S5K4E1_MASTER_CLK_RATE 24000000
#define QS_S5K4E1_OFFSET			8

/* AF Total steps parameters */
#define QS_S5K4E1_TOTAL_STEPS_NEAR_TO_FAR    32

uint16_t qs_s5k4e1_step_position_table[QS_S5K4E1_TOTAL_STEPS_NEAR_TO_FAR+1];
uint16_t qs_s5k4e1_nl_region_boundary1 = 3;
uint16_t qs_s5k4e1_nl_region_code_per_step1 = 30;
uint16_t qs_s5k4e1_l_region_code_per_step = 4;
uint16_t qs_s5k4e1_damping_threshold = 10;
uint16_t qs_s5k4e1_sw_damping_time_wait = 1;

struct qs_s5k4e1_work_t {
	struct work_struct work;
};

static struct qs_s5k4e1_work_t *qs_s5k4e1_sensorw;
static struct i2c_client *qs_s5k4e1_client;

static bool cali_data_status;
struct qs_s5k4e1_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider;/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;/* init to 1 * 0x00000400 */
	uint16_t fps;

	uint16_t curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;

	enum qs_s5k4e1_resolution_t prev_res;
	enum qs_s5k4e1_resolution_t pict_res;
	enum qs_s5k4e1_resolution_t curr_res;
	enum qs_s5k4e1_test_mode_t  set_test;
	enum qs_s5k4e1_cam_mode_t cam_mode;
};

static uint16_t prev_line_length_pck;
static uint16_t prev_frame_length_lines;
static uint16_t snap_line_length_pck;
static uint16_t snap_frame_length_lines;

static bool CSI_CONFIG;
static struct qs_s5k4e1_ctrl_t *qs_s5k4e1_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(qs_s5k4e1_wait_queue);
DEFINE_MUTEX(qs_s5k4e1_mut);

static int cam_debug_init(void);
static struct dentry *debugfs_base;
/*=============================================================*/

static int qs_s5k4e1_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = length,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = length,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(qs_s5k4e1_client->adapter, msgs, 2) < 0) {
		CDBG("qs_s5k4e1_i2c_rxdata faild 0x%x\n", saddr);
		return -EIO;
	}
	return 0;
}

static int32_t qs_s5k4e1_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
	if (i2c_transfer(qs_s5k4e1_client->adapter, msg, 1) < 0) {
		CDBG("qs_s5k4e1_i2c_txdata faild 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}

static int32_t qs_s5k4e1_i2c_read(unsigned short raddr,
	unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = qs_s5k4e1_i2c_rxdata(qs_s5k4e1_client->addr>>1, buf, rlen);
	if (rc < 0) {
		CDBG("qs_s5k4e1_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	CDBG("qs_s5k4e1_i2c_read 0x%x val = 0x%x!\n", raddr, *rdata);
	return rc;
}

static int32_t qs_s5k4e1_i2c_write_w_sensor(unsigned short waddr,
	 uint16_t wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);
	CDBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, wdata);
	rc = qs_s5k4e1_i2c_txdata(qs_s5k4e1_client->addr>>1, buf, 4);
	if (rc < 0) {
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, wdata);
	}
	return rc;
}

static int32_t qs_s5k4e1_i2c_write_b_sensor(unsigned short waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;
	CDBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	rc = qs_s5k4e1_i2c_txdata(qs_s5k4e1_client->addr>>1, buf, 3);
	if (rc < 0) {
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	}
	return rc;
}

static int32_t qs_s5k4e1_i2c_write_b_table(struct qs_s5k4e1_i2c_reg_conf const
					 *reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EIO;
	for (i = 0; i < num; i++) {
		rc = qs_s5k4e1_i2c_write_b_sensor(reg_conf_tbl->waddr,
			reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}

static int32_t qs_s5k4e1_i2c_write_seq_sensor(unsigned short waddr,
		unsigned char *seq_data, int len)
{
	int32_t rc = -EFAULT;
	unsigned char buf[len+2];
	int i = 0;
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	for (i = 0; i < len; i++)
		buf[i+2] = seq_data[i];
	rc = qs_s5k4e1_i2c_txdata(qs_s5k4e1_client->addr>>1, buf, len+2);
	return rc;
}

static int32_t af_i2c_write_b_sensor(unsigned short baddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[2];
	memset(buf, 0, sizeof(buf));
	buf[0] = baddr;
	buf[1] = bdata;
	CDBG("i2c_write_b addr = 0x%x, val = 0x%x\n", baddr, bdata);
	rc = qs_s5k4e1_i2c_txdata(AF_ADDR>>1, buf, 2);
	if (rc < 0) {
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			baddr, bdata);
	}
	return rc;
}

static int32_t bridge_i2c_write_w(unsigned short waddr, uint16_t wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);
	CDBG("bridge_i2c_write_w addr = 0x%x, val = 0x%x\n", waddr, wdata);
	rc = qs_s5k4e1_i2c_txdata(BRIDGE_ADDR>>1, buf, 4);
	if (rc < 0) {
		CDBG("bridge_i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
			waddr, wdata);
	}
	return rc;
}

static int32_t bridge_i2c_read(unsigned short raddr,
	unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = qs_s5k4e1_i2c_rxdata(BRIDGE_ADDR>>1, buf, rlen);
	if (rc < 0) {
		CDBG("bridge_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	CDBG("bridge_i2c_read 0x%x val = 0x%x!\n", raddr, *rdata);
	return rc;
}

static int32_t qs_s5k4e1_eeprom_i2c_read(unsigned short raddr,
	unsigned char *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned short i2caddr = 0xA0 >> 1;
	unsigned char buf[rlen+2];
	int i = 0;
	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = qs_s5k4e1_i2c_rxdata(i2caddr, buf, rlen);
	if (rc < 0) {
		CDBG("qs_s5k4e1_eeprom_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	for (i = 0; i < rlen; i++) {
		rdata[i] = buf[i];
		CDBG("qs_s5k4e1_eeprom_i2c_read 0x%x index: %d val = 0x%x!\n",
			raddr, i, buf[i]);
	}
	return rc;
}

static int32_t qs_s5k4e1_eeprom_i2c_read_b(unsigned short raddr,
	unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	rc = qs_s5k4e1_eeprom_i2c_read(raddr, &buf[0], rlen);
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	CDBG("qs_s5k4e1_eeprom_i2c_read 0x%x val = 0x%x!\n", raddr, *rdata);
	return rc;
}

static int32_t qs_s5k4e1_get_calibration_data(
	struct sensor_3d_cali_data_t *cdata)
{
	int32_t rc = 0;
	cali_data_status = 1;
	rc = qs_s5k4e1_eeprom_i2c_read(0x0,
		&(cdata->left_p_matrix[0][0][0]), 96);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read(0x60,
		&(cdata->right_p_matrix[0][0][0]), 96);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read(0xC0, &(cdata->square_len[0]), 8);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read(0xC8, &(cdata->focal_len[0]), 8);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read(0xD0, &(cdata->pixel_pitch[0]), 8);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x100, &(cdata->left_r), 1);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x101, &(cdata->right_r), 1);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x102, &(cdata->left_b), 1);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x103, &(cdata->right_b), 1);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x104, &(cdata->left_gb), 1);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x105, &(cdata->right_gb), 1);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x110, &(cdata->left_af_far), 2);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x112, &(cdata->right_af_far), 2);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x114, &(cdata->left_af_mid), 2);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x116, &(cdata->right_af_mid), 2);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x118, &(cdata->left_af_short), 2);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x11A, &(cdata->right_af_short), 2);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x11C, &(cdata->left_af_5um), 2);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x11E, &(cdata->right_af_5um), 2);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x120, &(cdata->left_af_50up), 2);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x122, &(cdata->right_af_50up), 2);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x124, &(cdata->left_af_50down), 2);
	if (rc < 0)
		goto fail;
	rc = qs_s5k4e1_eeprom_i2c_read_b(0x126, &(cdata->right_af_50down), 2);
	if (rc < 0)
		goto fail;

	/*Fix: Temporary disable the 3D Calibration data*/
	goto fail;
	return 0;

fail:
	cali_data_status = 0;
	return -EIO;

}
static int32_t qs_s5k4e1_write_left_lsc(char *left_lsc)
{
	bridge_i2c_write_w(0x06, 0x01);
	qs_s5k4e1_i2c_write_seq_sensor(0x3200, &left_lsc[0], 216);
	qs_s5k4e1_i2c_write_seq_sensor(0x32D8, &left_lsc[216], 216);
	/* qs_s5k4e1_i2c_write_b_sensor(0x3096, 0x40); Fix : lsc setting*/
	return 0;
}

static int32_t qs_s5k4e1_write_right_lsc(char *right_lsc)
{
	bridge_i2c_write_w(0x06, 0x02);
	qs_s5k4e1_i2c_write_seq_sensor(0x3200, &right_lsc[0], 216);
	qs_s5k4e1_i2c_write_seq_sensor(0x32D8, &right_lsc[216], 216);
	/*qs_s5k4e1_i2c_write_b_sensor(0x3096, 0x40); Fix : lsc setting*/
	return 0;
}

static int32_t qs_s5k4e1_write_lsc(char *lsc)
{
	qs_s5k4e1_write_left_lsc(&lsc[0]);
	qs_s5k4e1_write_right_lsc(&lsc[432]);
	bridge_i2c_write_w(0x06, 0x03);
	return 0;
}

static int32_t qs_s5k4e1_read_left_lsc(char *left_lsc)
{
	qs_s5k4e1_eeprom_i2c_read(0x200, &left_lsc[0], 216);
	qs_s5k4e1_eeprom_i2c_read(0x2D8, &left_lsc[216], 216);
	return 0;
}

static int32_t qs_s5k4e1_read_right_lsc(char *right_lsc)
{
	qs_s5k4e1_eeprom_i2c_read(0x3B0, &right_lsc[0], 216);
	qs_s5k4e1_eeprom_i2c_read(0x488, &right_lsc[216], 216);
	return 0;
}

static int32_t qs_s5k4e1_read_lsc(char *lsc)
{
	qs_s5k4e1_read_left_lsc(&lsc[0]);
	qs_s5k4e1_read_right_lsc(&lsc[432]);
	return 0;
}

static void qs_s5k4e1_bridge_reset(void){
	unsigned short RegData = 0, GPIOInState = 0;
	bridge_i2c_write_w(0x50, 0x00);
	bridge_i2c_write_w(0x53, 0x00);
	msleep(30);
	bridge_i2c_write_w(0x53, 0x01);
	msleep(30);
	bridge_i2c_write_w(0x14, 0x0C);
	bridge_i2c_write_w(0x0E, 0xFFFF);

	bridge_i2c_read(0x54, &RegData, 2);
	bridge_i2c_write_w(0x54, (RegData | 0x1));
	msleep(30);
	bridge_i2c_write_w(0x54, (RegData | 0x3));
	bridge_i2c_read(0x54, &RegData, 2);
	bridge_i2c_write_w(0x54, (RegData | 0x4));
	bridge_i2c_write_w(0x54, (RegData | 0xC));

	bridge_i2c_read(0x55, &GPIOInState, 2);
	bridge_i2c_write_w(0x55, (GPIOInState | 0x1));
	msleep(30);
	bridge_i2c_write_w(0x55, (GPIOInState | 0x3));

	bridge_i2c_read(0x55, &GPIOInState, 2);
	bridge_i2c_write_w(0x55, (GPIOInState | 0x4));
	msleep(30);
	bridge_i2c_write_w(0x55, (GPIOInState | 0xC));
	bridge_i2c_read(0x55, &GPIOInState, 2);
	GPIOInState = ((GPIOInState >> 4) & 0x1);

	bridge_i2c_read(0x08, &GPIOInState, 2);
	bridge_i2c_write_w(0x08, GPIOInState | 0x4000);
}

static void qs_s5k4e1_bridge_config(int mode, int rt)
{
	if (mode == MODE_3D) {
		bridge_i2c_write_w(0x16, 0x00);
		bridge_i2c_write_w(0x51, 0x3);
		bridge_i2c_write_w(0x52, 0x1);
		bridge_i2c_write_w(0x06, 0x03);
		bridge_i2c_write_w(0x04, 0x2018);
		bridge_i2c_write_w(0x50, 0x00);
	} else if (mode == MODE_2D_LEFT) {
		bridge_i2c_write_w(0x51, 0x3);
		bridge_i2c_write_w(0x06, 0x01);
		bridge_i2c_write_w(0x04, 0x2018);
		bridge_i2c_write_w(0x50, 0x01);
	} else if (mode == MODE_2D_RIGHT) {
		bridge_i2c_write_w(0x51, 0x3);
		bridge_i2c_write_w(0x06, 0x02);
		bridge_i2c_write_w(0x04, 0x2018);
		bridge_i2c_write_w(0x50, 0x02);
	}
}

static void qs_s5k4e1_group_hold_on(void)
{
	qs_s5k4e1_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD);
}

static void qs_s5k4e1_group_hold_off(void)
{
	qs_s5k4e1_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD_OFF);
}

static void qs_s5k4e1_start_stream(void)
{
	qs_s5k4e1_i2c_write_b_sensor(0x0100, 0x01);
}

static void qs_s5k4e1_stop_stream(void)
{
	qs_s5k4e1_i2c_write_b_sensor(0x0100, 0x00);
}

static void qs_s5k4e1_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider, d1, d2;

	d1 = prev_frame_length_lines * 0x00000400 / snap_frame_length_lines;
	d2 = prev_line_length_pck * 0x00000400 / snap_line_length_pck;
	divider = d1 * d2 / 0x400;

	/*Verify PCLK settings and frame sizes.*/
	*pfps = (uint16_t) (fps * divider / 0x400);
	/* 2 is the ratio of no.of snapshot channels
	to number of preview channels */
}

static uint16_t qs_s5k4e1_get_prev_lines_pf(void)
{

	return prev_frame_length_lines;

}

static uint16_t qs_s5k4e1_get_prev_pixels_pl(void)
{
	return prev_line_length_pck;

}

static uint16_t qs_s5k4e1_get_pict_lines_pf(void)
{
	return snap_frame_length_lines;
}

static uint16_t qs_s5k4e1_get_pict_pixels_pl(void)
{
	return snap_line_length_pck;
}


static uint32_t qs_s5k4e1_get_pict_max_exp_lc(void)
{
	return snap_frame_length_lines  * 24;
}

static int32_t qs_s5k4e1_set_fps(struct fps_cfg   *fps)
{
	uint16_t total_lines_per_frame;
	int32_t rc = 0;
	qs_s5k4e1_ctrl->fps_divider = fps->fps_div;
	qs_s5k4e1_ctrl->pict_fps_divider = fps->pict_fps_div;
	if (qs_s5k4e1_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
		total_lines_per_frame = (uint16_t)
		((prev_frame_length_lines) * qs_s5k4e1_ctrl->fps_divider/0x400);
	} else {
		total_lines_per_frame = (uint16_t)
		((snap_frame_length_lines) * qs_s5k4e1_ctrl->fps_divider/0x400);
	}
	qs_s5k4e1_group_hold_on();
	rc = qs_s5k4e1_i2c_write_w_sensor(REG_FRAME_LENGTH_LINES,
							total_lines_per_frame);
	qs_s5k4e1_group_hold_off();
	return rc;
}

static int32_t qs_s5k4e1_write_exp_gain(struct sensor_3d_exp_cfg exp_cfg)
{
	uint16_t max_legal_gain = 0x0200;
	uint16_t min_ll_pck = 0x0AB2;
	uint32_t ll_pck, fl_lines;
	uint16_t gain = exp_cfg.gain;
	uint32_t line = exp_cfg.line;
	uint32_t ll_ratio;
	int32_t rc = 0;
	if (gain > max_legal_gain) {
		CDBG("Max legal gain Line:%d\n", __LINE__);
		gain = max_legal_gain;
	}
	CDBG("qs_s5k4e1_write_exp_gain : gain = %d line = %d\n", gain, line);

	if (qs_s5k4e1_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
		qs_s5k4e1_ctrl->my_reg_gain = gain;
		qs_s5k4e1_ctrl->my_reg_line_count = (uint16_t) line;
		line = (uint32_t) (line * qs_s5k4e1_ctrl->fps_divider);
		fl_lines = prev_frame_length_lines;
		ll_pck = prev_line_length_pck;
	} else {
		line = (uint32_t) (line * qs_s5k4e1_ctrl->pict_fps_divider);
		fl_lines = snap_frame_length_lines;
		ll_pck = snap_line_length_pck;
	}

	if (fl_lines < (line / 0x400))
		ll_ratio = (line / (fl_lines - QS_S5K4E1_OFFSET));
	else
		ll_ratio = 0x400;

	ll_pck = ll_pck * ll_ratio / 0x400;
	line = line / ll_ratio;
	if (ll_pck < min_ll_pck)
		ll_pck = min_ll_pck;

	qs_s5k4e1_group_hold_on();
	rc = qs_s5k4e1_i2c_write_w_sensor(REG_GLOBAL_GAIN, gain);
	rc = qs_s5k4e1_i2c_write_w_sensor(REG_LINE_LENGTH_PCK, ll_pck);
	rc = qs_s5k4e1_i2c_write_w_sensor(REG_COARSE_INTEGRATION_TIME, line);
	if ((qs_s5k4e1_ctrl->cam_mode == MODE_3D) && (cali_data_status == 1)) {
		bridge_i2c_write_w(0x06, 0x02);
		rc = qs_s5k4e1_i2c_write_w_sensor(REG_GR_GAIN, gain);
		rc = qs_s5k4e1_i2c_write_w_sensor(REG_R_GAIN,
				exp_cfg.r_gain);
		rc = qs_s5k4e1_i2c_write_w_sensor(REG_B_GAIN,
				exp_cfg.b_gain);
		rc = qs_s5k4e1_i2c_write_w_sensor(REG_GB_GAIN,
				exp_cfg.gb_gain);
		bridge_i2c_write_w(0x06, 0x03);
	}
	qs_s5k4e1_group_hold_off();
	return rc;
}

static int32_t qs_s5k4e1_set_pict_exp_gain(struct sensor_3d_exp_cfg exp_cfg)
{
	int32_t rc = 0;
	rc = qs_s5k4e1_write_exp_gain(exp_cfg);
	return rc;
}

#define DIV_CEIL(x, y) ((x/y + ((x%y) ? 1 : 0))

static int32_t qs_s5k4e1_move_focus(int direction,
	int32_t num_steps)
{
	int16_t step_direction, actual_step, next_position;
	uint8_t code_val_msb, code_val_lsb;
	if (direction == MOVE_NEAR)
		step_direction = 16;
	else
		step_direction = -16;

	actual_step = (int16_t) (step_direction * (int16_t) num_steps);
	next_position = (int16_t) (qs_s5k4e1_ctrl->curr_lens_pos + actual_step);

	if (next_position > 1023)
		next_position = 1023;
	else if (next_position < 0)
		next_position = 0;

	code_val_msb = next_position >> 4;
	code_val_lsb = (next_position & 0x000F) << 4;

	if (af_i2c_write_b_sensor(code_val_msb, code_val_lsb) < 0) {
		CDBG("move_focus failed at line %d ...\n", __LINE__);
		return -EBUSY;
	}

	qs_s5k4e1_ctrl->curr_lens_pos = next_position;
	return 0;
}

static int32_t qs_s5k4e1_set_default_focus(uint8_t af_step)
{
	int32_t rc = 0;
	return 0;
	if (qs_s5k4e1_ctrl->curr_step_pos != 0) {
		rc = qs_s5k4e1_move_focus(MOVE_FAR,
		qs_s5k4e1_ctrl->curr_step_pos);
	} else {
		af_i2c_write_b_sensor(0x00, 0x00);
	}

	qs_s5k4e1_ctrl->curr_lens_pos = 0;
	qs_s5k4e1_ctrl->curr_step_pos = 0;

	return rc;
}

static void qs_s5k4e1_init_focus(void)
{
	uint8_t i;
	qs_s5k4e1_step_position_table[0] = 0;
	for (i = 1; i <= QS_S5K4E1_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		if (i <= qs_s5k4e1_nl_region_boundary1) {
			qs_s5k4e1_step_position_table[i] =
				qs_s5k4e1_step_position_table[i-1]
				+ qs_s5k4e1_nl_region_code_per_step1;
		} else {
			qs_s5k4e1_step_position_table[i] =
				qs_s5k4e1_step_position_table[i-1]
				+ qs_s5k4e1_l_region_code_per_step;
		}

		if (qs_s5k4e1_step_position_table[i] > 255)
			qs_s5k4e1_step_position_table[i] = 255;
	}
}

static int32_t qs_s5k4e1_test(enum qs_s5k4e1_test_mode_t mo)
{
	int32_t rc = 0;
	if (mo == TEST_OFF)
		return rc;
	else {
		/* REG_0x30D8[4] is TESBYPEN: 0: Normal Operation,
		1: Bypass Signal Processing
		REG_0x30D8[5] is EBDMASK: 0:
		Output Embedded data, 1: No output embedded data */
		if (qs_s5k4e1_i2c_write_b_sensor(REG_TEST_PATTERN_MODE,
			(uint8_t) mo) < 0) {
			return rc;
		}
	}
	return rc;
}

static int32_t qs_s5k4e1_sensor_setting(int update_type, int rt)
{

	int32_t rc = 0;
	struct msm_camera_csi_params qs_s5k4e1_csi_params;

	qs_s5k4e1_stop_stream();
	msleep(30);
	bridge_i2c_write_w(0x53, 0x00);
	msleep(30);
	if (update_type == REG_INIT) {
		CSI_CONFIG = 0;
		bridge_i2c_write_w(0x53, 0x01);
		msleep(30);
		qs_s5k4e1_bridge_config(qs_s5k4e1_ctrl->cam_mode, rt);
		msleep(30);
		qs_s5k4e1_i2c_write_b_table(qs_s5k4e1_regs.rec_settings,
				qs_s5k4e1_regs.rec_size);
		msleep(10);
	} else if (update_type == UPDATE_PERIODIC) {
		if (!CSI_CONFIG) {
			if (qs_s5k4e1_ctrl->cam_mode == MODE_3D)
				qs_s5k4e1_csi_params.lane_cnt = 4;
			else
				qs_s5k4e1_csi_params.lane_cnt = 2;
			qs_s5k4e1_csi_params.data_format = CSI_10BIT;
			qs_s5k4e1_csi_params.lane_assign = 0xe4;
			qs_s5k4e1_csi_params.dpcm_scheme = 0;
			qs_s5k4e1_csi_params.settle_cnt = 24;
			rc = msm_camio_csi_config(&qs_s5k4e1_csi_params);
			msleep(10);
			cam_debug_init();
			CSI_CONFIG = 1;

		}
		bridge_i2c_write_w(0x53, 0x01);
		msleep(30);
		qs_s5k4e1_i2c_write_b_table(qs_s5k4e1_regs.conf_array[rt].conf,
			qs_s5k4e1_regs.conf_array[rt].size);
		msleep(10);
		qs_s5k4e1_start_stream();
		msleep(50);
	}
	return rc;
}

static int32_t qs_s5k4e1_video_config(int mode)
{

	int32_t rc = 0;
	/* change sensor resolution if needed */
	if (qs_s5k4e1_sensor_setting(UPDATE_PERIODIC,
			qs_s5k4e1_ctrl->prev_res) < 0)
		return rc;
	if (qs_s5k4e1_ctrl->set_test) {
		if (qs_s5k4e1_test(qs_s5k4e1_ctrl->set_test) < 0)
			return  rc;
	}

	qs_s5k4e1_ctrl->curr_res = qs_s5k4e1_ctrl->prev_res;
	qs_s5k4e1_ctrl->sensormode = mode;
	return rc;
}

static int32_t qs_s5k4e1_snapshot_config(int mode)
{
	int32_t rc = 0;
	/*change sensor resolution if needed */
	if (qs_s5k4e1_ctrl->curr_res != qs_s5k4e1_ctrl->pict_res) {
		if (qs_s5k4e1_sensor_setting(UPDATE_PERIODIC,
				qs_s5k4e1_ctrl->pict_res) < 0)
			return rc;
	}

	qs_s5k4e1_ctrl->curr_res = qs_s5k4e1_ctrl->pict_res;
	qs_s5k4e1_ctrl->sensormode = mode;
	return rc;
} /*end of qs_s5k4e1_snapshot_config*/

static int32_t qs_s5k4e1_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	/* change sensor resolution if needed */
	if (qs_s5k4e1_ctrl->curr_res != qs_s5k4e1_ctrl->pict_res) {
		if (qs_s5k4e1_sensor_setting(UPDATE_PERIODIC,
				qs_s5k4e1_ctrl->pict_res) < 0)
			return rc;
	}

	qs_s5k4e1_ctrl->curr_res = qs_s5k4e1_ctrl->pict_res;
	qs_s5k4e1_ctrl->sensormode = mode;
	return rc;
} /*end of qs_s5k4e1_raw_snapshot_config*/

static int32_t qs_s5k4e1_mode_init(int mode, struct sensor_init_cfg init_info)
{
	int32_t rc = 0;
	if (mode != qs_s5k4e1_ctrl->cam_mode) {
		qs_s5k4e1_ctrl->prev_res = init_info.prev_res;
		qs_s5k4e1_ctrl->pict_res = init_info.pict_res;
		qs_s5k4e1_ctrl->cam_mode = mode;

		prev_frame_length_lines =
		((qs_s5k4e1_regs.conf_array[qs_s5k4e1_ctrl->prev_res]\
			.conf[QS_S5K4E1_FRAME_LENGTH_LINES_H].wdata << 8)
			| qs_s5k4e1_regs.conf_array[qs_s5k4e1_ctrl->prev_res]\
			.conf[QS_S5K4E1_FRAME_LENGTH_LINES_L].wdata);
		prev_line_length_pck =
		(qs_s5k4e1_regs.conf_array[qs_s5k4e1_ctrl->prev_res]\
			.conf[QS_S5K4E1_LINE_LENGTH_PCK_H].wdata << 8)
			| qs_s5k4e1_regs.conf_array[qs_s5k4e1_ctrl->prev_res]\
			.conf[QS_S5K4E1_LINE_LENGTH_PCK_L].wdata;
		snap_frame_length_lines =
		(qs_s5k4e1_regs.conf_array[qs_s5k4e1_ctrl->pict_res]\
			.conf[QS_S5K4E1_FRAME_LENGTH_LINES_H].wdata << 8)
			| qs_s5k4e1_regs.conf_array[qs_s5k4e1_ctrl->pict_res]\
			.conf[QS_S5K4E1_FRAME_LENGTH_LINES_L].wdata;
		snap_line_length_pck =
		(qs_s5k4e1_regs.conf_array[qs_s5k4e1_ctrl->pict_res]\
			.conf[QS_S5K4E1_LINE_LENGTH_PCK_H].wdata << 8)
			| qs_s5k4e1_regs.conf_array[qs_s5k4e1_ctrl->pict_res]\
			.conf[QS_S5K4E1_LINE_LENGTH_PCK_L].wdata;

	rc = qs_s5k4e1_sensor_setting(REG_INIT,
		qs_s5k4e1_ctrl->prev_res);
	}
	return rc;
}
static int32_t qs_s5k4e1_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		qs_s5k4e1_ctrl->prev_res = res;
		rc = qs_s5k4e1_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		qs_s5k4e1_ctrl->pict_res = res;
		rc = qs_s5k4e1_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		qs_s5k4e1_ctrl->pict_res = res;
		rc = qs_s5k4e1_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int32_t qs_s5k4e1_power_down(void)
{
	return 0;
}

static int qs_s5k4e1_probe_init_done(const struct msm_camera_sensor_info *data)
{
	CDBG("probe done\n");
	gpio_free(data->sensor_reset);
	return 0;
}

static int
	qs_s5k4e1_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	CDBG("%s: %d\n", __func__, __LINE__);
	rc = gpio_request(data->sensor_reset, "qs_s5k4e1");
	CDBG(" qs_s5k4e1_probe_init_sensor\n");
	if (!rc) {
		CDBG("sensor_reset = %d\n", rc);
		gpio_direction_output(data->sensor_reset, 0);
		msleep(50);
		gpio_set_value_cansleep(data->sensor_reset, 1);
		msleep(13);
	} else {
		goto init_probe_done;
	}
	msleep(70);
	qs_s5k4e1_bridge_reset();
	qs_s5k4e1_bridge_config(MODE_3D, RES_PREVIEW);
	msleep(30);

	CDBG(" qs_s5k4e1_probe_init_sensor is called\n");
	rc = qs_s5k4e1_i2c_read(0x0000, &chipid, 2);
	CDBG("ID: %d\n", chipid);
	/* 4. Compare sensor ID to QS_S5K4E1 ID: */
	if (chipid != 0x4e10) {
		rc = -ENODEV;
		CDBG("qs_s5k4e1_probe_init_sensor fail chip id mismatch\n");
		goto init_probe_fail;
	}
	goto init_probe_done;
init_probe_fail:
	CDBG(" qs_s5k4e1_probe_init_sensor fails\n");
	gpio_set_value_cansleep(data->sensor_reset, 0);
	qs_s5k4e1_probe_init_done(data);
init_probe_done:
	CDBG(" qs_s5k4e1_probe_init_sensor finishes\n");
	return rc;
}
/* camsensor_qs_s5k4e1_reset */

int qs_s5k4e1_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	CDBG("%s: %d\n", __func__, __LINE__);
	CDBG("Calling qs_s5k4e1_sensor_open_init\n");

	qs_s5k4e1_ctrl = kzalloc(sizeof(struct qs_s5k4e1_ctrl_t), GFP_KERNEL);
	if (!qs_s5k4e1_ctrl) {
		CDBG("qs_s5k4e1_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	qs_s5k4e1_ctrl->fps_divider = 1 * 0x00000400;
	qs_s5k4e1_ctrl->pict_fps_divider = 1 * 0x00000400;
	qs_s5k4e1_ctrl->set_test = TEST_OFF;
	qs_s5k4e1_ctrl->cam_mode = MODE_INVALID;

	if (data)
		qs_s5k4e1_ctrl->sensordata = data;
	if (rc < 0) {
		CDBG("Calling qs_s5k4e1_sensor_open_init fail1\n");
		return rc;
	}
	CDBG("%s: %d\n", __func__, __LINE__);
	/* enable mclk first */
	msm_camio_clk_rate_set(QS_S5K4E1_MASTER_CLK_RATE);
	rc = qs_s5k4e1_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail;
/*Default mode is 3D*/
	qs_s5k4e1_write_lsc(data->eeprom_data);
	qs_s5k4e1_ctrl->fps = 30*Q8;
	qs_s5k4e1_init_focus();
	if (rc < 0) {
		gpio_set_value_cansleep(data->sensor_reset, 0);
		goto init_fail;
	} else
		goto init_done;
init_fail:
	CDBG("init_fail\n");
	qs_s5k4e1_probe_init_done(data);
init_done:
	CDBG("init_done\n");
	return rc;
} /*endof qs_s5k4e1_sensor_open_init*/

static int qs_s5k4e1_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&qs_s5k4e1_wait_queue);
	return 0;
}

static const struct i2c_device_id qs_s5k4e1_i2c_id[] = {
	{"qs_s5k4e1", 0},
	{ }
};

static int qs_s5k4e1_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	CDBG("qs_s5k4e1_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	qs_s5k4e1_sensorw = kzalloc(sizeof(struct qs_s5k4e1_work_t),
		 GFP_KERNEL);
	if (!qs_s5k4e1_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, qs_s5k4e1_sensorw);
	qs_s5k4e1_init_client(client);
	qs_s5k4e1_client = client;

	msleep(50);

	CDBG("qs_s5k4e1_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	CDBG("qs_s5k4e1_probe failed! rc = %d\n", rc);
	return rc;
}

static int qs_s5k4e1_send_wb_info(struct wb_info_cfg *wb)
{
	return 0;

} /*end of qs_s5k4e1_snapshot_config*/

static int __exit qs_s5k4e1_remove(struct i2c_client *client)
{
	struct qs_s5k4e1_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	qs_s5k4e1_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver qs_s5k4e1_i2c_driver = {
	.id_table = qs_s5k4e1_i2c_id,
	.probe  = qs_s5k4e1_i2c_probe,
	.remove = __exit_p(qs_s5k4e1_i2c_remove),
	.driver = {
		.name = "qs_s5k4e1",
	},
};

int qs_s5k4e1_3D_sensor_config(void __user *argp)
{
	struct sensor_large_data cdata;
	long rc;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_large_data)))
		return -EFAULT;
	mutex_lock(&qs_s5k4e1_mut);
	rc = qs_s5k4e1_get_calibration_data
		(&cdata.data.sensor_3d_cali_data);
	if (rc < 0)
		goto fail;
	if (copy_to_user((void *)argp,
		&cdata,
		sizeof(struct sensor_large_data)))
		rc = -EFAULT;
fail:
	mutex_unlock(&qs_s5k4e1_mut);
	return rc;
}

int qs_s5k4e1_2D_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&qs_s5k4e1_mut);
	CDBG("qs_s5k4e1_sensor_config: cfgtype = %d\n",
	cdata.cfgtype);
		switch (cdata.cfgtype) {
		case CFG_GET_PICT_FPS:
			qs_s5k4e1_get_pict_fps(
				cdata.cfg.gfps.prevfps,
				&(cdata.cfg.gfps.pictfps));

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_L_PF:
			cdata.cfg.prevl_pf =
			qs_s5k4e1_get_prev_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_P_PL:
			cdata.cfg.prevp_pl =
				qs_s5k4e1_get_prev_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_L_PF:
			cdata.cfg.pictl_pf =
				qs_s5k4e1_get_pict_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_P_PL:
			cdata.cfg.pictp_pl =
				qs_s5k4e1_get_pict_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_MAX_EXP_LC:
			cdata.cfg.pict_max_exp_lc =
				qs_s5k4e1_get_pict_max_exp_lc();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_FPS:
		case CFG_SET_PICT_FPS:
			rc = qs_s5k4e1_set_fps(&(cdata.cfg.fps));
			break;

		case CFG_SET_EXP_GAIN:
			rc =
				qs_s5k4e1_write_exp_gain(
					cdata.cfg.sensor_3d_exp);
			break;

		case CFG_SET_PICT_EXP_GAIN:
			rc =
				qs_s5k4e1_set_pict_exp_gain(
				cdata.cfg.sensor_3d_exp);
			break;

		case CFG_SET_MODE:
			rc = qs_s5k4e1_set_sensor_mode(cdata.mode,
					cdata.rs);
			break;

		case CFG_PWR_DOWN:
			rc = qs_s5k4e1_power_down();
			break;

		case CFG_MOVE_FOCUS:
			rc =
				qs_s5k4e1_move_focus(
				cdata.cfg.focus.dir,
				cdata.cfg.focus.steps);
			break;

		case CFG_SET_DEFAULT_FOCUS:
			rc =
				qs_s5k4e1_set_default_focus(
				cdata.cfg.focus.steps);
			break;

		case CFG_GET_AF_MAX_STEPS:
			cdata.max_steps = QS_S5K4E1_TOTAL_STEPS_NEAR_TO_FAR;
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_EFFECT:
			rc = qs_s5k4e1_set_default_focus(
				cdata.cfg.effect);
			break;


		case CFG_SEND_WB_INFO:
			rc = qs_s5k4e1_send_wb_info(
				&(cdata.cfg.wb_info));
			break;

		case CFG_SENSOR_INIT:
			rc = qs_s5k4e1_mode_init(cdata.mode,
					cdata.cfg.init_info);
			break;

		default:
			rc = -EFAULT;
			break;
		}

	mutex_unlock(&qs_s5k4e1_mut);

	return rc;
}

int qs_s5k4e1_sensor_config(void __user *argp)
{
	int cfgtype;
	long rc;
	if (copy_from_user(&cfgtype,
		(void *)argp,
		sizeof(int)))
		return -EFAULT;
	if (cfgtype != CFG_GET_3D_CALI_DATA)
		rc = qs_s5k4e1_2D_sensor_config(argp);
	else
		rc = qs_s5k4e1_3D_sensor_config(argp);
	return rc;
}

static int qs_s5k4e1_sensor_release(void)
{
	int rc = -EBADF;
	mutex_lock(&qs_s5k4e1_mut);
	qs_s5k4e1_power_down();
	bridge_i2c_write_w(0x53, 0x00);
	msleep(20);
	gpio_set_value_cansleep(qs_s5k4e1_ctrl->sensordata->sensor_reset, 0);
	msleep(5);
	gpio_free(qs_s5k4e1_ctrl->sensordata->sensor_reset);
	kfree(qs_s5k4e1_ctrl);
	qs_s5k4e1_ctrl = NULL;
	CDBG("qs_s5k4e1_release completed\n");
	mutex_unlock(&qs_s5k4e1_mut);

	return rc;
}

static int qs_s5k4e1_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	rc = i2c_add_driver(&qs_s5k4e1_i2c_driver);
	if (rc < 0 || qs_s5k4e1_client == NULL) {
		rc = -ENOTSUPP;
		CDBG("I2C add driver failed");
		goto probe_fail;
	}
	msm_camio_clk_rate_set(QS_S5K4E1_MASTER_CLK_RATE);
	rc = qs_s5k4e1_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;
	qs_s5k4e1_read_lsc(info->eeprom_data); /*Default mode is 3D*/
	s->s_init = qs_s5k4e1_sensor_open_init;
	s->s_release = qs_s5k4e1_sensor_release;
	s->s_config  = qs_s5k4e1_sensor_config;
	s->s_mount_angle = 90;
	s->s_camera_type = BACK_CAMERA_3D;
	s->s_video_packing = SIDE_BY_SIDE_HALF;
	s->s_snap_packing = SIDE_BY_SIDE_FULL;
	bridge_i2c_write_w(0x53, 0x00);
	msleep(20);
	gpio_set_value_cansleep(info->sensor_reset, 0);
	qs_s5k4e1_probe_init_done(info);
	return rc;

probe_fail:
	CDBG("qs_s5k4e1_sensor_probe: SENSOR PROBE FAILS!\n");
	return rc;
}

static bool streaming = 1;

static int qs_s5k4e1_focus_test(void *data, u64 *val)
{
	int i = 0;
	qs_s5k4e1_set_default_focus(0);

	for (i = 90; i < 256; i++) {
		qs_s5k4e1_i2c_write_w_sensor(REG_VCM_NEW_CODE, i);
		msleep(5000);
	}
	msleep(5000);
	for (i = 255; i > 90; i--) {
		qs_s5k4e1_i2c_write_w_sensor(REG_VCM_NEW_CODE, i);
		msleep(5000);
	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_focus, qs_s5k4e1_focus_test,
			NULL, "%lld\n");

static int qs_s5k4e1_step_test(void *data, u64 *val)
{
	int i = 0;
	uint16_t RegMSB, RegLSB;
	for (i = 0; i < 1024; i += 10) {
		RegMSB = i >> 4;
		RegLSB = (i & 0x000F) << 4;
		af_i2c_write_b_sensor(RegMSB, RegLSB);
		msleep(20);
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_step, qs_s5k4e1_step_test,
			NULL, "%lld\n");

static int cam_debug_stream_set(void *data, u64 val)
{
	int rc = 0;

	if (val) {
		qs_s5k4e1_start_stream();
		streaming = 1;
	} else {
		qs_s5k4e1_stop_stream();
		streaming = 0;
	}

	return rc;
}

static int cam_debug_stream_get(void *data, u64 *val)
{
	*val = streaming;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cam_stream, cam_debug_stream_get,
			cam_debug_stream_set, "%llu\n");

static int cam_debug_init(void)
{
	struct dentry *cam_dir;
	debugfs_base = debugfs_create_dir("sensor", NULL);
	if (!debugfs_base)
		return -ENOMEM;

	cam_dir = debugfs_create_dir("qs_s5k4e1", debugfs_base);
	if (!cam_dir)
		return -ENOMEM;

	if (!debugfs_create_file("focus", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_focus))
		return -ENOMEM;
	if (!debugfs_create_file("step", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_step))
		return -ENOMEM;
	if (!debugfs_create_file("stream", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_stream))
		return -ENOMEM;
	return 0;
}

static int __qs_s5k4e1_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, qs_s5k4e1_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __qs_s5k4e1_probe,
	.driver = {
		.name = "msm_camera_qs_s5k4e1",
	.owner = THIS_MODULE,
	},
};

static int __init qs_s5k4e1_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(qs_s5k4e1_init);
void qs_s5k4e1_exit(void)
{
	i2c_del_driver(&qs_s5k4e1_i2c_driver);
}
MODULE_DESCRIPTION("Samsung 5MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");
