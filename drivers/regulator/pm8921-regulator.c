/*
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/pm8921-regulator.h>
#include <linux/mfd/pm8xxx/core.h>

#define REGULATOR_TYPE_LDO		0
#define REGULATOR_TYPE_NLDO1200		1
#define REGULATOR_TYPE_SMPS		2
#define REGULATOR_TYPE_FTSMPS		3
#define REGULATOR_TYPE_VS		4
#define REGULATOR_TYPE_VS300		5
#define REGULATOR_TYPE_NCP		6

/* Common Masks */
#define REGULATOR_ENABLE_MASK		0x80
#define REGULATOR_ENABLE		0x80
#define REGULATOR_DISABLE		0x00

#define REGULATOR_BANK_MASK		0xF0
#define REGULATOR_BANK_SEL(n)		((n) << 4)
#define REGULATOR_BANK_WRITE		0x80

#define LDO_TEST_BANKS			7
#define NLDO1200_TEST_BANKS		5
#define SMPS_TEST_BANKS			8
#define REGULATOR_TEST_BANKS_MAX	SMPS_TEST_BANKS

/* LDO masks and values */

/* CTRL register */
#define LDO_ENABLE_MASK			0x80
#define LDO_ENABLE			0x80
#define LDO_PULL_DOWN_ENABLE_MASK	0x40
#define LDO_PULL_DOWN_ENABLE		0x40

#define LDO_CTRL_PM_MASK		0x20
#define LDO_CTRL_PM_HPM			0x00
#define LDO_CTRL_PM_LPM			0x20

#define LDO_CTRL_VPROG_MASK		0x1F

/* TEST register bank 0 */
#define LDO_TEST_LPM_MASK		0x40
#define LDO_TEST_LPM_SEL_CTRL		0x00
#define LDO_TEST_LPM_SEL_TCXO		0x40

/* TEST register bank 2 */
#define LDO_TEST_VPROG_UPDATE_MASK	0x08
#define LDO_TEST_RANGE_SEL_MASK		0x04
#define LDO_TEST_FINE_STEP_MASK		0x02
#define LDO_TEST_FINE_STEP_SHIFT	1

/* TEST register bank 4 */
#define LDO_TEST_RANGE_EXT_MASK		0x01

/* TEST register bank 5 */
#define LDO_TEST_PIN_CTRL_MASK		0x0F
#define LDO_TEST_PIN_CTRL_EN3		0x08
#define LDO_TEST_PIN_CTRL_EN2		0x04
#define LDO_TEST_PIN_CTRL_EN1		0x02
#define LDO_TEST_PIN_CTRL_EN0		0x01

/* TEST register bank 6 */
#define LDO_TEST_PIN_CTRL_LPM_MASK	0x0F

/* Allowable voltage ranges */
#define PLDO_LOW_UV_MIN			750000
#define PLDO_LOW_UV_MAX			1537500
#define PLDO_LOW_FINE_STEP_UV		12500

#define PLDO_NORM_UV_MIN		1500000
#define PLDO_NORM_UV_MAX		3075000
#define PLDO_NORM_FINE_STEP_UV		25000

#define PLDO_HIGH_UV_MIN		1750000
#define PLDO_HIGH_UV_MAX		4900000
#define PLDO_HIGH_FINE_STEP_UV		50000

#define NLDO_UV_MIN			750000
#define NLDO_UV_MAX			1537500
#define NLDO_FINE_STEP_UV		12500

/* NLDO1200 masks and values */

/* CTRL register */
#define NLDO1200_ENABLE_MASK		0x80
#define NLDO1200_LDO_ENABLE		0x80

/* Legacy mode */
#define NLDO1200_LEGACY_PM_MASK		0x20
#define NLDO1200_LEGACY_PM_HPM		0x00
#define NLDO1200_LEGACY_PM_LPM		0x20

/* Advanced mode */
#define NLDO1200_CTRL_RANGE_MASK	0x40
#define NLDO1200_CTRL_RANGE_HIGH	0x00
#define NLDO1200_CTRL_RANGE_LOW		0x40
#define NLDO1200_CTRL_VPROG_MASK	0x3F

#define NLDO1200_LOW_UV_MIN		375000
#define NLDO1200_LOW_UV_MAX		768750
#define NLDO1200_LOW_UV_STEP		6250

#define NLDO1200_HIGH_UV_MIN		750000
#define NLDO1200_HIGH_UV_MAX		1537500
#define NLDO1200_HIGH_UV_STEP		12500

/* TEST register bank 0 */
#define NLDO1200_TEST_LPM_MASK		0x04
#define NLDO1200_TEST_LPM_SEL_CTRL	0x00
#define NLDO1200_TEST_LPM_SEL_TCXO	0x04

/* TEST register bank 1 */
#define NLDO1200_PULL_DOWN_ENABLE_MASK	0x02
#define NLDO1200_PULL_DOWN_ENABLE	0x02

/* TEST register bank 2 */
#define NLDO1200_ADVANCED_MODE_MASK	0x08
#define NLDO1200_ADVANCED_MODE		0x00
#define NLDO1200_LEGACY_MODE		0x08

/* Advanced mode power mode control */
#define NLDO1200_ADVANCED_PM_MASK	0x02
#define NLDO1200_ADVANCED_PM_HPM	0x00
#define NLDO1200_ADVANCED_PM_LPM	0x02

#define NLDO1200_IN_ADVANCED_MODE(vreg) \
	((vreg->test_reg[2] & NLDO1200_ADVANCED_MODE_MASK) \
	 == NLDO1200_ADVANCED_MODE)

/* SMPS masks and values */

/* CTRL register */

/* Legacy mode */
#define SMPS_LEGACY_ENABLE		0x80
#define SMPS_LEGACY_PULL_DOWN_ENABLE	0x40
#define SMPS_LEGACY_VREF_SEL_MASK	0x20
#define SMPS_LEGACY_VPROG_MASK		0x1F

/* Advanced mode */
#define SMPS_ADVANCED_BAND_MASK		0xC0
#define SMPS_ADVANCED_BAND_OFF		0x00
#define SMPS_ADVANCED_BAND_1		0x40
#define SMPS_ADVANCED_BAND_2		0x80
#define SMPS_ADVANCED_BAND_3		0xC0
#define SMPS_ADVANCED_VPROG_MASK	0x3F

/* Legacy mode voltage ranges */
#define SMPS_MODE1_UV_MIN		1500000
#define SMPS_MODE1_UV_MAX		3050000
#define SMPS_MODE1_UV_STEP		50000

#define SMPS_MODE2_UV_MIN		750000
#define SMPS_MODE2_UV_MAX		1525000
#define SMPS_MODE2_UV_STEP		25000

#define SMPS_MODE3_UV_MIN		375000
#define SMPS_MODE3_UV_MAX		1150000
#define SMPS_MODE3_UV_STEP		25000

/* Advanced mode voltage ranges */
#define SMPS_BAND3_UV_MIN		1500000
#define SMPS_BAND3_UV_MAX		3075000
#define SMPS_BAND3_UV_STEP		25000

#define SMPS_BAND2_UV_MIN		750000
#define SMPS_BAND2_UV_MAX		1537500
#define SMPS_BAND2_UV_STEP		12500

#define SMPS_BAND1_UV_MIN		375000
#define SMPS_BAND1_UV_MAX		1162500
#define SMPS_BAND1_UV_STEP		12500

#define SMPS_UV_MIN			SMPS_MODE3_UV_MIN
#define SMPS_UV_MAX			SMPS_MODE1_UV_MAX

/* Test2 register bank 1 */
#define SMPS_LEGACY_VLOW_SEL_MASK	0x01

/* Test2 register bank 6 */
#define SMPS_ADVANCED_PULL_DOWN_ENABLE	0x08

/* Test2 register bank 7 */
#define SMPS_ADVANCED_MODE_MASK		0x02
#define SMPS_ADVANCED_MODE		0x02
#define SMPS_LEGACY_MODE		0x00

#define SMPS_IN_ADVANCED_MODE(vreg) \
	((vreg->test_reg[7] & SMPS_ADVANCED_MODE_MASK) == SMPS_ADVANCED_MODE)

/* BUCK_SLEEP_CNTRL register */
#define SMPS_PIN_CTRL_MASK		0xF0
#define SMPS_PIN_CTRL_A1		0x80
#define SMPS_PIN_CTRL_A0		0x40
#define SMPS_PIN_CTRL_D1		0x20
#define SMPS_PIN_CTRL_D0		0x10

#define SMPS_PIN_CTRL_LPM_MASK		0x0F
#define SMPS_PIN_CTRL_LPM_A1		0x08
#define SMPS_PIN_CTRL_LPM_A0		0x04
#define SMPS_PIN_CTRL_LPM_D1		0x02
#define SMPS_PIN_CTRL_LPM_D0		0x01

/* BUCK_CLOCK_CNTRL register */
#define SMPS_CLK_DIVIDE2		0x40

#define SMPS_CLK_CTRL_MASK		0x30
#define SMPS_CLK_CTRL_FOLLOW_TCXO	0x00
#define SMPS_CLK_CTRL_PWM		0x10
#define SMPS_CLK_CTRL_PFM		0x20

/* FTSMPS masks and values */

/* CTRL register */
#define FTSMPS_VCTRL_BAND_MASK		0xC0
#define FTSMPS_VCTRL_BAND_OFF		0x00
#define FTSMPS_VCTRL_BAND_1		0x40
#define FTSMPS_VCTRL_BAND_2		0x80
#define FTSMPS_VCTRL_BAND_3		0xC0
#define FTSMPS_VCTRL_VPROG_MASK		0x3F

#define FTSMPS_BAND1_UV_MIN		350000
#define FTSMPS_BAND1_UV_MAX		650000
#define FTSMPS_BAND1_UV_STEP		6250

#define FTSMPS_BAND2_UV_MIN		700000
#define FTSMPS_BAND2_UV_MAX		1400000
#define FTSMPS_BAND2_UV_STEP		12500

#define FTSMPS_BAND3_UV_SETPOINT_MIN	1500000
#define FTSMPS_BAND3_UV_MIN		1400000
#define FTSMPS_BAND3_UV_MAX		3300000
#define FTSMPS_BAND3_UV_STEP		50000

#define FTSMPS_UV_MIN			FTSMPS_BAND_1_UV_MIN
#define FTSMPS_UV_MAX			FTSMPS_BAND_3_UV_MAX

/* FTS_CNFG1 register bank 0 */
#define FTSMPS_CNFG1_PM_MASK		0x0C
#define FTSMPS_CNFG1_PM_PWM		0x00
#define FTSMPS_CNFG1_PM_PFM		0x08

/* PWR_CNFG register */
#define FTSMPS_PULL_DOWN_ENABLE_MASK	0x40
#define FTSMPS_PULL_DOWN_ENABLE		0x40

/*
 * Band 1 of PMIC 8921 FTSMPS regulators only supports set points with the 3
 * LSB's equal to 0.  This is accomplished in the macro by truncating the bits.
 */
#define PM8921_FTSMPS_BAND_1_COMPENSATE(vprog)	((vprog) & ~0x7)

/* VS masks and values */

/* CTRL register */
#define VS_ENABLE_MASK			0x80
#define VS_ENABLE			0x80
#define VS_PULL_DOWN_ENABLE_MASK	0x40
#define VS_PULL_DOWN_ENABLE		0x40

#define VS_PIN_CTRL_MASK		0x0F
#define VS_PIN_CTRL_EN0			0x08
#define VS_PIN_CTRL_EN1			0x04
#define VS_PIN_CTRL_EN2			0x02
#define VS_PIN_CTRL_EN3			0x01

/* VS300 masks and values */

/* CTRL register */
#define VS300_CTRL_ENABLE_MASK		0xC0
#define VS300_CTRL_DISABLE		0x00
#define VS300_CTRL_ENABLE		0x40

#define VS300_PULL_DOWN_ENABLE_MASK	0x20
#define VS300_PULL_DOWN_ENABLE		0x20

/* NCP masks and values */

/* CTRL register */
#define NCP_ENABLE_MASK			0x80
#define NCP_ENABLE			0x80
#define NCP_VPROG_MASK			0x1F

#define NCP_UV_MIN			1500000
#define NCP_UV_MAX			3050000
#define NCP_UV_STEP			50000

struct pm8921_vreg {
	/* Configuration data */
	struct regulator_dev			*rdev;
	struct device				*dev;
	const char				*name;
	struct pm8921_regulator_platform_data	pdata;
	const int				hpm_min_load;
	const u16				ctrl_addr;
	const u16				test_addr;
	const u16				clk_ctrl_addr;
	const u16				sleep_ctrl_addr;
	const u16				pfm_ctrl_addr;
	const u16				pwr_cnfg_addr;
	const u8				type;
	/* State data */
	int					save_uV;
	unsigned				pc_vote;
	unsigned				optimum;
	u8				test_reg[REGULATOR_TEST_BANKS_MAX];
	u8					ctrl_reg;
	u8					clk_ctrl_reg;
	u8					sleep_ctrl_reg;
	u8					pfm_ctrl_reg;
	u8					pwr_cnfg_reg;
	u8					state;
};

#define vreg_err(vreg, fmt, ...) \
	pr_err("%s: " fmt, vreg->name, ##__VA_ARGS__)

#define LDO(_id, _ctrl_addr, _test_addr, _hpm_min_load) \
	[PM8921_VREG_ID_##_id] = { \
		.type		= REGULATOR_TYPE_LDO, \
		.ctrl_addr	= _ctrl_addr, \
		.test_addr	= _test_addr, \
		.hpm_min_load	= PM8921_VREG_##_hpm_min_load##_HPM_MIN_LOAD, \
	}

#define NLDO1200(_id, _ctrl_addr, _test_addr, _hpm_min_load) \
	[PM8921_VREG_ID_##_id] = { \
		.type		= REGULATOR_TYPE_NLDO1200, \
		.ctrl_addr	= _ctrl_addr, \
		.test_addr	= _test_addr, \
		.hpm_min_load	= PM8921_VREG_##_hpm_min_load##_HPM_MIN_LOAD, \
	}

#define SMPS(_id, _ctrl_addr, _test_addr, _clk_ctrl_addr, _sleep_ctrl_addr, \
	     _hpm_min_load) \
	[PM8921_VREG_ID_##_id] = { \
		.type		= REGULATOR_TYPE_SMPS, \
		.ctrl_addr	= _ctrl_addr, \
		.test_addr	= _test_addr, \
		.clk_ctrl_addr	= _clk_ctrl_addr, \
		.sleep_ctrl_addr = _sleep_ctrl_addr, \
		.hpm_min_load	= PM8921_VREG_##_hpm_min_load##_HPM_MIN_LOAD, \
	}

#define FTSMPS(_id, _pwm_ctrl_addr, _fts_cnfg1_addr, _pfm_ctrl_addr, \
	       _pwr_cnfg_addr, _hpm_min_load) \
	[PM8921_VREG_ID_##_id] = { \
		.type		= REGULATOR_TYPE_FTSMPS, \
		.ctrl_addr	= _pwm_ctrl_addr, \
		.test_addr	= _fts_cnfg1_addr, \
		.pfm_ctrl_addr = _pfm_ctrl_addr, \
		.pwr_cnfg_addr = _pwr_cnfg_addr, \
		.hpm_min_load	= PM8921_VREG_##_hpm_min_load##_HPM_MIN_LOAD, \
	}

#define VS(_id, _ctrl_addr) \
	[PM8921_VREG_ID_##_id] = { \
		.type		= REGULATOR_TYPE_VS, \
		.ctrl_addr	= _ctrl_addr, \
	}

#define VS300(_id, _ctrl_addr) \
	[PM8921_VREG_ID_##_id] = { \
		.type		= REGULATOR_TYPE_VS300, \
		.ctrl_addr	= _ctrl_addr, \
	}

#define NCP(_id, _ctrl_addr) \
	[PM8921_VREG_ID_##_id] = { \
		.type		= REGULATOR_TYPE_NCP, \
		.ctrl_addr	= _ctrl_addr, \
	}

static struct pm8921_vreg pm8921_vreg[] = {
	/*  id   ctrl   test   hpm_min */
	LDO(L1,  0x0AE, 0x0AF, LDO_150),
	LDO(L2,  0x0B0, 0x0B1, LDO_150),
	LDO(L3,  0x0B2, 0x0B3, LDO_150),
	LDO(L4,  0x0B4, 0x0B5, LDO_50),
	LDO(L5,  0x0B6, 0x0B7, LDO_300),
	LDO(L6,  0x0B8, 0x0B9, LDO_600),
	LDO(L7,  0x0BA, 0x0BB, LDO_150),
	LDO(L8,  0x0BC, 0x0BD, LDO_300),
	LDO(L9,  0x0BE, 0x0BF, LDO_300),
	LDO(L10, 0x0C0, 0x0C1, LDO_600),
	LDO(L11, 0x0C2, 0x0C3, LDO_150),
	LDO(L12, 0x0C4, 0x0C5, LDO_150),
	LDO(L14, 0x0C8, 0x0C9, LDO_50),
	LDO(L15, 0x0CA, 0x0CB, LDO_150),
	LDO(L16, 0x0CC, 0x0CD, LDO_300),
	LDO(L17, 0x0CE, 0x0CF, LDO_150),
	LDO(L18, 0x0D0, 0x0D1, LDO_150),
	LDO(L21, 0x0D6, 0x0D7, LDO_150),
	LDO(L22, 0x0D8, 0x0D9, LDO_150),
	LDO(L23, 0x0DA, 0x0DB, LDO_150),

	/*       id   ctrl   test   hpm_min */
	NLDO1200(L24, 0x0DC, 0x0DD, LDO_1200),
	NLDO1200(L25, 0x0DE, 0x0DF, LDO_1200),
	NLDO1200(L26, 0x0E0, 0x0E1, LDO_1200),
	NLDO1200(L27, 0x0E2, 0x0E3, LDO_1200),
	NLDO1200(L28, 0x0E4, 0x0E5, LDO_1200),

	/*  id   ctrl   test   hpm_min */
	LDO(L29, 0x0E6, 0x0E7, LDO_150),

	/*   id  ctrl   test2  clk    sleep  hpm_min */
	SMPS(S1, 0x1D0, 0x1D5, 0x009, 0x1D2, SMPS_1500),
	SMPS(S2, 0x1D8, 0x1DD, 0x00A, 0x1DA, SMPS_1500),
	SMPS(S3, 0x1E0, 0x1E5, 0x00B, 0x1E2, SMPS_1500),
	SMPS(S4, 0x1E8, 0x1ED, 0x011, 0x1EA, SMPS_1500),

	/*     id  ctrl fts_cnfg1 pfm  pwr_cnfg  hpm_min */
	FTSMPS(S5, 0x025, 0x02E, 0x026, 0x032, SMPS_2000),
	FTSMPS(S6, 0x036, 0x03F, 0x037, 0x043, SMPS_2000),

	/*   id  ctrl   test2  clk    sleep  hpm_min */
	SMPS(S7, 0x1F0, 0x1F5, 0x012, 0x1F2, SMPS_1500),
	SMPS(S8, 0x1F8, 0x1FD, 0x013, 0x1FA, SMPS_1500),

	/* id		ctrl */
	VS(LVS1,	0x060),
	VS300(LVS2,     0x062),
	VS(LVS3,	0x064),
	VS(LVS4,	0x066),
	VS(LVS5,	0x068),
	VS(LVS6,	0x06A),
	VS(LVS7,	0x06C),
	VS300(USB_OTG,  0x06E),
	VS300(HDMI_MVS, 0x070),

	/*  id   ctrl */
	NCP(NCP, 0x090),
};

/*
 * Perform a masked write to a PMIC register only if the new value differs
 * from the last value written to the register.  This removes redundant
 * register writing.
 */
static int pm8921_vreg_masked_write(struct pm8921_vreg *vreg, u16 addr, u8 val,
		u8 mask, u8 *reg_save)
{
	int rc = 0;
	u8 reg;

	reg = (*reg_save & ~mask) | (val & mask);
	if (reg != *reg_save)
		rc = pm8xxx_writeb(vreg->dev->parent, addr, reg);

	if (rc)
		pr_err("pm8xxx_writeb failed; addr=0x%03X, rc=%d\n", addr, rc);
	else
		*reg_save = reg;

	return rc;
}

/*
 * Perform a masked write to a PMIC register without checking the previously
 * written value.  This is needed for registers that must be rewritten even if
 * the value hasn't changed in order for changes in other registers to take
 * effect.
 */
static int pm8921_vreg_masked_write_forced(struct pm8921_vreg *vreg, u16 addr,
		u8 val, u8 mask, u8 *reg_save)
{
	int rc = 0;
	u8 reg;

	reg = (*reg_save & ~mask) | (val & mask);
	rc = pm8xxx_writeb(vreg->dev->parent, addr, reg);

	if (rc)
		pr_err("pm8xxx_writeb failed; addr=0x%03X, rc=%d\n", addr, rc);
	else
		*reg_save = reg;

	return rc;
}

static int pm8921_vreg_is_pin_controlled(struct pm8921_vreg *vreg)
{
	int ret = 0;

	switch (vreg->type) {
	case REGULATOR_TYPE_LDO:
		ret = ((vreg->test_reg[5] & LDO_TEST_PIN_CTRL_MASK) << 4)
			| (vreg->test_reg[6] & LDO_TEST_PIN_CTRL_LPM_MASK);
		break;
	case REGULATOR_TYPE_SMPS:
		ret = vreg->sleep_ctrl_reg
			& (SMPS_PIN_CTRL_MASK | SMPS_PIN_CTRL_LPM_MASK);
		break;
	case REGULATOR_TYPE_VS:
		ret = vreg->ctrl_reg & VS_PIN_CTRL_MASK;
		break;
	}

	return ret;
}

static int _pm8921_vreg_is_enabled(struct pm8921_vreg *vreg)
{
	int rc = 0;

	/*
	 * All regulator types except advanced mode SMPS, FTSMPS, and VS300 have
	 * enable bit in bit 7 of the control register.  Pin control also does
	 * not work for advanced mode SMPS.
	 */
	switch (vreg->type) {
	case REGULATOR_TYPE_FTSMPS:
		if ((vreg->ctrl_reg & FTSMPS_VCTRL_BAND_MASK)
		    != FTSMPS_VCTRL_BAND_OFF)
			rc = 1;
		break;
	case REGULATOR_TYPE_VS300:
		if ((vreg->ctrl_reg & VS300_CTRL_ENABLE_MASK)
		    != VS300_CTRL_DISABLE)
			rc = 1;
		break;
	case REGULATOR_TYPE_SMPS:
		if (SMPS_IN_ADVANCED_MODE(vreg)) {
			if ((vreg->ctrl_reg & SMPS_ADVANCED_BAND_MASK)
			    != SMPS_ADVANCED_BAND_OFF)
				rc = 1;
		} else if ((vreg->ctrl_reg & SMPS_LEGACY_ENABLE)
			   || pm8921_vreg_is_pin_controlled(vreg)) {
			rc = 1;
		}
		break;
	default:
		if (((vreg->ctrl_reg & REGULATOR_ENABLE_MASK)
			== REGULATOR_ENABLE)
		    || pm8921_vreg_is_pin_controlled(vreg))
			rc = 1;
	}

	return rc;
}

static int pm8921_vreg_is_enabled(struct regulator_dev *rdev)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);

	return _pm8921_vreg_is_enabled(vreg);
}

static int pm8921_pldo_get_voltage(struct regulator_dev *rdev)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);
	int vmin, fine_step;
	u8 range_ext, range_sel, vprog, fine_step_reg;

	fine_step_reg = vreg->test_reg[2] & LDO_TEST_FINE_STEP_MASK;
	range_sel = vreg->test_reg[2] & LDO_TEST_RANGE_SEL_MASK;
	range_ext = vreg->test_reg[4] & LDO_TEST_RANGE_EXT_MASK;
	vprog = vreg->ctrl_reg & LDO_CTRL_VPROG_MASK;

	vprog = (vprog << 1) | (fine_step_reg >> LDO_TEST_FINE_STEP_SHIFT);

	if (range_sel) {
		/* low range mode */
		fine_step = PLDO_LOW_FINE_STEP_UV;
		vmin = PLDO_LOW_UV_MIN;
	} else if (!range_ext) {
		/* normal mode */
		fine_step = PLDO_NORM_FINE_STEP_UV;
		vmin = PLDO_NORM_UV_MIN;
	} else {
		/* high range mode */
		fine_step = PLDO_HIGH_FINE_STEP_UV;
		vmin = PLDO_HIGH_UV_MIN;
	}

	return fine_step * vprog + vmin;
}

static int pm8921_pldo_set_voltage(struct regulator_dev *rdev, int min_uV,
				   int max_uV)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);
	int rc = 0;
	int vmin;
	unsigned vprog, fine_step;
	u8 range_ext, range_sel, fine_step_reg, prev_reg;
	bool reg_changed = false;

	if (min_uV < PLDO_LOW_UV_MIN || min_uV > PLDO_HIGH_UV_MAX) {
		vreg_err(vreg, "request voltage %d is outside allowed range.\n",
			 min_uV);
		return -EINVAL;
	}

	if (min_uV < PLDO_NORM_UV_MIN) {
		vmin = PLDO_LOW_UV_MIN;
		fine_step = PLDO_LOW_FINE_STEP_UV;
		range_ext = 0;
		range_sel = LDO_TEST_RANGE_SEL_MASK;
	} else if (min_uV < PLDO_NORM_UV_MAX + PLDO_NORM_FINE_STEP_UV) {
		vmin = PLDO_NORM_UV_MIN;
		fine_step = PLDO_NORM_FINE_STEP_UV;
		range_ext = 0;
		range_sel = 0;
	} else {
		vmin = PLDO_HIGH_UV_MIN;
		fine_step = PLDO_HIGH_FINE_STEP_UV;
		range_ext = LDO_TEST_RANGE_EXT_MASK;
		range_sel = 0;
	}

	vprog = (min_uV - vmin) / fine_step;
	fine_step_reg = (vprog & 1) << LDO_TEST_FINE_STEP_SHIFT;
	vprog >>= 1;


	/* Write fine step, range select and program voltage update. */
	prev_reg = vreg->test_reg[2];
	rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
			fine_step_reg | range_sel | REGULATOR_BANK_SEL(2)
			 | REGULATOR_BANK_WRITE | LDO_TEST_VPROG_UPDATE_MASK,
			LDO_TEST_FINE_STEP_MASK | LDO_TEST_RANGE_SEL_MASK
			 | REGULATOR_BANK_MASK | LDO_TEST_VPROG_UPDATE_MASK,
			&vreg->test_reg[2]);
	if (rc)
		goto bail;
	if (prev_reg != vreg->test_reg[2])
		reg_changed = true;

	/* Write range extension. */
	prev_reg = vreg->test_reg[4];
	rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
			range_ext | REGULATOR_BANK_SEL(4)
			 | REGULATOR_BANK_WRITE,
			LDO_TEST_RANGE_EXT_MASK | REGULATOR_BANK_MASK,
			&vreg->test_reg[4]);
	if (rc)
		goto bail;
	if (prev_reg != vreg->test_reg[4])
		reg_changed = true;

	/* Write new voltage. */
	if (reg_changed) {
		/*
		 * Force a CTRL register write even if the value hasn't changed.
		 * This is neccessary because range select, range extension, and
		 * fine step will not update until a value is written into the
		 * control register.
		 */
		rc = pm8921_vreg_masked_write_forced(vreg, vreg->ctrl_addr,
			vprog, LDO_CTRL_VPROG_MASK, &vreg->ctrl_reg);
	} else {
		/* Only write to control register if new value is different. */
		rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr, vprog,
			LDO_CTRL_VPROG_MASK, &vreg->ctrl_reg);
	}
bail:
	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_nldo_get_voltage(struct regulator_dev *rdev)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);
	u8 vprog, fine_step_reg;

	fine_step_reg = vreg->test_reg[2] & LDO_TEST_FINE_STEP_MASK;
	vprog = vreg->ctrl_reg & LDO_CTRL_VPROG_MASK;

	vprog = (vprog << 1) | (fine_step_reg >> LDO_TEST_FINE_STEP_SHIFT);

	return NLDO_FINE_STEP_UV * vprog + NLDO_UV_MIN;
}

static int pm8921_nldo_set_voltage(struct regulator_dev *rdev, int min_uV,
				   int max_uV)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);
	unsigned vprog, fine_step_reg, prev_reg;
	int rc;

	if (min_uV < NLDO_UV_MIN || min_uV > NLDO_UV_MAX) {
		vreg_err(vreg, "request voltage %d is outside allowed range.\n",
			 min_uV);
		return -EINVAL;
	}

	vprog = (min_uV - NLDO_UV_MIN) / NLDO_FINE_STEP_UV;
	fine_step_reg = (vprog & 1) << LDO_TEST_FINE_STEP_SHIFT;
	vprog >>= 1;

	/* Write fine step. */
	prev_reg = vreg->test_reg[2];
	rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
			fine_step_reg | REGULATOR_BANK_SEL(2)
			 | REGULATOR_BANK_WRITE | LDO_TEST_VPROG_UPDATE_MASK,
			LDO_TEST_FINE_STEP_MASK | REGULATOR_BANK_MASK
			 | LDO_TEST_VPROG_UPDATE_MASK,
		       &vreg->test_reg[2]);
	if (rc)
		goto bail;

	/* Write new voltage. */
	if (prev_reg != vreg->test_reg[2]) {
		/*
		 * Force a CTRL register write even if the value hasn't changed.
		 * This is neccessary because fine step will not update until a
		 * value is written into the control register.
		 */
		rc = pm8921_vreg_masked_write_forced(vreg, vreg->ctrl_addr,
			vprog, LDO_CTRL_VPROG_MASK, &vreg->ctrl_reg);
	} else {
		/* Only write to control register if new value is different. */
		rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr, vprog,
			LDO_CTRL_VPROG_MASK, &vreg->ctrl_reg);
	}
bail:
	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int _pm8921_nldo1200_get_voltage(struct pm8921_vreg *vreg)
{
	int uV = 0;
	int vprog;

	if (!NLDO1200_IN_ADVANCED_MODE(vreg)) {
		pr_warn("%s: currently in legacy mode; voltage unknown.\n",
			vreg->name);
		return vreg->save_uV;
	}

	vprog = vreg->ctrl_reg & NLDO1200_CTRL_VPROG_MASK;

	if ((vreg->ctrl_reg & NLDO1200_CTRL_RANGE_MASK)
	    == NLDO1200_CTRL_RANGE_LOW)
		uV = vprog * NLDO1200_LOW_UV_STEP + NLDO1200_LOW_UV_MIN;
	else
		uV = vprog * NLDO1200_HIGH_UV_STEP + NLDO1200_HIGH_UV_MIN;

	return uV;
}

static int pm8921_nldo1200_get_voltage(struct regulator_dev *rdev)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);

	return _pm8921_nldo1200_get_voltage(vreg);
}

static int _pm8921_nldo1200_set_voltage(struct pm8921_vreg *vreg, int uV)
{
	u8 vprog, range;
	int rc;

	if (uV < NLDO1200_LOW_UV_MIN || uV > NLDO1200_HIGH_UV_MAX) {
		vreg_err(vreg, "request voltage %d is outside allowed range.\n",
			 uV);
		return -EINVAL;
	}

	if (uV < NLDO1200_HIGH_UV_MIN) {
		vprog = ((uV - NLDO1200_LOW_UV_MIN) / NLDO1200_LOW_UV_STEP);
		vprog &= NLDO1200_CTRL_VPROG_MASK;
		range = NLDO1200_CTRL_RANGE_LOW;
	} else {
		vprog = ((uV - NLDO1200_HIGH_UV_MIN) / NLDO1200_HIGH_UV_STEP);
		vprog &= NLDO1200_CTRL_VPROG_MASK;
		range = NLDO1200_CTRL_RANGE_HIGH;
	}

	/* Set to advanced mode */
	rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
		NLDO1200_ADVANCED_MODE, NLDO1200_ADVANCED_MODE_MASK,
		&vreg->test_reg[2]);
	if (rc)
		goto bail;

	/* Set voltage and range selection. */
	rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr, vprog | range,
			NLDO1200_CTRL_VPROG_MASK | NLDO1200_CTRL_RANGE_MASK,
			&vreg->ctrl_reg);
	if (rc)
		goto bail;

bail:
	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_nldo1200_set_voltage(struct regulator_dev *rdev, int min_uV,
				   int max_uV)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);

	return _pm8921_nldo1200_set_voltage(vreg, min_uV);
}

static int pm8921_smps_get_voltage_advanced(struct pm8921_vreg *vreg)
{
	u8 vprog, band;
	int uV = 0;

	vprog = vreg->ctrl_reg & SMPS_ADVANCED_VPROG_MASK;
	band = vreg->ctrl_reg & SMPS_ADVANCED_BAND_MASK;

	if (band == SMPS_ADVANCED_BAND_1)
		uV = vprog * SMPS_BAND1_UV_STEP + SMPS_BAND1_UV_MIN;
	else if (band == SMPS_ADVANCED_BAND_2)
		uV = vprog * SMPS_BAND2_UV_STEP + SMPS_BAND2_UV_MIN;
	else if (band == SMPS_ADVANCED_BAND_3)
		uV = vprog * SMPS_BAND3_UV_STEP + SMPS_BAND3_UV_MIN;
	else
		uV = vreg->save_uV;

	return uV;
}

static int pm8921_smps_get_voltage_legacy(struct pm8921_vreg *vreg)
{
	u8 vlow, vref, vprog;
	int uV;

	vlow = vreg->test_reg[1] & SMPS_LEGACY_VLOW_SEL_MASK;
	vref = vreg->ctrl_reg & SMPS_LEGACY_VREF_SEL_MASK;
	vprog = vreg->ctrl_reg & SMPS_LEGACY_VPROG_MASK;

	if (vlow && vref) {
		/* mode 3 */
		uV = vprog * SMPS_MODE3_UV_STEP + SMPS_MODE3_UV_MIN;
	} else if (vref) {
		/* mode 2 */
		uV = vprog * SMPS_MODE2_UV_STEP + SMPS_MODE2_UV_MIN;
	} else {
		/* mode 1 */
		uV = vprog * SMPS_MODE1_UV_STEP + SMPS_MODE1_UV_MIN;
	}

	return uV;
}

static int _pm8921_smps_get_voltage(struct pm8921_vreg *vreg)
{
	if (SMPS_IN_ADVANCED_MODE(vreg))
		return pm8921_smps_get_voltage_advanced(vreg);

	return pm8921_smps_get_voltage_legacy(vreg);
}

static int pm8921_smps_get_voltage(struct regulator_dev *rdev)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);

	return _pm8921_smps_get_voltage(vreg);
}

static int pm8921_smps_set_voltage_advanced(struct pm8921_vreg *vreg, int uV,
					    int force_on)
{
	u8 vprog, band;
	int rc, new_uV;

	if (uV < SMPS_BAND1_UV_MIN || uV > SMPS_BAND3_UV_MAX) {
		vreg_err(vreg, "request voltage %d is outside allowed range.\n",
			 uV);
		return -EINVAL;
	}

	if (uV < SMPS_BAND2_UV_MIN) {
		vprog = ((uV - SMPS_BAND1_UV_MIN) / SMPS_BAND1_UV_STEP);
		band = SMPS_ADVANCED_BAND_1;
		new_uV = SMPS_BAND1_UV_MIN + vprog * SMPS_BAND1_UV_STEP;
	} else if (uV < SMPS_BAND3_UV_MIN) {
		vprog = ((uV - SMPS_BAND2_UV_MIN) / SMPS_BAND2_UV_STEP);
		band = SMPS_ADVANCED_BAND_2;
		new_uV = SMPS_BAND2_UV_MIN + vprog * SMPS_BAND2_UV_STEP;
	} else {
		vprog = ((uV - SMPS_BAND3_UV_MIN) / SMPS_BAND3_UV_STEP);
		band = SMPS_ADVANCED_BAND_3;
		new_uV = SMPS_BAND3_UV_MIN + vprog * SMPS_BAND3_UV_STEP;
	}

	/* Do not set band if regulator currently disabled. */
	if (!_pm8921_vreg_is_enabled(vreg) && !force_on)
		band = SMPS_ADVANCED_BAND_OFF;

	/* Set advanced mode bit to 1. */
	rc = pm8921_vreg_masked_write(vreg, vreg->test_addr, SMPS_ADVANCED_MODE
		| REGULATOR_BANK_WRITE | REGULATOR_BANK_SEL(7),
		SMPS_ADVANCED_MODE_MASK | REGULATOR_BANK_MASK,
		&vreg->test_reg[7]);
	if (rc)
		goto bail;

	/* Set voltage and voltage band. */
	rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr, band | vprog,
			SMPS_ADVANCED_BAND_MASK | SMPS_ADVANCED_VPROG_MASK,
			&vreg->ctrl_reg);
	if (rc)
		goto bail;

	vreg->save_uV = new_uV;

bail:
	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_smps_set_voltage_legacy(struct pm8921_vreg *vreg, int uV)
{
	u8 vlow, vref, vprog, pd, en;
	int rc;

	if (uV < SMPS_MODE3_UV_MIN || uV > SMPS_MODE1_UV_MAX) {
		vreg_err(vreg, "request voltage %d is outside allowed range.\n",
			 uV);
		return -EINVAL;
	}

	if (uV < SMPS_MODE2_UV_MIN) {
		vprog = ((uV - SMPS_MODE3_UV_MIN) / SMPS_MODE3_UV_STEP);
		vref = SMPS_LEGACY_VREF_SEL_MASK;
		vlow = SMPS_LEGACY_VLOW_SEL_MASK;
	} else if (uV < SMPS_MODE1_UV_MIN) {
		vprog = ((uV - SMPS_MODE2_UV_MIN) / SMPS_MODE2_UV_STEP);
		vref = SMPS_LEGACY_VREF_SEL_MASK;
		vlow = 0;
	} else {
		vprog = ((uV - SMPS_MODE1_UV_MIN) / SMPS_MODE1_UV_STEP);
		vref = 0;
		vlow = 0;
	}

	/* set vlow bit for ultra low voltage mode */
	rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
		vlow | REGULATOR_BANK_WRITE | REGULATOR_BANK_SEL(1),
		REGULATOR_BANK_MASK | SMPS_LEGACY_VLOW_SEL_MASK,
		&vreg->test_reg[1]);
	if (rc)
		goto bail;

	/* Set advanced mode bit to 0. */
	rc = pm8921_vreg_masked_write(vreg, vreg->test_addr, SMPS_LEGACY_MODE
		| REGULATOR_BANK_WRITE | REGULATOR_BANK_SEL(7),
		SMPS_ADVANCED_MODE_MASK | REGULATOR_BANK_MASK,
		&vreg->test_reg[7]);
	if (rc)
		goto bail;

	en = (_pm8921_vreg_is_enabled(vreg) ? SMPS_LEGACY_ENABLE : 0);
	pd = (vreg->pdata.pull_down_enable ? SMPS_LEGACY_PULL_DOWN_ENABLE : 0);

	/* Set voltage (and the rest of the control register). */
	rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
		en | pd | vref | vprog,
		SMPS_LEGACY_ENABLE | SMPS_LEGACY_PULL_DOWN_ENABLE
		  | SMPS_LEGACY_VREF_SEL_MASK | SMPS_LEGACY_VPROG_MASK,
		&vreg->ctrl_reg);

	vreg->save_uV = pm8921_smps_get_voltage_legacy(vreg);

bail:
	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_smps_set_voltage(struct regulator_dev *rdev, int min_uV,
				   int max_uV)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);
	int rc = 0;

	if (SMPS_IN_ADVANCED_MODE(vreg) || !pm8921_vreg_is_pin_controlled(vreg))
		rc = pm8921_smps_set_voltage_advanced(vreg, min_uV, 0);
	else
		rc = pm8921_smps_set_voltage_legacy(vreg, min_uV);

	return rc;
}

static int _pm8921_ftsmps_get_voltage(struct pm8921_vreg *vreg)
{
	u8 vprog, band;
	int uV = 0;

	if ((vreg->test_reg[0] & FTSMPS_CNFG1_PM_MASK) == FTSMPS_CNFG1_PM_PFM) {
		vprog = vreg->pfm_ctrl_reg & FTSMPS_VCTRL_VPROG_MASK;
		band = vreg->pfm_ctrl_reg & FTSMPS_VCTRL_BAND_MASK;
		if (band == FTSMPS_VCTRL_BAND_OFF && vprog == 0) {
			/* PWM_VCTRL overrides PFM_VCTRL */
			vprog = vreg->ctrl_reg & FTSMPS_VCTRL_VPROG_MASK;
			band = vreg->ctrl_reg & FTSMPS_VCTRL_BAND_MASK;
		}
	} else {
		vprog = vreg->ctrl_reg & FTSMPS_VCTRL_VPROG_MASK;
		band = vreg->ctrl_reg & FTSMPS_VCTRL_BAND_MASK;
	}

	if (band == FTSMPS_VCTRL_BAND_1)
		uV = vprog * FTSMPS_BAND1_UV_STEP + FTSMPS_BAND1_UV_MIN;
	else if (band == FTSMPS_VCTRL_BAND_2)
		uV = vprog * FTSMPS_BAND2_UV_STEP + FTSMPS_BAND2_UV_MIN;
	else if (band == FTSMPS_VCTRL_BAND_3)
		uV = vprog * FTSMPS_BAND3_UV_STEP + FTSMPS_BAND3_UV_MIN;
	else
		uV = vreg->save_uV;

	return uV;
}

static int pm8921_ftsmps_get_voltage(struct regulator_dev *rdev)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);

	return _pm8921_ftsmps_get_voltage(vreg);
}

static int _pm8921_ftsmps_set_voltage(struct pm8921_vreg *vreg, int uV,
				      int force_on)
{
	int rc, new_uV;
	u8 vprog, band;

	if (uV < FTSMPS_BAND1_UV_MIN || uV > FTSMPS_BAND3_UV_MAX) {
		vreg_err(vreg, "request voltage %d is outside allowed range.\n",
			 uV);
		return -EINVAL;
	}

	/* Round down for set points in the gaps between bands. */
	if (uV > FTSMPS_BAND1_UV_MAX && uV < FTSMPS_BAND2_UV_MIN)
		uV = FTSMPS_BAND1_UV_MAX;
	else if (uV > FTSMPS_BAND2_UV_MAX
			&& uV < FTSMPS_BAND3_UV_SETPOINT_MIN)
		uV = FTSMPS_BAND2_UV_MAX;

	if (uV < FTSMPS_BAND2_UV_MIN) {
		vprog = ((uV - FTSMPS_BAND1_UV_MIN) / FTSMPS_BAND1_UV_STEP);
		vprog = PM8921_FTSMPS_BAND_1_COMPENSATE(vprog);
		band = FTSMPS_VCTRL_BAND_1;
		new_uV = FTSMPS_BAND1_UV_MIN + vprog * FTSMPS_BAND1_UV_STEP;
	} else if (uV < FTSMPS_BAND3_UV_SETPOINT_MIN) {
		vprog = ((uV - FTSMPS_BAND2_UV_MIN) / FTSMPS_BAND2_UV_STEP);
		band = FTSMPS_VCTRL_BAND_2;
		new_uV = FTSMPS_BAND2_UV_MIN + vprog * FTSMPS_BAND2_UV_STEP;
	} else {
		vprog = ((uV - FTSMPS_BAND3_UV_MIN) / FTSMPS_BAND3_UV_STEP);
		band = FTSMPS_VCTRL_BAND_3;
		new_uV = FTSMPS_BAND3_UV_MIN + vprog * FTSMPS_BAND3_UV_STEP;
	}

	/*
	 * Do not set voltage if regulator is currently disabled because doing
	 * so will enable it.
	 */
	if (_pm8921_vreg_is_enabled(vreg) || force_on) {
		rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
			band | vprog,
			FTSMPS_VCTRL_BAND_MASK | FTSMPS_VCTRL_VPROG_MASK,
			&vreg->ctrl_reg);
		if (rc)
			goto bail;

		/* Program PFM_VCTRL as 0x00 so that PWM_VCTRL overrides it. */
		rc = pm8921_vreg_masked_write(vreg, vreg->pfm_ctrl_addr, 0x00,
			FTSMPS_VCTRL_BAND_MASK | FTSMPS_VCTRL_VPROG_MASK,
			&vreg->pfm_ctrl_reg);
		if (rc)
			goto bail;
	}

	vreg->save_uV = new_uV;

bail:
	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_ftsmps_set_voltage(struct regulator_dev *rdev, int min_uV,
				     int max_uV)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);

	return _pm8921_ftsmps_set_voltage(vreg, min_uV, 0);
}

static int pm8921_ncp_get_voltage(struct regulator_dev *rdev)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);
	u8 vprog;

	vprog = vreg->ctrl_reg & NCP_VPROG_MASK;

	return NCP_UV_MIN + vprog * NCP_UV_STEP;
}

static int pm8921_ncp_set_voltage(struct regulator_dev *rdev, int min_uV,
				  int max_uV)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);
	int rc;
	u8 val;

	if (min_uV < NCP_UV_MIN || min_uV > NCP_UV_MAX) {
		vreg_err(vreg, "request voltage %d is outside allowed range.\n",
			 min_uV);
		return -EINVAL;
	}

	val = (min_uV - NCP_UV_MIN) / NCP_UV_STEP;

	/* voltage setting */
	rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr, val,
			NCP_VPROG_MASK, &vreg->ctrl_reg);
	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_vreg_set_pin_ctrl(struct pm8921_vreg *vreg, int on)
{
	unsigned pc = vreg->pdata.pin_ctrl, pf = vreg->pdata.pin_fn;
	int rc = 0;
	int bank;
	u8 val = 0;
	u8 mask;

	switch (vreg->type) {
	case REGULATOR_TYPE_LDO:
		if (on) {
			if (pc & PM8921_VREG_PIN_CTRL_D0)
				val |= LDO_TEST_PIN_CTRL_EN0;
			if (pc & PM8921_VREG_PIN_CTRL_D1)
				val |= LDO_TEST_PIN_CTRL_EN1;
			if (pc & PM8921_VREG_PIN_CTRL_A0)
				val |= LDO_TEST_PIN_CTRL_EN2;
			if (pc & PM8921_VREG_PIN_CTRL_A1)
				val |= LDO_TEST_PIN_CTRL_EN3;

			bank = (pf == PM8921_VREG_PIN_FN_ENABLE ? 5 : 6);
			rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
				val | REGULATOR_BANK_SEL(bank)
				  | REGULATOR_BANK_WRITE,
				LDO_TEST_PIN_CTRL_MASK | REGULATOR_BANK_MASK,
				&vreg->test_reg[bank]);
			if (rc)
				goto bail;

			val = LDO_TEST_LPM_SEL_CTRL | REGULATOR_BANK_WRITE
				| REGULATOR_BANK_SEL(0);
			mask = LDO_TEST_LPM_MASK | REGULATOR_BANK_MASK;
			rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
					val, mask, &vreg->test_reg[0]);
			if (rc)
				goto bail;

			if (pf == PM8921_VREG_PIN_FN_ENABLE) {
				/* Pin control ON/OFF */
				rc = pm8921_vreg_masked_write(vreg,
					vreg->ctrl_addr,
					LDO_CTRL_PM_HPM,
					LDO_ENABLE_MASK | LDO_CTRL_PM_MASK,
					&vreg->ctrl_reg);
				if (rc)
					goto bail;
			} else {
				/* Pin control LPM/HPM */
				rc = pm8921_vreg_masked_write(vreg,
					vreg->ctrl_addr,
					LDO_ENABLE | LDO_CTRL_PM_LPM,
					LDO_ENABLE_MASK | LDO_CTRL_PM_MASK,
					&vreg->ctrl_reg);
				if (rc)
					goto bail;
			}
		} else {
			/* Pin control off */
			rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
				REGULATOR_BANK_SEL(5) | REGULATOR_BANK_WRITE,
				LDO_TEST_PIN_CTRL_MASK | REGULATOR_BANK_MASK,
				&vreg->test_reg[5]);
			if (rc)
				goto bail;

			rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
				REGULATOR_BANK_SEL(6) | REGULATOR_BANK_WRITE,
				LDO_TEST_PIN_CTRL_MASK | REGULATOR_BANK_MASK,
				&vreg->test_reg[6]);
			if (rc)
				goto bail;
		}
		break;

	case REGULATOR_TYPE_SMPS:
		if (on) {
			if (pf == PM8921_VREG_PIN_FN_ENABLE) {
				/* Pin control ON/OFF */
				if (pc & PM8921_VREG_PIN_CTRL_D0)
					val |= SMPS_PIN_CTRL_D0;
				if (pc & PM8921_VREG_PIN_CTRL_D1)
					val |= SMPS_PIN_CTRL_D1;
				if (pc & PM8921_VREG_PIN_CTRL_A0)
					val |= SMPS_PIN_CTRL_A0;
				if (pc & PM8921_VREG_PIN_CTRL_A1)
					val |= SMPS_PIN_CTRL_A1;
			} else {
				/* Pin control LPM/HPM */
				if (pc & PM8921_VREG_PIN_CTRL_D0)
					val |= SMPS_PIN_CTRL_LPM_D0;
				if (pc & PM8921_VREG_PIN_CTRL_D1)
					val |= SMPS_PIN_CTRL_LPM_D1;
				if (pc & PM8921_VREG_PIN_CTRL_A0)
					val |= SMPS_PIN_CTRL_LPM_A0;
				if (pc & PM8921_VREG_PIN_CTRL_A1)
					val |= SMPS_PIN_CTRL_LPM_A1;
			}

			rc = pm8921_smps_set_voltage_legacy(vreg,
							    vreg->save_uV);
			if (rc)
				goto bail;

			rc = pm8921_vreg_masked_write(vreg,
				vreg->sleep_ctrl_addr, val,
				SMPS_PIN_CTRL_MASK | SMPS_PIN_CTRL_LPM_MASK,
				&vreg->sleep_ctrl_reg);
			if (rc)
				goto bail;

			rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
				(pf == PM8921_VREG_PIN_FN_ENABLE
				       ? 0 : SMPS_LEGACY_ENABLE),
				SMPS_LEGACY_ENABLE, &vreg->ctrl_reg);
			if (rc)
				goto bail;

			rc = pm8921_vreg_masked_write(vreg, vreg->clk_ctrl_addr,
				(pf == PM8921_VREG_PIN_FN_ENABLE
				       ? SMPS_CLK_CTRL_PWM : SMPS_CLK_CTRL_PFM),
				SMPS_CLK_CTRL_MASK, &vreg->clk_ctrl_reg);
			if (rc)
				goto bail;
		} else {
			/* Pin control off */
			if (!SMPS_IN_ADVANCED_MODE(vreg)) {
				if (_pm8921_vreg_is_enabled(vreg))
					val = SMPS_LEGACY_ENABLE;
				rc = pm8921_vreg_masked_write(vreg,
					vreg->ctrl_addr, val,
					SMPS_LEGACY_ENABLE, &vreg->ctrl_reg);
				if (rc)
					goto bail;
			}

			rc = pm8921_vreg_masked_write(vreg,
				vreg->sleep_ctrl_addr, 0,
				SMPS_PIN_CTRL_MASK | SMPS_PIN_CTRL_LPM_MASK,
				&vreg->sleep_ctrl_reg);
			if (rc)
				goto bail;

			rc = pm8921_smps_set_voltage_advanced(vreg,
							      vreg->save_uV, 0);
			if (rc)
				goto bail;
		}
		break;

	case REGULATOR_TYPE_VS:
		if (on) {
			if (pc & PM8921_VREG_PIN_CTRL_D0)
				val |= VS_PIN_CTRL_EN0;
			if (pc & PM8921_VREG_PIN_CTRL_D1)
				val |= VS_PIN_CTRL_EN1;
			if (pc & PM8921_VREG_PIN_CTRL_A0)
				val |= VS_PIN_CTRL_EN2;
			if (pc & PM8921_VREG_PIN_CTRL_A1)
				val |= VS_PIN_CTRL_EN3;

			rc = pm8921_vreg_masked_write(vreg,
					vreg->ctrl_addr, val,
					VS_PIN_CTRL_MASK | VS_ENABLE_MASK,
					&vreg->ctrl_reg);
			if (rc)
				goto bail;
		} else {
			/* Pin control off */
			if (_pm8921_vreg_is_enabled(vreg))
				val = VS_ENABLE;

			rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
					val, VS_ENABLE_MASK | VS_PIN_CTRL_MASK,
					&vreg->ctrl_reg);
			if (rc)
				goto bail;
		}
		break;
	}

bail:
	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static unsigned int pm8921_vreg_get_mode(struct regulator_dev *rdev)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);

	/* Check physical pin control state. */
	switch (vreg->type) {
	case REGULATOR_TYPE_LDO:
		if (!(vreg->ctrl_reg & LDO_ENABLE_MASK)
		    && (vreg->test_reg[5] & LDO_TEST_PIN_CTRL_MASK))
			return REGULATOR_MODE_IDLE;
		else if ((vreg->ctrl_reg & LDO_ENABLE_MASK)
		    && (vreg->ctrl_reg & LDO_CTRL_PM_MASK)
		    && (vreg->test_reg[6] & LDO_TEST_PIN_CTRL_LPM_MASK))
			return REGULATOR_MODE_IDLE;
		break;
	case REGULATOR_TYPE_SMPS:
		if (!SMPS_IN_ADVANCED_MODE(vreg)
		    && !(vreg->ctrl_reg & SMPS_LEGACY_ENABLE)
		    && (vreg->sleep_ctrl_reg & SMPS_PIN_CTRL_MASK))
			return REGULATOR_MODE_IDLE;
		else if (!SMPS_IN_ADVANCED_MODE(vreg)
		    && (vreg->ctrl_reg & SMPS_LEGACY_ENABLE)
		    && ((vreg->clk_ctrl_reg & SMPS_CLK_CTRL_MASK)
			== SMPS_CLK_CTRL_PFM)
		    && (vreg->sleep_ctrl_reg & SMPS_PIN_CTRL_LPM_MASK))
			return REGULATOR_MODE_IDLE;
		break;
	case REGULATOR_TYPE_VS:
		if (!(vreg->ctrl_reg & VS_ENABLE_MASK)
		    && (vreg->ctrl_reg & VS_PIN_CTRL_MASK))
			return REGULATOR_MODE_IDLE;
	}

	if (vreg->optimum == REGULATOR_MODE_FAST)
		return REGULATOR_MODE_FAST;
	else if (vreg->pc_vote)
		return REGULATOR_MODE_IDLE;
	else if (vreg->optimum == REGULATOR_MODE_STANDBY)
		return REGULATOR_MODE_STANDBY;
	return REGULATOR_MODE_FAST;
}

static int pm8921_ldo_set_mode(struct pm8921_vreg *vreg, unsigned int mode)
{
	int rc = 0;
	u8 mask, val;

	switch (mode) {
	case REGULATOR_MODE_FAST:
		/* HPM */
		val = (_pm8921_vreg_is_enabled(vreg) ? LDO_ENABLE : 0)
			| LDO_CTRL_PM_HPM;
		mask = LDO_ENABLE_MASK | LDO_CTRL_PM_MASK;
		rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr, val, mask,
					&vreg->ctrl_reg);
		if (rc)
			goto bail;

		if (pm8921_vreg_is_pin_controlled(vreg))
			rc = pm8921_vreg_set_pin_ctrl(vreg, 0);
		if (rc)
			goto bail;
		break;

	case REGULATOR_MODE_STANDBY:
		/* LPM */
		val = (_pm8921_vreg_is_enabled(vreg) ? LDO_ENABLE : 0)
			| LDO_CTRL_PM_LPM;
		mask = LDO_ENABLE_MASK | LDO_CTRL_PM_MASK;
		rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr, val, mask,
					&vreg->ctrl_reg);
		if (rc)
			goto bail;

		val = LDO_TEST_LPM_SEL_CTRL | REGULATOR_BANK_WRITE
			| REGULATOR_BANK_SEL(0);
		mask = LDO_TEST_LPM_MASK | REGULATOR_BANK_MASK;
		rc = pm8921_vreg_masked_write(vreg, vreg->test_addr, val, mask,
					&vreg->test_reg[0]);
		if (rc)
			goto bail;

		if (pm8921_vreg_is_pin_controlled(vreg))
			rc = pm8921_vreg_set_pin_ctrl(vreg, 0);
		if (rc)
			goto bail;
		break;

	case REGULATOR_MODE_IDLE:
		/* Pin Control */
		if (_pm8921_vreg_is_enabled(vreg))
			rc = pm8921_vreg_set_pin_ctrl(vreg, 1);
		if (rc)
			goto bail;
		break;

	default:
		vreg_err(vreg, "invalid mode: %u\n", mode);
		return -EINVAL;
	}

bail:
	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_nldo1200_set_mode(struct pm8921_vreg *vreg, unsigned int mode)
{
	int rc = 0;

	if (mode != REGULATOR_MODE_FAST && mode != REGULATOR_MODE_STANDBY) {
		vreg_err(vreg, "invalid mode: %u\n", mode);
		return -EINVAL;
	}

	/*
	 * Make sure that advanced mode is in use.  If it isn't, then set it
	 * and update the voltage accordingly.
	 */
	if (!NLDO1200_IN_ADVANCED_MODE(vreg)) {
		rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
			NLDO1200_ADVANCED_MODE, NLDO1200_ADVANCED_MODE_MASK,
			&vreg->test_reg[2]);
		if (rc)
			goto bail;

		_pm8921_nldo1200_set_voltage(vreg, vreg->save_uV);
	}

	if (mode == REGULATOR_MODE_FAST) {
		/* HPM */
		rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
			NLDO1200_ADVANCED_PM_HPM, NLDO1200_ADVANCED_PM_MASK,
			&vreg->test_reg[2]);
	} else {
		/* LPM */
		rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
			NLDO1200_ADVANCED_PM_LPM, NLDO1200_ADVANCED_PM_MASK,
			&vreg->test_reg[2]);
	}

bail:
	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_smps_set_mode(struct pm8921_vreg *vreg, unsigned int mode)
{
	int rc = 0;

	switch (mode) {
	case REGULATOR_MODE_FAST:
		/* HPM */
		rc = pm8921_vreg_masked_write(vreg, vreg->clk_ctrl_addr,
				       SMPS_CLK_CTRL_PWM, SMPS_CLK_CTRL_MASK,
				       &vreg->clk_ctrl_reg);
		if (rc)
			goto bail;

		if (pm8921_vreg_is_pin_controlled(vreg))
			rc = pm8921_vreg_set_pin_ctrl(vreg, 0);
		if (rc)
			goto bail;
		break;

	case REGULATOR_MODE_STANDBY:
		/* LPM */
		rc = pm8921_vreg_masked_write(vreg, vreg->clk_ctrl_addr,
				       SMPS_CLK_CTRL_PFM, SMPS_CLK_CTRL_MASK,
				       &vreg->clk_ctrl_reg);
		if (rc)
			goto bail;

		if (pm8921_vreg_is_pin_controlled(vreg))
			rc = pm8921_vreg_set_pin_ctrl(vreg, 0);
		if (rc)
			goto bail;
		break;

	case REGULATOR_MODE_IDLE:
		/* Pin Control */
		if (_pm8921_vreg_is_enabled(vreg))
			rc = pm8921_vreg_set_pin_ctrl(vreg, 1);
		if (rc)
			goto bail;
		break;

	default:
		vreg_err(vreg, "invalid mode: %u\n", mode);
		return -EINVAL;
	}

bail:
	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_ftsmps_set_mode(struct pm8921_vreg *vreg, unsigned int mode)
{
	int rc = 0;

	if (mode == REGULATOR_MODE_FAST) {
		/* HPM */
		rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
				FTSMPS_CNFG1_PM_PWM, FTSMPS_CNFG1_PM_MASK,
				&vreg->test_reg[0]);
	} else if (mode == REGULATOR_MODE_STANDBY) {
		/* LPM */
		rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
				FTSMPS_CNFG1_PM_PFM, FTSMPS_CNFG1_PM_MASK,
				&vreg->test_reg[0]);
	} else {
		vreg_err(vreg, "invalid mode: %u\n", mode);
		return -EINVAL;
	}

	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_lvs_set_mode(struct pm8921_vreg *vreg, unsigned int mode)
{
	int rc = 0;

	if (mode == REGULATOR_MODE_IDLE) {
		/* Use pin control. */
		if (_pm8921_vreg_is_enabled(vreg))
			rc = pm8921_vreg_set_pin_ctrl(vreg, 1);
	} else {
		/* Turn off pin control. */
		rc = pm8921_vreg_set_pin_ctrl(vreg, 0);
	}

	return rc;
}

/*
 * Optimum mode programming:
 * REGULATOR_MODE_FAST: Go to HPM (highest priority)
 * REGULATOR_MODE_STANDBY: Go to pin ctrl mode if there are any pin ctrl
 * votes, else go to LPM
 *
 * Pin ctrl mode voting via regulator set_mode:
 * REGULATOR_MODE_IDLE: Go to pin ctrl mode if the optimum mode is LPM, else
 * go to HPM
 * REGULATOR_MODE_NORMAL: Go to LPM if it is the optimum mode, else go to HPM
 */
static int pm8921_vreg_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);
	unsigned prev_optimum = vreg->optimum, prev_pc_vote = vreg->pc_vote;
	int rc = 0, new_mode = REGULATOR_MODE_FAST;

	/* Determine new mode to go into. */
	switch (mode) {
	case REGULATOR_MODE_FAST:
		new_mode = REGULATOR_MODE_FAST;
		vreg->optimum = mode;
		break;

	case REGULATOR_MODE_STANDBY:
		if (vreg->pc_vote)
			new_mode = REGULATOR_MODE_IDLE;
		else
			new_mode = REGULATOR_MODE_STANDBY;
		vreg->optimum = mode;
		break;

	case REGULATOR_MODE_IDLE:
		if (vreg->pc_vote++)
			return rc; /* already taken care of */

		if (vreg->optimum == REGULATOR_MODE_FAST)
			new_mode = REGULATOR_MODE_FAST;
		else
			new_mode = REGULATOR_MODE_IDLE;
		break;

	case REGULATOR_MODE_NORMAL:
		if (vreg->pc_vote && --(vreg->pc_vote))
			return rc; /* already taken care of */

		if (vreg->optimum == REGULATOR_MODE_STANDBY)
			new_mode = REGULATOR_MODE_STANDBY;
		else
			new_mode = REGULATOR_MODE_FAST;
		break;

	default:
		vreg_err(vreg, "unknown mode, mode=%u\n", mode);
		return -EINVAL;
	}

	switch (vreg->type) {
	case REGULATOR_TYPE_LDO:
		rc = pm8921_ldo_set_mode(vreg, new_mode);
		break;
	case REGULATOR_TYPE_NLDO1200:
		rc = pm8921_nldo1200_set_mode(vreg, new_mode);
		break;
	case REGULATOR_TYPE_SMPS:
		rc = pm8921_smps_set_mode(vreg, new_mode);
		break;
	case REGULATOR_TYPE_FTSMPS:
		rc = pm8921_ftsmps_set_mode(vreg, new_mode);
		break;
	case REGULATOR_TYPE_VS:
		rc = pm8921_lvs_set_mode(vreg, new_mode);
		break;
	}

	if (rc) {
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);
		vreg->optimum = prev_optimum;
		vreg->pc_vote = prev_pc_vote;
	}

	return rc;
}

static unsigned int pm8921_vreg_get_optimum_mode(struct regulator_dev *rdev,
		int input_uV, int output_uV, int load_uA)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);
	unsigned int mode;

	if (load_uA + vreg->pdata.system_uA >= vreg->hpm_min_load)
		mode = REGULATOR_MODE_FAST;
	else
		mode = REGULATOR_MODE_STANDBY;

	return mode;
}

static int pm8921_vreg_enable(struct regulator_dev *rdev)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);
	int rc = 0;
	int mode;

	mode = pm8921_vreg_get_mode(rdev);

	if (mode == REGULATOR_MODE_IDLE) {
		/* Turn on pin control. */
		rc = pm8921_vreg_set_pin_ctrl(vreg, 1);
		if (rc)
			goto bail;
		return rc;
	}
	/*
	 * All regulator types except advanced mode SMPS, FTSMPS, and VS300 have
	 * enable bit in bit 7 of the control register.
	 */
	switch (vreg->type) {
	case REGULATOR_TYPE_SMPS:
		if (SMPS_IN_ADVANCED_MODE(vreg)
		    || !pm8921_vreg_is_pin_controlled(vreg)) {
			/* Enable in advanced mode if not using pin control. */
			rc = pm8921_smps_set_voltage_advanced(vreg,
							vreg->save_uV, 1);
		} else {
			/* Leave in legacy mode because pin control is used. */
			rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
					SMPS_LEGACY_ENABLE, SMPS_LEGACY_ENABLE,
					&vreg->ctrl_reg);
		}
		break;
	case REGULATOR_TYPE_FTSMPS:
		rc = _pm8921_ftsmps_set_voltage(vreg, vreg->save_uV, 1);
		break;
	case REGULATOR_TYPE_VS300:
		rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
				VS300_CTRL_ENABLE, VS300_CTRL_ENABLE_MASK,
				&vreg->ctrl_reg);
		break;
	default:
		rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
				REGULATOR_ENABLE, REGULATOR_ENABLE_MASK,
				&vreg->ctrl_reg);
	}

bail:
	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_vreg_disable(struct regulator_dev *rdev)
{
	struct pm8921_vreg *vreg = rdev_get_drvdata(rdev);
	int rc = 0;

	/* Turn off pin control. */
	rc = pm8921_vreg_set_pin_ctrl(vreg, 0);
	if (rc)
		goto bail;

	/* Disable in control register. */
	switch (vreg->type) {
	case REGULATOR_TYPE_FTSMPS:
		rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
			FTSMPS_VCTRL_BAND_OFF, FTSMPS_VCTRL_BAND_MASK,
			&vreg->ctrl_reg);
		if (rc)
			goto bail;
		rc = pm8921_vreg_masked_write(vreg, vreg->pfm_ctrl_addr,
			FTSMPS_VCTRL_BAND_OFF, FTSMPS_VCTRL_BAND_MASK,
			&vreg->pfm_ctrl_reg);
		break;
	case REGULATOR_TYPE_VS300:
		rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
			VS300_CTRL_DISABLE, VS300_CTRL_ENABLE_MASK,
			&vreg->ctrl_reg);
		break;
	case REGULATOR_TYPE_SMPS:
		if (SMPS_IN_ADVANCED_MODE(vreg)) {
			/* Change SMPS to legacy mode before disabling. */
			rc = pm8921_smps_set_voltage_legacy(vreg,
				vreg->save_uV);
			if (rc)
				goto bail;
		}
		/* Fall through and disable SMPS via control bit 7. */
	default:
		rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
			REGULATOR_DISABLE, REGULATOR_ENABLE_MASK,
			&vreg->ctrl_reg);
	}

bail:
	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static struct regulator_ops pm8921_pldo_ops = {
	.enable			= pm8921_vreg_enable,
	.disable		= pm8921_vreg_disable,
	.is_enabled		= pm8921_vreg_is_enabled,
	.set_voltage		= pm8921_pldo_set_voltage,
	.get_voltage		= pm8921_pldo_get_voltage,
	.set_mode		= pm8921_vreg_set_mode,
	.get_mode		= pm8921_vreg_get_mode,
	.get_optimum_mode	= pm8921_vreg_get_optimum_mode,
};

static struct regulator_ops pm8921_nldo_ops = {
	.enable			= pm8921_vreg_enable,
	.disable		= pm8921_vreg_disable,
	.is_enabled		= pm8921_vreg_is_enabled,
	.set_voltage		= pm8921_nldo_set_voltage,
	.get_voltage		= pm8921_nldo_get_voltage,
	.set_mode		= pm8921_vreg_set_mode,
	.get_mode		= pm8921_vreg_get_mode,
	.get_optimum_mode	= pm8921_vreg_get_optimum_mode,
};

static struct regulator_ops pm8921_nldo1200_ops = {
	.enable			= pm8921_vreg_enable,
	.disable		= pm8921_vreg_disable,
	.is_enabled		= pm8921_vreg_is_enabled,
	.set_voltage		= pm8921_nldo1200_set_voltage,
	.get_voltage		= pm8921_nldo1200_get_voltage,
	.set_mode		= pm8921_vreg_set_mode,
	.get_mode		= pm8921_vreg_get_mode,
	.get_optimum_mode	= pm8921_vreg_get_optimum_mode,
};

static struct regulator_ops pm8921_smps_ops = {
	.enable			= pm8921_vreg_enable,
	.disable		= pm8921_vreg_disable,
	.is_enabled		= pm8921_vreg_is_enabled,
	.set_voltage		= pm8921_smps_set_voltage,
	.get_voltage		= pm8921_smps_get_voltage,
	.set_mode		= pm8921_vreg_set_mode,
	.get_mode		= pm8921_vreg_get_mode,
	.get_optimum_mode	= pm8921_vreg_get_optimum_mode,
};

static struct regulator_ops pm8921_ftsmps_ops = {
	.enable			= pm8921_vreg_enable,
	.disable		= pm8921_vreg_disable,
	.is_enabled		= pm8921_vreg_is_enabled,
	.set_voltage		= pm8921_ftsmps_set_voltage,
	.get_voltage		= pm8921_ftsmps_get_voltage,
	.set_mode		= pm8921_vreg_set_mode,
	.get_mode		= pm8921_vreg_get_mode,
	.get_optimum_mode	= pm8921_vreg_get_optimum_mode,
};

static struct regulator_ops pm8921_vs_ops = {
	.enable			= pm8921_vreg_enable,
	.disable		= pm8921_vreg_disable,
	.is_enabled		= pm8921_vreg_is_enabled,
	.set_mode		= pm8921_vreg_set_mode,
	.get_mode		= pm8921_vreg_get_mode,
};

static struct regulator_ops pm8921_vs300_ops = {
	.enable			= pm8921_vreg_enable,
	.disable		= pm8921_vreg_disable,
	.is_enabled		= pm8921_vreg_is_enabled,
};

static struct regulator_ops pm8921_ncp_ops = {
	.enable			= pm8921_vreg_enable,
	.disable		= pm8921_vreg_disable,
	.is_enabled		= pm8921_vreg_is_enabled,
	.set_voltage		= pm8921_ncp_set_voltage,
	.get_voltage		= pm8921_ncp_get_voltage,
};

#define VREG_DESC(_id, _name, _ops) \
	[PM8921_VREG_ID_##_id] = { \
		.id	= PM8921_VREG_ID_##_id, \
		.name	= _name, \
		.ops	= _ops, \
		.type	= REGULATOR_VOLTAGE, \
		.owner	= THIS_MODULE, \
	}

static struct regulator_desc pm8921_vreg_description[] = {
	VREG_DESC(L1,  "8921_l1",  &pm8921_nldo_ops),
	VREG_DESC(L2,  "8921_l2",  &pm8921_nldo_ops),
	VREG_DESC(L3,  "8921_l3",  &pm8921_pldo_ops),
	VREG_DESC(L4,  "8921_l4",  &pm8921_pldo_ops),
	VREG_DESC(L5,  "8921_l5",  &pm8921_pldo_ops),
	VREG_DESC(L6,  "8921_l6",  &pm8921_pldo_ops),
	VREG_DESC(L7,  "8921_l7",  &pm8921_pldo_ops),
	VREG_DESC(L8,  "8921_l8",  &pm8921_pldo_ops),
	VREG_DESC(L9,  "8921_l9",  &pm8921_pldo_ops),
	VREG_DESC(L10, "8921_l10", &pm8921_pldo_ops),
	VREG_DESC(L11, "8921_l11", &pm8921_pldo_ops),
	VREG_DESC(L12, "8921_l12", &pm8921_nldo_ops),
	VREG_DESC(L14, "8921_l14", &pm8921_pldo_ops),
	VREG_DESC(L15, "8921_l15", &pm8921_pldo_ops),
	VREG_DESC(L16, "8921_l16", &pm8921_pldo_ops),
	VREG_DESC(L17, "8921_l17", &pm8921_pldo_ops),
	VREG_DESC(L18, "8921_l18", &pm8921_nldo_ops),
	VREG_DESC(L21, "8921_l21", &pm8921_pldo_ops),
	VREG_DESC(L22, "8921_l22", &pm8921_pldo_ops),
	VREG_DESC(L23, "8921_l23", &pm8921_pldo_ops),
	VREG_DESC(L24, "8921_l24", &pm8921_nldo1200_ops),
	VREG_DESC(L25, "8921_l25", &pm8921_nldo1200_ops),
	VREG_DESC(L26, "8921_l26", &pm8921_nldo1200_ops),
	VREG_DESC(L27, "8921_l27", &pm8921_nldo1200_ops),
	VREG_DESC(L28, "8921_l28", &pm8921_nldo1200_ops),
	VREG_DESC(L29, "8921_l29", &pm8921_pldo_ops),

	VREG_DESC(S1, "8921_s1", &pm8921_smps_ops),
	VREG_DESC(S2, "8921_s2", &pm8921_smps_ops),
	VREG_DESC(S3, "8921_s3", &pm8921_smps_ops),
	VREG_DESC(S4, "8921_s4", &pm8921_smps_ops),
	VREG_DESC(S5, "8921_s5", &pm8921_ftsmps_ops),
	VREG_DESC(S6, "8921_s6", &pm8921_ftsmps_ops),
	VREG_DESC(S7, "8921_s7", &pm8921_smps_ops),
	VREG_DESC(S8, "8921_s8", &pm8921_smps_ops),

	VREG_DESC(LVS1, "8921_lvs1", &pm8921_vs_ops),
	VREG_DESC(LVS2, "8921_lvs2", &pm8921_vs300_ops),
	VREG_DESC(LVS3, "8921_lvs3", &pm8921_vs_ops),
	VREG_DESC(LVS4, "8921_lvs4", &pm8921_vs_ops),
	VREG_DESC(LVS5, "8921_lvs5", &pm8921_vs_ops),
	VREG_DESC(LVS6, "8921_lvs6", &pm8921_vs_ops),
	VREG_DESC(LVS7, "8921_lvs7", &pm8921_vs_ops),

	VREG_DESC(USB_OTG, "8921_usb_otg", &pm8921_vs300_ops),
	VREG_DESC(HDMI_MVS, "8921_hdmi_mvs", &pm8921_vs300_ops),
	VREG_DESC(NCP, "8921_ncp", &pm8921_ncp_ops),
};

static int pm8921_init_ldo(struct pm8921_vreg *vreg)
{
	int rc = 0;
	int i;
	u8 bank;

	/* Save the current test register state. */
	for (i = 0; i < LDO_TEST_BANKS; i++) {
		bank = REGULATOR_BANK_SEL(i);
		rc = pm8xxx_writeb(vreg->dev->parent, vreg->test_addr, bank);
		if (rc)
			goto bail;

		rc = pm8xxx_readb(vreg->dev->parent, vreg->test_addr,
				  &vreg->test_reg[i]);
		if (rc)
			goto bail;
		vreg->test_reg[i] |= REGULATOR_BANK_WRITE;
	}

	if ((vreg->ctrl_reg & LDO_CTRL_PM_MASK) == LDO_CTRL_PM_LPM)
		vreg->optimum = REGULATOR_MODE_STANDBY;
	else
		vreg->optimum = REGULATOR_MODE_FAST;

	/* Set pull down enable based on platform data. */
	rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
		     (vreg->pdata.pull_down_enable ? LDO_PULL_DOWN_ENABLE : 0),
		     LDO_PULL_DOWN_ENABLE_MASK, &vreg->ctrl_reg);
bail:
	if (rc)
		vreg_err(vreg, "pm8xxx_readb/writeb failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_init_nldo1200(struct pm8921_vreg *vreg)
{
	int rc = 0;
	int i;
	u8 bank;

	/* Save the current test register state. */
	for (i = 0; i < LDO_TEST_BANKS; i++) {
		bank = REGULATOR_BANK_SEL(i);
		rc = pm8xxx_writeb(vreg->dev->parent, vreg->test_addr, bank);
		if (rc)
			goto bail;

		rc = pm8xxx_readb(vreg->dev->parent, vreg->test_addr,
				  &vreg->test_reg[i]);
		if (rc)
			goto bail;
		vreg->test_reg[i] |= REGULATOR_BANK_WRITE;
	}

	/*
	 * get_voltage must return >0 in order for regulator_set_optimum_mode
	 * to succeed, but voltage may be unknown at probe time.
	 */
	vreg->save_uV = 1; /* This is not a no-op. */
	vreg->save_uV = _pm8921_nldo1200_get_voltage(vreg);

	if (NLDO1200_IN_ADVANCED_MODE(vreg)) {
		if ((vreg->test_reg[2] & NLDO1200_ADVANCED_PM_MASK)
		    == NLDO1200_ADVANCED_PM_LPM)
			vreg->optimum = REGULATOR_MODE_STANDBY;
		else
			vreg->optimum = REGULATOR_MODE_FAST;
	} else {
		if ((vreg->ctrl_reg & NLDO1200_LEGACY_PM_MASK)
		    == NLDO1200_LEGACY_PM_LPM)
			vreg->optimum = REGULATOR_MODE_STANDBY;
		else
			vreg->optimum = REGULATOR_MODE_FAST;
	}

	/* Set pull down enable based on platform data. */
	rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
		 (vreg->pdata.pull_down_enable ? NLDO1200_PULL_DOWN_ENABLE : 0),
		 NLDO1200_PULL_DOWN_ENABLE_MASK, &vreg->test_reg[1]);
bail:
	if (rc)
		vreg_err(vreg, "pm8xxx_readb/writeb failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_init_smps(struct pm8921_vreg *vreg)
{
	int rc = 0;
	int i;
	u8 bank;

	/* Save the current test2 register state. */
	for (i = 0; i < SMPS_TEST_BANKS; i++) {
		bank = REGULATOR_BANK_SEL(i);
		rc = pm8xxx_writeb(vreg->dev->parent, vreg->test_addr, bank);
		if (rc)
			goto bail;

		rc = pm8xxx_readb(vreg->dev->parent, vreg->test_addr,
				  &vreg->test_reg[i]);
		if (rc)
			goto bail;
		vreg->test_reg[i] |= REGULATOR_BANK_WRITE;
	}

	/* Save the current clock control register state. */
	rc = pm8xxx_readb(vreg->dev->parent, vreg->clk_ctrl_addr,
			  &vreg->clk_ctrl_reg);
	if (rc)
		goto bail;

	/* Save the current sleep control register state. */
	rc = pm8xxx_readb(vreg->dev->parent, vreg->sleep_ctrl_addr,
			  &vreg->sleep_ctrl_reg);
	if (rc)
		goto bail;

	/*
	 * get_voltage must return >0 in order for regulator_set_optimum_mode
	 * to succeed, but voltage may be unknown at probe time.
	 */
	vreg->save_uV = 1; /* This is not a no-op. */
	vreg->save_uV = _pm8921_smps_get_voltage(vreg);

	if ((vreg->clk_ctrl_reg & SMPS_CLK_CTRL_MASK) == SMPS_CLK_CTRL_PFM)
		vreg->optimum = REGULATOR_MODE_STANDBY;
	else
		vreg->optimum = REGULATOR_MODE_FAST;

	/* Set advanced mode pull down enable based on platform data. */
	rc = pm8921_vreg_masked_write(vreg, vreg->test_addr,
		(vreg->pdata.pull_down_enable
			? SMPS_ADVANCED_PULL_DOWN_ENABLE : 0)
		| REGULATOR_BANK_SEL(6) | REGULATOR_BANK_WRITE,
		REGULATOR_BANK_MASK | SMPS_ADVANCED_PULL_DOWN_ENABLE,
		&vreg->test_reg[6]);
	if (rc)
		goto bail;

	if (!SMPS_IN_ADVANCED_MODE(vreg)) {
		/* Set legacy mode pull down enable based on platform data. */
		rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
			(vreg->pdata.pull_down_enable
				? SMPS_LEGACY_PULL_DOWN_ENABLE : 0),
			SMPS_LEGACY_PULL_DOWN_ENABLE, &vreg->ctrl_reg);
		if (rc)
			goto bail;
	}

bail:
	if (rc)
		vreg_err(vreg, "pm8xxx_readb/writeb failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_init_ftsmps(struct pm8921_vreg *vreg)
{
	int rc, i;
	u8 bank;

	/* Store current regulator register values. */
	rc = pm8xxx_readb(vreg->dev->parent, vreg->pfm_ctrl_addr,
			  &vreg->pfm_ctrl_reg);
	if (rc)
		goto bail;

	rc = pm8xxx_readb(vreg->dev->parent, vreg->pwr_cnfg_addr,
			  &vreg->pwr_cnfg_reg);
	if (rc)
		goto bail;

	/* Save the current fts_cnfg1 register state (uses 'test' member). */
	for (i = 0; i < SMPS_TEST_BANKS; i++) {
		bank = REGULATOR_BANK_SEL(i);
		rc = pm8xxx_writeb(vreg->dev->parent, vreg->test_addr, bank);
		if (rc)
			goto bail;

		rc = pm8xxx_readb(vreg->dev->parent, vreg->test_addr,
				  &vreg->test_reg[i]);
		if (rc)
			goto bail;
		vreg->test_reg[i] |= REGULATOR_BANK_WRITE;
	}

	/*
	 * get_voltage must return >0 in order for regulator_set_optimum_mode
	 * to succeed, but voltage may be unknown at probe time.
	 */
	vreg->save_uV = 1; /* This is not a no-op. */
	vreg->save_uV = _pm8921_ftsmps_get_voltage(vreg);

	if ((vreg->test_reg[0] & FTSMPS_CNFG1_PM_MASK) == FTSMPS_CNFG1_PM_PFM)
		vreg->optimum = REGULATOR_MODE_STANDBY;
	else
		vreg->optimum = REGULATOR_MODE_FAST;

	/* Set pull down enable based on platform data. */
	rc = pm8921_vreg_masked_write(vreg, vreg->pwr_cnfg_addr,
		(vreg->pdata.pull_down_enable ? FTSMPS_PULL_DOWN_ENABLE : 0),
		FTSMPS_PULL_DOWN_ENABLE_MASK, &vreg->pwr_cnfg_reg);

bail:
	if (rc)
		vreg_err(vreg, "pm8xxx_readb/writeb failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_init_vs(struct pm8921_vreg *vreg)
{
	int rc = 0;

	vreg->optimum = REGULATOR_MODE_NORMAL;

	/* Set pull down enable based on platform data. */
	rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
		     (vreg->pdata.pull_down_enable ? VS_PULL_DOWN_ENABLE : 0),
		     VS_PULL_DOWN_ENABLE_MASK, &vreg->ctrl_reg);

	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_init_vs300(struct pm8921_vreg *vreg)
{
	int rc;

	/* Set pull down enable based on platform data. */
	rc = pm8921_vreg_masked_write(vreg, vreg->ctrl_addr,
		    (vreg->pdata.pull_down_enable ? VS300_PULL_DOWN_ENABLE : 0),
		    VS300_PULL_DOWN_ENABLE_MASK, &vreg->ctrl_reg);

	if (rc)
		vreg_err(vreg, "pm8921_vreg_write failed, rc=%d\n", rc);

	return rc;
}

static int pm8921_init_regulator(struct pm8921_vreg *vreg)
{
	int rc = 0;

	/* save the current control register state */
	rc = pm8xxx_readb(vreg->dev->parent, vreg->ctrl_addr, &vreg->ctrl_reg);
	if (rc) {
		vreg_err(vreg, "pm8xxx_readb failed, rc=%d\n", rc);
		return rc;
	}

	switch (vreg->type) {
	case REGULATOR_TYPE_LDO:
		rc = pm8921_init_ldo(vreg);
		break;
	case REGULATOR_TYPE_NLDO1200:
		rc = pm8921_init_nldo1200(vreg);
		break;
	case REGULATOR_TYPE_SMPS:
		rc = pm8921_init_smps(vreg);
		break;
	case REGULATOR_TYPE_FTSMPS:
		rc = pm8921_init_ftsmps(vreg);
		break;
	case REGULATOR_TYPE_VS:
		rc = pm8921_init_vs(vreg);
		break;
	case REGULATOR_TYPE_VS300:
		rc = pm8921_init_vs300(vreg);
		break;
	}

	return rc;
}

static int __devinit pm8921_vreg_probe(struct platform_device *pdev)
{
	struct regulator_desc *rdesc;
	struct pm8921_vreg *vreg;
	const char *reg_name = "";
	int rc = 0;

	if (pdev == NULL)
		return -EINVAL;

	if (pdev->id >= 0 && pdev->id < PM8921_VREG_ID_MAX) {
		rdesc = &pm8921_vreg_description[pdev->id];
		vreg = &pm8921_vreg[pdev->id];
		memcpy(&(vreg->pdata), pdev->dev.platform_data,
			sizeof(struct pm8921_regulator_platform_data));
		reg_name = pm8921_vreg_description[pdev->id].name;
		vreg->name = reg_name;
		vreg->dev = &pdev->dev;

		rc = pm8921_init_regulator(vreg);
		if (rc)
			goto bail;

		vreg->rdev = regulator_register(rdesc, &pdev->dev,
				&(vreg->pdata.init_data), vreg);
		if (IS_ERR(vreg->rdev)) {
			rc = PTR_ERR(vreg->rdev);
			vreg->rdev = NULL;
			pr_err("regulator_register failed for %s, rc=%d\n",
				reg_name, rc);
		}
	} else {
		rc = -ENODEV;
	}

bail:
	if (rc)
		pr_err("error for %s, rc=%d\n", reg_name, rc);

	return rc;
}

static int __devexit pm8921_vreg_remove(struct platform_device *pdev)
{
	regulator_unregister(pm8921_vreg[pdev->id].rdev);
	return 0;
}

static struct platform_driver pm8921_vreg_driver = {
	.probe	= pm8921_vreg_probe,
	.remove	= __devexit_p(pm8921_vreg_remove),
	.driver	= {
		.name	= PM8921_REGULATOR_DEV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init pm8921_vreg_init(void)
{
	return platform_driver_register(&pm8921_vreg_driver);
}
subsys_initcall(pm8921_vreg_init);

static void __exit pm8921_vreg_exit(void)
{
	platform_driver_unregister(&pm8921_vreg_driver);
}
module_exit(pm8921_vreg_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8921 regulator driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:" PM8921_REGULATOR_DEV_NAME);
