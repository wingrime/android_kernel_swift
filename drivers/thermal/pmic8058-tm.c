/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
/*
 * Qualcomm PMIC8058 Thermal Manager driver
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/thermal.h>
#include <linux/interrupt.h>
#include <linux/mfd/pmic8058.h>
#include <linux/completion.h>

#include <linux/msm_adc.h>

/* PMIC8058 TEMP_ALRM registers */
#define	SSBI_REG_TEMP_ALRM_CTRL		0x1B
#define	SSBI_REG_TEMP_ALRM_PWM		0x9B
#define	SSBI_REG_TEMP_ALRM_TEST1	0x7A
#define	SSBI_REG_TEMP_ALRM_TEST2	0xAB

/* TEMP_ALRM_CTRL */
#define	PM8058_TEMP_ST3_SD		0x80
#define	PM8058_TEMP_ST2_SD		0x40
#define	PM8058_TEMP_STATUS_MASK		0x30
#define	PM8058_TEMP_STATUS_SHIFT	4
#define	PM8058_TEMP_THRESH_MASK		0x0C
#define	PM8058_TEMP_THRESH_SHIFT	2
#define	PM8058_TEMP_OVRD_ST3		0x02
#define	PM8058_TEMP_OVRD_ST2		0x01
#define	PM8058_TEMP_OVRD_MASK		0x03

#define	PM8058_TEMP_STAGE_STEP		20000	/* Stage step: 20 C */
#define	PM8058_TEMP_STAGE_HYSTERESIS	2000

#define	PM8058_TEMP_THRESH_MIN		105000	/* Threshold Min: 105 C */
#define	PM8058_TEMP_THRESH_STEP		5000	/* Threshold step: 5 C */

/* TEMP_ALRM_PWM */
#define	PM8058_TEMP_PWM_EN_MASK		0xC0
#define	PM8058_TEMP_PWM_EN_SHIFT	6
#define	PM8058_TEMP_PWM_PER_PRE_MASK	0x38
#define	PM8058_TEMP_PWM_PER_PRE_SHIFT	3
#define	PM8058_TEMP_PWM_PER_DIV_MASK	0x07
#define	PM8058_TEMP_PWM_PER_DIV_SHIFT	0

/* Trips: from critical to less critical */
#define PM8058_TRIP_STAGE3	0
#define PM8058_TRIP_STAGE2	1
#define PM8058_TRIP_STAGE1	2
#define PM8058_TRIP_NUM		3

#define PM8058_TEMP_ADC_CH	CHANNEL_ADC_DIE_TEMP

struct pm8058_tm_device {
	struct pm8058_chip		*pm_chip;
	struct thermal_zone_device	*tz_dev;
	unsigned long			temp;
	enum thermal_device_mode	mode;
	unsigned int			thresh;
	unsigned int			stage;
	unsigned int			irq;
	void				*adc_handle;
};

enum pmic_thermal_override_mode {
	SOFTWARE_OVERRIDE_DISABLED = 0,
	SOFTWARE_OVERRIDE_ENABLED,
};

static inline int pm8058_tm_read_ctrl(struct pm8058_chip *chip, u8 *reg)
{
	int rc;

	rc = pm8058_read(chip, SSBI_REG_TEMP_ALRM_CTRL, reg, 1);
	if (rc)
		pr_err("%s: pm8058_read FAIL: rc=%d\n", __func__, rc);

	return rc;
}

static inline int pm8058_tm_write_ctrl(struct pm8058_chip *chip, u8 reg)
{
	int rc;

	rc = pm8058_write(chip, SSBI_REG_TEMP_ALRM_CTRL, &reg, 1);
	if (rc)
		pr_err("%s: pm8058_write FAIL: rc=%d\n", __func__, rc);

	return rc;
}

static inline int pm8058_tm_write_pwm(struct pm8058_chip *chip, u8 reg)
{
	int rc;

	rc = pm8058_write(chip, SSBI_REG_TEMP_ALRM_PWM, &reg, 1);
	if (rc)
		pr_err("%s: pm8058_write FAIL: rc=%d\n", __func__, rc);

	return rc;
}

static inline int
pm8058_tm_shutdown_override(struct pm8058_chip *chip,
			    enum pmic_thermal_override_mode mode)
{
	int rc;
	u8 reg;

	rc = pm8058_tm_read_ctrl(chip, &reg);
	if (rc < 0)
		return rc;

	reg &= ~(PM8058_TEMP_OVRD_MASK | PM8058_TEMP_STATUS_MASK);
	if (mode == SOFTWARE_OVERRIDE_ENABLED)
		reg |= (PM8058_TEMP_OVRD_ST3 | PM8058_TEMP_OVRD_ST2) &
			PM8058_TEMP_OVRD_MASK;

	rc = pm8058_tm_write_ctrl(chip, reg);

	return rc;
}

static int pm8058_tz_get_temp(struct thermal_zone_device *thermal,
			      unsigned long *temp)
{
	struct pm8058_tm_device *tm = thermal->devdata;
	DECLARE_COMPLETION_ONSTACK(wait);
	struct adc_chan_result adc_result = {
		.physical = 0lu,
	};
	int rc;

	if (!tm || !temp)
		return -EINVAL;

	*temp = tm->temp;

	rc = adc_channel_request_conv(tm->adc_handle, &wait);
	if (rc < 0) {
		pr_err("%s: adc_channel_request_conv() failed, rc = %d\n",
			__func__, rc);
		return rc;
	}

	wait_for_completion(&wait);

	rc = adc_channel_read_result(tm->adc_handle, &adc_result);
	if (rc < 0) {
		pr_err("%s: adc_channel_read_result() failed, rc = %d\n",
			__func__, rc);
		return rc;
	}

	*temp = adc_result.physical;
	tm->temp = adc_result.physical;

	return 0;
}

static int pm8058_tz_get_mode(struct thermal_zone_device *thermal,
			      enum thermal_device_mode *mode)
{
	struct pm8058_tm_device *tm = thermal->devdata;

	if (!tm || !mode)
		return -EINVAL;

	*mode = tm->mode;

	return 0;
}

static int pm8058_tz_set_mode(struct thermal_zone_device *thermal,
			      enum thermal_device_mode mode)
{
	struct pm8058_tm_device *tm = thermal->devdata;

	if (!tm)
		return -EINVAL;

	if (mode != tm->mode) {
		if (mode == THERMAL_DEVICE_ENABLED)
			pm8058_tm_shutdown_override(tm->pm_chip,
						    SOFTWARE_OVERRIDE_ENABLED);
		else
			pm8058_tm_shutdown_override(tm->pm_chip,
						    SOFTWARE_OVERRIDE_DISABLED);
	}
	tm->mode = mode;

	return 0;
}

static int pm8058_tz_get_trip_type(struct thermal_zone_device *thermal,
				   int trip, enum thermal_trip_type *type)
{
	struct pm8058_tm_device *tm = thermal->devdata;

	if (!tm || trip < 0 || !type)
		return -EINVAL;

	switch (trip) {
	case PM8058_TRIP_STAGE3:
		*type = THERMAL_TRIP_CRITICAL;
		break;
	case PM8058_TRIP_STAGE2:
		*type = THERMAL_TRIP_HOT;
		break;
	case PM8058_TRIP_STAGE1:
		*type = THERMAL_TRIP_HOT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int pm8058_tz_get_trip_temp(struct thermal_zone_device *thermal,
				   int trip, unsigned long *temp)
{
	struct pm8058_tm_device *tm = thermal->devdata;
	int thresh_temp;

	if (!tm || trip < 0 || !temp)
		return -EINVAL;

	thresh_temp = tm->thresh * PM8058_TEMP_THRESH_STEP +
		      PM8058_TEMP_THRESH_MIN;

	switch (trip) {
	case PM8058_TRIP_STAGE3:
		thresh_temp += 2 * PM8058_TEMP_STAGE_STEP;
		break;
	case PM8058_TRIP_STAGE2:
		thresh_temp += PM8058_TEMP_STAGE_STEP;
		break;
	case PM8058_TRIP_STAGE1:
		break;
	default:
		return -EINVAL;
	}

	*temp = thresh_temp;

	return 0;
}

static int pm8058_tz_get_crit_temp(struct thermal_zone_device *thermal,
				   unsigned long *temp)
{
	struct pm8058_tm_device *tm = thermal->devdata;

	if (!tm || !temp)
		return -EINVAL;

	*temp = tm->thresh * PM8058_TEMP_THRESH_STEP + PM8058_TEMP_THRESH_MIN +
		2 * PM8058_TEMP_STAGE_STEP;

	return 0;
}

static struct thermal_zone_device_ops pm8058_thermal_zone_ops = {
	.get_temp = pm8058_tz_get_temp,
	.get_mode = pm8058_tz_get_mode,
	.set_mode = pm8058_tz_set_mode,
	.get_trip_type = pm8058_tz_get_trip_type,
	.get_trip_temp = pm8058_tz_get_trip_temp,
	.get_crit_temp = pm8058_tz_get_crit_temp,
};

static irqreturn_t pm8058_tm_isr(int irq, void *data)
{
	struct pm8058_tm_device *tm = data;
	int rc;
	u8 reg;

	rc = pm8058_tm_read_ctrl(tm->pm_chip, &reg);
	if (rc < 0)
		goto isr_handled;

	tm->stage = (reg & PM8058_TEMP_STATUS_MASK) >> PM8058_TEMP_STATUS_SHIFT;
	tm->thresh = (reg & PM8058_TEMP_THRESH_MASK) >>
			PM8058_TEMP_THRESH_SHIFT;

	if (reg & (PM8058_TEMP_ST2_SD | PM8058_TEMP_ST3_SD)) {
		reg &= ~(PM8058_TEMP_ST2_SD | PM8058_TEMP_ST3_SD |
			 PM8058_TEMP_STATUS_MASK);
		pm8058_tm_write_ctrl(tm->pm_chip, reg);
	}

	thermal_zone_device_update(tm->tz_dev);

	/* Notify user space */
	if (tm->mode == THERMAL_DEVICE_ENABLED)
		kobject_uevent(&tm->tz_dev->device.kobj, KOBJ_CHANGE);

isr_handled:
	return IRQ_HANDLED;
}

static int pm8058_tm_init_reg(struct pm8058_tm_device *tm)
{
	int rc;
	u8 reg;

	rc = pm8058_tm_read_ctrl(tm->pm_chip, &reg);
	if (rc < 0)
		return rc;

	tm->stage = (reg & PM8058_TEMP_STATUS_MASK) >> PM8058_TEMP_STATUS_SHIFT;
	tm->temp = 0;

	/* Use temperature threshold set 0: (105, 125, 145) */
	tm->thresh = 0;
	reg = (tm->thresh << PM8058_TEMP_THRESH_SHIFT) &
	      PM8058_TEMP_THRESH_MASK;
	rc = pm8058_tm_write_ctrl(tm->pm_chip, reg);
	if (rc < 0)
		return rc;

	/*
	 * Set the PMIC alarm module PWM to have a frequency of 8 Hz. This
	 * helps cut down on the number of unnecessary interrupts fired when
	 * changing between thermal stages.  Also, Enable the over temperature
	 * PWM whenever the PMIC is enabled.
	 */
	reg =  1 << PM8058_TEMP_PWM_EN_SHIFT |
	       3 << PM8058_TEMP_PWM_PER_PRE_SHIFT |
	       3 << PM8058_TEMP_PWM_PER_DIV_SHIFT;

	rc = pm8058_tm_write_pwm(tm->pm_chip, reg);

	return rc;
}

static int __devinit pmic8058_tm_probe(struct platform_device *pdev)
{
	DECLARE_COMPLETION_ONSTACK(wait);
	struct pm8058_tm_device *tmdev;
	struct pm8058_chip *pm_chip;
	unsigned int irq;
	int rc;

	pm_chip = platform_get_drvdata(pdev);
	if (pm_chip == NULL) {
		pr_err("%s: no driver data passed in.\n", __func__);
		return -EFAULT;
	}

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		pr_err("%s: no IRQ passed in.\n", __func__);
		return -EFAULT;
	}

	tmdev = kzalloc(sizeof *tmdev, GFP_KERNEL);
	if (tmdev == NULL) {
		pr_err("%s: kzalloc() failed.\n", __func__);
		return -ENOMEM;
	}

	rc = adc_channel_open(PM8058_TEMP_ADC_CH, &(tmdev->adc_handle));
	if (rc < 0) {
		pr_err("%s: adc_channel_open() failed.\n", __func__);
		kfree(tmdev);
		return rc;
	}

	/* calibrate the die temperature sensor */
	if (adc_calib_request(tmdev->adc_handle, &wait) == CALIB_STARTED)
		wait_for_completion(&wait);

	tmdev->pm_chip = pm_chip;
	tmdev->tz_dev = thermal_zone_device_register("pm8058_tz",
						     PM8058_TRIP_NUM, tmdev,
						     &pm8058_thermal_zone_ops,
						     0, 0, 0, 0);
	if (tmdev->tz_dev == NULL) {
		pr_err("%s: thermal_zone_device_register() failed.\n",
		       __func__);
		adc_channel_close(tmdev->adc_handle);
		kfree(tmdev);
		return -ENODEV;
	}

	rc = pm8058_tm_init_reg(tmdev);
	pm8058_tm_shutdown_override(tmdev->pm_chip, SOFTWARE_OVERRIDE_DISABLED);
	if (rc < 0) {
		thermal_zone_device_unregister(tmdev->tz_dev);
		adc_channel_close(tmdev->adc_handle);
		kfree(tmdev);
		return rc;
	}

	/* start in HW control, switch to SW control when user changes mode */
	tmdev->mode = THERMAL_DEVICE_DISABLED;
	thermal_zone_device_update(tmdev->tz_dev);

	platform_set_drvdata(pdev, tmdev);

	rc = request_threaded_irq(irq, NULL, pm8058_tm_isr,
			 IRQF_TRIGGER_RISING | IRQF_DISABLED,
			 "pm8058-tm-irq", tmdev);
	if (rc < 0) {
		pr_err("%s: request_irq(%d) FAIL: %d\n", __func__, irq, rc);
		thermal_zone_device_unregister(tmdev->tz_dev);
		platform_set_drvdata(pdev, tmdev->pm_chip);
		adc_channel_close(tmdev->adc_handle);
		kfree(tmdev);
		return rc;
	}
	tmdev->irq = irq;

	pr_notice("%s: OK\n", __func__);
	return 0;
}

static int __devexit pmic8058_tm_remove(struct platform_device *pdev)
{
	struct pm8058_tm_device *tmdev = platform_get_drvdata(pdev);

	thermal_zone_device_unregister(tmdev->tz_dev);
	platform_set_drvdata(pdev, tmdev->pm_chip);
	pm8058_tm_shutdown_override(tmdev->pm_chip, THERMAL_DEVICE_DISABLED);
	adc_channel_close(tmdev->adc_handle);
	free_irq(tmdev->irq, tmdev);
	kfree(tmdev);

	return 0;
}

#ifdef CONFIG_PM
static int pmic8058_tm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pm8058_tm_device *tm = platform_get_drvdata(pdev);

	/* Clear override bits in suspend to allow hardware control */
	pm8058_tm_shutdown_override(tm->pm_chip, SOFTWARE_OVERRIDE_DISABLED);

	return 0;
}

static int pmic8058_tm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pm8058_tm_device *tm = platform_get_drvdata(pdev);

	/* Override hardware actions so software can control */
	if (tm->mode == THERMAL_DEVICE_ENABLED)
		pm8058_tm_shutdown_override(tm->pm_chip,
					    SOFTWARE_OVERRIDE_ENABLED);

	return 0;
}

static const struct dev_pm_ops pmic8058_tm_pm_ops = {
	.suspend = pmic8058_tm_suspend,
	.resume = pmic8058_tm_resume,
};

#define PM8058_TM_PM_OPS	(&pmic8058_tm_pm_ops)
#else
#define PM8058_TM_PM_OPS	NULL
#endif

static struct platform_driver pmic8058_tm_driver = {
	.probe	= pmic8058_tm_probe,
	.remove	= __devexit_p(pmic8058_tm_remove),
	.driver	= {
		.name = "pm8058-tm",
		.owner = THIS_MODULE,
		.pm = PM8058_TM_PM_OPS,
	},
};

static int __init pm8058_tm_init(void)
{
	return platform_driver_register(&pmic8058_tm_driver);
}

static void __exit pm8058_tm_exit(void)
{
	platform_driver_unregister(&pmic8058_tm_driver);
}

module_init(pm8058_tm_init);
module_exit(pm8058_tm_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8058 Thermal Manager driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pmic8058-tm");
