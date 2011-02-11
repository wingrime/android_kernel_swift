/* Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/time.h>
#include <linux/i2c/bq27520.h>
#include <linux/mfd/pmic8058.h>
#include <linux/regulator/pmic8058-regulator.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/err.h>

#define DRIVER_VERSION			"1.1.0"
/*Bq27520 standard data commands*/
#define BQ27520_REG_CNTL		0x00
#define BQ27520_REG_AR			0x02
#define BQ27520_REG_ARTTE		0x04
#define BQ27520_REG_TEMP		0x06
#define BQ27520_REG_VOLT		0x08
#define BQ27520_REG_FLAGS		0x0A
#define BQ27520_REG_NAC			0x0C
#define BQ27520_REG_FAC			0x0e
#define BQ27520_REG_RM			0x10
#define BQ27520_REG_FCC			0x12
#define BQ27520_REG_AI			0x14
#define BQ27520_REG_TTE			0x16
#define BQ27520_REG_TTF			0x18
#define BQ27520_REG_SI			0x1a
#define BQ27520_REG_STTE		0x1c
#define BQ27520_REG_MLI			0x1e
#define BQ27520_REG_MLTTE		0x20
#define BQ27520_REG_AE			0x22
#define BQ27520_REG_AP			0x24
#define BQ27520_REG_TTECP		0x26
#define BQ27520_REG_SOH			0x28
#define BQ27520_REG_SOC			0x2c
#define BQ27520_REG_NIC			0x2e
#define BQ27520_REG_ICR			0x30
#define BQ27520_REG_LOGIDX		0x32
#define BQ27520_REG_LOGBUF		0x34

#define BQ27520_FLAG_DSC		BIT(0)
#define BQ27520_FLAG_FC			BIT(9)

#define BQ27520_CS_DLOGEN		BIT(15)
#define BQ27520_CS_SS		    BIT(13)

/*Control subcommands*/
#define BQ27520_SUBCMD_CTNL_STATUS  0x0000
#define BQ27520_SUBCMD_DEVCIE_TYPE  0x0001
#define BQ27520_SUBCMD_FW_VER  0x0002
#define BQ27520_SUBCMD_HW_VER  0x0003
#define BQ27520_SUBCMD_DF_CSUM  0x0004
#define BQ27520_SUBCMD_PREV_MACW   0x0007
#define BQ27520_SUBCMD_CHEM_ID   0x0008
#define BQ27520_SUBCMD_BD_OFFSET   0x0009
#define BQ27520_SUBCMD_INT_OFFSET  0x000a
#define BQ27520_SUBCMD_CC_VER   0x000b
#define BQ27520_SUBCMD_OCV  0x000c
#define BQ27520_SUBCMD_BAT_INS   0x000d
#define BQ27520_SUBCMD_BAT_REM   0x000e
#define BQ27520_SUBCMD_SET_HIB   0x0011
#define BQ27520_SUBCMD_CLR_HIB   0x0012
#define BQ27520_SUBCMD_SET_SLP   0x0013
#define BQ27520_SUBCMD_CLR_SLP   0x0014
#define BQ27520_SUBCMD_FCT_RES   0x0015
#define BQ27520_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27520_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27520_SUBCMD_SEALED   0x0020
#define BQ27520_SUBCMD_ENABLE_IT    0x0021
#define BQ27520_SUBCMD_DISABLE_IT   0x0023
#define BQ27520_SUBCMD_CAL_MODE  0x0040
#define BQ27520_SUBCMD_RESET   0x0041
#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN   (-2731)
#define BQ27520_INIT_DELAY   ((HZ)*1)

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

struct bq27520_device_info;
struct bq27520_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27520_device_info *di);
};

struct bq27520_device_info {
	struct device	*dev;
	int			id;
	struct bq27520_access_methods	*bus;
	struct power_supply	bat;
	struct i2c_client	*client;
	struct bq27520_platform_data	*pdata;
	struct work_struct counter;
	/*300ms delay is needed after bq27520 is powered up
and before any successful I2C transaction*/
	struct  delayed_work   hw_config;
};

static int coulomb_counter;
static spinlock_t lock; /*protect access to coulomb_counter*/
static struct timer_list timer;/*charge counter timer every 30 secs*/

static enum power_supply_property bq27520_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,/*Coulomb counter*/
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL,
};


static int bq27520_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct bq27520_device_info *di);

static int bq27520_read(u8 reg, int *rt_value, int b_single,
			struct bq27520_device_info *di)
{
	return di->bus->read(reg, rt_value, b_single, di);
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27520_battery_temperature(struct bq27520_device_info *di)
{
	int ret;
	int temp = 0;

	ret = bq27520_read(BQ27520_REG_TEMP, &temp, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	return temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27520_battery_voltage(struct bq27520_device_info *di)
{
	int ret;
	int volt = 0;

	ret = bq27520_read(BQ27520_REG_VOLT, &volt, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}

	return volt * 1000;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27520_battery_current(struct bq27520_device_info *di)
{
	int ret;
	int curr = 0;

	ret = bq27520_read(BQ27520_REG_AI, &curr, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}

	curr = (int)(s16)curr;
	return curr * 1000;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27520_battery_rsoc(struct bq27520_device_info *di)
{
	int ret;
	int rsoc = 0;

	ret = bq27520_read(BQ27520_REG_SOC, &rsoc, 0, di);

	if (ret) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}

	return rsoc;
}

static int bq27520_battery_status(struct bq27520_device_info *di,
				  union power_supply_propval *val)
{
	int flags = 0;
	int status;
	int ret;

	ret = bq27520_read(BQ27520_REG_FLAGS, &flags, 0, di);
	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}

	if (flags & BQ27520_FLAG_FC)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (flags & BQ27520_FLAG_DSC)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;

	val->intval = status;
	return 0;
}

static int bq27520_battery_time(struct bq27520_device_info *di, int reg,
				union power_supply_propval *val)
{
	int tval = 0;
	int ret;

	ret = bq27520_read(reg, &tval, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading register %02x\n", reg);
		return ret;
	}
	if (tval == 65535)
		return -ENODATA;

	val->intval = tval * 60;
	return 0;
}

static int bq27520_battery_full_capacity(struct bq27520_device_info *di,
					union power_supply_propval *val)
{
	int ret, tval = 0;

	ret = bq27520_read(BQ27520_REG_FCC, &tval, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading register %02x\n",
			BQ27520_REG_FCC);
		return ret;
	}

	val->intval = tval * 1000;
	return 0;
}

#define to_bq27520_device_info(x) container_of((x), \
				struct bq27520_device_info, bat);

static int bq27520_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	unsigned long flags;
	struct bq27520_device_info *di = to_bq27520_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27520_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq27520_battery_voltage(di);
		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq27520_battery_current(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq27520_battery_rsoc(di);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27520_battery_temperature(di);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27520_battery_time(di, BQ27520_REG_TTE, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27520_battery_time(di, BQ27520_REG_TTECP, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27520_battery_time(di, BQ27520_REG_TTF, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:/*Coulomb counter*/
		spin_lock_irqsave(&lock, flags);
		val->intval = coulomb_counter;
		spin_unlock_irqrestore(&lock, flags);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = bq27520_battery_full_capacity(di, val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void bq27520_powersupply_init(struct bq27520_device_info *di)
{
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27520_battery_props;
	di->bat.num_properties = 1;
	di->bat.get_property = bq27520_battery_get_property;
	di->bat.external_power_changed = NULL;
}

static void bq27520_cntl_cmd(struct bq27520_device_info *di,
				int subcmd)
{
	bq27520_i2c_txsubcmd(BQ27520_REG_CNTL, subcmd, di);
}

/*
 * i2c specific code
 */
static int bq27520_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct bq27520_device_info *di)
{
	struct i2c_msg msg;
	unsigned char data[3];

	if (!di->client)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = subcmd & 0x00FF;
	data[2] = (subcmd & 0xFF00) >> 8;

	msg.addr = di->client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	if (i2c_transfer(di->client->adapter, &msg, 1) < 0)
		return -EIO;

	return 0;
}

static int bq27520_chip_config(struct bq27520_device_info *di)
{
	int flags = 0, ret = 0;

	bq27520_cntl_cmd(di, BQ27520_SUBCMD_CTNL_STATUS);
	udelay(66);
	ret = bq27520_read(BQ27520_REG_CNTL, &flags, 0, di);
	if (ret < 0) {
		dev_err(di->dev, "error reading register %02x\n",
			 BQ27520_REG_CNTL);
		return ret;
	}
	udelay(66);

	bq27520_cntl_cmd(di, BQ27520_SUBCMD_ENABLE_IT);
	udelay(66);

	if (!(flags & BQ27520_CS_DLOGEN)) {
		bq27520_cntl_cmd(di, BQ27520_SUBCMD_ENABLE_DLOG);
		udelay(66);
	}

	return 0;
}

static void bq27520_every_30secs(unsigned long data)
{
	struct bq27520_device_info *di = (struct bq27520_device_info *)data;
	schedule_work(&di->counter);

	mod_timer(&timer, jiffies + 30*HZ);
}

static void bq27520_coulomb_counter_work(struct work_struct *work)
{
	int value = 0, temp = 0, index = 0, ret = 0;
	struct bq27520_device_info *di;
	unsigned long flags;
	int count = 0;

	di = container_of(work, struct bq27520_device_info, counter);

	/*retrieve 30 values from FIFO of coulomb data logging buffer
	 and average over time */
	do {
		ret = bq27520_read(BQ27520_REG_LOGBUF, &temp, 0, di);
		if (ret < 0)
			break;
		if (temp != 0x7FFF) {
			++count;
			value += temp;
		}
		/*delay 66uS, waiting time between continuous reading results*/
		udelay(66);
		ret = bq27520_read(BQ27520_REG_LOGIDX, &index, 0, di);
		if (ret < 0)
			break;
		udelay(66);
	} while (index != 0 || temp != 0x7FFF);

	if (ret < 0) {
		dev_err(di->dev, "Error reading datalog register\n");
		return;
	}

	if (count) {
		spin_lock_irqsave(&lock, flags);
		coulomb_counter = value/count;
		spin_unlock_irqrestore(&lock, flags);
	}
}

static void bq27520_hw_config(struct work_struct *work)
{
	int ret = 0, flags = 0, type = 0, fw_ver = 0;
	struct bq27520_device_info *di;

	di  = container_of(work, struct bq27520_device_info, hw_config.work);
	ret = bq27520_chip_config(di);
	if (ret) {
		dev_err(di->dev, "Failed to config Bq27520\n");
		return;
	}

	ret = power_supply_register(di->dev, &di->bat);
	if (ret) {
		dev_err(di->dev, "failed to register battery\n");
		return;
	}
	schedule_work(&di->counter);
	init_timer(&timer);
	timer.function = &bq27520_every_30secs;
	timer.data = (unsigned long)di;
	timer.expires = jiffies + 30*HZ;
	add_timer(&timer);

	bq27520_cntl_cmd(di, BQ27520_SUBCMD_CTNL_STATUS);
	udelay(66);
	bq27520_read(BQ27520_REG_CNTL, &flags, 0, di);
	bq27520_cntl_cmd(di, BQ27520_SUBCMD_DEVCIE_TYPE);
	udelay(66);
	bq27520_read(BQ27520_REG_CNTL, &type, 0, di);
	bq27520_cntl_cmd(di, BQ27520_SUBCMD_FW_VER);
	udelay(66);
	bq27520_read(BQ27520_REG_CNTL, &fw_ver, 0, di);

	dev_info(di->dev, "DEVICE_TYPE is 0x%02X, FIRMWARE_VERSION\
		is 0x%02X\n", type, fw_ver);
	dev_info(di->dev, "Complete bq27520 configuration 0x%02X\n", flags);
}

static int bq27520_read_i2c(u8 reg, int *rt_value, int b_single,
			struct bq27520_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

#ifdef CONFIG_BQ27520_TEST_ENABLE
static int reg;
static int subcmd;
static ssize_t bq27520_read_stdcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct bq27520_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (reg <= BQ27520_REG_ICR && reg > 0x00) {
		ret = bq27520_read(reg, &temp, 0, di);
		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27520_write_stdcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	reg = cmd;
	return ret;
}

static ssize_t bq27520_read_subcmd(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct bq27520_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (subcmd == BQ27520_SUBCMD_DEVCIE_TYPE ||
		 subcmd == BQ27520_SUBCMD_FW_VER ||
		 subcmd == BQ27520_SUBCMD_HW_VER ||
		 subcmd == BQ27520_SUBCMD_CHEM_ID) {

		bq27520_cntl_cmd(di, subcmd);/*Retrieve Chip status*/
		udelay(66);
		ret = bq27520_read(BQ27520_REG_CNTL, &temp, 0, di);

		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27520_write_subcmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	subcmd = cmd;
	return ret;
}

static DEVICE_ATTR(std_cmd, S_IRUGO|S_IWUGO, bq27520_read_stdcmd,
	bq27520_write_stdcmd);
static DEVICE_ATTR(sub_cmd, S_IRUGO|S_IWUGO, bq27520_read_subcmd,
	bq27520_write_subcmd);
static struct attribute *fs_attrs[] = {
	&dev_attr_std_cmd.attr,
	&dev_attr_sub_cmd.attr,
	NULL,
};
static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};

static struct platform_device this_device = {
	.name = "bq27520-test",
	.id = -1,
	.dev.platform_data = NULL,
};
#endif

static struct regulator *vreg_bq27520;

static int bq27520_power(bool enable, struct bq27520_device_info *di)
{
	int rc = 0;
	int ret;
	struct bq27520_platform_data *platdata;

	platdata = di->pdata;
	if (enable) {
		/*switch on Vreg_S3 */
		rc = regulator_enable(vreg_bq27520);
		if (rc < 0) {
			dev_err(di->dev, "%s: vreg %s %s failed (%d)\n",
				__func__, platdata->vreg_name, "enable", rc);
			goto vreg_fail;
		}

		/*Battery gauge enable and switch on onchip 2.5V LDO*/
		rc = gpio_request(platdata->chip_en, "GAUGE_EN");
		if (rc) {
			dev_err(di->dev, "%s: fail to request gpio %d (%d)\n",
				__func__, platdata->chip_en, rc);
			goto vreg_fail;
		}

		gpio_direction_output(platdata->chip_en, 0);
		gpio_set_value(platdata->chip_en, 1);
	} else {
		/*switch off Vreg_S3*/
		rc = regulator_disable(vreg_bq27520);
		if (rc < 0) {
			dev_err(di->dev, "%s: vreg %s %s failed (%d)\n",
				__func__, platdata->vreg_name, "disable", rc);
			goto vreg_fail;
		}

		/*switch off on-chip 2.5V LDO and disable Battery gauge*/
		gpio_set_value(platdata->chip_en, 0);
		gpio_free(platdata->chip_en);
	}
	return rc;

vreg_fail:
	ret = !enable ? regulator_enable(vreg_bq27520) :
		regulator_disable(vreg_bq27520);
	if (ret < 0) {
		dev_err(di->dev, "%s: vreg %s %s failed (%d) in err path\n",
			__func__, platdata->vreg_name,
			!enable ? "enable" : "disable", ret);
	}
	return rc;
}

static int bq27520_dev_setup(bool enable, struct bq27520_device_info *di)
{
	int rc;
	struct bq27520_platform_data *platdata;

	platdata = di->pdata;
	if (enable) {
		/*enable and set voltage Vreg_S3*/
		vreg_bq27520 = regulator_get(NULL,
				platdata->vreg_name);
		if (IS_ERR(vreg_bq27520)) {
			dev_err(di->dev, "%s: regulator get of %s\
				failed (%ld)\n", __func__, platdata->vreg_name,
				PTR_ERR(vreg_bq27520));
			rc = PTR_ERR(vreg_bq27520);
			goto vreg_get_fail;
		}
		rc = regulator_set_voltage(vreg_bq27520,
			platdata->vreg_value, platdata->vreg_value);
		if (rc) {
			dev_err(di->dev, "%s: regulator_set_voltage(%s) failed\
				 (%d)\n", __func__, platdata->vreg_name, rc);
			goto vreg_get_fail;
		}
	} else {
		regulator_put(vreg_bq27520);
	}
	return 0;

vreg_get_fail:
	regulator_put(vreg_bq27520);
	return rc;
}

static int bq27520_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	char *name;
	struct bq27520_device_info *di;
	struct bq27520_access_methods *bus;
	struct bq27520_platform_data  *pdata;
	int num;
	int retval = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	pdata = client->dev.platform_data;

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		dev_err(&client->dev, "failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;
	di->pdata = pdata;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->bat.name = name;
	bus->read = &bq27520_read_i2c;
	di->bus = bus;
	di->client = client;

#ifdef CONFIG_BQ27520_TEST_ENABLE
	platform_set_drvdata(&this_device, di);
	retval = platform_device_register(&this_device);
	if (!retval) {
		retval = sysfs_create_group(&this_device.dev.kobj,
			 &fs_attr_group);
		if (retval)
			goto batt_failed_4;
	} else
		goto batt_failed_4;
#endif

	bq27520_powersupply_init(di);

	retval = bq27520_dev_setup(true, di);
	if (retval) {
		dev_err(&client->dev, "failed to setup bq27520\n");
		goto batt_failed_4;
	}

	retval = bq27520_power(true, di);
	if (retval) {
		dev_err(&client->dev, "failed to powerup bq27520\n");
		goto batt_failed_4;
	}

	spin_lock_init(&lock);

	INIT_WORK(&di->counter, bq27520_coulomb_counter_work);
	INIT_DELAYED_WORK(&di->hw_config, bq27520_hw_config);
	schedule_delayed_work(&di->hw_config, BQ27520_INIT_DELAY);
	return 0;

batt_failed_4:
	kfree(bus);
batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27520_battery_remove(struct i2c_client *client)
{
	struct bq27520_device_info *di = i2c_get_clientdata(client);

	del_timer_sync(&timer);
	cancel_work_sync(&di->counter);

	power_supply_unregister(&di->bat);
	bq27520_cntl_cmd(di, BQ27520_SUBCMD_DISABLE_DLOG);
	udelay(66);
	bq27520_cntl_cmd(di, BQ27520_SUBCMD_DISABLE_IT);
	cancel_delayed_work_sync(&di->hw_config);

	bq27520_dev_setup(false, di);
	bq27520_power(false, di);

	kfree(di->bat.name);
	kfree(di->bus);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);
	return 0;
}

static const struct i2c_device_id bq27520_id[] = {
	{ "bq27520", 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, BQ27520_id);

static struct i2c_driver bq27520_battery_driver = {
	.driver = {
		.name = "bq27520-battery",
	},
	.probe = bq27520_battery_probe,
	.remove = bq27520_battery_remove,
	.id_table = bq27520_id,
};

static int __init bq27520_battery_init(void)
{
	int ret;
	ret = i2c_add_driver(&bq27520_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27520 driver\n");

	return ret;
}
module_init(bq27520_battery_init);

static void __exit bq27520_battery_exit(void)
{
	i2c_del_driver(&bq27520_battery_driver);
}
module_exit(bq27520_battery_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("BQ27520 battery monitor driver");
