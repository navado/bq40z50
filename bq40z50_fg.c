/*
 * bq40z50 fuel gauge driver
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"[bq40z50] %s: " fmt, __func__
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>

#define bq_info	pr_info
#define bq_dbg	pr_debug
#define bq_err	pr_err
#ifdef CONFIG_BQ40Z50_DEBUG
#define bq_log	pr_err
#else
#define bq_log	pr_info
#endif

#define	INVALID_REG_ADDR	0xFF

#define FG_FLAGS_FD				BIT(4)
#define	FG_FLAGS_FC				BIT(5)
#define	FG_FLAGS_DSG				BIT(6)
#define FG_FLAGS_RCA				BIT(9)
#define FG_BM_CAPM				BIT(15)

// Once a minute
#define DEFAULT_POLL_INTERVAL		60
static unsigned int poll_interval = DEFAULT_POLL_INTERVAL;
module_param(poll_interval, uint, 0644);
MODULE_PARM_DESC(poll_interval, "battery polling interval in seconds. 0 disables polling");


enum bq_fg_reg_idx {
	BQ_FG_REG_MAC = 0,
	BQ_FG_REG_TEMP,		/* Battery Temperature */
	BQ_FG_REG_VOLT,		/* Battery Voltage */
	BQ_FG_REG_AI,		/* Average Current */
	BQ_FG_REG_BATT_STATUS,	/* BatteryStatus */
	BQ_FG_REG_TTE,		/* Time to Empty */
	BQ_FG_REG_TTF,		/* Time to Full */
	BQ_FG_REG_FCC,		/* Full Charge Capacity */
	BQ_FG_REG_RM,		/* Remaining Capacity */
	BQ_FG_REG_CC,		/* Cycle Count */
	BQ_FG_REG_SOC,		/* Relative State of Charge */
	BQ_FG_REG_SOH,		/* State of Health */
	BQ_FG_REG_DC,		/* Design Capacity */
	BQ_FG_REG_MBA,		/* ManufacturerBlockAccess*/
	BQ_FG_REG_SN,		/* Serial Number */
	BQ_FG_REG_DV,		/* Design Voltage */
	BQ_FG_REG_CCVM,		/* Constant Max Voltage */
	BQ_FG_REG_CCCM,		/* Constant Max Current */
	BQ_FG_REG_I,		/* Momentary current */
	BQ_FG_REG_BM,		/* Battery Mode */
	NUM_REGS,
};

static u8 bq40z50_regs[NUM_REGS] = {
	0x00,	/* CONTROL */
	0x08,	/* TEMP */
	0x09,	/* VOLT */
	0x0B,	/* AVG CURRENT */
	0x16,	/* FLAGS */
	0x12,	/* Time to empty */
	0x13,	/* Time to full */
	0x10,	/* Full charge capacity */
	0x0F,	/* Remaining Capacity */
	0x17,	/* CycleCount */
	0x0D,	/* State of Charge */
	0x4F,	/* State of Health */
	0x18,	/* Design Capacity */
	0x44,	/* ManufacturerBlockAccess*/
	0x1C,	/* Serial Number */
	0x19,	/* Design Voltage */
	0x15,	/* Max Charge Voltage */
	0x14,	/* Max Charge Current */
	0x0A,	/* Momentary Current */
	0x03,	/* Battery Mode */
};


char * bq_fg_reg_cmd_names[] = {
		"Control",
		"Temperature",
		"Battery Voltage",
		"Average Current",
		"Battery Status",
		"Time To Empty",
		"Time To Full",
		"Full Charge Capacity",
		"Remaining Capacity",
		"Cycle Count",
		"Relative State of Charge",
		"State of Health",
		"Design Capacity",
		"ManufacturerBlockAccess",
		"Serial Number",
		"Design Voltage",
		"Constant Max Voltage",
		"Constant Max Current",
		"Momentary Current",
		"Battery Mode"
};

enum bq_fg_mac_cmd {
	FG_MAC_CMD_OP_STATUS	= 0x0000,
	FG_MAC_CMD_DEV_TYPE	= 0x0001,
	FG_MAC_CMD_FW_VER	= 0x0002,
	FG_MAC_CMD_HW_VER	= 0x0003,
	FG_MAC_CMD_IF_SIG	= 0x0004,
	FG_MAC_CMD_CHEM_ID	= 0x0006,
	FG_MAC_CMD_GAUGING	= 0x0021,
	FG_MAC_CMD_SEAL		= 0x0030,
	FG_MAC_CMD_DEV_RESET	= 0x0041,
};


enum {
	SEAL_STATE_RSVED,
	SEAL_STATE_UNSEALED,
	SEAL_STATE_SEALED,
	SEAL_STATE_FA,
};


enum bq_fg_device {
	BQ40Z50,
};

static const unsigned char *device2str[] = {
	"bq40z50",
};

struct bq_fg_chip {
	struct device *dev;
	struct i2c_client *client;


	struct mutex i2c_rw_lock;
	struct mutex data_lock;
	struct mutex irq_complete;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	int fw_ver;
	int df_ver;

	u8 chip;
	u8 regs[NUM_REGS];

	/* status tracking */

	bool batt_fc;
	bool batt_fd;	/* full depleted */

	bool batt_dsg;
	bool batt_rca;	/* remaining capacity alarm */
	union{
		u16 bm_intval;
		struct{
			unsigned bm_icc		: 1;	/* Internal Charge Controller */
			unsigned bm_pbs		: 1;	/* Primary Battery Support */
			unsigned bm_rsvd1	: 5;	/* Reserved, do not use */
			unsigned bm_cf		: 1;	/* Conditioning flag */
			unsigned bm_cc		: 1;	/* Condition flag */
			unsigned bm_pb		: 1;	/* Primary battery */
			unsigned bm_rdvd2	: 3;	/* Reserved, do not use */
			unsigned bm_am		: 1;	/* Alarm Mode */
			unsigned bm_chgm	: 1;	/* Charger Mode */
			unsigned bm_capm 	: 1;	/* Capacity reporting units */
		};
	};
	int seal_state; /* 0 - Full Access, 1 - Unsealed, 2 - Sealed */
	u16 batt_tte;
	u16 batt_ttf;
	u16 batt_soc;
	u16 batt_fcc;	/* Full charge capacity */
	u16 batt_rm;	/* Remaining capacity */
	u16 batt_dc;	/* Design Capacity */
	u16 batt_dv;	/* Design Volt */
	u16 batt_volt;
	u16 batt_temp;
	s16 batt_curr;
	s16 batt_curr_avg;
	u16 batt_sn;
	u16 batt_cccm;
	u16 batt_ccvm;
	u16 batt_cyclecnt;	/* cycle count */

	/* debug */
	int skip_reads;
	int skip_writes;

	int fake_soc;
	int fake_temp;

	struct	delayed_work monitor_work;

	struct power_supply fg_psy;
};

static int __fg_read_word(struct i2c_client *client, u8 reg, u16 *val)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		bq_dbg("i2c read word fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u16)ret;

	return 0;
}

static int __fg_write_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0) {
		bq_dbg("i2c write word fail: can't write 0x%02X to reg 0x%02X\n",
				val, reg);
		return ret;
	}

	return 0;
}

static int __fg_read_block(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{

	int ret;

	/* len is ignored due to smbus block reading contains Num of bytes to be returned */
	ret = i2c_smbus_read_block_data(client, reg, buf);

	return ret;
}

static int __fg_write_block(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{
	int ret;

	ret = i2c_smbus_write_block_data(client, reg, len, buf);

	return ret;
}

static int fg_read_word(struct bq_fg_chip *bq, u8 reg, u16 *val)
{
	int ret;

	if (bq->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_word(bq->client, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int fg_write_word(struct bq_fg_chip *bq, u8 reg, u16 val)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_word(bq->client, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int fg_read_block(struct bq_fg_chip *bq, u8 reg, u8 *buf, u8 len)
{
	int ret;

	if (bq->skip_reads)
		return 0;
	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_block(bq->client, reg, buf, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;

}

static int fg_write_block(struct bq_fg_chip *bq, u8 reg, u8 *data, u8 len)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_block(bq->client, reg, data, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

#ifdef CONFIG_BQ40Z50_DEBUG
static u8 checksum(u8 *data, u8 len)
{
	u8 i;
	u16 sum = 0;

	for (i = 0; i < len; i++)
		sum += data[i];

	sum &= 0xFF;

	return 0xFF - sum;
}

static void fg_print_buf(const char *msg, u8 *buf, u8 len)
{
	int i;
	int idx = 0;
	int num;
	u8 strbuf[128];

	bq_err("%s buf: ", msg);
	for (i = 0; i < len; i++) {
		num = sprintf(&strbuf[idx], "%02X ", buf[i]);
		idx += num;
	}
	bq_err("%s\n", strbuf);
}
#else
static void fg_print_buf(const char *msg, u8 *buf, u8 len)
{}
#endif

static int fg_mac_read_block(struct bq_fg_chip *bq, u16 cmd, u8 *buf, u8 len)
{
	int ret;
	u8 t_buf[40];
	u8 t_len;
	int i;

	t_buf[0] = (u8)(cmd >> 8);
	t_buf[1] = (u8)cmd;
	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_MBA], t_buf, 2);
	if (ret < 0)
		return ret;

	msleep(100);

	ret = fg_read_block(bq, bq->regs[BQ_FG_REG_MBA], t_buf, 36);
	if (ret < 0)
		return ret;
	t_len = ret;
	/* ret contains number of data bytes in gauge's response*/
	fg_print_buf("mac_read_block", t_buf, t_len);

	for (i = 0; i < t_len - 2; i++)
		buf[i] = t_buf[i+2];

	return 0;
}

#if 0
static int fg_mac_write_block(struct bq_fg_chip *bq, u16 cmd, u8 *data, u8 len)
{
	int ret;
	u8 cksum;
	u8 t_buf[40];
	int i;

	if (len > 32)
		return -1;

	t_buf[0] = (u8)(cmd >> 8);
	t_buf[1] = (u8)cmd;
	for (i = 0; i < len; i++)
		t_buf[i+2] = data[i];

	/*write command/addr, data*/
	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_MBA], t_buf, len + 2);
	if (ret < 0)
		return ret;

	return ret;
}
#endif

static void fg_read_fw_version(struct bq_fg_chip *bq)
{

	int ret;
	u8 buf[36];

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_MBA], FG_MAC_CMD_FW_VER);

	if (ret < 0) {
		bq_err("Failed to send firmware version subcommand:%d\n", ret);
		return;
	}

	mdelay(2);

	ret = fg_mac_read_block(bq, bq->regs[BQ_FG_REG_MBA], buf, 11);
	if (ret < 0) {
		bq_err("Failed to read firmware version:%d\n", ret);
		return;
	}

	bq_log("FW Ver:%04X, Build:%04X\n",
		buf[2] << 8 | buf[3], buf[4] << 8 | buf[5]);
	bq_log("Ztrack Ver:%04X\n", buf[7] << 8 | buf[8]);
}


static int fg_read_word_cmd(struct bq_fg_chip *bq, int reg_id, u16 *value){
	int ret;
	u16 v = 0xFFFF;
	if (bq->regs[reg_id] == INVALID_REG_ADDR) {
		bq_err("command id = %02x not supported!\n",reg_id);
		return 0;
	}
	ret = fg_read_word(bq, bq->regs[reg_id], &v);
	// Lock moves here - less calls to lock and less code locked
	mutex_lock(&bq->data_lock);
	*value = (int)v;
	mutex_unlock(&bq->data_lock);
	if (ret < 0)
		bq_dbg("could not read %s, ret = %d\n",bq_fg_reg_cmd_names[reg_id], ret);
	return ret;
}

static int fg_read_status(struct bq_fg_chip *bq)
{
	int ret;
	u16 flags;
	ret = fg_read_word_cmd(bq,BQ_FG_REG_BATT_STATUS, &flags);
	if (ret < 0)
		return ret;
	mutex_lock(&bq->data_lock);
	bq->batt_fc		= !!(flags & FG_FLAGS_FC);
	bq->batt_fd		= !!(flags & FG_FLAGS_FD);
	bq->batt_rca		= !!(flags & FG_FLAGS_RCA);
	bq->batt_dsg		= !!(flags & FG_FLAGS_DSG);
	mutex_unlock(&bq->data_lock);

	return 0;
}

static int fg_read_bm(struct bq_fg_chip *bq){
	int ret;
	u16 flags;
	ret = fg_read_word_cmd(bq, BQ_FG_REG_BM, &flags);
	if (ret < 0)
		return ret;
	mutex_lock(&bq->data_lock);
	bq->bm_intval = flags;
	mutex_unlock(&bq->data_lock);
	return 0;
}

static inline int fg_read_ccvm(struct bq_fg_chip *bq)
{
	return fg_read_word_cmd(bq, BQ_FG_REG_CCVM, &bq->batt_ccvm);
}

static inline int fg_read_cccm(struct bq_fg_chip *bq)
{
	return fg_read_word_cmd(bq, BQ_FG_REG_CCCM, &bq->batt_cccm);
}

static inline int fg_read_rsoc(struct bq_fg_chip *bq)
{
	return fg_read_word_cmd(bq, BQ_FG_REG_SOC, &bq->batt_soc);
}

static inline int fg_read_temperature(struct bq_fg_chip *bq)
{
	return fg_read_word_cmd(bq, BQ_FG_REG_TEMP, &bq->batt_temp);
}

static inline int fg_read_volt(struct bq_fg_chip *bq)
{
	return fg_read_word_cmd(bq, BQ_FG_REG_VOLT, &bq->batt_volt);
}

static int _fg_read_current(struct bq_fg_chip *bq, s16* val)
{
	int ret;
	u16 c = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_I], &c);
	if (ret < 0) {
		bq_dbg("could not read current, ret = %d\n", ret);
		return ret;
	}

	*val = c;

	return ret;
}

static inline int fg_read_current(struct bq_fg_chip *bq)
{
	return _fg_read_current(bq,&bq->batt_curr);
}

static inline int fg_read_current_avg(struct bq_fg_chip *bq)
{
	return _fg_read_current(bq,&bq->batt_curr_avg);
}


static inline int fg_read_fcc(struct bq_fg_chip *bq)
{
	return fg_read_word_cmd(bq, BQ_FG_REG_FCC, &bq->batt_fcc);
}

static inline int fg_read_dv(struct bq_fg_chip *bq)
{
	return fg_read_word_cmd(bq, BQ_FG_REG_DV, &bq->batt_dv);
}

static inline int fg_read_dc(struct bq_fg_chip *bq)
{
	return fg_read_word_cmd(bq, BQ_FG_REG_DC, &bq->batt_dc);
}

static inline int fg_read_rm(struct bq_fg_chip *bq)
{
	return fg_read_word_cmd(bq, BQ_FG_REG_RM, &bq->batt_rm);
}

static inline int fg_read_cyclecount(struct bq_fg_chip *bq)
{
	return fg_read_word_cmd(bq, BQ_FG_REG_CC, &bq->batt_cyclecnt);
}

static int fg_read_tte(struct bq_fg_chip *bq)
{
	return fg_read_word_cmd(bq, BQ_FG_REG_TTE, &bq->batt_tte);
}

static int fg_read_ttf(struct bq_fg_chip *bq)
{
	return fg_read_word_cmd(bq, BQ_FG_REG_TTF, &bq->batt_ttf);
}

static int fg_read_sn(struct bq_fg_chip *bq)
{
	return fg_read_word_cmd(bq, BQ_FG_REG_SN, &bq->batt_sn);
}

static int fg_get_batt_status(struct bq_fg_chip *bq)
{

	if (fg_read_status(bq) < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	if (bq->batt_fc)
		return POWER_SUPPLY_STATUS_FULL;
	else if (bq->batt_dsg)
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else if (bq->batt_curr > 0)
		return POWER_SUPPLY_STATUS_CHARGING;
	else
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

}


static int fg_get_batt_capacity_level(struct bq_fg_chip *bq)
{

	if (bq->batt_fc)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (bq->batt_rca)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (bq->batt_fd)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

}


static enum power_supply_property fg_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	/*POWER_SUPPLY_PROP_HEALTH,*//*implement it in battery power_supply*/
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TYPE,
//	POWER_SUPPLY_PROP_RESISTANCE_ID,
//	POWER_SUPPLY_PROP_UPDATE_NOW,
};

static char bq_serial[5];
#define check_no_data()					{ if (val->intval == 0xFFFF) val->intval = -1; }
#define FG_KELVIN_TO_CELSIUS_DEC(x)		(x - 2730)
#define MILLI_TO_MICRO(x)				(x * 1000)

static int fg_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq_fg_chip *bq = container_of(psy, struct bq_fg_chip, fg_psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fg_get_batt_status(bq);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = fg_read_volt(bq);
		val->intval = MILLI_TO_MICRO(bq->batt_volt);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (fg_get_batt_status(bq) > 0)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = fg_read_current(bq);
		val->intval = MILLI_TO_MICRO(-bq->batt_curr);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = fg_read_current_avg(bq);
		val->intval = MILLI_TO_MICRO(-bq->batt_curr_avg);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = fg_read_rsoc(bq);
		val->intval = bq->batt_soc;
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = fg_get_batt_capacity_level(bq);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		ret = fg_read_temperature(bq);
		val->intval = FG_KELVIN_TO_CELSIUS_DEC(bq->batt_temp);
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = fg_read_tte(bq);
		val->intval = bq->batt_tte;
		check_no_data();
		break;

	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = fg_read_ttf(bq);
		val->intval = bq->batt_ttf;
		check_no_data();
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = fg_read_fcc(bq);
		val->intval = MILLI_TO_MICRO(bq->batt_fcc);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = fg_read_dc(bq);
		val->intval = MILLI_TO_MICRO(bq->batt_dc);
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = fg_read_cyclecount(bq);
		val->intval = bq->batt_cyclecnt;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;

	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		ret = sprintf(bq_serial,"%04x", bq->batt_sn);
		val->strval = bq_serial;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		ret = fg_read_dv(bq);
		val->intval = MILLI_TO_MICRO(bq->batt_dv);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = fg_read_cccm(bq);
		val->intval = MILLI_TO_MICRO(bq->batt_cccm);
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = fg_read_ccvm(bq);
		val->intval = MILLI_TO_MICRO(bq->batt_ccvm);
		break;

	case POWER_SUPPLY_PROP_TYPE:
		val->intval = POWER_SUPPLY_TYPE_BATTERY;
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = fg_read_rm(bq);
		val->intval = bq->batt_rm;
		break;

	default:
		return -EINVAL;
	}

	return ret;
}
#ifdef CONFIG_BQ40Z50_DEBUG
static void fg_dump_registers(struct bq_fg_chip *bq);
#endif
static int fg_set_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       const union power_supply_propval *val)
{
#ifdef CONFIG_BQ40Z50_DEBUG
	struct bq_fg_chip *bq = container_of(psy, struct bq_fg_chip, fg_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
		bq->fake_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		bq->fake_soc = val->intval;
		power_supply_changed(&bq->fg_psy);
		break;
	default:
		return -EINVAL;
	}

	return 0;
#else
	// Not supported in this driver
	return -EINVAL;
#endif
}

static int fg_prop_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
#ifdef CONFIG_BQ40Z50_DEBUG
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = 1;
		break;
#endif
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int fg_psy_register(struct bq_fg_chip *bq)
{
	int ret;
	bq->fg_psy.name = "BQ40Z50";
	bq->fg_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	bq->fg_psy.properties = fg_props;
	bq->fg_psy.num_properties = ARRAY_SIZE(fg_props);
	bq->fg_psy.get_property = fg_get_property;
	bq->fg_psy.set_property = fg_set_property;
	bq->fg_psy.property_is_writeable = fg_prop_is_writeable;
#ifdef CONFIG_BQ40Z50_DEBUG
	bq_info("properties filled");
#endif
	ret = power_supply_register(bq->dev, &bq->fg_psy);
	if (ret) {
		bq_err("Failed to register fg_psy");
		return ret;
	}
	return 0;
}

static void fg_psy_unregister(struct bq_fg_chip *bq)
{
	power_supply_unregister(&bq->fg_psy);
}

static const u8 fg_dump_regs[] = {
	0x08, 0x09, 0x0A, 0x0D,
	0x0F, 0x10, 0x16, 0x3C,
	0x3D, 0x3E, 0x3F, 0x54,
	0x55, 0x56, 0x57, 0x71,
	0x72, 0x74, 0x75, 0x76,
	0x77, 0x78,
};

static ssize_t fg_attr_show_Ra_table(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	u8 t_buf[40];
	u8 temp_buf[40];
	int ret;
	int i, idx, len;

	ret = fg_mac_read_block(bq, 0x4102, t_buf, 32);
	if (ret < 0)
		return 0;

	idx = 0;
	len = sprintf(temp_buf, "RaTable:\n");
	memcpy(&buf[idx], temp_buf, len);
	idx += len;
	for (i = 0; i < 15; i++) {
		len = sprintf(temp_buf, "%d ", t_buf[i*2] << 8 | t_buf[i*2 + 1]);
		memcpy(&buf[idx], temp_buf, len);
		idx += len;
	}

	return idx;
}

static ssize_t fg_attr_show_Qmax(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	int ret;
	u8 t_buf[32];
	u8 temp_buf[32];
	int i, idx, len;

	memset(t_buf, 0, 64);
	/* GaugingStatus3 register contains Qmax value */
	ret = fg_mac_read_block(bq, 0x0075, t_buf, 24);
	if (ret < 0)
		return 0;
	idx = 0;
	for (i = 0; i < 4; i++) {
		len = sprintf(temp_buf, "Qmax Cell %d = %d\n", i, (t_buf[i*2] << 8) | t_buf[i*2+1]);
		memcpy(&buf[idx], temp_buf, len);
		idx += len;
	}

	return idx;
}

static DEVICE_ATTR(RaTable, S_IRUGO, fg_attr_show_Ra_table, NULL);
static DEVICE_ATTR(Qmax, S_IRUGO, fg_attr_show_Qmax, NULL);

static struct attribute *fg_attributes[] = {
	&dev_attr_RaTable.attr,
	&dev_attr_Qmax.attr,
	NULL,
};

static const struct attribute_group fg_attr_group = {
	.attrs = fg_attributes,
};

#ifdef CONFIG_BQ40Z50_DEBUG
static void fg_dump_registers(struct bq_fg_chip *bq)
{
	int i;
	int ret;
	u16 val;

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		msleep(5);
		ret = fg_read_word(bq, fg_dump_regs[i], &val);
		if (!ret)
			bq_log("Reg[%02X] = 0x%04X\n", fg_dump_regs[i], val);
	}
}
#endif

static irqreturn_t fg_btp_irq_thread(int irq, void *dev_id)
{
	/*struct bq_fg_chip *bq = dev_id;*/

	/* user can update btp trigger point here*/


	return IRQ_HANDLED;
}

static void fg_log_battery_mode(struct bq_fg_chip *bq){
	bq_log("Battery Mode: ICC=%d PBS=%d CF=%d CC=%d PB=%d AM=%d CHGM=%d CAPM=%d\n",
			bq->bm_icc,
			bq->bm_pbs,
			bq->bm_cf,
			bq->bm_cc,
			bq->bm_pb,
			bq->bm_am,
			bq->bm_chgm,
			bq->bm_capm
			);
}

static void fg_update_status(struct bq_fg_chip *bq)
{
	fg_get_batt_status(bq);
	fg_read_bm(bq);
	fg_read_rsoc(bq);
	fg_read_volt(bq);
	fg_read_current(bq);
	fg_read_temperature(bq);
	fg_read_rm(bq);
	fg_read_tte(bq);
	fg_read_ttf(bq);
	fg_read_cyclecount(bq);
	fg_read_sn(bq);
	fg_read_dv(bq);

	bq_dbg("sn: %04x RSOC:%d, Volt:%d, Current:%d, Temperature:%d\n",
		bq->batt_sn, bq->batt_soc, bq->batt_volt, bq->batt_curr, bq->batt_temp);
}

static void fg_monitor_workfunc(struct work_struct *work)
{
	struct bq_fg_chip *bq =
			container_of(work, struct bq_fg_chip, monitor_work.work);

	if (poll_interval > 0){
		// Poll the battery
		fg_update_status(bq);
#ifdef CONFIG_BQ40Z50_DEBUG
		fg_dump_registers(bq);
		fg_log_battery_mode(bq);
#endif
		schedule_delayed_work(&bq->monitor_work, poll_interval * HZ);
	} else {
		// in case someone wants to reenable, we'll poll once in a while
		schedule_delayed_work(&bq->monitor_work, DEFAULT_POLL_INTERVAL * HZ);
	}
}


static void determine_initial_status(struct bq_fg_chip *bq)
{
	fg_update_status(bq);
	fg_log_battery_mode(bq);
}

static int bq_fg_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{

	int ret;
	struct bq_fg_chip *bq;
	u8 *regs;

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);

	if (!bq)
		return -ENOMEM;
	bq_dbg("-------------->>>>> Probing BQ40Z50 driver\n");
	bq->dev = &client->dev;
	bq->client = client;
	bq->chip = id->driver_data;

	bq->batt_soc	= -ENODATA;
	bq->batt_fcc	= -ENODATA;
	bq->batt_rm	= -ENODATA;
	bq->batt_dc	= -ENODATA;
	bq->batt_volt	= -ENODATA;
	bq->batt_temp	= -ENODATA;
	bq->batt_curr	= -ENODATA;
	bq->batt_cyclecnt = -ENODATA;

	bq->fake_soc	= -EINVAL;
	bq->fake_temp	= -EINVAL;

	if (bq->chip == BQ40Z50) {
		regs = bq40z50_regs;
	} else {
		bq_err("unexpected fuel gauge: %d\n", bq->chip);
		regs = bq40z50_regs;
	}

	memcpy(bq->regs, regs, NUM_REGS);

	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
			fg_btp_irq_thread,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"bq fuel gauge irq", bq);
		if (ret < 0) {
			bq_err("request irq for irq=%d failed, ret = %d\n", client->irq, ret);
			goto err_1;
		}
		enable_irq_wake(client->irq);
	}

	device_init_wakeup(bq->dev, 1);

	fg_read_fw_version(bq);

	fg_psy_register(bq);

	ret = sysfs_create_group(&bq->dev->kobj, &fg_attr_group);
	if (ret)
		bq_err("Failed to register sysfs, err:%d\n", ret);

	determine_initial_status(bq);

	INIT_DELAYED_WORK(&bq->monitor_work, fg_monitor_workfunc);
	schedule_delayed_work(&bq->monitor_work, poll_interval * HZ);

	bq_log("bq fuel gauge sn %04x probe successfully, %s\n",
			bq->batt_sn,
			device2str[bq->chip]);

	return 0;

err_1:
	fg_psy_unregister(bq);
	return ret;
}


static inline bool is_device_suspended(struct bq_fg_chip *bq)
{
	return !bq->resume_completed;
}


static int bq_fg_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);

	return 0;
}

static int bq_fg_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;

}


static int bq_fg_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		fg_btp_irq_thread(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	power_supply_changed(&bq->fg_psy);

	return 0;
}

static int bq_fg_remove(struct i2c_client *client)
{
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&bq->monitor_work);

	fg_psy_unregister(bq);

	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->irq_complete);

	sysfs_remove_group(&bq->dev->kobj, &fg_attr_group);

	return 0;

}

static void bq_fg_shutdown(struct i2c_client *client)
{
	pr_info("bq fuel gauge driver shutdown!\n");
}

static struct of_device_id bq_fg_match_table[] = {
	{.compatible = "ti,bq40z50",},
	{},
};
MODULE_DEVICE_TABLE(of, bq_fg_match_table);

static const struct i2c_device_id bq_fg_id[] = {
	{ "bq40z50", BQ40Z50 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq_fg_id);

static const struct dev_pm_ops bq_fg_pm_ops = {
	.resume		= bq_fg_resume,
	.suspend_noirq = bq_fg_suspend_noirq,
	.suspend	= bq_fg_suspend,
};

static struct i2c_driver bq_fg_driver = {
	.driver	= {
		.name   = "bq_fg",
		.owner  = THIS_MODULE,
		.of_match_table = bq_fg_match_table,
		.pm     = &bq_fg_pm_ops,
	},
	.id_table       = bq_fg_id,

	.probe          = bq_fg_probe,
	.remove		= bq_fg_remove,
	.shutdown	= bq_fg_shutdown,

};

module_i2c_driver(bq_fg_driver);

MODULE_DESCRIPTION("TI BQ40Z50 Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");

