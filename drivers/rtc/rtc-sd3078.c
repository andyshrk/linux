/*
 *  Copyright (C) 2022 shenzhen wave
 *  version:v1.0.2--2022-12-20
 *  GNU General Public License version 2
 *
 *  Driver for sd3078 RTC
 */

#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/err.h>
#include "rtc-sd3078.h"

/*time reg*/
#define SD3078_REG_SEC			0x00
#define SD3078_REG_MIN			0x01
#define SD3078_REG_HOUR			0x02
#define SD3078_REG_WDAY			0x03
#define SD3078_REG_MDAY			0x04
#define SD3078_REG_MON			0x05
#define SD3078_REG_YEAR			0x06

/*alarm reg*/
#define SD3078_REG_ALARM_SEC			0x07
#define SD3078_REG_ALARM_MIN			0x08
#define SD3078_REG_ALARM_HOUR			0x09
#define SD3078_REG_ALARM_WEEK			0x0A
#define SD3078_REG_ALARM_DAY			0x0B
#define SD3078_REG_ALARM_MONTH			0x0C
#define SD3078_REG_ALARM_YEAR			0x0D
#define SD3078_REG_ALARM_OE				0x0E

/* control reg*/
#define SD3078_REG_CTR1					0x0F
#define SD3078_REG_CTR2					0x10
#define SD3078_REG_CTR3					0x11
#define SD3078_REG_TTF					0x12
#define SD3078_REG_TD0					0x13
#define SD3078_REG_TD1					0x14
#define SD3078_REG_TD2					0x15

/*temperature reg*/
#define SD3078_REG_TEMP					0x16

/*i2c control reg*/
#define SD3078_REG_I2C_CTRL				0x17
/*charge reg*/
#define SD3078_REG_CHARGE				0x18

/*extend control reg*/
#define SD3078_REG_CTR4					0x19
#define SD3078_REG_CTR5					0x1A
/*battery voltage reg*/
#define SD3078_REG_BAT_VAL				0x1B

/*temperature low/hign alarm reg*/
#define SD3078_REG_TEMP_AL				0x1C
#define SD3078_REG_TEMP_AH				0x1D

/*history max/min temperature reg*/
#define SD3078_REG_HIS_L				0x1E
#define SD3078_REG_HIS_H				0x1F

/*history temperature lowest  time ram*/
#define SD3078_REG_HIS_L_MIN			0x20
#define SD3078_REG_HIS_L_HOUR			0x21
#define SD3078_REG_HIS_L_WEEK			0x22
#define SD3078_REG_HIS_L_DAY			0x23
#define SD3078_REG_HIS_L_MON			0x24
#define SD3078_REG_HIS_L_YEAR			0x25

/*history temperature highest  time ram*/
#define SD3078_REG_HIS_H_MIN			0x26
#define SD3078_REG_HIS_H_HOUR			0x27
#define SD3078_REG_HIS_H_WEEK			0x28
#define SD3078_REG_HIS_H_DAY			0x29
#define SD3078_REG_HIS_H_MON			0x2A
#define SD3078_REG_HIS_H_YEAR			0x2B

/*user ram reg 2cH -71H*/
#define SD3078_REG_USER_RAM_START		0x2C
#define SD3078_REG_USER_RAM_END			0x71

/*device id 72H-79H*/
#define SD3078_REG_DEVICE_ID			0x72

/*reg bit define*/
#define SD3078_WRTC1					BIT(7)
#define SD3078_WRTC2					BIT(2)
#define SD3078_WRTC3					BIT(7)

#define SD3078_TEMP_AL_OE				BIT(2)  /*temperature lowest alarm enable*/
#define SD3078_TEMP_AH_OE				BIT(3)  /*temperature lowest alarm enable*/
#define SD3078_IM						BIT(6)
#define SD3078_INTS1					BIT(5)
#define SD3078_INTS0					BIT(4)
#define SD3078_ALARM_EN					BIT(7)
#define SD3078_INT_AF					BIT(5)
#define SD3078_INTFE					BIT(0)
#define SD3078_INTAE					BIT(1)
#define SD3078_INTDE					BIT(2)
#define SD3078_TDS1						BIT(5)
#define SD3078_TDS0						BIT(4)

#define  DRIVER_VER		" v1.0.2"

struct sd3078_data {
	struct i2c_client *client;
	struct rtc_device *rtc;
};

static int sd3078_set_alarm_ext(struct i2c_client *client, struct sd3078_alarm_ext *alarm_ext);

/**
 * @brief sd3078 read protect disable
 *
 */
static int sd3078_read_block_data(struct i2c_client *client, unsigned char reg,
				   unsigned char length, unsigned char *buf)
{
	int ret;

	struct i2c_msg msgs[] = {
		{/* setup read ptr */
			.addr = client->addr,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = buf
		},
	};
	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret != 2) {
		dev_err(&client->dev, "%s: read error=%d\n", __func__, ret);
		return -EIO;
	}

	return 0;
}

/**
 * @brief sd3078 write block
 *
 */
static int sd3078_write_block_data(struct i2c_client *client,
				unsigned char reg, unsigned char length,
				unsigned char *buf)
{
	unsigned char temp[0x80];
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.len = length+1,
			.buf = temp,
		}
	};

	if(length >=0x80)
	{
		return -EIO;
	}

	temp[0] = reg;
	memcpy(&temp[1], buf, length);
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret != 1) {
		dev_err(&client->dev, "%s: write error\n", __func__);
		return -EIO;
	}

	return 0;
}

/**
 * @brief sd3078 write protect enable
 *
 */
static int sd3078_write_enable(struct i2c_client *client)
{
	unsigned char buf[2];
	int ret;

	ret = sd3078_read_block_data(client, SD3078_REG_CTR1, 2, buf);
	if (ret != 0)
		return ret < 0 ? ret : -EIO;

	buf[1] |=  SD3078_WRTC1;
	ret = sd3078_write_block_data(client, SD3078_REG_CTR2,1,  &buf[1]);
	if (ret != 0)
		return ret < 0 ? ret : -EIO;

	buf[0] |=  SD3078_WRTC2;
	buf[0] |=  SD3078_WRTC3;
	ret = sd3078_write_block_data(client, SD3078_REG_CTR1, 1, &buf[0]);
	if (ret != 0)
		return ret < 0 ? ret : -EIO;

	return 0;
}

/**
 * @brief sd3078 write protect disable
 *
 */
static int sd3078_write_disable(struct i2c_client *client)
{
	unsigned char buf[2];
	int ret;

	ret = sd3078_read_block_data(client, SD3078_REG_CTR1, 2, buf);
	if (ret != 0)
		return ret < 0 ? ret : -EIO;

	buf[0] &=  ~SD3078_WRTC2;
	buf[0] &=  ~SD3078_WRTC3;
	buf[1] &=  ~SD3078_WRTC1;

	ret = sd3078_write_block_data(client, SD3078_REG_CTR1, 2, buf);
	if (ret != 0)
		return ret < 0 ? ret : -EIO;
	return 0;
}

/**
 * @brief sd3078 get time
 *
 */
static int sd3078_get_time(struct device *dev, struct rtc_time *dt)
{
	struct sd3078_data *sd3078 = dev_get_drvdata(dev);
	int ret;
	unsigned char date[7];

	ret = sd3078_read_block_data(sd3078->client, SD3078_REG_SEC,7, date);
	if (ret != 0)
	{
		return ret < 0 ? ret : -EIO;
	}

	dt->tm_sec = bcd2bin(date[SD3078_REG_SEC - SD3078_REG_SEC] & 0x7f);
	dt->tm_min = bcd2bin(date[SD3078_REG_MIN - SD3078_REG_SEC] & 0x7f);
	dt->tm_hour = bcd2bin(date[SD3078_REG_HOUR - SD3078_REG_SEC] & 0x3f);
	dt->tm_wday = bcd2bin(date[SD3078_REG_WDAY - SD3078_REG_SEC] & 0x03);
	dt->tm_mday = bcd2bin(date[SD3078_REG_MDAY - SD3078_REG_SEC] & 0x3f);
	dt->tm_mon = bcd2bin(date[SD3078_REG_MON - SD3078_REG_SEC] & 0x1f) - 1;
	dt->tm_year = bcd2bin(date[SD3078_REG_YEAR - SD3078_REG_SEC]) + 100;

	return 0;
}

/**
 * @brief sd3078 set time
 *
 */
static int sd3078_set_time(struct device *dev, struct rtc_time *dt)
{
	struct sd3078_data *sd3078 = dev_get_drvdata(dev);
	unsigned char date[7];
	int ret;

	if ((dt->tm_year < 100) || (dt->tm_year > 199))
		return -EINVAL;
	date[SD3078_REG_SEC - SD3078_REG_SEC] = bin2bcd(dt->tm_sec);
	date[SD3078_REG_MIN - SD3078_REG_SEC] = bin2bcd(dt->tm_min);
	date[SD3078_REG_HOUR - SD3078_REG_SEC] = bin2bcd(dt->tm_hour)  |  0x80;
	date[SD3078_REG_WDAY - SD3078_REG_SEC] = bin2bcd( dt->tm_wday);
	date[SD3078_REG_MDAY - SD3078_REG_SEC] = bin2bcd(dt->tm_mday);
	date[SD3078_REG_MON - SD3078_REG_SEC] = bin2bcd(dt->tm_mon + 1);
	date[SD3078_REG_YEAR - SD3078_REG_SEC] = bin2bcd(dt->tm_year - 100);

	sd3078_write_enable(sd3078->client);
	ret = sd3078_write_block_data(sd3078->client, SD3078_REG_SEC, 7, date);
	sd3078_write_disable(sd3078->client);
	if (ret != 0)
		return ret < 0 ? ret : -EIO;

	return 0;
}

static int sd3078_read_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	unsigned char buf[8];
	int ret;
	struct sd3078_data *sd3078 = dev_get_drvdata(dev);

	ret = sd3078_read_block_data(sd3078->client, SD3078_REG_ALARM_SEC,8, buf);
	if (ret != 0)
		return ret < 0 ? ret : -EIO;

	tm->time.tm_sec = buf[0];
	tm->time.tm_min = buf[1];
	tm->time.tm_hour = buf[2];
	tm->time.tm_wday = buf[3];
	tm->time.tm_yday = buf[4];
	tm->time.tm_mon = buf[5];
	tm->time.tm_year = buf[6];

	if((buf[7] & 0x7f) != 0)
	{
		tm->enabled = 1;
	}
	ret = sd3078_read_block_data(sd3078->client, SD3078_REG_CTR1,1, &buf[0]);
	if (ret != 0)
		return ret < 0 ? ret : -EIO;

	if((buf[0] & SD3078_INT_AF) != 0)
	{
		tm->pending = 1;
	}
	else
	{
		tm->pending = 0;
	}
	return 0;
}

static int sd3078_set_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	int ret;
	struct sd3078_data *sd3078 = dev_get_drvdata(dev);
	struct sd3078_alarm_ext alarm_ext;

	alarm_ext.sec_a = tm->time.tm_sec ;
	alarm_ext.min_a = tm->time.tm_min ;
	alarm_ext.hour_a = tm->time.tm_hour;
	alarm_ext.week_a = tm->time.tm_wday;
	alarm_ext.day_a = tm->time.tm_yday;
	alarm_ext.mon_a = tm->time.tm_mon;
	alarm_ext.year_a = tm->time.tm_year;

	if(tm->enabled == 1)
	{
		alarm_ext.oe_a = 0x7f;
	}
	else
	{
		alarm_ext.oe_a = 0;
	}
	alarm_ext.ie_a = 1;
	alarm_ext.int_period = 0;
	ret = sd3078_set_alarm_ext(sd3078->client, &alarm_ext);
	return ret;
}

/**
 * @brief  get sd3078 chip temperature
 *
 */
static int sd3078_get_temperature(	struct i2c_client *client, int *temp,  unsigned int cmd)
{
	int ret;
	unsigned reg;
	unsigned char buf;
	if(cmd == RTC_SD3078_RD_TEMP)
		reg = SD3078_REG_TEMP;
	else if(cmd == RTC_SD3078_TEMP_HIS_L)
		reg = SD3078_REG_HIS_L;
	else if(cmd == RTC_SD3078_TEMP_HIS_H)
		reg = SD3078_REG_HIS_H;
	else
		return  -EIO;

	ret = sd3078_read_block_data(client, reg,1, &buf);
	if (ret != 0)
		return ret < 0 ? ret : -EIO;
	*temp = (signed  char)buf;
	return 0;
}

/**
 * @brief   sd3078 set when  power supply by battery i2c work state,
 * en:0,battery supply i2c is not work,1, battery supply i2c is  work
 *
 */
static int sd3078_battery_i2c_ctrl(struct i2c_client *client, int en)
{
	int ret;
	unsigned char buf;

	if(en !=0 &&en !=1)
		return  -EIO;
	if(en)
		buf = 0x80;
	else
		buf =0x00;

	sd3078_write_enable(client);
	ret = sd3078_write_block_data(client, SD3078_REG_I2C_CTRL, 1, &buf);
	sd3078_write_disable(client);

	if (ret != 0)
		return ret < 0 ? ret : -EIO;
	return 0;
}

/**
 * @brief   sd3078 read battery voltage
 *
 */
static int sd3078_read_battery_voltage(	struct i2c_client *client, unsigned int *vol)
{
	int ret;
	unsigned char buf[2];

	ret = sd3078_read_block_data(client, SD3078_REG_CTR5, 2, buf);

	if (ret != 0)
		return ret < 0 ? ret : -EIO;

	*vol = ((buf[0]& 0x80 )<<1) | buf[1] ;
	return 0;
}

/**
 * @brief   sd3078 read battery charge control
 *
 */
static int sd3078_battery_charge_ctrl(struct i2c_client *client, struct sd3078_charge *ctrl)
{
	unsigned char buf;
	int ret;

	if (ctrl->chage_en > 1)
		return -EIO;
	if (ctrl->resistance > 3)
		return -EIO;
	buf = (ctrl->chage_en <<7) | ctrl->resistance;

	sd3078_write_enable(client);
	ret = sd3078_write_block_data(client, SD3078_REG_CHARGE, 1, &buf);
	sd3078_write_disable(client);

	if (ret != 0)
		return ret < 0 ? ret : -EIO;
	return 0;
}

/**
 * @brief   sd3078 set temperature alarm value;
 * cmd:0:set lowest temperature alarm;1:set highest temperature alarm
 *
 */
static int sd3078_set_alarm_temp(struct i2c_client *client,  struct sd3078_temp_alarm *temp,  unsigned int cmd)
{
	unsigned char buf;
	int ret;
	unsigned char reg;
	unsigned char oe_bit;
	if(cmd == RTC_SD3078_TEMP_AL)
	{
		reg = SD3078_REG_TEMP_AL;
		oe_bit = SD3078_TEMP_AL_OE;
	}

	else if(cmd == RTC_SD3078_TEMP_AH)
	{
		reg = SD3078_REG_TEMP_AH;
		oe_bit = SD3078_TEMP_AH_OE;
	}
	else
		return -EIO;

	sd3078_write_enable(client);
	buf = (unsigned char)temp->temp;
	sd3078_write_block_data(client, reg, 1, &buf);

	ret = sd3078_read_block_data(client, SD3078_REG_CTR4, 1, &buf);
	if (ret != 0)
		return ret < 0 ? ret : -EIO;

	/*enable temperature alarm*/
	if (temp->oe == 1)
	{
		buf  |= oe_bit;
	}
	else
	{
		buf &= (~oe_bit);
	}

	ret = sd3078_write_block_data(client, SD3078_REG_CTR4, 1, &buf);
	sd3078_write_disable(client);

	if (ret != 0)
		return ret < 0 ? ret : -EIO;
	return 0;
}

/**
 * @brief sd3078 historical minimum / maximum temperature occurrence time
 *
 */
static int sd3078_get_his_temp_time(struct i2c_client *client, struct rtc_time *dt, unsigned int cmd)
{
	int ret;
	unsigned char date[6];
	unsigned char reg;

	if(cmd == RTC_SD3078_TEMP_HIS_L_T)
		reg = SD3078_REG_HIS_L_MIN;
	else if(cmd == RTC_SD3078_TEMP_HIS_H_T)
		reg = SD3078_REG_HIS_H_MIN;
	else
		return -EIO;

	ret = sd3078_read_block_data(client, reg, 6, date);
	if (ret != 0)
		return ret < 0 ? ret : -EIO;

	dt->tm_sec = 0;
	dt->tm_min = bcd2bin(date[SD3078_REG_MIN - SD3078_REG_MIN] & 0x7f);
	dt->tm_hour = bcd2bin(date[SD3078_REG_HOUR - SD3078_REG_MIN] & 0x1f);
	dt->tm_wday =   bcd2bin(date[SD3078_REG_WDAY - SD3078_REG_MIN] & 0x03);
	dt->tm_mday = bcd2bin(date[SD3078_REG_MDAY - SD3078_REG_MIN] & 0x3f);
	dt->tm_mon = bcd2bin(date[SD3078_REG_MON - SD3078_REG_MIN] & 0x1f) ;
	dt->tm_year = bcd2bin(date[SD3078_REG_YEAR - SD3078_REG_MIN]) + 100;

	return 0;
}

/**
 * @brief   sd3078 read user ram;
 *
 */
static int sd3078_read_ram(struct i2c_client *client,  unsigned char *buf, struct sd3078_ram *ram)
{
	int ret;

	if(ram->st_addr < SD3078_REG_USER_RAM_START ||
		ram->end_addr > SD3078_REG_USER_RAM_END ||
		ram->end_addr < ram->st_addr)
		return -EIO;

	ret = sd3078_read_block_data(client, ram->st_addr,  ram->end_addr - ram->st_addr + 1,   buf);
	if (ret != 0)
	{
		return ret ;
	}
	return 0;
}

/**
 * @brief   sd3078 write user ram;
 *
 */
static int sd3078_write_ram(struct i2c_client *client, unsigned char *buf, struct sd3078_ram *ram)
{
	int ret;

	if(ram->st_addr < SD3078_REG_USER_RAM_START ||
		ram->end_addr > SD3078_REG_USER_RAM_END ||
		ram->end_addr < ram->st_addr)
		return -EIO;

	sd3078_write_enable(client);
	ret = sd3078_write_block_data(client, ram->st_addr, ram->end_addr - ram->st_addr + 1, buf);
	sd3078_write_disable(client);

	if (ret != 0)
		return ret < 0 ? ret : -EIO;
	return 0;
}

/**
 * @brief   sd3078 read device id;
 *
 */
static int sd3078_read_device_id(struct i2c_client *client, unsigned char *buf)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client,  SD3078_REG_DEVICE_ID,
					   8, buf);
	if (ret != 8)
		return ret < 0 ? ret : -EIO;
	ret = sd3078_read_block_data(client, SD3078_REG_DEVICE_ID, 8, buf);
	if (ret != 0)
		return ret < 0 ? ret : -EIO;
	return 0;
}

/**
 * @brief   sd3078 set alarm extend;
 *
 */
static int sd3078_set_alarm_ext(struct i2c_client *client, struct sd3078_alarm_ext *alarm_ext)
{
	int ret;
	unsigned char buf[8];

	buf[0] = bin2bcd(alarm_ext->sec_a);
	buf[1] = bin2bcd(alarm_ext->min_a);
	buf[2] = bin2bcd(alarm_ext->hour_a);
	buf[3] = bin2bcd(alarm_ext->week_a);
	buf[4] = bin2bcd(alarm_ext->day_a);
	buf[5] = bin2bcd(alarm_ext->mon_a);
	buf[6] = bin2bcd(alarm_ext->year_a);
	buf[7] = alarm_ext->oe_a;
	ret = sd3078_write_enable(client);
	if (ret != 0)
	{
		return ret < 0 ? ret : -EIO;
	}
	ret = sd3078_write_block_data(client, SD3078_REG_ALARM_SEC, 8, buf);
	if (ret != 0)
	{
		sd3078_write_disable(client);
		return ret < 0 ? ret : -EIO;
	}

	/*if enable interrupt out?*/
	ret = sd3078_read_block_data(client, SD3078_REG_CTR2,1, &buf[0]);
	if (ret != 0)
	{
		sd3078_write_disable(client);
		return ret < 0 ? ret : -EIO;
	}
	buf[0] = buf[0];
	buf[0] &= ~SD3078_INTS1;
	buf[0] |= SD3078_INTS0;
	if(alarm_ext->ie_a == 1)
	{
		buf[0] |= SD3078_INTAE;
	}
	else
	{
		buf[0] &= ~SD3078_INTAE;
	}
	if(alarm_ext->int_period == 1)
	{
		buf[0] |= SD3078_IM;
	}
	else
	{
		buf[0] &= ~SD3078_IM;
	}
	ret = sd3078_write_block_data(client, SD3078_REG_CTR2, 1, &buf[0]);
	if (ret != 0)
	{
		sd3078_write_disable(client);
		return ret < 0 ? ret : -EIO;
	}
	sd3078_write_disable(client);
	return 0;
}

/**
 * @brief   sd3078 clock out
 *
 */
static int sd3078_clk_out(struct i2c_client *client,  struct sd3078_clk_out *clk)
{
	int ret;
	unsigned char buf[2];

	if(clk->freq > RTC_3078_CLK_OUT_1_SEC)
		return -EIO;

	ret = sd3078_write_enable(client);
	if (ret != 0)
	{
		return ret < 0 ? ret : -EIO;
	}
	ret = sd3078_read_block_data(client, SD3078_REG_CTR2, 2, buf);
	if (ret != 0)
	{
		sd3078_write_disable(client);
		return ret < 0 ? ret : -EIO;
	}

	buf[1] &= 0xf0;
	buf[1] |= clk->freq;
	if(clk->oe == 1)
	{
		buf[0] |= SD3078_INTFE;
	}
	else
	{
		buf[0] &= ~SD3078_INTFE;
	}
	buf[0] |= SD3078_INTS1;
	buf[0] &= ~SD3078_INTS0;

	ret = sd3078_write_block_data(client, SD3078_REG_CTR2, 2, buf);
	sd3078_write_disable(client);
	if (ret != 0)
	{
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

/**
 * @brief   sd3078  count down
 *
 */
static int sd3078_count_down(struct i2c_client *client,  struct sd3078_count_down *count_down)
{
	int ret;
	unsigned char buf[3];

	if(count_down->count > 0xffffff)
		return -EIO;
	if(count_down->source > RTC_3078_COUNT_DOWN_1_60)
		return -EIO;


	ret = sd3078_write_enable(client);
	if (ret != 0)
	{
		return ret < 0 ? ret : -EIO;
	}
	ret = sd3078_read_block_data(client, SD3078_REG_CTR2, 2, buf);
	if (ret != 0)
	{
		sd3078_write_disable(client);
		return ret < 0 ? ret : -EIO;
	}

	/*set INTDF =0*/
	buf[0] &= ~SD3078_INTDE;
	ret = sd3078_write_block_data(client, SD3078_REG_CTR2, 1, &buf[0]);
	if (ret != 0)
	{
		sd3078_write_disable(client);
		return ret < 0 ? ret : -EIO;
	}

	if (count_down->ie == 1)
	{
		buf[0]  |= SD3078_INTDE;
		buf[0] |= SD3078_INTS1;
		buf[0] |= SD3078_INTS0;
	}

	if (count_down->int_period == 1)
	{
		buf[0] |= SD3078_IM;
	}
	else
	{
		buf[0] &= ~SD3078_IM;
	}
	buf[1] &=  ~(0x03 << 4);
	buf[1] |=  (count_down->source  << 4);
	ret = sd3078_write_block_data(client, SD3078_REG_CTR2, 2, buf);
	if (ret != 0)
	{
		sd3078_write_disable(client);
		return ret < 0 ? ret : -EIO;
	}
	buf[0] = count_down->count & 0xff;
	buf[1] = (count_down->count >> 8 )& 0xff;
	buf[2] = (count_down->count >> 16 )& 0xff;
	ret = sd3078_write_block_data(client, SD3078_REG_TD0, 3, buf);
	sd3078_write_disable(client);
	if (ret != 0)
	{
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}
/**
 * @brief   sd3078 io ctrl;
 *
 */
static int sd3078_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct sd3078_data *sd3078 = dev_get_drvdata(dev);
	int ret;

	switch (cmd) {
		case RTC_SD3078_RD_TEMP:  //read chip temperature
		case RTC_SD3078_TEMP_HIS_L:  //history lowest temperature  value
		case RTC_SD3078_TEMP_HIS_H :  //history highest temperature  value
		{
			int temp;

			ret = sd3078_get_temperature(sd3078->client, &temp, cmd);
			if (ret != 0)
				return  ret;
			if (copy_to_user((void __user *)arg, &temp, sizeof(int)))
			return -EFAULT;
		}
		break;
		case RTC_SD3078_BAT_I2C:  //battery  i2c ctrl
		{
			ret = sd3078_battery_i2c_ctrl(sd3078->client,  arg);
			if (ret != 0)
			{
				return  ret;
			}
		}
		break;
		case RTC_SD3078_BAT_VOL:  //battery voltage read
		{
			unsigned int vol;

			if(sd3078_read_battery_voltage(sd3078->client, &vol) == 0)
			{
				if (copy_to_user((void __user *)arg, &vol, sizeof(vol)))
					return -EFAULT;
			}
			else
			{
				return -EFAULT;
			}
		}
		break;
		case RTC_SD3078_EN_CHARGE:  //battery charge control
		{
			struct sd3078_charge ctrl;

			if (copy_from_user(&ctrl, (void __user *)arg, sizeof(ctrl )))
				return -EFAULT;
			if (sd3078_battery_charge_ctrl(sd3078->client, &ctrl) != 0)
				return -EFAULT;
		}
		break;
		case RTC_SD3078_TEMP_AL :  //lowest temperature  alarm
		case RTC_SD3078_TEMP_AH :  //highest temperature  alarm
		{
			struct sd3078_temp_alarm alarm;

			if (copy_from_user(&alarm, (void __user *)arg, sizeof(alarm)))
				return -EFAULT;
			if (sd3078_set_alarm_temp(sd3078->client, &alarm, cmd) != 0)
				return -EFAULT;
		}
		break;
		case RTC_SD3078_TEMP_HIS_L_T :  //the occurrence time of the lowest temperature in history
		case RTC_SD3078_TEMP_HIS_H_T :  //the occurrence time of the highest temperature in history
		{
			struct rtc_time dt;

			if(sd3078_get_his_temp_time(sd3078->client, &dt, cmd) == 0)
			{
				if (copy_to_user((void __user *)arg, &dt, sizeof(dt)))
					return -EFAULT;
			}
			else
				return -EFAULT;

		}
		break;
		case RTC_SD3078_RAM_RD:  //user ram read
		{
			struct sd3078_ram ram_r;
			unsigned char buf_r[SD3078_REG_USER_RAM_END - SD3078_REG_USER_RAM_START + 1];

			if (copy_from_user(&ram_r, (void __user *)arg, sizeof(ram_r )))
				return -EFAULT;
			if (sd3078_read_ram(sd3078->client, buf_r, &ram_r) == 0)
			{
				if (copy_to_user((void __user *)ram_r.buf, buf_r, ram_r.end_addr -  ram_r.st_addr  +1))
					return -EFAULT;
			}
			else
			{
					return -EFAULT;
			}
		}
		break;
		case RTC_SD3078_RAM_WR:  //user ram write
		{
			struct sd3078_ram ram_w;
			unsigned char buf_w[SD3078_REG_USER_RAM_END - SD3078_REG_USER_RAM_START + 1];

			if (copy_from_user(&ram_w, (void __user *)arg, sizeof(ram_w )))
				return -EFAULT;
			if (copy_from_user(buf_w, ram_w.buf, ram_w.end_addr -  ram_w.st_addr  +1))
				return -EFAULT;
			if (sd3078_write_ram(sd3078->client, buf_w, &ram_w) != 0)
				return -EFAULT;
		}
		break;
		case RTC_SD3078_DEVICE_ID:  //read device id
		{
			unsigned char device_id[8];

			if (sd3078_read_device_id(sd3078->client, device_id) == 0)
			{
				if (copy_to_user((void __user *)arg, device_id, sizeof(device_id)))
					return -EFAULT;
			}
			else
			{
				return -EFAULT;
			}

		}
		break;
		case RTC_SD3078_ALARM_EXT_SET:  /*alarm extend set*/
		{
			struct sd3078_alarm_ext alarm_ext;

			if (copy_from_user(&alarm_ext, (void __user *)arg, sizeof(alarm_ext )))
				return -EFAULT;
			if (sd3078_set_alarm_ext(sd3078->client, &alarm_ext) != 0)
				return -EFAULT;
		}
		break;
		case RTC_SD3078_CLK_OUT:  /*freq output ctrl*/
		{
			 struct sd3078_clk_out clk;

			if (copy_from_user(&clk, (void __user *)arg, sizeof(clk )))
				return -EFAULT;
			if (sd3078_clk_out(sd3078->client, &clk) != 0)
				return -EFAULT;
		}
		break;
		case RTC_SD3078_COUNT_DOWN_SET:
		{
			struct sd3078_count_down count_down;

			if (copy_from_user(&count_down, (void __user *)arg, sizeof(count_down )))
				return -EFAULT;
			if (sd3078_count_down(sd3078->client,  &count_down) != 0)
				return -EFAULT;
		}
			break;
		default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

static struct rtc_class_ops sd3078_rtc_ops = {
	.read_time = sd3078_get_time,
	.set_time = sd3078_set_time,
	.set_alarm = sd3078_set_alarm,
	.read_alarm = sd3078_read_alarm,
	.ioctl = sd3078_ioctl,
};

static irqreturn_t sd3078_irq_thread_handler(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct sd3078_data *sd3078 = i2c_get_clientdata(client);

	mutex_lock(&sd3078->rtc->ops_lock);
	/*user code*/
	mutex_unlock(&sd3078->rtc->ops_lock);
	return IRQ_HANDLED;
}

static int sd3078_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct sd3078_data *sd3078;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA
		| I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&adapter->dev, "doesn't support required functionality\n");
		return -EIO;
	}

	sd3078 = devm_kzalloc(&client->dev, sizeof(struct sd3078_data),
				GFP_KERNEL);
	if (!sd3078)
		return -ENOMEM;

	sd3078->client = client;
	i2c_set_clientdata(client, sd3078);

	sd3078->rtc = devm_rtc_device_register(&client->dev, client->name,
			&sd3078_rtc_ops, THIS_MODULE);
	if (IS_ERR(sd3078->rtc)) {
			dev_err(&client->dev, "unable to register the class device\n");
			return PTR_ERR(sd3078->rtc);
	}

	if (client->irq > 0) {
		dev_info(&client->dev, "IRQ %d supplied\n", client->irq);
		err = devm_request_threaded_irq(&client->dev, client->irq, NULL,
						sd3078_irq_thread_handler,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						"sd3078", client);
		if (err) {
			dev_err(&client->dev, "unable to request IRQ\n");
			client->irq = 0;
		}
	}

	sd3078->rtc->max_user_freq = 1;
	sd3078->rtc->uie_unsupported = 1;

	dev_info(&client->dev, "sd3078 probe!,version is %s\n", DRIVER_VER);

	return err;
}

static const struct i2c_device_id sd3078_id[] = {
	{ "sd3078", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c,sd3078_id);

static const struct of_device_id sd3078_of_match[] = {
	{ .compatible = "wave,sd3078" },
	{ }
};

static struct i2c_driver sd3078_driver = {
	.driver = {
		.name = "rtc-sd3078",
		.of_match_table = of_match_ptr(sd3078_of_match),
	},
	.probe		= sd3078_probe,
	.id_table	= sd3078_id,
};

MODULE_DEVICE_TABLE(of, sd3078_of_match);

module_i2c_driver(sd3078_driver);

MODULE_AUTHOR("support@whwave.com.cn");
MODULE_DESCRIPTION("wave sd3078 rtc driver");
MODULE_LICENSE("GPL v2");
