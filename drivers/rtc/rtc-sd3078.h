/*
 *  Copyright (C) 2022 shenzhen wave
 *  version:v1.0.2--2022-12-20
 *  GNU General Public License version 2
 *
 *  Header for sd3078 RTC
 */

#ifndef  __RTC_SD3078__H
#define  __RTC_SD3078__H
#include <linux/ioctl.h>

/*clk output frequency define*/
#define RTC_3078_CLK_OUT_4096		0x02
#define RTC_3078_CLK_OUT_1024		0x03
#define RTC_3078_CLK_OUT_64			0x04
#define RTC_3078_CLK_OUT_32			0x05
#define RTC_3078_CLK_OUT_16			0x06
#define RTC_3078_CLK_OUT_8			0x07
#define RTC_3078_CLK_OUT_4			0x08
#define RTC_3078_CLK_OUT_2			0x09
#define RTC_3078_CLK_OUT_1			0x0a
#define RTC_3078_CLK_OUT_1_2		0x0b
#define RTC_3078_CLK_OUT_1_4		0x0c
#define RTC_3078_CLK_OUT_1_8		0x0d
#define RTC_3078_CLK_OUT_1_16		0x0e
#define RTC_3078_CLK_OUT_1_SEC		0x0f

/*conut down frequency source define*/
#define RTC_3078_COUNT_DOWN_4096	0x00
#define RTC_3078_COUNT_DOWN_1024	0x01
#define RTC_3078_COUNT_DOWN_1		0x02
#define RTC_3078_COUNT_DOWN_1_60	0x03

/*charge control struct*/
 struct sd3078_charge
{
	unsigned char chage_en; /*0:disable,1:enable*/
	unsigned char resistance; /*00:10k;01:5k;10:2k;11:open*/
};

/*user ram write/read struct*/
struct sd3078_ram
{
	unsigned char st_addr;
	unsigned char end_addr;
	unsigned char *buf;
};

/*temperature lowest/highest alarm set*/
struct sd3078_temp_alarm
{
	unsigned char type; /*0:lowest alarm,1:highest alarm*/
	unsigned char oe; /*temperature alarm set,0:disable,1:enable*/
	unsigned char ie; /*interrupt enable,when ie=1 and alarm happen,int pin output 0*/
	int temp;
};

/*time alarm extend struct*/
struct sd3078_alarm_ext
{
	unsigned char sec_a;
	unsigned char min_a;
	unsigned char hour_a;
	unsigned char week_a;
	unsigned char day_a;
	unsigned char mon_a;
	unsigned char year_a;

	unsigned char ie_a; /*interrupt enable ,0:disable,1:enable*/
	unsigned char int_period; /*0:int outpute 0,1:int output pulse*/
	unsigned char oe_a; /*alarm enable;bit[6]:year,bit[5]:month,bit[4]:day,bit[3]:week,bit[2]:hour,bit[1]:minute,bit[0]:second*/
};

/*frequency output struct*/
struct sd3078_clk_out
{
	 unsigned char freq;
	 unsigned char oe; /*= clk out enable, set,0:disable,1:enable*/
};
/*count down struct*/
struct sd3078_count_down
{
	unsigned int count;
	unsigned char source;
	unsigned char ie; /*interrupt enable ,0:disable,1:enable*/
	unsigned char int_period; /*0:int outpute 0,1:int output pulse*/
};

/*SD3078 io ctrl cmd，start with rtc.h ioctrl */
#define RTC_SD3078_RD_TEMP					_IOR('p', 0x15, int)     /*read chip temperature*/
#define RTC_SD3078_BAT_I2C					_IOW('p', 0x16, unsigned char)     /*battery  i2c ctrl*/
#define RTC_SD3078_BAT_VOL					_IOR('p', 0x17, unsigned int)     /*battery voltage read*/
#define RTC_SD3078_EN_CHARGE				_IOW('p', 0x18, struct sd3078_charge)      /*battery charge control*/
#define RTC_SD3078_TEMP_AL					_IOW('p', 0x19, struct sd3078_temp_alarm)   /*set lowest temperature  alarm  value*/
#define RTC_SD3078_TEMP_AH					_IOW('p', 0x1a,  struct sd3078_temp_alarm)    /*set highest temperature  alarm value*/
#define RTC_SD3078_TEMP_HIS_L				_IOR('p', 0x1b, signed char)   /*history lowest temperature  value*/
#define RTC_SD3078_TEMP_HIS_H				_IOR('p', 0x1c, signed char)  /*history highest temperature  value*/
#define RTC_SD3078_TEMP_HIS_L_T				_IOR('p', 0x1d, struct rtc_time)    /*the occurrence time of the lowest temperature in history*/
#define RTC_SD3078_TEMP_HIS_H_T				_IOR('p', 0x1e, struct rtc_time)  /*the occurrence time of the highest temperature in history*/
#define RTC_SD3078_RAM_RD					_IOW('p', 0x1f,  struct sd3078_ram)   /*user ram read*/
#define RTC_SD3078_RAM_WR					_IOW('p', 0x20,  struct sd3078_ram)   /*user ram write*/
#define RTC_SD3078_DEVICE_ID				_IOR('p', 0x21, int)   /*read device id*/
#define RTC_SD3078_ALARM_EXT_SET			_IOW('p', 0x22, struct sd3078_alarm_ext)    /*alarm extend set*/
#define RTC_SD3078_CLK_OUT					_IOW('p', 0x23, signed char)    /*freq output ctrl*/
#define RTC_SD3078_COUNT_DOWN_SET			_IOW('p', 0x24, signed char)    /*count down set*/

#endif
