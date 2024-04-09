/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */

#include "ar0231_raw.h"

#define ADDR_MAX9295	0x62
#define ADDR_AR0231_SENSOR	0x10
#define ADDR_MAX9295_LINKA	0x41
#define ADDR_MAX9295_LINKB	0x42
#define ADDR_MAX9295_LINKC	0x43
#define ADDR_MAX9295_LINKD	0x44
#define ADDR_MAX9295_VIRTUAL	0x45
#define ADDR_AR0231_LINKA	0x11
#define ADDR_AR0231_LINKB	0x12
#define ADDR_AR0231_LINKC	0x13
#define ADDR_AR0231_LINKD	0x14

#define SENSOR_REGADDR_CHIP_ID_REG  0x3000
#define SENSOR_REGADDR_CHIP_VERSION_REG 0x300E
#define MAX9295_REGADDR_DEV_ID	0x000D

#define AR0231_MAX_WIDTH    1928
#define AR0231_MAX_HEIGHT   1208
#define AR0231_SENSOR_WIDTH 1920
#define AR0231_SENSOR_HEIGHT    1080
#define AR0231_X_START  ((AR0231_SENSOR_WIDTH - AR0231_MAX_WIDTH) / 2)
#define AR0231_Y_START  ((AR0231_SENSOR_HEIGHT - AR0231_MAX_HEIGHT) / 2)
#define AR0231_X_END    (AR0231_X_START + AR0231_MAX_WIDTH - 1)
#define AR0231_Y_END    (AR0231_Y_START + AR0231_MAX_HEIGHT - 1)
#define TABLE_END	0xFFFF

struct reg_value16 ar0231_mode_1920X1080_12bit_linear_30fps[] = {
	{ 0x301A, 0x0018, 30 },
	{ 0x3056, 0x0080, 0 },
	{ 0x3058, 0x0080, 0 },
	{ 0x305A, 0x0080, 0 },
	{ 0x305C, 0x0080, 0 },
	{ 0x3138, 0x000B, 0 },
	{ 0x30FE, 0x0020, 0 },
	{ 0x3372, 0xF54F, 0 },
	{ 0x337A, 0x0D70, 0 },
	{ 0x337E, 0x1FFD, 0 },
	{ 0x3382, 0x00C0, 0 },
	{ 0x3092, 0x0024, 0 },
	{ 0x3C04, 0x0E80, 0 },
	{ 0x3F90, 0x06E1, 0 },
	{ 0x3F92, 0x06E1, 0 },
	{ 0x350E, 0xFF14, 0 },
	{ 0x3506, 0x4444, 0 },
	{ 0x3508, 0x4444, 0 },
	{ 0x350A, 0x4465, 0 },
	{ 0x350C, 0x055F, 0 },
	{ 0x30BA, 0x11F2, 0 },
	{ 0x3566, 0x1D28, 0 },
	{ 0x3518, 0x1FFE, 0 },
	{ 0x318E, 0x0200, 0 },
	{ 0x3190, 0x5000, 0 },
	{ 0x319E, 0x6060, 0 },
	{ 0x3520, 0x4688, 0 },
	{ 0x3522, 0x8840, 0 },
	{ 0x3524, 0x4046, 0 },
	{ 0x352C, 0xC6C6, 0 },
	{ 0x352A, 0x089F, 0 },
	{ 0x352E, 0x0011, 5 },
	{ 0x3530, 0x4400, 5 },
	{ 0x3536, 0xFF06, 10 },
	{ 0x3538, 0xFFFF, 10 },
	{ 0x353A, 0x9000, 0 },
	{ 0x353C, 0x3F00, 10 },
	{ 0x32EC, 0x72A1, 0 },
	{ 0x3540, 0xC659, 5 },
	{ 0x3556, 0x101F, 0 },
	{ 0x3566, 0x1D28, 10 },
	{ 0x3566, 0x1128, 0 },
	{ 0x3566, 0x1328, 0 },
	{ 0x3566, 0x3328, 0 },
	{ 0x3528, 0xDDDD, 0 },
	{ 0x3540, 0xC63E, 0 },
	{ 0x3542, 0x545B, 0 },
	{ 0x3544, 0x645A, 0 },
	{ 0x3546, 0x5A5A, 0 },
	{ 0x3548, 0x6400, 0 },
	{ 0x301A, 0x10D8, 10 },
	{ 0x2512, 0x8000, 0 },
	{ 0x2510, 0x0905, 0 },
	{ 0x2510, 0x3350, 0 },
	{ 0x2510, 0x2004, 0 },
	{ 0x2510, 0x1460, 0 },
	{ 0x2510, 0x1578, 0 },
	{ 0x2510, 0x1360, 0 },
	{ 0x2510, 0x7B24, 0 },
	{ 0x2510, 0xFF24, 0 },
	{ 0x2510, 0xFF24, 0 },
	{ 0x2510, 0xEA24, 0 },
	{ 0x2510, 0x1022, 0 },
	{ 0x2510, 0x2410, 0 },
	{ 0x2510, 0x155A, 0 },
	{ 0x2510, 0x1342, 0 },
	{ 0x2510, 0x1400, 0 },
	{ 0x2510, 0x24FF, 5 },
	{ 0x2510, 0x24EA, 0 },
	{ 0x2510, 0x2324, 0 },
	{ 0x2510, 0x647A, 0 },
	{ 0x2510, 0x2404, 0 },
	{ 0x2510, 0x052C, 0 },
	{ 0x2510, 0x400A, 0 },
	{ 0x2510, 0xFF0A, 5 },
	{ 0x2510, 0x1808, 0 },
	{ 0x2510, 0x3851, 0 },
	{ 0x2510, 0x1440, 0 },
	{ 0x2510, 0x0004, 0 },
	{ 0x2510, 0x0801, 0 },
	{ 0x2510, 0x0408, 0 },
	{ 0x2510, 0x1180, 0 },
	{ 0x2510, 0x15DC, 0 },
	{ 0x2510, 0x134C, 0 },
	{ 0x2510, 0x1002, 0 },
	{ 0x2510, 0x1016, 0 },
	{ 0x2510, 0x1181, 0 },
	{ 0x2510, 0x1189, 0 },
	{ 0x2510, 0x1056, 0 },
	{ 0x2510, 0x1210, 0 },
	{ 0x2510, 0x0901, 0 },
	{ 0x2510, 0x0D08, 0 },
	{ 0x2510, 0x0913, 0 },
	{ 0x2510, 0x13C8, 0 },
	{ 0x2510, 0x092B, 0 },
	{ 0x2510, 0x1588, 0 },
	{ 0x2510, 0x0901, 0 },
	{ 0x2510, 0x1388, 0 },
	{ 0x2510, 0x0909, 0 },
	{ 0x2510, 0x11D9, 0 },
	{ 0x2510, 0x091D, 0 },
	{ 0x2510, 0x1441, 0 },
	{ 0x2510, 0x0903, 0 },
	{ 0x2510, 0x1214, 0 },
	{ 0x2510, 0x0901, 0 },
	{ 0x2510, 0x10D6, 0 },
	{ 0x2510, 0x1210, 0 },
	{ 0x2510, 0x1212, 0 },
	{ 0x2510, 0x1210, 0 },
	{ 0x2510, 0x11DD, 0 },
	{ 0x2510, 0x11D9, 0 },
	{ 0x2510, 0x1056, 0 },
	{ 0x2510, 0x0905, 0 },
	{ 0x2510, 0x11DB, 0 },
	{ 0x2510, 0x092B, 0 },
	{ 0x2510, 0x119B, 0 },
	{ 0x2510, 0x11BB, 0 },
	{ 0x2510, 0x121A, 0 },
	{ 0x2510, 0x1210, 0 },
	{ 0x2510, 0x1460, 0 },
	{ 0x2510, 0x1250, 0 },
	{ 0x2510, 0x1076, 0 },
	{ 0x2510, 0x10E6, 0 },
	{ 0x2510, 0x0901, 0 },
	{ 0x2510, 0x15AB, 0 },
	{ 0x2510, 0x0901, 0 },
	{ 0x2510, 0x13A8, 0 },
	{ 0x2510, 0x1240, 0 },
	{ 0x2510, 0x1260, 0 },
	{ 0x2510, 0x0923, 0 },
	{ 0x2510, 0x158D, 0 },
	{ 0x2510, 0x138D, 0 },
	{ 0x2510, 0x0901, 0 },
	{ 0x2510, 0x0B09, 0 },
	{ 0x2510, 0x0108, 0 },
	{ 0x2510, 0x0901, 0 },
	{ 0x2510, 0x1440, 0 },
	{ 0x2510, 0x091D, 0 },
	{ 0x2510, 0x1588, 0 },
	{ 0x2510, 0x1388, 0 },
	{ 0x2510, 0x092D, 0 },
	{ 0x2510, 0x1066, 0 },
	{ 0x2510, 0x0905, 0 },
	{ 0x2510, 0x0C08, 0 },
	{ 0x2510, 0x090B, 0 },
	{ 0x2510, 0x1441, 0 },
	{ 0x2510, 0x090D, 0 },
	{ 0x2510, 0x10E6, 0 },
	{ 0x2510, 0x0901, 0 },
	{ 0x2510, 0x1262, 0 },
	{ 0x2510, 0x1260, 0 },
	{ 0x2510, 0x11BF, 0 },
	{ 0x2510, 0x11BB, 0 },
	{ 0x2510, 0x1066, 0 },
	{ 0x2510, 0x11FB, 0 },
	{ 0x2510, 0x0935, 0 },
	{ 0x2510, 0x11BB, 0 },
	{ 0x2510, 0x1263, 0 },
	{ 0x2510, 0x1260, 0 },
	{ 0x2510, 0x1400, 0 },
	{ 0x2510, 0x1510, 0 },
	{ 0x2510, 0x11B8, 0 },
	{ 0x2510, 0x12A0, 0 },
	{ 0x2510, 0x1200, 0 },
	{ 0x2510, 0x1026, 0 },
	{ 0x2510, 0x1000, 0 },
	{ 0x2510, 0x1342, 0 },
	{ 0x2510, 0x1100, 0 },
	{ 0x2510, 0x7A06, 0 },
	{ 0x2510, 0x0913, 0 },
	{ 0x2510, 0x0507, 0 },
	{ 0x2510, 0x0841, 0 },
	{ 0x2510, 0x3750, 0 },
	{ 0x2510, 0x2C2C, 0 },
	{ 0x2510, 0xFE05, 0 },
	{ 0x2510, 0xFE13, 0 },
	{ 0x1008, 0x0361, 0 },
	{ 0x100C, 0x0589, 0 },
	{ 0x100E, 0x07B1, 0 },
	{ 0x1010, 0x0139, 0 },
	{ 0x3230, 0x0304, 0 },
	{ 0x3232, 0x052C, 0 },
	{ 0x3234, 0x0754, 0 },
	{ 0x3236, 0x00DC, 0 },
	{ 0x3566, 0x3328, 0 },
	{ 0x350C, 0x055F, 0 },
	{ 0x32D0, 0x3A02, 0 },
	{ 0x32D2, 0x3508, 0 },
	{ 0x32D4, 0x3702, 0 },
	{ 0x32D6, 0x3C04, 0 },
	{ 0x32DC, 0x370A, 0 },
	{ 0x302A, 0x0006, 0 },
	{ 0x302C, 0x0001, 0 },
	{ 0x302E, 0x0002, 0 },
	{ 0x3030, 0x002C, 0 },
	{ 0x3036, 0x000C, 0 },
	{ 0x3038, 0x0001, 0 },
	{ 0x30B0, 0x0A00, 0 },
	{ 0x30A2, 0x0001, 0 },
	{ 0x30A6, 0x0001, 0 },
	{ 0x3040, 0x0000, 5 },
	{ 0x3082, 0x0008, 5 },
	{ 0x30BA, 0x11F2, 5 },
	{ 0x3044, 0x0400, 5 },
	{ 0x3064, 0x1882, 0 },
	{ 0x3064, 0x1802, 5 },
	{ 0x33E0, 0x0C80, 5 },
	{ 0x3180, 0x0080, 0 },
	{ 0x33E4, 0x0080, 0 },
	{ 0x33E0, 0x0C80, 5 },
	{ 0x3004, 0x0000, 0 },
	{ 0x3008, 0x077F, 0 },
	{ 0x3002, 0x0000, 0 },
	{ 0x3006, 0x0437, 0 },
	{ 0x3032, 0x0000, 0 },
	{ 0x3400, 0x0010, 0 },
	{ 0x3402, 0x0780, 0 },
	{ 0x3402, 0x0F10, 0 },
	{ 0x3404, 0x0438, 0 },
	{ 0x3404, 0x0970, 0 },
	{ 0x3082, 0x0000, 0 },
	{ 0x30BA, 0x11F1, 10 },
	{ 0x30BA, 0x11F0, 0 },
	{ 0x300C, 0x0788, 0 },
	{ 0x300A, 0x05F2, 0 },
	{ 0x3042, 0x0000, 0 },
	{ 0x3238, 0x0222, 5 },
	{ 0x3012, 0x0288, 0 },
	{ 0x3014, 0x032A, 0 },
	{ 0x30B0, 0x0A00, 0 },
	{ 0x32EA, 0x3C0C, 0 },
	{ 0x32EA, 0x3C08, 5 },
	{ 0x32EC, 0x72A1, 10 },
	{ 0x31D0, 0x0000, 0 },
	{ 0x31AE, 0x0004, 0 },
	{ 0x31AE, 0x0304, 0 },
	{ 0x31AC, 0x140C, 0 },
	{ 0x31AC, 0x0C0C, 0 },
	{ 0x301A, 0x1098, 0 },
	{ 0x301A, 0x1018, 0 },
	{ 0x301A, 0x0018, 0 },
	{ 0x31AE, 0x0204, 0 },
	{ 0x3342, 0x122C, 0 },
	{ 0x3346, 0x122C, 0 },
	{ 0x334A, 0x122C, 0 },
	{ 0x334E, 0x122C, 0 },
	{ 0x3344, 0x0011, 0 },
	{ 0x3348, 0x0111, 0 },
	{ 0x334C, 0x0211, 0 },
	{ 0x3350, 0x0311, 0 },
	{ 0x31B0, 0x003A, 0 },
	{ 0x31B2, 0x0020, 0 },
	{ 0x31B4, 0x2876, 0 },
	{ 0x31B6, 0x2193, 0 },
	{ 0x31B8, 0x3048, 0 },
	{ 0x31BA, 0x0188, 0 },
	{ 0x31BC, 0x8006, 0 },
	{ 0x301A, 0x001C, 0 },
	{ TABLE_END, 0x00, 0 },
};

struct reg_value8 max9296_9295_linkA_config[] = {
	{ 0x0053, 0x02, 0 },
	{ 0x005B, 0x00, 0 }, //swap stream ID, and output video from pipeline x
	{ 0x006B, 0x16, 0 },
	{ 0x0073, 0x17, 0 },
	{ 0x007B, 0x36, 0 },
	{ 0x0083, 0x36, 0 },
	{ 0x0093, 0x36, 0 },
	{ 0x009B, 0x36, 0 },
	{ 0x00A3, 0x36, 0 },
	{ 0x00AB, 0x36, 0 },
	{ 0x008B, 0x36, 0 }, //change control channel ID with link A
	{ 0x0000, ADDR_MAX9295_LINKA << 1, 0 }, //change link A address to 0x84
	{ TABLE_END, 0x00, 0 },
};

struct reg_value8 max96722_9295_linkA_config[] = {
	{ 0x0042, ADDR_MAX9295_VIRTUAL << 1, 0 },//broadcast address for all serializers
	{ 0x0043, ADDR_MAX9295_LINKA << 1, 0 },//set serializer address on link A
	{ 0x0044, ADDR_AR0231_LINKA << 1, 0 },//set unique sensor address on link A
	{ 0x0045, ADDR_AR0231_SENSOR << 1, 0 },//set original sensor address.
	{ 0x0000, ADDR_MAX9295_LINKA << 1, 0 },//enable to set serializer address on link A
	{ TABLE_END, 0x00, 0 },
};

struct reg_value8 max96722_9295_linkB_config[] = {
	{ 0x0042, ADDR_MAX9295_VIRTUAL << 1, 0 },//broadcast address for all serializers
	{ 0x0043, ADDR_MAX9295_LINKB << 1, 0 },//set serializer address on link B
	{ 0x0044, ADDR_AR0231_LINKB << 1, 0 },//set unique sensor address on link B
	{ 0x0045, ADDR_AR0231_SENSOR << 1, 0 },//set original sensor address.
	{ 0x0000, ADDR_MAX9295_LINKB << 1, 0 },//enable to set serializer address on link B
	{ TABLE_END, 0x00, 0 },
};

struct reg_value8 max96722_9295_linkC_config[] = {
	{ 0x0042, ADDR_MAX9295_VIRTUAL << 1, 0 },//broadcast address for all serializers
	{ 0x0043, ADDR_MAX9295_LINKC << 1, 0 },//set serializer address on link C
	{ 0x0044, ADDR_AR0231_LINKC << 1, 0 },//set unique sensor address on link C
	{ 0x0045, ADDR_AR0231_SENSOR << 1, 0 },//set original sensor address.
	{ 0x0000, ADDR_MAX9295_LINKC << 1, 0 },//enable to set serializer address on link C
	{ TABLE_END, 0x00, 0 },
};

struct reg_value8 max96722_9295_linkD_config[] = {
	{ 0x0042, ADDR_MAX9295_VIRTUAL << 1, 0 },//broadcast address for all serializers
	{ 0x0043, ADDR_MAX9295_LINKD << 1, 0 },//set serializer address on link D
	{ 0x0044, ADDR_AR0231_LINKD << 1, 0 },//set unique sensor address on link D
	{ 0x0045, ADDR_AR0231_SENSOR << 1, 0 },//set original sensor address
	{ 0x0000, ADDR_MAX9295_LINKD << 1, 0 },//enable to set serializer address on link D
	{ TABLE_END, 0x00, 0 },
};


static void sensor_delay(uint32_t msec)
{
	if (msec <= 20)
		usleep_range(msec * 1000, msec * 1000);
	else
		msleep(msec);
}

int max9295_read_reg(struct i2c_client *i2c_client,
		unsigned short reg, unsigned char *val)
{
	return sensor_read_reg8(i2c_client, ADDR_MAX9295, reg, val);
}

int max9295_write_reg(struct i2c_client *i2c_client, unsigned short reg, unsigned char val)
{
	return sensor_write_reg8(i2c_client, ADDR_MAX9295, reg, val);
}

int ar0231_read_reg(struct i2c_client *i2c_client, unsigned short reg, unsigned short *val)
{
	return sensor_read_reg16(i2c_client, ADDR_AR0231_SENSOR, reg, val);
}

int ar0231_write_reg(struct i2c_client *i2c_client, unsigned short reg, unsigned short val)
{
	return sensor_write_reg16(i2c_client, ADDR_AR0231_SENSOR, reg, val);
}

int max9295_read_dev_id(struct i2c_client *i2c_client)
{
	int ret = 0;
	unsigned char val_id = 0;

	ret = max9295_read_reg(i2c_client, MAX9295_REGADDR_DEV_ID, &val_id);
	if (ret != 0) {
		dev_err(&i2c_client->dev,
				"%s:check error: read id failed! ret = %d.\n", __func__, ret);
		return ret;
	}

	dev_info(&i2c_client->dev,
			 "%s max9295 id=0x%x\n", __func__, val_id);

	return ret;
}

static int ar0231_read_id_version(struct i2c_client *i2c_client)
{
	int ret = 0;
	unsigned short val_id = 0;
	unsigned short val_version = 0;


	ret = ar0231_read_reg(i2c_client, SENSOR_REGADDR_CHIP_ID_REG, &val_id);
	if (ret != 0) {
		dev_err(&i2c_client->dev,
				"%s:check error: read id failed! ret = %d.\n", __func__, ret);
		return ret;
	}

	ret = ar0231_read_reg(i2c_client, SENSOR_REGADDR_CHIP_VERSION_REG, &val_version);
	if (ret != 0) {
		dev_err(&i2c_client->dev,
				"%s:check error: read version failed! ret = %d.\n", __func__, ret);
		return ret;
	}

	dev_info(&i2c_client->dev,
			 "%s ar0231 id=0x%x, version=0x%x\n", __func__, val_id, val_version);

	return ret;
}

int max9295_initialize(struct i2c_client *i2c_client, struct reg_value8 *config_data)
{
	int i = 0;
	int retval = 0;

	while (1) {
		if (config_data[i].reg_addr == TABLE_END)
			break;

		retval = max9295_write_reg(i2c_client, config_data[i].reg_addr, config_data[i].val);
		if (retval < 0) {
			break;
		}
		if (config_data[i].delay_ms != 0) {
			sensor_delay(config_data[i].delay_ms);
		}
		i++;
	}

	return retval;
}

int ar0231_initialize(struct i2c_client *i2c_client, struct reg_value16 *config_data)
{
	int i = 0;
	int retval = 0;
	//ar0231_i2c_client = i2c_client;

	while (1) {
		if (config_data[i].reg_addr == TABLE_END)
			break;

		retval = ar0231_write_reg(i2c_client, config_data[i].reg_addr, config_data[i].val);
		if (retval < 0) {
			break;
		}
		if (config_data[i].delay_ms != 0) {
			sensor_delay(config_data[i].delay_ms);
		}
		i++;
	}

	//retval = ar0231_read_id_version(i2c_client);

	return retval;
}

