// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 Siengine
 */

#include <linux/delay.h>
#include <linux/i2c.h>

#ifdef CONFIG_OF
//#include <linux/of.h>
//#include <linux/of_gpio.h>
#endif

#define DES_ADDR0				(0x58)
#define DES_ALIAS0				(0x60)
#define DES_ADDR1				(0x58)
#define DES_ALIAS1				(0x62)

#define SERADDR					(0xc)
#define DES0ADDR				(DES_ALIAS0>>1)
#define DES1ADDR				(DES_ALIAS1>>1)

#define I2C_7BIT_ADDR(x)		(x>>1)

struct ds90ubxxx_i2c {
	struct i2c_client *i2c_client;

	u32 ser_addr;
	u32 des_addr;
	const char *serializer_type;

	bool des_dectected[2];
};

static s32 ds90ubxxx_write_reg(struct i2c_client *i2c_client,
			u8 i2c_addr, u8 reg, u8 val)
{
	u8 au8Buf[2] = {0};
	int i = 0;

	au8Buf[0] = reg;
	au8Buf[1] = val;

	i2c_client->addr = i2c_addr;
	for (i = 0; i < 5; i++) {
		if (!(i2c_master_send(i2c_client, au8Buf, 2) < 0))
			break;
		usleep_range(10*1000, 10*1000);
	}
	if (i == 5) {
		pr_err("%s:write reg err: i2c_addr:%x,reg=%x,val=%x\n",
			__func__, i2c_client->addr, reg, val);
		return -1;
	}

	return 0;
}

static u8 ds90ubxxx_read_reg(struct i2c_client *i2c_client, u8 i2c_addr, u8 reg)
{
	u8 au8RegBuf = reg;
	u8 u8RdVal = 0;
	int i = 0;

	i2c_client->addr = i2c_addr;
	for (i = 0; i < 3; i++) {
		if (i2c_master_send(i2c_client, &au8RegBuf, 1) != 1) {
			usleep_range(1*1000, 1*1000);
			continue;
		}

		if (i2c_master_recv(i2c_client, &u8RdVal, 1) != 1) {
			usleep_range(1*1000, 1*1000);
			continue;
		}
		break;
	}

	if (i == 3) {
		pr_err("%s:read reg err: i2c_addr:%x,reg=%x,val=%x\n",
			__func__, i2c_client->addr, reg, u8RdVal);
		return -1;
	}

	return u8RdVal;
}

static int dx11_csd_config(struct ds90ubxxx_i2c *ds90ubxxx_i2c)
{
	unsigned char serAddr = 0;
	unsigned char desAlias0 = 0;
	unsigned char Reg_value = 0;
	unsigned char TEMP_FINAL = 0;
	unsigned char TEMP_FINAL_C = 0;
	unsigned char Efuse_TS_CODE = 0;
	unsigned char Ramp_UP_Range_CODES_Needed = 0;
	unsigned char Ramp_DN_Range_CODES_Needed = 0;
	unsigned char Ramp_UP_CAP_DELTA = 0;
	unsigned char Ramp_DN_CAP_DELTA = 0;
	unsigned char TS_CODE_UP = 0;
	unsigned char rb = 0;
	unsigned char TS_CODE_DN = 0;
	unsigned char I2C_PASS_THROUGH = 0;
	unsigned char I2C_PASS_THROUGH_REG = 0;
	unsigned char I2C_PASS_THROUGH_MASK = 0;
	struct i2c_client *client = ds90ubxxx_i2c->i2c_client;

	/*
	 * Serializer: DS90Ux983-Q1
	 * User Inputs:
	 * Serializer I2C Address= 0x18
	 * Max DP Lane Count = 4
	 * Max DP Lane Rate = 5.4Gbps
	 * DPRX no SSC Mode Enabled
	 * DP MST Mode Enabled
	 * DP Mode Enabled
	 * FPD-Link Configuration: FPD-Link IV Independent - 6.75Gbps Port 0, 6.75Gbps Port 1


	 * Number of Displays = 1

	 * Video Processor 0 (Stream 0) Properties:
	 * Total Horizontal Pixels = 2756
	 * Total Vertical Lines = 1642
	 * Active Horizontal Pixels = 2560
	 * Active Vertical Lines = 1600
	 * Horizontal Back Porch = 56
	 * Vertical Back Porch = 8
	 * Horizontal Sync = 56
	 * Vertical Sync = 2
	 * Horizontal Front Porch = 84
	 * Vertical Front Porch = 32
	 * Horizontal Sync Polarity = Positive
	 * Vertical Sync Polarity = Positive
	 * Bits per pixel = 24
	 * Pixel Clock = 270MHz
	 * PATGEN Enabled

	 * Deserializer 0: DS90Ux984-Q1
	 * Note: The 984 should be strapped to 6.75Gbps FPD IV mode using MODE_SEL0
	 * User Inputs:
	 * Deserializer I2C Address = 0x58
	 * Deserializer I2C Alias = 0x58
	 * DP Port 0 Enabled
	 * DP0 is outputting video 0 from the SER
	 * DP Port 0 PatGen Disabled
	 * DP Port 1 Disabled
	 * DP Port 1 PatGen Disabled
	 * DP Rate set to 2.7 Gbps
	 * DP lane number set to 4 lanes
	 */

	/* Set up Variables */
	serAddr = SERADDR;
	desAlias0 = DES0ADDR;
	ds90ubxxx_write_reg(client, serAddr, 0x70, DES_ADDR0);
	ds90ubxxx_write_reg(client, serAddr, 0x78, DES_ALIAS0);
	ds90ubxxx_write_reg(client, serAddr, 0x88, 0x0);

	/* Program SER to FPD-Link IV mode */
	ds90ubxxx_write_reg(client, serAddr, 0x5b, 0x23); //Disable FPD3 FIFO pass through;
	ds90ubxxx_write_reg(client, serAddr, 0x5, 0x2c); //Force FPD4_TX independent mode;

	/* Set up FPD IV PLL Settings - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed */
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x8); //Select PLL reg page
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x1b);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x8); //Disable PLL0
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x5b);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x8); //Disable PLL1
	ds90ubxxx_write_reg(client, serAddr, 0x2, 0xd1); //Enable mode overwrite
	ds90ubxxx_write_reg(client, serAddr, 0x2d, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x8); //Select PLL page
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x5); //Select Ncount Reg
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x7d); //Set Ncount
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x13); //Select post div reg
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x90); //Set post div for 6.75 Gbps
	ds90ubxxx_write_reg(client, serAddr, 0x2d, 0x1); //Select write reg to port 0
	ds90ubxxx_write_reg(client, serAddr, 0x6a, 0xa); //set BC sampling rate
	ds90ubxxx_write_reg(client, serAddr, 0x6e, 0x80); //set BC fractional sampling
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x4); //Select FPD page and set BC settings for FPD IV port 0
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x6);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0xd);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x34);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0xe);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x53);
	ds90ubxxx_write_reg(client, serAddr, 0x2, 0x51); //Set HALFRATE_MODE;

	/* Zero out PLL fractional - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed */
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x8); //Select PLL page;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x4);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x1e);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x1f);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x20);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0);

	/* Configure and Enable PLLs - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed */
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0xe); //Select VCO reg;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xc7); //Set VCO;
	ds90ubxxx_write_reg(client, serAddr, 0x1, 0x30); //soft reset PLL;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x1b);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Enable PLL0;
	ds90ubxxx_write_reg(client, serAddr, 0x1, 0x1); //soft reset Ser;

	mdelay(0.04*1000);

	/* Enable I2C Passthrough */
	I2C_PASS_THROUGH = ds90ubxxx_read_reg(client, serAddr, 0x7);
	I2C_PASS_THROUGH_MASK = 0x08;
	I2C_PASS_THROUGH_REG = I2C_PASS_THROUGH | I2C_PASS_THROUGH_MASK;
	ds90ubxxx_write_reg(client, serAddr, 0x07, I2C_PASS_THROUGH_REG); //Enable I2C Passthrough

	if (ds90ubxxx_read_reg(client, desAlias0, 0x0) != DES_ADDR0) {
		pr_err("%s: can't detect deserializer 0\n", __func__);
		ds90ubxxx_i2c->des_dectected[0] = false;
		return -1;
	}
	ds90ubxxx_i2c->des_dectected[0] = true;

	ds90ubxxx_write_reg(client, serAddr, 0x2d, 0x1); //Select write to port0 reg

	/* Set DP Config */
	ds90ubxxx_write_reg(client, serAddr, 0x48, 0x1); //Enable APB Interface;
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x0); //Force HPD low to configure 983 DP settings
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x74); //Set max advertised link rate = 2.7Gbps
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x14);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x70); //Set max advertised lane count = 4
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x4);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x14); //Request min VOD swing of 0x02
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x2);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x2);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x18); //Set SST/MST mode and DP/eDP Mode
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x14);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x0); //Force HPD high to trigger link training
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);

	mdelay(0.5*1000); //Allow time after HPD is pulled high for the source to train and provide video (may need to adjust based on source properties*10000)

	/* Program VP Configs */
	// Configure VP 0
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x32);
	//ds90ubxxx_write_reg(client, serAddr, 0x41, 0x1);
	//ds90ubxxx_write_reg(client, serAddr, 0x42, 0xa8); //Set VP_SRC_SELECT to Stream 0 for SST Mode;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x2);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //VID H Active;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xa); //VID H Active;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x10);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Horizontal Active;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xa); //Horizontal Active;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x38); //Horizontal Back Porch;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Horizontal Back Porch;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x38); //Horizontal Sync;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Horizontal Sync;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xc4); //Horizontal Total;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xa); //Horizontal Total;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x40); //Vertical Active;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x6); //Vertical Active;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x8); //Vertical Back Porch;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Vertical Back Porch;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x2); //Vertical Sync;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Vertical Sync;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x20); //Vertical Front Porch;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Vertical Front Porch;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x27);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //HSYNC Polarity = +, VSYNC Polarity = +;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x23); //M/N Register;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x33); //M value;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x33); //M value;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xf); //N value;

	/* Enable VPs */
	ds90ubxxx_write_reg(client, serAddr, 0x43, 0x0); //Set number of VPs used = 1;
	ds90ubxxx_write_reg(client, serAddr, 0x44, 0x1); //Enable video processors;

	/* Configure Serializer TX Link Layer */
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x2e) ;//Link layer Reg page;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x1) ;//Link layer 0 stream enable;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x1) ;//Link layer 0 stream enable;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x6) ;//Link layer 0 time slot 0;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x41) ;//Link layer 0 time slot;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x20) ;//Set Link layer vp bpp;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x61) ;//Set Link layer vp bpp according to VP Bit per pixel;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x0) ;//Link layer 0 enable;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x3) ;//Link layer 0 enable;

	/* Read Deserializer 0 Temp */
	ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x6c);
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0xd);
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x13);
	TEMP_FINAL = ds90ubxxx_read_reg(client, desAlias0, 0x42);
	TEMP_FINAL_C = 2*TEMP_FINAL - 273;

	/* Set up Deserializer 0 Temp Ramp Optimizations */
	Efuse_TS_CODE = 2;
	Ramp_UP_Range_CODES_Needed = (int)((150-TEMP_FINAL_C)/(190/11)) + 1;
	Ramp_DN_Range_CODES_Needed = (int)((TEMP_FINAL_C-30)/(190/11)) + 1;
	Ramp_UP_CAP_DELTA = Ramp_UP_Range_CODES_Needed - 4;
	Ramp_DN_CAP_DELTA = Ramp_DN_Range_CODES_Needed - 7;
	ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x3c);
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0xf5);
	ds90ubxxx_write_reg(client, desAlias0, 0x42, (Efuse_TS_CODE<<4)+1); // Override TS_CODE Efuse Code
	if (Ramp_UP_CAP_DELTA > 0) {
		TS_CODE_UP = Efuse_TS_CODE - Ramp_UP_CAP_DELTA;
		if (TS_CODE_UP < 0)
			TS_CODE_UP = 0;
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0xf5);
		rb = ds90ubxxx_read_reg(client, desAlias0, 0x42);
		rb &= 0x8F;
		rb |= (TS_CODE_UP << 4);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, rb);
		rb = ds90ubxxx_read_reg(client, desAlias0, 0x42);
		rb &= 0xFE;
		rb |= 0x01;
		ds90ubxxx_write_reg(client, desAlias0, 0x42, rb);
		ds90ubxxx_write_reg(client, desAlias0, 0x1, 0x1);

		mdelay(0.04*1000);
	}
	if (Ramp_DN_CAP_DELTA > 0) {
		TS_CODE_DN = Efuse_TS_CODE + Ramp_DN_CAP_DELTA;
		if (TS_CODE_DN >= 7)
			TS_CODE_DN = 7;
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0xf5);
		rb = ds90ubxxx_read_reg(client, desAlias0, 0x42);
		rb &= 0x8F;
		rb |= (TS_CODE_DN << 4);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, rb);
		rb = ds90ubxxx_read_reg(client, desAlias0, 0x42);
		rb &= 0xFE;
		rb |= 0x01;
		ds90ubxxx_write_reg(client, desAlias0, 0x42, rb);
		ds90ubxxx_write_reg(client, desAlias0, 0x1, 0x1);

		mdelay(0.04*1000);
	}

	/* Clear CRC errors from initial link process */
	Reg_value = ds90ubxxx_read_reg(client, serAddr, 0x2);
	Reg_value = Reg_value | 0x20;
	ds90ubxxx_write_reg(client, serAddr, 0x2, Reg_value); //CRC Error Reset
	Reg_value = ds90ubxxx_read_reg(client, serAddr, 0x2);
	Reg_value = Reg_value & 0xdf;
	ds90ubxxx_write_reg(client, serAddr, 0x2, Reg_value); //CRC Error Reset Clear
	ds90ubxxx_write_reg(client, serAddr, 0x2d, 0x1);

	/* Hold Des DTG in reset */
	ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x50); //Select DTG Page
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x32);
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x6); //Hold Port 0 DTG in reset
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x62);
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x6); //Hold Port 1 DTG in reset

	/* Disable Stream Mapping */
	ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x3); //Select both Output Ports
	ds90ubxxx_write_reg(client, desAlias0, 0xd0, 0x0); //Disable FPD4 video forward to Output Port
	ds90ubxxx_write_reg(client, desAlias0, 0xd7, 0x0); //Disable FPD3 video forward to Output Port

	/* Force DP Rate */
	ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x2c); //Select DP Page
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x81);
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x60); //Set DP Rate to 2.7Gbps
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x82);
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x3); //Enable force DP rate with calibration disabled
	ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x2c); //Select DP Page
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x91);
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0xc); //Force 4 lanes
	ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x30); //Disable DP SSCG
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0xf);
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x1, 0x40);

	/* Setup DP ports */
	ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x12); //Select Port 1 registers
	ds90ubxxx_write_reg(client, desAlias0, 0x46, 0x0); //Disable DP Port 1
	ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x1); //Select Port 0 registers
	ds90ubxxx_write_reg(client, desAlias0, 0x1, 0x40); //DP-TX-PLL RESET Applied

	/* Map video to display output */
	ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x3); //Select both Output Ports
	ds90ubxxx_write_reg(client, desAlias0, 0xd0, 0xc); //Enable FPD_RX video forward to Output Port
	ds90ubxxx_write_reg(client, desAlias0, 0xd1, 0xf); //Every stream forwarded on DC
	ds90ubxxx_write_reg(client, desAlias0, 0xd6, 0x0); //Send Stream 0 to Output Port 0 and Send Stream 0 to Output Port 1
	ds90ubxxx_write_reg(client, desAlias0, 0xd7, 0x0); //FPD3 mapping disabled
	ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x1); //Select Port 0

	/* Program quad pixel clock for DP port 0 */
	ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x1); //Select Port0 registers
	ds90ubxxx_write_reg(client, desAlias0, 0xb1, 0x1); //Enable clock divider
	ds90ubxxx_write_reg(client, desAlias0, 0xb2, 0xb0); //Program M value lower byte
	ds90ubxxx_write_reg(client, desAlias0, 0xb3, 0x1e); //Program M value middle byte
	ds90ubxxx_write_reg(client, desAlias0, 0xb4, 0x4); //Program M value upper byte
	ds90ubxxx_write_reg(client, desAlias0, 0xb5, 0xc0); //Program N value lower byte
	ds90ubxxx_write_reg(client, desAlias0, 0xb6, 0x7a); //Program N value middle byte
	ds90ubxxx_write_reg(client, desAlias0, 0xb7, 0x10); //Program N value upper byte
	ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x1); //Select Port 0 registers

	/* Setup DTG for port 0 */
	ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x50); //Select DTG Page
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x20);
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x93); //Set up Local Display DTG BPP, Sync Polarities, and Measurement Type
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x29); //Set Hstart;
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x80); //Hstart upper byte
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x2a);
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x70); //Hstart lower byte
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x2f); //Set HSW
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x40); //HSW upper byte
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x30);
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x38); //HSW lower byte

	/* Program DPTX for DP port 0 */
	ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1); //Enable APB interface
	ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xa4); //Set bit per color
	ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x20);
	ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xb8); //Set pixel width
	ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x4);
	ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xac); //Set DP Mvid
	ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x80);
	ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xb4); //Set DP Nvid
	ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x80);
	ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xc8); //Set TU Mode
	ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xb0); //Set TU Size
	ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x40);
	ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x30);
	ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xc8); //Set FIFO Size
	ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x5);
	ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x40);
	ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xbc); //Set data count
	ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x80);
	ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x7);
	ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xc0); //Disable STREAM INTERLACED
	ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xc4); //Set SYNC polarity
	ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0xc);
	ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);

	/* Release Des DTG reset */
	ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x50); //Select DTG Page
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x32);
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x4); //elease Port 0 DTG
	ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x62);
	ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x4); //elease Port 1 DTG
	ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x49, 0x80); //Set Htotal
	ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0xc4);
	ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0xa);
	ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);

	/* Enable DP 0 output */
	ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x49, 0x84); //Enable DP output
	ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x1);
	ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);

	return 0;
}


static int dx11_dual_csd_config(struct ds90ubxxx_i2c *ds90ubxxx_i2c)
{
	unsigned char serAddr = 0;
	unsigned char desAlias0 = 0;
	unsigned char desAlias1 = 0;
	//unsigned char VP0sts = 0;
	unsigned char Reg_value = 0;
	unsigned char TEMP_FINAL = 0;
	unsigned char TEMP_FINAL_C = 0;
	unsigned char Efuse_TS_CODE = 0;
	unsigned char Ramp_UP_Range_CODES_Needed = 0;
	unsigned char Ramp_DN_Range_CODES_Needed = 0;
	unsigned char Ramp_UP_CAP_DELTA = 0;
	unsigned char Ramp_DN_CAP_DELTA = 0;
	unsigned char TS_CODE_UP = 0;
	unsigned char rb = 0;
	unsigned char TS_CODE_DN = 0;
	unsigned char I2C_PASS_THROUGH = 0;
	unsigned char I2C_PASS_THROUGH_REG = 0;
	unsigned char I2C_PASS_THROUGH_MASK = 0;
	struct i2c_client *client = ds90ubxxx_i2c->i2c_client;

	/*
	 * Serializer: DS90Ux983-Q1
	 * User Inputs:
	 * Serializer I2C Address= 0x18
	 * Max DP Lane Count = 4
	 * Max DP Lane Rate = 5.4Gbps
	 * DPRX no SSC Mode Enabled
	 * DP MST Mode Enabled
	 * DP Mode Enabled
	 * FPD-Link Configuration: FPD-Link IV Independent - 6.75Gbps Port 0, 6.75Gbps Port 1


	 * Number of Displays = 2

	 * Video Processor 0 (Stream 0) Properties:
	 * Total Horizontal Pixels = 2756
	 * Total Vertical Lines = 1642
	 * Active Horizontal Pixels = 2560
	 * Active Vertical Lines = 1600
	 * Horizontal Back Porch = 56
	 * Vertical Back Porch = 8
	 * Horizontal Sync = 56
	 * Vertical Sync = 2
	 * Horizontal Front Porch = 84
	 * Vertical Front Porch = 32
	 * Horizontal Sync Polarity = Positive
	 * Vertical Sync Polarity = Positive
	 * Bits per pixel = 24
	 * Pixel Clock = 270MHz
	 * PATGEN Enabled

	 * Video Processor 1 (Stream 1) Properties:
	 * Total Horizontal Pixels = 2756
	 * Total Vertical Lines = 1642
	 * Active Horizontal Pixels = 2560
	 * Active Vertical Lines = 1600
	 * Horizontal Back Porch = 56
	 * Vertical Back Porch = 8
	 * Horizontal Sync = 56
	 * Vertical Sync = 2
	 * Horizontal Front Porch = 84
	 * Vertical Front Porch = 32
	 * Horizontal Sync Polarity = Positive
	 * Vertical Sync Polarity = Positive
	 * Bits per pixel = 24
	 * Pixel Clock = 270MHz
	 * PATGEN Enabled

	 * Deserializer 0: DS90Ux984-Q1
	 * Note: The 984 should be strapped to 6.75Gbps FPD IV mode using MODE_SEL0
	 * User Inputs:
	 * Deserializer I2C Address = 0x58
	 * Deserializer I2C Alias = 0x58
	 * DP Port 0 Enabled
	 * DP0 is outputting video 0 from the SER
	 * DP Port 0 PatGen Disabled
	 * DP Port 1 Disabled
	 * DP Port 1 PatGen Disabled
	 * DP Rate set to 2.7 Gbps
	 * DP lane number set to 4 lanes

	 * Deserializer 1: DS90Ux984-Q1
	 * Note: The 984 should be strapped to 6.75Gbps FPD IV mode using MODE_SEL0
	 * User Inputs:
	 * Deserializer I2C Address = 0x58
	 * Deserializer I2C Alias = 0x60
	 * DP Port 0 Enabled
	 * DP0 is outputting video 1 from the SER
	 * DP Port 0 PatGen Disabled
	 * DP Port 1 Disabled
	 * DP Port 1 PatGen Disabled
	 * DP Rate set to 2.7 Gbps
	 * DP lane number set to 4 lanes
	 */

	/* Set up Variables */
	serAddr = SERADDR;
	desAlias0 = DES0ADDR;
	desAlias1 = DES1ADDR;
	ds90ubxxx_write_reg(client, serAddr, 0x70, DES_ADDR0);
	ds90ubxxx_write_reg(client, serAddr, 0x78, DES_ALIAS0);
	ds90ubxxx_write_reg(client, serAddr, 0x88, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x71, DES_ADDR1);
	ds90ubxxx_write_reg(client, serAddr, 0x79, DES_ALIAS1+1);
	ds90ubxxx_write_reg(client, serAddr, 0x89, 0x0);

	/* Program SER to FPD-Link IV mode */
	ds90ubxxx_write_reg(client, serAddr, 0x5b, 0x23); //Disable FPD3 FIFO pass through;
	ds90ubxxx_write_reg(client, serAddr, 0x5, 0x3c); //Force FPD4_TX independent mode;

	/* Set up FPD IV PLL Settings - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed */
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x8); //Select PLL reg page
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x1b);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x8); //Disable PLL0
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x5b);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x8); //Disable PLL1
	ds90ubxxx_write_reg(client, serAddr, 0x2, 0xd1); //Enable mode overwrite
	ds90ubxxx_write_reg(client, serAddr, 0x2d, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x8); //Select PLL page
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x5); //Select Ncount Reg
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x7d); //Set Ncount
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x13); //Select post div reg
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x90); //Set post div for 6.75 Gbps
	ds90ubxxx_write_reg(client, serAddr, 0x2d, 0x1); //Select write reg to port 0
	ds90ubxxx_write_reg(client, serAddr, 0x6a, 0xa); //set BC sampling rate
	ds90ubxxx_write_reg(client, serAddr, 0x6e, 0x80); //set BC fractional sampling
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x4); //Select FPD page and set BC settings for FPD IV port 0
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x6);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0xd);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x34);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0xe);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x53);
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x8); //Select PLL page
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x45); //Select Ncount Reg
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x7d); //Set Ncount
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x53); //Select post div reg
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x90); //Set post div for 6.75 Gbps
	ds90ubxxx_write_reg(client, serAddr, 0x2d, 0x12); //Select write reg to port 1
	ds90ubxxx_write_reg(client, serAddr, 0x6a, 0xa); //set BC sampling rate
	ds90ubxxx_write_reg(client, serAddr, 0x6e, 0x80); //set BC fractional sampling
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x4); //Select FPD page and set BC settings for FPD IV port 1
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x26);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x2d);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x34);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x2e);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x53);
	ds90ubxxx_write_reg(client, serAddr, 0x2, 0xd1); //#Set HALFRATE_MODE

	/* Zero out PLL fractional - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed */
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x8); //Select PLL page;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x4);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x1e);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x1f);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x20);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0);

	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x44);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x5e);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x5f);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x60);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0);

	/* Configure and Enable PLLs - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed */
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0xe); //Select VCO reg
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xc7); //Set VCO
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x4e); //Select VCO reg
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xc7); //Set VCO
	ds90ubxxx_write_reg(client, serAddr, 0x1, 0x30); //soft reset PLL
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x8); //Select PLL page
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x1b);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Enable PLL0
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x5b);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Enable PLL1
	ds90ubxxx_write_reg(client, serAddr, 0x1, 0x1); //soft reset Ser

	mdelay(0.04*1000);

	/* Enable I2C Passthrough */
	I2C_PASS_THROUGH = ds90ubxxx_read_reg(client, serAddr, 0x7);
	I2C_PASS_THROUGH_MASK = 0x08;
	I2C_PASS_THROUGH_REG = I2C_PASS_THROUGH | I2C_PASS_THROUGH_MASK;
	ds90ubxxx_write_reg(client, serAddr, 0x07, I2C_PASS_THROUGH_REG); //Enable I2C Passthrough

	if (ds90ubxxx_read_reg(client, desAlias0, 0x0) != DES_ADDR0) {
		pr_err("%s: can't detect deserializer 0\n", __func__);
		ds90ubxxx_i2c->des_dectected[0] = false;
	} else {
		ds90ubxxx_i2c->des_dectected[0] = true;
	}

	if (ds90ubxxx_read_reg(client, desAlias1, 0x0) != DES_ADDR1) {
		pr_err("%s: can't detect deserializer 1\n", __func__);
		ds90ubxxx_i2c->des_dectected[1] = false;
	} else {
		ds90ubxxx_i2c->des_dectected[1] = true;
	}

	if (ds90ubxxx_i2c->des_dectected[0] == false &&
		ds90ubxxx_i2c->des_dectected[1] == false)
		return -1;

	ds90ubxxx_write_reg(client, serAddr, 0x2d, 0x1); //Select write to port0 reg

	/* Set DP Config */
	ds90ubxxx_write_reg(client, serAddr, 0x48, 0x1); //Enable APB Interface;
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x0); //Force HPD low to configure 983 DP settings
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x74); //Set max advertised link rate = 2.7Gbps
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x14);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x70); //Set max advertised lane count = 4
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x4);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x14); //Request min VOD swing of 0x02
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x2);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x2);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x18); //Set SST/MST mode and DP/eDP Mode
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x14);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x4); //Set MST_PAYLOAD_ID
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x9);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x8); //Set MST_PAYLOAD_ID
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x9);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x2);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x4); //Set MST_PAYLOAD_ID
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x9);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x8); //Set MST_PAYLOAD_ID
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x9);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x2);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x0); //Set DP Virtual Sink 0 Settings
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0xa);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x4); //Set DP Virtual Sink 0 Settings
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0xa);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x4);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x18); //Set DP Virtual Sink 0 Settings
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0xa);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x14); //Set DP Virtual Sink 0 Settings
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0xa);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x0); //Set DP Virtual Sink 1 Settings
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0xb);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x4); //Set DP Virtual Sink 1 Settings
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0xb);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x4);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x18); //Set DP Virtual Sink 1 Settings
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0xb);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x14); //Set DP Virtual Sink 1 Settings
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0xb);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x49, 0x0); //Force HPD high to trigger link training
	ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x1);
	ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
	ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);

	mdelay(0.5*1000); //Allow time after HPD is pulled high for the source to train and provide video (may need to adjust based on source properties*10000)

	/* Program VP Configs */
	// Configure VP 0
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x32);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x2);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //VID H Active;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xa); //VID H Active;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x10);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Horizontal Active;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xa); //Horizontal Active;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x38); //Horizontal Back Porch;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Horizontal Back Porch;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x38); //Horizontal Sync;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Horizontal Sync;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xc4); //Horizontal Total;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xa); //Horizontal Total;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x40); //Vertical Active;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x6); //Vertical Active;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x8); //Vertical Back Porch;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Vertical Back Porch;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x2); //Vertical Sync;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Vertical Sync;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x20); //Vertical Front Porch;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Vertical Front Porch;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x27);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //HSYNC Polarity = +, VSYNC Polarity = +;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x23); //M/N Register;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x33); //M value;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x33); //M value;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xf); //N value;

	// Configure VP 1
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x32);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x42);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //VID H Active
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xa); //VID H Active
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x50);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Horizontal Active
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xa); //Horizontal Active
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x38); //Horizontal Back Porch
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Horizontal Back Porch
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x38); //Horizontal Sync
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Horizontal Sync
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xc4); //Horizontal Total
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xa); //Horizontal Total
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x40); //Vertical Active
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x6); //Vertical Active
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x8); //Vertical Back Porch
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Vertical Back Porch
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x2); //Vertical Sync
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Vertical Sync
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x20); //Vertical Front Porch
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //Vertical Front Porch
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x67);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x0); //HSYNC Polarity = +, VSYNC Polarity = +
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x63); //M/N Register
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x33); //M value
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x33); //M value
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xf); //N value

	/* Enable VPs */
	ds90ubxxx_write_reg(client, serAddr, 0x43, 0x1); //Set number of VPs used = 2
	ds90ubxxx_write_reg(client, serAddr, 0x44, 0x3); //Enable video processors

#if 0
	/* Enable PATGEN */
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x30);
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x29);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x8); //Set PATGEN Color Depth to 24bpp for VP0
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x28);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x95); //Enable PATGEN on VP0 - Comment out this line to disable PATGEN and enable end to end video

	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x69);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x8); //Set PATGEN Color Depth to 24bpp for VP1
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x68);
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x95); //Enable PATGEN on VP1 - Comment out this line to disable PATGEN and enable end to end video
#endif

#if 0
	mdelay(0.2*1000); //Delay for VPs to sync to DP source

	/* Check if VP is synchronized to DP input */
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x31); //Select VP Page;
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x30);
	VP0sts = ds90ubxxx_read_reg(client, serAddr, 0x42);
	if (VP0sts == 0) {
		ds90ubxxx_write_reg(client, serAddr, 0x49, 0x54); //Video Input Reset if VP is not synchronized
		ds90ubxxx_write_reg(client, serAddr, 0x4a, 0x0);
		ds90ubxxx_write_reg(client, serAddr, 0x4b, 0x1);
		ds90ubxxx_write_reg(client, serAddr, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, serAddr, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, serAddr, 0x4e, 0x0);
	}
#endif

	/* Configure Serializer TX Link Layer */
	ds90ubxxx_write_reg(client, serAddr, 0x40, 0x2e); //Link layer Reg page
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x1); //Link layer 0 stream enable
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x1); //Link layer 0 stream enable
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x6); //Link layer 0 time slot 0
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x41); //Link layer 0 time slot

	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x11); //Select LINK1_STREAM_EN
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x2); //Enable Link Layer 1 Streams
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x17); //Select LINK1_SLOT_REQ1
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x41); //Set number of time slots
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x12); //Select LINK1_MAP_REG0
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x10); //Assign link layer stream 1 map
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x13); //Select LINK1_MAP_REG1
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x32); //Assign link layer stream 2/3 map
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x20); //Set Link layer vp bpp;
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0x55); //Set Link layer vp bpp according to VP Bit per pixel
	ds90ubxxx_write_reg(client, serAddr, 0x41, 0x0); //Link layer 0 enable
	ds90ubxxx_write_reg(client, serAddr, 0x42, 0xf); //Link layer 0 enable

	if (ds90ubxxx_i2c->des_dectected[0]) {
		/* Read Deserializer 0 Temp */
		ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x6c);
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0xd);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x13);
		TEMP_FINAL = ds90ubxxx_read_reg(client, desAlias0, 0x42);
		TEMP_FINAL_C = 2*TEMP_FINAL - 273;

		/* Set up Deserializer 0 Temp Ramp Optimizations */
		Efuse_TS_CODE = 2;
		Ramp_UP_Range_CODES_Needed = (int)((150-TEMP_FINAL_C)/(190/11)) + 1;
		Ramp_DN_Range_CODES_Needed = (int)((TEMP_FINAL_C-30)/(190/11)) + 1;
		Ramp_UP_CAP_DELTA = Ramp_UP_Range_CODES_Needed - 4;
		Ramp_DN_CAP_DELTA = Ramp_DN_Range_CODES_Needed - 7;
		ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x3c);
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0xf5);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, (Efuse_TS_CODE<<4)+1); // Override TS_CODE Efuse Code
		if (Ramp_UP_CAP_DELTA > 0) {
			TS_CODE_UP = Efuse_TS_CODE - Ramp_UP_CAP_DELTA;
			if (TS_CODE_UP < 0)
				TS_CODE_UP = 0;
			ds90ubxxx_write_reg(client, desAlias0, 0x41, 0xf5);
			rb = ds90ubxxx_read_reg(client, desAlias0, 0x42);
			rb &= 0x8F;
			rb |= (TS_CODE_UP << 4);
			ds90ubxxx_write_reg(client, desAlias0, 0x42, rb);
			rb = ds90ubxxx_read_reg(client, desAlias0, 0x42);
			rb &= 0xFE;
			rb |= 0x01;
			ds90ubxxx_write_reg(client, desAlias0, 0x42, rb);
			ds90ubxxx_write_reg(client, desAlias0, 0x1, 0x1);

			mdelay(0.04*1000);
		}
		if (Ramp_DN_CAP_DELTA > 0) {
			TS_CODE_DN = Efuse_TS_CODE + Ramp_DN_CAP_DELTA;
			if (TS_CODE_DN >= 7)
				TS_CODE_DN = 7;
			ds90ubxxx_write_reg(client, desAlias0, 0x41, 0xf5);
			rb = ds90ubxxx_read_reg(client, desAlias0, 0x42);
			rb &= 0x8F;
			rb |= (TS_CODE_DN << 4);
			ds90ubxxx_write_reg(client, desAlias0, 0x42, rb);
			rb = ds90ubxxx_read_reg(client, desAlias0, 0x42);
			rb &= 0xFE;
			rb |= 0x01;
			ds90ubxxx_write_reg(client, desAlias0, 0x42, rb);
			ds90ubxxx_write_reg(client, desAlias0, 0x1, 0x1);

			mdelay(0.04*1000);
		}
	}

	if (ds90ubxxx_i2c->des_dectected[1]) {
		/* Read Deserializer 1 Temp */
		ds90ubxxx_write_reg(client, desAlias1,  0x40, 0x6c);
		ds90ubxxx_write_reg(client, desAlias1,  0x41, 0xd);
		ds90ubxxx_write_reg(client, desAlias1,  0x42, 0x0);
		ds90ubxxx_write_reg(client, desAlias1,  0x41, 0x13);
		TEMP_FINAL = ds90ubxxx_read_reg(client, desAlias1, 0x42);
		TEMP_FINAL_C = 2*TEMP_FINAL - 273;

		/* Set up Deserializer 0 Temp Ramp Optimizations */
		Efuse_TS_CODE = 2;
		Ramp_UP_Range_CODES_Needed = (int)((150-TEMP_FINAL_C)/(190/11)) + 1;
		Ramp_DN_Range_CODES_Needed = (int)((TEMP_FINAL_C-30)/(190/11)) + 1;
		Ramp_UP_CAP_DELTA = Ramp_UP_Range_CODES_Needed - 4;
		Ramp_DN_CAP_DELTA = Ramp_DN_Range_CODES_Needed - 7;
		ds90ubxxx_write_reg(client, desAlias1, 0x40, 0x3c);
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0xf5);
		ds90ubxxx_write_reg(client, desAlias1, 0x42, (Efuse_TS_CODE<<4)+1) ;// Override TS_CODE Efuse Code;
		if (Ramp_UP_CAP_DELTA > 0) {
			TS_CODE_UP = Efuse_TS_CODE - Ramp_UP_CAP_DELTA;
			if (TS_CODE_UP < 0)
				TS_CODE_UP = 0;
			ds90ubxxx_write_reg(client, desAlias1, 0x41, 0xf5);
			rb = ds90ubxxx_read_reg(client, desAlias1, 0x42);
			rb &= 0x8F;
			rb |= (TS_CODE_UP << 4);
			ds90ubxxx_write_reg(client, desAlias1, 0x42, rb);
			rb = ds90ubxxx_read_reg(client, desAlias1, 0x42);
			rb &= 0xFE;
			rb |= 0x01;
			ds90ubxxx_write_reg(client, desAlias1, 0x42, rb);
			ds90ubxxx_write_reg(client, desAlias1, 0x1, 0x1);

			mdelay(0.04*1000);
		}
		if (Ramp_DN_CAP_DELTA > 0) {
			TS_CODE_DN = Efuse_TS_CODE + Ramp_DN_CAP_DELTA;
			if (TS_CODE_DN >= 7)
				TS_CODE_DN = 7;
			ds90ubxxx_write_reg(client, desAlias1, 0x41, 0xf5);
			rb = ds90ubxxx_read_reg(client, desAlias1, 0x42);
			rb &= 0x8F;
			rb |= (TS_CODE_DN << 4);
			ds90ubxxx_write_reg(client, desAlias1, 0x42, rb);
			rb = ds90ubxxx_read_reg(client, desAlias1, 0x42);
			rb &= 0xFE;
			rb |= 0x01;
			ds90ubxxx_write_reg(client, desAlias1, 0x42, rb);
			ds90ubxxx_write_reg(client, desAlias1, 0x1, 0x1);

			mdelay(0.04*1000);
		}
	}

	/* Clear CRC errors from initial link process */
	Reg_value = ds90ubxxx_read_reg(client, serAddr, 0x2);
	Reg_value = Reg_value | 0x20;
	ds90ubxxx_write_reg(client, serAddr, 0x2, Reg_value); //CRC Error Reset
	Reg_value = ds90ubxxx_read_reg(client, serAddr, 0x2);
	Reg_value = Reg_value & 0xdf;
	ds90ubxxx_write_reg(client, serAddr, 0x2, Reg_value); //CRC Error Reset Clear
	ds90ubxxx_write_reg(client, serAddr, 0x2d, 0x1);

	if (ds90ubxxx_i2c->des_dectected[0]) {
		/* Hold Des DTG in reset */
		ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x50); //Select DTG Page
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x32);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x6); //Hold Port 0 DTG in reset
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x62);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x6); //Hold Port 1 DTG in reset

		/* Disable Stream Mapping */
		ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x3); //Select both Output Ports
		ds90ubxxx_write_reg(client, desAlias0, 0xd0, 0x0); //Disable FPD4 video forward to Output Port
		ds90ubxxx_write_reg(client, desAlias0, 0xd7, 0x0); //Disable FPD3 video forward to Output Port

		/* Force DP Rate */
		ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x2c); //Select DP Page
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x81);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x60); //Set DP Rate to 2.7Gbps
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x82);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x3); //Enable force DP rate with calibration disabled
		ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x2c); //Select DP Page
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x91);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0xc); //Force 4 lanes
		ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x30); //Disable DP SSCG
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0xf);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x1, 0x40);

		/* Setup DP ports */
		ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x12); //Select Port 1 registers
		ds90ubxxx_write_reg(client, desAlias0, 0x46, 0x0); //Disable DP Port 1
		ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x1); //Select Port 0 registers
		ds90ubxxx_write_reg(client, desAlias0, 0x1, 0x40); //DP-TX-PLL RESET Applied

		/* Map video to display output */
		ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x3); //Select both Output Ports
		ds90ubxxx_write_reg(client, desAlias0, 0xd0, 0xc); //Enable FPD_RX video forward to Output Port
		ds90ubxxx_write_reg(client, desAlias0, 0xd1, 0xf); //Every stream forwarded on DC
		ds90ubxxx_write_reg(client, desAlias0, 0xd6, 0x0); //Send Stream 0 to Output Port 0 and Send Stream 0 to Output Port 1
		ds90ubxxx_write_reg(client, desAlias0, 0xd7, 0x0); //FPD3 mapping disabled
		ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x1); //Select Port 0

		/* Program quad pixel clock for DP port 0 */
		ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x1); //Select Port0 registers
		ds90ubxxx_write_reg(client, desAlias0, 0xb1, 0x1); //Enable clock divider
		ds90ubxxx_write_reg(client, desAlias0, 0xb2, 0xb0); //Program M value lower byte
		ds90ubxxx_write_reg(client, desAlias0, 0xb3, 0x1e); //Program M value middle byte
		ds90ubxxx_write_reg(client, desAlias0, 0xb4, 0x4); //Program M value upper byte
		ds90ubxxx_write_reg(client, desAlias0, 0xb5, 0xc0); //Program N value lower byte
		ds90ubxxx_write_reg(client, desAlias0, 0xb6, 0x7a); //Program N value middle byte
		ds90ubxxx_write_reg(client, desAlias0, 0xb7, 0x10); //Program N value upper byte
		ds90ubxxx_write_reg(client, desAlias0, 0xe, 0x1); //Select Port 0 registers

		/* Setup DTG for port 0 */
		ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x50); //Select DTG Page
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x20);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x93); //Set up Local Display DTG BPP, Sync Polarities, and Measurement Type
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x29); //Set Hstart;
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x80); //Hstart upper byte
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x2a);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x70); //Hstart lower byte
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x2f); //Set HSW
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x40); //HSW upper byte
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x30);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x38); //HSW lower byte

		/* Program DPTX for DP port 0 */
		ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1); //Enable APB interface
		ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xa4); //Set bit per color
		ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x20);
		ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xb8); //Set pixel width
		ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x4);
		ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xac); //Set DP Mvid
		ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x80);
		ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xb4); //Set DP Nvid
		ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x80);
		ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xc8); //Set TU Mode
		ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xb0); //Set TU Size
		ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x40);
		ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x30);
		ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xc8); //Set FIFO Size
		ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x5);
		ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x40);
		ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xbc); //Set data count
		ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x80);
		ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x7);
		ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xc0); //Disable STREAM INTERLACED
		ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x49, 0xc4); //Set SYNC polarity
		ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0xc);
		ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);

		/* Release Des DTG reset */
		ds90ubxxx_write_reg(client, desAlias0, 0x40, 0x50); //Select DTG Page
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x32);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x4); //elease Port 0 DTG
		ds90ubxxx_write_reg(client, desAlias0, 0x41, 0x62);
		ds90ubxxx_write_reg(client, desAlias0, 0x42, 0x4); //elease Port 1 DTG
		ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x49, 0x80); //Set Htotal
		ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0xc4);
		ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0xa);
		ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);

		/* Enable DP 0 output */
		ds90ubxxx_write_reg(client, desAlias0, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x49, 0x84); //Enable DP output
		ds90ubxxx_write_reg(client, desAlias0, 0x4a, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4b, 0x1);
		ds90ubxxx_write_reg(client, desAlias0, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias0, 0x4e, 0x0);
	}

	if (ds90ubxxx_i2c->des_dectected[1]) {
		/* Hold Des DTG in reset */
		ds90ubxxx_write_reg(client, desAlias1, 0x40, 0x50); //Select DTG Page
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0x32);
		ds90ubxxx_write_reg(client, desAlias1, 0x42, 0x6); //Hold Port 0 DTG in reset
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0x62);
		ds90ubxxx_write_reg(client, desAlias1, 0x42, 0x6); //Hold Port 1 DTG in reset

		/* Disable Stream Mapping */
		ds90ubxxx_write_reg(client, desAlias1, 0xe, 0x3); //Select both Output Ports
		ds90ubxxx_write_reg(client, desAlias1, 0xd0, 0x0); //Disable FPD4 video forward to Output Port
		ds90ubxxx_write_reg(client, desAlias1, 0xd7, 0x0); //Disable FPD3 video forward to Output Port

		/* Force DP Rate */
		ds90ubxxx_write_reg(client, desAlias1, 0x40, 0x2c); //Select DP Page
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0x81);
		ds90ubxxx_write_reg(client, desAlias1, 0x42, 0x60); //Set DP Rate to 2.7Gbps
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0x82);
		ds90ubxxx_write_reg(client, desAlias1, 0x42, 0x3); //Enable force DP rate with calibration disabled
		ds90ubxxx_write_reg(client, desAlias1, 0x40, 0x2c); //Select DP Page
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0x91);
		ds90ubxxx_write_reg(client, desAlias1, 0x42, 0xc); //Force 4 lanes
		ds90ubxxx_write_reg(client, desAlias1, 0x40, 0x30); //Disable DP SSCG
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0xf);
		ds90ubxxx_write_reg(client, desAlias1, 0x42, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x1, 0x40);

		/* Setup DP ports */
		ds90ubxxx_write_reg(client, desAlias1, 0xe, 0x12); //Select Port 1 registers
		ds90ubxxx_write_reg(client, desAlias1, 0x46, 0x0); //Disable DP Port 1
		ds90ubxxx_write_reg(client, desAlias1, 0xe, 0x1); //Select Port 0 registers
		ds90ubxxx_write_reg(client, desAlias1, 0x1, 0x40); //DP-TX-PLL RESET Applied

		/* Map video to display output */
		ds90ubxxx_write_reg(client, desAlias1, 0xe, 0x3); //Select both Output Ports
		ds90ubxxx_write_reg(client, desAlias1, 0xd0, 0xc); //Enable FPD_RX video forward to Output Port
		ds90ubxxx_write_reg(client, desAlias1, 0xd1, 0xf); //Every stream forwarded on DC
		ds90ubxxx_write_reg(client, desAlias1, 0xd6, 0x9); //Send Stream 0 to Output Port 0 and Send Stream 0 to Output Port 1
		ds90ubxxx_write_reg(client, desAlias1, 0xd7, 0x0); //FPD3 to local display output mapping disabled
		ds90ubxxx_write_reg(client, desAlias1, 0xe, 0x1); //Select Port 0

		/* Program quad pixel clock for DP port 0 */
		ds90ubxxx_write_reg(client, desAlias1, 0xe, 0x1); //Select Port0 registers
		ds90ubxxx_write_reg(client, desAlias1, 0xb1, 0x1); //Enable clock divider
		ds90ubxxx_write_reg(client, desAlias1, 0xb2, 0xb0); //Program M value lower byte
		ds90ubxxx_write_reg(client, desAlias1, 0xb3, 0x1e); //Program M value middle byte
		ds90ubxxx_write_reg(client, desAlias1, 0xb4, 0x4); //Program M value upper byte
		ds90ubxxx_write_reg(client, desAlias1, 0xb5, 0xc0); //Program N value lower byte
		ds90ubxxx_write_reg(client, desAlias1, 0xb6, 0x7a); //Program N value middle byte
		ds90ubxxx_write_reg(client, desAlias1, 0xb7, 0x10); //Program N value upper byte
		ds90ubxxx_write_reg(client, desAlias1, 0xe, 0x1); //Select Port 0 registers

		/* Setup DTG for port 0 */
		ds90ubxxx_write_reg(client, desAlias1, 0x40, 0x50); //Select DTG Page
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0x20);
		ds90ubxxx_write_reg(client, desAlias1, 0x42, 0x93); //Set up Local Display DTG BPP, Sync Polarities, and Measurement Type
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0x29); //Set Hstart
		ds90ubxxx_write_reg(client, desAlias1, 0x42, 0x80); //Hstart upper byte
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0x2a);
		ds90ubxxx_write_reg(client, desAlias1, 0x42, 0x70); //Hstart lower byte
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0x2f); //Set HSW
		ds90ubxxx_write_reg(client, desAlias1, 0x42, 0x40); //HSW upper byte
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0x30);
		ds90ubxxx_write_reg(client, desAlias1, 0x42, 0x38); //HSW lower byte

		/* Program DPTX for DP port 0 */
		ds90ubxxx_write_reg(client, desAlias1, 0x48, 0x1); //Enable APB interface
		ds90ubxxx_write_reg(client, desAlias1, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x49, 0xa4); //Set bit per color
		ds90ubxxx_write_reg(client, desAlias1, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x4b, 0x20);
		ds90ubxxx_write_reg(client, desAlias1, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x49, 0xb8); //Set pixel width
		ds90ubxxx_write_reg(client, desAlias1, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x4b, 0x4);
		ds90ubxxx_write_reg(client, desAlias1, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x49, 0xac); //Set DP Mvid
		ds90ubxxx_write_reg(client, desAlias1, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x4b, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4c, 0x80);
		ds90ubxxx_write_reg(client, desAlias1, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x49, 0xb4); //Set DP Nvid
		ds90ubxxx_write_reg(client, desAlias1, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x4b, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4c, 0x80);
		ds90ubxxx_write_reg(client, desAlias1, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x49, 0xc8); //Set TU Mode
		ds90ubxxx_write_reg(client, desAlias1, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x4b, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x49, 0xb0); //Set TU Size
		ds90ubxxx_write_reg(client, desAlias1, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x4b, 0x40);
		ds90ubxxx_write_reg(client, desAlias1, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4d, 0x30);
		ds90ubxxx_write_reg(client, desAlias1, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x49, 0xc8); //Set FIFO Size
		ds90ubxxx_write_reg(client, desAlias1, 0x4a, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4b, 0x5);
		ds90ubxxx_write_reg(client, desAlias1, 0x4c, 0x40);
		ds90ubxxx_write_reg(client, desAlias1, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x49, 0xbc); //Set data count
		ds90ubxxx_write_reg(client, desAlias1, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x4b, 0x80);
		ds90ubxxx_write_reg(client, desAlias1, 0x4c, 0x7);
		ds90ubxxx_write_reg(client, desAlias1, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x49, 0xc0); //Disable STREAM INTERLACED
		ds90ubxxx_write_reg(client, desAlias1, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x4b, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4e, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x49, 0xc4); //Set SYNC polarity
		ds90ubxxx_write_reg(client, desAlias1, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x4b, 0xc);
		ds90ubxxx_write_reg(client, desAlias1, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4e, 0x0);

		/* Release Des DTG reset */
		ds90ubxxx_write_reg(client, desAlias1, 0x40, 0x50); //Select DTG Page
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0x32);
		ds90ubxxx_write_reg(client, desAlias1, 0x42, 0x4); //Release Local Display Output Port 0 DTG
		ds90ubxxx_write_reg(client, desAlias1, 0x41, 0x62);
		ds90ubxxx_write_reg(client, desAlias1, 0x42, 0x4); //Release Local Display Output Port 1 DTG
		ds90ubxxx_write_reg(client, desAlias1, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x49, 0x80); //Set Htotal
		ds90ubxxx_write_reg(client, desAlias1, 0x4a, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x4b, 0xc4);
		ds90ubxxx_write_reg(client, desAlias1, 0x4c, 0xa);
		ds90ubxxx_write_reg(client, desAlias1, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4e, 0x0);

		/* Enable DP 0 output */
		ds90ubxxx_write_reg(client, desAlias1, 0x48, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x49, 0x84); //Enable DP output
		ds90ubxxx_write_reg(client, desAlias1, 0x4a, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4b, 0x1);
		ds90ubxxx_write_reg(client, desAlias1, 0x4c, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4d, 0x0);
		ds90ubxxx_write_reg(client, desAlias1, 0x4e, 0x0);
	}

	return 0;
}

static unsigned char ub984_main_init_reg[][6] = {
	//addr, reg, value, rbdelay, retry, delay(defore)
	{DES0ADDR, 0x12, 0xdc, 0x01, 0x05, 0}, //GPIO IN CLEAN
	{DES0ADDR, 0x13, 0x00, 0x01, 0x05, 0}, //GPIO IN CLEAN

	{DES0ADDR, 0x15, 0xc0, 0x01, 0x05, 0}, //vgh_en GPIO0 | GPIO0_PIN_CTL Register (Address = 0x15)
	{DES0ADDR, 0x16, 0xc0, 0x01, 0x05, 0}, //vgl_en GPIO1 | GPIO1_PIN_CTL Register (Address = 0x16)
	{DES0ADDR, 0x17, 0xc0, 0x01, 0x05, 0},
	{DES0ADDR, 0x18, 0xc0, 0x01, 0x05, 0},
	{DES0ADDR, 0x19, 0xc0, 0x01, 0x05, 0}, //tft_reset GPIO4 | GPIO4_PIN_CTL Register (Address = 0x19)
	{DES0ADDR, 0x1a, 0xc0, 0x01, 0x05, 0}, //tp_reset  GPIO5 | GPIO5_PIN_CTL Register (Address = 0x1A)
	{DES0ADDR, 0x1b, 0xc0, 0x01, 0x05, 0}, //tft_pon   GPIO6 | GPIO6_PIN_CTL Register (Address = 0x1B)
	{DES0ADDR, 0x1c, 0xc0, 0x01, 0x05, 0}, //bl_en     GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)
	{DES0ADDR, 0x1d, 0xc0, 0x01, 0x05, 0},
	{DES0ADDR, 0x1e, 0xc0, 0x01, 0x05, 0},
	{DES0ADDR, 0x22, 0xc0, 0x01, 0x05, 0},

	{0x00,     0x00, 0x00, 0x00, 0x00, 30}, //delay 20ms
	{DES0ADDR, 0x17, 0xc1, 0x01, 0x05, 0x0a}, //vsp_en     GPIO2  | GPIO2_PIN_CTL Register (Address = 0x17)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES0ADDR, 0x22, 0xc1, 0x01, 0x05, 0x00}, //tcon_reset GPIO13 | GPIO13_PIN_CTL Register (Address = 0x22)
	{DES0ADDR, 0x18, 0xc1, 0x01, 0x05, 0x00}, //vsn_en	 GPIO3  | GPIO3_PIN_CTL Register (Address = 0x18)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES0ADDR, 0x15, 0xc1, 0x01, 0x05, 0x00}, //vgh_en     GPIO0  | GPIO0_PIN_CTL Register (Address = 0x15)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES0ADDR, 0x16, 0xc1, 0x01, 0x05, 0x00}, //vgl_en     GPIO1  | GPIO1_PIN_CTL Register (Address = 0x16)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES0ADDR, 0x19, 0xc1, 0x01, 0x05, 0x00}, //tft_reset  GPIO4  | GPIO4_PIN_CTL Register (Address = 0x1)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES0ADDR, 0x1a, 0xc1, 0x01, 0x05, 0x00}, //tp_reset   GPIO5  | GPIO5_PIN_CTL Register (Address = 0x1A)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x96}, //delay 150ms
	{DES0ADDR, 0x1b, 0xc1, 0x01, 0x05, 0x00}, //tft_pon    GPIO6  | GPIO6_PIN_CTL Register (Address = 0x1B)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES0ADDR, 0x1c, 0xc1, 0x01, 0x05, 0x00}, //bl_en      GPIO7  | GPIO7_PIN_CTL Register (Address = 0x1C)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES0ADDR, 0x1d, 0xc1, 0x00, 0x05, 0x00}, //pwm        GPIO7  | GPIO7_PIN_CTL Register (Address = 0x1C)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES0ADDR, 0x1e, 0xc0, 0x01, 0x05, 0x00}, //bist_en    GPIO7  | GPIO7_PIN_CTL Register (Address = 0x1C)

	{DES0ADDR, 0x1f, 0x00, 0x01, 0x05, 0x00}, //TFT_FAULT_DET_M GPIO10 | GPIO10_PIN_CTL Register (Address = 0x1F) - disable
	{DES0ADDR, 0x20, 0x00, 0x01, 0x05, 0x00}, //BL_FAULT        GPIO11 | GPIO11_PIN_CTL Register (Address = 0x20) - disable
	{DES0ADDR, 0x21, 0x00, 0x01, 0x05, 0x00}, //ADC_READY       GPIO12 | GPIO12_PIN_CTL Register (Address = 0x21) - disable

	{DES0ADDR, 0x12, 0x1c, 0x01, 0x05, 0x00}, //GPIO_IN_EN1     GPIO8-13 | GPIO_IN_EN1 Register (Address = 0x12)

	{DES0ADDR, 0x44, 0x81, 0x01, 0x05, 0x14}, //RX_INT_CTL Register
};

static unsigned char ub984_sub_init_reg[][6] = {
	{DES1ADDR, 0x12, 0xdc, 0x01, 0x05, 0}, //GPIO IN CLEAN
	{DES1ADDR, 0x13, 0x00, 0x01, 0x05, 0}, //GPIO IN CLEAN

	{DES1ADDR, 0x15, 0xc0, 0x01, 0x05, 0}, //vgh_en    GPIO0 | GPIO0_PIN_CTL Register (Address = 0x15)
	{DES1ADDR, 0x16, 0xc0, 0x01, 0x05, 0}, //vgl_en    GPIO1 | GPIO1_PIN_CTL Register (Address = 0x16)
	{DES1ADDR, 0x17, 0xc0, 0x01, 0x05, 0},
	{DES1ADDR, 0x18, 0xc0, 0x01, 0x05, 0},
	{DES1ADDR, 0x19, 0xc0, 0x01, 0x05, 0}, //tft_reset GPIO4 | GPIO4_PIN_CTL Register (Address = 0x19)
	{DES1ADDR, 0x1a, 0xc0, 0x01, 0x05, 0}, //tp_reset  GPIO5 | GPIO5_PIN_CTL Register (Address = 0x1A)
	{DES1ADDR, 0x1b, 0xc0, 0x01, 0x05, 0}, //tft_pon   GPIO6 | GPIO6_PIN_CTL Register (Address = 0x1B)
	{DES1ADDR, 0x1c, 0xc0, 0x01, 0x05, 0}, //bl_en     GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)
	{DES1ADDR, 0x1d, 0xc0, 0x01, 0x05, 0},
	{DES1ADDR, 0x1e, 0xc0, 0x01, 0x05, 0},
	{DES1ADDR, 0x22, 0xc0, 0x01, 0x05, 0},

	{0x00,     0x00, 0x00, 0x00, 0x00, 30}, //delay 20ms
	{DES1ADDR, 0x17, 0xc1, 0x01, 0x05, 0x0a}, //vsp_en    GPIO2 | GPIO2_PIN_CTL Register (Address = 0x17)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES1ADDR, 0x22, 0xc1, 0x01, 0x05, 0x00}, //tcon_resetGPIO13 | GPIO13_PIN_CTL Register (Address = 0x22)
	{DES1ADDR, 0x18, 0xc1, 0x01, 0x05, 0x00}, //vsn_en     GPIO3 | GPIO3_PIN_CTL Register (Address = 0x18)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES1ADDR, 0x15, 0xc1, 0x01, 0x05, 0x00}, //vgh_en     GPIO0 | GPIO0_PIN_CTL Register (Address = 0x15)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES1ADDR, 0x16, 0xc1, 0x01, 0x05, 0x00}, //vgl_en     GPIO1 | GPIO1_PIN_CTL Register (Address = 0x16)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES1ADDR, 0x19, 0xc1, 0x01, 0x05, 0x00}, //tft_reset  GPIO4 | GPIO4_PIN_CTL Register (Address = 0x1)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES1ADDR, 0x1a, 0xc1, 0x01, 0x05, 0x00}, //tp_reset   GPIO5 | GPIO5_PIN_CTL Register (Address = 0x1A)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x96}, //delay 150ms
	{DES1ADDR, 0x1b, 0xc1, 0x01, 0x05, 0x00}, //tft_pon    GPIO6 | GPIO6_PIN_CTL Register (Address = 0x1B)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES1ADDR, 0x1c, 0xc1, 0x01, 0x05, 0x00}, //bl_en      GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES1ADDR, 0x1d, 0xc1, 0x00, 0x05, 0x00}, //pwm        GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)

	{0x00,     0x00, 0x00, 0x00, 0x00, 0x14}, //delay 20ms
	{DES1ADDR, 0x1e, 0xc0, 0x01, 0x05, 0x00}, //bist_en    GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)

	{DES1ADDR, 0x1f, 0x00, 0x01, 0x05, 0x00}, //TFT_FAULT_DET_M  GPIO10 | GPIO10_PIN_CTL Register (Address = 0x1F) - disable
	{DES1ADDR, 0x20, 0x00, 0x01, 0x05, 0x00}, //BL_FAULT         GPIO11 | GPIO11_PIN_CTL Register (Address = 0x20) - disable
	{DES1ADDR, 0x21, 0x00, 0x01, 0x05, 0x00}, //ADC_READY        GPIO12 | GPIO12_PIN_CTL Register (Address = 0x21) - disable

	{DES1ADDR, 0x12, 0x1c, 0x01, 0x05, 0x00}, //GPIO_IN_EN1    GPIO8-13 | GPIO_IN_EN1 Register (Address = 0x12)

	{DES1ADDR, 0x44, 0x81, 0x01, 0x05, 0x14}, //RX_INT_CTL Register
};

unsigned char ub983_dual_csd_dev_init_reg[][6] = {
	{SERADDR, 0x2d, 0x01, 0x01, 0x05, 0x00},
	{SERADDR, 0xc6, 0x21, 0x01, 0x05, 0x00},
	{SERADDR, 0x2d, 0x12, 0x01, 0x05, 0x00},
	{SERADDR, 0xc6, 0x21, 0x01, 0x05, 0x00},
	{SERADDR, 0x1b, 0x88, 0x01, 0x05, 0x00},
	{SERADDR, 0x20, 0x98, 0x01, 0x05, 0x00},
	{SERADDR, 0x51, 0x83, 0x01, 0x05, 0x00},
};

unsigned char ub983_dual_csd_subdev_init_reg[][6] = {
	{SERADDR, 0x72, 0x90, 0x01, 0x05, 0x00}, //touchscreen i2c 8bit 0x90
	{SERADDR, 0x7a, 0xb0, 0x01, 0x05, 0x00},
	{SERADDR, 0x8a, 0x00, 0x01, 0x05, 0x00},

	{SERADDR, 0x76, 0x90, 0x01, 0x05, 0x00}, //touchscreen i2c 8bit 0x90
	{SERADDR, 0x7e, 0xb3, 0x01, 0x05, 0x00},
	{SERADDR, 0x8e, 0x00, 0x01, 0x05, 0x00},
};

unsigned char ub983_csd_dev_init_reg[][6] = {
	{SERADDR, 0x2d, 0x01, 0x01, 0x05, 0x00},
	{SERADDR, 0xc6, 0x21, 0x01, 0x05, 0x00},
	{SERADDR, 0x1b, 0x88, 0x01, 0x05, 0x00},
	{SERADDR, 0x51, 0x83, 0x01, 0x05, 0x00},
};

unsigned char ub983_csd_subdev_init_reg[][6] = {
	{SERADDR, 0x71, 0x90, 0x01, 0x05, 0x00}, //touchscreen i2c 8bit 0x90
	{SERADDR, 0x79, 0xb0, 0x01, 0x05, 0x00},
	{SERADDR, 0x89, 0x00, 0x01, 0x05, 0x00},
};

static bool ub983_i2c_read_register(struct i2c_client *client,
			unsigned char addr, unsigned char reg, unsigned char *data)
{
	int retry = 0;
	int ret = 0;
	unsigned char buf;
	struct i2c_msg msg[2] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= &buf,
		}
	};

	client->addr = addr;

	for (retry = 3; retry > 0; retry--) {
		ret = i2c_transfer(client->adapter, msg, 2);
		if (ret != 2) {
			pr_err("%s failed to read {0x%02x,0x%02x}\n",
				__func__, client->addr, reg);
		} else {
			*data = buf;
			return true;
		}
	}

	return false;
}

static bool ub983_i2c_write_register(struct i2c_client *client,
			unsigned char addr, unsigned char reg, unsigned char data)
{
	u8 tmp[2];
	struct i2c_msg msg;
	int ret;

	//mdelay(10);

	tmp[0] = reg;
	tmp[1] = data;

	client->addr = addr;

	msg.addr = client->addr;
	msg.flags = 0; //write
	msg.buf = tmp;
	msg.len = 2;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1)
		return false;

	return true;
}

static bool ub983_i2c_write_with_readback(struct i2c_client *client,
			unsigned char addr, unsigned char reg, unsigned char data,
			int rbdelay, char retry)
{
	bool ret = false;
	unsigned char readback = 0;
	int remain = 0;

	for (remain = retry+1; remain > 0; remain--) {
		ret = ub983_i2c_write_register(client, addr, reg, data);
		if (ret == true) {
			if (rbdelay > 0)
				mdelay(rbdelay);

			ret = ub983_i2c_read_register(client, addr, reg, &readback);
			if (ret == true && readback == data)
				return true;
		}
	}

	pr_err("%s failed to write {0x%02x,0x%02x,0x%02x}\n",
		__func__, addr, reg, data);
	return false;
}

static int ub98x_reg_array_write(struct i2c_client *client,
			unsigned char (*array)[6], unsigned int length)
{
	int i = 0;

	for (i = 0; i < length; i++) {
		if (array[i][5] > 0)
			mdelay(array[i][5]);

		if (array[i][0] == 0)
			continue;

		ub983_i2c_write_with_readback(client, array[i][0], array[i][1],
			array[i][2], array[i][3], array[i][4]);
	}

	return 0;
}

static int ds90ubxxx_i2c_probe(struct i2c_client *i2c_client,
			const struct i2c_device_id *id)
{
	int ret = 0;
	struct device *dev = &i2c_client->dev;
	struct device_node *i2c_np = dev->of_node;
	struct ds90ubxxx_i2c *ds90ubxxx_i2c;

	ds90ubxxx_i2c =  devm_kzalloc(dev, sizeof(*ds90ubxxx_i2c), GFP_KERNEL);
	if (ds90ubxxx_i2c == NULL)
		return -ENOMEM;
	ds90ubxxx_i2c->i2c_client = i2c_client;
	ds90ubxxx_i2c->ser_addr = i2c_client->addr;

	ret = of_property_read_string(i2c_np, "serializer-type",
		&ds90ubxxx_i2c->serializer_type);
	if (ret < 0) {
		pr_err("%s failed to get serializer-type\n", __func__);
		return -EINVAL;
	}

	ret = of_property_read_u32(i2c_np, "des-addr", &ds90ubxxx_i2c->des_addr);
	if (ret < 0) {
		pr_err("%s failed to get des-addr\n", __func__);
		return -EINVAL;
	}

	dev_set_drvdata(dev, ds90ubxxx_i2c);

	if (!(strcmp(ds90ubxxx_i2c->serializer_type, "dx11-dual-csd"))) {
		if (dx11_dual_csd_config(ds90ubxxx_i2c) < 0)
			goto END;

		if (ds90ubxxx_i2c->des_dectected[0])
			ub98x_reg_array_write(i2c_client, ub984_main_init_reg,
				ARRAY_SIZE(ub984_main_init_reg));

		if (ds90ubxxx_i2c->des_dectected[1])
			ub98x_reg_array_write(i2c_client, ub984_sub_init_reg,
				ARRAY_SIZE(ub984_sub_init_reg));

		ub98x_reg_array_write(i2c_client, ub983_dual_csd_dev_init_reg,
			ARRAY_SIZE(ub983_dual_csd_dev_init_reg));

		ub98x_reg_array_write(i2c_client, ub983_dual_csd_subdev_init_reg,
			ARRAY_SIZE(ub983_dual_csd_subdev_init_reg));
	} else if (!(strcmp(ds90ubxxx_i2c->serializer_type, "dx11-csd"))) {
		if (dx11_csd_config(ds90ubxxx_i2c) < 0)
			goto END;

		ub98x_reg_array_write(i2c_client, ub984_main_init_reg,
			ARRAY_SIZE(ub984_main_init_reg));

		ub98x_reg_array_write(i2c_client, ub983_csd_dev_init_reg,
			ARRAY_SIZE(ub983_csd_dev_init_reg));

		ub98x_reg_array_write(i2c_client, ub983_csd_subdev_init_reg,
			ARRAY_SIZE(ub983_csd_subdev_init_reg));
	}

END:
	return 0;
}

static int ds90ubxxx_i2c_remove(struct i2c_client *i2c_client)
{
	return 0;
}

static int ds90ubxxx_panel_suspend(struct device *dev)
{
	return 0;
}

static int ds90ubxxx_panel_resume(struct device *dev)
{
	struct ds90ubxxx_i2c *ds90ubxxx_i2c = dev_get_drvdata(dev);
	struct i2c_client *i2c_client = ds90ubxxx_i2c->i2c_client;

	if (!(strcmp(ds90ubxxx_i2c->serializer_type, "dx11-dual-csd"))) {
		if (dx11_dual_csd_config(ds90ubxxx_i2c) < 0)
			goto END;

		if (ds90ubxxx_i2c->des_dectected[0])
			ub98x_reg_array_write(i2c_client, ub984_main_init_reg,
				ARRAY_SIZE(ub984_main_init_reg));

		if (ds90ubxxx_i2c->des_dectected[1])
			ub98x_reg_array_write(i2c_client, ub984_sub_init_reg,
				ARRAY_SIZE(ub984_sub_init_reg));

		ub98x_reg_array_write(i2c_client, ub983_dual_csd_dev_init_reg,
			ARRAY_SIZE(ub983_dual_csd_dev_init_reg));

		ub98x_reg_array_write(i2c_client, ub983_dual_csd_subdev_init_reg,
			ARRAY_SIZE(ub983_dual_csd_subdev_init_reg));
	} else if (!(strcmp(ds90ubxxx_i2c->serializer_type, "dx11-csd"))) {
		if (dx11_csd_config(ds90ubxxx_i2c) < 0)
			goto END;

		ub98x_reg_array_write(i2c_client, ub984_main_init_reg,
			ARRAY_SIZE(ub984_main_init_reg));

		ub98x_reg_array_write(i2c_client, ub983_csd_dev_init_reg,
			ARRAY_SIZE(ub983_csd_dev_init_reg));

		ub98x_reg_array_write(i2c_client, ub983_csd_subdev_init_reg,
			ARRAY_SIZE(ub983_csd_subdev_init_reg));
	}

END:
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ds90ubxxx_i2c_of_ids[] = {
	 { .compatible = "ds90ub983_i2c" },
	 { } /* sentinel */
};
MODULE_DEVICE_TABLE(of, ds90ubxxx_i2c_of_ids);
#endif

static const struct dev_pm_ops ds90ubxxx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ds90ubxxx_panel_suspend, ds90ubxxx_panel_resume)
};

static struct i2c_driver ds90ubxxx_i2c_driver = {
	.probe = ds90ubxxx_i2c_probe,
	.remove = ds90ubxxx_i2c_remove,
	.driver = {
		.name = "ds90ubxxx_i2c",
		.of_match_table = ds90ubxxx_i2c_of_ids,
		.pm = &ds90ubxxx_pm_ops,
	}
};

module_i2c_driver(ds90ubxxx_i2c_driver);

MODULE_AUTHOR("Siengine");
MODULE_DESCRIPTION("PANEL DS90UBXXX CSD");
MODULE_LICENSE("GPL v2");

