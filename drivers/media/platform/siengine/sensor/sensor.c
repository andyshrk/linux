/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */

#include "sensor.h"


int sensor_read_reg8(struct i2c_client *i2c_client, unsigned short addr,
		unsigned short reg, unsigned char *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char tx_buf[2], rx_buf[1];;
	unsigned short addr_bak = i2c_client->addr;

	i2c_client->addr = addr;

	/* high byte goes out first */
	tx_buf[0] = (u8)(reg >> 8);
	tx_buf[1] = (u8)(reg & 0xff);

	msg[0].addr = i2c_client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = (char *)tx_buf;

	msg[1].addr = i2c_client->addr;
	msg[1].len = 1;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (char *)rx_buf;

	err = i2c_transfer(i2c_client->adapter, msg, 2);
	if (err != 2) {
		if (err >= 0) {
			err = -EIO;
		}
		dev_err(&i2c_client->dev,
				"read from offset 0x%x error %d\n", reg, err);
		return err;
	}
	*val = rx_buf[0];
	dev_info(&i2c_client->dev,
			 "read from {0x%x,val=0x%x\n", reg, *val);

	i2c_client->addr = addr_bak;

	return 0;
}
int sensor_write_reg8(struct i2c_client *i2c_client, unsigned short addr,
		unsigned short reg, unsigned char val)
{
	int err;
	struct i2c_msg msg;
	u8 data[3];
	unsigned short addr_bak = i2c_client->addr;

	i2c_client->addr = addr;

	msg.addr = i2c_client->addr;
	msg.buf = data;
	msg.len = 3;
	msg.flags = 0;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;
	data[2] = val;

	err = i2c_transfer(i2c_client->adapter, &msg, 1);

	if (err != 1) {
		return 0;
	}

#if WRITE_VERIFY
	unsigned char val_r;
	sensor_read_reg8(i2c_client, 0, reg, &val_r);
	if (val_r != val)
		dev_err(&i2c_client->dev,
			"%s:check error: {0x%x, val=0x%x, val_r=0x%x.\n", __func__, reg, val, val_r);
#endif
	i2c_client->addr = addr_bak;
	return 0;
}
int sensor_read_reg16(struct i2c_client *i2c_client, unsigned short addr,
		unsigned short reg, unsigned short *val)
{
	int err;
	struct i2c_msg msg[2];
	u8 tx_buf[2], rx_buf[2];
	unsigned short addr_bak = i2c_client->addr;

	i2c_client->addr = addr;

	/* high byte goes out first */
	tx_buf[0] = (u8)(reg >> 8);
	tx_buf[1] = (u8)(reg & 0xff);

	msg[0].addr = i2c_client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = (char *)tx_buf;

	msg[1].addr = i2c_client->addr;
	msg[1].len = 2;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (char *)rx_buf;

	err = i2c_transfer(i2c_client->adapter, msg, 2);
	if (err != 2) {
		if (err >= 0) {
			err = -EIO;
		}
		dev_err(&i2c_client->dev,
				"read from offset 0x%x error %d\n", reg, err);
		return err;
	}

	*val = ((unsigned short)rx_buf[0] << 8) | rx_buf[1];
	dev_info(&i2c_client->dev,
			 "read from {0x%x,val=0x%x\n", reg, *val);

	i2c_client->addr = addr_bak;

	return 0;
}

int sensor_write_reg16(struct i2c_client *i2c_client, unsigned short addr,
		unsigned short reg, unsigned short val)
{
	int err;
	struct i2c_msg msg;
	u8 data[4];
	unsigned short addr_bak = i2c_client->addr;

	i2c_client->addr = addr;

	msg.addr = i2c_client->addr;
	msg.buf = data;
	msg.len = 4;
	msg.flags = 0;

	data[0] = (reg >> 8) & 0xff;
	data[1] = reg & 0xff;
	data[2] = (val >> 8) & 0xff;
	data[3] = val & 0xff;
	err = i2c_transfer(i2c_client->adapter, &msg, 1);

	if (err != 1) {
		return 0;
	}

#if WRITE_VERIFY
	unsigned short val_r;
	sensor_read_reg16(i2c_client, 0, reg, &val_r);
	if (val_r != val)
		dev_err(&i2c_client->dev,
			"%s:check error: {0x%x, val=0x%x, val_r=0x%x.\n", __func__, reg, val, val_r);
#endif
	i2c_client->addr = addr_bak;
	return 0;
}


