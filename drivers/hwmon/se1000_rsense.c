#include <linux/module.h>
#include <linux/init.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/device.h>
#include "se1000_rsense.h"

#define RSENSE_MAX_ATTRIBUTE_GROUPS 1
#define RSENSE_CONFIG			0x0

enum RSENSE_TYPE{
	INA237,
	SQ52205,
};

int se1000_rsense_read(struct i2c_client *c, u8 reg, u32 *val)
{
	struct i2c_msg msg[2];
	u8 buf[3];
	int ret;

	msg[0].addr = c->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].addr = c->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 3;
	msg[1].buf = buf;

	ret = i2c_transfer(c->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&c->dev, "i2c read transfer error: %d\n", ret);
		return ret;
	}

	*val = ((buf[0] << 8) | (buf[1])) & 0xffff;

	return 0;
}

int se1000_rsense_write(struct i2c_client *c, u8 reg, u16 val)
{
	struct i2c_msg msg;
	u8 buf[3] = { 0 };
	int ret;

	buf[0] = reg;
	buf[1] = val >> 8;
	buf[2] = val & 0xff;

	msg.addr = c->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;

	ret = i2c_transfer(c->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&c->dev, "i2c write transfer error: %d\n", ret);
		return ret;
	}

	return 0;
}

static int se1000_get_rsense_type(struct i2c_client *client)
{
	unsigned int config;
	int ret;

	do {
		ret = se1000_rsense_write(client, RSENSE_CONFIG, 0x7);
		if (ret)
			break;

		ret = se1000_rsense_read(client, RSENSE_CONFIG, &config);
		if (ret)
			break;
	}while(0);

	if (ret)
		return ret;

	if (config & 0x7)
		ret = SQ52205;
	else
		ret = INA237;

	return ret;
}

static int se1000_rsense_probe(struct i2c_client *client)
{
	int ret;

	ret = se1000_get_rsense_type(client);
	switch(ret)
	{
		case INA237:
			ret = se1000_ina237_probe(client);
			break;
		case SQ52205:
			ret = se1000_sq52205_probe(client);
			break;
		default:
			return ret;
	}

	return ret;
}

static int se1000_rsense_resume(struct device *dev)
{
	int ret = -1;
	struct rsense* rsense;

	if (!dev)
		return -EINVAL;

	rsense = dev_get_drvdata(dev);
	if (!rsense || !rsense->client)
		return -EINVAL;

	ret = se1000_get_rsense_type(rsense->client);
	switch(ret)
	{
		case INA237:
			ret = se1000_ina237_init(rsense);
			break;
		case SQ52205:
			ret = se1000_sq52205_init(rsense);
			break;
		default:
			return ret;
	}

	return ret;
}

static int se1000_rsense_suspend(struct device *dev)
{
	return 0;
}

static const struct i2c_device_id se1000_rsense_id[] = {
	{ "se1000_rsense", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, se1000_rsense_id);

static const struct of_device_id __maybe_unused se1000_rsense_of_match[] = {
	{
		.compatible = "se1000, rsense",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, se1000_rsense_of_match);

const struct dev_pm_ops se1000_rsense_pm_ops = {
	.resume = se1000_rsense_resume,
	.suspend = se1000_rsense_suspend,
};

static struct i2c_driver se1000_rsense_driver = {
	.driver = {
		.name	= "se1000_rsense",
		.of_match_table = of_match_ptr(se1000_rsense_of_match),
		.pm = &se1000_rsense_pm_ops,
	},
	.probe_new	= se1000_rsense_probe,
	.id_table	= se1000_rsense_id,
};

module_i2c_driver(se1000_rsense_driver);

MODULE_AUTHOR("gloria.zhang@siengine.com");
MODULE_DESCRIPTION("Se1000 Rsense driver");
MODULE_LICENSE("GPL v2");