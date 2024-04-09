#include <linux/module.h>
#include <linux/init.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/device.h>
#include "se1000_rsense.h"

/*SQ52205 Registers*/
#define SQ52205_CONFIG			0x00
#define SQ52205_SHUNT_VOLTAGE		0x01 /* readonly */
#define SQ52205_BUS_VOLTAGE		0x02 /* readonly */
#define SQ52205_POWER			0x03 /* readonly */
#define SQ52205_CURRENT			0x04 /* readonly */
#define SQ52205_CALIBRATION		0x05
#define SQ52205_MASK_ENABLE		0x06
#define SQ52205_ALERT_LIMIT		0x07
#define SQ52205_EIN			0x0A
#define SQ52205_ACCUM_CONFIG		0x0D

#define SQ52205_CONFIG_DEFAULT		0x4127

static int se1000_sq52205_get_value(struct rsense *_sq52205_dev, u8 reg, unsigned int regval)
{
	int val;

	switch (reg) {
		case SQ52205_CALIBRATION:
			val = regval;
			break;

		case SQ52205_SHUNT_VOLTAGE:
			/*Shunt_Voltage = SHUNT_VOLTAGE[R] * SHUNT_LSB*/
			val = regval * 25 / 10000;
			break;

		case SQ52205_BUS_VOLTAGE:
			/*Bus_Voltage = BUS_VOLTAGE[R] * BUS_LSBV*/
			val = regval * 125 / 100;
			break;

		case SQ52205_POWER:
			/*Power = POWER[R] * POWER_LSB*/
			val = regval * 25;
			break;

		case SQ52205_CURRENT:
			/*Current [mA] = CURRENT[R] *CURRENT_LSB*/
			val = regval * 1;
			break;

		default:
			/* programmer goofed */
			WARN_ON_ONCE(1);
			val = 0;
			break;
	}

	return val;
}

static ssize_t se1000_sq52205_value_show(struct device *dev, struct device_attribute *da, char *buf)
{
	int val, err;
	struct rsense *_sq52205_dev = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);

	mutex_lock(&_sq52205_dev->lock);
	err = se1000_rsense_read(_sq52205_dev->client , attr->index, &val);
	mutex_unlock(&_sq52205_dev->lock);

	if (err < 0)
		return err;

	 return snprintf(buf, PAGE_SIZE, "%d\n",
			se1000_sq52205_get_value(_sq52205_dev, attr->index, val));
}

static ssize_t se1000_sq52205_shunt_show(struct device *dev, struct device_attribute *da, char *buf)
{
	struct rsense *_sq52205_dev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", _sq52205_dev->rshunt);
}

/* calculated current */
static SENSOR_DEVICE_ATTR_RO(current_input, se1000_sq52205_value, SQ52205_CURRENT);

/* calculated power */
static SENSOR_DEVICE_ATTR_RO(power_input, se1000_sq52205_value, SQ52205_POWER);

/* calculated shunt voltage */
static SENSOR_DEVICE_ATTR_RO(v_shunt_input, se1000_sq52205_value, SQ52205_SHUNT_VOLTAGE);

/* calculated bus voltage */
static SENSOR_DEVICE_ATTR_RO(v_bus_input, se1000_sq52205_value, SQ52205_BUS_VOLTAGE);

/* shunt resistance */
static SENSOR_DEVICE_ATTR_RO(shunt_resistor_input, se1000_sq52205_shunt, SQ52205_CALIBRATION);

static struct attribute *se1000_sq52205_attrs[] = {
	&sensor_dev_attr_current_input.dev_attr.attr,
	&sensor_dev_attr_v_shunt_input.dev_attr.attr,
	&sensor_dev_attr_v_bus_input.dev_attr.attr,
	&sensor_dev_attr_power_input.dev_attr.attr,
	&sensor_dev_attr_shunt_resistor_input.dev_attr.attr,
	NULL,
};

static const struct attribute_group se1000_sq52205_group = {
	.attrs = se1000_sq52205_attrs,
};

int se1000_sq52205_init(struct rsense* sq52205_dev)
{
	int ret = 0;
	sq52205_dev->groups[0] = &se1000_sq52205_group;

	mutex_lock(&sq52205_dev->lock);

	ret = se1000_rsense_write(sq52205_dev->client, SQ52205_CONFIG, SQ52205_CONFIG_DEFAULT);
	if (ret)
		goto out;

	/*CAL[R] =  2048 * Shunt_LSB / (Current_LSB * R) */
	ret = se1000_rsense_write(sq52205_dev->client, SQ52205_CALIBRATION, (2048 * 25 / (sq52205_dev->rshunt * 10)));

out:
	mutex_unlock(&sq52205_dev->lock);
	return ret;
}
EXPORT_SYMBOL(se1000_sq52205_init);

int se1000_sq52205_probe(struct i2c_client *client)
{
	struct rsense* sq52205_dev;
	struct device *hwmon_dev;
	u32 val = 2;
	struct device *dev = &client->dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "adapter does not support true I2C\n");
		return -ENODEV;
	}

	sq52205_dev = devm_kzalloc(dev, sizeof(*sq52205_dev), GFP_KERNEL);
	if (!sq52205_dev)
		return -ENOMEM;

	of_property_read_u32(dev->of_node, "shunt-resistor", &val);
		sq52205_dev->rshunt = val;

	sq52205_dev->groups[0] = &se1000_sq52205_group;
	sq52205_dev->client = client;

	mutex_init(&sq52205_dev->lock);
	se1000_sq52205_init(sq52205_dev);
	dev_set_drvdata(dev, sq52205_dev);

	hwmon_dev = devm_hwmon_device_register_with_groups(dev,
							client->name, sq52205_dev, sq52205_dev->groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}
EXPORT_SYMBOL(se1000_sq52205_probe);

static const struct i2c_device_id se1000_sq52205_id[] = {
	{ "se1000_sq52205", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, se1000_sq52205_id);

static const struct of_device_id __maybe_unused se1000_sq52205_of_match[] = {
	{
		.compatible = "se1000, sq52205",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, se1000_sq52205_of_match);

static struct i2c_driver se1000_sq52205_driver = {
	.driver = {
		.name	= "se1000_sq52205",
		.of_match_table = of_match_ptr(se1000_sq52205_of_match),
	},
	.probe_new	= se1000_sq52205_probe,
	.id_table	= se1000_sq52205_id,
};

module_i2c_driver(se1000_sq52205_driver);

MODULE_AUTHOR("gloria.zhang@siengine.com");
MODULE_DESCRIPTION("sq52205 driver");
MODULE_LICENSE("GPL v2");