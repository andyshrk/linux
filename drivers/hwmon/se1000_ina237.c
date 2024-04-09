#include <linux/module.h>
#include <linux/init.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/device.h>
#include "se1000_rsense.h"

/*INA237 Registers*/
#define INA237_CONFIG				0x00
#define INA237_ADC_CONFIG			0x01
#define INA237_SHUNT_CAL			0x02
#define INA237_SHUNT_VOLTAGE			0x04 /* readonly */
#define INA237_BUS_VOLTAGE			0x05 /* readonly */
#define INA237_DIE_TEMP				0x06 /* readonly */
#define INA237_CURRENT				0x07 /* readonly */
#define INA237_POWER				0x08 /* readonly */
#define INA237_DIAG_ALERT			0x0B
#define INA237_SOVL				0x0C
#define INA237_SUVL				0x0D
#define INA237_BOVL				0x0E
#define INA237_BUVL				0x0F
#define INA237_TEMP_LIMIT			0x10
#define INA237_POWER_LIMIT			0x11
#define INA237_MANUFACTURER_ID			0x3E /* readonly */

/*INA237 CONFIGURATIONS*/
#define INA237_CONFIG_VAL			0x50
#define INA237_ADC_CONFIG_VAL			0xF492

#define INA237_MAX_CURRENT			20
#define INA237_REGISTER_NUM			16

static int se1000_ina237_read(struct i2c_client *c, u8 reg, u32 *val)
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

	/*The value of Register INA237_POWER has 24-bit address,
		while others have 16-bit address.*/
	if (reg == INA237_POWER)
		*val = ((buf[0] << 16) | (buf[1] << 8) | buf[2]) & 0xffffff;
	else
		*val = ((buf[0] << 8) | (buf[1])) & 0xffff;

	return 0;
}

static int se1000_ina237_get_value(struct rsense *_ina237_dev, u8 reg, unsigned int regval)
{
	int val, ret;
	unsigned int config;
	unsigned int shunt_cal;

	mutex_lock(&_ina237_dev->lock);
	do {
		ret = se1000_ina237_read(_ina237_dev->client, INA237_CONFIG, &config);
		if (ret)
			break;

		ret = se1000_ina237_read(_ina237_dev->client, INA237_SHUNT_CAL, &shunt_cal);
		if (ret)
			break;
	}while(0);

	mutex_unlock(&_ina237_dev->lock);

	if (ret)
		return ret;

	switch (reg) {
		case INA237_SHUNT_CAL:
			val = regval;
			break;

		case INA237_SHUNT_VOLTAGE:
			/*If ADCRANGE == 1, LSB = 0.005 mV.
			  If ADCRANGE == 0, LSB = 0.00125mV*/
			if (config & 0x10)
				val = regval * 5 / 1000;
			else
				val = regval * 125 / 100000;
			break;

		case INA237_BUS_VOLTAGE:
			/*LSB is 3.125mV*/
			val = regval * 3125 / 1000;
			break;

		case INA237_POWER:
			/*For ADCRANGE = 1, the value of SHUNT_CAL must be multiplied by 4
			SHUNT_CAL = 819200000 *CURRENT_LSB x RSHUNT * (4)
			POWER [mW] = 200 x CURRENT_LSB x POWER[R]*/
			if (config & 0x10)
				val = regval * shunt_cal / (4 * 4096 * _ina237_dev->rshunt);
			else
				val = regval * shunt_cal / (4096 * _ina237_dev->rshunt);
			break;

		case INA237_CURRENT:
			/*Current [mA] = 1000 * CURRENT_LSB x CURRENT*/
			if (config & 0x10)
				val = 25 * regval * shunt_cal / (81920 * _ina237_dev->rshunt);
			else
				val = 10 * regval * shunt_cal / (8192 * _ina237_dev->rshunt);
			break;

		default:
			/* programmer goofed */
			WARN_ON_ONCE(1);
			val = 0;
			break;
	}

	return val;
}

static ssize_t se1000_ina237_value_show(struct device *dev, struct device_attribute *da, char *buf)
{
	int val, err;
	struct rsense *_ina237_dev = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);

	mutex_lock(&_ina237_dev->lock);
	err = se1000_ina237_read(_ina237_dev->client , attr->index, &val);
	mutex_unlock(&_ina237_dev->lock);

	if (err < 0)
		return err;

	 return snprintf(buf, PAGE_SIZE, "%d\n",
			se1000_ina237_get_value(_ina237_dev, attr->index, val));
}

static ssize_t se1000_ina237_shunt_show(struct device *dev, struct device_attribute *da, char *buf)
{
	struct rsense *_ina237_dev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", _ina237_dev->rshunt);
}

/* calculated current */
static SENSOR_DEVICE_ATTR_RO(current_input, se1000_ina237_value, INA237_CURRENT);

/* calculated power */
static SENSOR_DEVICE_ATTR_RO(power_input, se1000_ina237_value, INA237_POWER);

/* calculated shunt voltage */
static SENSOR_DEVICE_ATTR_RO(v_shunt_input, se1000_ina237_value, INA237_SHUNT_VOLTAGE);

/* calculated bus voltage */
static SENSOR_DEVICE_ATTR_RO(v_bus_input, se1000_ina237_value, INA237_BUS_VOLTAGE);

/* shunt resistance */
static SENSOR_DEVICE_ATTR_RO(shunt_resistor_input, se1000_ina237_shunt, INA237_SHUNT_CAL);

static struct attribute *se1000_ina237_attrs[] = {
	&sensor_dev_attr_current_input.dev_attr.attr,
	&sensor_dev_attr_v_shunt_input.dev_attr.attr,
	&sensor_dev_attr_v_bus_input.dev_attr.attr,
	&sensor_dev_attr_power_input.dev_attr.attr,
	&sensor_dev_attr_shunt_resistor_input.dev_attr.attr,
	NULL,
};

static const struct attribute_group se1000_ina237_group = {
	.attrs = se1000_ina237_attrs,
};

int se1000_ina237_init(struct rsense* _ina237_dev)
{
	int ret = 0;

	mutex_lock(&_ina237_dev->lock);

	/*Register INA237_CONFIG 0x50
	Bit[6:13]——1 Sets the Delay for initial ADC conversion in steps of 2 ms: 2ms
	Bit4 —— 1 shunt fullscale range selection IN+ and IN- 40.96mV*/
	ret = se1000_rsense_write(_ina237_dev->client, INA237_CONFIG, INA237_CONFIG_VAL);
	if (ret)
		goto out;

	/* Register INA237_ADC_CONFIG
	Set bit[12:15] to 1111 : mode bit for continuous bus voltage, shunt voltage and temperature
	Set bit[3:11] to 010 010 010 : Set the conversion time of bus voltage measurement 150us
	Set bit[2:0] to 0x2 : set number of averages to 16*/
	ret = se1000_rsense_write(_ina237_dev->client, INA237_ADC_CONFIG, INA237_ADC_CONFIG_VAL);
	if (ret)
		goto out;

	/*calibration_value = ((4) * 25 * INA237_MAX_CURRENT * data->config->rshunt);*/
	ret = se1000_rsense_write(_ina237_dev->client, INA237_SHUNT_CAL, (4 * 25 * INA237_MAX_CURRENT * _ina237_dev->rshunt));

out:
	mutex_unlock(&_ina237_dev->lock);
	return ret;
}
EXPORT_SYMBOL(se1000_ina237_init);

int se1000_ina237_probe(struct i2c_client *client)
{
	int ret;
	struct rsense* ina237_dev;
	struct device *hwmon_dev;
	u32 val = 2;
	struct device *dev = &client->dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "adapter does not support true I2C\n");
		return -ENODEV;
	}

	ina237_dev = devm_kzalloc(dev, sizeof(*ina237_dev), GFP_KERNEL);
	if (!ina237_dev)
		return -ENOMEM;

	of_property_read_u32(dev->of_node, "shunt-resistor", &val);
		ina237_dev->rshunt = val;

	ina237_dev->groups[0] = &se1000_ina237_group;
	ina237_dev->client = client;
	dev_set_drvdata(dev, ina237_dev);

	mutex_init(&ina237_dev->lock);
	ret = se1000_ina237_init(ina237_dev);
	if (ret)
		return ret;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev,
					client->name, ina237_dev, ina237_dev->groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}
EXPORT_SYMBOL(se1000_ina237_probe);

static const struct i2c_device_id se1000_ina237_id[] = {
	{ "se1000_ina237", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, se1000_ina237_id);

static const struct of_device_id __maybe_unused se1000_ina237_of_match[] = {
	{
		.compatible = "se1000, ina237",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, se1000_ina237_of_match);

static struct i2c_driver se1000_ina237_driver = {
	.driver = {
		.name	= "se1000_ina237",
		.of_match_table = of_match_ptr(se1000_ina237_of_match),
	},
	.probe_new	= se1000_ina237_probe,
	.id_table	= se1000_ina237_id,
};

module_i2c_driver(se1000_ina237_driver);

MODULE_AUTHOR("gloria.zhang@siengine.com");
MODULE_DESCRIPTION("ina237 driver");
MODULE_LICENSE("GPL v2");