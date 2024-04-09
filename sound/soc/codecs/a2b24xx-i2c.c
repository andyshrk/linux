/*
 * A2B24xx driver
 *
 * Copyright 2019 Analog Devices Inc.
 * ADI Automotive Software Team, Bangalore
 *
 * Licensed under the GPL-2.
 */

#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <sound/soc.h>
#include "a2b24xx.h"


static int a2b24xx_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct regmap_config config;

	config = a2b24xx_regmap_config;
	config.val_bits = 8;
	config.reg_bits = 8;

	return a2b24xx_probe(&client->dev,
		devm_regmap_init_i2c(client, &config),
		id->driver_data, NULL);
}

static int a2b24xx_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_component(&client->dev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id a2b24xx_dt_ids[] = {
	{ .compatible = "adi,a2b24xx", },
	{ }
};
MODULE_DEVICE_TABLE(of, a2b24xx_dt_ids);
#endif

static const struct i2c_device_id a2b24xx_i2c_ids[] = {
	{ "a2b24xx", A2B24XX },
	{ }
};
MODULE_DEVICE_TABLE(i2c, a2b24xx_i2c_ids);

static struct i2c_driver a2b24xx_i2c_driver = {
	.driver = {
		.name = "a2b24xx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(a2b24xx_dt_ids),
	},
	.probe = a2b24xx_i2c_probe,
	.remove = a2b24xx_i2c_remove,
	.id_table = a2b24xx_i2c_ids,
};
module_i2c_driver(a2b24xx_i2c_driver);

MODULE_DESCRIPTION("ASoC A2B24xx driver");
MODULE_AUTHOR("ADI Automotive Software Team, Bangalore");
MODULE_LICENSE("GPL");
