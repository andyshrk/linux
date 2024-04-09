#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/bitops.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#define SER_ADDR	(0x0c)  //7bit
#define DES_ADDR	(0x34)  //7bit 0x2c
#define DESADDR0	(0x68)	//0x58
#define DESALIAS0	(0x68)	//0x58
#define BUSID	(10)

typedef enum reg_blk {
	MAIN_PAGE = 0,
	PAGE_01,
	PAGE_02,
	PAGE_03,
	PAGE_04,
	PAGE_05,
	PAGE_06,
	PAGE_07,
	PAGE_08,
	PAGE_09,
	PAGE_10,
	PAGE_11,
	PAGE_12,
	PAGE_13,
	PAGE_14,
	APB_REG,
} reg_blk_t;

struct reg_cmd {
	unsigned char chip_addr;
	reg_blk_t blk_index;
	int offset;
	int value;
	int delay;
};

static struct reg_cmd ti_983_hs5_720p[] = {
	{SER_ADDR, MAIN_PAGE, 0x70, DESADDR0, 0},
	{SER_ADDR, MAIN_PAGE, 0x78, DESALIAS0, 0},
	{SER_ADDR, MAIN_PAGE, 0x88, 0x00, 0},
	{SER_ADDR, MAIN_PAGE, 0x48, 0x01, 0},
	{SER_ADDR, APB_REG, 0x0000, 0x00000000, 0},
	{SER_ADDR, APB_REG, 0x0074, 0x0000000a, 0},
	{SER_ADDR, APB_REG, 0x0070, 0x00000004, 0},
	{SER_ADDR, APB_REG, 0x0214, 0x00000000, 0},
	{SER_ADDR, APB_REG, 0x0018, 0x00000014, 0},
	{SER_ADDR, APB_REG, 0x0000, 0x00000001, 0},
	{SER_ADDR, MAIN_PAGE, 0x07, 0x81, 0},
	{SER_ADDR, MAIN_PAGE, 0x07, 0x99, 0},
	{SER_ADDR, MAIN_PAGE, 0x05, 0x00, 0},
	{SER_ADDR, MAIN_PAGE, 0x59, 0x01, 0},
	{SER_ADDR, MAIN_PAGE, 0x29, 0x04, 0},   //BCC_WDOG_CTL
	{SER_ADDR, MAIN_PAGE, 0x40, 0x08, 0},
#if 1 //96M
	{SER_ADDR, PAGE_02, 0x05, 0x7c, 0},
	{SER_ADDR, PAGE_02, 0x06, 0x00, 0},
	{SER_ADDR, PAGE_02, 0x18, 0xf6, 0},
	{SER_ADDR, PAGE_02, 0x19, 0xff, 0},
	{SER_ADDR, PAGE_02, 0x1a, 0xff, 0},
	{SER_ADDR, PAGE_02, 0x1e, 0x18, 0},
	{SER_ADDR, PAGE_02, 0x1f, 0xc7, 0},
	{SER_ADDR, PAGE_02, 0x20, 0x71, 0},
	{SER_ADDR, PAGE_02, 0x13, 0xe0, 0},
	{SER_ADDR, PAGE_02, 0x17, 0x00, 0},
	{SER_ADDR, PAGE_02, 0x14, 0x80, 0},
	{SER_ADDR, PAGE_02, 0x04, 0x09, 0},
	{SER_ADDR, MAIN_PAGE, 0x02, 0xd1, 0},
	{SER_ADDR, MAIN_PAGE, 0x02, 0xd1, 0},
#else  //50M
	{SER_ADDR, PAGE_02, 0x05, 0x81, 0},
	{SER_ADDR, PAGE_02, 0x06, 0x00, 0},
	{SER_ADDR, PAGE_02, 0x18, 0xf6, 0},
	{SER_ADDR, PAGE_02, 0x19, 0xff, 0},
	{SER_ADDR, PAGE_02, 0x1a, 0xff, 0},
	{SER_ADDR, PAGE_02, 0x1e, 0x62, 0},
	{SER_ADDR, PAGE_02, 0x1f, 0x2f, 0},
	{SER_ADDR, PAGE_02, 0x20, 0xa1, 0},
	{SER_ADDR, PAGE_02, 0x13, 0xf0, 0},
	{SER_ADDR, PAGE_02, 0x17, 0x00, 0},
	{SER_ADDR, PAGE_02, 0x14, 0x80, 0},
	{SER_ADDR, PAGE_02, 0x04, 0x09, 0},
	{SER_ADDR, MAIN_PAGE, 0x02, 0xd1, 0},
	{SER_ADDR, MAIN_PAGE, 0x02, 0xd1, 0},
#endif
	{SER_ADDR, MAIN_PAGE, 0x01, 0x30, 100},
	{SER_ADDR, APB_REG, 0x0054, 0x00000001, 0},
	{SER_ADDR, MAIN_PAGE, 0x40, 0x30, 0},
	{SER_ADDR, PAGE_12, 0x01, 0xa8, 0},
	{SER_ADDR, PAGE_12, 0x02, 0x80, 0},
	{SER_ADDR, PAGE_12, 0x03, 0x07, 0},
	{SER_ADDR, PAGE_12, 0x10, 0x80, 0},
	{SER_ADDR, PAGE_12, 0x11, 0x07, 0},
	{SER_ADDR, PAGE_12, 0x12, 0x34, 0},
	{SER_ADDR, PAGE_12, 0x13, 0x00, 0},
	{SER_ADDR, PAGE_12, 0x14, 0x10, 0},
	{SER_ADDR, PAGE_12, 0x15, 0x00, 0},
	{SER_ADDR, PAGE_12, 0x16, 0xdc, 0},
	{SER_ADDR, PAGE_12, 0x17, 0x07, 0},
	{SER_ADDR, PAGE_12, 0x18, 0xd0, 0},
	{SER_ADDR, PAGE_12, 0x19, 0x02, 0},
	{SER_ADDR, PAGE_12, 0x1A, 0x08, 0},
	{SER_ADDR, PAGE_12, 0x1B, 0x00, 0},
	{SER_ADDR, PAGE_12, 0x1C, 0x07, 0},
	{SER_ADDR, PAGE_12, 0x1D, 0x00, 0},
	{SER_ADDR, PAGE_12, 0x1E, 0x39, 0},
	{SER_ADDR, PAGE_12, 0x1F, 0x00, 0},
	{SER_ADDR, PAGE_12, 0x27, 0x01, 0},
	{SER_ADDR, MAIN_PAGE, 0x43, 0x00, 0},
	{SER_ADDR, MAIN_PAGE, 0x44, 0x01, 0},
	{SER_ADDR, MAIN_PAGE, 0x40, 0x30, 0},
	{SER_ADDR, PAGE_12, 0x29, 0x08, 0},
	{SER_ADDR, MAIN_PAGE, 0x40, 0x2E, 0},
	{SER_ADDR, PAGE_11, 0x00, 0x03, 0},
	{SER_ADDR, MAIN_PAGE, 0x2d, 0x01, 0},
	{SER_ADDR, MAIN_PAGE, 0x57, 0x00, 0},
	{SER_ADDR, MAIN_PAGE, 0x5b, 0x2b, 0},
	{SER_ADDR, MAIN_PAGE, 0x02, 0xf0, 0},
	{SER_ADDR, MAIN_PAGE, 0x02, 0xd0, 0},
	{SER_ADDR, MAIN_PAGE, 0x1b, 0x88, 0},
	{SER_ADDR, MAIN_PAGE, 0x07, 0x99, 0},
	{DES_ADDR, MAIN_PAGE, 0x21, 0x09, 0},
	{DES_ADDR, MAIN_PAGE, 0x20, 0x09, 0},
	{DES_ADDR, MAIN_PAGE, 0x34, 0x02, 0},
	{DES_ADDR, MAIN_PAGE, 0x1f, 0x09, 0},
	{DES_ADDR, MAIN_PAGE, 0x1d, 0x19, 0},
	{DES_ADDR, MAIN_PAGE, 0x34, 0x01, 0},
	{DES_ADDR, MAIN_PAGE, 0x1e, 0x99, 0},
	{DES_ADDR, MAIN_PAGE, 0x1f, 0x09, 0},
};

#if 0
static int gpio_set(const struct device *dev, int gpio_num, int value) {
	int ret = 0 ;
	if (!gpio_is_valid(gpio_num))
	{
		dev_err(dev, "failed to get %d!\n", gpio_num);
		return -1;
	}
	ret = gpio_request(gpio_num, "gpio_num");
	if (ret < 0) {
		dev_err(dev,"request gpio %d failed\n", gpio_num);
		return ret;
	}
	gpio_direction_output(gpio_num, value);

	return ret;
}
#endif

static int uh983_write_regs(struct i2c_client *client, struct reg_cmd *cmd, int len) {
	reg_blk_t  blk_info = MAIN_PAGE;
	int ret = 0;
	blk_info = cmd->blk_index;

	switch (blk_info)
	{
	case MAIN_PAGE:
		if (cmd->chip_addr != SER_ADDR) {
			client->addr = DES_ADDR;
		} else {
			client->addr = SER_ADDR;
		}
		ret = i2c_smbus_write_byte_data(client, cmd->offset, cmd->value);
		break;

	case APB_REG:
		client->addr = SER_ADDR;
		ret = i2c_smbus_write_byte_data(client, 0x49, cmd->offset & 0xFF);
		ret = i2c_smbus_write_byte_data(client, 0x4a, (cmd->offset >> 8 ) & 0xFF);
		ret = i2c_smbus_write_byte_data(client, 0x4b, cmd->value & 0xFF);
		ret = i2c_smbus_write_byte_data(client, 0x4c, (cmd->value >> 8) & 0xFF);
		ret = i2c_smbus_write_byte_data(client, 0x4d, (cmd->value >> 16) & 0xFF);
		ret = i2c_smbus_write_byte_data(client, 0x4e, (cmd->value >> 24) & 0xFF);
		break;

	default:
		client->addr = SER_ADDR;
		ret = i2c_smbus_write_byte_data(client, 0x41, cmd->offset);
		ret = i2c_smbus_write_byte_data(client, 0x42, cmd->value);
		break;
	}

	if (cmd->delay)
		mdelay(cmd->delay);
	else
		mdelay(1);

	return ret;
}

static int ds90uh983_init(const struct device *dev, struct i2c_client *client) {
	int ret = 0;
	int i = 0;
	int len = 0;
	struct reg_cmd *cmd;
	int value = -1;

	len = ARRAY_SIZE(ti_983_hs5_720p);

	for (i = 0; i < len; i++) {
		cmd = &ti_983_hs5_720p[i];
		ret = uh983_write_regs(client, cmd, len);

		if (ret != 0)
			dev_err(dev, "i2c_transfer page:%d addr:0x%2x val:0x%02x fail. \n", cmd->blk_index, cmd->offset, cmd->value);
		else
			dev_dbg(dev, "i2c_transfer page:%d addr:0x%2x val:0x%02x success. \n", cmd->blk_index, cmd->offset, cmd->value);

	}

	return ret;
}

static int hs5_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id) {
	struct device *dev = &client->dev;
	struct device_node *np = client->dev.of_node;
	int gpio_num;
	int ret = 0;

	dev_info(dev, "hs5_i2c_probe ...\n");

#if 0
	gpio_num = of_get_named_gpio(np, "ser2_pdb_gpio", 0);

	dev_info(dev, "gpio_num = %d\n", gpio_num);

	if (!gpio_is_valid(gpio_num))
	{
		dev_err(dev, "failed to get %d!\n", gpio_num);
		return -1;
	}

	ret = gpio_request(gpio_num, "gpio_num");
	if (ret < 0) {
		dev_err(dev,"request gpio %d failed\n", gpio_num);
		return ret;
	}

	gpio_direction_output(gpio_num, 0);
	mdelay(10);
	gpio_direction_output(gpio_num, 1);
	mdelay(200);

	dev_info(dev, "hs5_i2c_probe PDB reset, delay 200ms ......\n");
#endif


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "i2c check functionality error\n");
		return -ENXIO;
	}

	ds90uh983_init(dev, client);
	return 0;
}

static const struct i2c_device_id hs5_i2c_id[] = {
	{ "siengine,dp-panel-hs5-720p", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hs5_i2c_id);


#ifdef CONFIG_OF
static const struct of_device_id hs5_of_match[] = {
	{ .compatible = "siengine,dp-panel-hs5-720p" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hs5_of_match);
#endif

static struct i2c_driver hs5_i2c_driver = {
	.probe = hs5_i2c_probe,
	.id_table = hs5_i2c_id,
	.driver = {
		.name = "panel-hs5-720p",
		.of_match_table = of_match_ptr(hs5_of_match),
	},
};

#if 1
module_i2c_driver(hs5_i2c_driver);
#else
static int __init hs5_i2c_init(void)
{
	return i2c_add_driver(&hs5_i2c_driver);
}

fs_initcall_sync(hs5_i2c_init);

static void __exit hs5_i2c_exit(void)
{
	i2c_del_driver(&hs5_i2c_driver);
}
module_exit(hs5_i2c_exit);
#endif

MODULE_AUTHOR("Qiuyue Li<qiuyue.li@siengine.com>");
MODULE_DESCRIPTION("Siengine HS5 Panel driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
