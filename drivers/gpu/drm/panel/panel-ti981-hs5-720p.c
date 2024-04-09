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
#define DES_ADDR	(0x34)  //7bit
#define DESADDR0	(0x58)
#define DESALIAS0	(0x58)
#define DES_ADDR1	(0x34)  //7bit

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

static struct reg_cmd ti_981_hs5_720p[] = {
	{SER_ADDR, MAIN_PAGE, 0x70, 0x68, 0},//DESADDR0
	{SER_ADDR, MAIN_PAGE, 0x78, 0x68, 0},//DESALIAS0

	{SER_ADDR, MAIN_PAGE,  0x88, 0x0, 0},


	{SER_ADDR, MAIN_PAGE, 0x02, 0x99, 100},//Disable DSI

	{SER_ADDR, MAIN_PAGE, 0x2d, 0x1, 0}, //Select port 0

	{SER_ADDR, MAIN_PAGE, 0x40, 0x10, 0}, //Change indirect page to page 4
	{SER_ADDR, PAGE_04, 0x5, 0x26, 0},//Port 0 TSKIP value:19

	{SER_ADDR, MAIN_PAGE, 0x4f, 0x8d, 0},//Set number of lanes and continuous or non-continuous

	{SER_ADDR, MAIN_PAGE, 0x2d, 0x3, 0}, //Select write port 0 and 1
	{SER_ADDR, MAIN_PAGE, 0xbd, 0x0, 0}, //Set DSI source for the Video processors 0 and 1
	{SER_ADDR, MAIN_PAGE, 0xbe, 0x0, 0}, //Set DSI source for the Video processors 2 and 3
	{SER_ADDR, MAIN_PAGE, 0x2d, 0x1, 0}, //Select port 0

	{SER_ADDR, MAIN_PAGE, 0x40, 0x14, 0},//page 5
	{SER_ADDR, PAGE_05, 0x16, 0x1f, 0}, //change P/N polarity


	{SER_ADDR, MAIN_PAGE, 0x05, 0x0, 0},
	{SER_ADDR, MAIN_PAGE, 0x59, 0x1, 0},


	//page_2
	{SER_ADDR, MAIN_PAGE, 0x40, 0x8, 0},
	{SER_ADDR, PAGE_02, 0x1b, 0x8, 0},

	{SER_ADDR, PAGE_02, 0x5b, 0x8, 0},

	{SER_ADDR, MAIN_PAGE, 0x2, 0xd1, 0},
	{SER_ADDR, MAIN_PAGE, 0x2d, 0x1, 0},

	//page_2
	{SER_ADDR, MAIN_PAGE, 0x40, 0x8, 0},
	{SER_ADDR, PAGE_02, 0x5, 0x7d, 0},

	{SER_ADDR, PAGE_02, 0x13, 0x90, 0},

	{SER_ADDR, MAIN_PAGE, 0x2d, 0x1, 0},
	{SER_ADDR, MAIN_PAGE, 0x6a, 0xa, 0},
	{SER_ADDR, MAIN_PAGE, 0x6e, 0x80, 0},

	{SER_ADDR, MAIN_PAGE, 0x40, 0x4, 0},

	{SER_ADDR, MAIN_PAGE, 0x2, 0x91, 0},

	{SER_ADDR, MAIN_PAGE, 0x40, 0x8, 0},
	{SER_ADDR, PAGE_02, 0x44,0x9 , 0},
	{SER_ADDR, PAGE_02, 0x53, 0xe0 , 0},
	{SER_ADDR, MAIN_PAGE, 0x40, 0x8 , 0},
	{SER_ADDR, PAGE_02, 0x45,0x71, 0},
	{SER_ADDR, PAGE_02, 0x46,0x0, 0},

	{SER_ADDR, PAGE_02, 0x58, 0x30, 0},
	{SER_ADDR, PAGE_02, 0x59, 0xed, 0},
	{SER_ADDR, PAGE_02, 0x5a, 0xff, 0},
	{SER_ADDR, PAGE_02, 0x5e, 0xfe, 0},
	{SER_ADDR, PAGE_02, 0x5f, 0xf8, 0},
	{SER_ADDR, PAGE_02, 0x60, 0x73, 0},
	{SER_ADDR, MAIN_PAGE, 0x1, 0x30, 0},


	{SER_ADDR, MAIN_PAGE, 0x40, 0x4, 0},
	{SER_ADDR, PAGE_01, 0x26, 0xff, 0},
	{SER_ADDR, PAGE_01, 0x2d, 0x70, 0},
	{SER_ADDR, PAGE_01, 0x2e, 0x70, 0},
	{SER_ADDR, MAIN_PAGE, 0x1, 0x30, 0},


	{SER_ADDR, MAIN_PAGE, 0x40, 0x8, 0},
	{SER_ADDR, PAGE_02, 0x4, 0x1, 0},
	{SER_ADDR, PAGE_02, 0x1e, 0x0, 0},
	{SER_ADDR, PAGE_02, 0x1f, 0x0, 0},
	{SER_ADDR, PAGE_02, 0x20, 0x0, 0},


	{SER_ADDR, PAGE_02, 0xe, 0xc7, 0},

	{SER_ADDR, MAIN_PAGE, 0x1, 0x30, 0},

	{SER_ADDR, MAIN_PAGE, 0x40, 0x8, 0},
	{SER_ADDR, PAGE_02, 0x1b,0x0, 0},
	{SER_ADDR, PAGE_02, 0x5b, 0x0, 0},
	{SER_ADDR, MAIN_PAGE, 0x1, 0x1, 0},

	{SER_ADDR, MAIN_PAGE, 0x07, 0x88, 0},

	{SER_ADDR, MAIN_PAGE, 0x2d,0x1, 0},

	//page_12
	{SER_ADDR, MAIN_PAGE, 0x40, 0x30, 0},
	{SER_ADDR, PAGE_12, 0x01, 0xa8, 0}, //Set VP_SRC_SELECT to Stream 0 for SST Mode

	{SER_ADDR, PAGE_12,  0x2, 0x80,0}, //VID H Active
	{SER_ADDR, PAGE_12,  0x3, 0x7,0}, //VID H Active

	{SER_ADDR, PAGE_12,  0x10, 0x80, 0}, //Horizontal Active
	{SER_ADDR, PAGE_12,  0x11, 0x7, 0}, //Horizontal Active
	{SER_ADDR, PAGE_12,  0x12, 0x34, 0}, //Horizontal Back Porch
	{SER_ADDR, PAGE_12,  0x13, 0x0, 0}, //Horizontal Back Porch
	{SER_ADDR, PAGE_12,  0x14, 0x10, 0}, //Horizontal Sync
	{SER_ADDR, PAGE_12,  0x15, 0x0, 0}, //Horizontal Sync
	{SER_ADDR, PAGE_12,  0x16, 0xdc, 0}, //Horizontal Total
	{SER_ADDR, PAGE_12,  0x17, 0x7, 0}, //Horizontal Total
	{SER_ADDR, PAGE_12,  0x18, 0xd0, 0}, //Vertical Active
	{SER_ADDR, PAGE_12,  0x19, 0x2, 0}, //Vertical Active
	{SER_ADDR, PAGE_12,  0x1a, 0x8, 0}, //Vertical Back Porch
	{SER_ADDR, PAGE_12,  0x1b, 0x0, 0}, //Vertical Back Porch
	{SER_ADDR, PAGE_12,  0x1c, 0x7, 0}, //Vertical Sync
	{SER_ADDR, PAGE_12,  0x1d, 0x0, 0}, //Vertical Sync
	{SER_ADDR, PAGE_12,  0x1e, 0x39, 0}, //Vertical Front Porch
	{SER_ADDR, PAGE_12,  0x1f, 0x0, 0}, //Vertical Front Porch
	{SER_ADDR, PAGE_12,  0x27, 0x1, 0}, //HSYNC Polarity = +, VSYNC Polarity = +
	{SER_ADDR, PAGE_12,  0x23, 0xf7, 0}, //M value
	{SER_ADDR, PAGE_12,  0x24, 0x12, 0}, //M value
	{SER_ADDR, PAGE_12,  0x25, 0xf, 0}, //N value

	{SER_ADDR, MAIN_PAGE, 0x43, 0x0, 0}, //Set number of VPs used = 1
	{SER_ADDR, MAIN_PAGE, 0x44, 0x1, 0}, //Enable video processors

	//Set FPD3 Stream Mapping
	{SER_ADDR, MAIN_PAGE, 0x2d, 0x1, 0}, //Select FPD TX Port 1
	{SER_ADDR, MAIN_PAGE, 0x57, 0x0, 0}, //Set FPD TX Port 1 Stream Source = VP1

	//Configure Serializer TX Link Layer page_11
	{SER_ADDR, MAIN_PAGE, 0x40, 0x2e, 0}, //Link layer Reg page
	{SER_ADDR, PAGE_11, 0x01, 0x1, 0}, //Link layer 0 stream enable
	{SER_ADDR, PAGE_11, 0x06, 0x41, 0}, //Link layer 0 time slot
	{SER_ADDR, PAGE_11, 0x20, 0x55, 0}, //Set Link layer vp bpp according to VP Bit per pixel
	{SER_ADDR, PAGE_11, 0x00, 0x3, 0}, //Link layer 0 enable

	{SER_ADDR, MAIN_PAGE, 0x5b,0x2b, 0}, //Enable FPD III FIFO

	{SER_ADDR, MAIN_PAGE, 0x2, 0x91, 0},

	{SER_ADDR, MAIN_PAGE, 0x2d, 0x1, 100},
	//set deserial
	{DES_ADDR1, MAIN_PAGE, 0x21, 0x09, 0},
	{DES_ADDR1, MAIN_PAGE, 0x20, 0x09, 0},
	{DES_ADDR1, MAIN_PAGE, 0x34, 0x02, 0},
	{DES_ADDR1, MAIN_PAGE, 0x1f, 0x09, 0},
	{DES_ADDR1, MAIN_PAGE, 0x1d, 0x19, 0},
	{DES_ADDR1, MAIN_PAGE, 0x34, 0x01, 0},
	{DES_ADDR1, MAIN_PAGE, 0x1e, 0x99, 10},
	{DES_ADDR1, MAIN_PAGE, 0x1f, 0x09, 0},

	{SER_ADDR, MAIN_PAGE, 0x1, 0x1, 100},
};

static int uh98x_write_regs(struct i2c_client *client, struct reg_cmd *cmd, int len) {
	reg_blk_t  blk_info = MAIN_PAGE;
	int ret = 0;
	blk_info = cmd->blk_index;

	switch (blk_info)
	{
	case MAIN_PAGE:
		if (cmd->chip_addr == DES_ADDR) {
			client->addr = DES_ADDR;
		}

		ret = i2c_smbus_write_byte_data(client, cmd->offset, cmd->value);
		break;

	case APB_REG:
		//client->addr = SER_ADDR;
		ret = i2c_smbus_write_byte_data(client, 0x49, cmd->offset & 0xFF);
		ret = i2c_smbus_write_byte_data(client, 0x4a, (cmd->offset >> 8 ) & 0xFF);
		ret = i2c_smbus_write_byte_data(client, 0x4b, cmd->value & 0xFF);
		ret = i2c_smbus_write_byte_data(client, 0x4c, (cmd->value >> 8) & 0xFF);
		ret = i2c_smbus_write_byte_data(client, 0x4d, (cmd->value >> 16) & 0xFF);
		ret = i2c_smbus_write_byte_data(client, 0x4e, (cmd->value >> 24) & 0xFF);
		break;

	default:
		//client->addr = SER_ADDR;
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


static int ds90uh981_init(const struct device *dev, struct i2c_client *client) {
	int ret = 0;
	int i = 0;
	int len = 0;
	struct reg_cmd *cmd;

	char addr ;

	len = ARRAY_SIZE(ti_981_hs5_720p);

	addr = client->addr;
	for (i = 0; i < len; i++) {
		cmd = &ti_981_hs5_720p[i];
		ret = uh98x_write_regs(client, cmd, len);

		if (ret != 0)
			dev_dbg(dev, "981 i2c_transfer page:%d addr:0x%2x val:0x%02x fail. \n", cmd->blk_index, cmd->offset, cmd->value);
		else
			dev_dbg(dev, "981 i2c_transfer page:%d addr:0x%2x val:0x%02x success. \n", cmd->blk_index, cmd->offset, cmd->value);

	}

	return ret;
}

static int se_hs5_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id) {
	struct device *dev = &client->dev;

	dev_info(dev, "hs5_i2c_probe...\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "i2c check functionality error\n");
		return -ENXIO;
	}

	ds90uh981_init(dev, client);

	return 0;
}

static const struct i2c_device_id se_hs5_i2c_id[] = {
	{ "siengine,dsi-panel-hs5-720p", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hs5_i2c_id);


#ifdef CONFIG_OF
static const struct of_device_id hs5_of_match[] = {
	{ .compatible = "siengine,dsi-panel-hs5-720p" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hs5_of_match);
#endif

static struct i2c_driver hs5_i2c_driver = {
	.probe = se_hs5_i2c_probe,
	.id_table = se_hs5_i2c_id,
	.driver = {
		.name = "dsi-panel-hs5-720p",
		.of_match_table = of_match_ptr(hs5_of_match),
	},
};


module_i2c_driver(hs5_i2c_driver);

MODULE_AUTHOR("Songqin zhang<songqin.zhang@siengine.com>");
MODULE_DESCRIPTION("Siengine HS5 Panel driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
