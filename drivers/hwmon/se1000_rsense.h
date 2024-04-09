#include <linux/i2c.h>
#include <linux/mutex.h>

#define RSENSE_MAX_ATTRIBUTE_GROUPS 1

struct rsense {
	u32 rshunt;
	struct mutex lock;
	struct i2c_client *client;
	const struct attribute_group *groups[RSENSE_MAX_ATTRIBUTE_GROUPS];
};

int se1000_rsense_read(struct i2c_client *c, u8 reg, u32 *val);
int se1000_rsense_write(struct i2c_client *c, u8 reg, u16 val);
int se1000_sq52205_probe(struct i2c_client *client);
int se1000_ina237_probe(struct i2c_client *client);
int se1000_sq52205_init(struct rsense* sq52205_dev);
int se1000_ina237_init(struct rsense* _ina237_dev);