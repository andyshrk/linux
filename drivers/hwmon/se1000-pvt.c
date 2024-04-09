/* Texas Instruments TMP108 SMBus temperature sensor driver
 *
 * Copyright (C) 2016 John Muir <john@jmuir.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/rpmsg.h>
#include "../rpmsg/rpmsg_internal.h"
#include "../rpmsg/se1000-rpmsg-dev.h"
#include <linux/rpmsg/siengine-rpmsg.h>

#define	DRIVER_NAME "se1000-pvt"

#define PVT_TS_NUM	4
#define PVT_PD_NUM  	6
#define	DC_NUM		17
#define PVT_VM_NUM	2
#define VM_CH_NUM	16
#define VOLT_NUM	18
#define COPY_DONE	0xaa

/*pvt info*/
struct se1000_pvt_alarm {
	int high;
	int low;
};

struct pvt_info {
	uint32_t pd[PVT_PD_NUM][DC_NUM];//0.01Mhz
	uint32_t vm[PVT_VM_NUM][VM_CH_NUM];//1mV
	int32_t ts[PVT_TS_NUM];//1mC
	struct se1000_pvt_alarm pd_alarm[PVT_PD_NUM];
	struct se1000_pvt_alarm vm_alarm[VOLT_NUM];
	struct se1000_pvt_alarm ts_alarm[PVT_TS_NUM];
	volatile u32 copy_flag;
} __attribute__((packed));

typedef enum PVT_SENSOR_E {
	PVT_SENSOR_TS = 0,
	PVT_SENSOR_PD,
	PVT_SENSOR_VM,
	PVT_SENSOR_MAX,
} pvt_sensor_et;

/*pvt rpmsg_data*/
struct pvt_rpmsg_data {
	int type;
	int channel;
	int flag; //0:low 1:high
	int val;
}__attribute__((packed));

struct se1000_pvt {
	struct device *hwmon;
	struct mutex pvt_lock;    /* protects register data */
	struct platform_device *pdev;
	int ts_num;
	int vm_alarm_irq;
	int ts_alarm_irq;
	struct rpmsg_endpoint *ept;
	struct pvt_info *pvt;     /*pvt_data*/
};

static int se1000_ts_read(int channel, u32 attr, struct se1000_pvt *priv, int *temp)
{
	u32 val;
	int ret;

	if (!priv || !priv->pvt)
		return -EINVAL;

	ret = read_poll_timeout(readl, val,
		(val == COPY_DONE), 10, 5000, false, &priv->pvt->copy_flag);
	if(ret < 0)
		return -EAGAIN;

	if(channel >= 4)
		return -EINVAL;

	switch(attr) {
		case hwmon_temp_input:
			*temp =  priv->pvt->ts[channel];
			break;
		case hwmon_temp_min_alarm:
			*temp = priv->pvt->ts_alarm[channel].low;
			break;
		case hwmon_temp_max_alarm:
			*temp = priv->pvt->ts_alarm[channel].high;
			break;
		default:
			return -EOPNOTSUPP;
	}

	return 0;
}

static int se1000_vm_read(int channel, u32 attr, struct se1000_pvt *priv, int *vm)
{
	int m, n, ret;
	u32 val;
	if (!priv || !priv->pvt)
		return -EINVAL;

	ret = read_poll_timeout(readl, val,
		(val == COPY_DONE), 10, 5000, false, &priv->pvt->copy_flag);
	if(ret < 0)
		return -EAGAIN;

	if(channel <= 6) {
		m =0;
		n = channel;
	}
	else if(channel <= 17){
		m = 1;
		n = channel - 7;
	}
	else
		return -EINVAL;

	switch(attr) {
		case hwmon_power_input:
			*vm = priv->pvt->vm[m][n];
			break;
		case hwmon_power_min_alarm:
			*vm = priv->pvt->vm_alarm[channel].low;
			break;
		case hwmon_power_max_alarm:
			*vm = priv->pvt->vm_alarm[channel].high;
			break;
		default:
			return -EOPNOTSUPP;
	}

	return 0;
}
static int se1000_pvt_read(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long *temp)
{
	int ret;
	int temp_value;
	struct se1000_pvt *priv;
	priv = dev_get_drvdata(dev);
	if (!priv || !priv->pvt)
		return -ENOMEM;

	switch(type) {
		case hwmon_temp:
			switch(attr) {
				case hwmon_temp_input:
				case hwmon_temp_min_alarm:
				case hwmon_temp_max_alarm:
					ret = se1000_ts_read(channel, attr, priv, &temp_value);
					if(ret)
						return ret;
					*temp = temp_value;
					break;
				default:
					return -EOPNOTSUPP;
			}
			break;
		case hwmon_power:
			switch(attr) {
				case hwmon_power_input:
				case hwmon_power_min_alarm:
				case hwmon_power_max_alarm:
					ret = se1000_vm_read(channel, attr, priv, &temp_value);
					if(ret)
						return ret;
					*temp = temp_value;
					break;
				default:
					return -EOPNOTSUPP;
			}
			break;
		default:
			return 0;
	}
	return 0;
}

static int se1000_pvt_write(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long temp)
{
	struct se1000_pvt *priv;
	struct pvt_rpmsg_data rpmsg_data;
	int ret;
	u32 val;
	priv = dev_get_drvdata(dev);
	if (!priv || !priv->pvt) {
		return -EINVAL;
	}

	ret = read_poll_timeout(readl, val,
		(val == COPY_DONE), 10, 500, false, &priv->pvt->copy_flag);
	if(ret < 0)
		return -EAGAIN;

	rpmsg_data.val = temp;
	rpmsg_data.channel = channel;
	switch(type) {
		case hwmon_temp:
			switch(attr) {
				case hwmon_temp_min_alarm:
					rpmsg_data.type = PVT_SENSOR_TS;
					rpmsg_data.flag = 0;
					break;
				case hwmon_temp_max_alarm:
					rpmsg_data.type = PVT_SENSOR_TS;
					rpmsg_data.flag = 1;
					break;
				default:
					return -EOPNOTSUPP;
			}
			break;
		case hwmon_power:
			switch(attr) {
				case hwmon_power_min_alarm:
					rpmsg_data.type = PVT_SENSOR_VM;
					rpmsg_data.flag = 0;
					break;
				case hwmon_power_max_alarm:
					rpmsg_data.type = PVT_SENSOR_VM;
					rpmsg_data.flag = 1;
					break;
				default:
					return -EOPNOTSUPP;
			}
			break;
		default:
			return -EOPNOTSUPP;
	}

	ret = rpmsg_send(priv->ept, &rpmsg_data, sizeof(rpmsg_data));
	if(ret < 0) {
		pr_err("Err sending rpmsg  message channel:%d,temp:%d\n",rpmsg_data.channel, rpmsg_data.val);
		ret = -EINVAL;
	}

	return 0;
}

static umode_t se1000_pvt_is_visible(const void *data, enum hwmon_sensor_types type,
				 u32 attr, int channel)
{
	if (type == hwmon_chip && attr == hwmon_chip_update_interval)
		return 0644;

	switch(type) {
		case hwmon_temp:
			switch (attr) {
				case hwmon_temp_input:
				case hwmon_temp_min_alarm:
				case hwmon_temp_max_alarm:
					return 0444;
				default:
					return 0;
			}
			break;
		case hwmon_power:
			switch (attr) {
				case hwmon_power_input:
					return 0444;
				case hwmon_power_min_alarm:
				case hwmon_power_max_alarm:
					return 0644;
				default:
					return -EOPNOTSUPP;
			}
			break;
		default:
			return -EOPNOTSUPP;
	}
}

static const struct hwmon_channel_info *se1000_pvt_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_REGISTER_TZ | HWMON_C_UPDATE_INTERVAL),
	HWMON_CHANNEL_INFO(temp,
					HWMON_T_INPUT | HWMON_T_MIN_ALARM | HWMON_T_MAX_ALARM,	/* GPU0 */
		 			HWMON_T_INPUT | HWMON_T_MIN_ALARM | HWMON_T_MAX_ALARM,	/* TOP */
					HWMON_T_INPUT | HWMON_T_MIN_ALARM | HWMON_T_MAX_ALARM,	/* SAF */
					HWMON_T_INPUT | HWMON_T_MIN_ALARM | HWMON_T_MAX_ALARM	/* AP0 */
					),
	HWMON_CHANNEL_INFO(power,
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* AP1 */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* DDR0 */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* DDR1 */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* Q_DDR0*/
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* Q_DDR1 */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* GPU0 */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* NPU0 */

					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* SAF */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* SAF_PLL_REF */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* SAF_PLL0 */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* SAF_PLL1 */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* SAF_PLL2 */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* SAF_PLL3 */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* SAF_METAL1 */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* GPU1 */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* GPU2 */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM,	/* AP0 */
					HWMON_P_INPUT | HWMON_P_MIN_ALARM | HWMON_P_MAX_ALARM	/* AP0_METAL1 */
			   ),
	NULL
};

static const struct hwmon_ops se1000_pvt_hwmon_ops = {
	.is_visible = se1000_pvt_is_visible,
	.read = se1000_pvt_read,
	.write = se1000_pvt_write,
};

static const struct hwmon_chip_info se1000_pvt_chip_info = {
	.ops = &se1000_pvt_hwmon_ops,
	.info = se1000_pvt_info,
};

static const struct of_device_id se1000_pvt_of_match[] = {
	{ .compatible = "siengine,se1000-pvt", },
	{},
};
MODULE_DEVICE_TABLE(of, se1000_pvt_of_match);

static int se1000_pvt_remove(struct platform_device *pdev)
{
	hwmon_device_unregister(&pdev->dev);
	return 0;
}

static irqreturn_t se1000_pvt_vm_irq(int irq, void *data)
{
	return IRQ_HANDLED;
}

static irqreturn_t se1000_pvt_vm_irq_thread(int irq, void *dev)
{
	struct se1000_pvt *pvt = dev;
	int i;

	for(i = 0; i < VOLT_NUM; i++)
		hwmon_notify_event(pvt->hwmon, hwmon_power, hwmon_power_max_alarm, i);

	return IRQ_HANDLED;
}

static irqreturn_t se1000_pvt_ts_irq(int irq, void *dev)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t se1000_pvt_ts_irq_thread(int irq, void *dev)
{
	struct se1000_pvt *pvt = dev;
	int i;
	for(i = 0; i < PVT_TS_NUM; i++)
		hwmon_notify_event(pvt->hwmon, hwmon_temp, hwmon_temp_max_alarm, i);

	return IRQ_HANDLED;
}

static struct rpmsg_channel_info chinfo_pvt_ts =
{
	.name = "se1000_pvt_ts",
	.src = SELINK_RPMSG_ID_TS_ALARM,
	.dst = SELINK_RPMSG_ID_TS_ALARM,
};

int se1000_pvt_create_ept(struct platform_device *pdev, struct rpmsg_channel_info chinfo, rpmsg_rx_cb_t cb)
{
	struct rpmsg_device *rpdev;
	struct device *dev = &pdev->dev;
	struct se1000_pvt *priv = platform_get_drvdata(pdev);

	rpdev = devm_kzalloc(dev, sizeof(*rpdev), GFP_KERNEL);
	if (!rpdev) {
		dev_err(dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	rpdev = find_rpmsg_device_by_phandle(dev->of_node);
	if (!rpdev) {
		pr_err("find_rpmsg_device_by_phandle failed\n");
		return -ENOMEM;
	}

	priv->ept = rpmsg_create_ept(rpdev, cb, NULL, chinfo);
	if (!priv->ept) {
		pr_err("rpmsg_create_ept failed\n");
		return -ENOMEM;
	}

	return 0;
}

static int se1000_pvt_probe(struct platform_device *pdev)
{
	struct se1000_pvt *priv;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *node;
	struct resource r;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pdev = pdev;

	platform_set_drvdata(pdev, priv);

	//resource
	node = of_parse_phandle(np, "memory-region", 0);
	if (!node) {
		dev_err(&pdev->dev, "no memory-region specified\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(node, 0, &r);
	if (ret)
		return ret;
	of_node_put(node);

	priv->pvt = devm_ioremap_resource(&pdev->dev, &r);
	if (!priv->pvt) {
		dev_err(&pdev->dev, "unable to map memory region\n");
		return -ENOMEM;
	}

	priv->vm_alarm_irq = platform_get_irq_byname(pdev, "vm_alarmb");
	if (priv->vm_alarm_irq == -ENXIO) {
		dev_err(&pdev->dev, "cannot obtain vm_alarm IRQ\n");
		ret = -ENXIO;
		return ret;
	}

	ret = devm_request_threaded_irq(dev, priv->vm_alarm_irq, se1000_pvt_vm_irq,
			se1000_pvt_vm_irq_thread, IRQF_TRIGGER_RISING, "pvt_vm_alarm", priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request vm_alarm irq: %d\n", ret);
		return ret;
	}

	priv->ts_alarm_irq = platform_get_irq_byname(pdev, "ts_alarmb");
	if (priv->ts_alarm_irq == -ENXIO) {
		dev_err(&pdev->dev, "cannot obtain ts_alarm IRQ\n");
		ret = -ENXIO;
		return ret;
	}

	ret = devm_request_threaded_irq(dev, priv->ts_alarm_irq, se1000_pvt_ts_irq,
			se1000_pvt_ts_irq_thread, IRQF_TRIGGER_RISING, "pvt_ts_alarm", priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request ts_alarmb irq: %d\n", ret);
		return ret;
	}

	ret = se1000_pvt_create_ept(pdev, chinfo_pvt_ts, NULL);
	if (ret)
		return ret;

	priv->hwmon = devm_hwmon_device_register_with_info(dev, "pvt",
				priv, &se1000_pvt_chip_info, NULL);
	return PTR_ERR_OR_ZERO(priv->hwmon);
}


static struct platform_driver se1000_pvt_drv = {
	.probe = se1000_pvt_probe,
	.remove = se1000_pvt_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = se1000_pvt_of_match,
	},
};

static int __init se1000_pvt_init(void)
{
	return platform_driver_register(&se1000_pvt_drv);
}

static void __exit se1000_pvt_exit(void)
{
	platform_driver_unregister(&se1000_pvt_drv);
}

fs_initcall(se1000_pvt_init);
module_exit(se1000_pvt_exit);

MODULE_DESCRIPTION("Siengine pvt interface");
MODULE_AUTHOR("Siengine @siengine.com");
MODULE_LICENSE("GPL v2");
