/**
 * usbgl3510.c - Siengine Genesys logic hub controller driver
 *
 * Copyright (c) Siengine Co., Ltd.
 *
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/module.h>

struct usbgl3510 {
	struct platform_device *pdev;
	struct device *dev;
	unsigned int num_enable_gpios;
	const unsigned int *enable_gpios;
	bool active_low;
};

static int usbgl3510_probe(struct platform_device *pdev)
{
	struct usbgl3510 *hub;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;

	int ret, i;
	int ngpios;
	unsigned int *gpios;

	hub = devm_kzalloc(dev, sizeof(*hub), GFP_KERNEL);
	if (!hub)
		return -ENOMEM;

	platform_set_drvdata(pdev, hub);
	hub->dev = dev;

	/* get enable gpios number from devicetree */
	hub->num_enable_gpios = ngpios = of_gpio_named_count(node, "enable-gpios");
	if (ngpios <= 0) {
		dev_err(dev, "number of enable gpios not specified\n");
		return -EINVAL;
	}

	if (of_get_property(node, "gpio-activelow", NULL)) {
		hub->active_low = true;
	}

	gpios = devm_kcalloc(dev,
					hub->num_enable_gpios,
					sizeof(unsigned int),
					GFP_KERNEL);

	/* get enable gpios from devicetree */
	for (i = 0; i < ngpios; i++) {
		ret = of_get_named_gpio(node, "enable-gpios", i);
		if (ret < 0) {
			dev_err(dev, "of get named gpio failed\n");
			return ret;
		}
		gpios[i] = ret;
	}

	hub->enable_gpios = gpios;

	/* enable gpios to release hub */
	for (i = 0; i < ngpios; i++) {
		gpio_direction_output(hub->enable_gpios[i], !hub->active_low);
	}

	return 0;
}

static int usbgl3510_remove(struct platform_device *pdev)
{
	int	i;
	struct usbgl3510 *hub = platform_get_drvdata(pdev);

	for (i = 0; i < hub->num_enable_gpios; i++) {
		gpio_direction_output(hub->enable_gpios[i], hub->active_low);
	}

	return 0;
}

static const struct of_device_id usbgl3510_match[] = {
	{ .compatible = "genesys,gl3510" },
	{},
};
MODULE_DEVICE_TABLE(of, usbgl3510_match);

#ifdef CONFIG_PM_SLEEP
static int usbgl3510_suspend(struct device *dev)
{
	int i;
	struct usbgl3510 *hub = dev_get_drvdata(dev);

	for (i = 0; i < hub->num_enable_gpios; i++) {
		gpio_direction_output(hub->enable_gpios[i], hub->active_low);
	}

	return 0;
}

static int usbgl3510_resume(struct device *dev)
{
	int	i;
	struct usbgl3510 *hub = dev_get_drvdata(dev);

	for (i = 0; i < hub->num_enable_gpios; i++) {
		gpio_direction_output(hub->enable_gpios[i], !hub->active_low);
	}

	return 0;
}

static const struct dev_pm_ops usbgl3510_dev_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(usbgl3510_suspend, usbgl3510_resume)
};

#define DEV_PM_OPS	(&usbgl3510_dev_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver usbgl3510_driver = {
	.probe		= usbgl3510_probe,
	.remove		= usbgl3510_remove,
	.driver		= {
		.name	= "genesys-gl3510",
		.of_match_table = usbgl3510_match,
		.pm	= DEV_PM_OPS,
	},
};

module_platform_driver(usbgl3510_driver);

MODULE_AUTHOR("jian.shen <jian.shen@siengine.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Siengine Genesys Logic USB3.1 Hub Controller Driver");
