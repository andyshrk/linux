/*
 *Copyright (C) 2019-2021 Siengine, Inc. (www.siengine.com)
 */
 /* Siengine rfkill driver for wifi
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/rfkill.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/rfkill-wlan.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <dt-bindings/gpio/gpio.h>
#include <linux/skbuff.h>
#include <linux/fb.h>
#include <linux/device.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#if 0
#define DBG(x...)	pr_info("[WLAN_RFKILL]: " x)
#define LOG(x...)	pr_info("[WLAN_RFKILL]: " x)
#else
#define DBG(x...)
#define LOG(x...)
#endif

static int rfkill_wlan_setup_gpio(struct platform_device *pdev,
				struct rfkill_wlan_gpio *gpio, const char *prefix,
				const char *name)
{
	if (gpio_is_valid(gpio->io)) {
		int ret = 0;

		sprintf(gpio->name, "%s_%s", prefix, name);
		ret = devm_gpio_request(&pdev->dev, gpio->io, gpio->name);
		if (ret) {
			LOG("Failed to get %s gpio.\n", gpio->name);
			return -1;
		}
	}

	return 0;
}

#ifdef CONFIG_OF
static int wlan_platdata_parse_dt(struct device *dev, struct rfkill_wlan_data *data)
{
	struct device_node *node = dev->of_node;
	int gpio;
	enum of_gpio_flags flags;

	if(!node) {
		LOG("%s: node is null", __func__);
		return -1;
	}

	gpio = of_get_named_gpio_flags(node, "wlan,reset_gpio", 0, &flags);
	LOG("%s: gpio = %d", __func__, gpio);
	if(gpio_is_valid(gpio)) {
		data->reset_gpio.io = gpio;
		data->reset_gpio.enable = (flags == GPIO_ACTIVE_HIGH) ? 1 : 0;
		LOG("%s: get property: wlan,reset_gpio = %d.\n", __func__, gpio);
	} else {
		data->reset_gpio.io = -1;
		LOG("%s: get wlan,reset_gpio failed\n", __func__);
	}

	return 0;
}
#endif

static int rfkill_wlan_probe(struct platform_device *pdev)
{
	struct rfkill_wlan_data *rfkill = pdev->dev.platform_data;
	int ret = -1;

	if(!rfkill) {
#ifdef CONFIG_OF
		rfkill = kzalloc(sizeof(struct rfkill_wlan_data), GFP_KERNEL);
		if(!rfkill) {
			LOG("%s: pdata kzalloc failed\n", __func__);
			return -ENOMEM;
		}

		ret = wlan_platdata_parse_dt(&pdev->dev, rfkill);
		if(ret < 0)
#endif
		{
			LOG("%s: No platform data specified\n", __func__);
			goto fail_gpio;
		}
	}

	rfkill->name = RFKILL_WLAN_NAME;

	LOG("%s: init gpio\n", __func__);

	ret = rfkill_wlan_setup_gpio(pdev, &rfkill->reset_gpio, rfkill->name, "reset");
	if(ret) {
		LOG("%s: set reset_gpio failed\n", __func__);
		goto fail_gpio;
	}

	if(gpio_is_valid(rfkill->reset_gpio.io)) {
		gpio_direction_output(rfkill->reset_gpio.io, !rfkill->reset_gpio.enable);
		msleep(20);
		gpio_direction_output(rfkill->reset_gpio.io, rfkill->reset_gpio.enable);
	}

	platform_set_drvdata(pdev, rfkill);

	LOG("%s device registered.\n", rfkill->name);
	return 0;
fail_gpio:
	kfree(rfkill);
	return ret;
}

static int rfkill_wlan_remove(struct platform_device *pdev)
{
	struct rfkill_wlan_data *rfkill = platform_get_drvdata(pdev);

	if(gpio_is_valid(rfkill->reset_gpio.io)) {
		gpio_free(rfkill->reset_gpio.io);
	}

	kfree(rfkill);
	return 0;
}

static int rfkill_wlan_suspend(struct platform_device *pdev, pm_message_t status)
{
	LOG("Enter %s\n", __func__);
	return 0;
}

static int rfkill_wlan_resume(struct platform_device *pdev)
{
	LOG("Enter %s\n", __func__);
	return 0;
}

static void rfkill_wlan_shutdown(struct platform_device *pdev)
{
	struct rfkill_wlan_data *rfkill = platform_get_drvdata(pdev);
	LOG("Enter %s\n", __func__);
	if(gpio_is_valid(rfkill->reset_gpio.io)) {
		gpio_direction_output(rfkill->reset_gpio.io, !rfkill->reset_gpio.enable);
	}
}

#ifdef CONFIG_OF
static struct of_device_id wlan_platdata_of_match[] = {
	{.compatible = "wlan-platdata"},
	{}
};
MODULE_DEVICE_TABLE(of, wlan_platdata_of_match);
#endif

static struct platform_driver rfkill_wlan_driver = {
	.probe = rfkill_wlan_probe,
	.remove = rfkill_wlan_remove,
	.suspend = rfkill_wlan_suspend,
	.resume = rfkill_wlan_resume,
	.shutdown = rfkill_wlan_shutdown,
	.driver = {
		.name = "wlan-platdata",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(wlan_platdata_of_match),
	},
};

int __init rfkill_wlan_init(void)
{
	LOG("Enter %s\n", __func__);
	return platform_driver_register(&rfkill_wlan_driver);
}

void __exit rfkill_wlan_exit(void)
{
	LOG("Enter %s\n", __func__);
	platform_driver_unregister(&rfkill_wlan_driver);
}

//subsys_initcall_sync(rfkill_wlan_init);
module_init(rfkill_wlan_init);
module_exit(rfkill_wlan_exit);

MODULE_DESCRIPTION("siengine rfkill for wifi v0.1");
MODULE_AUTHOR("xinan.xiao@siengine.com");
MODULE_LICENSE("GPL");
