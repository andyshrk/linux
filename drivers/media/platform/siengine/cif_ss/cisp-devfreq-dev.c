#include <linux/module.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/types.h>
#include <linux/pm_opp.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>

#define DRV_NAME		"cisp-devfreq-dev"
#define DRV_VERSION		"1.0"

static int cisp_devfreq_target(struct device *dev, unsigned long *freq,
							   u32 flags)
{
	struct dev_pm_opp *opp;
	int err;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	dev_pm_opp_put(opp);

	err = dev_pm_opp_set_rate(dev, *freq);
	if (err)
		return err;

	return 0;
}

static int cisp_devfreq_get_dev_status(struct device *dev,
									   struct devfreq_dev_status *status)
{
	struct clk *clk = dev_get_drvdata(dev);

	status->busy_time = 0;
	status->total_time = 0;
	status->current_frequency = clk_get_rate(clk);

	return 0;
}

static struct devfreq_dev_profile cisp_devfreq_profile = {
	.polling_ms = 1000, /* 1 second */
	.target = cisp_devfreq_target,
	.get_dev_status = cisp_devfreq_get_dev_status,
};


static int cisp_devfreq_dev_probe(struct platform_device *pdev)
{
	int i;
	struct clk *clk;
	struct device_node *np;
	struct device *dev = &pdev->dev;
	struct opp_table *opp_table;
	unsigned long max_freq;
	unsigned long cur_freq;
	struct dev_pm_opp *opp;
	struct devfreq *devfreq;
	struct thermal_cooling_device *cooling;

	np = of_node_get(dev->of_node);
	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		dev_warn(dev, "could not get clk\n");
		return PTR_ERR(clk);
	}

	dev_set_drvdata(dev, clk);

	cur_freq = max_freq = clk_get_rate(clk);
	dev_dbg(dev, "isp-core cur_freq = %lld, name=%s\n", cur_freq, __clk_get_name(clk));

	opp_table = dev_pm_opp_get_opp_table(dev);
	//opp_table = dev_pm_opp_set_clkname(dev, __clk_get_name(clk));
	if (IS_ERR(opp_table)) {
		dev_warn(dev, "could not get alloc opp_table\n");
		return PTR_ERR(opp_table);
	}

#define DEV_PM_OPP_MAX 4
	for (i = 0; i < DEV_PM_OPP_MAX; i++) {
		dev_pm_opp_add(dev, cur_freq, DEV_PM_OPP_MAX - i);
		cur_freq >>=1;
	}

	opp = devfreq_recommended_opp(dev, &max_freq, 0);
	if (IS_ERR(opp)) {
		dev_warn(dev, "could not devfreq_recommended_opp\n");
		return PTR_ERR(opp);
	}

	cisp_devfreq_profile.initial_freq = max_freq;
	dev_pm_opp_put(opp);

	devfreq = devm_devfreq_add_device(dev, &cisp_devfreq_profile,
								  DEVFREQ_GOV_SIMPLE_ONDEMAND, NULL);
	if (IS_ERR(devfreq)) {
		dev_warn(dev, "Couldn't initialize cisp devfreq\n");
		return PTR_ERR(devfreq);
	}

	cooling = of_devfreq_cooling_register(dev->of_node, devfreq);
	if (IS_ERR(cooling))
		dev_warn(dev, "Failed to register cooling device\n");

	return 0;
}
static const struct of_device_id cisp_devfreq_of_match[] = {
	{
		.compatible = "siengine,se1000-cisp-devfreq",
	},
	{},
};
static struct platform_driver cisp_devfreq_driver = {
	.driver = {
		.name	= DRV_NAME,
		.of_match_table = cisp_devfreq_of_match,
	},
	.probe = cisp_devfreq_dev_probe,
};

module_platform_driver(cisp_devfreq_driver);

MODULE_AUTHOR("Mingrui Zhou<mingrui.zhou@siengine.com>");
MODULE_DESCRIPTION("Siengine "DRV_NAME" Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
