/*
 * Copyright (C) 2018 Siengine Technology Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt)		"pwm-siengine : " fmt

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/bits.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>
#define PWM_N_LOAD_COUNT		0x00
#define PWM_N_CURRENT_VALUE		0x04
#define PWM_N_CONTROL			0x08
#define PWM_N_EOI				0x0c
#define PWM_N_INT_STATUS		0x10

#define PWMS_INT_STATUS			0xa0
#define PWMS_EOI				0xa4
#define PWMS_RAW_INT_STATUS		0xa8
#define PWMS_COMP_VERSION		0xac

#define PWM_N_LOAD_COUNT2		0xb0
#define PWM_N_PROT_LEVEL		0xd0

#define PWM_CONTROL_ENABLE		BIT(0)
/* 1: periodic, 0:free running. */
#define PWM_CONTROL_MODE_PERIODIC	BIT(1)
#define PWM_CONTROL_INT			BIT(2)
#define PWM_CONTROL_TOGGLE_OUTPUT       BIT(3)
#define PWM_CONTROL_TIMER_0N100PWM_EN   BIT(4)
#define PWMS_REG_SIZE			0x14
#define TIMER_WIDTH_N       		32
#define PWM_DIV_SCALE			200
#define PWM_PERIOD_MAX			BIT(TIMER_WIDTH_N)-1
#define PWM_CYCLE_MAX			BIT(TIMER_WIDTH_N)-1

struct siengine_pwm_chip {
	struct pwm_chip chip;
	struct clk *pclk;
	void __iomem *base;
	int irq;
	unsigned long rate;
	bool suspend;
};

static inline struct siengine_pwm_chip *to_siengine_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct siengine_pwm_chip, chip);
}

static inline unsigned int siengine_reg_convert(unsigned int hwpwm, unsigned int offset)
{
	if (offset <= PWM_N_INT_STATUS)
		return (hwpwm * PWMS_REG_SIZE + offset);
	else if (offset <= PWMS_COMP_VERSION)
		return offset;
	else
		return (hwpwm * 0x4) + offset;
}

static inline u32 siengine_pwm_readl(struct siengine_pwm_chip *siengine_pwm,
			unsigned int hwpwm, unsigned int offset)
{
	return readl(siengine_pwm->base + siengine_reg_convert(hwpwm, offset));
}

static inline void siengine_pwm_writel(struct siengine_pwm_chip *siengine_pwm,
			unsigned int hwpwm, unsigned int offset, u32 value)
{
	writel(value, siengine_pwm->base + siengine_reg_convert(hwpwm, offset));
}

static void siengine_pwm_eoi(struct siengine_pwm_chip *siengine_pwm, unsigned int hwpwm)
{
	siengine_pwm_readl(siengine_pwm, hwpwm, PWM_N_EOI);
}

static int siengine_pwm_shutdown(struct siengine_pwm_chip *siengine_pwm, unsigned int hwpwm)
{
	u32 ctrl;

	ctrl = siengine_pwm_readl(siengine_pwm, hwpwm, PWM_N_CONTROL);
	ctrl &= ~PWM_CONTROL_ENABLE;
	siengine_pwm_writel(siengine_pwm, hwpwm, PWM_N_CONTROL, ctrl);
	return 0;
}

static int siengine_pwm_enable(struct siengine_pwm_chip *siengine_pwm, unsigned int hwpwm)
{
	u32 ctrl;

	ctrl = siengine_pwm_readl(siengine_pwm, hwpwm, PWM_N_CONTROL);
	ctrl |= PWM_CONTROL_ENABLE;
	siengine_pwm_writel(siengine_pwm, hwpwm, PWM_N_CONTROL, ctrl);
	return 0;
}

static int siengine_pwm_set_mode(struct siengine_pwm_chip *siengine_pwm, unsigned int hwpwm)
{
	u32 ctrl;

	ctrl = siengine_pwm_readl(siengine_pwm, hwpwm, PWM_N_CONTROL);
	ctrl |= PWM_CONTROL_TIMER_0N100PWM_EN; //Timer 0% and 100% PWM duty cycle mode is enabled
	ctrl |= PWM_CONTROL_MODE_PERIODIC;
	ctrl |= PWM_CONTROL_TOGGLE_OUTPUT;
	siengine_pwm_writel(siengine_pwm, hwpwm, PWM_N_CONTROL, ctrl);
	return 0;
}

static int siengine_pwm_init(struct siengine_pwm_chip *siengine_pwm, unsigned int hwpwm)
{
	u32 ctrl;

	ctrl = siengine_pwm_readl(siengine_pwm, hwpwm, PWM_N_CONTROL);

	/* set as user mode */
	ctrl |= PWM_CONTROL_MODE_PERIODIC;

	/* disable intr */
	ctrl |= PWM_CONTROL_INT;

	/* disable pwm */
	ctrl &= ~PWM_CONTROL_ENABLE;
	siengine_pwm_writel(siengine_pwm, hwpwm, PWM_N_CONTROL, ctrl);
	return 0;
}

static void siengine_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			     struct pwm_state *state)
{
	struct siengine_pwm_chip *siengine_pwm = to_siengine_pwm_chip(chip);
	u64 count0, count1;

	u32 ctrl = siengine_pwm_readl(siengine_pwm, pwm->hwpwm, PWM_N_CONTROL);

	if (ctrl & PWM_CONTROL_ENABLE)
		state->enabled = true;
	else
		state->enabled = false;

	count1 = siengine_pwm_readl(siengine_pwm, pwm->hwpwm, PWM_N_LOAD_COUNT2);
	count1 *= NSEC_PER_SEC;
	state->duty_cycle = DIV_ROUND_CLOSEST_ULL(count1, siengine_pwm->rate);

	count0 = siengine_pwm_readl(siengine_pwm, pwm->hwpwm, PWM_N_LOAD_COUNT);
	count0 *= NSEC_PER_SEC;
	state->period = DIV_ROUND_CLOSEST_ULL(count0 + count1, siengine_pwm->rate);
}

static int siengine_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			 unsigned int duty_ns, unsigned int period_ns)
{
	struct siengine_pwm_chip *siengine_pwm = to_siengine_pwm_chip(chip);
	unsigned int duty_cycles, empty_cycles;
	unsigned long long c;
	unsigned long rate;
	unsigned int empty_ns;
	/* Find out the best divider */
	rate = siengine_pwm->rate;

	/* Calculate cycles */
	if (duty_ns == 0) {
		duty_cycles = 0;
		empty_cycles = 1;
	} else if (duty_ns == period_ns) {
		duty_cycles = 1;
		empty_cycles = 0;
	} else {
		empty_ns = period_ns - duty_ns;

		c = rate * duty_ns;
		do_div(c, NSEC_PER_SEC);
		duty_cycles = c;

		c = rate * empty_ns;
		do_div(c, NSEC_PER_SEC);
		empty_cycles = c;

		if (duty_cycles > PWM_CYCLE_MAX || empty_cycles > PWM_CYCLE_MAX) {
			return -ERANGE;
		}
	}
	siengine_pwm_shutdown(siengine_pwm, 0);
	/* Set up registers */
	siengine_pwm_writel(siengine_pwm, pwm->hwpwm, PWM_N_LOAD_COUNT, empty_cycles);
	siengine_pwm_writel(siengine_pwm, pwm->hwpwm, PWM_N_LOAD_COUNT2, duty_cycles);

	return 0;
}

static int siengine_pwm_configs_enable(struct siengine_pwm_chip *siengine_pwm, struct pwm_device *pwm)
{
	int ret;
	ret = clk_prepare_enable(siengine_pwm->pclk);
	if (ret)
		return ret;
	siengine_pwm_enable(siengine_pwm, pwm->hwpwm);
	return 0;
}

static int siengine_pwm_configs_disable(struct siengine_pwm_chip *siengine_pwm, struct pwm_device *pwm)
{
	siengine_pwm_shutdown(siengine_pwm, pwm->hwpwm);
	clk_disable_unprepare(siengine_pwm->pclk);
	return 0;
}

static int siengine_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			const struct pwm_state *state)
{
	struct siengine_pwm_chip *siengine_pwm = to_siengine_pwm_chip(chip);
	struct pwm_state cstate;
	int ret;

	pwm_get_state(pwm, &cstate);

	/* This HW/driver only supports normal polarity */
	if (state->polarity != PWM_POLARITY_NORMAL)
		return -ENOTSUPP;

	if ((state->period != cstate.period ||
		state->duty_cycle != cstate.duty_cycle) || siengine_pwm->suspend) {
		ret = siengine_pwm_config(chip, pwm, state->duty_cycle,
					state->period);
		if (ret)
			return ret;
	}

	if ((state->enabled != cstate.enabled) || (siengine_pwm->suspend)) {
		if (state->enabled) {
			ret = siengine_pwm_configs_enable(siengine_pwm, pwm);
			if (ret)
				return ret;
		}
		else
			siengine_pwm_configs_disable(siengine_pwm, pwm);
		siengine_pwm->suspend = false;
	}
	else if(state->period != cstate.period ||
		state->duty_cycle != cstate.duty_cycle) {
		if (state->enabled) {
			ret = siengine_pwm_configs_enable(siengine_pwm, pwm);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static const struct pwm_ops siengine_pwm_ops = {
	.apply = siengine_pwm_apply,
	.get_state = siengine_pwm_get_state,
	.owner = THIS_MODULE,
};

static int siengine_pwm_probe(struct platform_device *pdev)
{
	struct siengine_pwm_chip *siengine_pwm;
	struct resource *res;
	int ret, i;
	struct device *dev = &pdev->dev;

	siengine_pwm = devm_kzalloc(dev, sizeof(*siengine_pwm), GFP_KERNEL);
	if (!siengine_pwm)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	siengine_pwm->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(siengine_pwm->base))
		return PTR_ERR(siengine_pwm->base);

	siengine_pwm->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(siengine_pwm->pclk))
		return PTR_ERR(siengine_pwm->pclk);
	siengine_pwm->rate = clk_get_rate(siengine_pwm->pclk);

	siengine_pwm->chip.npwm = 1;
	if (of_find_property(dev->of_node, "safety-pwm", NULL))
		siengine_pwm->chip.npwm = 2;

	siengine_pwm->chip.dev = dev;
	siengine_pwm->chip.ops = &siengine_pwm_ops;
	siengine_pwm->chip.base = -1;

	siengine_pwm->chip.of_xlate = of_pwm_xlate_with_flags;
	siengine_pwm->chip.of_pwm_n_cells = 3;

	siengine_pwm->suspend = false;
	/*
	 * disable all of them initially to save power.
	 */
	siengine_pwm_init(siengine_pwm, 0);
	siengine_pwm_set_mode(siengine_pwm, 0);

	ret = pwmchip_add(&siengine_pwm->chip);
	if (ret < 0) {
		dev_err(dev, "failed to add PWM chip: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, siengine_pwm);

	return 0;
}

static int siengine_pwm_remove(struct platform_device *pdev)
{
	struct siengine_pwm_chip *siengine_pwm = platform_get_drvdata(pdev);
	int ret;

	ret = pwmchip_remove(&siengine_pwm->chip);
	clk_disable_unprepare(siengine_pwm->pclk);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int siengine_pwm_suspend(struct device *dev)
{
	struct siengine_pwm_chip *siengine_pwm = dev_get_drvdata(dev);
	pinctrl_pm_select_sleep_state(dev);
	siengine_pwm->suspend = true;
	return 0;
}

static int siengine_pwm_resume(struct device *dev)
{
	struct siengine_pwm_chip *siengine_pwm = dev_get_drvdata(dev);
#ifdef CONFIG_SE1000_STR
	pinctrl_pm_force_default_state(dev);
#else
	pinctrl_pm_select_default_state(dev);
#endif
	siengine_pwm_init(siengine_pwm, 0);
	siengine_pwm_set_mode(siengine_pwm, 0);

	return 0;
}
#else
#define siengine_pwm_suspend	NULL
#define siengine_pwm_resume	NULL
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops siengine_pwm_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(siengine_pwm_suspend, siengine_pwm_resume)
};
static const struct of_device_id siengine_pwm_dt_ids[] = {
	{ .compatible = "siengine,se1000-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, siengine_pwm_dt_ids);

static struct platform_driver siengine_pwm_driver = {
	.driver = {
		.name = "siengine-pwm",
		.pm = &siengine_pwm_pm,
		.of_match_table = siengine_pwm_dt_ids,
	},
	.probe = siengine_pwm_probe,
	.remove = siengine_pwm_remove,
};
module_platform_driver(siengine_pwm_driver);

MODULE_ALIAS("platform:siengine-pwm");
MODULE_AUTHOR("Mingrui Zhou <mingrui.zhou@siengine.com>");
MODULE_DESCRIPTION("siengine,se1000 PWM Driver");
MODULE_LICENSE("GPL v2");

