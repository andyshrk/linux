// SPDX-License-Identifier: GPL-2.0
// (C) 2019-2021 Siengine, Inc. (www.siengine.com)

/*
 * Siengine pinmux Controller driver.
 *
 * Author: Mingrui Zhou <Mingrui.Zhou@siengine.com>
 */
#define pr_fmt(fmt) "pinctrl-se1000: " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_data/pinctrl-single.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/bsearch.h>
#include <linux/rbtree.h>
#include <linux/bits.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinctrl-utils.h"
#include "../pinmux.h"

#define PINCTRL_SE1000_BIT_MUX_MAX					4

#define PINCTRL_SE1000_BIT_SCHMITT_TRIGGER			16
#define PINCTRL_SE1000_BIT_MUX_CONFIGURATION		12
#define PINCTRL_SE1000_BIT_DRIVER_STRENGTH			8
#define PINCTRL_SE1000_BIT_PULL_DOWN				5
#define PINCTRL_SE1000_BIT_PULL_UP					4
#define PINCTRL_SE1000_BIT_INPUT_ENABLE				3

#define PINCTRL_SE1000_SCHMITT_TRIGGER			BIT(PINCTRL_SE1000_BIT_SCHMITT_TRIGGER	)
#define PINCTRL_SE1000_MUX_CONFIGURATION		BIT(PINCTRL_SE1000_BIT_MUX_CONFIGURATION)
#define PINCTRL_SE1000_DRIVER_STRENGTH			BIT(PINCTRL_SE1000_BIT_DRIVER_STRENGTH	)
#define PINCTRL_SE1000_PULL_DOWN				BIT(PINCTRL_SE1000_BIT_PULL_DOWN		)
#define PINCTRL_SE1000_PULL_UP					BIT(PINCTRL_SE1000_BIT_PULL_UP			)
#define PINCTRL_SE1000_INPUT_ENABLE				BIT(PINCTRL_SE1000_BIT_INPUT_ENABLE		)

#define PINCTRL_SE1000_MASK_SCHMITT_TRIGGER		(1  << PINCTRL_SE1000_BIT_SCHMITT_TRIGGER	)
#define PINCTRL_SE1000_MASK_MUX_CONFIGURATION	(3  << PINCTRL_SE1000_BIT_MUX_CONFIGURATION	)
#define PINCTRL_SE1000_MASK_DRIVER_STRENGTH		(15 << PINCTRL_SE1000_BIT_DRIVER_STRENGTH	)
#define PINCTRL_SE1000_MASK_PULL_DOWN			(1  << PINCTRL_SE1000_BIT_PULL_DOWN			)
#define PINCTRL_SE1000_MASK_PULL_UP				(1  << PINCTRL_SE1000_BIT_PULL_UP			)
#define PINCTRL_SE1000_MASK_INPUT_ENABLE		(1  << PINCTRL_SE1000_BIT_INPUT_ENABLE		)

#define PINCTRL_SE1000_DRIV_STRENGTH_MAX    	0xF
#define PINCTRL_SAF_REG_MASK				0xFF
#define PINCTRL_SAF_MASK_FUNC				BIT(7)
#define PINCTRL_SAF_MASK_DRIVER_STRENGTH	(0xF << 3)
#define PINCTRL_SAF_BIT_FUNC					7
#define PINCTRL_SAF_BIT_DRIVER_STRENGTH			3
#define PINCTRL_SAF_BIT_PULL_DOWN				2
#define PINCTRL_SAF_BIT_PULL_UP					1
#define PINCTRL_SAF_BIT_INPUT_ENABLE			0
#define PINCTRL_SAF_DRIVER_STRENGTH			BIT(3)
#define PINCTRL_SAF_PULL_DOWN				BIT(2)
#define PINCTRL_SAF_PULL_UP					BIT(1)
#define PINCTRL_SAF_INPUT_ENABLE			BIT(0)
struct se_safety_pin_desc {
	bool enable;
	u32 saf_idx;
	u32 saf_reg;
	u32 shift;
	void __iomem *saf_base;
	const char *fn_name;
};

struct se_pinmux_fn_desc {
	u32 mux_idx;
	struct rb_node fn_node; /* rb_mux_fn */
	const char *mux_name;
	struct se_pinmux_desc *pin_desc;
};

struct se_pinmux_desc {
	u32 idx;
	u32 pin;
	u32 reg;
	u32 nr_mux;
	struct se_pinmux_fn_desc *pinmux_fn;

	atomic_t ref_count;

	struct se_safety_pin_desc *safety;
};

struct siengine_pinctrl {
	struct device *dev;
	void __iomem *base_mem;
	void __iomem *saf_base;
	struct regmap *regmap;

	struct pinctrl_dev *pctl_dev;

	u32 nr_pinmux;
	u32 nr_mux_max;
	struct rb_root rb_mux_fn;
	struct se_pinmux_desc *pin_desc;
};

static int __siengine_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
				unsigned long *configs, unsigned num_configs,
				struct se_pinmux_fn_desc *fn_desc);

void dump_se_pinmux_desc(struct se_pinmux_desc *desc)
{
	int i;

	pr_err("idx = %d, pin = %d, reg = %04x, nr_mux = %d\n",
		desc->idx, desc->pin, desc->reg, desc->nr_mux);

	for (i = 0; i < desc->nr_mux; i++)
		pr_err("mux_idx = %d, mux_name = %s\n",
			desc->pinmux_fn[i].mux_idx,	 desc->pinmux_fn[i].mux_name);
}

struct se_pinmux_fn_desc *mux_fn_rb_search(struct rb_root *root, const char *mux_name)
{
	struct rb_node *node = root->rb_node;

	while (node) {
		int result;
		struct se_pinmux_fn_desc *desc;

		desc = container_of(node, struct se_pinmux_fn_desc, fn_node);
		result = strcmp(mux_name, desc->mux_name);

		if (result < 0)
			node = node->rb_left;
		else if (result > 0)
			node = node->rb_right;
		else
			return desc;
	}

	return NULL;
}

int mux_fn_rb_insert(struct rb_root *root, struct se_pinmux_fn_desc *desc)
{
	struct rb_node **new = &(root->rb_node), *parent = NULL;

	/* Figure out where to put new node */
	while (*new) {
		struct se_pinmux_fn_desc *this = container_of(*new, struct se_pinmux_fn_desc, fn_node);
		int result = strcmp(desc->mux_name, this->mux_name);

		parent = *new;
		if (result < 0)
			new = &((*new)->rb_left);
		else if (result > 0)
			new = &((*new)->rb_right);
		else
			return -1;
	}

	/* Add new node and rebalance tree. */
	rb_link_node(&desc->fn_node, parent, new);
	rb_insert_color(&desc->fn_node, root);

	return 0;
}

static int inline pmx_bsearch_compare(const void *key, const void *elt)
{
	uint32_t pmx_id = *(uint32_t *)key;
	struct se_pinmux_desc *pin_desc = (struct se_pinmux_desc *)elt;

	return ((int)pmx_id - (int)pin_desc->pin);
}

static inline struct se_pinmux_desc *siengine_pinmux_desc_bsearch(
	struct siengine_pinctrl *pinctrl, uint32_t pmx_id)
{
	return bsearch(&pmx_id, pinctrl->pin_desc, pinctrl->nr_pinmux,
					sizeof (struct se_pinmux_desc), pmx_bsearch_compare);
}


static int inline siengine_raw_read(void __iomem *base, u32 offset, uint32_t *val)
{
	*val = readl(base + offset);

	return 0;
}

static int inline siengine_pinmux_read(struct siengine_pinctrl *pinctrl,
	struct se_pinmux_desc *pinmux_desc, uint32_t *val)
{
#ifdef CONFIG_PINMUX_SAF_ENABLE
	uint32_t data = 0;
#endif
	if (NULL == pinmux_desc || 0 == pinmux_desc->nr_mux)
		return -EINVAL;
#ifdef CONFIG_PINMUX_SAF_ENABLE
	if (unlikely(pinmux_desc->safety && pinmux_desc->safety->enable)) {
		siengine_raw_read(pinctrl->saf_base, pinmux_desc->safety->saf_reg, &data);
		*val = (data >> pinmux_desc->safety->shift) & PINCTRL_SAF_REG_MASK;
		return 0;
	}
#endif
	return siengine_raw_read(pinctrl->base_mem, pinmux_desc->reg, val);
}

static int siengine_regmap_read(void *ctx, uint32_t pmx_id, uint32_t *val)
{
	struct se_pinmux_desc *pin_desc;
	struct siengine_pinctrl *pinctrl;

	pinctrl =(struct siengine_pinctrl *)ctx;

	pin_desc = siengine_pinmux_desc_bsearch(pinctrl, pmx_id);
	if (NULL == pin_desc)
		return -EINVAL;

	return siengine_pinmux_read(pinctrl, pin_desc, val);
}

static int inline siengine_raw_write(void __iomem *base, u32 offset, uint32_t val)
{
	writel(val, base + offset);

	return 0;
}

static int inline siengine_pinmux_write(struct siengine_pinctrl *pinctrl,
	struct se_pinmux_desc *pinmux_desc, uint32_t val)
{
#ifdef CONFIG_PINMUX_SAF_ENABLE
	uint32_t data = 0;
#endif
	if (NULL == pinmux_desc || 0 == pinmux_desc->nr_mux)
		return -EINVAL;
#ifdef CONFIG_PINMUX_SAF_ENABLE
	if (unlikely(pinmux_desc->safety && pinmux_desc->safety->enable)) {
		siengine_raw_read(pinctrl->saf_base, pinmux_desc->safety->saf_reg, &data);
		data = data & (~(PINCTRL_SAF_REG_MASK << pinmux_desc->safety->shift));
		data = data | (val << pinmux_desc->safety->shift);
		return siengine_raw_write(pinctrl->saf_base,
				pinmux_desc->safety->saf_reg, data);
	}
#endif
	return siengine_raw_write(pinctrl->base_mem, pinmux_desc->reg, val);
}

static int siengine_regmap_write(void *ctx, uint32_t pmx_id, uint32_t val)
{
	struct se_pinmux_desc *pin_desc;
	struct siengine_pinctrl *pinctrl;

	pinctrl =(struct siengine_pinctrl *)ctx;

	pin_desc = siengine_pinmux_desc_bsearch(pinctrl, pmx_id);
	if (NULL == pin_desc)
		return -EINVAL;

	return siengine_pinmux_write(pinctrl, pin_desc, val);
}

static struct regmap_config siengine_regmap_config = {
	.name = "siengine,pinctrl",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_read = siengine_regmap_read,
	.reg_write = siengine_regmap_write,
	.cache_type = REGCACHE_NONE,
};

static inline const struct siengine_pin_group *pinctrl_name_to_group(
		const struct siengine_pinctrl *info, const char *name)
{
	return NULL;
}

/*
 * Pinctrl_ops handling
 */
#ifdef CONFIG_DEBUG_FS
static void siengine_pctl_pin_dbg_show(struct pinctrl_dev *pctldev,
				struct seq_file *s, unsigned offset)
{
	seq_printf(s, "%s", dev_name(pctldev->dev));
}
#endif
static const struct pinctrl_ops siengine_pctrl_ops = {
	.get_groups_count	= pinctrl_generic_get_group_count,
	.get_group_name		= pinctrl_generic_get_group_name,
	.get_group_pins		= pinctrl_generic_get_group_pins,
#ifdef CONFIG_OF
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_all,
	.dt_free_map		= pinctrl_utils_free_map,
#endif
#ifdef CONFIG_DEBUG_FS
	.pin_dbg_show		= siengine_pctl_pin_dbg_show,
#endif
};

#ifdef CONFIG_PINMUX_SAF_ENABLE
static int inline siengine_pinmux_safety_func(struct siengine_pinctrl *pinctrl,
	struct se_pinmux_desc *pin_desc)
{
	int ret;
	u32 reg;

	ret = siengine_pinmux_read(pinctrl, pin_desc, &reg);
	if (ret) {
		dev_err(pinctrl->dev, "%s: fail to read saf_reg '%08x'\n",
			 __func__, pin_desc->safety->saf_reg);
		return ret;
	}

	reg &= ~(PINCTRL_SAF_MASK_FUNC);
	reg |= (pin_desc->safety->enable << PINCTRL_SAF_BIT_FUNC);

	return siengine_pinmux_write(pinctrl, pin_desc, reg);
}
#endif

/*
 * Pinmux_ops handling
 */
static int siengine_pmx_set(struct pinctrl_dev *pctldev, unsigned function,
			unsigned group)
{
	int ret;
	u32 reg;
	unsigned int pin;
	struct group_desc *grp;
	struct function_desc *func;
	struct se_pinmux_desc *pin_desc;
	struct se_pinmux_fn_desc *fn_desc;
	struct siengine_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);

	func = pinmux_generic_get_function(pctldev, function);
	if (unlikely(!func)) {
		dev_err(pctldev->dev, "can not get function = %d\n", function);
		return -ENOTSUPP;
	}

	grp = pinctrl_generic_get_group(pctldev, group);
	if (unlikely(!grp)) {
		dev_err(pctldev->dev, "can not get group = %d\n", group);
		return -ENOTSUPP;
	}

	dev_dbg(pctldev->dev, "enable function %s group %s\n",
			 func->name, grp->name);

	pin = *grp->pins;
	fn_desc = grp->data;
	pin_desc = fn_desc->pin_desc;
	if (unlikely(pin_desc->safety && pin_desc->safety->enable)) {
		dev_err(pctldev->dev, "pinmux-%d has been set as safe mode %s\n",
			pin, pin_desc->safety->fn_name ? : "safe");
#ifdef CONFIG_PINMUX_SAF_ENABLE
		siengine_pinmux_safety_func(pinctrl, pin_desc);
#endif
		return 0;
	}
	if (unlikely(0 == pin_desc->nr_mux)) {
		dev_warn(pctldev->dev, "pin '%u', has not mux-fn\n", pin);
		return -ENOTSUPP;
	}

	ret = siengine_pinmux_read(pinctrl, pin_desc, &reg);
	if (unlikely(ret)) {
		dev_warn(pctldev->dev, "%s: Can not read reg '%08x'\n", __func__, pin_desc->reg);
		dump_se_pinmux_desc(pin_desc);
		return ret;
	}

	reg &= ~PINCTRL_SE1000_MASK_MUX_CONFIGURATION;
	reg |= fn_desc->mux_idx << PINCTRL_SE1000_BIT_MUX_CONFIGURATION;

	return siengine_pinmux_write(pinctrl, pin_desc, reg);
}

/*
 * function called from the gpiolib interface).
 */
static int siengine_pmx_gpio_set_direction(struct pinctrl_dev *pctldev,
					struct pinctrl_gpio_range *range,
					unsigned offset, bool input)
{
	// pr_err("--Mingrui Zhou : %s %d\n", __func__, __LINE__);
	// pr_err("--id = %d, base = %d, pin_base = %d, npins = %d\n",
		// range->id, range->base, range->pin_base, range->npins);
	unsigned long configs = 0;

	if (input) {
		configs |= PIN_CONFIG_INPUT_ENABLE;
	} else {
		configs |= PIN_CONFIG_OUTPUT_ENABLE;
	}
	__siengine_pinconf_set(pctldev, offset, &configs, 1, NULL);

	return 0;
}

int siengine_request(struct pinctrl_dev *pctldev, unsigned pin)
{
	struct se_pinmux_desc *pin_desc;
	struct siengine_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);

	pin_desc = siengine_pinmux_desc_bsearch(pinctrl, pin);
	if (unlikely(NULL == pin_desc)) {
		dev_warn(pctldev->dev, "Can not configuration pin '%u'\n", pin);
		return -ENOTSUPP;
	}

	if (unlikely(pin_desc->safety && pin_desc->safety->enable)) {
		dev_dbg(pctldev->dev, "pinmux-%d has been set as safe mode %s\n",
			pin, pin_desc->safety->fn_name ? : "safe");
		return 0;
	}

	if (unlikely(0 == pin_desc->nr_mux)) {
		dev_err(pctldev->dev, "pin '%u', has not mux-fn\n", pin);
		return -EIO;
	}

	return 0;
}

int siengine_free(struct pinctrl_dev *pctldev, unsigned offset)
{
	struct se_pinmux_desc *pin_desc;
	struct siengine_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);

	pin_desc = siengine_pinmux_desc_bsearch(pinctrl, offset);

	atomic_dec(&pin_desc->ref_count);

	// pr_err("--Mingrui Zhou : %s %d\n", __func__, __LINE__);
	return 0;
}

int siengine_gpio_request_enable(struct pinctrl_dev *pctldev,
				struct pinctrl_gpio_range *range, unsigned offset)
{
	int ret;
	unsigned gpio_offset;
	struct se_pinmux_desc *pin_desc;
	u32 reg;

	struct siengine_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);

	pin_desc = siengine_pinmux_desc_bsearch(pinctrl, offset);
	if (unlikely(NULL == pin_desc)) {
		dev_warn(pctldev->dev, "Can not configuration pin '%u'\n", offset);
		return -ENOTSUPP;
	}

	if (unlikely(pin_desc->safety && pin_desc->safety->enable)) {
		dev_err(pctldev->dev, "pinmux-%d has been set as safe mode %s\n",
			offset, pin_desc->safety->fn_name ? : "safe");
		return 0;
	}

	ret = siengine_pinmux_read(pinctrl, pin_desc, &reg);
	if (unlikely(ret)) {
		dev_warn(pctldev->dev, "%s: Can not read reg '%08x'\n", __func__, pin_desc->reg);
		dump_se_pinmux_desc(pin_desc);
		return ret;
	}

	reg &= ~PINCTRL_SE1000_MASK_MUX_CONFIGURATION;

	ret = siengine_pinmux_write(pinctrl, pin_desc, reg);
	if (unlikely(ret)) {
		dev_warn(pctldev->dev, "%s: Can not write reg '%08x'\n", __func__, pin_desc->reg);
		return ret;
	}

	gpio_offset = offset - range->pin_base;
	// pr_err("--Mingrui Zhou : %s %d\n", __func__, __LINE__);
	// pr_err("--Mingrui Zhou : offset = %d, gpio_offset = %d\n", offset, gpio_offset);
	// pr_err("--name = %s, id = %d, base = %d, pin_base = %d, npins = %d, gc->label = %s\n",
		// range->name, range->id, range->base, range->pin_base, range->npins, range->gc->label);
	return 0;
}

void siengine_gpio_disable_free(struct pinctrl_dev *pctldev,
			   struct pinctrl_gpio_range *range, unsigned offset)
{
	// pr_err("--Mingrui Zhou : %s %d\n", __func__, __LINE__);
	// pr_err("--Mingrui Zhou : offset = %d\n", offset);
	// pr_err("--name = %s, id = %d, base = %d, pin_base = %d, npins = %d, gc->label = %s\n",
		// range->name, range->id, range->base, range->pin_base, range->npins, range->gc->label);
}

static const struct pinmux_ops siengine_pmx_ops = {
	.request				= siengine_request,
	.free					= siengine_free,
	.gpio_request_enable	= siengine_gpio_request_enable,
	.gpio_disable_free		= siengine_gpio_disable_free,
	.get_functions_count	= pinmux_generic_get_function_count,
	.get_function_name		= pinmux_generic_get_function_name,
	.get_function_groups	= pinmux_generic_get_function_groups,
	.set_mux				= siengine_pmx_set,
	.gpio_set_direction		= siengine_pmx_gpio_set_direction,
};

#ifdef CONFIG_PINMUX_SAF_ENABLE
static int inline siengine_pinmux_safety_get(struct siengine_pinctrl *pinctrl,
				unsigned int pin, unsigned int param, unsigned long *config)
{
	unsigned int arg = 0;
	u32 reg;
	int ret;

	ret = regmap_read(pinctrl->regmap, pin, &reg);
	if (unlikely(ret)) {
		dev_err(pinctrl->dev, "Can not read saf pin '%u' register\n", pin);
		return -EIO;
	}

	switch (param) {
	case PIN_CONFIG_DRIVE_STRENGTH:
		arg = reg & PINCTRL_SAF_MASK_DRIVER_STRENGTH;
		arg >>= PINCTRL_SAF_BIT_DRIVER_STRENGTH;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		arg = !!(reg & PINCTRL_SAF_PULL_UP);
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		arg = !!(reg & PINCTRL_SAF_PULL_DOWN);
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		arg = !!(reg & PINCTRL_SAF_INPUT_ENABLE);
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
		arg = !(reg & PINCTRL_SAF_INPUT_ENABLE);
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}
#endif

/* get the pin config settings for a specified pin */
static int siengine_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
							unsigned long *config)
{
	unsigned int arg = 0;
	struct se_pinmux_desc *pin_desc;
	u32 reg;
	int ret;

	unsigned int param = pinconf_to_config_param(*config);
	struct siengine_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);

	pin_desc = siengine_pinmux_desc_bsearch(pinctrl, pin);
	if (unlikely(NULL == pin_desc)) {
		dev_err(pctldev->dev, "Can not configuration pin '%u'\n", pin);
		return -ENOTSUPP;
	}

	if (unlikely(pin_desc->safety && pin_desc->safety->enable)) {
		dev_info(pctldev->dev, "pinconf_get pinmux-%d has been set as safe mode %s\n",
			pin, pin_desc->safety->fn_name ? : "safe");
#ifdef CONFIG_PINMUX_SAF_ENABLE
		siengine_pinmux_safety_get(pinctrl, pin, param, config);
#endif
		return 0;
	}

	ret = regmap_read(pinctrl->regmap, pin, &reg);
	if (unlikely(ret)) {
		dev_err(pctldev->dev, "Can not read pin '%u' register\n", pin);
		return -EIO;
	}

	switch (param) {
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		arg = !!(reg & PINCTRL_SE1000_MASK_SCHMITT_TRIGGER);
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		arg = reg & PINCTRL_SE1000_MASK_DRIVER_STRENGTH;
		arg >>= PINCTRL_SE1000_BIT_DRIVER_STRENGTH;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		arg = !!(reg & PINCTRL_SE1000_MASK_PULL_UP);
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		arg = !!(reg & PINCTRL_SE1000_MASK_PULL_DOWN);
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		arg = !!(reg & PINCTRL_SE1000_MASK_INPUT_ENABLE);
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
		arg = !(reg & PINCTRL_SE1000_MASK_INPUT_ENABLE);
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

#ifdef CONFIG_PINMUX_SAF_ENABLE
static int inline siengine_pinmux_safety_set(struct siengine_pinctrl *pinctrl,
				struct se_pinmux_desc *pin_desc, unsigned long *configs,
				unsigned num_configs, struct se_pinmux_fn_desc *fn_desc)
{
	int ret = 0;
	u32 reg = 0;
	int i = 0;

	ret = siengine_pinmux_read(pinctrl, pin_desc, &reg);
	if (ret) {
		dev_err(pinctrl->dev, "%s: fail to read saf_reg '%08x'\n",
			__func__, pin_desc->safety->saf_reg);
		return ret;
	}

	for (i = 0; i < num_configs; i++) {
		unsigned int param = pinconf_to_config_param(configs[i]);
		unsigned int arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_DRIVE_STRENGTH:
			if(arg > PINCTRL_SE1000_DRIV_STRENGTH_MAX){
				return -EINVAL;
			}
			reg &= ~PINCTRL_SAF_MASK_DRIVER_STRENGTH;
			reg |= arg << PINCTRL_SAF_BIT_DRIVER_STRENGTH;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			if (arg)
				reg |= PINCTRL_SAF_PULL_UP;
			else
				reg &= ~PINCTRL_SAF_PULL_UP;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			if (arg)
				reg |= PINCTRL_SAF_PULL_DOWN;
			else
				reg &= ~PINCTRL_SAF_PULL_DOWN;
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			reg |= PINCTRL_SAF_INPUT_ENABLE;
			break;
		case PIN_CONFIG_OUTPUT_ENABLE:
			reg &= ~PINCTRL_SAF_INPUT_ENABLE;
			break;
		default:
			continue;
		}
	}

	if (fn_desc) {
		reg &= ~(PINCTRL_SAF_MASK_FUNC);
		reg |= (pin_desc->safety->enable << PINCTRL_SAF_BIT_FUNC);
	}

	return siengine_pinmux_write(pinctrl, pin_desc, reg);
}
#endif

/* set the pin config settings for a specified pin */
static int __siengine_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
				unsigned long *configs, unsigned num_configs,
				struct se_pinmux_fn_desc *fn_desc)
{
	struct se_pinmux_desc *pin_desc;
	u32 reg;
	int ret;
	int i;

	struct siengine_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctldev);

	/* pin: has been checked at siengine_request */
	pin_desc = siengine_pinmux_desc_bsearch(pinctrl, pin);
	if (unlikely(NULL == pin_desc)) {
		dev_warn(pctldev->dev, "Can not configuration pin '%u'\n", pin);
		return -ENOTSUPP;
	}

	if (unlikely(pin_desc->safety && pin_desc->safety->enable)) {
		dev_err(pctldev->dev, "pinconf_set pinmux-%d has been set as safe mode %s\n",
			pin, pin_desc->safety->fn_name ? : "safe");
#ifdef CONFIG_PINMUX_SAF_ENABLE
		siengine_pinmux_safety_set(pinctrl, pin_desc, configs,
				num_configs, fn_desc);
#endif
		return 0;
	}

	if (unlikely(0 == pin_desc->nr_mux)) {
		dev_warn(pctldev->dev, "pin '%u', has not mux-fn\n", pin);
		return -ENOTSUPP;
	}

	ret = siengine_pinmux_read(pinctrl, pin_desc, &reg);
	if (unlikely(ret)) {
		dev_warn(pctldev->dev, "%s: Can not read reg '%08x'\n", __func__, pin_desc->reg);
		dump_se_pinmux_desc(pin_desc);
		return ret;
	}

	if (atomic_read(&pin_desc->ref_count)) {
		u32 mux = reg;

		mux &= PINCTRL_SE1000_MASK_MUX_CONFIGURATION;
		mux >>= PINCTRL_SE1000_BIT_MUX_CONFIGURATION;

		dev_warn(pctldev->dev, "pinmux-%d has been set as %s\n",
			pin, pin_desc->pinmux_fn[mux].mux_name);
	}

	for (i = 0; i < num_configs; i++) {
		unsigned int param = pinconf_to_config_param(configs[i]);
		unsigned int arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
			if (arg)
				reg |= PINCTRL_SE1000_SCHMITT_TRIGGER;
			else
				reg &= ~PINCTRL_SE1000_SCHMITT_TRIGGER;
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			if(arg > PINCTRL_SE1000_DRIV_STRENGTH_MAX){
				return -EINVAL;
			}
			reg &= ~PINCTRL_SE1000_MASK_DRIVER_STRENGTH;
			reg |= arg << PINCTRL_SE1000_BIT_DRIVER_STRENGTH;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			if (arg)
				reg |= PINCTRL_SE1000_PULL_UP;
			else
				reg &= ~PINCTRL_SE1000_PULL_UP;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			if (arg)
				reg |= PINCTRL_SE1000_PULL_DOWN;
			else
				reg &= ~PINCTRL_SE1000_PULL_DOWN;
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			reg |= PINCTRL_SE1000_INPUT_ENABLE;
			break;
		case PIN_CONFIG_OUTPUT_ENABLE:
			reg &= ~PINCTRL_SE1000_INPUT_ENABLE;  //input disable
			break;
		default:
			dev_warn(pctldev->dev,
				 "unsupported configuration parameter '%u'\n", param);
			continue;
		}
	}

	if (fn_desc) {
		reg &= ~PINCTRL_SE1000_MASK_MUX_CONFIGURATION;
		reg |= fn_desc->mux_idx << PINCTRL_SE1000_BIT_MUX_CONFIGURATION;
	}

	ret = siengine_pinmux_write(pinctrl, pin_desc, reg);
	if (unlikely(ret)) {
		return ret;
	}

	atomic_inc(&pin_desc->ref_count);

	return 0;
}

/* set the pin config settings for a specified pin */
static int siengine_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
				unsigned long *configs, unsigned num_configs)
{
	return __siengine_pinconf_set(pctldev, pin, configs, num_configs, NULL);
}

static int siengine_pinconf_group_set(struct pinctrl_dev *pctldev,
		unsigned int group, unsigned long *configs,
		unsigned int num_configs)
{
	int i, ret;
	struct group_desc *grp;

	grp = pinctrl_generic_get_group(pctldev, group);
	if (unlikely(!grp)) {
		dev_err(pctldev->dev, "can not get group = %d\n", group);
		return -ENOTSUPP;
	}

	for (i = 0; i < grp->num_pins; i++) {
		ret = __siengine_pinconf_set(pctldev, grp->pins[i], configs, num_configs, NULL);
		if (unlikely(ret))
			return ret;
	}

	return 0;
}

int siengine_pinconf_group_get(struct pinctrl_dev *pctldev,
		unsigned group, unsigned long *config)
{
	// pr_err("--Mingrui Zhou : %s %d\n", __func__, __LINE__);
	return 0;
}

static const struct pinconf_ops siengine_pinconf_ops = {
	.pin_config_get			= siengine_pinconf_get,
	.pin_config_set			= siengine_pinconf_set,
	.pin_config_group_set	= siengine_pinconf_group_set,
	.pin_config_group_get	= siengine_pinconf_group_get,
	.is_generic				= true,
};

static struct pinctrl_desc siengine_pinctrl_desc = {
	.name = "siengine_pinctrl",
	.pctlops = &siengine_pctrl_ops,
	.pmxops = &siengine_pmx_ops,
	.confops = &siengine_pinconf_ops,
	.owner = THIS_MODULE,
};

static int siengine_pinctrl_parse_mux_fn(struct device_node *np,
						struct siengine_pinctrl *pinctrl,
						struct se_pinmux_desc *se_pinmux, u32 mux_cnt)
{
	int i;
	int ret;
	struct se_pinmux_fn_desc *pinmux_fn;

	if (unlikely(0 == mux_cnt)) {
		dev_err(pinctrl->dev, "%pOF, mux-fn array size is empty!\n", np);
		return -EINVAL;
	}

	if (unlikely(mux_cnt > pinctrl->nr_mux_max))
		mux_cnt = pinctrl->nr_mux_max;

	se_pinmux->nr_mux = mux_cnt;

	pinmux_fn = devm_kzalloc(pinctrl->dev, sizeof(*pinmux_fn) *
		mux_cnt, GFP_KERNEL);
	if (unlikely(!pinmux_fn)) {
		dev_err(pinctrl->dev, "mem alloc for pinmux_fn failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < mux_cnt; i++) {
		const char *name;
		struct se_pinmux_fn_desc *pin_fn = &pinmux_fn[i];

		of_property_read_string_index(np, "mux-fn", i, &name);

		if (strcmp(name, "none")) {

			pin_fn->mux_idx = i;
			pin_fn->mux_name = name;
			pin_fn->pin_desc = se_pinmux;

			ret = mux_fn_rb_insert(&pinctrl->rb_mux_fn, pin_fn);
			if (ret) {
				struct se_pinmux_fn_desc *found;

				found = mux_fn_rb_search(&pinctrl->rb_mux_fn, name);
				pr_err("pinmux %d[fn%d] and %d[fn%d] has the same mux name\n",
						found->pin_desc->pin, found->mux_idx, se_pinmux->pin, i);

				return -EINVAL;
			}
		}
	}

	se_pinmux->pinmux_fn = pinmux_fn;

	return 0;
}

static int siengine_pinctrl_parse_pmx_map(struct device_node *np,
						struct siengine_pinctrl *pinctrl)
{
	int i;
	int ret;
	int tmp;
	struct device_node *child = NULL;
	struct se_pinmux_desc *pin_desc;
	struct pinctrl_desc *pctrl = &siengine_pinctrl_desc;

	pin_desc = devm_kzalloc(pinctrl->dev, sizeof(struct se_pinmux_desc) *
			pinctrl->nr_pinmux, GFP_KERNEL);
	if (unlikely(!pin_desc)) {
		dev_err(pinctrl->dev, "mem alloc for se_pinmux_desc descriptors failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < pinctrl->nr_pinmux; i++) {
		u32 hex;
		bool has_safety;
		struct se_pinmux_desc *se_pinmux;
		struct pinctrl_pin_desc *pdesc;

		child = of_get_next_child(np, child);
		if (!child) {
			dev_err(pinctrl->dev, "failed to find pmx-map node %u\n", i);
			return -ENODEV;
		}

		pdesc = (void *)&pctrl->pins[i];
		se_pinmux = &pin_desc[i];

		ret = of_property_read_u32(child, "idx", &tmp);
		if (unlikely(ret)) {
			dev_err(pinctrl->dev, "pmx-map %u idx is not exist\n", i);
			return -EINVAL;
		}
		se_pinmux->idx = i;
		se_pinmux->pin = tmp;
		pdesc->number = tmp;
		pdesc->name = kasprintf(GFP_KERNEL, "%s-%d", pctrl->name, pdesc->number);

		ret = of_property_read_u32(child, "reg", &hex);
		if (unlikely(ret)) {
			dev_err(pinctrl->dev, "pmx-map %u hex is not exist\n", i);
			return -EINVAL;
		}
		se_pinmux->reg = hex;

		ret = of_property_count_strings(child, "mux-fn");
		if (unlikely(ret < 0)) {
			se_pinmux->nr_mux = 0;
			se_pinmux->pinmux_fn = NULL;
		} else {
			tmp = ret;
			ret = siengine_pinctrl_parse_mux_fn(child, pinctrl, se_pinmux, tmp);
			if (ret)
				return ret;
		}

		has_safety = of_property_read_bool(child, "safety");
		if (has_safety)
		{
			bool enable;
			const char *name;
			struct se_safety_pin_desc *safety_desc;
			safety_desc = devm_kzalloc(pinctrl->dev, sizeof(*safety_desc)
								, GFP_KERNEL);
			if (!safety_desc) {
				dev_err(pinctrl->dev, "mem alloc for safety failed\n");
				return -ENOMEM;
			}

			ret = of_property_read_string(child, "safe-fn", &name);
			if (ret) {
				safety_desc->fn_name = NULL;
			} else {
				safety_desc->fn_name = name;
			}

			enable = of_property_read_bool(child, "safe-enable");
			safety_desc->enable = enable;
			if (enable) {
				ret = of_property_read_u32(child, "safe-idx", &tmp);
				if (unlikely(ret)) {
					dev_err(pinctrl->dev, "pmx-map %u safety-idx is not exist\n", i);
					return -EINVAL;
				}
				safety_desc->saf_idx = tmp;
				safety_desc->shift = 8 * (safety_desc->saf_idx % 4);
				ret = of_property_read_u32(child, "safe-reg", &hex);
				if (unlikely(ret)) {
					dev_err(pinctrl->dev, "pmx-map %u safe-reg is not exist\n", i);
					return -EINVAL;
				}
				safety_desc->saf_reg = hex;
#ifdef CONFIG_PINMUX_SAF_ENABLE
				ret = siengine_raw_read(pinctrl->saf_base, safety_desc->saf_reg, &tmp);
				tmp &= ~(PINCTRL_SAF_MASK_FUNC << safety_desc->shift);
				tmp |= ((safety_desc->enable << PINCTRL_SAF_BIT_FUNC) << safety_desc->shift);
				ret = siengine_raw_write(pinctrl->saf_base, safety_desc->saf_reg, tmp);
#endif
			}

			se_pinmux->safety = safety_desc;
		} else {
			se_pinmux->safety = NULL;
		}
	}

	pinctrl->pin_desc = pin_desc;

	return 0;
}

static int siengine_pinctrl_parse_grp_map(struct device_node *pinmux_np,
						struct siengine_pinctrl *pinctrl)
{
	int ret;
	struct device_node *grp;

	for_each_available_child_of_node(pinmux_np, grp) {
		struct device_node *np;

		for_each_available_child_of_node(grp, np) {
			int i;
			const char *function;
			const char **groups;
			unsigned int nr_grp;
			unsigned int *pins;
			unsigned int nr_pins;

			ret = of_property_read_string(np, "function", &function);
			if (ret < 0) {
				continue;
			}

			/* add function */
			ret = of_property_count_strings(np, "groups");
			if (ret < 0) {
				dev_err(pinctrl->dev, "%pOF: could not count groups ret = %d\n", np, ret);
				continue;
			}

			nr_grp = ret;

			groups = devm_kzalloc(pinctrl->dev, sizeof(const char *)
					* nr_grp, GFP_KERNEL);
			if (unlikely(!groups)) {
				dev_err(pinctrl->dev, "mem alloc for function descriptors failed\n");
				return -ENOMEM;
			}

			ret = of_property_read_string_array(np, "groups", groups, nr_grp);
			if (unlikely(ret < 0)) {
				dev_err(pinctrl->dev, "%pOF: could not found groups node property\n", np);
				return ret;
			}

			ret = pinmux_generic_add_function(pinctrl->pctl_dev, function, groups, nr_grp, NULL);
			if (unlikely(ret < 0)) {
				dev_err(pinctrl->dev, "Failed to register function %s\n", function);
				return ret;
			}

			/* add groups */
			nr_pins = nr_grp;
			pins = devm_kzalloc(pinctrl->dev, sizeof(*pins) * nr_pins, GFP_KERNEL);
			if (!pins) {
				dev_err(pinctrl->dev, "mem alloc for function descriptors failed\n");
				return -ENOMEM;
			}

			for (i = 0; i < nr_pins; i++) {
				const char *name = groups[i];
				struct se_pinmux_fn_desc *found;

				found = mux_fn_rb_search(&pinctrl->rb_mux_fn, name);
				if (!found) {
					pr_err("pinmux can not found alt function %s in pinmux-map\n", name);
					continue;
				}

				pins[i] = found->pin_desc->pin;

				ret = pinctrl_generic_add_group(pinctrl->pctl_dev, name, pins + i, 1, found);
				if (ret < 0) {
					dev_err(pinctrl->dev, "Failed to register group %s\n", name);
					return ret;
				}
			}
		}
	}

	return 0;
}

static int siengine_pinctrl_parse_dt(struct platform_device *pdev,
					struct siengine_pinctrl *pinctrl)
{
	int ret;
	int tmp;
	struct pinctrl_pin_desc *pindesc;
	struct device_node *pmx_map_np;
	struct device_node *grp_map_np;
	struct pinctrl_desc *pctrl = &siengine_pinctrl_desc;

	pmx_map_np = of_get_compatible_child(pdev->dev.of_node, "siengine,pin-map");
	if (unlikely(!pmx_map_np)) {
		dev_err(&pdev->dev, "siengine,pin-map dts node not found\n");
		return -EINVAL;
	}

	tmp = of_get_child_count(pmx_map_np);
	if (unlikely(!tmp)) {
		dev_err(pinctrl->dev, "siengine,pin-map dts node is empty\n");
		return -EINVAL;
	}
	pinctrl->nr_pinmux = tmp;

	ret = of_property_read_u32(pmx_map_np, "siengine,max-nr-mux-fn", &tmp);
	if (unlikely(ret || tmp <= 0)) {
		dev_err(pinctrl->dev, "siengine,max-nr-mux-fn dts node is bad\n");
		return -EINVAL;
	}
	pinctrl->nr_mux_max = tmp;

	pindesc = devm_kzalloc(&pdev->dev, sizeof(*pindesc) *
			pinctrl->nr_pinmux, GFP_KERNEL);
	if (unlikely(!pindesc)) {
		dev_err(&pdev->dev, "mem alloc for pin descriptors failed\n");
		return -ENOMEM;
	}

	pctrl->pins = pindesc;
	pctrl->npins = pinctrl->nr_pinmux;

	ret = siengine_pinctrl_parse_pmx_map(pmx_map_np, pinctrl);
	if (ret) {
		dev_err(&pdev->dev, "siengine_pinctrl_parse_pmx_map err\n");
		return -EINVAL;
	}

	ret = pinctrl_register_and_init(pctrl, &pdev->dev, pinctrl, &pinctrl->pctl_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register pinctrl\n");
		return ret;
	}

	grp_map_np = of_get_compatible_child(pdev->dev.of_node, "siengine,grp-map");
	if (unlikely(!pmx_map_np)) {
		dev_err(&pdev->dev, "siengine,grp-map dts node not found\n");
		return -EINVAL;
	}

	ret = siengine_pinctrl_parse_grp_map(grp_map_np, pinctrl);
	if (ret)
		return ret;

	pinctrl_enable(pinctrl->pctl_dev);

	return 0;
}

static int siengine_pinctrl_register(struct platform_device *pdev,
					struct siengine_pinctrl *pinctrl)
{
	return siengine_pinctrl_parse_dt(pdev, pinctrl);
}

static int __maybe_unused siengine_pinctrl_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused siengine_pinctrl_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(siengine_pinctrl_dev_pm_ops, siengine_pinctrl_suspend,
			 siengine_pinctrl_resume);

static int siengine_pinctrl_probe(struct platform_device *pdev)
{
	int ret;
	struct siengine_pinctrl *pinctrl;
	struct device *dev = &pdev->dev;
	struct resource *res;
	void __iomem *base;

	if (!dev->of_node) {
		dev_err(dev, "device tree node not found\n");
		return -ENODEV;
	}

	pinctrl = devm_kzalloc(dev, sizeof(*pinctrl), GFP_KERNEL);
	if (unlikely(!pinctrl)) {
		dev_err(dev, "pinctrl devm_kzalloc failed\n");
		return -ENOMEM;
	}

	pinctrl->dev = dev;
	pinctrl->rb_mux_fn = RB_ROOT;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base)) {
		dev_err(dev, "ioremap failed\n");
		return PTR_ERR(base);
	}

	pinctrl->base_mem = base;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base)) {
		dev_err(dev, "safety base ioremap failed\n");
		return PTR_ERR(base);
	}
	pinctrl->saf_base = base;

	siengine_regmap_config.max_register = resource_size(res) - 4;
	pinctrl->regmap = devm_regmap_init(dev, NULL, pinctrl,
						&siengine_regmap_config);
	if (IS_ERR(pinctrl->regmap)) {
		ret = PTR_ERR(pinctrl->regmap);
		dev_err(dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, pinctrl);

	return siengine_pinctrl_register(pdev, pinctrl);
}

static int siengine_pinctrl_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id siengine_pinctrl_dt_match[] = {
	{ .compatible = "siengine,se1000-pinmux"},
	{},
};
MODULE_DEVICE_TABLE(of, siengine_pinctrl_dt_match);

static struct platform_driver siengine_pinctrl_driver = {
	.probe		= siengine_pinctrl_probe,
	.remove		= siengine_pinctrl_remove,
	.driver = {
		.name	= "siengine-pinctrl",
		.pm = &siengine_pinctrl_dev_pm_ops,
		.of_match_table = of_match_ptr(siengine_pinctrl_dt_match),
	},
};

#if 1
static int __init siengine_pinctrl_drv_register(void)
{
	return platform_driver_register(&siengine_pinctrl_driver);
}
postcore_initcall(siengine_pinctrl_drv_register);
#else
module_platform_driver(siengine_pinctrl_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Siengine pinmux Controller driver");
MODULE_AUTHOR("Mingrui Zhou <Mingrui.Zhou@siengine.com>");

#endif
