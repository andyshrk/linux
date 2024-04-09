// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Wuhan SiEngine Co.Ltd
 * Author:
 *      Xiaohu Qian <xiaohu.qian@siengine.com>
 *      Mingrui Zhou <mingrui.zhou@siengine.com>
 */

#define pr_fmt(fmt) "pcie-siengine: "fmt

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/regmap.h>

#include <asm/compiler.h>
#include <linux/compiler.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/pci_regs.h>
#include <linux/reset.h>
#include <linux/irqchip/arm-gic-v3.h>

#include "pcie-designware.h"

#define to_siengine_pcie(x) dev_get_drvdata((x)->dev)

/* info located in ahb-sysreg */
#define PCIE_SYS_CTR0 0x0
#define PCIE_SYS_CTR1 0x4

#define PCIE_SIDEBAND_SIZE 0x50

#define PCIE_EP_MSI_MIN_ADDR 0x54000000ULL
#define PCIE_EP_MSI_MAX_ADDR 0x54020000ULL

#define PCIE_MSI64_CFG0 0x13c
#define PCIE_MSI64_CFG1 0x140
#define PCIE_MSI64_CFG2 0x144
#define PCIE_MSI64_CFG3 0x148

#define DRV_NAME		"siengine-pcie"
#define DRV_VERSION		"1.0"

struct dw_plat_pcie_of_data {
	enum dw_pcie_device_mode	mode;
};

struct pcie_top_bus {
	bool init;

	u_int32_t did_remap;
	void __iomem *ahb_misc_base;
	void __iomem *phy_cr_base;
	void __iomem *phy_misc_base;
	void __iomem *sysreg;

	u64 msi_its_addr;
	u64 msi_ep_addr;
};

static struct pcie_top_bus *pcie_top = NULL;

struct dw_plat_pcie {
	int rc_idx;

	// struct dw_pcie	*pci;
	void __iomem	*ahb_base;
	struct dw_pcie			*pci;
	struct regmap			*regmap;
	enum dw_pcie_device_mode	mode;
	int phy_irq;

#if 0
	struct clk	*apb_clk;
	struct clk	*ahb_clk;
	struct clk	*aclk;
	struct clk	*aux_clk;
	struct clk	*phy_ref_clk;

	struct reset_control *phyrst;

	struct reset_control *perst;
	struct reset_control *pwrrst;
	struct reset_control *btnrst;
#endif

	bool linkup;

	/* true : msi is lpi, false : msi is spi */
	bool msi_lpi;
};

/* Registers in apb */
static inline void siengine_apb_ctrl_writel(struct dw_plat_pcie *siengine_pcie,
					 u32 val, u32 reg)
{
	writel(val, pcie_top->phy_cr_base + reg);
}

static inline u32 siengine_apb_ctrl_readl(struct dw_plat_pcie *siengine_pcie, u32 reg)
{
	return readl(pcie_top->phy_cr_base + reg);
}

/* Registers in sideband ahb */
static inline void siengine_sideband_writel(struct dw_plat_pcie *siengine_pcie,
					u32 val, u32 reg)
{
	if (reg < PCIE_SIDEBAND_SIZE) {
		reg += siengine_pcie->rc_idx * PCIE_SIDEBAND_SIZE;
	}

	writel(val, pcie_top->ahb_misc_base + reg);
}

static inline u32 siengine_sideband_readl(struct dw_plat_pcie *siengine_pcie, u32 reg)
{
	if (reg < PCIE_SIDEBAND_SIZE) {
		reg += siengine_pcie->rc_idx * PCIE_SIDEBAND_SIZE;
	}

	return readl(pcie_top->ahb_misc_base + reg);
}

/* Registers in port ahb */
static inline void siengine_ahb_writeb(struct dw_plat_pcie *siengine_pcie,
					u8 val, u32 reg)
{
	writeb(val, siengine_pcie->ahb_base + reg);
}

static inline u8 siengine_ahb_readb(struct dw_plat_pcie *siengine_pcie, u32 reg)
{
	return readb(siengine_pcie->ahb_base + reg);
}

static inline void siengine_ahb_writel(struct dw_plat_pcie *siengine_pcie,
					u32 val, u32 reg)
{
	writel(val, siengine_pcie->ahb_base + reg);
}

static inline u32 siengine_ahb_readl(struct dw_plat_pcie *siengine_pcie, u32 reg)
{
	return readl(siengine_pcie->ahb_base + reg);
}

#if 0 /* clk reset init at ATF */
static long siengine_pcie_prepare_clk_rst(struct dw_plat_pcie *siengine_pcie,
			       struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int err;

	siengine_pcie->apb_clk = devm_clk_get(dev, "pcie_apb");
	if (IS_ERR(siengine_pcie->apb_clk)) {
		dev_warn(dev, "could not get apb_clk\n");
	}
	err = clk_prepare_enable(siengine_pcie->apb_clk);
	if (err) {
		dev_warn(dev, "could not enable apb_clk: %d\n", err);
	}

	siengine_pcie->ahb_clk = devm_clk_get(dev, "pcie_ahb");
	if (IS_ERR(siengine_pcie->ahb_clk)) {
		dev_warn(dev, "could not get ahb_clk\n");
	}
	err = clk_prepare_enable(siengine_pcie->ahb_clk);
	if (err) {
		dev_warn(dev, "could not enable ahb_clk: %d\n", err);
	}

	siengine_pcie->aux_clk = devm_clk_get(dev, "pcie_aux");
	if (IS_ERR(siengine_pcie->aux_clk)) {
		dev_warn(dev, "could not get pcie_aux\n");
	}
	err = clk_prepare_enable(siengine_pcie->aux_clk);
	if (err) {
		dev_warn(dev, "could not enable aux_clk: %d\n", err);
	}

	siengine_pcie->aclk = devm_clk_get(dev, "pcie_aclk");
	if (IS_ERR(siengine_pcie->aclk)) {
		dev_warn(dev, "could not get pcie_aclk\n");
	}
	err = clk_prepare_enable(siengine_pcie->aclk);
	if (err) {
		dev_warn(dev, "could not enable aclk: %d\n", err);
	}

	/* to be confirmed
		siengine_pcie->phy_ref_clk = devm_clk_get(dev, "pcie_phy_ref");
		if (IS_ERR(siengine_pcie->phy_ref_clk))
			return PTR_ERR(siengine_pcie->phy_ref_clk);
	*/

	siengine_pcie->phyrst = devm_reset_control_get_exclusive(dev,
								"phyrst");
	if (IS_ERR(siengine_pcie->phyrst)) {
		dev_warn(dev, "could not get phyrst\n");
	}

	err = reset_control_deassert(siengine_pcie->phyrst);
	if (err) {
		dev_warn(dev, "cannot deassert phyrst： %d\n", err);
	}

	siengine_pcie->perst = devm_reset_control_get_exclusive(dev,
								"perst");
	if (IS_ERR(siengine_pcie->perst)) {
		dev_warn(dev, "could not get perst\n");
	}

	err = reset_control_deassert(siengine_pcie->perst);
	if (err) {
		dev_warn(dev, "cannot deassert perst： %d\n", err);
	}

	siengine_pcie->pwrrst = devm_reset_control_get_exclusive(dev,
								"pwrrst");
	if (IS_ERR(siengine_pcie->pwrrst)) {
		dev_warn(dev, "could not get pwrrst\n");
	}
	err = reset_control_deassert(siengine_pcie->pwrrst);
	if (err) {
		dev_warn(dev, "cannot deassert pwrrst: %d\n", err);
	}

	siengine_pcie->btnrst = devm_reset_control_get_exclusive(dev,
								"btnrst");
	if (IS_ERR(siengine_pcie->btnrst)) {
		dev_warn(dev, "could not get btnrst\n");
	}
	err = reset_control_deassert(siengine_pcie->btnrst);
	if (err) {
		dev_warn(dev, "cannot deassert btnrst: %d\n", err);
	}

	return 0;
}
#endif

static void siengine_pcie_reset_control_off(struct device_node *np, const char *id)
{
	struct reset_control *rst;

	rst = of_reset_control_get_exclusive(np, id);
	if (IS_ERR(rst)) {
		pr_warn("could not get reset control %s from %pOF\n", id, np);
		return;
	}

	if (reset_control_status(rst) > 0)
		reset_control_reset(rst);
}

static int siengine_pcie_rc_disable(struct device_node *np)
{
	// siengine_pcie_reset_control_off(np, "phyrst");
	siengine_pcie_reset_control_off(np, "perst");
	siengine_pcie_reset_control_off(np, "pwrrst");
	siengine_pcie_reset_control_off(np, "btnrst");

	return 0;
}

static int siengine_pcie_clk_gate(struct pcie_top_bus *pcie_top, u32 rc)
{
	u32 val;

	rc *= 4;
	rc = 0xf << rc;

	val = readl(pcie_top->sysreg);
	val &= ~rc;

	writel(val, pcie_top->sysreg);

	return 0;
}

static const struct of_device_id dw_plat_pcie_of_match[];

static int siengine_pcie_top_pm(struct pcie_top_bus *pcie_top)
{
	int ret;
	struct device_node *np;

	for_each_matching_node(np, dw_plat_pcie_of_match) {
		u32 rc_idx;

		if (of_device_is_available(np)) {
			continue;
		}

		ret = of_property_read_u32(np, "rc-id", &rc_idx);
		if (ret) {
			pr_err("read rc-id property error!\n");
			return ret;
		}

		/* disable rc clk */
		siengine_pcie_rc_disable(np);
		siengine_pcie_clk_gate(pcie_top, rc_idx);
	}

	return 0;
}

static int siengine_pcie_top_resource_init(struct platform_device *pdev)
{
	int ret;
	struct device_node *top_node;
	struct device *dev = &pdev->dev;

	if (pcie_top)
		return 0;

	top_node = of_parse_phandle(dev->of_node, "pcie_top", 0);
	if (!top_node) {
		pr_err("unable to find top handle\n");
		return -ENODEV;
	}

	/* Check dts support phy bus. */
	if (!of_device_is_compatible(top_node, "siengine,se1000-pcie-top")) {
		dev_err(dev, "unable to find siengine,se1000-pcie-top controller\n");
		return -ENODEV;
	}

	pcie_top = kzalloc(sizeof(*pcie_top), GFP_KERNEL);
	if (!pcie_top)
		return -ENOMEM;

	ret = of_property_read_u32(top_node, "did-remap-type", &pcie_top->did_remap);
	if (ret < 0) {
		pr_err("unable to obtain did-remap-type property\n");
		goto err_did_remap_type;
	}

	/* ahb axi misc */
	pcie_top->ahb_misc_base = of_iomap(top_node, 0);
	if (IS_ERR(pcie_top->ahb_misc_base)) {
		pr_err("unable to map ahb axi misc memory region\n");
		goto err_did_remap_type;
	}

	/* pcie phy cr */
	pcie_top->phy_cr_base = of_iomap(top_node, 1);
	if (IS_ERR(pcie_top->phy_cr_base)) {
		pr_err("unable to map pcie phy cr memory region\n");
		goto err_phy_cr_base;
	}

	/* pcie phy misc */
	pcie_top->phy_misc_base = of_iomap(top_node, 2);
	if (IS_ERR(pcie_top->phy_misc_base)) {
		pr_err("unable to map pcie phy misc memory region\n");
		goto err_phy_misc_base;
	}

	/* sysreg */
	pcie_top->sysreg = ioremap(0x5822e004, 4);
	if (IS_ERR(pcie_top->sysreg)) {
		pr_err("unable to map pcie sysreg\n");
		goto err_sysreg;
	}

	siengine_pcie_top_pm(pcie_top);

	return 0;
err_sysreg:
	iounmap(pcie_top->phy_misc_base);

err_phy_misc_base:
	iounmap(pcie_top->phy_cr_base);

err_phy_cr_base:
	iounmap(pcie_top->ahb_misc_base);

err_did_remap_type:
	kfree(pcie_top);

	pcie_top = NULL;

	return ret;
}

static long siengine_pcie_get_resource(struct dw_plat_pcie *siengine_pcie,
				   struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct resource *ahb;

	ret = of_property_read_u32(dev->of_node, "rc-id", &siengine_pcie->rc_idx);
	if (ret) {
		dev_err(dev, "read rc-id property error!\n");
		return ret;
	}

	ahb = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ahb-cfg");
	siengine_pcie->ahb_base = devm_ioremap_resource(dev, ahb);
	if (IS_ERR(siengine_pcie->ahb_base))
		return PTR_ERR(siengine_pcie->ahb_base);

#if 0
	dbi = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	siengine_pcie->pci->dbi_base = devm_ioremap_resource(dev, dbi);
	if (IS_ERR(siengine_pcie->pci->dbi_base))
		return PTR_ERR(siengine_pcie->pci->dbi_base);
#endif

	return siengine_pcie_top_resource_init(pdev);
}

static int siengine_pcie_phy_init(struct dw_plat_pcie *siengine_pcie)
{
	return 0;
}

static void siengine_pcie_config_msi(struct pcie_port *pp)
{
	u64 msi_its_addr;
	u64 msi_ep_addr;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct dw_plat_pcie *pcie = to_siengine_pcie(pci);

	msi_ep_addr = pcie_top->msi_ep_addr;

	/* configure msi remap */
	msi_ep_addr |= (u64)pcie_top->did_remap;
	siengine_sideband_writel(pcie, lower_32_bits(msi_ep_addr), PCIE_MSI64_CFG0);
	siengine_sideband_writel(pcie, upper_32_bits(msi_ep_addr), PCIE_MSI64_CFG1);

	msi_its_addr = pcie_top->msi_its_addr;
	siengine_sideband_writel(pcie, lower_32_bits(msi_its_addr), PCIE_MSI64_CFG2);
	siengine_sideband_writel(pcie, upper_32_bits(msi_its_addr), PCIE_MSI64_CFG3);

	dsb(sy);
}

static void siengine_pcie_lpi_msi_init(struct pcie_port *pp)
{
	u64 msi_ep_addr;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	msi_ep_addr = pcie_top->msi_ep_addr;

	/* program the msi_data */
	dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_LO, lower_32_bits(msi_ep_addr));
	dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_HI, upper_32_bits(msi_ep_addr));

	dsb(sy);
}

static int siengine_pcie_spi_msi_init(struct pcie_port *pp)
{
	int ret;
	u64 msi_target;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	ret = dma_set_mask(pci->dev, DMA_BIT_MASK(32));
	if (ret)
		dev_warn(pci->dev, "Failed to set DMA mask to 32-bit. Devices with only 32-bit MSI support may not work properly\n");

	pp->msi_data = dma_map_single_attrs(pci->dev, &pp->msi_msg,
						sizeof(pp->msi_msg),
						DMA_FROM_DEVICE,
						DMA_ATTR_SKIP_CPU_SYNC);
	if (dma_mapping_error(pci->dev, pp->msi_data)) {
		dev_err(pci->dev, "Failed to map MSI data\n");
		pp->msi_data = 0;
		return -ENOMEM;
	}

	msi_target = pp->msi_data;

	/* program the msi_data */
	dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_LO, lower_32_bits(msi_target));
	dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_HI, upper_32_bits(msi_target));

	dsb(sy);

	return 0;
}

/* configure msi config, so only run once */
static int siengine_pcie_msi_cfg(struct pcie_port *pp)
{
	int ret;
	struct resource res;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	u64 its_target;
	struct device_node *msi_node;

	msi_node = of_parse_phandle(pci->dev->of_node, "msi-parent", 0);
	if (!msi_node) {
		dev_err(pci->dev, "unable to find ITS-MSI handle\n");
		return -ENODEV;
	}

	/* derive GITS_TRANSLATER address from GICv3 */
	ret = of_address_to_resource(msi_node, 0, &res);
	if (ret < 0) {
		dev_err(pci->dev, "unable to obtain MSI controller resources\n");
		return ret;
	}

	/*
	 * Check points to ARM GICv3 ITS, which is the only
	 * supported external MSI controller that requires steering.
	 */
	if (!of_device_is_compatible(msi_node, "arm,gic-v3-its")) {
		dev_err(pci->dev, "unable to find compatible MSI controller\n");
		return -ENODEV;
	}

	its_target = res.start + GITS_TRANSLATER;

	pcie_top->msi_its_addr = its_target;
	pcie_top->msi_ep_addr = PCIE_EP_MSI_MIN_ADDR + 0x40;

	/* program the msi_data */
	siengine_pcie_config_msi(pp);

	return 0;
}

static int dw_plat_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	dw_pcie_setup_rc(pp);
	dw_pcie_wait_for_link(pci);
	dw_pcie_msi_init(pp);

	return 0;
}

static void dw_plat_set_num_vectors(struct pcie_port *pp)
{
	pp->num_vectors = MAX_MSI_IRQS;
}

static const struct dw_pcie_host_ops dw_plat_pcie_host_ops = {
	.host_init = dw_plat_pcie_host_init,
	.set_num_vectors = dw_plat_set_num_vectors,
};

static int siengine_plat_pcie_host_init(struct pcie_port *pp)
{
	int ret = 0;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct dw_plat_pcie *siengine_pcie = to_siengine_pcie(pci);

	dw_pcie_setup_rc(pp);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		if (siengine_pcie->msi_lpi) {

			if (!pcie_top->init) {
				/* run once */
				ret = siengine_pcie_msi_cfg(pp);
				if (ret) {
					pr_err("siengine_pcie_msi_cfg error %d\n", ret);
					return ret;
				}
				pcie_top->init = true;
			}

			/* msi to its-lpi */
			siengine_pcie_lpi_msi_init(pp);
		} else {
			/* msi to its-spi */
			ret = siengine_pcie_spi_msi_init(pp);
			if (ret)
				return ret;
		}
	}

	/* maybe already linkup in bootloader */
	if (siengine_pcie->linkup)
		dev_info(pci->dev, "Link up\n");
	else {
		uint8_t ctr0;

		/* Set app_ltssm_enable, start training */
		ctr0 = siengine_ahb_readb(siengine_pcie, PCIE_SYS_CTR0);
		ctr0 |= BIT(0);
		siengine_ahb_writeb(siengine_pcie, ctr0, PCIE_SYS_CTR0);

		/* waiting linkup interrupt */
		ret = -EPROBE_DEFER;
	}

	return ret;
}

static phys_addr_t siengine_lpi_pcie_get_msi_addr(struct pcie_port *pp)
{
	return pcie_top->msi_ep_addr;
}

static const struct dw_pcie_host_ops dw_lpi_pcie_host_ops = {
	.host_init = siengine_plat_pcie_host_init,
	.get_msi_addr = siengine_lpi_pcie_get_msi_addr,
	.set_num_vectors = dw_plat_set_num_vectors,
};

static phys_addr_t siengine_spi_pcie_get_msi_addr(struct pcie_port *pp)
{
	return pp->msi_data;
}

static const struct dw_pcie_host_ops dw_spi_pcie_host_ops = {
	.host_init = siengine_plat_pcie_host_init,
	.get_msi_addr = siengine_spi_pcie_get_msi_addr,
	.set_num_vectors = dw_plat_set_num_vectors,
};

static int dw_plat_pcie_establish_link(struct dw_pcie *pci)
{
	return 0;
}

static u64 siengine_cpu_addr_fixup(struct dw_pcie *pcie, u64 cpu_addr)
{
	return lower_32_bits(cpu_addr);
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.start_link = dw_plat_pcie_establish_link,
	.cpu_addr_fixup = siengine_cpu_addr_fixup,
};

static void dw_plat_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	enum pci_barno bar;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++)
		dw_pcie_ep_reset_bar(pci, bar);
}

static int dw_plat_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
				     enum pci_epc_irq_type type,
				     u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return dw_pcie_ep_raise_legacy_irq(ep, func_no);
	case PCI_EPC_IRQ_MSI:
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	case PCI_EPC_IRQ_MSIX:
		return dw_pcie_ep_raise_msix_irq(ep, func_no, interrupt_num);
	default:
		dev_err(pci->dev, "UNKNOWN IRQ type\n");
	}

	return 0;
}

static const struct pci_epc_features dw_plat_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = true,
};

static const struct pci_epc_features*
dw_plat_pcie_get_features(struct dw_pcie_ep *ep)
{
	return &dw_plat_pcie_epc_features;
}

static const struct dw_pcie_ep_ops pcie_ep_ops = {
	.ep_init = dw_plat_pcie_ep_init,
	.raise_irq = dw_plat_pcie_ep_raise_irq,
	.get_features = dw_plat_pcie_get_features,
};

static int siengine_plat_add_pcie_port(struct dw_plat_pcie *dw_plat_pcie,
				 struct platform_device *pdev)
{
	struct dw_pcie *pci = dw_plat_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;

	int ret;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq_byname(pdev, "msi");
		if (pp->msi_irq < 0)
			return pp->msi_irq;
	}

	if (dw_plat_pcie->msi_lpi)
		pp->ops = &dw_lpi_pcie_host_ops;
	else
		pp->ops = &dw_spi_pcie_host_ops;

	ret = siengine_pcie_get_resource(dw_plat_pcie, pdev);
	if (ret) {
		dev_err(dev, "get resource failed\n");
		return ret;
	}

	dw_plat_pcie->msi_lpi =
		of_property_read_bool(dev->of_node, "msi-parent") &&
		of_property_read_bool(dev->of_node, "msi-map");

	siengine_pcie_phy_init(dw_plat_pcie);

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_dbg(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int dw_plat_add_pcie_port(struct dw_plat_pcie *dw_plat_pcie,
				 struct platform_device *pdev)
{
	struct dw_pcie *pci = dw_plat_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;

	pp->irq = platform_get_irq(pdev, 1);
	if (pp->irq < 0)
		return pp->irq;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq(pdev, 0);
		if (pp->msi_irq < 0)
			return pp->msi_irq;
	}

	pp->ops = &dw_plat_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_warn(dev, "Failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int dw_plat_add_pcie_ep(struct dw_plat_pcie *dw_plat_pcie,
			       struct platform_device *pdev)
{
	int ret;
	struct dw_pcie_ep *ep;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = dw_plat_pcie->pci;

	ep = &pci->ep;
	ep->ops = &pcie_ep_ops;

	pci->dbi_base2 = devm_platform_ioremap_resource_byname(pdev, "dbi2");
	if (IS_ERR(pci->dbi_base2))
		return PTR_ERR(pci->dbi_base2);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	if (!res)
		return -EINVAL;

	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);

	ret = dw_pcie_ep_init(ep);
	if (ret) {
		dev_err(dev, "Failed to initialize endpoint\n");
		return ret;
	}
	return 0;
}

static irqreturn_t pcie_phy_interrupt(int irq, void *dev_id)
{
	struct dw_plat_pcie *dw_plat_pcie = dev_id;

	disable_irq_nosync(irq);

	dw_plat_pcie->linkup = true;

	return IRQ_HANDLED;
}

static int dw_plat_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_plat_pcie *dw_plat_pcie;
	struct dw_pcie *pci;
	struct resource *res;  /* Resource from DT */
	int ret;
	const struct of_device_id *match;
	const struct dw_plat_pcie_of_data *data;
	enum dw_pcie_device_mode mode;

	match = of_match_device(dw_plat_pcie_of_match, dev);
	if (!match)
		return -EINVAL;

	data = (struct dw_plat_pcie_of_data *)match->data;
	mode = (enum dw_pcie_device_mode)data->mode;

	dw_plat_pcie = devm_kzalloc(dev, sizeof(*dw_plat_pcie), GFP_KERNEL);
	if (!dw_plat_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &dw_pcie_ops;

	dw_plat_pcie->pci = pci;
	dw_plat_pcie->mode = mode;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (!res)
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	pci->dbi_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);

	platform_set_drvdata(pdev, dw_plat_pcie);

	switch (dw_plat_pcie->mode) {
	case DW_PCIE_RC_TYPE:

		if (!(IS_ENABLED(CONFIG_PCIE_DW_PLAT_HOST) || IS_ENABLED(CONFIG_PCIE_SIENGINE)))
			return -ENODEV;

		if (!strcmp(match->compatible, "siengine,se1000-pcie")) {

			/* ignore phy linkup interrupt request error */
			do {
				int irq;

				ret = irq = platform_get_irq_byname(pdev, "phy");
				if (ret < 0) {
					pr_err("pcie link irq get failed.\n");
					break;
				}

				ret = devm_request_irq(dev, irq, pcie_phy_interrupt,
							IRQF_TRIGGER_HIGH | IRQF_ONESHOT, "pcie-phy", dw_plat_pcie);
				if (ret) {
					pr_err("pcie phy irq request failed.\n");
					break;
				}

				dw_plat_pcie->phy_irq = irq;
			} while(0);

			if (ret < 0)
				return ret;

			ret = siengine_plat_add_pcie_port(dw_plat_pcie, pdev);
		}
		else
			ret = dw_plat_add_pcie_port(dw_plat_pcie, pdev);
		if (ret < 0)
			return ret;
		break;
	case DW_PCIE_EP_TYPE:
		if (!IS_ENABLED(CONFIG_PCIE_DW_PLAT_EP))
			return -ENODEV;

		ret = dw_plat_add_pcie_ep(dw_plat_pcie, pdev);
		if (ret < 0)
			return ret;
		break;
	default:
		dev_err(dev, "INVALID device type %d\n", dw_plat_pcie->mode);
		return -ENODEV;
	}

	platform_set_drvdata(pdev, dw_plat_pcie);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pcie_pm_suspend(struct device *dev)
{
	struct dw_pcie *pci;
	struct pcie_port *pp;
	struct dw_plat_pcie *dw_plat_pcie;

	dw_plat_pcie = platform_get_drvdata(to_platform_device(dev));
	if (NULL == dw_plat_pcie)
		return -EINVAL;

	pci = dw_plat_pcie->pci;
	pp = &pci->pp;

	if (IS_ENABLED(CONFIG_PCI_MSI))
		disable_irq_nosync(pp->msi_irq);

	disable_irq_nosync(dw_plat_pcie->phy_irq);

	return 0;
}

static int pcie_pm_resume(struct device *dev)
{
	struct dw_pcie *pci;
	struct pcie_port *pp;
	struct dw_plat_pcie *dw_plat_pcie;

	dw_plat_pcie = platform_get_drvdata(to_platform_device(dev));
	if (NULL == dw_plat_pcie)
		return -EINVAL;

	enable_irq(dw_plat_pcie->phy_irq);

	pci = dw_plat_pcie->pci;
	pp = &pci->pp;

	dw_pcie_setup_rc(pp);
	dw_pcie_msi_init(pp);

	usleep_range(500,5000);

	/* add root port and downstream devices */
	pci_lock_rescan_remove();
	pci_rescan_bus(pp->bridge->bus);
	pci_unlock_rescan_remove();

	if (IS_ENABLED(CONFIG_PCI_MSI))
		enable_irq(pp->msi_irq);

	return 0;
}
#else
#define pcie_pm_suspend	NULL
#define pcie_pm_resume	NULL
#endif /* CONFIG_PM_SLEEP */

static const struct dw_plat_pcie_of_data dw_plat_pcie_rc_of_data = {
	.mode = DW_PCIE_RC_TYPE,
};

static const struct dw_plat_pcie_of_data dw_plat_pcie_ep_of_data = {
	.mode = DW_PCIE_EP_TYPE,
};

static const struct of_device_id dw_plat_pcie_of_match[] = {
	{
		.compatible = "siengine,se1000-pcie",
		.data = &dw_plat_pcie_rc_of_data,
	},
	{},
};

static const struct dev_pm_ops pcie_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(pcie_pm_suspend, pcie_pm_resume)
};

static struct platform_driver dw_plat_pcie_driver = {
	.driver = {
		.name	= "siengine-pcie",
		.of_match_table = dw_plat_pcie_of_match,
		.suppress_bind_attrs = true,
		.pm = &pcie_pm_ops,
	},
	.probe = dw_plat_pcie_probe,
};

#if 1
int __init siengine_pcie_driver_init(void)
{
	return platform_driver_register(&dw_plat_pcie_driver);
}

void __exit siengine_pcie_driver_exit(void)
{
	platform_driver_unregister(&dw_plat_pcie_driver);
}

device_initcall_sync(siengine_pcie_driver_init);
module_exit(siengine_pcie_driver_exit);
#else
builtin_platform_driver(dw_plat_pcie_driver);
#endif

MODULE_AUTHOR("Mingrui Zhou<mingrui.zhou@siengine.com>");
MODULE_DESCRIPTION("Siengine "DRV_NAME" Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
