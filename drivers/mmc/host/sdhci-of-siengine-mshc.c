// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Synopsys DesignWare Cores Mobile Storage Host Controller
 *
 * Copyright (C) 2018 Synaptics Incorporated
 *
 * Author: Jisheng Zhang <jszhang@kernel.org>
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/sizes.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "sdhci-pltfm.h"

#define SDHCI_DWCMSHC_ARG2_STUFF	GENMASK(31, 16)

/* DWCMSHC specific Mode Select value */
#define DWCMSHC_CTRL_HS400		0x7

#define BOUNDARY_OK(addr, len) \
	((addr | (SZ_128M - 1)) == ((addr + len - 1) | (SZ_128M - 1)))

/*
 * Define PHY registers.
 */
#define SDHCI_PHY_CONFIG	1

#define VENDOR_REG_BASE		0x500
#define EMMC_CTRL_R			(VENDOR_REG_BASE+0x2C)
#define AT_CTRL_R			(VENDOR_REG_BASE+0x40)

#define ENH_STROBE_ENABLE 	BIT(8)
#define CARD_IS_EMMC		BIT(0)
#define SWIN_TH_EN			BIT(2)

#define PHY_REG_BASE			0x300

#define PHY_CNFG_PAD_SN_MASK	0xF
#define PHY_CNFG_PAD_SN_SHIFT	20
#define PHY_CNFG_PAD_SP_MASK	0xF
#define PHY_CNFG_PAD_SP_SHIFT	16

#define PHY_PWRGOOD			BIT(1)
#define PHY_RSTN			BIT(0)

#define PHY_CNFG			(PHY_REG_BASE + 0x0)
#define PHY_CMDPAD_CNFG		(PHY_REG_BASE + 0x4)
#define PHY_DATPAD_CNFG		(PHY_REG_BASE + 0x6)
#define PHY_CLKPAD_CNFG		(PHY_REG_BASE + 0x8)
#define PHY_STBPAD_CNFG		(PHY_REG_BASE + 0xa)
#define PHY_RSTNPAD_CNFG	(PHY_REG_BASE + 0xc)
#define PHY_SDCLKDL_CNFG	(PHY_REG_BASE + 0x1D)
#define PHY_SDCLKDL_DC		(PHY_REG_BASE + 0x1E)

#define PHY_SMPLDL_CNFG		(PHY_REG_BASE + 0x20)
#define PHY_ATDL_CNFG		(PHY_REG_BASE + 0x21)
#define PHY_DLL_CTRL		(PHY_REG_BASE + 0x24)
#define PHY_DLL_CNFG1		(PHY_REG_BASE + 0x25)
#define PHY_DLL_CNFG2		(PHY_REG_BASE + 0x26)
#define PHY_DLLDL_CNFG		(PHY_REG_BASE + 0x28)
#define PHY_DLL_OFFST		(PHY_REG_BASE + 0x29)
#define PHY_DLLBT_CNFG		(PHY_REG_BASE + 0x2c)
#define PHY_DLL_STATUS		(PHY_REG_BASE + 0x2e)
#define PHY_DLLDBGSLKDC_CNFG	(PHY_REG_BASE + 0x32)

#define TXSLEW_CTRL_N_MASK	0xF
#define TXSLEW_CTRL_N_SHIFT	9
#define TXSLEW_CTRL_P_MASK	0xF
#define TXSLEW_CTRL_P_SHIFT	5
#define WEAKPULL_EN_MASK	0x3
#define WEAKPULL_EN_SHIFT	3
#define RXSEL_MASK		0x7
#define RXSEL_SHIFT		0
#define INPSEL_CNFG_MASK	0x3
#define INPSEL_CNFG_SHIFT	2
#define BYPASS_EN		BIT(1)
#define EXTDLY_EN		BIT(0)


struct dwcmshc_priv {
	struct clk	*bus_clk;
	struct clk	*tx_clk;
	void __iomem *sfc_ioaddr;	/* Sfc Mapped address */
};

#define SFC_EMMC_INIT_DONE			0x01
#define SFC_EMMC_INIT_UNDONE		0X0
#define SFC_EMMC_INIT_REG_OFST		0x10

static bool hs400initdone = false;

/*
 * If DMA addr spans 128MB boundary, we split the DMA transfer into two
 * so that each DMA transfer doesn't exceed the boundary.
 */
static void dwcmshc_adma_write_desc(struct sdhci_host *host, void **desc,
				    dma_addr_t addr, int len, unsigned int cmd)
{
	int tmplen, offset;

	if (likely(!len || BOUNDARY_OK(addr, len))) {
		sdhci_adma_write_desc(host, desc, addr, len, cmd);
		return;
	}

	offset = addr & (SZ_128M - 1);
	tmplen = SZ_128M - offset;
	sdhci_adma_write_desc(host, desc, addr, tmplen, cmd);

	addr += tmplen;
	len -= tmplen;
	sdhci_adma_write_desc(host, desc, addr, len, cmd);
}

static void dwcmshc_check_auto_cmd23(struct mmc_host *mmc,
				     struct mmc_request *mrq)
{
	struct sdhci_host *host = mmc_priv(mmc);

	/*
	 * No matter V4 is enabled or not, ARGUMENT2 register is 32-bit
	 * block count register which doesn't support stuff bits of
	 * CMD23 argument on dwcmsch host controller.
	 */
	if (mrq->sbc && (mrq->sbc->arg & SDHCI_DWCMSHC_ARG2_STUFF))
		host->flags &= ~SDHCI_AUTO_CMD23;
	else
		host->flags |= SDHCI_AUTO_CMD23;
}

static void dwcmshc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	dwcmshc_check_auto_cmd23(mmc, mrq);

	sdhci_request(mmc, mrq);
}

static void dwcmshc_set_uhs_signaling(struct sdhci_host *host,
				      unsigned int timing)
{
	u16 ctrl_2;

	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	/* Select Bus Speed Mode for host */
	ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
	if ((timing == MMC_TIMING_MMC_HS200) ||
	    (timing == MMC_TIMING_UHS_SDR104))
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
	else if (timing == MMC_TIMING_UHS_SDR12)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
	else if ((timing == MMC_TIMING_UHS_SDR25) ||
		 (timing == MMC_TIMING_MMC_HS))
		ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
	else if (timing == MMC_TIMING_UHS_SDR50)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR50;
	else if ((timing == MMC_TIMING_UHS_DDR50) ||
		 (timing == MMC_TIMING_MMC_DDR52))
		ctrl_2 |= SDHCI_CTRL_UHS_DDR50;
	else if (timing == MMC_TIMING_MMC_HS400) {
		u16 at_ctrl;
		u16 emmc_ctrl;

		ctrl_2 |= DWCMSHC_CTRL_HS400;

		emmc_ctrl = sdhci_readw(host, EMMC_CTRL_R);
		emmc_ctrl = emmc_ctrl | ENH_STROBE_ENABLE | CARD_IS_EMMC;
		sdhci_writew(host, emmc_ctrl, EMMC_CTRL_R);

		at_ctrl = sdhci_readw(host, AT_CTRL_R);
		at_ctrl = at_ctrl & 0xFFFFFFFB;
		sdhci_writew(host, at_ctrl, AT_CTRL_R);
	}

	sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);
}

static int se1000_sdhci_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);

	/* This field indicating that the MSHC has been initialized.*/
	if (!readl(priv->sfc_ioaddr + SFC_EMMC_INIT_REG_OFST))
		writel(SFC_EMMC_INIT_DONE, priv->sfc_ioaddr + SFC_EMMC_INIT_REG_OFST);

	wmb();

	usleep_range(100, 500);

	return sdhci_execute_tuning(mmc, opcode);
}

static void se1000_sdhci_phy_init(struct sdhci_host *host)
{
	u32 reg = 0;

	reg = sdhci_readl(host, PHY_CNFG);
	reg |= 1;
	sdhci_writel(host, reg, PHY_CNFG);

	//program phy
	reg = sdhci_readl(host, PHY_CNFG);
	reg &= ~(PHY_CNFG_PAD_SN_MASK << PHY_CNFG_PAD_SN_SHIFT);
	reg |= 0xe << PHY_CNFG_PAD_SN_SHIFT;
	reg &= ~(PHY_CNFG_PAD_SP_MASK << PHY_CNFG_PAD_SP_SHIFT);
	reg |= 0xe << PHY_CNFG_PAD_SP_SHIFT;
	sdhci_writel(host, reg, PHY_CNFG);

	reg = sdhci_readw(host, PHY_CMDPAD_CNFG);
	reg &= ~(WEAKPULL_EN_MASK << WEAKPULL_EN_SHIFT);
	reg |= 0x1 << WEAKPULL_EN_SHIFT;
	reg &= ~(RXSEL_MASK << RXSEL_SHIFT);
	reg |= 0x1 << RXSEL_SHIFT;
	reg &= ~(TXSLEW_CTRL_P_MASK << TXSLEW_CTRL_P_SHIFT);
	reg |= 0x3 << TXSLEW_CTRL_P_SHIFT;
	reg &= ~(TXSLEW_CTRL_N_MASK << TXSLEW_CTRL_N_SHIFT);
	reg |= 0x3 << TXSLEW_CTRL_N_SHIFT;
	sdhci_writew(host, reg, PHY_CMDPAD_CNFG);

	reg = sdhci_readw(host, PHY_DATPAD_CNFG);
	reg &= ~(WEAKPULL_EN_MASK << WEAKPULL_EN_SHIFT);
	reg |= 0x1 << WEAKPULL_EN_SHIFT;
	reg &= ~(RXSEL_MASK << RXSEL_SHIFT);
	reg |= 0x1 << RXSEL_SHIFT;
	reg &= ~(TXSLEW_CTRL_P_MASK << TXSLEW_CTRL_P_SHIFT);
	reg |= 0x3 << TXSLEW_CTRL_P_SHIFT;
	reg &= ~(TXSLEW_CTRL_N_MASK << TXSLEW_CTRL_N_SHIFT);
	reg |= 0x3 << TXSLEW_CTRL_N_SHIFT;
	sdhci_writew(host, reg, PHY_DATPAD_CNFG);

	reg = sdhci_readb(host, PHY_SDCLKDL_CNFG);
	reg &= ~(INPSEL_CNFG_MASK << INPSEL_CNFG_SHIFT);
	sdhci_writeb(host, reg, PHY_SDCLKDL_CNFG);

	sdhci_writeb(host, 0x1c, PHY_SDCLKDL_DC);

	reg = sdhci_readw(host, PHY_CLKPAD_CNFG);
	reg &= ~(WEAKPULL_EN_MASK << WEAKPULL_EN_SHIFT);
	reg &= ~(RXSEL_MASK << RXSEL_SHIFT);
	reg |= 0x1 << RXSEL_SHIFT;
	reg &= ~(TXSLEW_CTRL_P_MASK << TXSLEW_CTRL_P_SHIFT);
	reg |= 0x3 << TXSLEW_CTRL_P_SHIFT;
	reg &= ~(TXSLEW_CTRL_N_MASK << TXSLEW_CTRL_N_SHIFT);
	reg |= 0x3 << TXSLEW_CTRL_N_SHIFT;
	sdhci_writew(host, reg, PHY_CLKPAD_CNFG);

	reg = sdhci_readw(host, PHY_STBPAD_CNFG);
	reg &= ~(WEAKPULL_EN_MASK << WEAKPULL_EN_SHIFT);
	reg |= 0x2 << WEAKPULL_EN_SHIFT;
	reg &= ~(RXSEL_MASK << RXSEL_SHIFT);
	reg |= 0x1 << RXSEL_SHIFT;
	sdhci_writew(host, reg, PHY_STBPAD_CNFG);

	reg = sdhci_readw(host, PHY_RSTNPAD_CNFG);
	reg &= ~(WEAKPULL_EN_MASK << WEAKPULL_EN_SHIFT);
	reg |= 0x1 << WEAKPULL_EN_SHIFT;
	reg &= ~(RXSEL_MASK << RXSEL_SHIFT);
	reg |= 0x1 << RXSEL_SHIFT;
	sdhci_writew(host, reg, PHY_RSTNPAD_CNFG);

	reg = sdhci_readb(host, PHY_SMPLDL_CNFG);
	reg &= ~(INPSEL_CNFG_MASK << INPSEL_CNFG_SHIFT);
	reg |= 0x2 << INPSEL_CNFG_SHIFT;
	reg &= ~(BYPASS_EN | EXTDLY_EN);
	sdhci_writeb(host, reg, PHY_SMPLDL_CNFG);

	reg = sdhci_readb(host, PHY_ATDL_CNFG);
	reg &= ~(INPSEL_CNFG_MASK << INPSEL_CNFG_SHIFT);
	reg |= 0x2 << INPSEL_CNFG_SHIFT;
	reg &= ~(BYPASS_EN | EXTDLY_EN);
	sdhci_writeb(host, reg, PHY_ATDL_CNFG);
}

static void sdhci_runtime_pm_bus_off(struct sdhci_host *host)
{
	if (!host->bus_on)
		return;
	host->bus_on = false;
	pm_runtime_put_noidle(host->mmc->parent);
}

void se1000_sdhci_reset(struct sdhci_host *host, u8 mask)
{
	int ret;
	u32 reg;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);

	sdhci_writeb(host, mask, SDHCI_SOFTWARE_RESET);

	if (mask & SDHCI_RESET_ALL) {
		if (readl(priv->sfc_ioaddr + SFC_EMMC_INIT_REG_OFST))
			writel(SFC_EMMC_INIT_UNDONE, priv->sfc_ioaddr + SFC_EMMC_INIT_REG_OFST);

		hs400initdone = false;
		host->clock = 0;
		/* Reset-all turns off SD Bus Power */
		if (host->quirks2 & SDHCI_QUIRK2_CARD_ON_NEEDS_BUS_ON)
			sdhci_runtime_pm_bus_off(host);
	}

	/* hw clears the bit when it's done */
	ret = read_poll_timeout_atomic(sdhci_readb, reg,
		!(reg & mask), 100, 1000000, false, host, SDHCI_SOFTWARE_RESET);

	if (ret) {
		pr_err("%s: Reset 0x%x never completed.\n",
			mmc_hostname(host->mmc), (int)mask);
		sdhci_dumpregs(host);
		return;
	}

	se1000_sdhci_phy_init(host);
}

void se1000_sdhci_set_clock(struct sdhci_host *host, unsigned int clock)
{
	int ret;
	u16 clk;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);

	if (host->clock >= 26000000 && host->clock < 200000000) {
		if (!readl(priv->sfc_ioaddr + SFC_EMMC_INIT_REG_OFST))
			writel(SFC_EMMC_INIT_DONE, priv->sfc_ioaddr + SFC_EMMC_INIT_REG_OFST);
	}

	host->mmc->actual_clock = 0;

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		return;

	clk = sdhci_calc_clk(host, clock, &host->mmc->actual_clock);
	sdhci_enable_clk(host, clk);

	if (host->timing == MMC_TIMING_MMC_HS400 &&
		hs400initdone == false && host->clock == 200000000){
		u32 reg;

		wmb();

		usleep_range(100, 500);

		hs400initdone = true;

		//step2 DLL_CNFG1.WAITCYCL=0x5 DLL_CNFG1.SLVDLY=0x1
		reg = 0x15;
		sdhci_writeb(host, reg, PHY_DLL_CNFG1);//325

		//DLL_CNFG2.JUMPSTEP=0x24
		reg = 0x24;
		sdhci_writeb(host, reg, PHY_DLL_CNFG2);//326

		//step3
		//DLL_OFFST.OFFST=0x2d
		reg = 0x2D;
		sdhci_writeb(host, reg, PHY_DLL_OFFST);//329

		//DLL_DL_CNFG.SLV_INPSEL=0x3
		reg = sdhci_readb(host, PHY_DLLDL_CNFG);//328
		reg |= 0x60;
		sdhci_writeb(host, reg, PHY_DLLDL_CNFG);//328

		//DLLLBT_CNFG.LBT_LOADVAL=0x640
		reg = 0x640;
		sdhci_writew(host, reg, PHY_DLLBT_CNFG);//32c

		//step4 DLL_CTRL.DLL_EN=0x1
		reg = sdhci_readb(host, PHY_DLL_CTRL);//324
		reg |= 0x1;
		sdhci_writeb(host, reg, PHY_DLL_CTRL);//324

		//wait DLL_STATUS.LOCK_STS=0x1
		ret = read_poll_timeout_atomic(sdhci_readb, reg,
			reg & 0x01, 100, 500000, false, host, PHY_DLL_STATUS);
		if (ret)
			pr_err("PLL DLL_STATUS LOCK error,0x%x\n", reg);

		//step5 check DLL_STATUS.ERR_STS=0
		reg = sdhci_readb(host, PHY_DLL_STATUS);
		if (reg & 0x02)//32e
			pr_err("PLL DLL_STATUS ERROR,0x%x\n", reg);
	}
}

static const struct sdhci_ops sdhci_dwcmshc_ops = {
	.set_clock		= se1000_sdhci_set_clock,
	.set_bus_width		= sdhci_set_bus_width,
	.set_uhs_signaling	= dwcmshc_set_uhs_signaling,
	.get_max_clock		= sdhci_pltfm_clk_get_max_clock,
	.reset			= se1000_sdhci_reset,
	.adma_write_desc	= dwcmshc_adma_write_desc,
};

static const struct sdhci_pltfm_data sdhci_dwcmshc_pdata = {
	.ops = &sdhci_dwcmshc_ops,
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
};

static int dwcmshc_probe(struct platform_device *pdev)
{
	int err;
	u32 extra;
	struct sdhci_host *host;
	struct resource *sfc_res;
	struct dwcmshc_priv *priv;
	struct sdhci_pltfm_host *pltfm_host;
	int card_gpio_strong;

	host = sdhci_pltfm_init(pdev, &sdhci_dwcmshc_pdata,
				sizeof(struct dwcmshc_priv));
	if (IS_ERR(host))
		return PTR_ERR(host);

	/*
	 * extra adma table cnt for cross 128M boundary handling.
	 */
	extra = DIV_ROUND_UP_ULL(dma_get_required_mask(&pdev->dev), SZ_128M);
	if (extra > SDHCI_MAX_SEGS)
		extra = SDHCI_MAX_SEGS;
	host->adma_table_cnt += extra;

	pltfm_host = sdhci_priv(host);
	priv = sdhci_pltfm_priv(pltfm_host);

	sfc_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sfc");
	if (!sfc_res) {
		dev_err(&pdev->dev, "siengine sfc_res register zone fail\n");
		goto free_pltfm;
	}

	priv->sfc_ioaddr = devm_ioremap(&pdev->dev, sfc_res->start, resource_size(sfc_res));
	if (!priv->sfc_ioaddr) {
		dev_err(&pdev->dev, "siengine,se1000-sfc_ctrl reg is NULL\n");
		goto free_pltfm;
	}

	pltfm_host->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(pltfm_host->clk)) {
		err = PTR_ERR(pltfm_host->clk);
		dev_err(&pdev->dev, "failed to get core clk: %d\n", err);
		goto free_pltfm;
	}
	err = clk_prepare_enable(pltfm_host->clk);
	if (err)
		goto free_pltfm;

	priv->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (!IS_ERR(priv->bus_clk))
		clk_prepare_enable(priv->bus_clk);

	priv->tx_clk = devm_clk_get(&pdev->dev, "tclk");
	if (!IS_ERR(priv->tx_clk))
		clk_prepare_enable(priv->tx_clk);

	err = mmc_of_parse(host->mmc);
	if (err)
		goto err_clk;

	if (of_find_property(pdev->dev.of_node, "card-gpio-strong",NULL)) {
		card_gpio_strong = of_get_named_gpio(pdev->dev.of_node, "card-gpio-strong", 0);
		if (!gpio_is_valid(card_gpio_strong)) {
			printk("card-gpio-strong: %d is invalid\n", card_gpio_strong);
			return -ENODEV;
		}

		if (gpio_request(card_gpio_strong, "card-gpio-strong")) {
			printk("gpio %d request failed!\n", card_gpio_strong);
			gpio_free(card_gpio_strong);
			return -ENODEV;
		}

		gpio_direction_output(card_gpio_strong, 1);
		gpio_set_value(card_gpio_strong,1);
	}

	sdhci_get_of_property(pdev);

	host->mmc_host_ops.request = dwcmshc_request;
	host->mmc_host_ops.execute_tuning = se1000_sdhci_execute_tuning;

	err = sdhci_add_host(host);
	if (err)
		goto err_clk;

	return 0;

err_clk:
	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);
	clk_disable_unprepare(priv->tx_clk);
free_pltfm:
	sdhci_pltfm_free(pdev);
	return err;
}

static int dwcmshc_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);

	sdhci_remove_host(host, 0);

	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(priv->bus_clk);
	clk_disable_unprepare(priv->tx_clk);

	sdhci_pltfm_free(pdev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dwcmshc_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int ret;

	ret = sdhci_suspend_host(host);
	if (ret)
		return ret;

	clk_disable_unprepare(pltfm_host->clk);
	if (!IS_ERR(priv->bus_clk))
		clk_disable_unprepare(priv->bus_clk);
	if (!IS_ERR(priv->tx_clk))
		clk_disable_unprepare(priv->tx_clk);

	return ret;
}

static int dwcmshc_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct dwcmshc_priv *priv = sdhci_pltfm_priv(pltfm_host);
	int ret;

	ret = clk_prepare_enable(pltfm_host->clk);
	if (ret)
		return ret;

	if (!IS_ERR(priv->bus_clk)) {
		ret = clk_prepare_enable(priv->bus_clk);
		if (ret)
			return ret;
	}

	if (!IS_ERR(priv->tx_clk)) {
		ret = clk_prepare_enable(priv->tx_clk);
		if (ret)
			return ret;
	}

	return sdhci_resume_host(host);
}
#endif

static SIMPLE_DEV_PM_OPS(dwcmshc_pmops, dwcmshc_suspend, dwcmshc_resume);

static const struct of_device_id sdhci_dwcmshc_dt_ids[] = {
	{ .compatible = "siengine,se1000-mshc-sdhci" },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_dwcmshc_dt_ids);

static struct platform_driver sdhci_dwcmshc_driver = {
	.driver	= {
		.name	= "sdhci-dwcmshc",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = sdhci_dwcmshc_dt_ids,
		.pm = &dwcmshc_pmops,
	},
	.probe	= dwcmshc_probe,
	.remove	= dwcmshc_remove,
};
module_platform_driver(sdhci_dwcmshc_driver);

MODULE_DESCRIPTION("SDHCI platform driver for siengine se1000 DWC MSHC");
MODULE_AUTHOR("mingrui zhou <mingrui.zhou@siengine.com>");
MODULE_LICENSE("GPL v2");
