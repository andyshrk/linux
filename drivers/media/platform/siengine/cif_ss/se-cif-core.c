// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie Camera Interface driver
 *
 * Author: Siengine Technology, Inc.
 */

#include "se-media-dev.h"
#include <linux/of_reserved_mem.h>

#if IS_ENABLED(CONFIG_SE_CIF_DEBUG)
unsigned int cif_perf_enable;
module_param(cif_perf_enable, uint, 0644);
MODULE_PARM_DESC(cif_perf_enable, "cif performance enable");
unsigned int jitter_limit = 10;
module_param(jitter_limit, uint, 0644);
MODULE_PARM_DESC(jitter_limit, "cif jitter limit frame");
#endif

static const struct of_device_id se_cif_of_match[];
static struct cif_dm_dev *cif_dm;

static irqreturn_t se_cif_irq_handler(int irq, void *priv)
{
	struct se_cif_dev *se_cif = priv;
	struct device *dev = &se_cif->pdev->dev;
	u32 status;
	u32 act_a, act_b;
	unsigned long flags;
	dev_dbg(dev, "enter %s\n", __func__);
	spin_lock_irqsave(&se_cif->slock, flags);
	if (se_cif->cif_cap.se_nr_enable && se_cif->cif_cap.nr_dev.rdma_mode == NORMAL_MODE) {
		cisp_nr_start_rdma(&(se_cif->cif_cap.nr_dev));
		cisp_nr_process_irq(&(se_cif->cif_cap.nr_dev));
		dev_dbg(dev, "nr start rdma called");
	}

	status = se_cif_get_irq_status(se_cif);
	se_cif_clean_irq_status(se_cif, status);
	se_cif->status = status;

	if(status & CIF_INT_SOURCE_BUF_A2B_MASK)
	{
		se_cif->buf.id = SE_CIF_BUFA;
		act_a = se_cif_read_buffer_act_a(se_cif);
		dev_dbg(dev, "buffer act a: 0x%x", act_a);
	}
	else if (status & CIF_INT_SOURCE_BUF_B2A_MASK)
	{
		se_cif->buf.id = SE_CIF_BUFB;
		act_b = se_cif_read_buffer_act_b(se_cif);
		dev_dbg(dev, "buffer act b: 0x%x", act_b);
	}
	else if (status & CIF_INT_SOURCE_DISCARD_MASK)
	{
		dev_warn(dev, "int discard");
		goto irq_done;
	}
	else if (status & CIF_INT_SOURCE_WRONG_WIDTH_MASK)
	{
		dev_err(dev, "wrong width");
		goto irq_done;
	}
	else if (status & CIF_INT_SOURCE_WRONG_HEIGHT_MASK)
	{
		dev_err(dev, "wrong height");
		goto irq_done;
	}
	else if (status & CIF_INT_SOURCE_ERR_AXI_RESPONSE_MASK)
	{
		dev_err(dev, "axi response error");
		goto irq_done;
	}
	if ((status & CIF_INT_SOURCE_BUF_A2B_MASK) || (status & CIF_INT_SOURCE_BUF_B2A_MASK))
		se_cif_cap_frame_write_done(se_cif);
/*  Need to update according lastest CIF MAS
	if (status & CHNL_STS_FRM_STRD_MASK) {
		se_cif_cap_frame_write_done(se_cif);
	}
	if (status & (CHNL_STS_AXI_WR_ERR_Y_MASK |
					CHNL_STS_AXI_WR_ERR_U_MASK |
					CHNL_STS_AXI_WR_ERR_V_MASK))
		dev_dbg(dev, "%s, IRQ AXI Error stat=0x%X\n", __func__, status);
*/
	/*
	if (se_cif->cif_cap.se_nr_enable) {
		cisp_nr_set_rf_wr_addr(&se_cif->cif_cap.nr_dev, se_cif->cif_cap.nr_dev.rf_wr_addr);
		cisp_nr_set_rf_rd_addr(&se_cif->cif_cap.nr_dev, se_cif->cif_cap.nr_dev.rf_rd_addr);
		cisp_nr_start_rdma(&(se_cif->cif_cap.nr_dev));
	}
	*/
irq_done:
	spin_unlock_irqrestore(&se_cif->slock, flags);
	return IRQ_HANDLED;
}

static int se_cif_parse_dt(struct se_cif_dev *se_cif)
{
	struct device *dev = &se_cif->pdev->dev;
	struct device_node *node = dev->of_node;
	int ret = 0;

	se_cif->id = of_alias_get_id(node, "cif");
	ret = of_property_read_u32_array(node, "interface", se_cif->interface, 3);
	if (ret < 0)
		return ret;

	dev_info(dev, "%s, cif_%d,interface(%d, %d, %d)\n", __func__, se_cif->id,
		se_cif->interface[0], se_cif->interface[1], se_cif->interface[2]);

	return 0;
}


static int se_cif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct se_cif_dev *se_cif;
	struct resource *res;
	const struct of_device_id *of_id;
	int ret = 0;

	of_id = of_match_node(se_cif_of_match, dev->of_node);
	if (!of_id)
		return -EINVAL;

	if( !strcmp(of_id->compatible, se_cif_of_match[1].compatible) ) {
		cif_dm = devm_kzalloc(dev, sizeof(*cif_dm), GFP_KERNEL);
		if (!cif_dm)
			return -ENOMEM;
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		cif_dm->dm_regs = devm_ioremap_resource(dev, res);
		dev_dbg(dev, "cif_dm->dm_regs: 0x%llx\n", (uint64_t)(cif_dm->dm_regs));
		if (IS_ERR(cif_dm->dm_regs)) {
			dev_err(dev, "Failed to get CIF DM register map\n");
			return PTR_ERR(cif_dm->dm_regs);
		}
		return 0;
	}

	se_cif = devm_kzalloc(dev, sizeof(*se_cif), GFP_KERNEL);
	if (!se_cif)
		return -ENOMEM;
	se_cif->pdev = pdev;

	ret = se_cif_parse_dt(se_cif);
	if (ret < 0)
		return ret;
	dev_info(dev, "cif device id: %d\n", se_cif->id);
	if (se_cif->id >= SE_CIF_MAX_DEVS || se_cif->id < 0) {
		dev_err(dev, "Invalid driver data or device id (%d)\n",
			se_cif->id);
		return -EINVAL;
	}

	init_waitqueue_head(&se_cif->irq_queue);
	spin_lock_init(&se_cif->slock);
	mutex_init(&se_cif->lock);
	atomic_set(&se_cif->open_count, 0);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	se_cif->regs = devm_ioremap_resource(dev, res);
	dev_dbg(dev, "se_cif->regs: 0x%llx\n", (uint64_t)(se_cif->regs));
	if (IS_ERR(se_cif->regs)) {
		dev_err(dev, "Failed to get CIF register map\n");
		return PTR_ERR(se_cif->regs);
	}

	if ((se_cif->id >=0) && (se_cif->id < 18)) {
		cisp_nr_set_default(&(se_cif->cif_cap.nr_dev));
		se_cif->cif_cap.nr_dev.base = se_cif->regs + 0x1000;  //only for cif0 - cif17
	}
	//se_cif->cif_cap.se_nr_enable = 1;
	dev_dbg(dev, "nr base address 0x%llx\n", (uint64_t)(se_cif->cif_cap.nr_dev.base));
	se_cif->dm_regs = cif_dm->dm_regs;
	dev_dbg(dev, "se_cif->dm_regs: %pK\n", (cif_dm->dm_regs));

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(dev, "Failed to get IRQ resource\n");
		return -ENXIO;
	}

	dev_dbg(dev, "irq number: %d\n", (int)res->start);

	ret = devm_request_irq(dev, res->start, se_cif_irq_handler,
		(res->flags & IRQF_TRIGGER_MASK ) | IRQF_SHARED, dev_name(dev), se_cif);
	if (ret < 0) {
		dev_err(dev, "failed to install irq (%d)\n", ret);
		goto err_clk;
	}

	ret = se_cif_initialize_capture_subdev(se_cif);
	if (ret < 0) {
		dev_err(dev, "failed to init cap subdev (%d)\n", ret);
		goto err_clk;
	}

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if ( ret < 0 )
		dev_err(dev, "dma_set_mask_and_coherent failed.");

	platform_set_drvdata(pdev, se_cif);

	pm_runtime_enable(dev);
#if IS_ENABLED(CONFIG_SE_CIF_DEBUG)
	se_cif_create_capabilities_sysfs(pdev);
#endif
	dev_dbg(dev, "se_cif.%d registered successfully\n", se_cif->id);

	return 0;

err_clk:
	se_cif_clock_disable(se_cif);
	//se_cif_unregister_capture_subdev(se_cif);
	return ret;
}

static int se_cif_remove(struct platform_device *pdev)
{
	//struct se_cif_dev *se_cif = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	//se_cif_unregister_capture_subdev(se_cif);
	pm_runtime_disable(dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int se_cif_pm_suspend(struct device *dev)
{
	/* disable this function to make suspend arch work */
#if 0
	struct se_cif_dev *se_cif = dev_get_drvdata(dev);

	if (se_cif->is_streaming) {
		dev_warn(dev, "running, prevent entering suspend.\n");
		return -EAGAIN;
	}

	return pm_runtime_force_suspend(dev);
#endif
	return 0;
}

static int se_cif_pm_resume(struct device *dev)
{
	/* disable this function to make suspend arch work */
#if 0
	int ret = 0;

	ret = pm_runtime_force_resume(dev);
	return ret;
#endif
	return 0;
}
#endif

static int se_cif_runtime_suspend(struct device *dev)
{
	return 0;
}

static int se_cif_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops se_cif_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(se_cif_pm_suspend, se_cif_pm_resume)
	SET_RUNTIME_PM_OPS(se_cif_runtime_suspend, se_cif_runtime_resume, NULL)
};

static const struct of_device_id se_cif_of_match[] = {
	{.compatible = "siengine,se-cif", },
	{.compatible = "siengine,cif-dm", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, se_cif_of_match);

static struct platform_driver se_cif_driver = {
	.probe		= se_cif_probe,
	.remove		= se_cif_remove,
	.driver = {
		.of_match_table = se_cif_of_match,
		.name		= CIF_DRIVER_NAME,
		.pm		= &se_cif_pm_ops,
	}
};

module_platform_driver(se_cif_driver);

MODULE_AUTHOR("Siengine Technology, Inc.");
MODULE_DESCRIPTION("CIF Subsystem driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("CIF");
MODULE_VERSION("1.0");
