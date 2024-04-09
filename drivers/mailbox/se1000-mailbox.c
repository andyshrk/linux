/*
 * Siengine's Se1000 mailbox driver
 *
 * Copyright (c) 2018 Siengine Limited.
 *
 * Author: Mingrui.Zhou <mingrui.zhou@siengine.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt)		"se1000-mailbox : " fmt

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/version.h>
#include <linux/irq.h>

#include "mailbox.h"
#include "se1000-mailbox.h"

/* Configuration and Status Registers */
#define MBOX_REG_TXD			0x00
#define MBOX_REG_TXST			0x04
#define MBOX_REG_RXD			0x08
#define MBOX_REG_RXST			0x0c
#define MBOX_REG_RSTN			0x10
#define MBOX_REG_ERR			0x14


/* MBOX_TXST */
#define MBOX_TXFIFO_CNT_BIT		4
#define MBOX_TXFIFO_CNT_MASK		0xf
#define MBOX_TXFIFO_EMPTY		BIT(1)
#define MBOX_TXFIFO_FULL			BIT(0)

/* MBOX_RXST */
#define MBOX_RXFIFO_CNT_BIT		4
#define MBOX_RXFIFO_CNT_MASK		0xf
#define MBOX_RXFIFO_EMPTY		BIT(1)
#define MBOX_RXFIFO_FULL			BIT(0)

/* MBOX_RSTN */
#define MBOX_RXFIFO_RST			BIT(1)
#define MBOX_TXFIFO_RST			BIT(0)

/* IPS_ERR */
#define MBOX_RXFIFO_UDFL			BIT(1)
#define MBOX_TXFIFO_OVFL			BIT(0)

static struct se1000_mbox *to_se1000_mbox(struct mbox_controller *mbox)
{
	return container_of(mbox, struct se1000_mbox, controller);
}

static int se1000_mbox_startup(struct mbox_chan *chan)
{
	unsigned int retry;
	struct se1000_mbox_chan *mchan = chan->con_priv;

	for (retry = 10; retry; retry--) {
		/* Check if channel tx fifo is empty */
		if ((readl(mchan->ch_base + MBOX_REG_TXST) & MBOX_TXFIFO_FULL))
			continue;

		break;
	}

	if (!retry)
		dev_err(mchan->mbox->dev, "%s: failed to acquire channel\n", __func__);

	if (retry) {
		enable_irq(mchan->irq);
	}

	return retry ? 0 : -ETIMEDOUT;
}

static int se1000_mbox_send_data(struct mbox_chan *chan, void *msg)
{
	u32 data = *(u32*)msg;
	struct se1000_mbox_chan *mchan = chan->con_priv;

	/* check if Mailbox tx fifo is full */
	if ((readl(mchan->ch_base + MBOX_REG_TXST) & MBOX_TXFIFO_FULL))
		return -EBUSY;

	/* Fill message data */
	writel_relaxed(data, mchan->ch_base + MBOX_REG_TXD);

	return 0;
}

static void se1000_mbox_shutdown(struct mbox_chan *chan)
{
	u32 val;
	struct se1000_mbox_chan *mchan = chan->con_priv;

	disable_irq(mchan->irq);

	/* reset tx and rx fifo */
	val = (MBOX_RXFIFO_RST | MBOX_TXFIFO_RST);
	writel(val, mchan->ch_base + MBOX_REG_RSTN);
}

static bool se1000_mbox_peek_data(struct mbox_chan *chan)
{
	struct se1000_mbox_chan *mchan = chan->con_priv;

	return !(readl(mchan->ch_base + MBOX_REG_RXST) & MBOX_RXFIFO_EMPTY);
}

static bool se1000_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct se1000_mbox_chan *mchan = chan->con_priv;

	return !(readl(mchan->ch_base + MBOX_REG_TXST) & MBOX_TXFIFO_FULL);
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 0, 0)
int se1000_mbox_flush(struct mbox_chan *chan, unsigned long timeout)
{
	return 0;
}
#endif

static const struct mbox_chan_ops se1000_mbox_ops = {
	.startup		= se1000_mbox_startup,
	.send_data		= se1000_mbox_send_data,
	.shutdown		= se1000_mbox_shutdown,
	.peek_data		= se1000_mbox_peek_data,
	.last_tx_done	= se1000_mbox_last_tx_done,
#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 0, 0)
	.flush			= se1000_mbox_flush,
#endif
};

static irqreturn_t mbox_interrupt(int irq, void *dev_id)
{
	struct se1000_mbox_chan *mchan = dev_id;
	struct mbox_chan *chan = &mchan->mbox->chans[mchan->idx];
	int virq;

	while (se1000_mbox_peek_data(chan)) {
		u32 msg;
		msg = readl(mchan->ch_base + MBOX_REG_RXD);

		virq = irq_find_mapping(mchan->irq_domain, msg);
		if(!virq)
			mbox_chan_received_data(chan, (void *)(uintptr_t)msg);
		else
			generic_handle_irq(virq);
	}

	return IRQ_HANDLED;
}

static struct mbox_chan *se1000_mbox_xlate(struct mbox_controller *controller,
					   const struct of_phandle_args *spec)
{
	int ch;
	struct device_node *node;
	phandle phandle = spec->args[0];
	struct se1000_mbox *mbox = to_se1000_mbox(controller);

	node = of_find_node_by_phandle(phandle);
	if (!node) {
		pr_err("%s: could not find node phandle 0x%x\n",
			__func__, phandle);
		return ERR_PTR(-ENODEV);
	}

	for (ch = 0; ch < mbox->nr_mbox; ch++) {
		struct se1000_mbox_chan *mchan = &mbox->mboxes[ch];

		if (mchan->ch_np == node)
			break;
	}

	return (mbox->nr_mbox == ch) ? NULL : &mbox->chans[ch];
}

static const char * const se1000_channel_match[] = {
	"siengine,se1000-ap0-mbox-channel",
	"siengine,se1000-ap1-mbox-channel",
	NULL,
};

static int se1000_mbox_channel_count(struct device_node *np)
{
	struct device_node *child;
	int cnt = 0;

	for_each_child_of_node(np, child) {
		if (!of_device_compatible_match(child, se1000_channel_match))
			continue;
		cnt++;
	}

	return cnt;
}

/* empty function see kernel/irq/chip.c */
static void se1000_ipc_mask_irq(struct irq_data *irqd)
{
}

static void se1000_ipc_unmask_irq(struct irq_data *irqd)
{
}

static int se1000_ipc_set_type(struct irq_data *data, unsigned int flow_type)
{
	return 0;
}

static struct irq_chip se1000_ipc_irq_chip = {
	.name = "se1000-ipc",
	.irq_mask = se1000_ipc_mask_irq,
	.irq_unmask = se1000_ipc_unmask_irq,
	.irq_set_type = se1000_ipc_set_type,
	.flags = IRQCHIP_SKIP_SET_WAKE,
};

static int se1000_ipc_domain_map(struct irq_domain *d, unsigned int irq,
				irq_hw_number_t hw)
{
	struct se1000_mbox_chan *mchan  = d->host_data;
	irq_set_chip_and_handler(irq, &se1000_ipc_irq_chip, handle_level_irq);
	irq_set_chip_data(irq, mchan);
	irq_set_noprobe(irq);
	return 0;
}

static const struct irq_domain_ops se1000_ipc_irq_ops = {
	.map = se1000_ipc_domain_map,
	.xlate = irq_domain_xlate_onetwocell,
};

void *se1000_mailbox_find_channel_by_phandle_index(struct device_node *np, int index)
{
	struct device_node *node, *parent;
	struct platform_device *pdev;
	struct se1000_mbox *mbox;
	int ch;

	node = of_parse_phandle(np, "mailbox-irq-domain", index);
	if (!node) {
		pr_err("of_parse_phandle error\n");
		return NULL;
	}

	parent = node->parent;
	if (!parent)
		return NULL;

	pdev = of_find_device_by_node(parent);
	of_node_put(parent);
	if (!pdev) {
		pr_err("of_find_device_by_node error\n");
		return NULL;
	}

	mbox = platform_get_drvdata(pdev);
	if (!mbox) {
		pr_err("platform_get_drvdata error\n");
		return NULL;
	}

	for (ch = 0; ch < mbox->nr_mbox; ch++) {
		struct se1000_mbox_chan *mchan = &mbox->mboxes[ch];
		if (mchan->ch_np == node)
			return &mbox->chans[ch];
	}

	return NULL;
}
EXPORT_SYMBOL(se1000_mailbox_find_channel_by_phandle_index);

void *se1000_mailbox_find_channel_by_phandle(struct device_node *np)
{
	return se1000_mailbox_find_channel_by_phandle_index(np ,0);
}
EXPORT_SYMBOL(se1000_mailbox_find_channel_by_phandle);

int se1000_mailbox_raise_hwirq(void *chan, int hwirq)
{
	if (!chan)
		return -EINVAL;

	return se1000_mbox_send_data(chan, (void *)&hwirq);
}
EXPORT_SYMBOL(se1000_mailbox_raise_hwirq);

int se1000_mailbox_chan_check(void *chan)
{
	struct se1000_mbox_chan *mchan;

	if (!chan)
		return -EINVAL;

	mchan = ((struct mbox_chan *)chan)->con_priv;

	/* check if Mailbox tx fifo is full */
	if ((readl(mchan->ch_base + MBOX_REG_TXST) & MBOX_TXFIFO_FULL))
		return -EBUSY;

	return 0;
}
EXPORT_SYMBOL(se1000_mailbox_chan_check);


static int mailbox_show(struct seq_file *file, void *v)
{
	struct se1000_mbox_chan *mchan = file->private;

	seq_printf(file, "mchan:%px\n", mchan);

	return 0;
}

static int mialbox_open(struct inode *inode, struct file *file)
{
	return single_open(file, mailbox_show, PDE_DATA(inode));
}

static ssize_t mailbox_proc_write(struct file *fp, const char __user *ubuf,
			    size_t cnt, loff_t *ppos)
{
	struct se1000_mbox_chan *mchan = PDE_DATA(file_inode(fp));
	struct mbox_chan *chan;
	int err, hwirq;

	if (!mchan) {
		printk(KERN_ERR "Invalid mchan is NULL\n");
		return -EINVAL;
	}

	err = kstrtoint_from_user(ubuf, cnt, 0, &hwirq);
	if (err || hwirq < 0) {
		printk(KERN_ERR "Invalid argument\n");
		return -EINVAL;
	}

	chan = &mchan->mbox->chans[mchan->idx];
	if (se1000_mbox_send_data(chan, (void *)&hwirq))
		printk(KERN_ERR "Failed:%s\n", __func__);

	return cnt;
}

static const struct proc_ops procfs_mailbox_fops = {
	.proc_open	= mialbox_open,
	.proc_read = seq_read,
	.proc_release = single_release,
	.proc_write	= mailbox_proc_write,
};

/* Initialize mailbox channel data */
static int se1000_mbox_channel_probe(struct device_node *np, struct se1000_mbox *mbox, int ch)
{
	int err;
	u32 offset;
	struct mbox_chan *chan = &mbox->chans[ch];
	struct se1000_mbox_chan *mchan = &mbox->mboxes[ch];

	mchan->irq = irq_of_parse_and_map(np, 0);
	if (mchan->irq <= 0) {
		pr_err("irq get err %pOF.\n",  np);
		return -EINVAL;
	}

	err = of_property_read_u32(np, "reg", &offset);
	if (err) {
		pr_err("%s: No reg specified.\n", __func__);
		return err;
	}

	mchan->ch_base = mbox->base + offset;

	err = of_property_count_strings(np, "mbox-links");
	if (err < 0) {
		pr_err("mbox-links get err.\n");
		return -EINVAL;
	}

	if (2 != err) {
		pr_err("mbox-links count err.\n");
		return -EINVAL;
	}

	err = of_property_read_string_index(np, "mbox-links", 0, &mchan->src_name);
	if (err)
		return err;

	err = of_property_read_string_index(np, "mbox-links", 1, &mchan->dst_name);
	if (err < 0)
		return err;

	mchan->irq_domain = irq_domain_add_tree(np, &se1000_ipc_irq_ops, mchan);
	if (!mchan->irq_domain)
		return -ENOMEM;

	if (mbox->mboxes_dir)
		mchan->entry = proc_create_data(np->name, 0644, mbox->mboxes_dir, &procfs_mailbox_fops, mchan);

	mchan->idx = ch;
	mchan->ch_np = np;
	mchan->mbox = mbox;

	chan->con_priv = mchan;

	return 0;
}

static const struct of_device_id se1000_mbox_of_match[] = {
	{ .compatible = "siengine,se1000-mbox", },
	{},
};

MODULE_DEVICE_TABLE(of, se1000_mbox_of_match);


static int se1000_mbox_probe(struct platform_device *pdev)
{
	int ch;
	int err;
	struct resource *res;
	struct se1000_mbox *mbox;
	struct device *dev = &pdev->dev;
	struct device_node *child;
	struct device_node *np = dev->of_node;
	u32 val;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mbox->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(mbox->base))
		return PTR_ERR(mbox->base);

	mbox->nr_mbox = se1000_mbox_channel_count(np);
	if (!mbox->nr_mbox) {
		return -EBUSY;
	}

	mbox->mboxes = devm_kzalloc(dev, mbox->nr_mbox *
		(sizeof(struct se1000_mbox_chan) + sizeof(struct mbox_chan)),
		GFP_KERNEL);
	if (!mbox->mboxes) {
		dev_err(dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	mbox->chans = (void *)(mbox->mboxes + mbox->nr_mbox);

	if (!mbox->mboxes_dir)
		mbox->mboxes_dir =  proc_mkdir(/*dev_name(dev)*/"mailbox", NULL);

	ch = 0;
	for_each_child_of_node(np, child) {

		if (!of_device_compatible_match(child, se1000_channel_match))
			continue;

		err = se1000_mbox_channel_probe(child, mbox, ch);
		if (err) {
			dev_err(&pdev->dev, "failed to parse mailbox channel %d\n", ch);
			of_node_put(child);
			return err;
		}

		ch++;
	}

	mbox->dev = dev;
	mbox->controller.dev = dev;
	mbox->controller.chans = mbox->chans;
	mbox->controller.num_chans = mbox->nr_mbox;
	mbox->controller.ops = &se1000_mbox_ops;
	mbox->controller.txdone_poll = true;
	mbox->controller.txpoll_period = 5;
	mbox->controller.of_xlate = se1000_mbox_xlate;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0)
	err = mbox_controller_register(&mbox->controller);
	if (err) {
		dev_err(dev, "Failed to register mailbox %d\n", err);
		return err;
	}
#else
	err = devm_mbox_controller_register(dev, &mbox->controller);
	if (err) {
		dev_err(dev, "Failed to register mailbox %d\n", err);
		return err;
	}
#endif

	platform_set_drvdata(pdev, mbox);

	/* mailbox can not mask interrupt, so irq register at last */
	for (ch = 0; ch < mbox->nr_mbox; ch++) {
		struct se1000_mbox_chan *mchan = &mbox->mboxes[ch];
		irq_set_status_flags(mchan->irq, IRQ_NOAUTOEN);
		err = devm_request_irq(dev, mchan->irq, mbox_interrupt, IRQF_TRIGGER_HIGH,
				dev_name(dev), mchan);

		if (err) {
			dev_err(dev, "Failed to register a mailbox IRQ handler: %d\n",
				err);
			return -ENODEV;
		}

		val = (MBOX_RXFIFO_RST | MBOX_TXFIFO_RST);
		writel(val, mchan->ch_base + MBOX_REG_RSTN);
		enable_irq(mchan->irq);
	}

	dev_info(dev, "Mailbox enabled\n");
	return 0;
}

static int se1000_mbox_remove(struct platform_device *pdev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0)
	struct se1000_mbox *mbox = platform_get_drvdata(pdev);

	mbox_controller_unregister(&mbox->controller);
#endif

	return 0;
}

static struct platform_driver se1000_mbox_driver = {
	.probe  = se1000_mbox_probe,
	.remove	= se1000_mbox_remove,
	.driver = {
		.name = "se1000-mbox",
		.of_match_table = se1000_mbox_of_match,
	},
};

static int __init se1000_mbox_init(void)
{
	return platform_driver_register(&se1000_mbox_driver);
}
core_initcall(se1000_mbox_init);

static void __exit se1000_mbox_exit(void)
{
	platform_driver_unregister(&se1000_mbox_driver);
}
module_exit(se1000_mbox_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Siengine SE1000 Mailbox Controller");
MODULE_AUTHOR("Mingrui.Zhou <mingrui.zhou@siengine.com>");
