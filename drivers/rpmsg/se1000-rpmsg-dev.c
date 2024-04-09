/*
 * rpmsg dev driver
 *
 * Copyright (c) 2018- Siengine Technologies CO., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt)		"se1000-rpmsg-dev : " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/kfifo.h>
#include <linux/bitops.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <uapi/linux/rpmsg.h>
#include <linux/rpmsg.h>
#include "rpmsg_internal.h"

#include "se1000-rpmsg-dev.h"
#include <dt-bindings/interrupt-controller/se1000-mailbox-irq.h>
#define CREATE_TRACE_POINTS
#include "se1000-rpmsg-trace.h"

#define RPMSG_MAX_CHANNELS	64

#define RPMSG_LOCATE_DATA(p) ((unsigned char *)(p) + sizeof(struct se1000_rpmsg_info_head))

#define ept_to_eptdev(i_ept)	container_of(i_ept, struct se1000_rpmsg_endpoint, ept)
#define rpdev_to_rpmsg_dev(i_rpdev) container_of(i_rpdev, struct se1000_rpmsg_dev, rpdev)
#define ept_to_se_ept(i_ept)	container_of(i_ept, struct se1000_rpmsg_endpoint, ept)

#define MAGIC_DATA	0x55;

static struct shm_ring* se1000_init_fifo(void *buffer)
{
	struct shm_ring* fifo = NULL;

	if(!buffer)
		return NULL;

	fifo = (struct shm_ring*)buffer;
	if(!fifo)
		return NULL;

	fifo->in = 0;
	fifo->out = 0;

	return fifo;
}

static inline void* se1000_get_inptr(struct shm_ring *fifo)
{
	int in = fifo->in;

	return (((in + 1) % MAX_FIFO_SIZE) != fifo->out) ? (void *)&fifo->data[in][0] : NULL;
}

static inline void se1000_move_inptr(struct shm_ring *fifo)
{
	fifo->in = (fifo->in + 1) % MAX_FIFO_SIZE;
}

static inline void* se1000_get_outptr(struct shm_ring *fifo)
{
	int out = fifo->out;

	return (out != fifo->in) ? (void *)&fifo->data[out][0] : NULL;
}

static inline void se1000_move_outptr(struct shm_ring *fifo)
{
	fifo->out = (fifo->out + 1) % MAX_FIFO_SIZE;
}

static void se1000_rpmsg_device_release(struct device *dev)
{
	//TODO : resource release
}

static void se1000_rpmsg_destroy_ept(struct rpmsg_endpoint *ept)
{
	struct se1000_rpmsg_endpoint *se_ept = ept_to_eptdev(ept);

	if (!se_ept)
		return;

	spin_lock(&se_ept->rpmsg_dev->ept_lock);

	se_ept->ept.cb = NULL;

	if(!list_empty(&se_ept->list)) {
		list_del(&se_ept->list);
	}
	spin_unlock(&se_ept->rpmsg_dev->ept_lock);

	kfree(se_ept);

}

static int se1000_rpmsg_send(struct rpmsg_endpoint *ept, void *data, int len)
{
	int ret;
	struct shm_ring *pFifo;
	struct se1000_rpmsg_info_head *rp_hdr;
	int rh_len = sizeof(struct se1000_rpmsg_info_head);
	struct se1000_rpmsg_dev *rpmsg_dev = ept_to_se_ept(ept)->rpmsg_dev;

	if (se1000_mailbox_chan_check(rpmsg_dev->chan)) {
		return -EAGAIN;
	}

	if(len > CACHE_LINE - rh_len)
		len = CACHE_LINE - rh_len;

	pFifo = rpmsg_dev->tx_fifo;
	spin_lock(&rpmsg_dev->rpmsg_dev_lock);
	rp_hdr = se1000_get_inptr(pFifo);
	if(!rp_hdr) {
		spin_unlock(&rpmsg_dev->rpmsg_dev_lock);
		return -ENOMEM;
	}

	rp_hdr->src = ept_to_se_ept(ept)->chinfo.src;
	rp_hdr->dst = ept_to_se_ept(ept)->chinfo.dst;
	rp_hdr->length = len;
	rp_hdr->reverse = 0;
	rp_hdr->flags = 0;
	memcpy(RPMSG_LOCATE_DATA(rp_hdr), data, len);
	se1000_move_inptr(pFifo);
	spin_unlock(&rpmsg_dev->rpmsg_dev_lock);

	ret = se1000_mailbox_raise_hwirq(rpmsg_dev->chan, SE1000_MBOX_IRQ_RPMSG);
	if(ret < 0) {
		pr_err("Err sending rpmsg %s message.ret:%d\n",
			rpmsg_dev->name, ret);
		return -EINVAL;
	}
	return ret;
}

static const struct rpmsg_endpoint_ops se1000_rpmsg_endpoint_ops = {
	.destroy_ept = se1000_rpmsg_destroy_ept,
	.send = se1000_rpmsg_send,
};

//have problem:multiple coverage
static struct rpmsg_endpoint *se1000_rpmsg_create_ept(struct rpmsg_device *rpdev,
				rpmsg_rx_cb_t cb, void *priv, struct rpmsg_channel_info chinfo)
{
	struct rpmsg_endpoint *ept;
	struct se1000_rpmsg_endpoint *se_ept;
	struct se1000_rpmsg_dev *rpmsg_dev = rpdev_to_rpmsg_dev(rpdev);

	se_ept = kzalloc(sizeof(*se_ept), GFP_KERNEL);
	if(!se_ept)
		return NULL;

	se_ept->rpmsg_dev = rpmsg_dev;
	se_ept->chinfo = chinfo;

	spin_lock(&rpmsg_dev->ept_lock);
	ept = &se_ept->ept;

	ept->rpdev = rpdev;
	ept->addr = chinfo.src;
	ept->cb = cb;
	ept->priv = priv;
	ept->ops = &se1000_rpmsg_endpoint_ops;

	list_add(&se_ept->list, &rpmsg_dev->ep_list_head);
	spin_unlock(&rpmsg_dev->ept_lock);

	return ept;
}

static const struct rpmsg_device_ops se1000_rpmsg_device_ops = {
	.create_ept = se1000_rpmsg_create_ept,
};

static int se1000_rpmsg_register_device(struct se1000_rpmsg_dev *rpmsg_dev)
{
	struct rpmsg_device *rpdev;

	rpdev = &rpmsg_dev->rpdev;
	rpdev->ops = &se1000_rpmsg_device_ops;
	rpdev->dev.parent = rpmsg_dev->dev;
	rpdev->dev.release = se1000_rpmsg_device_release;

	strscpy(rpdev->id.name, rpmsg_dev->name, RPMSG_NAME_SIZE);
	dev_set_name(&rpdev->dev, rpmsg_dev->name);

	return rpmsg_chrdev_register_device(rpdev);
}

/*find the ept from ept_list*/
static struct rpmsg_endpoint* se1000_get_ept(struct se1000_rpmsg_dev *rpmsg_dev,u32 dst)
{
	struct list_head *pos;
	struct rpmsg_endpoint *ept = NULL;

	list_for_each(pos, &rpmsg_dev->ep_list_head) {
		struct se1000_rpmsg_endpoint *rpmsg_ept;

		rpmsg_ept = list_entry(pos, struct se1000_rpmsg_endpoint, list);
		if(dst == rpmsg_ept->chinfo.src) {
			ept = &rpmsg_ept->ept;
			break;
		}
	}

	return ept;
}

static irqreturn_t se1000_rpmsg_isr(int irq, void *dev_id)
{
	struct se1000_rpmsg_dev *dev = dev_id;

	// schedule_work(&dev->ipc_work);
	queue_work(system_highpri_wq, &dev->ipc_work);

	return IRQ_HANDLED;
}

static void se1000_ept_receive_work(struct work_struct *work)
{
	struct rpmsg_endpoint *ept;
	struct se1000_rpmsg_info_head *rp_hdr;
	struct se1000_rpmsg_dev *rpmsg_dev = container_of(work, struct se1000_rpmsg_dev, ipc_work);

	rp_hdr = se1000_get_outptr(rpmsg_dev->rx_fifo);
	while (rp_hdr) {
		ept = se1000_get_ept(rpmsg_dev,rp_hdr->dst);

		if (ept) {
			if (ept->addr == RPMSG_ADDR_ANY) {
				ept->addr = rp_hdr->src;
			}
			if(ept->cb) {
				trace_se1000_ipc_rpmsg_ept_cb('S', rpmsg_dev->name, ept->addr, rp_hdr->dst ,ept->cb);

				ept->cb(ept->rpdev, RPMSG_LOCATE_DATA(rp_hdr),
					rp_hdr->length, ept->priv, rp_hdr->src);

				trace_se1000_ipc_rpmsg_ept_cb('E', rpmsg_dev->name, ept->addr, rp_hdr->dst ,ept->cb);
			}
		}

		se1000_move_outptr(rpmsg_dev->rx_fifo);
		rp_hdr = se1000_get_outptr(rpmsg_dev->rx_fifo);
	}

}

#if 0
static irqreturn_t se1000_rpmsg_interrupt_thread(int irq, void *dev_id)
{
	struct rpmsg_endpoint *ept;
	struct se1000_rpmsg_info_head *rp_hdr;
	struct se1000_rpmsg_dev *rpmsg_dev = dev_id;

	rp_hdr = se1000_get_outptr(rpmsg_dev->rx_fifo);
	while (rp_hdr) {
		ept = se1000_get_ept(rpmsg_dev,rp_hdr->dst);

		if (ept) {
			if (ept->addr == RPMSG_ADDR_ANY) {
				ept->addr = rp_hdr->src;
			}
			if(ept->cb) {
				trace_se1000_ipc_rpmsg_ept_cb('S', rpmsg_dev->name, ept->addr, rp_hdr->dst ,ept->cb);

				ept->cb(ept->rpdev, RPMSG_LOCATE_DATA(rp_hdr),
					rp_hdr->length, ept->priv, rp_hdr->src);

				trace_se1000_ipc_rpmsg_ept_cb('E', rpmsg_dev->name, ept->addr, rp_hdr->dst ,ept->cb);
			}
		}

		se1000_move_outptr(rpmsg_dev->rx_fifo);
		rp_hdr = se1000_get_outptr(rpmsg_dev->rx_fifo);
	}

	return IRQ_HANDLED;
}
#endif

/*Init the rpmsg_dev by device_tree and add device_node*/
static int rpmsg_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node;
	struct device_node *np = dev->of_node;
	struct resource r;
	int ret;
	size_t mem_size;
	phys_addr_t phys_addr;
	void __iomem *virt_addr;
	const char *rpmsg_name;
	const char *link_port;
	size_t alloc_size;
	const char *port = "PortA";
	struct se1000_rpmsg_dev *rpmsg_dev;

	alloc_size = sizeof(struct se1000_rpmsg_dev);
	rpmsg_dev = devm_kzalloc(dev, alloc_size, GFP_KERNEL);
	if (!rpmsg_dev) {
		dev_err(dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	rpmsg_dev->irq = platform_get_irq(pdev, 0);
    if (rpmsg_dev->irq < 0)	{
		dev_err(&pdev->dev, "could not get IRQ\n");
		return rpmsg_dev->irq;
	}

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

	mem_size = resource_size(&r);
	phys_addr = r.start;
	virt_addr = devm_ioremap_resource_wc(&pdev->dev, &r);
	if (!virt_addr) {
		dev_err(&pdev->dev, "unable to map memory region: %pa+%zx\n",
			(void *)(uintptr_t)phys_addr, mem_size);
		return -ENOMEM;
	}

	rpmsg_dev->phys_addr = phys_addr;
	rpmsg_dev->mem_size = mem_size;
	rpmsg_dev->virt_addr = virt_addr;

	//init lock
	spin_lock_init(&rpmsg_dev->rpmsg_dev_lock);

	spin_lock_init(&rpmsg_dev->ept_lock);
	//link-port
	ret = of_property_read_string(np, "link-arch", &link_port);
	if (ret < 0)
		return ret;

	rpmsg_dev->link_port = link_port;

	//name
	ret = of_property_read_string(np, "rpmsg-names", &rpmsg_name);
	if (ret < 0)
		return ret;

	rpmsg_dev->name = rpmsg_name;
	rpmsg_dev->dev = &pdev->dev;

	//parse phandle to get chn->irq and chan->reg
	rpmsg_dev->chan = se1000_mailbox_find_channel_by_phandle(np);
	if (IS_ERR(rpmsg_dev->chan)) {
		dev_err(rpmsg_dev->dev, "Failed to get rpmsg_chan!\n");
		return -EINVAL;
	}

	//init work
	INIT_WORK(&rpmsg_dev->ipc_work, se1000_ept_receive_work);

	//init ep_list_head
	INIT_LIST_HEAD(&rpmsg_dev->ep_list_head);

	//register rpmsg device
	ret = se1000_rpmsg_register_device(rpmsg_dev);
	if (ret) {
		dev_err(rpmsg_dev->dev, "failed to register chrdev for rpmsg_dev\n");
		put_device(rpmsg_dev->dev);
		return ret;
	}

	//init tx_fifo and rx_fifo
	if(!strcmp(rpmsg_dev->link_port, port)) {
		rpmsg_dev->tx_fifo = se1000_init_fifo(virt_addr);
		rpmsg_dev->rx_fifo = se1000_init_fifo((u8 *)virt_addr + mem_size/2);
	} else {
		rpmsg_dev->rx_fifo = se1000_init_fifo(virt_addr);
		rpmsg_dev->tx_fifo = se1000_init_fifo((u8 *)virt_addr + mem_size/2);
	}

	irq_set_status_flags(rpmsg_dev->irq, IRQ_NOAUTOEN);
	//request irq
//	ret = devm_request_threaded_irq(rpmsg_dev->dev, rpmsg_dev->irq, se1000_rpmsg_isr, se1000_rpmsg_interrupt_thread,
//				IRQF_TRIGGER_HIGH, dev_name(rpmsg_dev->dev), rpmsg_dev);
	ret = devm_request_irq(rpmsg_dev->dev, rpmsg_dev->irq, se1000_rpmsg_isr, IRQF_TRIGGER_HIGH,
				dev_name(rpmsg_dev->dev), rpmsg_dev);
	if (ret) {
		dev_err(rpmsg_dev->dev, "Failed to register a ipc IRQ:%d handler: %d\n",rpmsg_dev->irq,ret);
		return ret;
	}
	/* do not enable channel interrupt at start */
	enable_irq(rpmsg_dev->irq);

	rpmsg_dev->tx_fifo->magic = MAGIC_DATA;

	dev_info(dev, "%s init success\n", rpmsg_name);

	return 0;
}

static int rpmsg_dev_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id rpmsg_dev_match[] = {
	{ .compatible = "siengine,se1000-rpmsg-dev" },
	{},
};
MODULE_DEVICE_TABLE(of, rpmsg_dev_match);

static struct platform_driver rpmsg_dev_driver = {
	.driver = {
		.name = "rpmsg_dev",
		.of_match_table = rpmsg_dev_match,
	},
	.probe  = rpmsg_dev_probe,
	.remove = rpmsg_dev_remove,
};

#if 0
module_platform_driver(rpmsg_dev_driver);
#else
static int __init se1000_rpmsg_dev_init(void)
{
	return platform_driver_register(&rpmsg_dev_driver);
}
postcore_initcall(se1000_rpmsg_dev_init);

static void __exit se1000_rpmsg_dev_exit(void)
{
	platform_driver_unregister(&rpmsg_dev_driver);
}
module_exit(se1000_rpmsg_dev_exit);
#endif

MODULE_DESCRIPTION("Siengine rpmsg interface");
MODULE_AUTHOR("Siengine @siengine.com");
MODULE_LICENSE("GPL v2");
