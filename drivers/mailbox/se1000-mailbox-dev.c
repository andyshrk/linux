/*
 * mailbox dev driver
 *
 * Copyright (c) 2018- Siengine Technologies CO., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt)		"se1000-mailbox-dev : " fmt

#include <linux/module.h>
#include <linux/mailbox_client.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/cdev.h>
#include <linux/mailbox_client.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-mapping.h>
#include <linux/kfifo.h>

#include "se1000-mailbox.h"
#include "se1000-mailbox-dev.h"

#define MAILBOX_MAX_CHANNELS	64

static struct class *mbox_class;
static struct cdev *mbox_cdev = NULL;
static dev_t devid = 0;
static dev_t mbox_major;
static LIST_HEAD(mbox_dev_head);

struct se1000_mbox_dev {
	struct device *dev;
	const char *name;
	const char *to_arch;
	struct mbox_client client;
	struct mbox_chan *chan;
	int chan_idx;
	int dev_idx;
	struct list_head list;
	wait_queue_head_t waitq;
	bool event_occur;
	struct fasync_struct *async_queue;
	struct kfifo fifo_rx;

	size_t mem_size;
	phys_addr_t phys_addr;
	void __iomem *virt_addr;

	struct file_operations *fops;
};

static int mbox_dev_open(struct inode *inode, struct file *filp)
{
	struct list_head * pos;
	int minor = MINOR(inode->i_rdev);

	if (minor >= MAILBOX_MAX_CHANNELS)
		return -EIO;

	list_for_each(pos, &mbox_dev_head) {
		struct se1000_mbox_dev *mbox_dev;

		mbox_dev = list_entry(pos, struct se1000_mbox_dev, list);
		if (minor == mbox_dev->chan_idx) {
			if (mbox_dev->fops && mbox_dev->fops->open) {
				int ret = mbox_dev->fops->open(inode, filp);

				if (ret)
					return ret;
			}

			mbox_dev->chan = mbox_request_channel(&mbox_dev->client, mbox_dev->dev_idx);
			if (IS_ERR(mbox_dev->chan))
				return PTR_ERR(mbox_dev->chan);

			filp->private_data = mbox_dev;

			return 0;
		}
	}

	return -EINVAL;
}

static int mbox_dev_release(struct inode *inode, struct file *filp)
{
	struct se1000_mbox_dev *mbox_dev = filp->private_data;

	if (mbox_dev->fops && mbox_dev->fops->release) {
		int ret = mbox_dev->fops->release(inode, filp);

		if (ret)
			return ret;
	}

	mbox_free_channel(mbox_dev->chan);
	mbox_dev->chan = NULL;

	return 0;
}

static ssize_t mbox_dev_write(struct file *filp,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	int ret;
	u32 msg;
	phys_addr_t p = *ppos;
	struct se1000_mbox_dev *mbox_dev = filp->private_data;

	if (NULL == mbox_dev)
		return -EIO;

	if (p != *ppos)
		return -EFBIG;

	if (count < sizeof(u32))
		return -EINVAL;

	if (kfifo_is_full(&mbox_dev->fifo_rx)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		return -EBUSY;
	}
	if (count > sizeof(u32))
		count = sizeof(u32);

	if (copy_from_user(&msg, userbuf, count))
		return -EFAULT;

	if (mbox_dev->fops && mbox_dev->fops->write) {
		int ret = mbox_dev->fops->write(filp, userbuf, count, ppos);

		if (ret)
			return ret;
	}

	ret = mbox_send_message(mbox_dev->chan, &msg);
	if (ret < 0) {
		pr_err("Err sending mbox %s message. cmd:%d, ret:%d\n",
			   mbox_dev->name, msg, ret);
		return -EINVAL;
	}

	return count;
}

static ssize_t mbox_dev_read(struct file *filp,
	char __user *userbuf, size_t count, loff_t *ppos)
{
	int ret;
	unsigned int copied;
	struct se1000_mbox_dev *mbox_dev = filp->private_data;

	if (NULL == mbox_dev)
		return -EIO;

	if (count < sizeof(u32))
		return -EINVAL;

	if (kfifo_is_empty(&mbox_dev->fifo_rx)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else
			wait_event_interruptible(mbox_dev->waitq, mbox_dev->event_occur);
	}

	if (count > sizeof(u32))
		count = sizeof(u32);


	if (mbox_dev->fops && mbox_dev->fops->read) {
		int ret = mbox_dev->fops->read(filp, userbuf, count, ppos);

		if (ret)
			return ret;
	}

	ret = kfifo_to_user(&mbox_dev->fifo_rx, userbuf, count, &copied);
	if (ret == 0 && copied == sizeof(u32))
		ret = copied;

	if (kfifo_is_empty(&mbox_dev->fifo_rx))
		mbox_dev->event_occur = false;

	return ret;
}

static unsigned int mbox_dev_poll(struct file *filp,
	 struct poll_table_struct *wait)
{
	struct se1000_mbox_dev *mbox_dev = filp->private_data;

	if (NULL == mbox_dev)
		return -EIO;

	poll_wait(filp, &mbox_dev->waitq, wait);


	if (mbox_dev->fops && mbox_dev->fops->poll) {
		int ret = mbox_dev->fops->poll(filp, wait);

		if (ret)
			return ret;
	}

	if (mbox_dev->event_occur)
		return POLLIN | POLLRDNORM;

	return 0;
}

static int mbox_dev_fasync(int fd, struct file *filp, int on)
{
	struct se1000_mbox_dev *mbox_dev = filp->private_data;

	if (NULL == mbox_dev)
		return -EIO;

	if (mbox_dev->fops && mbox_dev->fops->fasync) {
		int ret = mbox_dev->fops->fasync(fd, filp, on);

		if (ret)
			return ret;
	}

	return fasync_helper(fd, filp, on, &mbox_dev->async_queue);
}

int mbox_dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;

	struct se1000_mbox_dev *mbox_dev = filp->private_data;

	if (NULL == mbox_dev)
		return -EIO;

	if (mbox_dev->fops && mbox_dev->fops->mmap) {
		int ret = mbox_dev->fops->mmap(filp, vma);

		if (ret)
			return ret;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return io_remap_pfn_range(vma, vma->vm_start,
					mbox_dev->phys_addr >> PAGE_SHIFT,
					size, vma->vm_page_prot);
}

long mbox_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	int ret = 0;
	struct se1000_mbox_dev *mbox_dev = filp->private_data;

	/* check magic number */
	if (_IOC_TYPE(cmd) != SE1000_MBOX_IOCTL_MAGIC)
		return -EINVAL;

	if (mbox_dev->fops) {
		long (*mbox_ioctl)(struct file *, unsigned int, unsigned long) = NULL;

		if (mbox_dev->fops->unlocked_ioctl)
			mbox_ioctl = mbox_dev->fops->unlocked_ioctl;
#ifdef CONFIG_COMPAT
		else if(mbox_dev->fops->compat_ioctl)
			mbox_ioctl = mbox_dev->fops->compat_ioctl;
#endif
		if (mbox_ioctl) {
			ret = mbox_ioctl(filp, cmd, args);

			if (ret)
				return ret;
		}
	}

	switch(cmd) {
		case SE1000_MBOX_IOCTL_PHY_ADDR:
			if (kfifo_is_full(&mbox_dev->fifo_rx)) {
				if (filp->f_flags & O_NONBLOCK)
					return -EAGAIN;
				return -EBUSY;
			}

			ret = mbox_send_message(mbox_dev->chan, &mbox_dev->phys_addr);
			if (ret < 0) {
				pr_err("Err sending mbox %s message. cmd:%d, ret:%d\n",
					   mbox_dev->name, (uint32_t)mbox_dev->phys_addr, ret);
				return -EINVAL;
			}
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static const struct file_operations mbox_dev_ops = {
	.write		= mbox_dev_write,
	.read		= mbox_dev_read,
	.fasync		= mbox_dev_fasync,
	.poll		= mbox_dev_poll,
	.open		= mbox_dev_open,
	.mmap		= mbox_dev_mmap,
	.release	= mbox_dev_release,
	.llseek		= no_llseek,
	.unlocked_ioctl = mbox_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= mbox_dev_ioctl,
#endif
};

void se1000_mbox_dev_register_filter(struct mbox_client *client, struct file_operations *filter_ops)
{
	struct se1000_mbox_dev *mbox_dev;

	mbox_dev = container_of(client, struct se1000_mbox_dev, client);
	if (mbox_dev->fops)
		pr_warn("%pOF has been registered filter\n", mbox_dev->dev->of_node);

	mbox_dev->fops = filter_ops;
}
EXPORT_SYMBOL(se1000_mbox_dev_register_filter);

static void mbox_dev_receive_message(struct mbox_client *client, void *message)
{
	u32 msg = (uintptr_t)message;
	struct se1000_mbox_dev *mbox_dev;

	mbox_dev = container_of(client, struct se1000_mbox_dev, client);
	if (!kfifo_is_full(&mbox_dev->fifo_rx))
		kfifo_in(&mbox_dev->fifo_rx, &msg, sizeof(msg));

	mbox_dev->event_occur = true;
	wake_up_interruptible(&mbox_dev->waitq);
	kill_fasync(&mbox_dev->async_queue, SIGIO, POLL_IN);
}

static void mbox_dev_prepare_message(struct mbox_client *client, void *message)
{
}

static void mbox_dev_message_sent(struct mbox_client *client,
				void *message, int r)
{
}

static void mbox_dev_client_init(struct se1000_mbox_dev *mbox_dev)
{
	struct mbox_client *client = &mbox_dev->client;

	client->dev				= mbox_dev->dev;
	client->rx_callback		= mbox_dev_receive_message;
	client->tx_prepare		= mbox_dev_prepare_message;
	client->tx_done			= mbox_dev_message_sent;
	client->tx_block		= false;
	client->knows_txdone	= false;
	client->tx_tout			= 500;
}

static int mbox_get_channel_idx(struct device_node *np, int index)
{
	struct of_phandle_args spec;
	struct platform_device *pdev;
	struct se1000_mbox *mbox;
	struct device_node *node;
	int ch;
	phandle handle;

	if (!of_get_property(np, "mboxes", NULL))
		return -ENODEV;

	if (of_parse_phandle_with_args(np, "mboxes",
				       "#mbox-cells", index, &spec)) {
		pr_err("%s: can't parse \"mboxes\" property\n", __func__);
	}

	pdev = of_find_device_by_node(spec.np);
	of_node_put(spec.np);
	if (!pdev)
		return -EPROBE_DEFER;

	mbox = platform_get_drvdata(pdev);
	if (!mbox)
		return -EPROBE_DEFER;

	handle = spec.args[0];
	node = of_find_node_by_phandle(handle);
	if (!node) {
		pr_err("%s: could not find node phandle 0x%x\n",
			__func__, handle);
		return -ENODEV;
	}

	for (ch = 0; ch < mbox->nr_mbox; ch++) {
		struct se1000_mbox_chan *mchan = &mbox->mboxes[ch];

		if (mchan->ch_np == node)
			return ch;
	}

	return -EINVAL;
}

static int mbox_add_devnode(const char *name, dev_t minor)
{
	int ret;
	struct device *devp;

	if (!mbox_cdev) {
		mbox_cdev = cdev_alloc();
		if(NULL == mbox_cdev){
			return -ENOMEM;
		}

		cdev_init(mbox_cdev, &mbox_dev_ops);
	}

	if (!devid) {
		ret = alloc_chrdev_region(&devid, 0, MAILBOX_MAX_CHANNELS, "siengine-mailbox");
		if (ret)
			return ret;

		ret = cdev_add(mbox_cdev, devid, MAILBOX_MAX_CHANNELS);
		mbox_major = MAJOR(devid);
	}

	if (!mbox_class) {
		mbox_class = class_create(THIS_MODULE, "siengine-mailbox");
		if (IS_ERR(mbox_class)) {
			return PTR_ERR(mbox_class);
		}
	}

	devp = device_create(mbox_class, NULL, MKDEV(mbox_major, minor), NULL, name);
	if(IS_ERR(devp)){
		return PTR_ERR(devp);
	}

	return 0;
}

static int mbox_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node;
	struct device_node *np = dev->of_node;
	struct resource r;
	int ret;
	size_t mem_size;
	phys_addr_t phys_addr;
	void __iomem *virt_addr;
	int count;
	const char *mbox_name;
	const char *arch;
	int idx;
	struct property *prop;
	size_t alloc_size;

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
	virt_addr = devm_ioremap_resource(&pdev->dev, &r);
	if (!virt_addr) {
		dev_err(&pdev->dev, "unable to map memory region: %pa+%zx\n",
			(void *)(uintptr_t)phys_addr, mem_size);
		return -ENOMEM;
	}

	ret = of_property_count_strings(np, "mbox-names");
	if (ret < 0)
		return -EINVAL;
	count = ret;

	ret = of_property_read_string(np, "link-arch", &arch);
	if (ret < 0)
		return ret;

	alloc_size = sizeof(struct se1000_mbox_dev);
	alloc_size += VFIFO_MSG_LEN; // for kfifo

	idx = 0;
	of_property_for_each_string(np, "mbox-names", prop, mbox_name) {
		struct se1000_mbox_dev *mbox_dev;

		mbox_dev = devm_kzalloc(dev, alloc_size, GFP_KERNEL);
		if (!mbox_dev) {
			dev_err(dev, "failed to allocate memory\n");
			return -ENOMEM;
		}

		mbox_dev->name = mbox_name;
		mbox_dev->to_arch = arch;
		mbox_dev->dev = &pdev->dev;

		mbox_dev->phys_addr = phys_addr;
		mbox_dev->mem_size = mem_size;
		mbox_dev->virt_addr = virt_addr;

		mbox_dev_client_init(mbox_dev);

		mbox_dev->dev_idx = idx;
		ret = mbox_get_channel_idx(np, idx++);
		if (ret < 0)
			return -EINVAL;

		mbox_dev->chan_idx = ret;

		init_waitqueue_head(&mbox_dev->waitq);

		kfifo_init(&mbox_dev->fifo_rx, (void *)(mbox_dev + 1), VFIFO_MSG_LEN);

		mbox_add_devnode(mbox_name, mbox_dev->chan_idx);

		list_add(&mbox_dev->list, &mbox_dev_head);

		dev_info(dev, "%s init success\n", mbox_name);
	}

	return 0;
}

static int mbox_dev_remove(struct platform_device *pdev)
{
	struct list_head * pos;
	list_for_each(pos, &mbox_dev_head) {
		struct se1000_mbox_dev *mbox_dev;

		mbox_dev = container_of(pos, struct se1000_mbox_dev, list);
		if (mbox_dev->chan) {
			pr_err("Err remove mbox %s, has been opened\n", mbox_dev->name);
			return -EFAULT;
		}

		device_destroy(mbox_class, MKDEV(mbox_major, mbox_dev->chan_idx));
	}

	class_destroy(mbox_class);

	unregister_chrdev_region(devid, MAILBOX_MAX_CHANNELS);

	cdev_del(mbox_cdev);

	return 0;
}

static const struct of_device_id mbox_dev_match[] = {
	{ .compatible = "siengine,se1000-mbox-dev" },
	{},
};
MODULE_DEVICE_TABLE(of, mbox_dev_match);

static struct platform_driver mbox_dev_driver = {
	.driver = {
		.name = "mailbox_dev",
		.of_match_table = mbox_dev_match,
	},
	.probe  = mbox_dev_probe,
	.remove = mbox_dev_remove,
};
module_platform_driver(mbox_dev_driver);

MODULE_DESCRIPTION("Siengine mailbox interface");
MODULE_AUTHOR("Mingrui Zhou <mingrui.zhou@siengine.com");
MODULE_LICENSE("GPL v2");
