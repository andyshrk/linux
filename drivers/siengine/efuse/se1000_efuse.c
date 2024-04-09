/*
 * Copyright (C) 2023 Siengine Technology Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include "se1000_efuse.h"

struct efuse_device {
	struct miscdevice efuse_misc_device;
	unsigned char *efuse_base_virt;
	phys_addr_t efuse_base_phy;
	size_t mem_size;
	size_t lmem_size;
};

static int efuse_open(struct inode *finode, struct file *fd)
{
	struct efuse_device *efuse_dev;

	efuse_dev = container_of(fd->private_data, struct efuse_device, efuse_misc_device);
	fd->f_pos = 0;

	return 0;
}

static loff_t efuse_lseek(struct file *file, loff_t offset, int orig)
{
	loff_t ret;
	struct efuse_device *efuse_dev;

	if (offset & 0x3) {
		pr_err("%s offset is not B-byte alignment\n", __func__);
		return -EFAULT;
	}
	efuse_dev = container_of(file->private_data, struct efuse_device, efuse_misc_device);

	inode_lock(file_inode(file));

	switch (orig) {
	case SEEK_CUR:
		if (file->f_pos + offset > efuse_dev->lmem_size || file->f_pos + offset < 0) {
			ret = -EOVERFLOW;
			break;
		}
		file->f_pos += offset;
		ret = file->f_pos;
		break;
	case SEEK_SET:
		if (offset > efuse_dev->lmem_size || offset < 0) {
			ret = -EOVERFLOW;
			break;
		}
		file->f_pos = offset;
		ret = file->f_pos;
		break;
	case SEEK_END:
		if (offset > 0 || offset < -efuse_dev->lmem_size) {
			ret = -EOVERFLOW;
			break;
		}
		file->f_pos = efuse_dev->lmem_size + offset;
		ret = file->f_pos;
		break;
	default:
		ret = -EINVAL;
	}
	inode_unlock(file_inode(file));
	return ret;
}

/* efuse power down mode */
static void efuse_pd_enable(struct efuse_device *efuse_dev)
{
#if 0
	unsigned int data = 0;
	data = *(unsigned int *)(efuse_dev->efuse_base_virt + EFUSE_MODE_CFG_OFFSET);
	data |= EFUSE_MODE_PD;
	*(unsigned int *)(efuse_dev->efuse_base_virt + EFUSE_MODE_CFG_OFFSET) = data;
#endif
}

static void efuse_pd_disable(struct efuse_device *efuse_dev)
{
#if 0
	unsigned int data = 0;
	data = *(unsigned int *)(efuse_dev->efuse_base_virt + EFUSE_MODE_CFG_OFFSET);
	data &= ~EFUSE_MODE_PD;
	*(unsigned int *)(efuse_dev->efuse_base_virt + EFUSE_MODE_CFG_OFFSET) = data;
	/* 1us delay ensure efuse power-up work*/
	udelay(1);
#endif
}


static ssize_t efuse_read(struct file *fd, char __user *buffer, size_t size, loff_t *offset)
{
	unsigned long of = 0;
	struct efuse_device *efuse_dev;

	efuse_dev = container_of(fd->private_data, struct efuse_device, efuse_misc_device);

	of = *offset;

	if (of & 0x3) {
		pr_err("%s of is not 4-Byte alignment\n", __func__);
		return -EFAULT;
	}
	if (size & 0x3) {
		pr_err("%s size is not 4-Byte alignment\n", __func__);
		return -EFAULT;
	}

	if (of + size > efuse_dev->lmem_size) {
		pr_err("%s offset is over range\n", __func__);
		return -EFAULT;
	}

	if (size > (efuse_dev->lmem_size - of))
		size = efuse_dev->lmem_size - of;
	/* efuse power on */
	efuse_pd_disable(efuse_dev);
	if (copy_to_user(buffer, (void *)(efuse_dev->efuse_base_virt + of), size)) {
		pr_err("%s read memory failed.\n", __func__);
		return -EFAULT;
	}
	/* efuse power down */
	efuse_pd_enable(efuse_dev);
	*offset += size;
	return size;
}

static int efuse_close(struct inode *inode, struct file *fd)
{
	return 0;
}

static const struct file_operations efuse_fops = {
	.owner = THIS_MODULE,
	.open = efuse_open,
	.release = efuse_close,
	.read = efuse_read,
	.llseek = efuse_lseek,
};

static struct miscdevice efuse_ops_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "efuse",
	.fops = &efuse_fops,
	.this_device = NULL,
};

static const struct of_device_id efuse_match[] = {
	{.compatible = "siengine,efuse",},
	{},
};

MODULE_DEVICE_TABLE(of, efuse_match);

static int efuse_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct efuse_device *efuse_dev;
	struct device_node *np = dev->of_node;
	struct device_node *node;
	struct resource res;
	int ret = -1;

	efuse_dev = devm_kzalloc(dev, sizeof(struct efuse_device), GFP_KERNEL);
	if (!efuse_dev)
		return -ENOMEM;

	memset(efuse_dev, 0, sizeof(struct efuse_device));

	node = of_parse_phandle(np, "memory-region", 0);
	if (!node) {
		pr_err("%s no memory-region specified\n", __func__);
		return -EINVAL;
	}

	ret = of_address_to_resource(node, 0, &res);
	if (ret) {
		pr_err("%s failed to get reserved region address\n", __func__);
		of_node_put(node);
		return ret;
	}

	of_node_put(node);
	efuse_dev->mem_size = resource_size(&res);
	efuse_dev->efuse_base_phy = res.start;

	efuse_dev->efuse_base_virt = (unsigned char *)ioremap(efuse_dev->efuse_base_phy, efuse_dev->mem_size);
	efuse_dev->lmem_size = EFUSE_LMEM_SIZE;

	if (!(efuse_dev->efuse_base_virt)) {
		pr_err("%s efuse ioremap failed!\n", __func__);
		return -ENOMEM;
	}

	efuse_dev->efuse_misc_device = efuse_ops_misc_device;
	ret = misc_register(&(efuse_dev->efuse_misc_device));
	if (ret < 0) {
		pr_err("%s efuse memory miscdrv register failed.\n", __func__);
		devm_kfree(dev, efuse_dev);
		return -EFAULT;
	}

	platform_set_drvdata(pdev, efuse_dev);

	return 0;
}

static int efuse_remove(struct platform_device *pdev)
{
	struct efuse_device *efuse_dev = platform_get_drvdata(pdev);

	misc_deregister(&(efuse_dev->efuse_misc_device));

	return 0;
}

static struct platform_driver efuse_drv = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "efuse",
		.of_match_table = efuse_match,
	},

	.probe = efuse_probe,
	.remove = efuse_remove,
};

static int __init efuse_init(void)
{
	return platform_driver_register(&efuse_drv);
}

static void __exit efuse_exit(void)
{
	platform_driver_unregister(&efuse_drv);
}

module_init(efuse_init);
module_exit(efuse_exit);

MODULE_AUTHOR("jingtao.li@siengine.com");
MODULE_DESCRIPTION("Driver for efuse access");
MODULE_LICENSE("GPL");
