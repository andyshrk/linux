/*
 * Copyright (C) 2022 Siengine Technology Co., Ltd.
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


struct drm_shm_device {
	struct miscdevice shm_misc_device;
	unsigned char *shmem_base_virt;
	phys_addr_t shmem_base_phy;
	size_t mem_size;
};

static int drm_shm_open(struct inode *finode, struct file *fd)
{
	struct drm_shm_device *shm_dev;

	shm_dev = container_of(fd->private_data, struct drm_shm_device, shm_misc_device);
	fd->f_pos = 0;

	return 0;
}

static loff_t drm_shm_lseek(struct file *file, loff_t offset, int orig)
{
	loff_t ret;
	struct drm_shm_device *shm_dev;

	shm_dev = container_of(file->private_data, struct drm_shm_device, shm_misc_device);

	inode_lock(file_inode(file));
	switch (orig) {
	case SEEK_CUR:
		if (file->f_pos + offset > shm_dev->mem_size || file->f_pos + offset < 0) {
			ret = -EOVERFLOW;
			break;
		}
		file->f_pos += offset;
		ret = file->f_pos;
		break;
	case SEEK_SET:
		if (offset > shm_dev->mem_size || offset < 0) {
			ret = -EOVERFLOW;
			break;
		}
		file->f_pos = offset;
		ret = file->f_pos;
		break;
	case SEEK_END:
		if (offset > 0 || offset < -shm_dev->mem_size) {
			ret = -EOVERFLOW;
			break;
		}
		file->f_pos = shm_dev->mem_size + offset;
		ret = file->f_pos;
		break;
	default:
		ret = -EINVAL;
	}
	inode_unlock(file_inode(file));
	return ret;
}

static ssize_t drm_shm_read(struct file *fd, char __user *buffer, size_t size, loff_t *offset)
{
	unsigned long of = 0;
	struct drm_shm_device *shm_dev;

	shm_dev = container_of(fd->private_data, struct drm_shm_device, shm_misc_device);

	of = *offset;

	if (of > shm_dev->mem_size)
		return 0;

	if (size > (shm_dev->mem_size - of))
		size = shm_dev->mem_size - of;

	if (copy_to_user(buffer, (void *)(shm_dev->shmem_base_virt + of), size)) {
		pr_err("%s,read memory failed.\n", __func__);
		return -EFAULT;
	}
	*offset += size;

	return size;
}

static ssize_t drm_shm_write(struct file *fd, const char __user *buffer, size_t size, loff_t *offset)
{
	unsigned long of = 0;
	struct drm_shm_device *shm_dev;

	shm_dev = container_of(fd->private_data, struct drm_shm_device, shm_misc_device);

	of = *offset;

	if (of > shm_dev->mem_size)
		return 0;

	if (size > (shm_dev->mem_size - of))
		size = shm_dev->mem_size - of;

	if (copy_from_user((void *)(shm_dev->shmem_base_virt + of), buffer, size)) {
		pr_err("write memory failed.\n");
		return -EFAULT;
	}
	*offset += size;

	return size;
}

static int drm_shm_close(struct inode *inode, struct file *fd)
{
	return 0;
}

static const struct file_operations drm_share_fops = {
	.owner = THIS_MODULE,
	.open = drm_shm_open,
	.release = drm_shm_close,
	.read = drm_shm_read,
	.write = drm_shm_write,
	.llseek = drm_shm_lseek,
};

static struct miscdevice drmshm_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hsm_shmem",
	.fops = &drm_share_fops,
	.this_device = NULL,
};

static const struct of_device_id drm_shmem_match[] = {
	{.compatible = "siengine,hsm_shmem",},
	{},
};

MODULE_DEVICE_TABLE(of, drm_shmem_match);

static int drm_shm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_shm_device *shm_dev;
	struct device_node *np = dev->of_node;
	struct device_node *node;
	struct resource res;
	int ret = -1;

	shm_dev = devm_kzalloc(dev, sizeof(struct drm_shm_device), GFP_KERNEL);
	if (!shm_dev)
		return -ENOMEM;

	memset(shm_dev, 0, sizeof(struct drm_shm_device));

	node = of_parse_phandle(np, "memory-region", 0);
	if (!node) {
		dev_err(dev, "no memory-region specified\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(node, 0, &res);
	if (ret) {
		dev_err(dev, "failed to get reserved region address\n");
		of_node_put(node);
		return ret;
	}

	of_node_put(node);
	shm_dev->mem_size = resource_size(&res);
	shm_dev->shmem_base_phy = res.start;

	shm_dev->shmem_base_virt = (unsigned char *)ioremap_wc(shm_dev->shmem_base_phy, shm_dev->mem_size);

	if (!(shm_dev->shmem_base_virt)) {
		dev_err(&pdev->dev, "drm ioremap failed!\n");
		return -ENOMEM;
	}

	shm_dev->shm_misc_device = drmshm_misc_device;
	ret = misc_register(&(shm_dev->shm_misc_device));
	if (ret < 0) {
		dev_err(dev, "memory miscdrv register failed.\n");
		devm_kfree(dev, shm_dev);
		return -EFAULT;
	}

	platform_set_drvdata(pdev, shm_dev);

	return 0;
}

static int drm_shm_remove(struct platform_device *pdev)
{
	struct drm_shm_device *shm_dev = platform_get_drvdata(pdev);

	misc_deregister(&(shm_dev->shm_misc_device));

	return 0;
}

static struct platform_driver drm_shm_drv = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "drm_shmem",
		.of_match_table = drm_shmem_match,
	},

	.probe = drm_shm_probe,
	.remove = drm_shm_remove,
};

static int __init drm_shmdrv_init(void)
{
	return platform_driver_register(&drm_shm_drv);
}

static void __exit drm_shmdrv_exit(void)
{
	platform_driver_unregister(&drm_shm_drv);
}

module_init(drm_shmdrv_init);
module_exit(drm_shmdrv_exit);

MODULE_AUTHOR("jian.zhao@siengine.com");
MODULE_DESCRIPTION("Driver for drm shmem");
MODULE_LICENSE("GPL");
