/*
 * SPDX-License-Identifier: GPL-2.0.
 * drivers/block/se1000_virtblk
 * Copyright (C) 2022-2023 SiEngine or its affiliates
*/

#include "se1000_virtblk.h"
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>

#define DEVICE_NAME "sv_ramblk"
#define BLK_NAME "ramblk"
#define SIENGINE_RAM_BLK_MAJOR	262
#define SV_BLKDEV_MINOR_MAX (16)
#define SV_BLKDEV_DISK_MAX_PARTS (1)
#define RAMBLK_MAX_UNITS (4)
#define FS_BLOCK_SIZE_SHIFT (12) /* 4KB unit */
#define INIT_BLOCK_DEVICE_TIMEOUT_MS (10 * 1000)

static DEFINE_MUTEX(ramblk_mutex);
static struct request_queue *sv_blk_queue;
int32_t blk_count_max;
static struct task_struct *service_task;

struct ram_blk_dev {
	uint64_t	phy_addr;
	void		*virt_addr;
	uint64_t	blk_size;
	struct gendisk *sv_blk_disk;
	struct blk_mq_tag_set tag_set;
	/* below is for phy block device data */
	const char  *blk_name;  /* block device name */
	int         init_flag;  /* 1: block device inited */
	struct file *filp;      /* file to block device */
	uint32_t    *index_list;         /* block index list */
	uint32_t    index_list_maxlen;   /* block index list max len */
	uint32_t    *copy_index_list;    /* need copy block index list */
	uint32_t    copy_index_list_len; /* need copy block index list len */
} ram_blk_dev_node[RAMBLK_MAX_UNITS];

static void release_block_data(struct ram_blk_dev *ramdev, int dev_len)
{
	int i = 0;

	for (i = 0; i < dev_len; i++) {
		if (ramdev[i].index_list) {
			vfree(ramdev[i].index_list);
			ramdev[i].index_list = NULL;
			ramdev[i].index_list_maxlen = 0;
		}
		if (ramdev[i].copy_index_list) {
			vfree(ramdev[i].copy_index_list);
			ramdev[i].copy_index_list = NULL;
			ramdev[i].copy_index_list_len = 0;
		}
	}
}

static blk_status_t sv_ramblk_queue_rq(struct blk_mq_hw_ctx *hctx,
				const struct blk_mq_queue_data *bd)
{
	struct request *req = bd->rq;
	unsigned long start, len, total;
	blk_status_t err = BLK_STS_OK;
	struct ram_blk_dev *ramdev = req->rq_disk->private_data;
	void *buffer, *ramblk_addr;
	uint64_t file_offset = 0, block_offset = 0;

	mutex_lock(&ramblk_mutex);

	blk_mq_start_request(req);
	total = blk_rq_bytes(req);
	start = blk_rq_pos(req) << SECTOR_SIZE_SHIFT;

	if (start + total > ramdev->blk_size) {
		pr_err(DEVICE_NAME ": bad access: block=%llu, "
		       "count=%u\n",
		       (unsigned long long)blk_rq_pos(req),
		       blk_rq_cur_sectors(req));
		err = BLK_STS_IOERR;
		goto done;
	}

	ramblk_addr = ramdev->virt_addr + start;
	do {
		len  = blk_rq_cur_bytes(req);
		buffer = bio_data(req->bio);
		if (rq_data_dir(req) == READ) {
			memcpy(buffer, ramblk_addr, len);
		} else {
			if (ramdev->blk_name) {
				if (ramdev->init_flag) {
					memcpy(ramblk_addr, buffer, len);
					file_offset = ramblk_addr - ramdev->virt_addr;
					kernel_write(ramdev->filp, buffer, len, &file_offset);
				} else {
					memcpy(ramblk_addr, buffer, len);
					/* Save the index of the changed block */
					block_offset = (ramblk_addr - ramdev->virt_addr) >> FS_BLOCK_SIZE_SHIFT;
					if (ramdev->index_list[block_offset] == 0) {
						ramdev->index_list[block_offset] = 1;
						ramdev->copy_index_list[ramdev->copy_index_list_len] = block_offset;
						ramdev->copy_index_list_len++;
					}
				}
			} else {
				memcpy(ramblk_addr, buffer, len);
			}
		}
		ramblk_addr += len;
	} while (blk_update_request(req, err, len));
done:
	blk_mq_end_request(req, err);

	mutex_unlock(&ramblk_mutex);

	return err;
}

static int sv_ramblk_open(struct block_device *dev, fmode_t mode)
{
	return 0;
}

static void sv_ramblk_release(struct gendisk *gd, fmode_t mode)
{
}

static const struct block_device_operations sv_ramblk_fops = {
	.owner = THIS_MODULE,
	.open = sv_ramblk_open,
	.release = sv_ramblk_release,
};

static struct kobject *sv_ramblk_find(dev_t dev, int *part, void *data)
{
	int drive = *part & 3;
	*part = 0;
	return get_disk_and_module(ram_blk_dev_node[drive].sv_blk_disk);
}

static const struct blk_mq_ops sv_ramblk_mq_ops = {
	.queue_rq       = sv_ramblk_queue_rq,
};

static int sv_init_block_service(void *arg)
{
	int wait_ms = 0;
	int i = 0, j = 0;
	int wait_init_block_device = 0;
	uint64_t file_offset = 0;
	struct ram_blk_dev *ramdev = (struct ram_blk_dev *)arg;

	while (wait_ms < INIT_BLOCK_DEVICE_TIMEOUT_MS) {
		msleep(200);
		wait_ms += 200;
		wait_init_block_device = 0;
		for (i = 0; i < blk_count_max; i++) {
			/* Init the real block device */
			if (ramdev[i].blk_name && (ramdev[i].init_flag == 0)) {
				wait_init_block_device++;
				/* Open the real block device */
				ramdev[i].filp = filp_open(ramdev[i].blk_name, O_RDWR, 0);
				if (IS_ERR(ramdev[i].filp)) {
					ramdev[i].filp = NULL;
				} else {
					mutex_lock(&ramblk_mutex);
					/* Save the flag */
					ramdev[i].init_flag = 1;
					wait_init_block_device--;

					/* Copy the DDR data to real block device */
					for (j = 0; j < ramdev[i].copy_index_list_len; j++) {
						file_offset = (ramdev[i].copy_index_list[j]) << FS_BLOCK_SIZE_SHIFT;
						kernel_write(ramdev[i].filp, ramdev[i].virt_addr + file_offset,
							1 << FS_BLOCK_SIZE_SHIFT, &file_offset);
					}
					mutex_unlock(&ramblk_mutex);
				}
			}
		}
		/* exit when all device be inited */
		if (wait_init_block_device == 0) {
			release_block_data(ramdev, blk_count_max);
			break;
		}
	}

	sv_info("%s exit", __func__);
	return 0;
}

static int __init sv_ramblk_c_init(void)
{
	struct resource			ram_blk_res;
	struct device_node		*root_node;
	struct device_node		*blk_node;
	struct device_node		*reserved_node;
	static struct gendisk		*sv_blk_disk;
	int i, ret, blk_index, has_real_block = 0;
	const char		*blk_name;

	ret = -EINVAL;
	memset(&ram_blk_res, 0, sizeof(ram_blk_res));

	root_node = of_find_node_by_name(NULL, "si_ram_blk");
	if (!root_node) {
		pr_devel(DEVICE_NAME ": Failed to find the sv_ram blk node\n");
		goto err_dts;
	}

	blk_node = NULL;
	blk_index = 0;
	while (1) {
		blk_node = of_get_next_child(root_node, blk_node);
		if (blk_node == NULL) {
			break;
		}
		reserved_node = of_parse_phandle(blk_node, "memory-region", 0);
		if (!reserved_node) {
			pr_err(DEVICE_NAME ": Failed to find the log_collector mem reserved node\n");
			goto err_dts;
		}

		if (of_address_to_resource(reserved_node, 0, &ram_blk_res)) {
			pr_err(DEVICE_NAME ": Failed to get reg address\n");
			of_node_put(reserved_node);
			goto err_dts;
		}
		of_node_put(reserved_node);

		ram_blk_dev_node[blk_index].phy_addr = ram_blk_res.start;
		ram_blk_dev_node[blk_index].blk_size = resource_size(&ram_blk_res);
		ram_blk_dev_node[blk_index].virt_addr = ioremap_cache(ram_blk_dev_node[blk_index].phy_addr, ram_blk_dev_node[blk_index].blk_size);
		sv_blk_disk = alloc_disk(SV_BLKDEV_DISK_MAX_PARTS);
		if (sv_blk_disk) {
			sprintf(sv_blk_disk->disk_name, blk_node->name);
			ram_blk_dev_node[blk_index].sv_blk_disk = sv_blk_disk;
		}

		if (of_property_read_string(blk_node, "block_name", &blk_name) < 0) {
			ram_blk_dev_node[blk_index].blk_name = NULL;
		} else {
			/* init real block device data */
			has_real_block = 1;
			ram_blk_dev_node[blk_index].blk_name = blk_name;
			ram_blk_dev_node[blk_index].init_flag = 0;

			ram_blk_dev_node[blk_index].index_list_maxlen =
				ram_blk_dev_node[blk_index].blk_size >> FS_BLOCK_SIZE_SHIFT;
			ram_blk_dev_node[blk_index].index_list =
				vmalloc(sizeof(uint32_t) * ram_blk_dev_node[blk_index].index_list_maxlen);

			ram_blk_dev_node[blk_index].copy_index_list_len = 0;
			ram_blk_dev_node[blk_index].copy_index_list =
				vmalloc(sizeof(uint32_t) * ram_blk_dev_node[blk_index].index_list_maxlen);

			if (IS_ERR(ram_blk_dev_node[blk_index].index_list) ||
				IS_ERR(ram_blk_dev_node[blk_index].copy_index_list)) {
				sv_err("%s vmalloc failed", blk_name);
				goto err_register;
			}
			memset(ram_blk_dev_node[blk_index].index_list, 0, ram_blk_dev_node[blk_index].index_list_maxlen);
		}
		blk_index++;

	}

	blk_count_max = blk_index;
	ret = -EBUSY;
	if (register_blkdev(SIENGINE_RAM_BLK_MAJOR, DEVICE_NAME))
		goto err_register;


	blk_register_region(MKDEV(SIENGINE_RAM_BLK_MAJOR, 0), SV_BLKDEV_MINOR_MAX, THIS_MODULE,
				sv_ramblk_find, NULL, NULL);

	for (i = 0; i < blk_count_max; i++) {
		ret = -ENOMEM;
		sv_blk_disk = ram_blk_dev_node[i].sv_blk_disk;
		if (!sv_blk_disk)
			goto err_alloc_disk;
		/* Create the request queue */
		sv_blk_queue = blk_mq_init_sq_queue(&ram_blk_dev_node[i].tag_set, &sv_ramblk_mq_ops,
							128, BLK_MQ_F_SHOULD_MERGE | BLK_MQ_F_BLOCKING);
		if (IS_ERR(sv_blk_queue)) {
			ret = PTR_ERR(sv_blk_queue);
			sv_blk_queue = NULL;
			goto err_init_queue;
		}
		set_capacity(sv_blk_disk, ram_blk_dev_node[i].blk_size >> SECTOR_SIZE_SHIFT);
		sv_blk_disk->major = SIENGINE_RAM_BLK_MAJOR;
		sv_blk_disk->first_minor = i;
		sv_blk_disk->fops = &sv_ramblk_fops;
		sv_blk_disk->queue = sv_blk_queue;
		blk_queue_logical_block_size(sv_blk_queue, IPC_CHUNKSIZE);
		blk_queue_max_segments(sv_blk_queue, 16);
		blk_queue_max_segment_size(sv_blk_queue, IPC_CHUNKSIZE);
		sv_blk_disk->private_data = &ram_blk_dev_node[i];
		add_disk(sv_blk_disk);
	}

	if (has_real_block) {
		service_task = kthread_run(sv_init_block_service, (void *)&ram_blk_dev_node[0], "sv_init_block_service");
		if (IS_ERR(service_task)) {
			pr_err(DEVICE_NAME "Unable to start kernel thread");
			goto err_init_queue;
		}
	}

	return 0;

err_init_queue:
	put_disk(sv_blk_disk);
err_alloc_disk:
	unregister_blkdev(SIENGINE_RAM_BLK_MAJOR, DEVICE_NAME);
err_register:
err_dts:
	release_block_data(ram_blk_dev_node, blk_count_max);
	return ret;
}

static void __exit sv_ramblk_c_exit(void)
{
	int i;
	static struct gendisk *sv_blk_disk;

	blk_unregister_region(MKDEV(SIENGINE_RAM_BLK_MAJOR, 0), SV_BLKDEV_MINOR_MAX);
	unregister_blkdev(SIENGINE_RAM_BLK_MAJOR, DEVICE_NAME);
	for (i = 0; i < blk_count_max; i++) {
		sv_blk_disk = ram_blk_dev_node[i].sv_blk_disk;
		blk_cleanup_queue(sv_blk_disk->queue);
		blk_mq_free_tag_set(&ram_blk_dev_node[i].tag_set);
		del_gendisk(sv_blk_disk);
		put_disk(sv_blk_disk);
		if (ram_blk_dev_node[i].filp) {
			filp_close(ram_blk_dev_node[i].filp, NULL);
			ram_blk_dev_node[i].filp = NULL;
		}
	}
	release_block_data(ram_blk_dev_node, blk_count_max);
}

device_initcall_sync(sv_ramblk_c_init);
module_exit(sv_ramblk_c_exit);

MODULE_LICENSE("GPL v2");
