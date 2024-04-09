// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/initrd.h>
#include <asm/cacheflush.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define ROOTFS_SYNC_DATA_LEN   4
#define ROOTFS_SYNC_DATA       0XAA55AA55
#define ROOTFS_SYNC_CHECK_MAX  200
#define DEVICE_NAME            "rootfs_load_sync"
#define ROOTFS_MODE_STR_LEN    16
#define INITRD_MODE            "initrd\n"
#define RAMBLK_MODE            "ramblk\n"
#define RAMBLK_ROOTFS_RESERVED_MEMORY "ram_rootfs_blk_reserved"

char saved_rootfs_mode[ROOTFS_MODE_STR_LEN] = {0};
static bool skip_rootfs_sync;

#if (!IS_ENABLED(CONFIG_BLK_DEV_INITRD))
struct ram_blk_reserved_memory {
	uint64_t        phy_addr;
	void            *virt_addr;
	uint64_t        blk_size;
};

static int si_ram_blk_reserved_memory_parse(struct ram_blk_reserved_memory *ram_blk)
{
	struct resource     ram_blk_res;
	struct device_node  *reserved_node;

	if (!ram_blk) {
		pr_err(DEVICE_NAME ":ram_blk is null, please check!!!\n");
		return -1;
	}

	reserved_node = of_find_node_by_name(NULL, RAMBLK_ROOTFS_RESERVED_MEMORY);
	if (!reserved_node) {
		pr_err(DEVICE_NAME ": Failed to find the ram_blk reserved mmeory node\n");
		return -1;
	}

	if (of_address_to_resource(reserved_node, 0, &ram_blk_res)) {
		pr_err(DEVICE_NAME ": Failed to get reg address\n");
		of_node_put(reserved_node);
		return -1;
	}
	of_node_put(reserved_node);

	ram_blk->phy_addr = ram_blk_res.start;
	ram_blk->blk_size = resource_size(&ram_blk_res);
	ram_blk->virt_addr = ioremap_cache(ram_blk->phy_addr, ram_blk->blk_size);

	return 0;
}
#endif

static int rootfs_load_proc_show(struct seq_file *m, void *v)
{
	seq_puts(m, saved_rootfs_mode);
	return 0;
}

static int __init rootfs_load_sync(void)
{
	uint32_t retry = 0;
	uint32_t data;
	uint64_t adr_start, adr_end;
	uint32_t rootfs_mode_len = ROOTFS_MODE_STR_LEN;

	if (skip_rootfs_sync)
		return retry;

#if (!IS_ENABLED(CONFIG_BLK_DEV_INITRD))
	uint32_t err;
	struct ram_blk_reserved_memory ram_blk;

	err = si_ram_blk_reserved_memory_parse(&ram_blk);
	if (err)
		pr_err(DEVICE_NAME ": Failed parse ram_blk reserved memory.\n");
	adr_start = (uint64_t)ram_blk.virt_addr;
	adr_end = (uint64_t)ram_blk.virt_addr + ram_blk.blk_size;
	if (rootfs_mode_len > sizeof(RAMBLK_MODE))
		rootfs_mode_len = sizeof(RAMBLK_MODE);
	strncpy(saved_rootfs_mode, RAMBLK_MODE, rootfs_mode_len);
#else
	if (!initrd_start || IS_ENABLED(CONFIG_INITRAMFS_FORCE))
		return 0;
	adr_start = initrd_start;
	adr_end = initrd_end;
	if (rootfs_mode_len > sizeof(INITRD_MODE))
		rootfs_mode_len = sizeof(INITRD_MODE);
	strncpy(saved_rootfs_mode, INITRD_MODE, rootfs_mode_len);
#endif

	__inval_dcache_area((void *)(adr_end - ROOTFS_SYNC_DATA_LEN), ROOTFS_SYNC_DATA_LEN);
	data = *(uint32_t *)(adr_end - ROOTFS_SYNC_DATA_LEN);

	/* We only check 2 seconds */
	while (data != ROOTFS_SYNC_DATA) {
		usleep_range(10000, 11000);
		__inval_dcache_area((void *)(adr_end - ROOTFS_SYNC_DATA_LEN), ROOTFS_SYNC_DATA_LEN);
		data = *(uint32_t *)(adr_end - ROOTFS_SYNC_DATA_LEN);
		retry++;
		if (retry >= ROOTFS_SYNC_CHECK_MAX) {
			pr_emerg("rootfs load fail, please check...\n");
			BUG();
		}
	}
	__inval_dcache_area((void *)adr_start, adr_end - adr_start);

	proc_create_single("rootfs_mode", 0, NULL, rootfs_load_proc_show);

	return 0;
}

static int __init early_rootfs_load_sync_skip(char *buf)
{
	return strtobool(buf, &skip_rootfs_sync);
}
early_param("skip.rootfs_sync", early_rootfs_load_sync_skip);

#if (!IS_ENABLED(CONFIG_BLK_DEV_INITRD))
device_initcall_sync(rootfs_load_sync);
#else
fs_initcall(rootfs_load_sync);
#endif

MODULE_DESCRIPTION("Siengine SE1000 rootfs and ramfs load sync driver");
MODULE_AUTHOR("jinhua.chen <jinhua.chen@siengine.com>");
MODULE_LICENSE("GPL v2");
