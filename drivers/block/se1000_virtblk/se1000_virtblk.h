/*
 * Siengine Virtual Block device driver
 *
 * Copyright (c) 2018- Siengine Technologies CO., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/blk-mq.h>
#include <linux/cdev.h>
#include <linux/rpmsg.h>
#include <linux/kthread.h>
#include <linux/kfifo.h>
#include <linux/mailbox_client.h>
 #include <linux/rpmsg/siengine-rpmsg.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>
#include <linux/init_syscalls.h>
#include <linux/platform_device.h>

#define SECTOR_SIZE_SHIFT (9)

#define IPC_CHUNKCOUNT		(128)
#define IPC_CHUNKSIZE		(0x0001000)
#define IPC_CHUNKMASK		(0x0000fff)
#define IPC_CHUNKSHIFT	(12)
#define NOCACHE_HEADER_SIZE	(0x1000)

#define BLOCK_SIZE_ERR		(0)
#define WAIT_SERVICE_READY_MAX	(50)

#define sv_err(format, ...)	pr_err(DEVICE_NAME ": "format".\n", ##__VA_ARGS__)
#define sv_info(format, ...)	pr_info(DEVICE_NAME ": "format".\n", ##__VA_ARGS__)

enum rpmsg_data_type {
	RPMSG_REGISTER = 1,
	RPMSG_DEREGISTER,
	RPMSG_WAKEUP,
	RPMSG_SLEEP,
	RPMSG_READ,
	RPMSG_WRITE,
	RPMSG_RELEASE,
	RPMSG_OPEN,
};

enum rw_flag {
	IDLE = 0,
	READY,
	ACTION,
	DONE,
	ERROR,
};

enum sv_blk_status {
	SVBLK_OPEN = 0,
	SVBLK_RELEASE,
	SVBLK_BUSY,
	SVBLK_ERROR,
};

struct sv_blk_dev {
	uint64_t	phy_addr;
	void		*virt_addr;
	void		*buf_addr;
	const char	*blk_node;
	const char	*blk_node_standby;
	struct rpmsg_endpoint	*ept;
	struct workqueue_struct *wq;
	struct work_struct work;
	spinlock_t queue_lock;
	struct mutex lock;
};

struct rpmsg_payload {
	enum			rpmsg_data_type type;
	uint64_t		phy_addr;
	uint64_t		block_size;
};

struct shmem_header {
	enum			rpmsg_data_type type;
	enum			rw_flag flag;
	enum			sv_blk_status status;
	uint64_t		pos;
	uint64_t		size;
};

struct sv_blk_rq_status {
	uint64_t		ipc_data_size[IPC_CHUNKCOUNT];
	void			*ipc_data_addr[IPC_CHUNKCOUNT];
};

void sv_blk_flush_dcache_area(void *kaddr, size_t len);
void sv_blk_inval_dcache_area(void *kaddr, size_t len);

static inline __init int create_dev(char *name, dev_t dev)
{
	init_unlink(name);
	return init_mknod(name, S_IFBLK | 0600, new_encode_dev(dev));
}
