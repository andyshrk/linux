/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __SE1000_RPMSG_DEV_H__
#define __SE1000_RPMSG_DEV_H__

//#define SE1000_RPMSG_IOCTL_MAGIC		'r'

//* Configuration and Status Registers */
#define IPC_REG_TXD			    0x00
#define IPC_REG_TXST			0x04
#define IPC_REG_RXD			    0x08
#define IPC_REG_RXST			0x0c
#define IPC_REG_RSTN			0x10
#define IPC_REG_ERR			    0x14


/* IPC_TXST */
#define IPC_TXFIFO_CNT_BIT		    4
#define IPC_TXFIFO_CNT_MASK		0xf
#define IPC_TXFIFO_EMPTY		BIT(1)
#define IPC_TXFIFO_FULL			BIT(0)

/* IPC_RXST */
#define IPC_RXFIFO_CNT_BIT		4
#define IPC_RXFIFO_CNT_MASK		0xf
#define IPC_RXFIFO_EMPTY		BIT(1)
#define IPC_RXFIFO_FULL			BIT(0)

/* IPC_RSTN */
#define IPC_RXFIFO_RST			BIT(1)
#define IPC_TXFIFO_RST			BIT(0)

/* IPS_ERR */
#define IPC_RXFIFO_UDFL			BIT(1)
#define IPC_TXFIFO_OVFL			BIT(0)

#define VFIFO_MSG_LEN			(16 * sizeof(u32))

struct se1000_rpmsg_info_head {
	u32 src;
	u32 dst;
	u32 reverse;
	u16 length;
	u16 flags;
	u8 data[0];
}__packed;

#define CACHE_LINE     512
#define MAX_FIFO_SIZE  32

struct shm_ring {
	union {
		u8 magic_data[CACHE_LINE];
		u32 magic;
	};
	union {
		u8 in_data[CACHE_LINE];
		u32 in;
	};
	union {
		u8 out_data[CACHE_LINE];
		u32 out;
	};
	u8 data[0][CACHE_LINE];
} __attribute__((aligned(CACHE_LINE)));

struct se1000_rpmsg_endpoint {
	struct device dev;
	struct rpmsg_endpoint ept;
	struct se1000_rpmsg_dev *rpmsg_dev;
	struct rpmsg_channel_info chinfo;
	struct list_head list;
};

/*rpmsg_chan*/
struct se1000_rpmsg_dev {
	struct device *dev;
	struct rpmsg_device rpdev;
	void *chan;
	int irq;
	const char *name;
	const char *link_port;             //link_port
	size_t mem_size;
	phys_addr_t phys_addr;
	void __iomem *virt_addr;
	struct list_head ep_list_head;        //ep_list_head
	struct shm_ring  *tx_fifo;           //shm_ring_buf
	struct shm_ring  *rx_fifo;
	spinlock_t rpmsg_dev_lock;
	spinlock_t ept_lock;
	struct work_struct ipc_work;        //work for ipc
//	bool link_up;						//PortB send after link_up is true
};

#endif
