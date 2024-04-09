
#ifndef __SE1000_MAILBOX_H__
#define __SE1000_MAILBOX_H__

#include <linux/mailbox_controller.h>
#include <linux/irqdomain.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

struct se1000_mbox_chan {
	unsigned int irq;
	int idx;
	void __iomem *ch_base;
	struct device_node *ch_np;
	const char *src_name;
	const char *dst_name;
	struct se1000_mbox *mbox;
	struct irq_domain *irq_domain;
	struct proc_dir_entry *entry;
};

struct se1000_mbox {
	struct device *dev;
	void __iomem *base;
	u32 nr_mbox;
	struct se1000_mbox_chan *mboxes;
	struct mbox_chan *chans;
	struct mbox_controller controller;
	struct proc_dir_entry *mboxes_dir;
};

#define MBOX_MSG_LEN			4
#define MBOX_BIT_LEN			32
#define MBOX_BYTE_LEN			(MBOX_BIT_LEN >> 3)
#define VFIFO_MSG_LEN			(16 * sizeof(u32))

#endif /* __SE1000_MAILBOX_H__ */
