/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __SE1000_MAILBOX_DEV_H__
#define __SE1000_MAILBOX_DEV_H__

#define SE1000_MBOX_IOCTL_MAGIC		'm'

/* send reserved memory phy address to peer core */
#define SE1000_MBOX_IOCTL_PHY_ADDR		_IOW(SE1000_MBOX_IOCTL_MAGIC, 0 ,int)

#define SE1000_MBOX_IOCTL_MAX		1


#endif /* __SE1000_MAILBOX_DEV_H__ */
