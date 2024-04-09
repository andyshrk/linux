// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Siengine
 */

#ifndef _KOMEDA_KMS_AGENT_DEBUG_H
#define _KOMEDA_KMS_AGENT_DEBUG_H

#define AGNET_ERR_LEVEL		0x01
#define AGENT_INFO_LEVEL	0x02
#define AGENT_DEBUG_LEVEL	0x04

#define IPC_ERR_LEVEL		0x10
#define IPC_INFO_LEVEL		0x20
#define IPC_DEBUG_LEVEL		0x40

#define RPMSG_ERR_LEVEL		0x100
#define RPMSG_INFO_LEVEL	0x200
#define RPMSG_DEBUG_LEVEL	0x400

#define KOMEDA_AGENT_ERR(fmt, ...) \
	komeda_agent_printk(KERN_ERR, __func__, "agent", AGNET_ERR_LEVEL, fmt, ##__VA_ARGS__)
#define KOMEDA_AGENT_INFO(fmt, ...) \
	komeda_agent_printk(KERN_INFO, __func__, "agent", AGENT_INFO_LEVEL, fmt, ##__VA_ARGS__)
#define KOMEDA_AGENT_DEBUG(fmt, ...) \
	komeda_agent_printk(KERN_DEBUG, __func__, "agent", AGENT_DEBUG_LEVEL, fmt, ##__VA_ARGS__)

#define KOMEDA_IPC_ERR(fmt, ...) \
	komeda_agent_printk(KERN_ERR, __func__, "ipc", IPC_ERR_LEVEL, fmt, ##__VA_ARGS__)
#define KOMEDA_IPC_INFO(fmt, ...) \
	komeda_agent_printk(KERN_INFO, __func__, "ipc", IPC_INFO_LEVEL, fmt, ##__VA_ARGS__)
#define KOMEDA_IPC_DEBUG(fmt, ...) \
	komeda_agent_printk(KERN_DEBUG, __func__, "ipc", IPC_DEBUG_LEVEL, fmt, ##__VA_ARGS__)

#define KOMEDA_RPMSG_ERR(fmt, ...) \
	komeda_agent_printk(KERN_ERR, __func__, "rpmsg", RPMSG_ERR_LEVEL, fmt, ##__VA_ARGS__)
#define KOMEDA_RPMSG_INFO(fmt, ...) \
	komeda_agent_printk(KERN_INFO, __func__, "rpmsg", RPMSG_INFO_LEVEL, fmt, ##__VA_ARGS__)
#define KOMEDA_RPMSG_DEBUG(fmt, ...) \
	komeda_agent_printk(KERN_DEBUG, __func__, "rpmsg", RPMSG_DEBUG_LEVEL, fmt, ##__VA_ARGS__)

void komeda_agent_printk(const char *level, const char* func, char *class, unsigned int category,
		const char *fmt, ...);

#endif