// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 *
 * This file contains interrupt handling operations for drivers.
 */

#include <linux/printk.h>
#include <linux/interrupt.h>

#include "ips_irq.h"
#include "crypto_ips_init.h"
#include "crypto_ips_internal.h"
#include "crypto_util.h"
#include "crypto_res_mgr.h"

static struct ips_irq_args g_irq_args[VFx_CMD_COUNT];
static int g_irq_num;
static uint16_t g_irq_ref;
static spinlock_t g_irq_ref_lock;

inline struct ips_irq_args *get_irq_args(int index)
{
	if (unlikely(index >= VFx_CMD_COUNT))
		return NULL;

	return &g_irq_args[index];
}

void ips_set_irq_num(int irq)
{
	int i;

	g_irq_num = irq;

	/* init irq args */
	for (i = 0; i < VFx_CMD_COUNT; i++) {
		init_completion(&g_irq_args[i].wait);
		g_irq_args[i].ret_code = 0;
	}

	spin_lock_init(&g_irq_ref_lock);
	g_irq_ref = 0;
}

static irqreturn_t ips_irq_process(int irq, void *dev_id)
{
	CRYPTO_IRQ_STAT irq_stat = {0};
	int producer_idx;
	int consumer_idx;
	uint16_t cmd_idx;
	struct ips_irq_args *args = NULL;
	uint32_t target_vfx_irq = 1 << (g_use_vf + 8);

	if ((READ_REG(IPS_NS_IRQ) & target_vfx_irq) == 0)
		return IRQ_NONE;

	REG_FROM_UINT32(irq_stat, READ_REG(IPS_VFx_REG_IRQ_STAT));
	if (irq_stat.IOC_STAT_0 || irq_stat.STS_ERR_STAT_0) {
		free_cmd_desc();

		producer_idx = READ_REG(IPS_VFx_REG_C0_MAILBOX_2);
		consumer_idx = READ_REG(IPS_VFx_REG_C0_MAILBOX_3);

		cmd_idx = g_vfx_shmem->status[consumer_idx].sw_id;
		args = get_irq_args(cmd_idx);
		if (args != NULL) {
			args->ret_code = g_vfx_shmem->status[consumer_idx].ret_code;
			complete(&args->wait);
			if (args->async_req) {
				args->async_req->complete(args->async_req, args->ret_code);
			}
		}
		memset(&(g_vfx_shmem->status[consumer_idx]), 0, 8);

		WRITE_REG(IPS_VFx_REG_C0_MAILBOX_3, (consumer_idx + 1) % VFx_STATUS_COUNT);
		WRITE_REG(IPS_VFx_REG_IRQ_CLR, REG_TO_UINT32(irq_stat));

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

void ips_irq_init(void)
{
	uint16_t irq_ref_num;

	spin_lock(&g_irq_ref_lock);
	irq_ref_num = g_irq_ref++;
	spin_unlock(&g_irq_ref_lock);

	if (irq_ref_num == 0)
		if (request_irq(g_irq_num, ips_irq_process, 0, "ips", NULL) != 0)
			pr_err("irq init failed.");
}

void ips_irq_uninit(void)
{
	uint16_t irq_ref_num;

	spin_lock(&g_irq_ref_lock);
	irq_ref_num = --g_irq_ref;
	spin_unlock(&g_irq_ref_lock);

	if (irq_ref_num == 0)
		free_irq(g_irq_num, NULL);
}
