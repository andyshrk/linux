// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright(C) 2023-2023 SiEngine Technology Co., Ltd
 *
 * This file contains interrupt handling operations for drivers.
 */

#include <linux/interrupt.h>

#include "smx_irq.h"
#include "crypto_ips_internal.h"
#include "crypto_util.h"
#include "crypto_res_mgr.h"

static struct smx_irq_args g_smx_irq_args[VFx_CMD_COUNT];

inline struct smx_irq_args *get_smx_irq_args(int index)
{
	if (unlikely(index >= VFx_CMD_COUNT))
		return NULL;

	return &g_smx_irq_args[index];
}

static irqreturn_t smx_irq_process(int irq, void *dev_id)
{
	CRYPTO_IRQ_STAT irq_stat = {0};
	int producer_idx;
	int consumer_idx;
	uint16_t cmd_idx;
	struct smx_irq_args *args = NULL;
	struct smx_status *status = NULL;

	REG_FROM_UINT32(irq_stat, READ_REG(SMX_VFx_REG_IRQ_STAT));
	if (irq_stat.IOC_STAT_0 || irq_stat.STS_ERR_STAT_0) {
		free_smx_cmd();

		producer_idx = READ_REG(SMX_VFx_REG_C0_MAILBOX_2);
		consumer_idx = READ_REG(SMX_VFx_REG_C0_MAILBOX_3);

		status = SHMEM_SMX_STS(consumer_idx);
		cmd_idx = status->sw_id;
		args = get_smx_irq_args(cmd_idx);
		if (args != NULL) {
			args->ret_code = status->ret_code;
			complete(&args->wait);
			if (args->async_req)
				args->async_req->complete(args->async_req, args->ret_code);
		}

		WRITE_REG(SMX_VFx_REG_C0_MAILBOX_3, (consumer_idx + 1) % VFx_STATUS_COUNT);
		WRITE_REG(SMX_VFx_REG_IRQ_CLR, REG_TO_UINT32(irq_stat));
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

void smx_irq_init(int irq)
{
	int i;

	/* init irq args */
	for (i = 0; i < VFx_CMD_COUNT; i++) {
		init_completion(&g_smx_irq_args[i].wait);
		g_smx_irq_args[i].ret_code = 0;
	}

	if (request_irq(irq, smx_irq_process, 0, "smx", NULL) != 0)
		pr_err("irq init failed.");
}

void smx_irq_uninit(int irq)
{
	free_irq(irq, NULL);
}
