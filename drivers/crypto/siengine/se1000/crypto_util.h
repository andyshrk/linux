// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright(C) 2023-2023 SiEngine Technology Co., Ltd
 *
 * This file contains the driver common operation macro.
 */

#ifndef __CRYPTO_UTIL_H__
#define __CRYPTO_UTIL_H__

#include <linux/io.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>

#define DESC(x)		1

#define WRITE_REG(addr, value)	writel((value), (addr))
#define READ_REG(addr)			readl((addr))

#define REG_TO_UINT32(reg) (*(uint32_t *)(&reg))
#define REG_FROM_UINT32(reg, value) (*(uint32_t *)(&reg) = value)

struct ddt_pdu {
	uint8_t *ptr __aligned(8);
	uint64_t len __aligned(8);
};

inline uint32_t get_ddt_len(struct ddt_pdu *ddtlist);
inline uint32_t get_ddt_data_len(struct ddt_pdu *ddtlist);
struct ddt_pdu *sg_to_ddt(struct device *dev, struct scatterlist *sglist,
	enum dma_data_direction direct);

#define DDT_LIST_SIZE(ddt_list) ((get_ddt_len(ddt_list) + 1) * sizeof(struct ddt_pdu))

bool check_sg_valid(struct scatterlist *sg);

int parse_dts(struct device *dev, uint8_t **virt_addr, u32 *use_vf, u32 *ddt, struct resource *shmem_res);
#endif
