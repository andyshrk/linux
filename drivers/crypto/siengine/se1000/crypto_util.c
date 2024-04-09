// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 SiEngine Technology Co., Ltd
 * This file contains the driver common operations.
 *
 */
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/dma-direct.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include "crypto_util.h"

bool check_sg_valid(struct scatterlist *sg)
{
	struct page *page;

	do {
		if (!sg)
			return false;

		page = sg_page(sg);
		if (!page || !pfn_valid(page_to_pfn(page)))
			return false;

		sg = sg_next(sg);
	} while (sg);

	return true;
}

inline uint32_t get_ddt_len(struct ddt_pdu *ddtlist)
{
	uint32_t count = 0;

	while (ddtlist->ptr != NULL) {
		count++;
		ddtlist++;
	}
	return count;
}

inline uint32_t get_ddt_data_len(struct ddt_pdu *ddtlist)
{
	uint32_t len = 0;

	while (ddtlist->ptr != NULL) {
		len += ddtlist->len;
		ddtlist++;
	}

	return len;
}

struct ddt_pdu *sg_to_ddt(struct device *dev, struct scatterlist *sglist,
	enum dma_data_direction direct)
{
	struct scatterlist *sg = NULL;
	struct ddt_pdu *ddt_list = NULL;
	int size, i;

	size = sg_nents(sglist);
	ddt_list = kzalloc(sizeof(struct ddt_pdu) * (size + 1), GFP_NOFS);
	size = dma_map_sg(dev, sglist, size, direct);
	if (size <= 0)
		return NULL;

	for_each_sg(sglist, sg, size, i) {
		ddt_list[i].ptr = (uint8_t *)sg_dma_address(sg);
		ddt_list[i].len = sg_dma_len(sg);
	}

	return ddt_list;
}

int parse_dts(struct device *dev, uint8_t **virt_addr, u32 *use_vf, u32 *ddt, struct resource *shmem_res)
{
	int err;
	struct device_node *np = dev->of_node;
	struct device_node *shmem = NULL;

	err = of_property_read_u32_index(np, "use_vf", 0, use_vf);
	if (err) {
		dev_err(dev, "dts do not have use_vf property.");
		return -ENODEV;
	}

	err = of_property_read_u32_index(np, "ddt_mode", 0, ddt);
	if (err) {
		dev_err(dev, "dts do not have ddt_mode property.");
		return -ENODEV;
	}

	shmem = of_parse_phandle(np, "shmem_region", 0);
	if (!shmem) {
		dev_err(dev, "Failed to find the shmem-region node.");
		return -ENODEV;
	}
	if (of_address_to_resource(shmem, 0, shmem_res)) {
		dev_err(dev, "Failed to get shmem address.");
		of_node_put(shmem);
		return -ENODEV;
	}
	of_node_put(shmem);

	*virt_addr = of_iomap(np, 0);
	if (*virt_addr == NULL)
		return -EFAULT;

	return 0;
}
