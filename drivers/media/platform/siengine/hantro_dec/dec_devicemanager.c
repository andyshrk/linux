// SPDX-License-Identifier: GPL-2.0
/*
 * dec_devicemanager.c - Siengine Decoder Device Manager
 *
 * Copyright (C) 2022 - 2023 Siengine
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 */

#include <linux/io.h>

#ifdef CONFIG_OF
#include <linux/of_platform.h>
#endif

#include "dec_devicemanager.h"

#define VPU_DEVMGR_BASE			0xED000020
#define VPU_RES_STATUS_MASK		0x7
#define VPU_DEC_RESID			0    //enc 1
#define VPU_ALLOCATE_ADDR_OFFSET	0
#define VPU_RELEASE_ADDR_OFFSET		4
#define VPU_STATUS_ADDR_OFFSET		8

enum device_mgr_resource_status {
	resource_alloc_mp,
	resource_alloc_ap1,
	resource_alloc_ap0,
	resource_alloc_rp,
	resource_free,
};

static int check_resource_status(void *status_addr, int resource_index,
		enum device_mgr_resource_status *resource_status)
{
	unsigned int data = 0;

	data = ioread32(status_addr);
	*resource_status = (data >> (resource_index * 3)) & VPU_RES_STATUS_MASK;
	return 0;
}

int dec_dev_release(void)
{
	int ret = 0;
	enum device_mgr_resource_status res_status;
	char *dec_res_base = NULL;
	void *res_status_addr = NULL;
	void *res_release_addr = NULL;

	if (!request_mem_region(VPU_DEVMGR_BASE, 4 * 3, "vpudecres")) {
		pr_err("dec: res release request_mem_region fail\n");
		return -EBUSY;
	}

	dec_res_base = (char *)ioremap(VPU_DEVMGR_BASE, 4 * 3);
	res_status_addr = (void *)(dec_res_base + VPU_STATUS_ADDR_OFFSET);
	res_release_addr = (void *)(dec_res_base + VPU_RELEASE_ADDR_OFFSET);

	check_resource_status(res_status_addr, VPU_DEC_RESID, &res_status);
	if (resource_free == res_status) {
		pr_err("dec: res status = %d is already free\n", res_status);
		goto end;
	}
	iowrite32(1 << VPU_DEC_RESID, (void *)res_release_addr);
	check_resource_status(res_status_addr, VPU_DEC_RESID, &res_status);
	if (resource_free != res_status) {
		pr_err("dec: res release failed, status = %d\n", res_status);
		ret = -1;
		goto end;
	}
	pr_info("dec: res release success, status = %d\n", res_status);

end:
	iounmap(dec_res_base);
	release_mem_region(VPU_DEVMGR_BASE, 4 * 3);
	return ret;
}

int dec_dev_alloc(void)
{
	int ret = 0;
	enum device_mgr_resource_status res_status;
	char *dec_res_base = NULL;
	void *res_status_addr = NULL;
	void *res_alloc_addr = NULL;

	if (!request_mem_region(VPU_DEVMGR_BASE, 4 * 3, "vpudecres")) {
		pr_err("dec: res alloc request_mem_region regs fail\n");
		return -EBUSY;
	}

	dec_res_base = (char *)ioremap(VPU_DEVMGR_BASE, 4 * 3);
	res_status_addr = (void *)(dec_res_base + VPU_STATUS_ADDR_OFFSET);
	res_alloc_addr = (void *)(dec_res_base + VPU_ALLOCATE_ADDR_OFFSET);

	check_resource_status(res_status_addr, VPU_DEC_RESID, &res_status);
	if (resource_free != res_status) {
		pr_err("dec: res status = %d is not free\n", res_status);
		ret = -1;
		goto end;
	}
	iowrite32(1 << VPU_DEC_RESID, res_alloc_addr);
	check_resource_status(res_status_addr, VPU_DEC_RESID, &res_status);
	if (resource_free == res_status) {
		pr_err("dec: alloc res failed, status = %d\n", res_status);
		ret = -1;
		goto end;
	}
	pr_info("dec: alloc res success, status = %d\n", res_status);

end:
	iounmap(dec_res_base);
	release_mem_region(VPU_DEVMGR_BASE, 4 * 3);
	return ret;
}
