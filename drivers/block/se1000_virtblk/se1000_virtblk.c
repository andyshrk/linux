/*
 * Siengine Virtual Block device driver
 *
 * Copyright (c) 2018- Siengine Technologies CO., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "se1000_virtblk.h"


void  sv_blk_flush_dcache_area(void *kaddr, size_t len)
{
	__flush_dcache_area(kaddr, len);
}
EXPORT_SYMBOL(sv_blk_flush_dcache_area);

void  sv_blk_inval_dcache_area(void *kaddr, size_t len)
{
	__inval_dcache_area(kaddr, len);
}
EXPORT_SYMBOL(sv_blk_inval_dcache_area);
