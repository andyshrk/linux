/*
 *
 *    The GPL License (GPL)
 *
 *    Copyright (C) 2014 - 2021 VERISILICON
 *
 *    This program is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU General Public License
 *    as published by the Free Software Foundation; either version 2
 *    of the License, or (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software Foundation,
 *    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#ifndef _HANTRO_ENC_H_
#define _HANTRO_ENC_H_
#include <linux/types.h>
#include <linux/ioctl.h>

/*
 * Macros to help debugging
 */

#undef PDEBUG
#ifdef HANTROENC_DEBUG
#  define PDEBUG(fmt, args...) pr_debug("hantroenc: " fmt, ## args)
#else
#  define PDEBUG(fmt, args...)
#endif

#define ENC_HW_ID1			0x48320100
#define ENC_HW_ID2			0x80006000
#define CORE_INFO_AMOUNT_OFFSET		28

struct dma_buf_desc {
	int fd;			/* dma_buf fd */
	__u64 dma_addr;		/* dma address */
};

/* Use 'k' as magic number */
#define HANTRO_IOC_MAGIC  'k'

/*
 * S means "Set" through a ptr,
 * T means "Tell" directly with the argument value
 * G means "Get": reply by setting through a pointer
 * Q means "Query": response is on the return value
 * X means "eXchange": G and S atomically
 * H means "sHift": T and Q atomically
 */

#define HANTRO_IOCG_HWOFFSET	_IOR(HANTRO_IOC_MAGIC,  3, unsigned long *)
#define HANTRO_IOCG_HWIOSIZE	_IOR(HANTRO_IOC_MAGIC,  4, unsigned int *)
#define HANTRO_IOCG_SRAMOFFSET	_IOR(HANTRO_IOC_MAGIC,  9, unsigned long *)
#define HANTRO_IOCG_SRAMEIOSIZE	_IOR(HANTRO_IOC_MAGIC, 10, unsigned int *)
#define HANTRO_IOCH_ENC_RESERVE	_IOR(HANTRO_IOC_MAGIC, 11, unsigned int *)
#define HANTRO_IOCH_ENC_RELEASE	_IOR(HANTRO_IOC_MAGIC, 12, unsigned int *)
#define HANTRO_IOCG_CORE_NUM	_IOR(HANTRO_IOC_MAGIC, 13, unsigned int *)
#define HANTRO_IOCG_CORE_INFO	_IOR(HANTRO_IOC_MAGIC, 14, \
						struct subsys_core_info *)
#define HANTRO_IOCG_CORE_WAIT	_IOR(HANTRO_IOC_MAGIC, 15, unsigned int *)
#define HANTRO_IOCG_ANYCORE_WAIT	_IOR(HANTRO_IOC_MAGIC,  16, \
						struct core_wait_out *)

#define HANTRO_IOCX_DMA_BUF_ATTACH	_IOWR(HANTRO_IOC_MAGIC, 34, \
						struct dma_buf_desc *)
#define HANTRO_IOCX_DMA_BUF_DETACH	_IOWR(HANTRO_IOC_MAGIC, 35, \
						struct dma_buf_desc *)


#define GET_ENCODER_IDX(type_info)	(CORE_VC8000E)
#define HANTRO_IOC_MAXNR		60

#ifdef SUPPORT_SFBC
#define HANTRO_SFBCDEC_REG_BASE_OFFSET	0x10000
#define HANTRO_SFBCDEC_IRQ_RAW_STATUS	0x4
#define HANTRO_SFBCDEC_COMMAND			0x14
#endif

#define ASIC_STATUS_SEGMENT_READY		0x1000
#define ASIC_STATUS_FUSE_ERROR			0x200
#define ASIC_STATUS_SLICE_READY			0x100
#define ASIC_STATUS_LINE_BUFFER_DONE		0x080  /* low latency */
#define ASIC_STATUS_HW_TIMEOUT			0x040
#define ASIC_STATUS_BUFF_FULL			0x020
#define ASIC_STATUS_HW_RESET			0x010
#define ASIC_STATUS_ERROR			0x008
#define ASIC_STATUS_FRAME_READY			0x004

#define ASIC_STATUS_ALL       (ASIC_STATUS_SEGMENT_READY |\
			       ASIC_STATUS_FUSE_ERROR |\
			       ASIC_STATUS_SLICE_READY |\
			       ASIC_STATUS_LINE_BUFFER_DONE |\
			       ASIC_STATUS_HW_TIMEOUT |\
			       ASIC_STATUS_BUFF_FULL |\
			       ASIC_STATUS_HW_RESET |\
			       ASIC_STATUS_ERROR |\
			       ASIC_STATUS_FRAME_READY)

enum {
	CORE_VC8000E	= 0,
	CORE_VC8000EJ	= 1,
	CORE_CUTREE	= 2,
	CORE_L2CACHE	= 5,
	CORE_AXIFE	= 6,
	CORE_APBFT	= 7,
	CORE_AXIFE_1	= 9,
	CORE_MAX
};

/* module_type support */
struct core_wait_out {
	u32 job_id[4];
	u32 irq_status[4];
	u32 irq_num;
};

struct subsys_core_info {
	/*
	 * indicate which IP is contained in this subsystem
	 * and each uses one bit of this variable
	 */
	u32 type_info;
	u64 offset[CORE_MAX];
	u64 regSize[CORE_MAX];
	int irq[CORE_MAX];
};

#endif /* _HANTRO_ENC_H_ */
