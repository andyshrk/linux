/*
 *
 *    The GPL License (GPL)
 *
 *    Copyright (C) 2014 - 2021 VERISILICON
 *    Copyright (C) 2022 - 2023 Siengine
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

#ifndef _HANTRO_DEC_H_
#define _HANTRO_DEC_H_
#include <linux/ioctl.h>
#include <linux/types.h>

#define MAX_SUBSYS_NUM	4   /* up to 4 subsystem (temporary) */
#define HXDEC_MAX_CORES	MAX_SUBSYS_NUM

enum CoreType {
	/* Decoder */
	HW_VC8000D = 0,
	HW_VC8000DJ,
	HW_BIGOCEAN,
	HW_L2CACHE,
	HW_SHAPER,
	/* Encoder */
	/* Auxiliary IPs */
	HW_NOC,
	HW_AXIFE,
	HW_APBFILTER,
#ifdef SUPPORT_SFBC
	HW_SFBC_ENC,
#endif
	HW_CORE_MAX	/* max number of cores supported */
};

struct core_desc {
	__u32 id;	/* id of the subsystem */
	__u32 type;	/* type of core to be written */
	__u64 regs_addr;/* user registers address, 32bit/64bit compatibility */
	__u32 size;	/* size of register space */
	__u32 reg_id;	/* id of reigster to be read/written */
};

struct regsize_desc {
	__u32 slice;	/* id of the slice */
	__u32 id;	/* id of the subsystem */
	__u32 type;	/* type of core to be written */
	__u32 size;	/* iosize of the core */
};

struct core_param {
	__u32 slice;	/* id of the slice */
	__u32 id;	/* id of the subsystem */
	__u32 type;	/* type of core to be written */
	__u32 size;	/* iosize of the core */
	__u32 asic_id;	/* asic id of the core */
};

struct subsys_desc {
	__u32 subsys_num;/* total subsystems count */
};

struct dma_buf_desc {
	__u32 fd;	/* dma_buf fd */
	__u64 dma_addr;	/* dma address */
};

struct SubsysDesc {
	int slice_index;/* slice this subsys belongs to */
	int index;	/* subsystem index */
	long base;
};

struct CoreDesc {
	int slice;
	int subsys;	/* subsys this core belongs to */
	enum CoreType core_type;
	int offset;	/* offset to subsystem base */
	int iosize;
	int irq;
};

/* internal config struct (translated from SubsysDesc & CoreDesc) */
struct subsys_config {
	unsigned long base_addr;
	int irq;
	u32 subsys_type;	/* identifier for each subsys */
	u32 submodule_offset[HW_CORE_MAX];	/* in bytes */
	u16 submodule_iosize[HW_CORE_MAX];	/* in bytes */
	u8 *submodule_hwregs[HW_CORE_MAX];	/* virtual address */
};

/* Use 'k' as magic number */
#define HANTRODEC_IOC_MAGIC  'k'

/*
 * S means "Set" through a ptr,
 * T means "Tell" directly with the argument value
 * G means "Get": reply by setting through a pointer
 * Q means "Query": response is on the return value
 * X means "eXchange": G and S atomically
 * H means "sHift": T and Q atomically
 */

#define HANTRODEC_PP_INSTANCE		_IO(HANTRODEC_IOC_MAGIC, 1)
#define HANTRODEC_HW_PERFORMANCE	_IO(HANTRODEC_IOC_MAGIC, 2)
#define HANTRODEC_IOCGHWOFFSET		_IOR(HANTRODEC_IOC_MAGIC,  3, \
							unsigned long *)
#define HANTRODEC_IOCGHWIOSIZE		_IOR(HANTRODEC_IOC_MAGIC,  4, \
							struct regsize_desc *)

#define HANTRODEC_IOC_CLI		_IO(HANTRODEC_IOC_MAGIC,  5)
#define HANTRODEC_IOC_STI		_IO(HANTRODEC_IOC_MAGIC,  6)
#define HANTRODEC_IOC_MC_OFFSETS	_IOR(HANTRODEC_IOC_MAGIC, 7, \
							unsigned long *)
#define HANTRODEC_IOC_MC_CORES		_IOR(HANTRODEC_IOC_MAGIC, 8, \
							unsigned int *)


#define HANTRODEC_IOCS_DEC_PUSH_REG	_IOW(HANTRODEC_IOC_MAGIC,  9, \
							struct core_desc *)
#define HANTRODEC_IOCS_PP_PUSH_REG	_IOW(HANTRODEC_IOC_MAGIC, 10, \
							struct core_desc *)

#define HANTRODEC_IOCH_DEC_RESERVE	_IO(HANTRODEC_IOC_MAGIC, 11)
#define HANTRODEC_IOCT_DEC_RELEASE	_IO(HANTRODEC_IOC_MAGIC, 12)
#define HANTRODEC_IOCQ_PP_RESERVE	_IO(HANTRODEC_IOC_MAGIC, 13)
#define HANTRODEC_IOCT_PP_RELEASE	_IO(HANTRODEC_IOC_MAGIC, 14)

#define HANTRODEC_IOCX_DEC_WAIT		_IOWR(HANTRODEC_IOC_MAGIC, 15, \
							struct core_desc *)
#define HANTRODEC_IOCX_PP_WAIT		_IOWR(HANTRODEC_IOC_MAGIC, 16, \
							struct core_desc *)

#define HANTRODEC_IOCS_DEC_PULL_REG	_IOWR(HANTRODEC_IOC_MAGIC, 17, \
							struct core_desc *)
#define HANTRODEC_IOCS_PP_PULL_REG	_IOWR(HANTRODEC_IOC_MAGIC, 18, \
							struct core_desc *)

#define HANTRODEC_IOCG_CORE_WAIT	_IOR(HANTRODEC_IOC_MAGIC, 19, __u32 *)

#define HANTRODEC_IOX_ASIC_ID		_IOWR(HANTRODEC_IOC_MAGIC, 20, \
							struct core_param *)

#define HANTRODEC_IOCG_CORE_ID		_IOR(HANTRODEC_IOC_MAGIC, 21, \
							unsigned long)

#define HANTRODEC_IOCS_DEC_WRITE_REG	_IOW(HANTRODEC_IOC_MAGIC, 22, \
							struct core_desc *)

#define HANTRODEC_IOCS_DEC_READ_REG	_IOWR(HANTRODEC_IOC_MAGIC, 23, \
							struct core_desc *)

#define HANTRODEC_IOX_ASIC_BUILD_ID	_IOWR(HANTRODEC_IOC_MAGIC, 24, __u32 *)

#define HANTRODEC_IOX_SUBSYS		_IOWR(HANTRODEC_IOC_MAGIC, 25, \
							struct subsys_desc *)

#define HANTRODEC_IOCX_POLL		_IO(HANTRODEC_IOC_MAGIC, 26)

#define HANTRODEC_IOCX_DMA_BUF_ATTACH	_IOWR(HANTRODEC_IOC_MAGIC, 27, \
							struct dma_buf_desc *)
#define HANTRODEC_IOCX_DMA_BUF_DETACH	_IOWR(HANTRODEC_IOC_MAGIC, 28, \
							struct dma_buf_desc *)

#define HANTRODEC_DEBUG_STATUS		_IO(HANTRODEC_IOC_MAGIC, 29)

#define HANTRODEC_IOC_MAXNR		29

#endif /* _HANTRO_DEC_H_ */
