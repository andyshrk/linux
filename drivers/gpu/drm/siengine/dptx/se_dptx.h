/*
 * Copyright (c) 2016 Synopsys, Inc.
 *
 * Synopsys DP TX Linux Software Driver and documentation (hereinafter,
 * "Software") is an Unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef __DPTX_DRIVER_H__
#define __DPTX_DRIVER_H__

#include <linux/types.h>
#include <asm/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <drm/drm_fixed.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_dp_mst_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_encoder.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_panel.h>
#include <linux/version.h>
#include <linux/regmap.h>
#include "avgen.h"
#include "reg.h"
#include "dbg.h"

// Rounding up to the nearest multiple of a number
#define ROUND_UP_TO_NEAREST(numToRound, mult) ((((numToRound+(mult)-1) / (mult)) * (mult)))

#define MAX_DPU_NUM					2
#define DPTX_MAX_STREAM_NUM			4
//aux clk 16M
#define DP_AUX16MHZ_CLK_FREQ		16000000

/* The max rate and lanes supported by the core */
#define DPTX_MAX_LINK_RATE			DPTX_PHYIF_CTRL_RATE_HBR3
#define DPTX_MAX_LINK_LANES			4

/* The default rate and lanes to use for link training */
#define DPTX_DEFAULT_LINK_RATE		DPTX_MAX_LINK_RATE
#define DPTX_DEFAULT_LINK_LANES		DPTX_MAX_LINK_LANES

#define DPTX_DEFAULT_COLOR_DEPTH	COLOR_DEPTH_8

#define DPTX_SDP_NUM				0x10
#define DPTX_SDP_LEN				0x9
#define DPTX_SDP_SIZE				(9 * 4)

//DSC definitions coming from RC files of DSC C model
#define RC_MODEL_SIZE					8192
#define INITIAL_OFFSET					6144
#define INITIAL_DELAY					512
#define PIXELS_PER_GROUP				3
#define PIXEL_HOLD_DELAY				5
#define PIXEL_GROUP_DELAY				3
#define MUXER_INITIAL_BUFFERING_DELAY	9
#define DSC_MAX_AUX_ORIG_WIDTH			24
#define PIXEL_FLATNESSBUF_DELAY			DSC_MAX_AUX_ORIG_WIDTH
#define FLATNESS_MIN_QP					3
#define FLATNESS_MAX_QP					12
#define RC_EDGE_FACTOR					6
#define RC_QUANT_INCR_LIMIT0			11
#define RC_QUANT_INCR_LIMIT1			11
#define RC_TGT_OFFSET_HIGH				3
#define RC_TGT_OFFSET_LOW				3
#define INITIAL_SCALE_VALUE_SHIFT		3
#define OFFSET_FRACTIONAL_BITS			11

#define DPTX_DEFAULT_EDID_BUFLEN		128

struct dptx;

struct rc_range_param {
	int minQP	:5;
	int maxQP	:5;
	int offset	:6;
};

/**
 * struct dptx_link - The link state.
 * @status: Holds the sink status register values.
 * @trained: True if the link is successfully trained.
 * @rate: The current rate that the link is trained at.
 * @lanes: The current number of lanes that the link is trained at.
 * @preemp_level: The pre-emphasis level used for each lane.
 * @vswing_level: The vswing level used for each lane.
 */
struct dptx_link {
	u8 status[DP_LINK_STATUS_SIZE];
	bool trained;
	u8 rate;
	u8 lanes;
	u8 preemp_level[DPTX_MAX_LINK_LANES];
	u8 vswing_level[DPTX_MAX_LINK_LANES];
};

/**
 * struct dptx_aux - The aux state
 * @sts: The AUXSTS register contents.
 * @data: The AUX data register contents.
 * @abort: Indicates that the AUX transfer should be aborted.
 */
struct dptx_aux {
	u32 sts;
	u32 data[4];
	int abort;
};

struct dptx_eq_setting {
	u8 vswing_level;
	u8 preemp_level;
	u8 tx_eq_main;
	u8 tx_eq_post;
	u8 tx_eq_pre;
};

struct dptx_encoder {
	struct drm_encoder base;
	int pipe;
	struct dptx *dptx;
};

struct dptx_connector {
	struct drm_connector base;
	int pipe;
	struct dptx *dptx;
};

struct dptx_pipeline {
	struct dptx *dptx;
	struct clk *aclk;
	struct clk *pxlclk;
	struct dptx_encoder enc;
	struct dptx_connector conn;
	struct drm_panel *panel;
#ifdef UNUSE_DRM
	struct display_timing dt;
#else
	struct drm_display_mode mode;
#endif
	u16 pbn;
	u32 slots;
	int status;
	bool bind;
	bool enabled;
	struct video_params vparams;
	struct device_node *of_node; /* pipeline dt node */
	struct edid *edid4drm;
};

/**
 * struct dptx - The representation of the DP TX core
 * @lock: dptx mutex
 * @base: Base address of the registers
 * @irq: IRQ number
 * @version: Contents of the IP_VERSION register
 * @max_rate: The maximum rate that the controller supports
 * @max_lanes: The maximum lane count that the controller supports
 * @dev: The struct device
 * @root: The debugfs root
 * @regset: The debugfs regset
 * @rx_caps: The sink's receiver capabilities.
 * @edid: The sink's EDID.
 * @aux: AUX channel state for performing an AUX transfer.
 * @link: The current link state.
 * @multipixel: Controls multipixel configuration. 0-Single, 1-Dual, 2-Quad.
 */
struct dptx {
	struct drm_device *drm_dev;
	struct dptx_pipeline ppls[DPTX_MAX_STREAM_NUM];
	struct regmap* misc_base;
	int stream_id[DPTX_MAX_STREAM_NUM];

	struct mutex lock; /* generic mutex for dptx */
	struct clk *apb_clk;
	struct clk *aux_clk;

	struct {
		u8 multipixel;
		u8 streams;
		bool gen2phy;
		bool dsc;
	} hwparams;

	void __iomem *base;
	int irq;

	u32 version;

	u8 fixed_rate;
	u8 fixed_lanes;
	u8 max_rate;
	u8 max_lanes;
	u8 streams;
	u8 total_streams;

	bool dpu_used[MAX_DPU_NUM];
	bool bind_flag[MAX_DPU_NUM];

	bool display_timings_fixed;
	bool no_hpd_irq;

	bool mst;
	bool cr_fail;

	u8 multipixel;

	bool ssc_en;
	bool fec;
	bool dsc;
	bool automated_test;
	bool suspended;
	atomic_t hpd;
	atomic_t connect;
	atomic_t sink_request;

	struct device *dev;
	struct dentry *root;
	struct debugfs_regset32 *regset;

	u8 rx_caps[DP_RECEIVER_CAP_SIZE];

	u8 *edid;

	struct dptx_aux aux;
	struct dptx_link link;

	bool tx_eq_en;
	struct dptx_eq_setting eq_setting;
};

/*
 * Core interface functions
 */
#define REG_DUMP(dptx, offset) \
		dptx_info(dptx, "%s:%d: DUMP: %s addr=0x%05x data=0x%08x\n", \
				__func__, __LINE__, #offset, offset, readl(dptx->base + offset))

int dptx_core_init(struct dptx *dptx);
void dptx_init_hwparams(struct dptx *dptx);
bool dptx_check_dptx_id(struct dptx *dptx);
void dptx_core_init_phy(struct dptx *dptx);
int dptx_core_program_ssc(struct dptx *dptx, bool sink_ssc);
bool dptx_sink_enabled_ssc(struct dptx *dptx);
void dptx_enable_ssc(struct dptx *dptx);

int dptx_core_deinit(struct dptx *dptx);
void dptx_soft_reset(struct dptx *dptx, u32 bits);
void dptx_soft_reset_all(struct dptx *dptx);
void dptx_phy_soft_reset(struct dptx *dptx);

void dptx_global_intr_en(struct dptx *dp);
void dptx_global_intr_dis(struct dptx *dp);
void dptx_dsc_enable(struct dptx* dptx);
void dptx_hpd_intr_en(struct dptx *dptx);

int dptx_get_sink_cap_by_dpcd(struct dptx *dptx);
void dptx_mst_en(struct dptx *dptx);
void dptx_mst_dis(struct dptx *dptx);
void dptx_disable_fast_link_training(struct dptx *dptx);

int dptx_bw_to_link_rate(unsigned int bw);
void dptx_aux_misc_init(struct dptx *dptx);
void dptx_phy_reset_init(struct dptx *dptx);

/*
 * PHY IF Control
 */
void dptx_phy_set_lanes(struct dptx *dptx, unsigned int num);
unsigned int dptx_phy_get_lanes(struct dptx *dptx);
void dptx_phy_set_rate(struct dptx *dptx, unsigned int rate);
unsigned int dwc_phy_get_rate(struct dptx *dptx);
int dptx_phy_wait_busy(struct dptx *dptx, unsigned int lanes);
void dptx_phy_set_pre_emphasis(struct dptx *dptx,
			unsigned int lane,
			unsigned int level);
void dptx_phy_set_vswing(struct dptx *dptx,
			unsigned int lane,
			unsigned int level);
void dptx_phy_set_pattern(struct dptx *dptx,
			unsigned int pattern);
void dptx_phy_enable_xmit(struct dptx *dptx, unsigned int lane, bool enable);

int dptx_phy_rate_to_bw(unsigned int rate);
int dptx_bw_to_phy_rate(unsigned int bw);

/*
 * AUX Channel
 */

#define DPTX_AUX_TIMEOUT 2000

int dptx_aux_rw_bytes(struct dptx *dptx,
			bool rw, bool i2c, u32 addr,
			u8 *bytes, unsigned int len);

int dptx_read_bytes_from_i2c(struct dptx *dptx,
			unsigned int device_addr,
			u8 *bytes, u32 len);

int dptx_i2c_address_only(struct dptx *dptx,
			unsigned int device_addr);

int dptx_write_bytes_to_i2c(struct dptx *dptx,
			unsigned int device_addr,
			u8 *bytes, u32 len);

int __dptx_read_dpcd(struct dptx *dptx, u32 addr, u8 *byte);
int __dptx_write_dpcd(struct dptx *dptx, u32 addr, u8 byte);

int __dptx_read_bytes_from_dpcd(struct dptx *dptx,
			unsigned int reg_addr,
			u8 *bytes, u32 len);

int __dptx_write_bytes_to_dpcd(struct dptx *dptx,
			unsigned int reg_addr,
			u8 *bytes, u32 len);

#ifndef DPTX_DEBUG_DPCD_CMDS
static inline int dptx_read_dpcd(struct dptx *dptx, u32 addr, u8 *byte)
{
	return __dptx_read_dpcd(dptx, addr, byte);
}

static inline int dptx_write_dpcd(struct dptx *dptx, u32 addr, u8 byte)
{
	return __dptx_write_dpcd(dptx, addr, byte);
}

static inline int dptx_read_bytes_from_dpcd(struct dptx *dptx,
					    unsigned int reg_addr,
					    u8 *bytes, u32 len)
{
	return __dptx_read_bytes_from_dpcd(dptx, reg_addr, bytes, len);
}

static inline int dptx_write_bytes_to_dpcd(struct dptx *dptx,
					   unsigned int reg_addr,
					   u8 *bytes, u32 len)
{
	return __dptx_write_bytes_to_dpcd(dptx, reg_addr, bytes, len);
}

#else
#define dptx_read_dpcd(_dptx, _addr, _byteptr) ({		\
	int _ret = __dptx_read_dpcd(_dptx, _addr, _byteptr);	\
	dptx_dbg(dptx, "%s: DPCD Read %s(0x%03x) = 0x%02x\n",	\
		 __func__, #_addr, _addr, *(_byteptr));		\
	_ret;							\
})

#define dptx_write_dpcd(_dptx, _addr, _byte) ({			\
	int _ret;						\
	dptx_dbg(dptx, "%s: DPCD Write %s(0x%03x) = 0x%02x\n",	\
		 __func__, #_addr, _addr, _byte);		\
	_ret = __dptx_write_dpcd(_dptx, _addr, _byte);		\
	_ret;							\
})

char *__bytes_str(u8 *bytes, unsigned int n);

#define dptx_read_bytes_from_dpcd(_dptx, _addr, _b, _len) ({	\
	int _ret;						\
	char *_str;						\
	_ret = __dptx_read_bytes_from_dpcd(_dptx,		\
					   _addr, _b, _len);	\
	_str = __bytes_str(_b, _len);				\
	dptx_dbg(dptx, "%s: Read %llu bytes from %s(0x%02x) = [ %s ]\n", \
		 __func__, (u64)_len, #_addr, _addr, _str);		\
	_ret;							\
})

#define dptx_write_bytes_to_dpcd(_dptx, _addr, _b, _len) ({	\
	int _ret;						\
	char *_str = __bytes_str(_b, _len);			\
	dptx_dbg(dptx, "%s: Writing %llu bytes to %s(0x%02x) = [ %s ]\n", \
		 __func__, (u64)_len, #_addr, _addr, _str);		\
	_ret = __dptx_write_bytes_to_dpcd(_dptx, _addr, _b, _len); \
	_ret;							\
})

#endif

/*
 * Link training
 */
int dptx_link_training(struct dptx *dptx, u8 rate, u8 lanes);
bool dptx_link_check_status(struct dptx *dptx);
int dptx_link_adjust_drive_settings(struct dptx *dptx, int *out_changed);
void dptx_link_set_eq(struct dptx *dptx, u8 vs, u8 pe);

/*
 * Register read and write functions
 */
static inline u32 __dptx_readl(struct dptx *dp, char const *func, int line, u32 offset)
{
	u32 data = readl(dp->base + offset);
#ifdef DPTX_DEBUG_REG
	static u32 last_offset = 0;
	static u32 last_data = 0;
	bool print = false;

	if (offset != last_offset) {
		print = true;
	} else {
		if (data != last_data) {
			print = true;
		}
	}

	if (print) {
		dptx_dbg(dp, "%s:%d: READ: addr=0x%05x data=0x%08x\n", func, line, offset, data);
	} else {
		//dptx_dbg(dp, "%s: .\n", __func__);
	}

	last_offset = offset;
	last_data = data;
#endif

	return data;
}

static inline void __dptx_writel(struct dptx *dp, char const *func, int line, u32 offset, u32 data)
{
#ifdef DPTX_DEBUG_REG
	dptx_dbg(dp, "%s:%d: WRITE: addr=0x%05x data=0x%08x\n", func, line, offset, data);
#endif
	writel(data, dp->base + offset);
}


#define dptx_readl(_dptx, _offset) ({ \
	__dptx_readl(_dptx, __func__, __LINE__, _offset); \
})

#define dptx_writel(_dptx, _offset, _data) do { \
	__dptx_writel(_dptx, __func__, __LINE__, _offset, _data); \
} while(0)

/*
 * Wait functions
 */
#define dptx_wait(_dptx, _cond, _timeout)				\
	({								\
		int __retval;						\
		__retval = wait_event_interruptible_timeout(		\
			_dptx->waitq,					\
			((_cond) || (atomic_read(&_dptx->shutdown))),	\
			msecs_to_jiffies(_timeout));			\
		if (atomic_read(&_dptx->shutdown)) {			\
			__retval = -ESHUTDOWN;				\
		}							\
		else if (!__retval) {					\
			__retval = -ETIMEDOUT;				\
		}							\
		__retval;						\
	})

/*
 * debugfs functions
 */
void dptx_debugfs_init(struct dptx *dptx);
void dptx_debugfs_exit(struct dptx *dptx);

/*
 * irq functions
 */
int dptx_handle_hotplug(struct dptx *dptx);
int dptx_handle_hotunplug(struct dptx *dptx);

irqreturn_t dptx_irq(int irq, void *dev);
irqreturn_t dptx_threaded_irq(int irq, void *dev);

int dptx_read_edid_block(void *data, u8 *buf, unsigned int block, size_t len);
int dptx_read_edid(struct dptx *dptx);

void dptx_vcpid_init(struct dptx *dptx);
int dptx_set_xmit_act(struct dptx *dptx);
void dptx_clear_vcpid_table(struct dptx *dptx);

int dptx_en_fec(struct dptx *dptx, u8 enable);

#endif
