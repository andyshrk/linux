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

#include "se_dptx.h"

#if 0
static int print_buf(u8 *buf, int len)
{
	int i;
#define PRINT_BUF_SIZE 1024
	char str[PRINT_BUF_SIZE];
	int written = 0;

	written += snprintf(&str[written], PRINT_BUF_SIZE - written, "Buffer:");

	for (i = 0; i < len; i++) {
		if (!(i % 16)) {
			written += snprintf(&str[written],
				PRINT_BUF_SIZE - written,
				"\n%04x:", i);

			if (written >= PRINT_BUF_SIZE)
				break;

		}

		written += snprintf(&str[written],
			PRINT_BUF_SIZE - written,
			" %02x", buf[i]);

		if (written >= PRINT_BUF_SIZE)
			break;
	}

	printf("%s\n\n", str);

	return 0;
}
#endif

int dptx_read_edid_block(void *data, u8 *buf, unsigned int block, size_t len)
{
	int retval;
	int retry = 0;
	u8 offset = block * DPTX_DEFAULT_EDID_BUFLEN;
	u8 segment = block >> 1;
	struct dptx *dptx = data;

again:
	retval = dptx_write_bytes_to_i2c(dptx, 0x30, &segment, 1);
	/* TODO Skip if no E-DDC */
	retval = dptx_write_bytes_to_i2c(dptx, 0x50, &offset, 1);

	retval = dptx_read_bytes_from_i2c(dptx, 0x50, buf, len);
	if ((retval == -EINVAL) && !retry) {
		retry = 1;
		goto again;
	}

	return 0;
}

int dptx_read_edid(struct dptx *dptx)
{
	int i;
	int retval = 0;
	unsigned int ext_blocks = 0;
	u8 *new = NULL;

	memset(dptx->edid, 0, DPTX_DEFAULT_EDID_BUFLEN);
	retval = dptx_read_edid_block(dptx,
		&dptx->edid[0], 0, DPTX_DEFAULT_EDID_BUFLEN);
	if (retval)
		goto fail;

	if (dptx->edid[126] > 10){
		/* Workaround for QD equipment */
		/* TODO investigate corruptions of EDID blocks */
		ext_blocks = 2;
		dptx_dbg(dptx, "%s: num_ext_blocks=%d\n",  __func__, dptx->edid[126]);
	} else {
		ext_blocks = dptx->edid[126];
		dptx_dbg(dptx, "%s: num_ext_blocks=%d\n",  __func__, dptx->edid[126]);
	}

	new = krealloc(dptx->edid, (ext_blocks + 1) * DPTX_DEFAULT_EDID_BUFLEN, GFP_KERNEL);
	if (!new)
		goto fail;
	dptx->edid = new;

	for (i = 1; i <= ext_blocks; i++) {
		retval = dptx_read_edid_block(dptx,
			&dptx->edid[i * DPTX_DEFAULT_EDID_BUFLEN], i, DPTX_DEFAULT_EDID_BUFLEN);
		if (retval)
			goto fail;
	}

fail:
	return retval;
}

int dptx_check_edid(struct dptx *dptx)
{
	int i;
	u32 edid_sum = 0;

	for (i = 0; i < 128; i++)
		edid_sum += dptx->edid[i];
	if (edid_sum & 0xFF) {
		dptx_err(dptx, "Invalid EDID checksum\n");
		return -EINVAL;
	}
	return 0;
}

void dptx_clear_vcpid_table(struct dptx *dptx)
{
	int i;

	for (i = 0; i < 8; i++)
		dptx_writel(dptx, DPTX_MST_VCP_TABLE_REG_N(i), 0);
}

static void dptx_set_vcpid_table_slot(struct dptx *dptx,
				u32 slot, u32 stream)
{
	u32 offset;
	u32 reg;
	u32 lsb;
	u32 mask;

	if (slot > 63) {
		dptx_warn(dptx, "Invalid slot number > 63");
		return;
	}
	offset = DPTX_MST_VCP_TABLE_REG_N(slot >> 3);
	reg = dptx_readl(dptx, offset);

	lsb = (slot & 0x7) * 4;
	mask = GENMASK(lsb + 3, lsb);

	reg &= ~mask;
	reg |= (stream << lsb) & mask;

	dptx_dbg(dptx, "Writing 0x%08x val=0x%08x", offset, reg);
	dptx_writel(dptx, offset, reg);
}

void dptx_set_vcpid_table_range(struct dptx *dptx,
				u32 start, u32 count, u32 stream)
{
	int i;

	if ((start + count) > 64) {
		dptx_warn(dptx, "Invalid slot number > 63");
		return;
	}

	for (i = 0; i < count; i++) {
		dptx_dbg(dptx, "---------  setting slot %d for stream %d",
			start + i, stream);
		dptx_set_vcpid_table_slot(dptx, start + i, stream);
	}
}

static int dptx_initiate_mst_act(struct dptx *dptx)
{
	u32 reg;
	int count = 0, retry = 0;

	reg = dptx_readl(dptx, DPTX_CCTL);
	reg |= DPTX_CCTL_INITIATE_MST_ACT;
	dptx_writel(dptx, DPTX_CCTL, reg);

	while (1) {
		dptx_dbg(dptx, "wait CCTL.ACT \n");
		reg = dptx_readl(dptx, DPTX_CCTL);
		if (!(reg & DPTX_CCTL_INITIATE_MST_ACT))
			break;

		count++;
		if (count > 10) {
			dptx_warn(dptx, "CCTL.ACT timeout\n");
			retry++;
			if (retry > 3)
				return -1;
			reg = dptx_readl(dptx, DPTX_CCTL);
			reg &= ~DPTX_CCTL_INITIATE_MST_ACT;
			dptx_writel(dptx, DPTX_CCTL, reg);

			dptx_warn(dptx, "restart an ACT sequence\n");
			reg = dptx_readl(dptx, DPTX_CCTL);
			reg |= DPTX_CCTL_INITIATE_MST_ACT;
			dptx_writel(dptx, DPTX_CCTL, reg);

			count = 0;
		}
		udelay(10);
	}

	return 0;
}

void dptx_dpcd_clear_vcpid_table(struct dptx *dptx)
{
	u8 bytes[] = { 0x00, 0x00, 0x3f };
	u8 status;
	int count = 0;

	dptx_read_dpcd(dptx, DP_PAYLOAD_TABLE_UPDATE_STATUS, &status);
	dptx_write_dpcd(dptx, DP_PAYLOAD_TABLE_UPDATE_STATUS, DP_PAYLOAD_TABLE_UPDATED);
	dptx_read_dpcd(dptx, DP_PAYLOAD_TABLE_UPDATE_STATUS, &status);

	dptx_write_bytes_to_dpcd(dptx, DP_PAYLOAD_ALLOCATE_SET, bytes, 3);

	status = 0;
	while (!(status & DP_PAYLOAD_TABLE_UPDATED)) {
		dptx_read_dpcd(dptx, DP_PAYLOAD_TABLE_UPDATE_STATUS, &status);
		count++;
		if (count > 2000) {
			dptx_warn(dptx, "Timeout waiting for DPCD VCPID table update\n");
			break;
		}
		udelay(1000);
	}

	dptx_write_dpcd(dptx, DP_PAYLOAD_TABLE_UPDATE_STATUS, DP_PAYLOAD_TABLE_UPDATED);
	dptx_read_dpcd(dptx, DP_PAYLOAD_TABLE_UPDATE_STATUS, &status);
}

void dptx_dpcd_set_vcpid_table(struct dptx *dptx,
				u32 start, u32 count, u32 stream)
{
	u8 bytes[3];
	u8 status;
	int tries = 0;

	bytes[0] = stream;
	bytes[1] = start;
	bytes[2] = count;

	dptx_write_bytes_to_dpcd(dptx, DP_PAYLOAD_ALLOCATE_SET, bytes, 3);

	while (!(status & DP_PAYLOAD_TABLE_UPDATED)) {
		dptx_read_dpcd(dptx, DP_PAYLOAD_TABLE_UPDATE_STATUS, &status);
		tries++;
		if (tries > 100) {
			dptx_warn(dptx, "Timeout waiting for DPCD VCPID table update");
			break;
		}
	}

	dptx_write_dpcd(dptx, DP_PAYLOAD_TABLE_UPDATE_STATUS, DP_PAYLOAD_TABLE_UPDATED);
	dptx_read_dpcd(dptx, DP_PAYLOAD_TABLE_UPDATE_STATUS, &status);
}

#if 0
void dptx_print_vcpid_table(struct dptx *dptx)
{
	u8 bytes[64] = { 0 };

	dptx_read_bytes_from_dpcd(dptx, DP_PAYLOAD_TABLE_UPDATE_STATUS, bytes, 64);
	print_buf(bytes, 64);
}
#endif

#ifdef UNUSE_DRM
/**
 * div64_u64_rem - unsigned 64bit divide with 64bit divisor and remainder
 * @dividend: unsigned 64bit dividend
 * @divisor: unsigned 64bit divisor
 * @remainder: pointer to unsigned 64bit remainder
 *
 * Return: sets ``*remainder``, then returns dividend / divisor
 */
static inline u64 div64_u64_rem(u64 dividend, u64 divisor, u64 *remainder)
{
	*remainder = dividend % divisor;
	return dividend / divisor;
}
static inline int drm_fixp2int(s64 a)
{
	return ((s64)a) >> DRM_FIXED_POINT;
}
static inline int drm_fixp2int_ceil(s64 a)
{
	if (a > 0)
		return drm_fixp2int(a + DRM_FIXED_ALMOST_ONE);
	else
		return drm_fixp2int(a - DRM_FIXED_ALMOST_ONE);
}
static inline s64 drm_fixp_from_fraction(s64 a, s64 b)
{
	s64 res;
	bool a_neg = a < 0;
	bool b_neg = b < 0;
	u64 a_abs = a_neg ? -a : a;
	u64 b_abs = b_neg ? -b : b;
	u64 rem;
	/* determine integer part */
	u64 res_abs = div64_u64_rem(a_abs, b_abs, &rem);
	/* determine fractional part */
	{
		u32 i = DRM_FIXED_POINT;
		do {
			rem <<= 1;
			res_abs <<= 1;
			if (rem >= b_abs) {
				res_abs |= 1;
				rem -= b_abs;
			}
		} while (--i != 0);
	}
	/* round up LSB */
	{
		u64 summand = (rem << 1) >= b_abs;
		res_abs += summand;
	}
	res = (s64) res_abs;
	if (a_neg ^ b_neg)
		res = -res;
	return res;
}
/**
 * drm_dp_calc_pbn_mode() - Calculate the PBN for a mode.
 * @clock: dot clock for the mode
 * @bpp: bpp for the mode.
 *
 * This uses the formula in the spec to calculate the PBN value for a mode.
 */
static int drm_dp_calc_pbn_mode(int clock, int bpp)
{
	u64 kbps;
	s64 peak_kbps;
	u32 numerator;
	u32 denominator;
	kbps = clock * bpp;
	/*
	 * margin 5300ppm + 300ppm ~ 0.6% as per spec, factor is 1.006
	 * The unit of 54/64Mbytes/sec is an arbitrary unit chosen based on
	 * common multiplier to render an integer PBN for all link rate/lane
	 * counts combinations
	 * calculate
	 * peak_kbps *= (1006/1000)
	 * peak_kbps *= (64/54)
	 * peak_kbps *= 8    convert to bytes
	 */
	numerator = 64 * 1006;
	denominator = 54 * 8 * 1000 * 1000;
	kbps *= numerator;
	peak_kbps = drm_fixp_from_fraction(kbps, denominator);
	return drm_fixp2int_ceil(peak_kbps);
}
#endif

u32 dptx_calc_num_slots(struct dptx *dptx, int pbn)
{
	int div;
	int dp_link_bw = dptx_phy_rate_to_bw(dptx->link.rate);
	int dp_link_count = dptx->link.lanes;

	switch (dp_link_bw) {
	case DP_LINK_BW_1_62:
		div = 3 * dp_link_count;
		break;
	case DP_LINK_BW_2_7:
		div = 5 * dp_link_count;
		break;
	case DP_LINK_BW_5_4:
		div = 10 * dp_link_count;
		break;
	case DP_LINK_BW_8_1:
		div = 15 * dp_link_count;
		break;
	default:
		return 0;
	}

	return DIV_ROUND_UP(pbn, div);
}

void dptx_vcpid_init(struct dptx *dptx)
{
	int i;
	u32 slot_start = 1, pipe, pxlclk;
	struct dptx_pipeline *ppl;

	mdelay(1);

	dptx_dpcd_clear_vcpid_table(dptx);
	dptx_clear_vcpid_table(dptx);

	for (i = 0; i < dptx->streams; ++i) {
		pipe = dptx->stream_id[i];
		ppl = &dptx->ppls[pipe];
	#ifdef UNUSE_DRM
		pxlclk = ppl->dt.pixelclock.typ / 1000;
		ppl->pbn = (u16)drm_dp_calc_pbn_mode(pxlclk, ppl->vparams.bpc * 3);
	#else
		pxlclk = ppl->mode.clock;
		ppl->pbn = (u16)drm_dp_calc_pbn_mode(pxlclk, ppl->vparams.bpc * 3, dptx->dsc);
	#endif
		ppl->slots = dptx_calc_num_slots(dptx, ppl->pbn);
		dptx_info(dptx, "stream %d NUM SLOTS = %d", pipe, ppl->slots);

		dptx_set_vcpid_table_range(dptx, slot_start, ppl->slots, pipe + 1);
		dptx_dpcd_set_vcpid_table(dptx, slot_start, ppl->slots, pipe + 1);
		slot_start += ppl->slots;
	}

	//dptx_print_vcpid_table(dptx);
}

int dptx_set_xmit_act(struct dptx *dptx)
{
	int ret;

	ret = dptx_initiate_mst_act(dptx);
	if (ret)
		return ret;

	{
		int tries = 0;
		u8 status = 0;

		while (!(status & DP_PAYLOAD_ACT_HANDLED)) {
			dptx_read_dpcd(dptx, DP_PAYLOAD_TABLE_UPDATE_STATUS, &status);
			tries++;
			if (tries > 100) {
				dptx_err(dptx, "Timeout waiting for ACT_HANDLED\n");
				return -1;
			}
			//mdelay(20);
		}

		dptx_write_dpcd(dptx, DP_PAYLOAD_TABLE_UPDATE_STATUS, 0x3);
		dptx_read_dpcd(dptx, DP_PAYLOAD_TABLE_UPDATE_STATUS, &status);
	}

	return 0;
}

int dptx_en_fec(struct dptx *dptx, u8 enable)
{
	int retval;
	u32 reg;
	u8 result;

	if (!enable) {
		dptx->fec = true;
	}
	else {
		dptx->fec = false;
	}

	reg = dptx_readl(dptx, DPTX_CCTL);
	reg |= DPTX_CCTL_ENH_FRAME_FEC_EN;
	dptx_writel(dptx, DPTX_CCTL, reg);

	// Set FEC_READY on the sink side
	retval = dptx_write_dpcd(dptx, DP_FEC_CONFIGURATION, DP_FEC_READY);
	  if (retval)
		return retval;

	retval = dptx_link_training(dptx, dptx->max_rate, dptx->max_lanes);
	if (retval)
	        return retval;

	udelay(1000);

	// Enable forward error correction
	reg = dptx_readl(dptx, DPTX_CCTL);
	reg |= DPTX_CCTL_ENABLE_FEC;
	dptx_writel(dptx, DPTX_CCTL, reg);

	dptx_dbg(dptx, "%s: Enabling Forward Error Correction\n", __func__);

	retval = dptx_read_dpcd(dptx, 0x280, &result);
	if (retval)
		dptx_dbg(dptx, "DPCD read failed\n");
	dptx_dbg(dptx,"fec status = %x\n", result);

	retval = dptx_read_dpcd(dptx, 0x281, &result);
	if (retval)
		dptx_dbg(dptx, "DPCD read failed\n");

	dptx_dbg(dptx,"fec error count %x\n", result);

    return 0;
}
