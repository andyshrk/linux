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

static int dptx_handle_aux_reply(struct dptx *dptx)
{
	u32 auxsts;
	u32 status;
	u32 auxm;
	u32 br;
	int count;

	count = 0;
	while(1) {
		auxsts = dptx_readl(dptx, DPTX_AUX_STS);

		if (!(auxsts & DPTX_AUX_STS_REPLY_RECEIVED))
			break;

		count++;
		if (count > 5000)
			return -ETIMEDOUT;

		udelay(1);
	};

	auxsts = dptx_readl(dptx, DPTX_AUX_STS);

	status = (auxsts & DPTX_AUX_STS_STATUS_MASK) >>
		DPTX_AUX_STS_STATUS_SHIFT;

	auxm = (auxsts & DPTX_AUX_STS_AUXM_MASK) >>
		DPTX_AUX_STS_AUXM_SHIFT;
	br = (auxsts & DPTX_AUX_STS_BYTES_READ_MASK) >>
		DPTX_AUX_STS_BYTES_READ_SHIFT;

	dptx_dbg(dptx, "%s: 0x%08x: sts=%d, auxm=%d, br=%d, replyrcvd=%d, "
		"replyerr=%d, timeout=%d, disconn=%d\n",
		__func__, auxsts, status, auxm, br,
		!!(auxsts & DPTX_AUX_STS_REPLY_RECEIVED),
		!!(auxsts & DPTX_AUX_STS_REPLY_ERR),
		!!(auxsts & DPTX_AUX_STS_TIMEOUT),
		!!(auxsts & DPTX_AUX_STS_SINK_DWA));

	switch (status) {
	case DPTX_AUX_STS_STATUS_ACK:
		dptx_dbg(dptx, "%s: DPTX_AUX_STS_STATUS_ACK\n", __func__);
		break;
	case DPTX_AUX_STS_STATUS_NACK:
		dptx_dbg(dptx, "%s: DPTX_AUX_STS_STATUS_NACK\n", __func__);
		break;
	case DPTX_AUX_STS_STATUS_DEFER:
		dptx_dbg(dptx, "%s: DPTX_AUX_STS_STATUS_DEFER\n", __func__);
		break;
	case DPTX_AUX_STS_STATUS_I2C_NACK:
		dptx_dbg(dptx, "%s: DPTX_AUX_STS_STATUS_I2C_NACK\n",
			     __func__);
		break;
	case DPTX_AUX_STS_STATUS_I2C_DEFER:
		dptx_dbg(dptx, "%s: DPTX_AUX_STS_STATUS_I2C_DEFER\n",
			     __func__);
		break;
	default:
		dptx_err(dptx, "Invalid AUX status 0x%x\n", status);
		break;
	}

	dptx->aux.data[0] = dptx_readl(dptx, DPTX_AUX_DATA0);
	dptx->aux.data[1] = dptx_readl(dptx, DPTX_AUX_DATA1);
	dptx->aux.data[2] = dptx_readl(dptx, DPTX_AUX_DATA2);
	dptx->aux.data[3] = dptx_readl(dptx, DPTX_AUX_DATA3);
	dptx->aux.sts = auxsts;

	return 0;
}

static void dptx_aux_clear_data(struct dptx *dptx)
{
	dptx_writel(dptx, DPTX_AUX_DATA0, 0);
	dptx_writel(dptx, DPTX_AUX_DATA1, 0);
	dptx_writel(dptx, DPTX_AUX_DATA2, 0);
	dptx_writel(dptx, DPTX_AUX_DATA3, 0);
}

static int dptx_aux_read_data(struct dptx *dptx, u8 *bytes, unsigned int len)
{
	unsigned int i;

	u32 *data = dptx->aux.data;

	for (i = 0; i < len; i++)
		bytes[i] = (data[i / 4] >> ((i % 4) * 8)) & 0xff;

	return len;
}

static int dptx_aux_write_data(struct dptx *dptx, u8 const *bytes,
			       unsigned int len)
{
	unsigned int i;
	u32 data[4];

	memset(data, 0, sizeof(u32) * 4);

	for (i = 0; i < len; i++)
		data[i / 4] |= (bytes[i] << ((i % 4) * 8));

	dptx_writel(dptx, DPTX_AUX_DATA0, data[0]);
	dptx_writel(dptx, DPTX_AUX_DATA1, data[1]);
	dptx_writel(dptx, DPTX_AUX_DATA2, data[2]);
	dptx_writel(dptx, DPTX_AUX_DATA3, data[3]);

	return len;
}

static int dptx_aux_rw(struct dptx *dptx,
		       bool rw,
		       bool i2c,
		       bool mot,
		       bool addr_only,
		       u32 addr,
		       u8 *bytes,
		       unsigned int len)
{
	int retval;
	int tries = 0;
	u32 auxcmd;
	u32 type;
	unsigned int status;
	unsigned int br;

again:
	mdelay(1);
	tries++;

	if (tries > 20) {
		dptx_err(dptx, "AUX exceeded retries %s (0x%x)\n",  rw ? "r" : "w", addr);
		return -EINVAL;
	}

	dptx_dbg(dptx, "%s: addr=0x%08x, len=%d, try=%d\n",
		     __func__, addr, len, tries);

	if ((len > 16) || (len == 0)) {
		dptx_dbg(dptx, "AUX read/write len must be 1-15, len=%d\n", len);
		return -EINVAL;
	}
	type = rw ? DPTX_AUX_CMD_TYPE_READ : DPTX_AUX_CMD_TYPE_WRITE;

	if (!i2c)
		type |= DPTX_AUX_CMD_TYPE_NATIVE;

	if (i2c && mot)
		type |= DPTX_AUX_CMD_TYPE_MOT;

	dptx_aux_clear_data(dptx);

	if (!rw)
		dptx_aux_write_data(dptx, bytes, len);

	auxcmd = ((type << DPTX_AUX_CMD_TYPE_SHIFT) |
		  (addr << DPTX_AUX_CMD_ADDR_SHIFT) |
		  ((len - 1) << DPTX_AUX_CMD_REQ_LEN_SHIFT));

	if (addr_only)
		auxcmd |= DPTX_AUX_CMD_I2C_ADDR_ONLY;

	dptx_writel(dptx, DPTX_AUX_CMD, auxcmd);

	retval = dptx_handle_aux_reply(dptx);

	if (retval == -ETIMEDOUT) {
		dptx_warn(dptx, "AUX timed out %s (0x%x)\n",  rw ? "r" : "w", addr);
		return retval;
	}

	if (retval == -ESHUTDOWN) {
		dptx_dbg(dptx, "AUX aborted on driver shutdown\n");
		return retval;
	}

	if (dptx->aux.abort) {
		dptx_err(dptx, "AUX aborted\n");
		return -ETIMEDOUT;
	}

	status = (dptx->aux.sts & DPTX_AUX_STS_STATUS_MASK) >>
		DPTX_AUX_STS_STATUS_SHIFT;

	br = (dptx->aux.sts & DPTX_AUX_STS_BYTES_READ_MASK) >>
		DPTX_AUX_STS_BYTES_READ_SHIFT;

	switch (status) {
	case DPTX_AUX_STS_STATUS_ACK:
		dptx_dbg(dptx, "AUX Success\n");
		if (br == 0) {
			dptx_dbg(dptx, "BR=0, Retry\n");
			dptx_soft_reset(dptx, DPTX_SRST_CTRL_AUX);
			goto again;
		}
		break;
	case DPTX_AUX_STS_STATUS_NACK:
	case DPTX_AUX_STS_STATUS_I2C_NACK:
		dptx_dbg(dptx, "AUX Nack\n");
		return -EINVAL;
	case DPTX_AUX_STS_STATUS_I2C_DEFER:
	case DPTX_AUX_STS_STATUS_DEFER:
		dptx_info(dptx, "AUX Defer\n");
		goto again;
	default:
		dptx_err(dptx, "AUX Status Invalid\n");
		dptx_soft_reset(dptx, DPTX_SRST_CTRL_AUX);
		goto again;
	}

	if (rw)
		dptx_aux_read_data(dptx, bytes, len);

	return 0;
}

int dptx_aux_rw_bytes(struct dptx *dptx,
		      bool rw,
		      bool i2c,
		      u32 addr,
		      u8 *bytes,
		      unsigned int len)
{
	int i, retval;
	u32 reg_addr;

	for (i = 0; i < len; ) {
		unsigned int curlen;

		curlen = min_t(unsigned int, len - i, 16);

		reg_addr = i2c ? addr : (addr + i);
		retval = dptx_aux_rw(dptx, rw, i2c, true, false,
			reg_addr, &bytes[i], curlen);
		if (retval)
			return retval;

		i += curlen;
	}

	return 0;
}

int dptx_read_bytes_from_i2c(struct dptx *dptx,
			     u32 device_addr,
			     u8 *bytes,
			     u32 len)
{
	return dptx_aux_rw_bytes(dptx, true, true,
				 device_addr, bytes, len);
}

int dptx_i2c_address_only(struct dptx *dptx, unsigned int device_addr)
{
	u8 byte;

	return dptx_aux_rw(dptx, 0, true, false, true, device_addr, &byte, 1);
}

int dptx_write_bytes_to_i2c(struct dptx *dptx,
			    u32 device_addr,
			    u8 *bytes,
			    u32 len)
{
	return dptx_aux_rw_bytes(dptx, false, true,
				 device_addr, bytes, len);
}

int __dptx_read_bytes_from_dpcd(struct dptx *dptx,
				u32 reg_addr,
				u8 *bytes,
				u32 len)
{
	return dptx_aux_rw_bytes(dptx, true, false,
				 reg_addr, bytes, len);
}

int __dptx_write_bytes_to_dpcd(struct dptx *dptx,
			       u32 reg_addr,
			       u8 *bytes,
			       u32 len)
{
	return dptx_aux_rw_bytes(dptx, false, false,
				 reg_addr, bytes, len);
}

int __dptx_read_dpcd(struct dptx *dptx, u32 addr, u8 *byte)
{
	return __dptx_read_bytes_from_dpcd(dptx, addr, byte, 1);
}

int __dptx_write_dpcd(struct dptx *dptx, u32 addr, u8 byte)
{
	return __dptx_write_bytes_to_dpcd(dptx, addr, &byte, 1);
}
