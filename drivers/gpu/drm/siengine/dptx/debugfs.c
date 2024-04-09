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

#include <linux/debugfs.h>
#include "se_dptx.h"

static const struct debugfs_reg32 dptx_regs[];
static const int dptx_regs_size;
static int aux_addr;
static u32 dpcd_addr = 0;
static u8 aux_type;
static int aux_size;
/**
 * DOC: DEBUGFS Interface
 *
 * Top level:
 *
 * max_lane_count [rw] - The maximum lane count supported. Write to
 * this to set the max lane count.
 *
 * max_rate [rw] - The maximum rate supported. Write to this to set
 * the maximum rate.
 *
 *
 * pixel_mode_sel [rw] - Pixel mode selection. Write to this to set pixel mode.
 *
 * regdump [r] - Read this to see the values of all the core
 * registers.
 *
 * Link attributes:
 *
 * link/lane_count [r] - The current lanes in use. This will be 1, 2,
 * or 4.
 *
 * link/rate [r] - The current rate. 0 - RBR, 1 - HBR, 2 - HBR2, 3 -
 * HBR3.
 *
 * link/retrain [w] - Write to this to retrain the link. The value to
 * write is the desired rate and lanes separated by a space. For
 * example to retrain the link at 4 lanes at RBR write "0 4" to this
 * file.
 *
 * link/status [r] - Shows the status of the link.
 *
 * link/trained [r] - True if the link training was successful.
 *
 * aux_type [rw] - Type of AUX transaction. 0 - Native AUX Native Read, 1 - I2C
 * over AUX read, 2 - AUX Native Write, 3 - I2C over AUx write.
 *
 * aux_addr [rw] - Address used for AUX transaction data.
 *
 * aux_size [rw] - Size of aux transaction in bytes.
 *
 * aux [rw] - Data for AUX transaction.
 *
 * edid [rw] - EDID data.
 *
 * edid_size [rw] - Number of EDID blocks to read.
 *
 */


/**
 * dptx_aux_transfer() - Send AUX transfers
 * @dptx: The dptx struct
 * @aux_msg: AUX message
 *
 * Returns result of AUX transfer on success otherwise negative errno.
 */
static int dptx_aux_transfer(struct dptx *dptx, struct drm_dp_aux_msg *aux_msg)
{
	unsigned int addr;
	u8 req;
	void *buf;
	int len;
	int result;
	u32 hpdsts;

	hpdsts = dptx_readl(dptx, DPTX_HPDSTS);

	if (!(hpdsts & DPTX_HPDSTS_STATUS)) {
		dptx_dbg(dptx, "%s: Not connected\n", __func__);
		return -ENODEV;
	}

	result = 0;
	addr = aux_msg->address;
	req = aux_msg->request;
	buf = aux_msg->buffer;
	len = aux_msg->size;

	switch (req) {
	case DP_AUX_NATIVE_WRITE:
		result = dptx_aux_rw_bytes(dptx, false, false, addr, buf, len);
		break;
	case DP_AUX_NATIVE_READ:
		result = dptx_aux_rw_bytes(dptx, true, false, addr, buf, len);
		break;
	case DP_AUX_I2C_WRITE:
		result = dptx_write_bytes_to_i2c(dptx, addr, buf, len);
		break;
	case DP_AUX_I2C_READ:
		result = dptx_read_bytes_from_i2c(dptx, addr, buf, len);
		break;
	}

	return result;
}

/**
 * dptx_get_edid() - Get EDID raw data
 * @dptx: The dptx struct
 * @buf: The buffer to copy the EDID into
 * @buflen: The length of the buffer
 *
 * This function copies the EDID into @buf and returns the amount of
 * byte written or a negative error code. It will not copy any more
 * than buflen bytes.
 */
int dptx_get_edid(struct dptx *dptx, u8 *buf, size_t buflen)
{
	u32 hpdsts;
	int retval = 0;

	hpdsts = dptx_readl(dptx, DPTX_HPDSTS);

	if (!(hpdsts & DPTX_HPDSTS_STATUS)) {
		dptx_dbg(dptx, "%s: Not connected\n", __func__);
		retval = -ENODEV;
		goto fail;
	}

	memcpy(buf, dptx->edid, buflen);
fail:
	return retval;
}

/**
 * dptx_get_edid_size() - Get the size of the EDID
 * @dptx: The dptx struct
 *
 * Returns the size in bytes of the EDID data or -EINVAL if there is
 * no EDID data.
 */
int dptx_get_edid_size(struct dptx *dptx)
{
	int blocks;
	u32 hpdsts;
	int retval = 0;

	hpdsts = dptx_readl(dptx, DPTX_HPDSTS);

	if (!(hpdsts & DPTX_HPDSTS_STATUS)) {
		dptx_dbg(dptx, "%s: Not connected\n", __func__);
		retval = -ENODEV;
		goto fail;
	}

	blocks = (dptx->edid[126] + 1) * 128;
	retval = blocks;
fail:
	return retval;
}

/*
 * Link Status
 */
static int dptx_link_status_show(struct seq_file *s, void *unused)
{
	int i;
	struct dptx *dptx = s->private;

	mutex_lock(&dptx->lock);

	seq_printf(s, "trained = %d\n", dptx->link.trained);
	seq_printf(s, "rate = %d\n", dptx->link.rate);
	seq_printf(s, "lanes = %d\n", dptx->link.lanes);

	if (!dptx->link.trained)
		goto done;

	for (i = 0; i < dptx->link.lanes; i++) {
		seq_printf(s, "preemp and vswing level [%d] = %d, %d\n",
			   i, dptx->link.preemp_level[i],
			   dptx->link.vswing_level[i]);
	}

done:
	mutex_unlock(&dptx->lock);
	return 0;
}

static int dptx_link_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, dptx_link_status_show, inode->i_private);
}

static const struct file_operations dptx_link_status_fops = {
	.open		= dptx_link_status_open,
	.write		= NULL,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*
 * Link Retrain
 */
static int dptx_link_retrain_show(struct seq_file *s, void *unused)
{
	struct dptx *dptx = s->private;

	mutex_lock(&dptx->lock);
	seq_printf(s, "trained = %d\n", dptx->link.trained);
	mutex_unlock(&dptx->lock);

	return 0;
}

static ssize_t dptx_link_retrain_write(struct file *file,
				       const char __user *ubuf,
				       size_t count, loff_t *ppos)
{
	int retval = 0;
	struct seq_file *s = file->private_data;
	struct dptx *dptx = s->private;
	u8 buf[32];
	u8 rate;
	u8 lanes;
	u8 fixed;
	u8 saved_fixed_rate;
	u8 saved_fixed_lanes;
	u8 saved_max_rate;
	u8 saved_max_lanes;

	mutex_lock(&dptx->lock);
	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		retval = -EFAULT;
		goto done;
	}
	fixed = buf[0] - '0';
	rate = buf[2] - '0';
	lanes = buf[4] - '0';

	saved_fixed_rate = dptx->fixed_rate;
	saved_fixed_lanes = dptx->fixed_lanes;
	saved_max_rate = dptx->max_rate;
	saved_max_lanes = dptx->max_lanes;

	if (fixed) {
		dptx->fixed_rate = rate;
		dptx->fixed_lanes = lanes;
	} else {
		dptx->fixed_rate = 0;
		dptx->fixed_lanes = 0;
		dptx->max_rate = rate;
		dptx->max_lanes = lanes;
	}

	retval = dptx_handle_hotunplug(dptx);
	retval |= dptx_handle_hotplug(dptx);
	if (retval)
		goto done;

	dptx->fixed_rate = saved_fixed_rate;
	dptx->fixed_lanes = saved_fixed_lanes;
	dptx->max_rate = saved_max_rate;
	dptx->max_lanes = saved_max_lanes;
	retval = count;
done:
	mutex_unlock(&dptx->lock);
	return retval;
}

static int dptx_link_retrain_open(struct inode *inode, struct file *file)
{
	return single_open(file, dptx_link_retrain_show, inode->i_private);
}

static const struct file_operations dptx_link_retrain_fops = {
	.open		= dptx_link_retrain_open,
	.write		= dptx_link_retrain_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*
 * Aux
*/

static ssize_t dptx_aux_write(struct file *file,
			      const char __user *ubuf,
			      size_t count, loff_t *ppos)
{
	int retval = 0;
	u8 *buf;
	struct drm_dp_aux_msg *aux_msg = NULL;
	int i = 0;
	struct seq_file *s = file->private_data;
	struct dptx *dptx = s->private;

	mutex_lock(&dptx->lock);
	buf =  kmalloc(sizeof(count), GFP_KERNEL);
	if (copy_from_user(buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		retval = -EFAULT;
		goto done;
	}

	for (i = 0; i < count; i++)
		buf[i] = buf[i] - '0';

	aux_msg = kmalloc(sizeof(*aux_msg), GFP_KERNEL);
	if (!aux_msg) {
		retval = -EFAULT;
		goto done;
	}
	memset(aux_msg, 0, sizeof(*aux_msg));
	switch (aux_type) {
	case 2:
		aux_msg->request = DP_AUX_NATIVE_WRITE;
		break;
	case 3:
		aux_msg->request = DP_AUX_I2C_WRITE;
		break;
	}
	aux_msg->address = aux_addr;
	aux_msg->buffer = buf;
	aux_msg->size = count;

	retval = dptx_aux_transfer(dptx, aux_msg);
	if (retval)
		goto done;

	retval = count;
done:
	kfree(buf);
	kfree(aux_msg);
	mutex_unlock(&dptx->lock);
	return retval;
}

static ssize_t dptx_aux_read(struct file *file,
			     char __user *ubuf,
			     size_t count, loff_t *ppos)
{
	int retval = 0;
	u8 *aux_buf = NULL;
	struct drm_dp_aux_msg *aux_msg = NULL;

	struct seq_file *s = file->private_data;
	struct dptx *dptx = s->private;

	mutex_lock(&dptx->lock);
	aux_buf = kmalloc(aux_size, GFP_KERNEL);

	if (!aux_buf) {
		retval = -EFAULT;
		goto done;
	}
	aux_msg =  kmalloc(sizeof(*aux_msg), GFP_KERNEL);
	if (!aux_msg) {
		retval = -EFAULT;
		goto done;
	}

	memset(aux_buf, 0, sizeof(*aux_buf));
	memset(aux_msg, 0, sizeof(*aux_msg));

	switch (aux_type) {
	case 0:
		aux_msg->request = DP_AUX_NATIVE_READ;
		break;
	case 1:
		aux_msg->request = DP_AUX_I2C_READ;
		break;
	}
	aux_msg->address = aux_addr;
	aux_msg->buffer = aux_buf;
	aux_msg->size = aux_size;

	retval = dptx_aux_transfer(dptx, aux_msg);
	if (retval)
		goto done;

	if (copy_to_user(ubuf, aux_msg->buffer, aux_size) != 0) {
		retval = -EFAULT;
		goto done;
	}
	retval = count;
done:
	kfree(aux_buf);
	kfree(aux_msg);
	mutex_unlock(&dptx->lock);
	return retval;
}

static int dptx_aux_show(struct seq_file *s, void *unused)
{
	return 0;
}

static int dptx_aux_open(struct inode *inode, struct file *file)
{
	return single_open(file, dptx_aux_show, inode->i_private);
}

static const struct file_operations dptx_aux_fops = {
	.open           = dptx_aux_open,
	.write          = dptx_aux_write,
	.read           = dptx_aux_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

/*
* edid
*/

static ssize_t dptx_edid_read(struct file *file,
			      char __user *ubuf,
			      size_t count, loff_t *ppos)
{
	int retval = 0;
	int edid_size;
	struct seq_file *s = file->private_data;
	struct dptx *dptx = s->private;

	mutex_lock(&dptx->lock);

	if (*ppos)
		goto done;

	retval = dptx_read_edid(dptx);
	if (retval) {
		retval = 0;
		goto done;
	}

	edid_size = (dptx->edid[126] + 1) * 128;

	if (clear_user(ubuf, count)) {
		retval = -EIO;
		goto done;
	}

	if (copy_to_user(ubuf, dptx->edid, edid_size) != 0) {
		retval = -EFAULT;
		goto done;
	}
	*ppos += edid_size;
	retval = edid_size;

done:
	mutex_unlock(&dptx->lock);
	return retval;
}

static int dptx_edid_show(struct seq_file *s, void *unused)
{
	return 0;
}

static int dptx_edid_open(struct inode *inode, struct file *file)
{
	return single_open(file, dptx_edid_show, inode->i_private);
}

static const struct file_operations dptx_edid_fops = {
	.open           = dptx_edid_open,
	.write          = NULL,
	.read           = dptx_edid_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

/*
 * edid_size
 */

static int dptx_edid_size_show(struct seq_file *s, void *unused)
{
	struct dptx *dptx = s->private;
	int edid_size = 0;
	int retval = 0;

	mutex_lock(&dptx->lock);

	retval = dptx_read_edid(dptx);
	if (retval)
		goto done;

	edid_size = (dptx->edid[126] + 1) * 128;
	seq_printf(s, "edid_size = %d\n", edid_size);

done:
	mutex_unlock(&dptx->lock);
	return retval;
}

static int dptx_edid_size_open(struct inode *inode, struct file *file)
{
	return single_open(file, dptx_edid_size_show, inode->i_private);
}

static const struct file_operations dptx_edid_size_fops = {
	.open           = dptx_edid_size_open,
	.write          = NULL,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int dptx_rx_caps_show(struct seq_file *s, void *unused)
{
	struct dptx *dptx = s->private;
	int retval = 0;
	u8 byte = 0;

	mutex_lock(&dptx->lock);
	dptx_read_dpcd(dptx, DP_MSTM_CAP, &byte);
	seq_printf(s, "%x\n", byte);
	mutex_unlock(&dptx->lock);

	return retval;
}

static int dptx_rx_caps_open(struct inode *inode, struct file *file)
{
	return single_open(file, dptx_rx_caps_show, inode->i_private);
}

static const struct file_operations dptx_rx_caps_fops = {
	.open		= dptx_rx_caps_open,
	.write          = NULL,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int dptx_dpcd_read_show(struct seq_file *s, void *unused)
{
	struct dptx *dptx = s->private;
	int retval = 0;
	u8 byte;

	mutex_lock(&dptx->lock);
	dptx_read_dpcd(dptx, dpcd_addr, &byte);
	seq_printf(s, "0x%02x\n", byte);
	mutex_unlock(&dptx->lock);

	return retval;
}

static int dptx_dpcd_read_open(struct inode *inode, struct file *file)
{
	return single_open(file, dptx_dpcd_read_show, inode->i_private);
}

static const struct file_operations dptx_dpcd_read_fops = {
	.open		= dptx_dpcd_read_open,
	.write          = NULL,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

/*FEC*/
static ssize_t dptx_fec_en_write(struct file *file,
				    const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	int retval = 0;
	char buf[3];
	u8 fec_en;

	struct seq_file *s = file->private_data;
	struct dptx *dptx = s->private;

	mutex_lock(&dptx->lock);
	memset(buf, 0, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		retval = -EFAULT;
		goto done;
	}

	if (kstrtou8(buf, 10, &fec_en) < 0) {
		retval = -EINVAL;
		goto done;
	}

	retval = dptx_en_fec(dptx, fec_en);
	if (retval)
		goto done;
	retval = count;

done:
	mutex_unlock(&dptx->lock);
	return retval;
}

static int dptx_fec_en_show(struct seq_file *s, void *unused)
{
	return 0;
}

static int dptx_fec_en_open(struct inode *inode, struct file *file)
{
	return single_open(file, dptx_fec_en_show, inode->i_private);
}

static const struct file_operations dptx_fec_en_fops = {
	.open           = dptx_fec_en_open,
	.write          = dptx_fec_en_write,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

/*
 * TX_EQ
 */
static int dptx_set_tx_eq_show(struct seq_file *s, void *unused)
{
	struct dptx *dptx = s->private;

	mutex_lock(&dptx->lock);
	seq_printf(s, "tx_eq_en = %d vswing_level = %d preemp_level = %d\n"
		"tx_eq_main = %d tx_eq_post = %d tx_eq_pre = %d\n",
		dptx->tx_eq_en, dptx->eq_setting.vswing_level,
		dptx->eq_setting.preemp_level, dptx->eq_setting.tx_eq_main,
		dptx->eq_setting.tx_eq_post, dptx->eq_setting.tx_eq_pre);
	mutex_unlock(&dptx->lock);

	return 0;
}

static ssize_t dptx_set_tx_eq_write(struct file *file,
				       const char __user *ubuf,
				       size_t count, loff_t *ppos)
{
	int retval = 0;
	struct seq_file *s = file->private_data;
	struct dptx *dptx = s->private;
	u8 buf[32];

	mutex_lock(&dptx->lock);
	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		retval = -EFAULT;
		goto done;
	}

	dptx->tx_eq_en = buf[0] - '0';
	if (dptx->tx_eq_en) {
		dptx->eq_setting.vswing_level = buf[2] - '0';
		dptx->eq_setting.preemp_level = buf[4] - '0';
		dptx->eq_setting.tx_eq_main = (buf[6] - '0') * 10 + (buf[7] - '0');
		dptx->eq_setting.tx_eq_post = (buf[9] - '0') * 10 + (buf[10] - '0');
		dptx->eq_setting.tx_eq_pre = 0;

		dptx_link_adjust_drive_settings(dptx, NULL);

		dptx->tx_eq_en = 0;
	}

	retval = count;
done:
	mutex_unlock(&dptx->lock);
	return retval;
}

static int dptx_set_tx_eq_open(struct inode *inode, struct file *file)
{
	return single_open(file, dptx_set_tx_eq_show, inode->i_private);
}

static const struct file_operations dptx_set_tx_eq_fops = {
	.open		= dptx_set_tx_eq_open,
	.write		= dptx_set_tx_eq_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*
 * SSC
 */
static int dptx_config_ssc_show(struct seq_file *s, void *unused)
{
	struct dptx *dptx = s->private;

	mutex_lock(&dptx->lock);
	seq_printf(s, "ssc_en = %d\n", dptx->ssc_en);
	mutex_unlock(&dptx->lock);

	return 0;
}

static ssize_t dptx_config_ssc_write(struct file *file,
				       const char __user *ubuf,
				       size_t count, loff_t *ppos)
{
	int retval = 0;
	struct seq_file *s = file->private_data;
	struct dptx *dptx = s->private;
	u8 buf[32];

	mutex_lock(&dptx->lock);
	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		retval = -EFAULT;
		goto done;
	}

	dptx->ssc_en = buf[0] - '0';

	dptx_handle_hotunplug(dptx);
	dptx_handle_hotplug(dptx);
	retval = count;

done:
	mutex_unlock(&dptx->lock);
	return retval;
}

static int dptx_config_ssc_open(struct inode *inode, struct file *file)
{
	return single_open(file, dptx_config_ssc_show, inode->i_private);
}

static const struct file_operations dptx_config_ssc_fops = {
	.open		= dptx_config_ssc_open,
	.write		= dptx_config_ssc_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#if 0
/*MST*/
static ssize_t dptx_mst_en_write(struct file *file,
				    const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	int retval = 0;
	char buf[3];
	u8 mst_en;

	struct seq_file *s = file->private_data;
	struct dptx *dptx = s->private;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		retval = -EFAULT;
		goto done;
	}

	if (kstrtou8(buf, 10, &mst_en) < 0) {
		retval = -EINVAL;
		goto done;
	}

    retval = dptx_en_mst(dptx, mst_en);
    if (retval)
		goto done;
	retval = count;
done:

	return retval;
}

static int dptx_mst_en_show(struct seq_file *s, void *unused)
{
	return 0;
}

static int dptx_mst_en_open(struct inode *inode, struct file *file)
{
	return single_open(file, dptx_mst_en_show, inode->i_private);
}

static const struct file_operations dptx_mst_en_fops = {
	.open           = dptx_mst_en_open,
	.write          = dptx_mst_en_write,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};
#endif

void dptx_debugfs_init(struct dptx *dptx)
{
	struct dentry *root;
	struct dentry *video;
	struct dentry *link;
	struct dentry *file;

	root = debugfs_create_dir(dev_name(dptx->dev), NULL);
	if (IS_ERR_OR_NULL(root)) {
		dptx_err(dptx, "Can't create debugfs root\n");
		return;
	}

	link = debugfs_create_dir("link", root);
	if (IS_ERR_OR_NULL(link)) {
		dptx_err(dptx, "Can't create debugfs link\n");
		debugfs_remove_recursive(root);
		return;
	}

	video = debugfs_create_dir("video", root);
	if (IS_ERR_OR_NULL(video)) {
		dptx_err(dptx, "Can't create debugfs video\n");
		debugfs_remove_recursive(root);
		return;
	}

	/* Registers */
	dptx->regset = kzalloc(sizeof(*dptx->regset), GFP_KERNEL);
	if (!dptx->regset) {
		debugfs_remove_recursive(root);
		return;
	}

	dptx->regset->regs = dptx_regs;
	dptx->regset->nregs = dptx_regs_size;
	dptx->regset->base = dptx->base;

	debugfs_create_regset32("regdump", S_IRUGO, root, dptx->regset);

	/* Core driver */
	debugfs_create_u8("max_rate", S_IRUGO | S_IWUSR, root,
			  &dptx->max_rate);
	debugfs_create_u8("max_lane_count", S_IRUGO | S_IWUSR, root,
			  &dptx->max_lanes);
	debugfs_create_u8("pixel_mode_sel", S_IRUGO | S_IWUSR, root,
			  &dptx->multipixel);

	/* DPCD */
	file = debugfs_create_file("rx_caps", S_IRUGO | S_IWUSR,
				   root, dptx, &dptx_rx_caps_fops);
	if (!file)
		dev_dbg(dptx->dev, "Can't create debugfs video rx_caps\n");

	debugfs_create_u32("dpcd_addr", S_IRUGO | S_IWUSR, root,
			   &dpcd_addr);

	file = debugfs_create_file("dpcd_read", S_IRUGO | S_IWUSR,
				   root, dptx, &dptx_dpcd_read_fops);
	if (!file)
		dev_dbg(dptx->dev, "Can't create debugfs video dpcd_read\n");

	/*FEC*/
	file = debugfs_create_file("fec_en", S_IRUGO | S_IWUSR,
				   root, dptx, &dptx_fec_en_fops);
	if (!file)
		dev_dbg(dptx->dev, "Can't create debugfs fec_en\n");

#if 0
    /* MST */
	file = debugfs_create_file("mst_en", S_IRUGO | S_IWUSR,
				   root, dptx, &dptx_mst_en_fops);
	if (!file)
		dev_dbg(dptx->dev, "Can't create debugfs mst_en\n");
#endif

	/* Link */
	debugfs_create_u8("rate", S_IRUGO, link,
			  &dptx->link.rate);
	debugfs_create_u8("lane_count", S_IRUGO, link,
			  &dptx->link.lanes);
	debugfs_create_u8("aux_type", S_IRUGO | S_IWUSR, link,
			  &aux_type);
	debugfs_create_u32("aux_addr", S_IRUGO | S_IWUSR, link,
			   &aux_addr);
	debugfs_create_u32("aux_size", S_IRUGO | S_IWUSR, link,
			   &aux_size);

	debugfs_create_bool("trained", S_IRUGO, link,
			    &dptx->link.trained);

	file = debugfs_create_file("status", S_IRUGO,
				   link, dptx, &dptx_link_status_fops);
	if (!file)
		dev_dbg(dptx->dev, "Can't create debugfs link status\n");

	file = debugfs_create_file("retrain", S_IRUGO | S_IWUSR,
				   link, dptx, &dptx_link_retrain_fops);
	if (!file)
		dev_dbg(dptx->dev, "Can't create debugfs link retrain\n");

	file = debugfs_create_file("txeq", S_IRUGO | S_IWUSR,
				   link, dptx, &dptx_set_tx_eq_fops);
	if (!file)
		dev_dbg(dptx->dev, "Can't create debugfs set tx eq\n");

	file = debugfs_create_file("ssc", S_IRUGO | S_IWUSR,
				   link, dptx, &dptx_config_ssc_fops);
	if (!file)
		dev_dbg(dptx->dev, "Can't create debugfs config ssc\n");

	file = debugfs_create_file("aux", S_IRUGO | S_IWUSR,
				   link, dptx, &dptx_aux_fops);
	if (!file)
		dev_dbg(dptx->dev, "Can't create debugfs aux\n");

	/* Video */
	file = debugfs_create_file("edid", S_IRUGO | S_IWUSR, video, dptx,
				   &dptx_edid_fops);
	if (!file)
		dev_dbg(dptx->dev, "Can't create debugfs video edid\n");

	file = debugfs_create_file("edid_size", S_IRUGO | S_IWUSR, video,
				   dptx, &dptx_edid_size_fops);
	if (!file)
		dev_dbg(dptx->dev, "Can't create debugfs video edid size\n");

	dptx->root = root;
}

void dptx_debugfs_exit(struct dptx *dptx)
{
	debugfs_remove_recursive(dptx->root);
	kfree(dptx->regset);
}

#define DEBUGFS_REG32(_name)				\
{							\
	.name	= #_name,				\
	.offset	= DPTX_##_name,				\
}

static const struct debugfs_reg32 dptx_regs[] = {
	DEBUGFS_REG32(VER_NUMBER),
	DEBUGFS_REG32(VER_TYPE),
	DEBUGFS_REG32(CONFIG1),
	DEBUGFS_REG32(CCTL),
	DEBUGFS_REG32(SRST_CTRL),
	{ .name = "VSAMPLE_CTRL_STREAM_1", .offset = DPTX_VSAMPLE_CTRL_N(0), },
	{ .name = "VSAMPLE_CTRL_STREAM_2", .offset = DPTX_VSAMPLE_CTRL_N(1), },
	{ .name = "VSAMPLE_CTRL_STREAM_3", .offset = DPTX_VSAMPLE_CTRL_N(2), },
	{ .name = "VSAMPLE_CTRL_STREAM_4", .offset = DPTX_VSAMPLE_CTRL_N(3), },
	{ .name = "VSAMPLE_RESERVED1_STREAM_1", .offset = DPTX_VSAMPLE_RESERVED1_N(0), },
	{ .name = "VSAMPLE_RESERVED1_STREAM_2", .offset = DPTX_VSAMPLE_RESERVED1_N(1), },
	{ .name = "VSAMPLE_RESERVED1_STREAM_3", .offset = DPTX_VSAMPLE_RESERVED1_N(2), },
	{ .name = "VSAMPLE_RESERVED1_STREAM_4", .offset = DPTX_VSAMPLE_RESERVED1_N(3), },
	{ .name = "VSAMPLE_RESERVED2_STREAM_1", .offset = DPTX_VSAMPLE_RESERVED1_N(0), },
	{ .name = "VSAMPLE_RESERVED2_STREAM_2", .offset = DPTX_VSAMPLE_RESERVED1_N(1), },
	{ .name = "VSAMPLE_RESERVED2_STREAM_3", .offset = DPTX_VSAMPLE_RESERVED1_N(2), },
	{ .name = "VSAMPLE_RESERVED2_STREAM_4", .offset = DPTX_VSAMPLE_RESERVED1_N(3), },
	{ .name = "VSAMPLE_POLARITY_CTRL_STREAM_1", .offset = DPTX_VSAMPLE_POLARITY_CTRL_N(0), },
	{ .name = "VSAMPLE_POLARITY_CTRL_STREAM_2", .offset = DPTX_VSAMPLE_POLARITY_CTRL_N(1), },
	{ .name = "VSAMPLE_POLARITY_CTRL_STREAM_3", .offset = DPTX_VSAMPLE_POLARITY_CTRL_N(2), },
	{ .name = "VSAMPLE_POLARITY_CTRL_STREAM_4", .offset = DPTX_VSAMPLE_POLARITY_CTRL_N(3), },
	{ .name = "VIDEO_CONFIG1_STREAM_1", .offset = DPTX_VIDEO_CONFIG1_N(0), },
	{ .name = "VIDEO_CONFIG1_STREAM_2", .offset = DPTX_VIDEO_CONFIG1_N(1), },
	{ .name = "VIDEO_CONFIG1_STREAM_3", .offset = DPTX_VIDEO_CONFIG1_N(2), },
	{ .name = "VIDEO_CONFIG1_STREAM_4", .offset = DPTX_VIDEO_CONFIG1_N(3), },
	{ .name = "VIDEO_CONFIG2_STREAM_1", .offset = DPTX_VIDEO_CONFIG2_N(0), },
	{ .name = "VIDEO_CONFIG2_STREAM_2", .offset = DPTX_VIDEO_CONFIG2_N(1), },
	{ .name = "VIDEO_CONFIG2_STREAM_3", .offset = DPTX_VIDEO_CONFIG2_N(2), },
	{ .name = "VIDEO_CONFIG2_STREAM_4", .offset = DPTX_VIDEO_CONFIG2_N(3), },
	{ .name = "VIDEO_CONFIG3_STREAM_1", .offset = DPTX_VIDEO_CONFIG3_N(0), },
	{ .name = "VIDEO_CONFIG3_STREAM_2", .offset = DPTX_VIDEO_CONFIG3_N(1), },
	{ .name = "VIDEO_CONFIG3_STREAM_3", .offset = DPTX_VIDEO_CONFIG3_N(2), },
	{ .name = "VIDEO_CONFIG3_STREAM_4", .offset = DPTX_VIDEO_CONFIG3_N(3), },
	{ .name = "VIDEO_CONFIG4_STREAM_1", .offset = DPTX_VIDEO_CONFIG4_N(0), },
	{ .name = "VIDEO_CONFIG4_STREAM_2", .offset = DPTX_VIDEO_CONFIG4_N(1), },
	{ .name = "VIDEO_CONFIG4_STREAM_3", .offset = DPTX_VIDEO_CONFIG4_N(2), },
	{ .name = "VIDEO_CONFIG4_STREAM_4", .offset = DPTX_VIDEO_CONFIG4_N(3), },
	{ .name = "VIDEO_CONFIG5_STREAM_1", .offset = DPTX_VIDEO_CONFIG5_N(0), },
	{ .name = "VIDEO_CONFIG5_STREAM_2", .offset = DPTX_VIDEO_CONFIG5_N(1), },
	{ .name = "VIDEO_CONFIG5_STREAM_3", .offset = DPTX_VIDEO_CONFIG5_N(2), },
	{ .name = "VIDEO_CONFIG5_STREAM_4", .offset = DPTX_VIDEO_CONFIG5_N(3), },

	{ .name = "VIDEO_MSA1_STREAM_1", .offset = DPTX_VIDEO_MSA1_N(0), },
	{ .name = "VIDEO_MSA1_STREAM_2", .offset = DPTX_VIDEO_MSA1_N(1), },
	{ .name = "VIDEO_MSA1_STREAM_3", .offset = DPTX_VIDEO_MSA1_N(2), },
	{ .name = "VIDEO_MSA1_STREAM_4", .offset = DPTX_VIDEO_MSA1_N(3), },

	{ .name = "VIDEO_MSA2_STREAM_1", .offset = DPTX_VIDEO_MSA2_N(0), },
	{ .name = "VIDEO_MSA2_STREAM_2", .offset = DPTX_VIDEO_MSA2_N(1), },
	{ .name = "VIDEO_MSA2_STREAM_3", .offset = DPTX_VIDEO_MSA2_N(2), },
	{ .name = "VIDEO_MSA2_STREAM_4", .offset = DPTX_VIDEO_MSA2_N(3), },

	{ .name = "VIDEO_MSA3_STREAM_1", .offset = DPTX_VIDEO_MSA3_N(0), },
	{ .name = "VIDEO_MSA3_STREAM_2", .offset = DPTX_VIDEO_MSA3_N(1), },
	{ .name = "VIDEO_MSA3_STREAM_3", .offset = DPTX_VIDEO_MSA3_N(2), },
	{ .name = "VIDEO_MSA3_STREAM_4", .offset = DPTX_VIDEO_MSA3_N(3), },

	{ .name = "VIDEO_HBLANK_INTERVAL_1", .offset = DPTX_VIDEO_HBLANK_INTERVAL_N(0), },
	{ .name = "VIDEO_HBLANK_INTERVAL_2", .offset = DPTX_VIDEO_HBLANK_INTERVAL_N(1), },
	{ .name = "VIDEO_HBLANK_INTERVAL_3", .offset = DPTX_VIDEO_HBLANK_INTERVAL_N(2), },
	{ .name = "VIDEO_HBLANK_INTERVAL_4", .offset = DPTX_VIDEO_HBLANK_INTERVAL_N(3), },

	{ .name = "AUD_CONFIG1_1", .offset = DPTX_AUD_CONFIG1_N(0), },
	{ .name = "AUD_CONFIG1_2", .offset = DPTX_AUD_CONFIG1_N(1), },
	{ .name = "AUD_CONFIG1_3", .offset = DPTX_AUD_CONFIG1_N(2), },
	{ .name = "AUD_CONFIG1_4", .offset = DPTX_AUD_CONFIG1_N(3), },

	{ .name = "VG_CONFIG1_STREAM_1", .offset = DPTX_VG_CONFIG1_N(0), },
	{ .name = "VG_CONFIG1_STREAM_2", .offset = DPTX_VG_CONFIG1_N(1), },
	{ .name = "VG_CONFIG1_STREAM_3", .offset = DPTX_VG_CONFIG1_N(2), },
	{ .name = "VG_CONFIG1_STREAM_4", .offset = DPTX_VG_CONFIG1_N(3), },

	{ .name = "VG_CONFIG2_STREAM_1", .offset = DPTX_VG_CONFIG2_N(0), },
	{ .name = "VG_CONFIG2_STREAM_2", .offset = DPTX_VG_CONFIG2_N(1), },
	{ .name = "VG_CONFIG2_STREAM_3", .offset = DPTX_VG_CONFIG2_N(2), },
	{ .name = "VG_CONFIG2_STREAM_4", .offset = DPTX_VG_CONFIG2_N(3), },

	{ .name = "VG_CONFIG3_STREAM_1", .offset = DPTX_VG_CONFIG3_N(0), },
	{ .name = "VG_CONFIG3_STREAM_2", .offset = DPTX_VG_CONFIG3_N(1), },
	{ .name = "VG_CONFIG3_STREAM_3", .offset = DPTX_VG_CONFIG3_N(2), },
	{ .name = "VG_CONFIG3_STREAM_4", .offset = DPTX_VG_CONFIG3_N(3), },

	{ .name = "VG_CONFIG4_STREAM_1", .offset = DPTX_VG_CONFIG4_N(0), },
	{ .name = "VG_CONFIG4_STREAM_2", .offset = DPTX_VG_CONFIG4_N(1), },
	{ .name = "VG_CONFIG4_STREAM_3", .offset = DPTX_VG_CONFIG4_N(2), },
	{ .name = "VG_CONFIG4_STREAM_4", .offset = DPTX_VG_CONFIG4_N(3), },

	{ .name = "VG_CONFIG5_STREAM_1", .offset = DPTX_VG_CONFIG5_N(0), },
	{ .name = "VG_CONFIG5_STREAM_2", .offset = DPTX_VG_CONFIG5_N(1), },
	{ .name = "VG_CONFIG5_STREAM_3", .offset = DPTX_VG_CONFIG5_N(2), },
	{ .name = "VG_CONFIG5_STREAM_4", .offset = DPTX_VG_CONFIG5_N(3), },

	{ .name = "MST_VCP_TABLE_0", .offset = DPTX_MST_VCP_TABLE_REG_N(0), },
	{ .name = "MST_VCP_TABLE_1", .offset = DPTX_MST_VCP_TABLE_REG_N(1), },
	{ .name = "MST_VCP_TABLE_2", .offset = DPTX_MST_VCP_TABLE_REG_N(2), },
	{ .name = "MST_VCP_TABLE_3", .offset = DPTX_MST_VCP_TABLE_REG_N(3), },
	{ .name = "MST_VCP_TABLE_4", .offset = DPTX_MST_VCP_TABLE_REG_N(4), },
	{ .name = "MST_VCP_TABLE_5", .offset = DPTX_MST_VCP_TABLE_REG_N(5), },
	{ .name = "MST_VCP_TABLE_6", .offset = DPTX_MST_VCP_TABLE_REG_N(6), },
	{ .name = "MST_VCP_TABLE_7", .offset = DPTX_MST_VCP_TABLE_REG_N(7), },

	DEBUGFS_REG32(AG_CONFIG1),
	DEBUGFS_REG32(AG_CONFIG2),
	DEBUGFS_REG32(AG_CONFIG3),
	DEBUGFS_REG32(AG_CONFIG4),
	DEBUGFS_REG32(AG_CONFIG5),
	DEBUGFS_REG32(AG_CONFIG6),
	DEBUGFS_REG32(SDP_VERTICAL_CTRL),
	DEBUGFS_REG32(PHYIF_CTRL),
	DEBUGFS_REG32(PHY_TX_EQ),
	DEBUGFS_REG32(CUSTOMPAT0),
	DEBUGFS_REG32(CUSTOMPAT1),
	DEBUGFS_REG32(CUSTOMPAT2),
	DEBUGFS_REG32(PHYREG_CMDADDR),
	DEBUGFS_REG32(PHYREG_DATA),
	DEBUGFS_REG32(TYPE_C_CTRL),
	DEBUGFS_REG32(AUX_CMD),
	DEBUGFS_REG32(AUX_STS),
	DEBUGFS_REG32(AUX_DATA0),
	DEBUGFS_REG32(AUX_DATA1),
	DEBUGFS_REG32(AUX_DATA2),
	DEBUGFS_REG32(AUX_DATA3),
	DEBUGFS_REG32(ISTS),
	DEBUGFS_REG32(IEN),
	DEBUGFS_REG32(HPDSTS),
	DEBUGFS_REG32(HPD_IEN),
	DEBUGFS_REG32(HDCP_CONFIG),
};

static const int dptx_regs_size = ARRAY_SIZE(dptx_regs);

