// SPDX-License-Identifier:	 GPL-2.0
// (C) 2017-2018 Synopsys, Inc. (www.synopsys.com)
// (C) 2020-2021 Siengine, Inc. (www.siengine.com/)

/*
 * Synopsys DesignWare AXI 2.0a DMA Controller driver.
 *
 * Author: Eugeniy Paltsev <Eugeniy.Paltsev@synopsys.com>
 * Author: Mingrui Zhou <Mingrui.Zhou@siengine.com>
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/dmaengine.h>

#include <linux/iopoll.h>
//#include "dw-axi-dmac.h"
#include "dw-axi-dmac-audio.h"
#include "../virt-dma.h"

#include <linux/cdev.h>
#include <linux/rpmsg.h>
#include <linux/rpmsg/siengine-rpmsg.h>
#include "../../rpmsg/rpmsg_internal.h"
#include "../../rpmsg/se1000-rpmsg-dev.h"
/*
 * The set of bus widths supported by the DMA controller. DW AXI DMAC supports
 * master data bus width up to 512 bits (for both AXI master interfaces), but
 * it depends on IP block configurarion.
 */
#define AXI_DMA_BUSWIDTHS		  \
	(DMA_SLAVE_BUSWIDTH_1_BYTE	| \
	DMA_SLAVE_BUSWIDTH_2_BYTES	| \
	DMA_SLAVE_BUSWIDTH_4_BYTES	| \
	DMA_SLAVE_BUSWIDTH_8_BYTES	| \
	DMA_SLAVE_BUSWIDTH_16_BYTES	| \
	DMA_SLAVE_BUSWIDTH_32_BYTES	| \
	DMA_SLAVE_BUSWIDTH_64_BYTES)


static inline void
axi_dma_iowrite32(struct axi_dma_chip *chip, u32 reg, u32 val)
{
	iowrite32(val, chip->regs + reg);
}

static inline u32 axi_dma_ioread32(struct axi_dma_chip *chip, u32 reg)
{
	return ioread32(chip->regs + reg);
}

static inline void
axi_chan_iowrite32(struct axi_dma_chan *chan, u32 reg, u32 val)
{
	iowrite32(val, chan->chan_regs + reg);
}

static inline u32 axi_chan_ioread32(struct axi_dma_chan *chan, u32 reg)
{
	return ioread32(chan->chan_regs + reg);
}

static inline void
axi_dma_iowrite64(struct axi_dma_chip *chip, u32 reg, u64 val)
{
	/*
	 * We split one 64 bit write for two 32 bit write as some HW doesn't
	 * support 64 bit access.
	 */
	iowrite32(lower_32_bits(val), chip->regs + reg);
	iowrite32(upper_32_bits(val), chip->regs + reg + 4);
}

static inline u64 axi_dma_ioread64(struct axi_dma_chip *chip, u32 reg)
{
    u32 lower_val = ioread32(chip->regs + reg);
    u32 upper_val = ioread32(chip->regs + reg + 4);
    u64 val = ((u64)upper_val << 32) | lower_val;
    return val;
}

static inline void
axi_chan_iowrite64(struct axi_dma_chan *chan, u32 reg, u64 val)
{
	/*
	 * We split one 64 bit write for two 32 bit write as some HW doesn't
	 * support 64 bit access.
	 */
	iowrite32(lower_32_bits(val), chan->chan_regs + reg);
	iowrite32(upper_32_bits(val), chan->chan_regs + reg + 4);
}

static inline u64 axi_chan_ioread64(struct axi_dma_chan *chan, u32 reg)
{
    u32 lower_val = ioread32(chan->chan_regs + reg);
    u32 upper_val = ioread32(chan->chan_regs + reg + 4);

    u64 val = ((u64)upper_val << 32) | lower_val;
    return val;
}

static inline void axi_chan_config_write(struct axi_dma_chan *chan,
					 struct axi_dma_chan_config *config)
{
	u32 cfg_lo, cfg_hi;

	cfg_lo = (config->dst_multblk_type << CH_CFG_L_DST_MULTBLK_TYPE_POS |
		  config->src_multblk_type << CH_CFG_L_SRC_MULTBLK_TYPE_POS);
	if (chan->chip->dw->hdata->reg_map_8_channels &&
			 (chan->chip->dw->hdata->hw_handshake_num <= 16)) {
		cfg_hi = config->tt_fc << CH_CFG_H_TT_FC_POS |
			 config->hs_sel_src << CH_CFG_H_HS_SEL_SRC_POS |
			 config->hs_sel_dst << CH_CFG_H_HS_SEL_DST_POS |
			 config->src_per << CH_CFG_H_SRC_PER_POS |
			 config->dst_per << CH_CFG_H_DST_PER_POS |
			 config->prior << CH_CFG_H_PRIORITY_POS;
	} else {
		cfg_lo |= config->src_per << CH_CFG2_L_SRC_PER_POS |
				config->dst_per << CH_CFG2_L_DST_PER_POS;
		cfg_hi = config->tt_fc << CH_CFG2_H_TT_FC_POS |
			 config->hs_sel_src << CH_CFG2_H_HS_SEL_SRC_POS |
			 config->hs_sel_dst << CH_CFG2_H_HS_SEL_DST_POS |
			 config->prior << CH_CFG2_H_PRIORITY_POS;
	}

	//axi_chan_iowrite32(chan, CH_CFG_L, cfg_lo);
	//axi_chan_iowrite32(chan, CH_CFG_H, cfg_hi);
	axi_chan_iowrite64(chan, CH_CFG, ((u64)cfg_hi << 32) | cfg_lo);
}

static inline void axi_dma_disable(struct axi_dma_chip *chip)
{
	u64 val;

	val = axi_dma_ioread64(chip, DMAC_CFG);
	val &= ~DMAC_EN_MASK;
	axi_dma_iowrite64(chip, DMAC_CFG, val);
}

static inline void axi_dma_enable(struct axi_dma_chip *chip)
{
	u64 val;

	val = axi_dma_ioread64(chip, DMAC_CFG);
	val |= DMAC_EN_MASK;
	axi_dma_iowrite64(chip, DMAC_CFG, val);
}

static inline void axi_dma_irq_disable(struct axi_dma_chip *chip)
{
	u64 val;

	val = axi_dma_ioread64(chip, DMAC_CFG);
	val &= ~INT_EN_MASK;
	axi_dma_iowrite64(chip, DMAC_CFG, val);
}

static inline void axi_dma_irq_enable(struct axi_dma_chip *chip)
{
	u64 val;

	val = axi_dma_ioread64(chip, DMAC_CFG);
	val |= INT_EN_MASK;
	axi_dma_iowrite64(chip, DMAC_CFG, val);
	/* check enable bit */
	val = axi_dma_ioread64(chip, DMAC_CFG);
	if (!(val & INT_EN_MASK))
		dev_info(chip->dev, "DMA IRQ EN failed: 0x%x\n", val);
}

static inline void axi_chan_irq_disable(struct axi_dma_chan *chan, u64 irq_mask)
{
	u64 val;

	if (likely(irq_mask == DWAXIDMAC_IRQ_ALL)) {
		axi_chan_iowrite64(chan, CH_INTSTATUS_ENA, DWAXIDMAC_IRQ_NONE);
	} else {
		val = axi_chan_ioread64(chan, CH_INTSTATUS_ENA);
		val &= ~irq_mask;
		axi_chan_iowrite64(chan, CH_INTSTATUS_ENA, val);
	}
}

static inline void axi_chan_irq_set(struct axi_dma_chan *chan, u64 irq_mask)
{
	axi_chan_iowrite64(chan, CH_INTSTATUS_ENA, irq_mask);
}

static inline void axi_chan_irq_sig_set(struct axi_dma_chan *chan, u64 irq_mask)
{
	axi_chan_iowrite64(chan, CH_INTSIGNAL_ENA, irq_mask);
}

static inline void axi_chan_irq_clear(struct axi_dma_chan *chan, u64 irq_mask)
{
	axi_chan_iowrite64(chan, CH_INTCLEAR, irq_mask);
}

static inline u64 axi_chan_irq_read(struct axi_dma_chan *chan)
{
	return axi_chan_ioread64(chan, CH_INTSTATUS);
}

static inline void axi_chan_disable(struct axi_dma_chan *chan)
{
	u64 val;
	u64 chan_shift = (BIT(chan->id) << DMAC_CHAN_EN_SHIFT);

	val = axi_dma_ioread64(chan->chip, DMAC_CHEN);

	val &= ~chan_shift;
	if (chan->chip->dw->hdata->reg_map_8_channels)
		val |=   BIT(chan->id) << DMAC_CHAN_EN_WE_SHIFT;
	else
		val |=   BIT(chan->id) << DMAC_CHAN_EN2_WE_SHIFT;
	axi_dma_iowrite64(chan->chip, DMAC_CHEN, val);

	read_poll_timeout_atomic(axi_dma_ioread64, val,
		!(val & chan_shift), 100, 50000, false, chan->chip, DMAC_CHEN);
}

static inline void axi_chan_enable(struct axi_dma_chan *chan)
{
	u64 val;

	val = axi_dma_ioread64(chan->chip, DMAC_CHEN);
	if (chan->chip->dw->hdata->reg_map_8_channels)
		val |= BIT(chan->id) << DMAC_CHAN_EN_SHIFT |
			BIT(chan->id) << DMAC_CHAN_EN_WE_SHIFT;
	else
		val |= BIT(chan->id) << DMAC_CHAN_EN_SHIFT |
			BIT(chan->id) << DMAC_CHAN_EN2_WE_SHIFT;
	axi_dma_iowrite64(chan->chip, DMAC_CHEN, val);
}

static inline bool axi_chan_is_hw_enable(struct axi_dma_chan *chan)
{
	u64 val;

	u8 id = chan->id;

	if (unlikely(id >= 16)) {
		id += 16;
	}

	val = axi_dma_ioread64(chan->chip, DMAC_CHEN);

	return !!(val & (BIT(id) << DMAC_CHAN_EN_SHIFT));
}

static void axi_dma_hw_init(struct axi_dma_chip *chip)
{
	int ret = 0;
	u32 i;

	for (i = 0; i < chip->dw->hdata->nr_channels; i++) {
		axi_chan_irq_disable(&chip->dw->chan[i], DWAXIDMAC_IRQ_ALL);
		axi_chan_disable(&chip->dw->chan[i]);
	}

	ret = dma_set_mask_and_coherent(chip->dev, DMA_BIT_MASK(64));
	if (ret)
		dev_warn(chip->dev, "Unable to set coherent mask\n");
}

static u32 axi_chan_get_xfer_width(struct axi_dma_chan *chan, dma_addr_t src,
				   dma_addr_t dst, size_t len)
{
	u32 max_width = chan->chip->dw->hdata->m_data_width;

	return __ffs(src | dst | len | BIT(max_width));
}

static inline const char *axi_chan_name(struct axi_dma_chan *chan)
{
	return dma_chan_name(&chan->vc.chan);
}

static struct axi_dma_desc *axi_desc_alloc(u32 num)
{
	struct axi_dma_desc *desc;

	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->hw_desc = kcalloc(num, sizeof(*desc->hw_desc), GFP_NOWAIT);
	if (!desc->hw_desc) {
		kfree(desc);
		return NULL;
	}

	return desc;
}

static struct axi_dma_lli *axi_desc_get(struct axi_dma_chan *chan,
					dma_addr_t *addr)
{
	struct axi_dma_lli *lli;
	dma_addr_t phys;

	lli = dma_pool_zalloc(chan->desc_pool, GFP_NOWAIT, &phys);
	if (unlikely(!lli)) {
		dev_err(chan2dev(chan), "%s: not enough descriptors available\n",
			axi_chan_name(chan));
		return NULL;
	}

	atomic_inc(&chan->descs_allocated);
	*addr = phys;

	return lli;
}

static void axi_desc_put(struct axi_dma_desc *desc)
{
	struct axi_dma_chan *chan = desc->chan;
	int count = atomic_read(&chan->descs_allocated);
	struct axi_dma_hw_desc *hw_desc;
	int descs_put;
	unsigned long flags;

	for (descs_put = 0; descs_put < count; descs_put++) {
		hw_desc = &desc->hw_desc[descs_put];
		if (hw_desc->lli && hw_desc->llp)
			dma_pool_free(chan->desc_pool, hw_desc->lli, hw_desc->llp);
	}

	spin_lock_irqsave(&chan->vc.lock, flags);
	chan->vc.cyclic = NULL;
	kfree(desc->hw_desc);
	kfree(desc);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
	atomic_sub(descs_put, &chan->descs_allocated);
	dev_vdbg(chan2dev(chan), "%s: %d descs put, %d still allocated\n",
		axi_chan_name(chan), descs_put,
		atomic_read(&chan->descs_allocated));
}

static void vchan_desc_put(struct virt_dma_desc *vdesc)
{
	axi_desc_put(vd_to_axi_desc(vdesc));
}

static enum dma_status
dma_chan_tx_status(struct dma_chan *dchan, dma_cookie_t cookie,
		  struct dma_tx_state *txstate)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	struct virt_dma_desc *vdesc;
	struct axi_dma_desc *desc;
	enum dma_status status;
	u32 completed_length = 0;
	unsigned long flags;
	u32 completed_blocks;
	size_t bytes = 0;
	u32 length;

	status = dma_cookie_status(dchan, cookie, txstate);

	if (status == DMA_COMPLETE || !txstate)
		return status;

	spin_lock_irqsave(&chan->vc.lock, flags);

	vdesc = vchan_find_desc(&chan->vc, cookie);
	desc = vd_to_axi_desc(vdesc);
	if (vdesc) {
		int i;
		length = desc->length;
		completed_blocks = desc->completed_blocks;
		for (i = 0; i < completed_blocks; i++) {
			if (NULL != desc->hw_desc[i].lli)
				completed_length += desc->hw_desc[i].len;
		}

		bytes = length - completed_length;
	} else {
		bytes = desc->length;
	}

	spin_unlock_irqrestore(&chan->vc.lock, flags);
	dma_set_residue(txstate, bytes);

	return status;
}

static void write_desc_llp(struct axi_dma_hw_desc *desc, dma_addr_t adr)
{
	desc->lli->llp = cpu_to_le64(adr);
}

static void write_chan_llp(struct axi_dma_chan *chan, dma_addr_t adr)
{
	axi_chan_iowrite64(chan, CH_LLP, adr);
}

/* Called in chan locked context */
static void axi_chan_block_xfer_start(struct axi_dma_chan *chan,
					  struct axi_dma_desc *first)
{
	u32 priority = chan->chip->dw->hdata->priority[chan->id];
	struct axi_dma_chan_config config = {};
	u64 irq_mask;
	u8 lms = 0; /* Select AXI0 master for LLI fetching */

	if (unlikely(axi_chan_is_hw_enable(chan))) {
		dev_err(chan2dev(chan), "%s is non-idle!\n",
			axi_chan_name(chan));

		return;
	}

	axi_dma_enable(chan->chip);

	config.dst_multblk_type = DWAXIDMAC_MBLK_TYPE_LL;
	config.src_multblk_type = DWAXIDMAC_MBLK_TYPE_LL;
	config.tt_fc = DWAXIDMAC_TT_FC_MEM_TO_MEM_DMAC;
	config.prior = priority;
	config.hs_sel_dst = DWAXIDMAC_HS_SEL_HW;
	config.hs_sel_src = DWAXIDMAC_HS_SEL_HW;
	switch (chan->direction) {
	case DMA_MEM_TO_DEV:
		config.tt_fc = chan->config.device_fc ?
				DWAXIDMAC_TT_FC_MEM_TO_PER_DST :
				DWAXIDMAC_TT_FC_MEM_TO_PER_DMAC;
		config.dst_per = chan->hw_handshake_num;
		break;
	case DMA_DEV_TO_MEM:
		config.tt_fc = chan->config.device_fc ?
				DWAXIDMAC_TT_FC_PER_TO_MEM_SRC :
				DWAXIDMAC_TT_FC_PER_TO_MEM_DMAC;

		config.src_per = chan->hw_handshake_num;
		break;
	default:
		break;
	}
	axi_chan_config_write(chan, &config);

	write_chan_llp(chan, first->hw_desc[0].llp | lms);

	irq_mask = DWAXIDMAC_IRQ_DMA_TRF | DWAXIDMAC_IRQ_ALL_ERR;
	if (chan->chip->dma_ringbuff_enable)
		irq_mask |= DWAXIDMAC_IRQ_BLOCK_TRF;

	axi_chan_irq_sig_set(chan, irq_mask);

	/* Generate 'suspend' status but don't generate interrupt */
	irq_mask |= DWAXIDMAC_IRQ_SUSPENDED;
	axi_chan_irq_set(chan, irq_mask);

	axi_chan_enable(chan);
}

static void axi_chan_start_first_queued(struct axi_dma_chan *chan)
{
	struct axi_dma_desc *desc;
	struct virt_dma_desc *vd;

	vd = vchan_next_desc(&chan->vc);
	if (!vd)
		return;

	desc = vd_to_axi_desc(vd);
	dev_vdbg(chan2dev(chan), "%s: started %u\n", axi_chan_name(chan),
		vd->tx.cookie);
	axi_chan_block_xfer_start(chan, desc);
}

static void dma_chan_issue_pending(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (vchan_issue_pending(&chan->vc))
		axi_chan_start_first_queued(chan);
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static void dw_axi_dma_synchronize(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);

	vchan_synchronize(&chan->vc);
}

static int dma_chan_alloc_chan_resources(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);

#if 0
	/* ASSERT: channel is idle */
	if (axi_chan_is_hw_enable(chan)) {
		dev_err(chan2dev(chan), "%s is non-idle!\n",
			axi_chan_name(chan));
		return -EBUSY;
	}
#endif

	/* LLI address must be aligned to a 64-byte boundary */
	chan->desc_pool = dma_pool_create(dev_name(chan2dev(chan)),
					  chan->chip->dev,
					  sizeof(struct axi_dma_lli),
					  64, 0);
	if (!chan->desc_pool) {
		dev_err(chan2dev(chan), "No memory for descriptors\n");
		return -ENOMEM;
	}
	dev_vdbg(dchan2dev(dchan), "%s: allocating\n", axi_chan_name(chan));

	pm_runtime_get(chan->chip->dev);

	return 0;
}

static void dma_chan_free_chan_resources(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);

	/* ASSERT: channel is idle */
	if (axi_chan_is_hw_enable(chan))
		dev_err(dchan2dev(dchan), "%s is non-idle!\n",
			axi_chan_name(chan));

	axi_chan_disable(chan);
	axi_chan_irq_disable(chan, DWAXIDMAC_IRQ_ALL);

	vchan_free_chan_resources(&chan->vc);

	dma_pool_destroy(chan->desc_pool);
	chan->desc_pool = NULL;
	dev_vdbg(dchan2dev(dchan),
		 "%s: free resources, descriptor still allocated: %u\n",
		 axi_chan_name(chan), atomic_read(&chan->descs_allocated));

	pm_runtime_put(chan->chip->dev);
}

/*
 * If DW_axi_dmac sees CHx_CTL.ShadowReg_Or_LLI_Last bit of the fetched LLI
 * as 1, it understands that the current block is the final block in the
 * transfer and completes the DMA transfer operation at the end of current
 * block transfer.
 */
static void set_desc_last(struct axi_dma_hw_desc *desc)
{
	u32 val;

	val = le32_to_cpu(desc->lli->ctl_hi);
	val |= CH_CTL_H_LLI_LAST;
	desc->lli->ctl_hi = cpu_to_le32(val);
}

static void write_desc_sar(struct axi_dma_hw_desc *desc, dma_addr_t adr)
{
	desc->lli->sar = cpu_to_le64(adr);
}

static void write_desc_dar(struct axi_dma_hw_desc *desc, dma_addr_t adr)
{
	desc->lli->dar = cpu_to_le64(adr);
}

#if 0
static void set_desc_src_master(struct axi_dma_hw_desc *desc)
{
	u32 val;

	/* Select AXI0 for source master */
	val = le32_to_cpu(desc->lli->ctl_lo);
	val &= ~CH_CTL_L_SRC_MAST;
	desc->lli->ctl_lo = cpu_to_le32(val);
}

static void set_desc_dest_master(struct axi_dma_hw_desc *hw_desc,
				 struct axi_dma_desc *desc)
{
	u32 val;

	/* Select AXI1 for source master if available */
	val = le32_to_cpu(hw_desc->lli->ctl_lo);
	if (desc->chan->chip->dw->hdata->nr_masters > 1)
		val |= CH_CTL_L_DST_MAST;
	else
		val &= ~CH_CTL_L_DST_MAST;

	hw_desc->lli->ctl_lo = cpu_to_le32(val);
}
#endif

static int dw_axi_dma_set_hw_desc(struct axi_dma_chan *chan,
				  struct axi_dma_hw_desc *hw_desc,
				  dma_addr_t mem_addr, size_t len)
{
	unsigned int data_width = BIT(chan->chip->dw->hdata->m_data_width);
	unsigned int reg_width;
	unsigned int mem_width;
	dma_addr_t device_addr;
	size_t axi_block_ts;
	size_t block_ts;
	u32 ctllo, ctlhi;
	u32 burst_len;

	axi_block_ts = chan->chip->dw->hdata->block_size[chan->id];

	mem_width = __ffs(data_width | mem_addr | len);
	if (mem_width > DWAXIDMAC_TRANS_WIDTH_32)
		mem_width = DWAXIDMAC_TRANS_WIDTH_32;

	if (!IS_ALIGNED(mem_addr, 4)) {
		dev_err(chan->chip->dev, "invalid buffer alignment\n");
		return -EINVAL;
	}

	switch (chan->direction) {
	case DMA_MEM_TO_DEV:
		reg_width = __ffs(chan->config.dst_addr_width);
		device_addr = chan->config.dst_addr;
		ctllo = reg_width << CH_CTL_L_DST_WIDTH_POS |
			mem_width << CH_CTL_L_SRC_WIDTH_POS |
			DWAXIDMAC_CH_CTL_L_NOINC << CH_CTL_L_DST_INC_POS |
			DWAXIDMAC_CH_CTL_L_INC << CH_CTL_L_SRC_INC_POS;
		block_ts = len >> mem_width;
		break;
	case DMA_DEV_TO_MEM:
		reg_width = __ffs(chan->config.src_addr_width);
		device_addr = chan->config.src_addr;
		ctllo = reg_width << CH_CTL_L_SRC_WIDTH_POS |
			mem_width << CH_CTL_L_DST_WIDTH_POS |
			DWAXIDMAC_CH_CTL_L_INC << CH_CTL_L_DST_INC_POS |
			DWAXIDMAC_CH_CTL_L_NOINC << CH_CTL_L_SRC_INC_POS;
		block_ts = len >> reg_width;
		break;
	default:
		return -EINVAL;
	}

	if (block_ts > axi_block_ts)
		return -EINVAL;

	hw_desc->lli = axi_desc_get(chan, &hw_desc->llp);
	if (unlikely(!hw_desc->lli))
		return -ENOMEM;

	ctlhi = CH_CTL_H_LLI_VALID;
	/* enable block done intr for ringbuffer mode */
	if (chan->chip->dma_ringbuff_enable)
		ctlhi |= CH_CTL_H_BLKTFR_INTR;

	if (chan->chip->dw->hdata->restrict_axi_burst_len) {
		burst_len = chan->chip->dw->hdata->axi_rw_burst_len;
		ctlhi |= CH_CTL_H_ARLEN_EN | CH_CTL_H_AWLEN_EN |
			 burst_len << CH_CTL_H_ARLEN_POS |
			 burst_len << CH_CTL_H_AWLEN_POS;
	}

	hw_desc->lli->ctl_hi = cpu_to_le32(ctlhi);

	if (chan->direction == DMA_MEM_TO_DEV) {
		write_desc_sar(hw_desc, mem_addr);
		write_desc_dar(hw_desc, device_addr);
	} else {
		write_desc_sar(hw_desc, device_addr);
		write_desc_dar(hw_desc, mem_addr);
	}

	hw_desc->lli->block_ts_lo = cpu_to_le32(block_ts - 1);

	ctllo |= DWAXIDMAC_BURST_TRANS_LEN_4 << CH_CTL_L_DST_MSIZE_POS |
		 DWAXIDMAC_BURST_TRANS_LEN_4 << CH_CTL_L_SRC_MSIZE_POS;
	hw_desc->lli->ctl_lo = cpu_to_le32(ctllo);

	//set_desc_src_master(hw_desc);

	hw_desc->len = len;
	return 0;
}

static size_t calculate_block_len(struct axi_dma_chan *chan,
				  dma_addr_t dma_addr, size_t buf_len,
				  enum dma_transfer_direction direction)
{
	u32 data_width, reg_width, mem_width;
	size_t axi_block_ts, block_len;

	axi_block_ts = chan->chip->dw->hdata->block_size[chan->id];

	switch (direction) {
	case DMA_MEM_TO_DEV:
		data_width = BIT(chan->chip->dw->hdata->m_data_width);
		mem_width = __ffs(data_width | dma_addr | buf_len);
		if (mem_width > DWAXIDMAC_TRANS_WIDTH_32)
			mem_width = DWAXIDMAC_TRANS_WIDTH_32;

		block_len = axi_block_ts << mem_width;
		break;
	case DMA_DEV_TO_MEM:
		reg_width = __ffs(chan->config.src_addr_width);
		block_len = axi_block_ts << reg_width;
		break;
	default:
		block_len = 0;
	}

	return block_len;
}

static struct dma_async_tx_descriptor *
dw_axi_dma_chan_prep_cyclic(struct dma_chan *dchan, dma_addr_t dma_addr,
				size_t buf_len, size_t period_len,
				enum dma_transfer_direction direction,
				unsigned long flags)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	struct axi_dma_hw_desc *hw_desc = NULL, *head_desc = NULL;
	struct axi_dma_hw_desc *prev_desc = NULL;
	struct axi_dma_desc *desc = NULL;
	u32 num_periods, num_segments;
	size_t axi_block_len;
	u32 total_segments;
	u32 segment_len;
	unsigned int i, j;
	int status;
	u8 lms = 0; /* Select AXI0 master for LLI fetching */

	if (unlikely(!is_slave_direction(direction)))
		return NULL;

	if (chan->chip->dma_share_enable == false) {
		pr_err("the dmac is dma_audio_disabled!\n");
		return NULL;
	}

	if (buf_len % period_len)
		return NULL;

	num_periods = buf_len / period_len;

	axi_block_len = calculate_block_len(chan, dma_addr, buf_len, direction);
	if (axi_block_len == 0)
		return NULL;

	num_segments = DIV_ROUND_UP(period_len, axi_block_len);
	segment_len = DIV_ROUND_UP(period_len, num_segments);

	total_segments = num_periods * num_segments;

	desc = axi_desc_alloc(total_segments);
	if (unlikely(!desc))
		goto err_desc_get;

	chan->direction = direction;
	desc->chan = chan;
	chan->cyclic = true;
	desc->length = 0;
	desc->period_len = period_len;
	desc->blocks = num_segments;

	hw_desc = desc->hw_desc;
	head_desc = desc->hw_desc;
	for (i = 0; i < num_periods; i++) {
		size_t len = period_len;
		dma_addr_t src_addr = dma_addr + period_len * i;

		for (j = 0; j < num_segments; j++) {
			size_t xfer_len;

			xfer_len = (len > segment_len) ? segment_len : len;


			status = dw_axi_dma_set_hw_desc(chan, hw_desc, src_addr,
							xfer_len);
			if (status < 0)
				goto err_desc_get;

			desc->length += xfer_len;

			src_addr += xfer_len;
			len -= xfer_len;

			if (prev_desc) {
				write_desc_llp(prev_desc, hw_desc->llp | lms);
			}
			prev_desc = hw_desc;
			hw_desc++;
		}

		if (!chan->chip->dma_ringbuff_enable) {
			/* Set end-of-link to the linked descriptor, so that cyclic
			* callback function can be triggered during interrupt.
			*/
			if (prev_desc) {
				set_desc_last(prev_desc);
			}
		}

	}

	if (chan->chip->dma_ringbuff_enable) {
		/* Set the last link to the header so that DMA is working on
		* ring buffer mode. DMA won't stop until call terminate. */
		write_desc_llp(prev_desc, head_desc->llp | lms);
	}

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);

err_desc_get:
	if (desc)
		axi_desc_put(desc);

	return NULL;
}

static struct dma_async_tx_descriptor *
dw_axi_dma_chan_prep_slave_sg(struct dma_chan *dchan, struct scatterlist *sgl,
				  unsigned int sg_len,
				  enum dma_transfer_direction direction,
				  unsigned long flags, void *context)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	struct axi_dma_hw_desc *hw_desc = NULL;
	struct axi_dma_desc *desc = NULL;
	u32 num_segments, segment_len;
	unsigned int loop = 0;
	struct scatterlist *sg;
	size_t axi_block_len;
	u32 len, num_sgs = 0;
	unsigned int i;
	dma_addr_t mem;
	int status;
	u64 llp = 0;
	u8 lms = 0; /* Select AXI0 master for LLI fetching */

	if (unlikely(!is_slave_direction(direction) || !sg_len))
		return NULL;

	mem = sg_dma_address(sgl);
	len = sg_dma_len(sgl);

	axi_block_len = calculate_block_len(chan, mem, len, direction);
	if (axi_block_len == 0)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i)
		num_sgs += DIV_ROUND_UP(sg_dma_len(sg), axi_block_len);

	desc = axi_desc_alloc(num_sgs);
	if (unlikely(!desc))
		goto err_desc_get;

	desc->chan = chan;
	desc->length = 0;
	chan->direction = direction;
	desc->blocks = num_sgs;

	for_each_sg(sgl, sg, sg_len, i) {
		mem = sg_dma_address(sg);
		len = sg_dma_len(sg);
		num_segments = DIV_ROUND_UP(sg_dma_len(sg), axi_block_len);
		segment_len = DIV_ROUND_UP(sg_dma_len(sg), num_segments);

		do {
			hw_desc = &desc->hw_desc[loop++];
			status = dw_axi_dma_set_hw_desc(chan, hw_desc, mem, segment_len);
			if (status < 0)
				goto err_desc_get;

			desc->length += hw_desc->len;
			len -= segment_len;
			mem += segment_len;
		} while (len >= segment_len);
	}

	/* Set end-of-link to the last link descriptor of list */
	set_desc_last(&desc->hw_desc[num_sgs - 1]);

	/* Managed transfer list */
	do {
		hw_desc = &desc->hw_desc[--num_sgs];
		write_desc_llp(hw_desc, llp | lms);
		llp = hw_desc->llp;
	} while (num_sgs);

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);

err_desc_get:
	if (desc)
		axi_desc_put(desc);

	return NULL;
}

static struct dma_async_tx_descriptor *
dma_chan_prep_dma_memcpy(struct dma_chan *dchan, dma_addr_t dst_adr,
			 dma_addr_t src_adr, size_t len, unsigned long flags)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	size_t block_ts, max_block_ts, xfer_len;
	struct axi_dma_hw_desc *hw_desc = NULL;
	struct axi_dma_desc *desc = NULL;
	u32 xfer_width, reg, num;
	u64 llp = 0;
	u8 lms = 0; /* Select AXI0 master for LLI fetching */

	dev_dbg(chan2dev(chan), "%s: memcpy: src: %pad dst: %pad length: %zd flags: %#lx",
		axi_chan_name(chan), &src_adr, &dst_adr, len, flags);

	max_block_ts = chan->chip->dw->hdata->block_size[chan->id];
	xfer_width = axi_chan_get_xfer_width(chan, src_adr, dst_adr, len);
	num = DIV_ROUND_UP(len, max_block_ts << xfer_width);
	desc = axi_desc_alloc(num);
	if (unlikely(!desc))
		goto err_desc_get;

	desc->chan = chan;
	num = 0;
	desc->length = 0;
	while (len) {
		xfer_len = len;

		hw_desc = &desc->hw_desc[num];
		/*
		 * Take care for the alignment.
		 * Actually source and destination widths can be different, but
		 * make them same to be simpler.
		 */
		xfer_width = axi_chan_get_xfer_width(chan, src_adr, dst_adr, xfer_len);

		/*
		 * block_ts indicates the total number of data of width
		 * to be transferred in a DMA block transfer.
		 * BLOCK_TS register should be set to block_ts - 1
		 */
		block_ts = xfer_len >> xfer_width;
		if (block_ts > max_block_ts) {
			block_ts = max_block_ts;
			xfer_len = max_block_ts << xfer_width;
		}

		hw_desc->lli = axi_desc_get(chan, &hw_desc->llp);
		if (unlikely(!hw_desc->lli))
			goto err_desc_get;

		write_desc_sar(hw_desc, src_adr);
		write_desc_dar(hw_desc, dst_adr);
		hw_desc->lli->block_ts_lo = cpu_to_le32(block_ts - 1);

		reg = CH_CTL_H_LLI_VALID;
		if (chan->chip->dw->hdata->restrict_axi_burst_len) {
			u32 burst_len = chan->chip->dw->hdata->axi_rw_burst_len;

			reg |= (CH_CTL_H_ARLEN_EN |
				burst_len << CH_CTL_H_ARLEN_POS |
				CH_CTL_H_AWLEN_EN |
				burst_len << CH_CTL_H_AWLEN_POS);
		}
		hw_desc->lli->ctl_hi = cpu_to_le32(reg);

		reg = (DWAXIDMAC_BURST_TRANS_LEN_4 << CH_CTL_L_DST_MSIZE_POS |
			   DWAXIDMAC_BURST_TRANS_LEN_4 << CH_CTL_L_SRC_MSIZE_POS |
			   xfer_width << CH_CTL_L_DST_WIDTH_POS |
			   xfer_width << CH_CTL_L_SRC_WIDTH_POS |
			   DWAXIDMAC_CH_CTL_L_INC << CH_CTL_L_DST_INC_POS |
			   DWAXIDMAC_CH_CTL_L_INC << CH_CTL_L_SRC_INC_POS);
		hw_desc->lli->ctl_lo = cpu_to_le32(reg);

		// set_desc_src_master(hw_desc);
		// set_desc_dest_master(hw_desc, desc);

		hw_desc->len = xfer_len;
		desc->length += hw_desc->len;
		/* update the length and addresses for the next loop cycle */
		len -= xfer_len;
		dst_adr += xfer_len;
		src_adr += xfer_len;
		num++;
	}

	desc->blocks = num;
	/* Total len of src/dest sg == 0, so no descriptor were allocated */
	if (unlikely(!desc))
		return NULL;

	/* Set end-of-link to the last link descriptor of list */
	set_desc_last(&desc->hw_desc[num - 1]);
	/* Managed transfer list */
	do {
		hw_desc = &desc->hw_desc[--num];
		write_desc_llp(hw_desc, llp | lms);
		llp = hw_desc->llp;
	} while (num);

	return vchan_tx_prep(&chan->vc, &desc->vd, flags);

err_desc_get:
	if (desc)
		axi_desc_put(desc);
	return NULL;
}

static int dw_axi_dma_chan_slave_config(struct dma_chan *dchan,
					struct dma_slave_config *config)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	struct dw_axi_dma *dw = chan->chip->dw;

	/* Check if chan will be configured for slave transfers */
	if (unlikely(!(dw->dma.directions & BIT(config->direction))))
		return -EINVAL;

	if (config->slave_id >= chan->chip->dw->hdata->hw_handshake_num) {
		dev_err(chan2dev(chan), "slave_id is out of bounds\n");
		return -EINVAL;
	}

	if (config->dst_addr_width == DMA_SLAVE_BUSWIDTH_3_BYTES) {
		dev_err(dchan2dev(dchan), "dst_width is not support 3bytes, fix to 4bytes\n");
		return -EINVAL;
	}

	if (config->src_addr_width == DMA_SLAVE_BUSWIDTH_3_BYTES) {
		dev_err(dchan2dev(dchan), "src_width is not support 3bytes, fix to 4bytes\n");
		return -EINVAL;
	}

	memcpy(&chan->config, config, sizeof(*config));

	return 0;
}

static void axi_chan_dump_lli(struct axi_dma_chan *chan,
				  struct axi_dma_hw_desc *desc)
{
	dev_err(dchan2dev(&chan->vc.chan),
		"SAR: 0x%llx DAR: 0x%llx LLP: 0x%llx BTS 0x%x CTL: 0x%x:%08x",
		le64_to_cpu(desc->lli->sar),
		le64_to_cpu(desc->lli->dar),
		le64_to_cpu(desc->lli->llp),
		le32_to_cpu(desc->lli->block_ts_lo),
		le32_to_cpu(desc->lli->ctl_hi),
		le32_to_cpu(desc->lli->ctl_lo));
}

static void axi_chan_list_dump_lli(struct axi_dma_chan *chan,
				   struct axi_dma_desc *desc_head)
{
	int count = atomic_read(&chan->descs_allocated);
	int i;

	for (i = 0; i < count; i++)
		axi_chan_dump_lli(chan, &desc_head->hw_desc[i]);
}

static noinline void axi_chan_handle_err(struct axi_dma_chan *chan, u32 status)
{
	struct virt_dma_desc *vd;
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);

	axi_chan_disable(chan);

	/* The bad descriptor currently is in the head of vc list */
	vd = vchan_next_desc(&chan->vc);
	if (NULL == vd)
		goto unlock;

	/* Remove the completed descriptor from issued list */
	list_del(&vd->node);

	/* WARN about bad descriptor */
	dev_err(chan2dev(chan),
		"Bad descriptor submitted for %s, cookie: %d, irq: 0x%08x\n",
		axi_chan_name(chan), vd->tx.cookie, status);
	axi_chan_list_dump_lli(chan, vd_to_axi_desc(vd));

	vchan_cookie_complete(vd);

unlock:
	/* Try to restart the controller */
	axi_chan_start_first_queued(chan);

	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static void axi_chan_rb_block_xfer_complete(struct axi_dma_chan *chan)
{
	struct virt_dma_desc *vd;
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);

	/* The completed descriptor currently is in the head of vc list */
	vd = vchan_next_desc(&chan->vc);
	if (NULL == vd)
		goto unlock;

	if (chan->cyclic) {
		vchan_cyclic_callback(vd);
	} else {
		/* Remove the completed descriptor from issued list before completing */
		list_del(&vd->node);
		vchan_cookie_complete(vd);
		/* Submit queued descriptors after processing the completed ones */
		axi_chan_start_first_queued(chan);
	}

unlock:
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static void axi_chan_block_xfer_complete(struct axi_dma_chan *chan)
{
	struct virt_dma_desc *vd;
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);
	if (unlikely(axi_chan_is_hw_enable(chan))) {
		dev_err(chan2dev(chan), "BUG: %s caught DWAXIDMAC_IRQ_DMA_TRF, but channel not idle!\n",
			axi_chan_name(chan));
		axi_chan_disable(chan);
	}

	/* The completed descriptor currently is in the head of vc list */
	vd = vchan_next_desc(&chan->vc);
	if (NULL == vd)
		goto unlock;

	if (chan->cyclic) {
		struct axi_dma_desc *desc;

		desc = vd_to_axi_desc(vd);
		if (desc) {
			u64 llp;
			struct axi_dma_hw_desc *hw_desc;

			llp = axi_chan_ioread64(chan, CH_LLP);

			if (llp)
				desc->completed_blocks += desc->blocks;
			else
				desc->completed_blocks = 0;

			hw_desc = &desc->hw_desc[desc->completed_blocks];
			write_chan_llp(chan, hw_desc->llp);
			axi_chan_enable(chan);

			vchan_cyclic_callback(vd);
		}
	} else {
		/* Remove the completed descriptor from issued list before completing */
		list_del(&vd->node);
		vchan_cookie_complete(vd);

		/* Submit queued descriptors after processing the completed ones */
		axi_chan_start_first_queued(chan);
	}

unlock:
	spin_unlock_irqrestore(&chan->vc.lock, flags);
}

static irqreturn_t dw_axi_dma_interrupt(int irq, void *dev_id)
{
	u32 i;
	u64 status;
	struct axi_dma_chip *chip = dev_id;
	struct dw_axi_dma *dw = chip->dw;
	struct axi_dma_chan *chan;

	/* Disable DMAC inerrupts. We'll enable them after processing chanels */
	axi_dma_irq_disable(chip);
	/* Poll, clear and process every chanel interrupt status */
	for (i = 0; i < dw->hdata->nr_channels; i++) {
		chan = &dw->chan[i];
		status = axi_chan_irq_read(chan);
		axi_chan_irq_clear(chan, status);

		dev_vdbg(chip->dev, "%s %d IRQ status: 0x%08llx\n",
			axi_chan_name(chan), i, status);

		if (status & DWAXIDMAC_IRQ_ALL_ERR)
			axi_chan_handle_err(chan, status);
		else if (status & DWAXIDMAC_IRQ_DMA_TRF)
			axi_chan_block_xfer_complete(chan);
		else if (status & DWAXIDMAC_IRQ_BLOCK_TRF)
			axi_chan_rb_block_xfer_complete(chan);

	}

	/* Re-enable interrupts */
	axi_dma_irq_enable(chip);
	return IRQ_HANDLED;
}

static int dma_chan_terminate_all(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vc.lock, flags);

	axi_chan_disable(chan);

	vchan_get_all_descriptors(&chan->vc, &head);

	chan->cyclic = false;
	spin_unlock_irqrestore(&chan->vc.lock, flags);

	vchan_dma_desc_free_list(&chan->vc, &head);

	dev_vdbg(dchan2dev(dchan), "terminated: %s\n", axi_chan_name(chan));

	return 0;
}

static int dma_chan_pause(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	unsigned long flags;
	unsigned int timeout = 20; /* timeout iterations */
	u64 val;

	spin_lock_irqsave(&chan->vc.lock, flags);

	if (chan->chip->dw->hdata->reg_map_8_channels) {
		val = axi_dma_ioread64(chan->chip, DMAC_CHEN);
		val |= BIT(chan->id) << DMAC_CHAN_SUSP_SHIFT |
			BIT(chan->id) << DMAC_CHAN_SUSP_WE_SHIFT;
		axi_dma_iowrite64(chan->chip, DMAC_CHEN, val);
	} else {
		u8 id = chan->id;
		u64 tmp;

		val = axi_dma_ioread64(chan->chip, DMAC_CHSUSPREG);
		tmp = BIT(id) << DMAC_CHAN_SUSP2_SHIFT |
		      BIT(id) << DMAC_CHAN_SUSP2_WE_SHIFT;

		if (id < 16) {
			tmp = BIT(id) << DMAC_CHAN_SUSP2_SHIFT |
				   BIT(id) << DMAC_CHAN_SUSP2_WE_SHIFT;
		} else {
			id -= 16;
			tmp = BIT(id) << DMAC_CHAN_SUSP2_SHIFT |
				   BIT(id) << DMAC_CHAN_SUSP2_WE_SHIFT;
			tmp <<= 32;
		}

		val |= tmp;

		axi_dma_iowrite64(chan->chip, DMAC_CHSUSPREG, val);
	}

	do {
		if (axi_chan_irq_read(chan) & DWAXIDMAC_IRQ_SUSPENDED)
			break;

		udelay(2);
	} while (--timeout);

	axi_chan_irq_clear(chan, DWAXIDMAC_IRQ_SUSPENDED);

	chan->is_paused = true;

	spin_unlock_irqrestore(&chan->vc.lock, flags);

	return timeout ? 0 : -EAGAIN;
}

/* Called in chan locked context */
static inline void axi_chan_resume(struct axi_dma_chan *chan)
{
	u64 val;

	if (chan->chip->dw->hdata->reg_map_8_channels) {
		val = axi_dma_ioread64(chan->chip, DMAC_CHEN);
		val &= ~(BIT(chan->id) << DMAC_CHAN_SUSP_SHIFT);
		val |=  (BIT(chan->id) << DMAC_CHAN_SUSP_WE_SHIFT);
		axi_dma_iowrite64(chan->chip, DMAC_CHEN, val);
	} else {
		u64 tmp;
		u8 id = chan->id;
		val = axi_dma_ioread64(chan->chip, DMAC_CHSUSPREG);

		if (id < 16) {
			val &= ~(BIT(id) << DMAC_CHAN_SUSP2_SHIFT);
			val |=  (BIT(id) << DMAC_CHAN_SUSP2_WE_SHIFT);
		} else {
			id -= 16;
			tmp = (BIT(id) << DMAC_CHAN_SUSP2_SHIFT) << 32;
			val &= ~tmp;
			tmp = (BIT(id) << DMAC_CHAN_SUSP2_WE_SHIFT) << 32;
			val |= tmp;
		}

		axi_dma_iowrite64(chan->chip, DMAC_CHSUSPREG, val);
	}

	chan->is_paused = false;
}

static int dma_chan_resume(struct dma_chan *dchan)
{
	struct axi_dma_chan *chan = dchan_to_axi_dma_chan(dchan);
	unsigned long flags;

	spin_lock_irqsave(&chan->vc.lock, flags);

	if (chan->is_paused)
		axi_chan_resume(chan);

	spin_unlock_irqrestore(&chan->vc.lock, flags);

	return 0;
}

static int axi_dma_suspend(struct axi_dma_chip *chip)
{
/* dma_share_enable true shows cpu has dma control */
	if (chip->dma_share_enable) {
		axi_dma_irq_disable(chip);
		axi_dma_disable(chip);
	}
#if 0
	clk_disable_unprepare(chip->core_clk);
	clk_disable_unprepare(chip->cfgr_clk);
#endif
	return 0;
}

static int axi_dma_resume(struct axi_dma_chip *chip)
{
#if 0
	int ret;

	ret = clk_prepare_enable(chip->cfgr_clk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(chip->core_clk);
	if (ret < 0)
		return ret;
#endif
/* dma_share_enable true shows cpu has dma control */
	if (chip->dma_share_enable) {
		axi_dma_enable(chip);
		axi_dma_irq_enable(chip);
	}

	return 0;
}

static int __maybe_unused axi_dma_runtime_suspend(struct device *dev)
{
	struct axi_dma_chip *chip = dev_get_drvdata(dev);

	return axi_dma_suspend(chip);
}

static int __maybe_unused axi_dma_runtime_resume(struct device *dev)
{
	struct axi_dma_chip *chip = dev_get_drvdata(dev);

	return axi_dma_resume(chip);
}

#ifdef CONFIG_PM_SLEEP
static int axi_dma_pm_suspend(struct device *dev)
{
	struct axi_dma_chip *chip = dev_get_drvdata(dev);

	return axi_dma_suspend(chip);
}

static int axi_dma_pm_resume(struct device *dev)
{
	struct axi_dma_chip *chip = dev_get_drvdata(dev);

	return axi_dma_resume(chip);
}
#else
#define axi_dma_pm_suspend	NULL
#define axi_dma_pm_resume	NULL
#endif /* CONFIG_PM_SLEEP */


static struct dma_chan *dw_axi_dma_of_xlate(struct of_phandle_args *dma_spec,
						struct of_dma *ofdma)
{
	struct dw_axi_dma *dw = ofdma->of_dma_data;
	struct axi_dma_chan *chan;
	struct dma_chan *dchan;

	dchan = dma_get_any_slave_channel(&dw->dma);
	if (!dchan)
		return NULL;

	chan = dchan_to_axi_dma_chan(dchan);
	chan->hw_handshake_num = dma_spec->args[0];
	return dchan;
}

static int parse_device_properties(struct axi_dma_chip *chip)
{
	int ret;
	struct device *dev = chip->dev;
	u32 tmp, carr[DMAC_MAX_CHANNELS];
	struct dw_axi_dma_hcfg *hdata = chip->dw->hdata;

	ret = device_property_read_u32(dev, "dma-channels", &tmp);
	if (ret)
		return ret;
	if (tmp == 0 || tmp > DMAC_MAX_CHANNELS)
		return -EINVAL;

	hdata->nr_channels = tmp;
	if (tmp <= DMA_REG_MAP_CH_REF)
		hdata->reg_map_8_channels = true;

	ret = device_property_read_u32(dev, "snps,dma-masters", &tmp);
	if (ret)
		return ret;
	if (tmp == 0 || tmp > DMAC_MAX_MASTERS)
		return -EINVAL;

	hdata->nr_masters = tmp;

	ret = device_property_read_u32(dev, "snps,data-width", &tmp);
	if (ret)
		return ret;
	if (tmp > DWAXIDMAC_TRANS_WIDTH_MAX)
		return -EINVAL;

	hdata->m_data_width = tmp;

	ret = device_property_read_u32_array(dev, "snps,block-size", carr,
						 hdata->nr_channels);
	if (ret)
		return ret;
	for (tmp = 0; tmp < hdata->nr_channels; tmp++) {
		if (carr[tmp] == 0 || carr[tmp] > DMAC_MAX_BLK_SIZE)
			return -EINVAL;

		hdata->block_size[tmp] = carr[tmp];
	}

	ret = device_property_read_u32_array(dev, "snps,priority", carr,
						 hdata->nr_channels);
	if (ret)
		return ret;
	/* Priority value must be programmed within [0:nr_channels-1] range */
	for (tmp = 0; tmp < hdata->nr_channels; tmp++) {
		if (carr[tmp] >= hdata->nr_channels)
			return -EINVAL;

		hdata->priority[tmp] = carr[tmp];
	}

	/* axi-max-burst-len is optional property */
	ret = device_property_read_u32(dev, "snps,axi-max-burst-len", &tmp);
	if (!ret) {
		if (tmp > DWAXIDMAC_ARWLEN_MAX + 1)
			return -EINVAL;
		if (tmp < DWAXIDMAC_ARWLEN_MIN + 1)
			return -EINVAL;

		hdata->restrict_axi_burst_len = true;
		hdata->axi_rw_burst_len = tmp;
	}

	ret = device_property_read_u32(dev, "snps,handshake-num", &tmp);
	if (ret)
		return ret;
	hdata->hw_handshake_num = tmp;

	/* some module only support soft handshake */
	hdata->handshake_softonly = device_property_read_bool(dev,
								"snps,handshake-soft-only");

	return 0;
}

static int se1000_dma_rx_cb(struct rpmsg_device *rpmsg_dev, void *data, int len, void *priv, u32 src)
{
	struct rpmsg_payload *payload;
	struct axi_dma_chip *chip = priv;

	payload = (struct rpmsg_payload *)data;

	switch (payload->type) {
	case RPMSG_UCP_TO_KAP:
		/*
		 * ap kernel space receive rpmsg from cp user space
		 * ap disable irq & set dma_share_enable false
		 * ap kernel space send rpmsg to cp kernel space
		 */
		disable_irq(chip->irq);
		chip->dma_share_enable = false;
		payload->type = RPMSG_KAP_TO_KCP;
		rpmsg_send(chip->ept, (void *)payload, sizeof(struct rpmsg_payload));
		break;
	case RPMSG_KAP_TO_KCP:
		/*
		 * cp kernel space receive rpmsg from ap kernel space
		 * cp enable irq & set dma_share_enable true
		 * cp init dma hw
		 */
		chip->dma_share_enable = true;
		enable_irq(chip->irq);
		axi_dma_hw_init(chip);
		break;
	case RPMSG_KCP_TO_KAP:
		/*
		 * ap kernel space receive rpmsg from cp kernel space
		 * ap enable irq & set dma_share_enable true
		 * ap init dma hw
		 */
		chip->dma_share_enable = true;
		enable_irq(chip->irq);
		axi_dma_hw_init(chip);
		break;
	case RPMSG_UAP_TO_KCP:
		/*
		 * cp kernel space receive rpmsg from ap user space
		 * cp disable irq & set dma_share_enable false
		 * cp kernel space send rpmsg to ap kernel space
		 */
		disable_irq(chip->irq);
		chip->dma_share_enable = false;
		payload->type = RPMSG_KCP_TO_KAP;
		rpmsg_send(chip->ept, (void *)payload, sizeof(struct rpmsg_payload));
		break;
	default:
		break;
	}

	return 0;
}

static int dw_axi_rpmsg_receive(struct axi_dma_chip *chip, struct device *dev)
{
	struct rpmsg_device *rpdev = NULL;
	struct rpmsg_channel_info ept_info = {
		.src = CLUSTER_RPMSG_ID_AUD,
		.dst = CLUSTER_RPMSG_ID_AUD,
		.name = "dw_axi_rpmsg",
	};

	rpdev = find_rpmsg_device_by_phandle(dev->of_node);
	if (!rpdev) {
		dev_err(dev, "find_rpmsg_device_by_phandle failed\n");
		return -EINVAL;
	}

	chip->ept = rpmsg_create_ept(rpdev, se1000_dma_rx_cb, chip, ept_info);
	if (!chip->ept) {
		dev_err(dev, "rpmsg_create_ept failed\n");
		return -EPROBE_DEFER;
	}

	return 0;
}

static int dw_probe(struct platform_device *pdev)
{
	struct axi_dma_chip *chip;
	struct resource *mem;
	struct dw_axi_dma *dw;
	struct dw_axi_dma_hcfg *hdata;
	const char *int_name;
	u32 i;
	int ret;
	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	dw = devm_kzalloc(&pdev->dev, sizeof(*dw), GFP_KERNEL);
	if (!dw)
		return -ENOMEM;

	hdata = devm_kzalloc(&pdev->dev, sizeof(*hdata), GFP_KERNEL);
	if (!hdata)
		return -ENOMEM;

	chip->dw = dw;
	chip->dev = &pdev->dev;
	chip->dw->hdata = hdata;

	chip->irq = platform_get_irq(pdev, 0);
	if (chip->irq < 0)
		return chip->irq;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chip->regs = devm_ioremap_resource(chip->dev, mem);
	if (IS_ERR(chip->regs))
		return PTR_ERR(chip->regs);

	axi_dma_irq_disable(chip);

#if 0
	chip->core_clk = devm_clk_get(chip->dev, "core-clk");
	if (IS_ERR(chip->core_clk))
		return PTR_ERR(chip->core_clk);

	chip->cfgr_clk = devm_clk_get(chip->dev, "cfgr-clk");
	if (IS_ERR(chip->cfgr_clk))
		return PTR_ERR(chip->cfgr_clk);
#endif
	ret = parse_device_properties(chip);
	if (ret)
		return ret;

	dw->chan = devm_kcalloc(chip->dev, hdata->nr_channels,
				sizeof(*dw->chan), GFP_KERNEL);
	if (!dw->chan)
		return -ENOMEM;

	ret = device_property_read_string(&pdev->dev, "interrupt-names", &int_name);
	if (ret < 0)
		int_name = KBUILD_MODNAME;

	ret = devm_request_irq(chip->dev, chip->irq, dw_axi_dma_interrupt,
				   IRQF_TRIGGER_HIGH, int_name, chip);
	if (ret)
		return ret;

	disable_irq(chip->irq);

	INIT_LIST_HEAD(&dw->dma.channels);
	for (i = 0; i < hdata->nr_channels; i++) {
		struct axi_dma_chan *chan = &dw->chan[i];

		chan->chip = chip;
		chan->id = i;
		chan->chan_regs = chip->regs + COMMON_REG_LEN + i * CHAN_REG_LEN;
		atomic_set(&chan->descs_allocated, 0);

		chan->vc.desc_free = vchan_desc_put;
		vchan_init(&chan->vc, &dw->dma);
	}

	/* Set capabilities */
	dma_cap_set(DMA_SLAVE, dw->dma.cap_mask);
	dma_cap_set(DMA_MEMCPY, dw->dma.cap_mask);
	if (dw->hdata->handshake_softonly == false) {
		dma_cap_set(DMA_CYCLIC, dw->dma.cap_mask);
	}

	/* DMA capabilities */
	dw->dma.chancnt = hdata->nr_channels;
	dw->dma.max_burst = hdata->axi_rw_burst_len;
	dw->dma.src_addr_widths = AXI_DMA_BUSWIDTHS;
	dw->dma.dst_addr_widths = AXI_DMA_BUSWIDTHS;
	dw->dma.directions = BIT(DMA_MEM_TO_MEM);

	if (dw->hdata->handshake_softonly == false)
		dw->dma.directions |= BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM);

	dw->dma.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;

	dw->dma.dev = chip->dev;
	dw->dma.device_tx_status = dma_chan_tx_status;
	dw->dma.device_issue_pending = dma_chan_issue_pending;
	dw->dma.device_terminate_all = dma_chan_terminate_all;
	dw->dma.device_pause = dma_chan_pause;
	dw->dma.device_resume = dma_chan_resume;

	dw->dma.device_alloc_chan_resources = dma_chan_alloc_chan_resources;
	dw->dma.device_free_chan_resources = dma_chan_free_chan_resources;

	dw->dma.device_prep_dma_memcpy = dma_chan_prep_dma_memcpy;
	dw->dma.device_synchronize = dw_axi_dma_synchronize;
	dw->dma.device_config = dw_axi_dma_chan_slave_config;

	if (dw->hdata->handshake_softonly == false) {
		dw->dma.device_prep_slave_sg = dw_axi_dma_chan_prep_slave_sg;
		dw->dma.device_prep_dma_cyclic = dw_axi_dma_chan_prep_cyclic;
	}

	/*
	 * Synopsis DesignWare AxiDMA datasheet mentioned Maximum
	 * supported blocks is 1024. Device register width is 4 bytes.
	 * Therefore, set constraint to 1024 * 4.
	 */
	dw->dma.dev->dma_parms = &dw->dma_parms;
	dma_set_max_seg_size(&pdev->dev, MAX_BLOCK_SIZE);
	platform_set_drvdata(pdev, chip);

	//pm_runtime_enable(chip->dev);

	/*
	 * We can't just call pm_runtime_get here instead of
	 * pm_runtime_get_noresume + axi_dma_resume because we need
	 * driver to work also without Runtime PM.
	 */
	//pm_runtime_get_noresume(chip->dev);
	ret = axi_dma_resume(chip);
	if (ret < 0)
		goto err_pm_disable;

	//axi_dma_hw_init(chip);

	//pm_runtime_put(chip->dev);

	ret = dma_async_device_register(&dw->dma);
	if (ret)
		goto err_pm_disable;

	/* Register with OF helpers for DMA lookups */
	ret = of_dma_controller_register(pdev->dev.of_node,
					 dw_axi_dma_of_xlate, dw);
	if (ret < 0)
		dev_warn(&pdev->dev,
			 "Failed to register OF DMA controller, fallback to MEM_TO_MEM mode\n");

	dev_info(chip->dev, "DesignWare AXI DMA Controller, %d channels\n",
		 dw->hdata->nr_channels);

	/* parse dma_share */
	chip->dma_share_enable = true;
	if (of_find_property(pdev->dev.of_node, "dma-share", NULL)) {
		of_property_read_string(pdev->dev.of_node, "dma-share", &chip->dma_share);
		if (!strcmp(chip->dma_share, "slave"))
			chip->dma_share_enable = false;
		dw_axi_rpmsg_receive(chip, &pdev->dev);
	}

	if (chip->dma_share_enable) {
		enable_irq(chip->irq);
		axi_dma_hw_init(chip);
	}

	chip->dma_ringbuff_enable = false;
	if (of_find_property(pdev->dev.of_node, "dma-ringbuff-enable", NULL)) {
		dw->dma.residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR; //don't support report dma residue
		chip->dma_ringbuff_enable = true;
	}

	dev_dbg(chip->dev, "DMA ringbuffer mode %s !\n", chip->dma_ringbuff_enable ? "enabled" : "disabled");

	axi_dma_irq_enable(chip);

	return 0;

err_pm_disable:
	//pm_runtime_disable(chip->dev);

	return ret;
}

static int dw_remove(struct platform_device *pdev)
{
	struct axi_dma_chip *chip = platform_get_drvdata(pdev);
	struct dw_axi_dma *dw = chip->dw;
	struct axi_dma_chan *chan, *_chan;
	u32 i;
#if 0
	/* Enable clk before accessing to registers */
	clk_prepare_enable(chip->cfgr_clk);
	clk_prepare_enable(chip->core_clk);
#endif
	if (chip->dma_share_enable) {
		axi_dma_irq_disable(chip);
		for (i = 0; i < dw->hdata->nr_channels; i++) {
			axi_chan_disable(&chip->dw->chan[i]);
			axi_chan_irq_disable(&chip->dw->chan[i], DWAXIDMAC_IRQ_ALL);
		}
		axi_dma_disable(chip);

		//pm_runtime_disable(chip->dev);

		devm_free_irq(chip->dev, chip->irq, chip);
	}

	of_dma_controller_free(chip->dev->of_node);

	list_for_each_entry_safe(chan, _chan, &dw->dma.channels,
			vc.chan.device_node) {
		list_del(&chan->vc.chan.device_node);
		tasklet_kill(&chan->vc.task);
	}

	return 0;
}

static const struct dev_pm_ops dw_axi_dma_pm_ops = {
	SET_RUNTIME_PM_OPS(axi_dma_runtime_suspend, axi_dma_runtime_resume, NULL)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(axi_dma_pm_suspend, axi_dma_pm_resume)
};

static const struct of_device_id dw_dma_of_id_table[] = {
	{ .compatible = "snps,axi-dma-1.01a-aud" },
	{}
};
MODULE_DEVICE_TABLE(of, dw_dma_of_id_table);

static struct platform_driver dw_driver = {
	.probe		= dw_probe,
	.remove		= dw_remove,
	.driver = {
		.name	= KBUILD_MODNAME,
		.of_match_table = of_match_ptr(dw_dma_of_id_table),
		.pm = &dw_axi_dma_pm_ops,
	},
};

static int __init dw_axi_dma_dev_init(void)
{
	return platform_driver_register(&dw_driver);
}
subsys_initcall(dw_axi_dma_dev_init);

static void __exit dw_axi_dma_dev_exit(void)
{
	platform_driver_unregister(&dw_driver);
}
module_exit(dw_axi_dma_dev_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Synopsys DesignWare 2.0a AXI DMA Controller platform driver");
MODULE_AUTHOR("Eugeniy Paltsev <Eugeniy.Paltsev@synopsys.com>");
MODULE_AUTHOR("Mingrui Zhou <Mingrui.Zhou@siengine.com>");
