/*
 * Siengine se1000 audio driver reg definition
 *
 * Copyright (c) 2022 Siengine Inc.
 * Author: jian.xu <jian.xu@siengine.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#ifndef _SE1000_AUD_REG_H_
#define _SE1000_AUD_REG_H_

#define I2S_CTRL				0x00
#define I2S_CTRL_FDX			0x04
#define I2S_SRES				0x08
#define I2S_SRES_FDR			0x0C
#define I2S_SRATE				0x10
#define I2S_STAT				0x14
#define FIFO_LEVEL				0x18
#define FIFO_AEMPTY				0x1C
#define FIFO_AFULL				0x20
#define FIFO_LEVEL_FDR			0x24
#define RX_FIFO_AEMPTY_FDR		0x28
#define RX_FIFO_AFULL_FDR		0x2C
#define TDM_CTRL				0x30
#define TDM_FD_DIR				0x34
#define I2S_FIF0_ADDR			0x40

/****** AUD_SS CLK ******/
#define AUDIO_DEVCKE_SET		0x10
#define AUDIO_DEVCKE_CLR		0x14
#define AUDIO_CLKEN				0x14
#define AUDIO_I2S_MCLK_BASE		0x60
#define AUDIO_I2S_MCLK(i)		((i*0x4) + AUDIO_I2S_MCLK_BASE)
#define AUDIO_I2S_LOOPBACK		0x80

/* AUDIO_I2S_CTRL: 0x00*/
#define I2S_LR_PACK				(0x01 << 31)
#define I2S_FIFO_AFULL_MASK		(0x01 << 30)
#define I2S_FIFO_FULL_MASK		(0x01 << 29)
#define I2S_FIFO_AEMPTY_MASK	(0x01 << 28)
#define I2S_FIFO_EMPTY_MASK		(0x01 << 27)
#define I2S_MASK				(0x01 << 26)
#define I2S_INTREQ_MASK			(0x01 << 25)
/* Transceiver clock enable 0:enable */
#define I2S_STB					(0x01 << 24)
/* 0:LSB 1:MSB ON HOST BUS*/
#define I2S_HOST_DATA_ALIGN		(0x01 << 23)
/* 0:LSB 1:MSB */
#define I2S_DATA_ORDER			(0x01 << 22)
/* 0:Left align 1:Right align */
#define I2S_DATA_ALIGN			(0x01 << 21)
#define I2S_DATA_ALIGN_SET(x)	((x) << 21)
#define I2S_DATA_WS_DELAY		(0x1F << 16)
#define I2S_DATA_WS_DEL_SET(x)	((x) << 16)
#define I2S_WS_POLAR			(0x1 << 15)
#define I2S_WS_POLAR_SET(x)		((x) << 15)
/* 0:rising edge 1:falling */
#define I2S_SCK_POLAR			(0x1 << 14)
#define I2S_SCK_POLAR_SET(x)	((x) << 14)
/* 0:mono 1:stereo */
#define I2S_AUDIO_MODE_SET(x)	((x) << 13)
#define I2S_AUDIO_MODE			(0x1 << 13)
/* 0:Left 1:Right */
#define I2S_MONO_MODE_SET(x)	((x) << 12)
#define I2S_MONO_MODE			(0x1 << 12)
#define I2S_WS_MODE				(0x0F << 8)
/* only bit 8 is relevant:0 TDM 1:I2S*/
#define I2S_WS_MODE_SET(x)		((x) << 8)
#define I2S_CHN_WIDTH			(0x07 << 5)
#define I2S_CHN_WIDTH_SET(x)	((x) << 5)
#define I2S_FIFO_RST			(0x1 << 4)
#define I2S_SFR_RST				(0x1 << 3)
/* 0:Slave 1:master */
#define I2S_MS_CFG				(0x1 << 2)
/* Hal-duplex 0:Receiver 1:Transmitter */
#define I2S_DIR_CFG				(0x1 << 1)
/* 0:enable 1:disanle */
#define I2S_EN					(0x1 << 0)

/* AUDIO_I2S_CTRL_FDX: 0x04 */
#define I2S_RFIFO_AFULL_MASK	(0x1 << 30)
#define I2S_RFIFO_FULL_MASK		(0x1 << 29)
#define I2S_RFIFO_AEMPTY_MASK	(0x1 << 28)
#define I2S_RFIFO_EMPTY_MASK	(0x1 << 27)
#define I2S_RI2S_MASK			(0x1 << 26)
/* Full-duplex mode receiver FIFO reset */
#define I2S_RFIFO_RST			(0x1 << 4)
#define I2S_FRX_EN				(0x1 << 2)
#define I2S_FTX_EN				(0x1 << 1)
/* 0:disanle 1:enable */
#define I2S_FULL_DUPLEX			(0x1 << 0)

/* AUDIO_I2S_SRES: 0x08 */
#define I2S_TX_SAMP_RES			(0x1F << 0)
/* range:0~32 */
#define I2S_TX_SAMP_RES_SET(x)	((x) << 0)

/* I2S_SRES_FDR: 0x0C */
#define I2S_RX_SAMP_RES			(0x1F << 0)
/* range:0~32 */
#define I2S_RX_SAMP_RES_SET(x)	((x) << 0)

/* I2S_SRATE: 0x10 */
#define I2S_SAMPLE_RATE			(0xFFFFF << 0)
/* sample_rate = AUD_PLL_CLK/(fs*chn_no*chn_width) */
#define I2S_SAMPLE_RATE_SET(x)	((x) << 0)

/* I2S_STAT: 0x14 */
#define I2S_STATUS_RFIFO_AFULL			(0x1 << 19)
#define I2S_STATUS_RFIFO_FULL			(0x1 << 18)
#define I2S_STATUS_RFIFO_AEMPTY			(0x1 << 17)
#define I2S_STATUS_RFIFO_EMPTY			(0x1 << 16)
#define I2S_STATUS_TFIFO_AFULL			(0x1 << 5)
#define I2S_STATUS_TFIFO_FULL			(0x1 << 4)
#define I2S_STATUS_TFIFO_AEMPTY			(0x1 << 3)
#define I2S_STATUS_TFIFO_EMPTY			(0x1 << 2)
#define I2S_STATUS_RDATA_OVERRUN		(0x1 << 1)
#define I2S_STATUS_TDATA_UNDERRUN		(0x1 << 0)

/* FIFO_LEVEL: 0x18 Ready only */

/* FIFO_AEMPTY: 0x1C */
#define AEMPTY_THRESHOLD   			(0x3F << 0)
#define AEMPTY_THRESHOLD_SET(x)		((x) << 0)
/* FIFO_AFULL: 0x20 */
#define AFULL_THRESHOLD   			(0x3F << 0)
#define AFULL_THRESHOLD_SET(x)		((x) << 0)

/* RX FIFO_AEMPTY_FDR: 0x28 */
#define RX_AEMPTY_THRESHOLD   			(0x3F << 0)
#define RX_AEMPTY_THRESHOLD_SET(x)		((x) << 0)
/* RX FIFO_AFULL_FDR: 0x2C */
#define RX_AFULL_THRESHOLD   			(0x3F << 0)
#define RX_AFULL_THRESHOLD_SET(x)		((x) << 0)

/* TDM_CTRL: 0x30 */
#define TDM_CHN_EN				(0xFFFF << 16)
#define TDM_CHN_EN_SET(x)		((x) << 16)
#define TDM_CHN_EN_CAL(x)		((1 << (x)) - 1)
#define TDM_CHN_NO				(0x0F << 1)
#define TDM_CHN_NO_SET(x)		((x) << 1)
#define TDM_EN					(0x1 << 0)

/* TDM_FD_DIR: 0x34 */
#define TDM_CHN_RXEN			(0xFFFF << 16)
#define TDM_CHN_RXEN_SET(x)		((x) << 16)
#define TDM_CHN_RXEN_CAL(x)		((1 << (x)) - 1)
#define TDM_CHN_TXEN			(0xFFFF << 0)
#define TDM_CHN_TXEN_SET(x)		((x) << 0)
#define TDM_CHN_TXEN_CAL(x)		((1 << (x)) - 1)

/* DEVCKESET: 0x10 */
#define SPDIF_CLK_EN			(0x01 << 7)
/* enable I2S0~6 */
#define I2S_CLK_EN_SET(x)		(0x01 << (x))

/* DEVCKECLR: 0x14 */
#define SPDIF_CLK_CLR			(0x01 << 7)
#define I2S_CLK_CLR				(0x7F << 0)
/* enable I2S0~6 */
#define I2S_CLK_CLR_SET(x)		((x) << 0)

/* MCLK SET: 0x60~0x78 */
#define I2S_MCLK_DIV			(0xFF << 1)
#define I2S_MCLK_DIV_SET(x)		((x) << 1)
#define I2S_MCLK_EN				(0x01 << 0)

/* loopback: 0x80 */
#define I2S_LOOPBACK_EN(x)		(0x01 << (x))
#endif
