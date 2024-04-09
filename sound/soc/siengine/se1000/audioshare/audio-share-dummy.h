/*
 * SPDX-License-Identifier: GPL-2.0.
 * sound/soc/siengine/audioshare
 * Copyright (C) 2021-2023 SiEngine or its affiliates
*/

#ifndef _AUDIO_SHARE_DUMMY_H_
#define _AUDIO_SHARE_DUMMY_H_

#define PERIOD_BYTE_MAX (4096 * 4)
#define BUFFER_BYTE_MAX (4096 * 16)
#define MAX_SHARE_CHANNELS (16)
#define MIN_SHARE_CHANNELS (1)
#define THREAD_WAIT (1)
#define MAX_SHARE_SESSIONS (11)

#define AUDIO_SHARE_RESEND_MAX (2)

struct shmem_data {
	uint32_t	write_idx __attribute__((__aligned__(4)));
	uint32_t	read_idx __attribute__((__aligned__(4)));
	uint32_t	write_flag __attribute__((__aligned__(4)));
	uint8_t	buffer[BUFFER_BYTE_MAX] __attribute__((__aligned__(4096)));
};

enum {
	DUMMY_TDM0,
	DUMMY_TDM1,
	DUMMY_TDM2,
	DUMMY_TDM3,
	DUMMY_TDM4,
	DUMMY_TDM5,
	DUMMY_TDM6,
	DUMMY_TDM7,
	DUMMY_TDM8,
	DUMMY_TDM9,
	DUMMY_TDM10,
	SE1000_HL_SHR_DUMMY0 = 20,
	SE1000_HL_SHR_DUMMY1,
	SE1000_HL_SHR_DUMMY2,
	SE1000_HL_SHR_DUMMY3,
	SE1000_HL_SHR_DUMMY4,
	SE1000_HL_SHR_DUMMY5,
	SE1000_HL_SHR_DUMMY6,
	SE1000_HL_SHR_DUMMY7,
	SE1000_HL_SHR_DUMMY8,
	SE1000_HL_SHR_DUMMY9,
	SE1000_HL_SHR_DUMMY10
};

struct aud_share_rpmsg_info {
	uint16_t        opcode;
	uint16_t        status;

	uint16_t        rate_low;        /* LSB16 of rate */
	uint16_t        rate_high;       /* MSB16 of rate */
	uint16_t        chs_and_bits;    /* MSB8 is channel, LSB8 is bits */
	uint16_t        period;          /* period size */
	uint16_t        counts_and_dir;  /* MSB15 is period counts, LSB0 is dir */
};

struct aud_dummy_stream_data {
	unsigned char *shm_addr_virt;
	phys_addr_t shm_addr_phy;
	size_t mem_size;
	struct mutex mutex;
	struct shmem_data *shm_t;
	struct snd_dma_buffer *dma_buf;
	unsigned int dma_ofs;
	size_t buffer_size;
	size_t period_size;
	size_t boundary;
	bool cap_done;
	unsigned int pos;
	struct snd_pcm_substream *substream;
	wait_queue_head_t waitq;
	bool rpmsg_event;
	uint32_t count;
	struct aud_share_rpmsg_info rpmsg;
};

enum msg_status {
	MSG_START = 1,
	MSG_READY,
	MSG_RUNING,
	MSG_BUSY,
	MSG_STOP,
	MSG_ERROR,
};

#define RPMSG_INFO_HW_PARAM_RATE_HIGH(x)    ((uint16_t)((x) >> 8))
#define RPMSG_INFO_HW_PARAM_RATE_LOW(x)     ((uint16_t)((x) & 0xffff))
#define RPMSG_INFO_HW_PARAM_DIR(x)          ((uint16_t)((x) << 0))
#define RPMSG_INFO_HW_PARAM_CHS(x)          ((uint16_t)((x) << 8))
#define RPMSG_INFO_HW_PARAM_BITS(x)         ((uint16_t)((x) << 0))
#define RPMSG_INFO_HW_PARAM_PERIOD(x)       ((uint16_t)(x))
#define RPMSG_INFO_HW_PARAM_COUNTS(x)       ((uint16_t)((x) << 1))

struct se1000_aud_dummy {
	struct device *dev;
	unsigned int id;
	struct rpmsg_endpoint *ept;
	struct rpmsg_endpoint *uept;
	struct aud_dummy_stream_data *stream_priv[2];
	uint32_t rpmsg_type;
	uint32_t internal_loop;
};

#endif

