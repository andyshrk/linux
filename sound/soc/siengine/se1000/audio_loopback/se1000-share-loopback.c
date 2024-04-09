/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Driver for hostless audio share loopback
 * Copyright 2023 SiEngine Technology Co., Ltd. All rights reserved
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>

#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <asm/dma.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include "../audio_tdm_mlp/se1000-multi-tdm.h"
#include "../audioshare/audio-share-dummy.h"
#include "audio-share-dummy-loop-ipc.h"
#include "se1000-share-loopback.h"

static DEFINE_MUTEX(share_loop_session_lock);

/* loopback copy from share dummy to tdm */
static int session_loopback_thread(void *data)
{
	share_loopback_pcm_t *pcm = (share_loopback_pcm_t *)data;
	struct snd_pcm_runtime *play_runtime = NULL, *cap_runtime = NULL;
	int wr_ret = 0, rd_ret = 0, avail = 0, hw_forward = 0;
	int buffer_bytes = 0;
	int ready = 0;

	int (*loop_dai_ioctl)(struct snd_pcm_substream *substream,
		     unsigned int cmd, void *arg);

	if (!pcm || !pcm->playback_substream || !pcm->capture_substream) {
		pr_err("%s: Invalid params data!\n", __func__);
		return -EINVAL;
	}

	play_runtime = pcm->playback_substream->runtime;
	cap_runtime = pcm->capture_substream->runtime;
	buffer_bytes = play_runtime->buffer_size * pcm->channels * (pcm->bits >> 3);
	pcm->data_buff = kzalloc(buffer_bytes, GFP_KERNEL);
	if (!pcm->data_buff) {
		pr_err("%s: Malloc memory failed!\n", __func__);
		return -ENOMEM;
	}
	pcm->thread_running = true;
	memset(pcm->data_buff, 0, buffer_bytes);

	pr_debug("%s: start loopback\n", __func__);
	/* for dl loopback */
	if (pcm->share_session > AUDIO_SHARE_LOOPBACK_UL_MAX) {
		/* wait for read share data ready */
		loop_dai_ioctl = pcm->capture_substream->ops->ioctl;
		loop_dai_ioctl(pcm->capture_substream, SNDRV_COMP_IOCTL_LOOP_DAI_READY_STATUS, &ready);
		while (!ready) {
			usleep_range(THREAD_WAIT * 1000, THREAD_WAIT * 1000); /* check every 1ms */
			loop_dai_ioctl(pcm->capture_substream, SNDRV_COMP_IOCTL_LOOP_DAI_READY_STATUS, &ready); /* write done then update read pointer */
		}

		if (pcm->playback_start) {
xrun_write:
			/* feed full playback DMA buffer at beginning */
			avail = snd_pcm_playback_avail(play_runtime);
			while (avail) {
				snd_pcm_kernel_write(pcm->playback_substream, pcm->data_buff, play_runtime->period_size);
				avail = snd_pcm_playback_avail(play_runtime);
			}
		}

		while (1) {
			/* dl session must be triggered by the hw playback TDM clk
			 * write to playback
			 */
			if (pcm->playback_start) {
				wr_ret = snd_pcm_kernel_write(pcm->playback_substream, pcm->data_buff, \
												rd_ret > 0 ? rd_ret : play_runtime->period_size);
				if (wr_ret == -EPIPE) {
					pr_debug("%s: playback xrun occurred!\n", __func__);
					wr_ret = snd_pcm_kernel_ioctl(pcm->playback_substream, SNDRV_PCM_IOCTL_PREPARE, NULL);
					if (wr_ret < 0) {
						pr_err("%s: Preparing playback device failed: %d\n", __func__, (int)wr_ret);
						goto stop;
					}
					memset(pcm->data_buff, 0, buffer_bytes);
					goto xrun_write;
				} else if (wr_ret == -EAGAIN) {
					continue;
				} else if (wr_ret < 0) {
					pr_debug("%s: playback stream stop: %d\n", __func__, (int)wr_ret);
					pcm->playback_start = false;
					goto stop;
				}
				hw_forward = frames_to_bytes(play_runtime, READ_ONCE(rd_ret));
				if (pcm->capture_start)
					loop_dai_ioctl(pcm->capture_substream, SNDRV_COMP_IOCTL_LOOP_DAI_HW_FORWARD, &hw_forward); /* write done then update read pointer */

				pr_debug("%s: write done ret %d. avail %ld\n", __func__, wr_ret, snd_pcm_playback_avail(play_runtime));
				memset(pcm->data_buff, 0, buffer_bytes);
			}
			/* read from share memory */
			if (pcm->capture_start) {
				rd_ret = snd_pcm_kernel_read(pcm->capture_substream, pcm->data_buff, play_runtime->period_size);
				pr_debug("%s: read done ret %d\n", __func__, rd_ret);
				if (rd_ret < 0)
					pcm->capture_start = false;
			}

			if (!pcm->playback_start && !pcm->capture_start)
				break;
		}
	} else if (pcm->share_session <= AUDIO_SHARE_LOOPBACK_UL_MAX) {
		/* ul session must be triggered by the hw capture TDM clk */
		loop_dai_ioctl = pcm->playback_substream->ops->ioctl;

		if (pcm->playback_start) {
			/* feed full playback empty buffer */
			avail = snd_pcm_playback_avail(play_runtime);
			while (avail) {
				wr_ret = snd_pcm_kernel_write(pcm->playback_substream, pcm->data_buff, play_runtime->period_size);
				if (wr_ret > 0)
					avail -= wr_ret;
			}
		}
		wr_ret = 0;
		while (1) {
			hw_forward = frames_to_bytes(cap_runtime, READ_ONCE(wr_ret));
			if (pcm->playback_start)
				loop_dai_ioctl(pcm->playback_substream, SNDRV_COMP_IOCTL_LOOP_DAI_HW_FORWARD, &hw_forward); /* read done then update write hw ptr */

			/* read from TDM */
			if (pcm->capture_start) {
xrun_read:
				rd_ret = snd_pcm_kernel_read(pcm->capture_substream, pcm->data_buff, cap_runtime->period_size);
				if (rd_ret == -EPIPE) {
					pr_debug("%s: capture xrun occurred!\n", __func__);
					rd_ret = snd_pcm_kernel_ioctl(pcm->capture_substream, SNDRV_PCM_IOCTL_PREPARE, NULL);
					if (rd_ret < 0) {
						pr_err("%s: Preparing capture device failed: %d\n", __func__, (int)rd_ret);
						goto stop;
					}
					goto xrun_read;
				} else if (rd_ret == -EAGAIN) {
					continue;
				} else if (rd_ret < 0) {
					pr_debug("%s: capture stream stop: %d\n", __func__, (int)rd_ret);
					pcm->capture_start = false;
					goto stop;
				}

				pr_debug("%s: read done ret %d\n", __func__, rd_ret);
			}
			/* write to playback */
			if (pcm->playback_start) {
				wr_ret = snd_pcm_kernel_write(pcm->playback_substream, pcm->data_buff, rd_ret);
				pr_debug("%s: write done ret %d\n", __func__, wr_ret);
				if (wr_ret < 0)
					pcm->playback_start = false;
				memset(pcm->data_buff, 0, buffer_bytes);
			}

			if (!pcm->playback_start && !pcm->capture_start)
				break;
		}
	} else
		pr_err("%s: unsupported loopback session %d\n", __func__, pcm->share_session);

stop:
	kfree(pcm->data_buff);
	pcm->data_buff = NULL;
	pcm->loop_thread = NULL;
	pcm->thread_running = false;
	pr_debug("%s: pcm loop thread stopped\n", __func__);
	return 0;
}

static void se1000_share_loopback_start_pcm(share_loopback_pcm_t *pcm)
{
	mutex_lock(&pcm->lock);
	if (pcm->loop_thread && !pcm->thread_running)
		wake_up_process(pcm->loop_thread);
	mutex_unlock(&pcm->lock);
}

static int se1000_share_get_session_idx(share_loopback_pcm_pdata_t *pdata, int dai_id, int stream)
{
	int i, idx = 0;

	for (i = 0; i < AUDIO_SHARE_LOOPBACK_MAX; i++) {
		if (dai_id == pdata->share_pcm_session[i][stream]) {
			idx = i;
			break;
		}
	}
	return idx;
}

static int se1000_share_loopback_get_session(struct snd_soc_component *component,
					struct snd_pcm_substream *substream,
					struct share_loopback_pcm **pcm)
{
	share_loopback_pcm_pdata_t *pdata;
	share_loopback_pcm_t *pcm_session = NULL;
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	int ret = 0;
	int session = 0;

	pdata = (share_loopback_pcm_pdata_t *)dev_get_drvdata(component->dev);
	if (!pdata) {
		dev_err(component->dev, "%s: platform data not populated\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&share_loop_session_lock);
	session = se1000_share_get_session_idx(pdata, rtd->dai_link->id, substream->stream);
	pcm_session = pdata->loop_session_map[session];
	/* session existed */
	if (pcm_session) {
		*pcm = pcm_session;
		goto exit;
	}

	pcm_session = kzalloc(sizeof(share_loopback_pcm_t), GFP_KERNEL);
	if (!pcm_session) {
		ret = -ENOMEM;
		goto exit;
	}
	dev_dbg(component->dev, "%s: loopback session %d inited\n", __func__, session);

	mutex_init(&pcm_session->lock);
	pcm_session->share_session = session;
	*pcm = pcm_session;
	pcm_session->loop_thread = NULL;
	pdata->loop_session_map[session] = pcm_session;
exit:
	mutex_unlock(&share_loop_session_lock);
	return ret;
}

static int se1000_share_loopback_open(struct snd_soc_component *component,
			      struct snd_pcm_substream *substream)
{
	share_loopback_pcm_t *pcm = NULL;
	int ret = 0;
	share_loopback_pcm_pdata_t *pdata = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s\n", __func__);
	if (!component || !substream) {
		dev_err(component->dev, "%s: pointer is NULL\n", __func__);
		return -EINVAL;
	}

	ret = se1000_share_loopback_get_session(component, substream, &pcm);
	if (ret) {
		dev_err(component->dev, "%s: get pcm session failed\n", __func__);
		return ret;
	}

	mutex_lock(&pcm->lock);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		pcm->playback_substream = substream;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		pcm->capture_substream = substream;

	pcm->instance++;
	/* init loopback thread when capture and playback both opened */
	if (pcm->instance == 2) {
		if (pcm->loop_thread == NULL)
			pcm->loop_thread = kthread_create(session_loopback_thread,
						(void *)pcm, "%s_%d_%d", "aud_sh_lp", pdata->id, pcm->share_session);

		if (IS_ERR(pcm->loop_thread)) {
			dev_err(component->dev, " %s: Unable to create kernel loop thread.", __func__);
			ret = PTR_ERR(pcm->loop_thread);
			pcm->loop_thread = NULL;
			mutex_unlock(&pcm->lock);
			return ret;
		}
		/* set to fifo prio */
		sched_set_fifo(pcm->loop_thread);
	}
	dev_dbg(component->dev, "%s: pcm %p session %d Instance = %d, Stream ID = %s\n",
			__func__, pcm, pcm->share_session, pcm->instance, substream->pcm->id);
	substream->runtime->private_data = pcm;
	mutex_unlock(&pcm->lock);
	return 0;
}

static int se1000_share_loopback_close(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	share_loopback_pcm_t *pcm = runtime->private_data;
	share_loopback_pcm_pdata_t *pdata = snd_soc_component_get_drvdata(component);

	if (!component || !substream) {
		dev_err(component->dev, "%s: pointer is NULL\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&pcm->lock);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		pcm->playback_start = false;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		pcm->capture_start = false;

	pcm->instance--;
	dev_dbg(component->dev, "%s: close pcm %p session %d Instance = %d, Stream ID = %s\n",
			__func__, pcm, pcm->share_session, pcm->instance, substream->pcm->id);
	if (!pcm->instance) {
		mutex_lock(&share_loop_session_lock);
		while (pcm->loop_thread) {
			dev_dbg(component->dev, "%s: waiting thread stop %d\n",
				__func__, pcm->share_session);
			usleep_range(THREAD_WAIT * 1000, THREAD_WAIT * 1000);
		}
		mutex_unlock(&pcm->lock);
		mutex_destroy(&pdata->loop_session_map[pcm->share_session]->lock);
		pdata->loop_session_map[pcm->share_session] = NULL;
		kfree(pcm);
		dev_dbg(component->dev, "%s: stream loop freed %d\n",
				__func__, pcm->share_session);
		mutex_unlock(&share_loop_session_lock);
	} else
		mutex_unlock(&pcm->lock);

	return 0;
}

static int se1000_share_loopback_trigger(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	share_loopback_pcm_t *pcm = runtime->private_data;

	dev_dbg(component->dev, "%s, Stream ID = %s cmd %d\n", __func__, substream->pcm->id, cmd);

	if (!component || !pcm) {
		dev_err(component->dev, "%s: pointer is NULL\n", __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			pcm->playback_start = true;
		else
			pcm->capture_start = true;

		dev_dbg(component->dev, "%s: session:%d, playback_start:%d,capture_start:%d cmd %d\n", __func__, \
								pcm->share_session, pcm->playback_start, pcm->capture_start, cmd);
		if (pcm->playback_start && pcm->capture_start)
			se1000_share_loopback_start_pcm(pcm);

		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_STOP:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			pcm->playback_start = false;
		else
			pcm->capture_start = false;

		dev_dbg(component->dev, "%s:session:%d Pause/Stop - playback_start:%d, capture_start:%d cmd %d\n", __func__, \
								pcm->share_session, pcm->playback_start, pcm->capture_start, cmd);

		break;
	default:
		dev_err(component->dev, "%s: default cmd %d\n", __func__, cmd);
		break;
	}

	return 0;
}

static int se1000_share_loopback_new(struct snd_soc_component *component,
			     struct snd_soc_pcm_runtime *rtd)
{
	share_loopback_pcm_pdata_t *pdata = snd_soc_component_get_drvdata(component);
	int ret = 0;

	dev_dbg(component->dev, "%s: id %d, dai id %d\n", __func__, pdata->id, rtd->dai_link->id);
	if (pdata->id == 0) {
		/* get current dummy pcm dai id for loopback session */
		switch (rtd->dai_link->id) {
		case SE1000_HL_SHR_I2S0:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY0][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_I2S0;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY0][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_I2S0;
			break;
		case SE1000_HL_SHR_I2S1:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY1][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_I2S1;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY1][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_I2S1;
			break;
		case SE1000_HL_SHR_I2S2:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY2][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_I2S2;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY2][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_I2S2;
			break;
		case SE1000_HL_SHR_DUMMY0:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY0][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY0;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY0][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY0;
			break;
		case SE1000_HL_SHR_DUMMY1:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY1][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY1;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY1][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY1;
			break;
		case SE1000_HL_SHR_DUMMY2:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY2][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY2;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY2][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY2;
			break;
		default:
			dev_err(component->dev, "%s: Invalid dai id for loop 0\n", __func__);
			ret = -EINVAL;
			break;
		}
	} else if (pdata->id == 2) {
		switch (rtd->dai_link->id) {
		case SE1000_MULTI_HL_FE_SHR0:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY0][SNDRV_PCM_STREAM_CAPTURE] = SE1000_MULTI_HL_FE_SHR0;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY0][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_MULTI_HL_FE_SHR0;
			break;
		case SE1000_MULTI_HL_FE_SHR1:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY1][SNDRV_PCM_STREAM_CAPTURE] = SE1000_MULTI_HL_FE_SHR1;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY1][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_MULTI_HL_FE_SHR1;
			break;
		case SE1000_MULTI_HL_FE_SHR2:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY2][SNDRV_PCM_STREAM_CAPTURE] = SE1000_MULTI_HL_FE_SHR2;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY2][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_MULTI_HL_FE_SHR2;
			break;
		case SE1000_MULTI_HL_FE_SHR3:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY3][SNDRV_PCM_STREAM_CAPTURE] = SE1000_MULTI_HL_FE_SHR3;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY3][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_MULTI_HL_FE_SHR3;
			break;
		case SE1000_MULTI_HL_FE_SHR4:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY4][SNDRV_PCM_STREAM_CAPTURE] = SE1000_MULTI_HL_FE_SHR4;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY4][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_MULTI_HL_FE_SHR4;
			break;
		case SE1000_MULTI_HL_FE_SHR5:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY5][SNDRV_PCM_STREAM_CAPTURE] = SE1000_MULTI_HL_FE_SHR5;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY5][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_MULTI_HL_FE_SHR5;
			break;
		case SE1000_MULTI_HL_FE_SHR6:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY6][SNDRV_PCM_STREAM_CAPTURE] = SE1000_MULTI_HL_FE_SHR6;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY6][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_MULTI_HL_FE_SHR6;
			break;
		case SE1000_MULTI_HL_FE_SHR7:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY7][SNDRV_PCM_STREAM_CAPTURE] = SE1000_MULTI_HL_FE_SHR7;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY7][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_MULTI_HL_FE_SHR7;
			break;
		case SE1000_MULTI_HL_FE_SHR8:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY8][SNDRV_PCM_STREAM_CAPTURE] = SE1000_MULTI_HL_FE_SHR8;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY8][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_MULTI_HL_FE_SHR8;
			break;
		case SE1000_MULTI_HL_FE_SHR9:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY9][SNDRV_PCM_STREAM_CAPTURE] = SE1000_MULTI_HL_FE_SHR9;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY9][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_MULTI_HL_FE_SHR9;
			break;
		case SE1000_MULTI_HL_FE_SHR10:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY10][SNDRV_PCM_STREAM_CAPTURE] = SE1000_MULTI_HL_FE_SHR10;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY10][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_MULTI_HL_FE_SHR10;
			break;
		case SE1000_HL_SHR_DUMMY0:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY0][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY0;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY0][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY0;
			break;
		case SE1000_HL_SHR_DUMMY1:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY1][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY1;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY1][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY1;
			break;
		case SE1000_HL_SHR_DUMMY2:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY2][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY2;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY2][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY2;
			break;
		case SE1000_HL_SHR_DUMMY3:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY3][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY3;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY3][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY3;
			break;
		case SE1000_HL_SHR_DUMMY4:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY4][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY4;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY4][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY4;
			break;
		case SE1000_HL_SHR_DUMMY5:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY5][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY5;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY5][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY5;
			break;
		case SE1000_HL_SHR_DUMMY6:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY6][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY6;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY6][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY6;
			break;
		case SE1000_HL_SHR_DUMMY7:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY7][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY7;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY7][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY7;
			break;
		case SE1000_HL_SHR_DUMMY8:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY8][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY8;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY8][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY8;
			break;
		case SE1000_HL_SHR_DUMMY9:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY9][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY9;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY9][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY9;
			break;
		case SE1000_HL_SHR_DUMMY10:
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_UL_DUMMY10][SNDRV_PCM_STREAM_PLAYBACK] = SE1000_HL_SHR_DUMMY10;
			pdata->share_pcm_session[AUDIO_SHARE_LOOPBACK_DL_DUMMY10][SNDRV_PCM_STREAM_CAPTURE] = SE1000_HL_SHR_DUMMY10;
			break;
		default:
			dev_err(component->dev, "%s: Invalid dai id for loop 2\n", __func__);
			ret = -EINVAL;
			break;
		}
	} else {
		dev_err(component->dev, "%s: unknown card loopback dev\n", __func__);
		return -EINVAL;
	}

	return ret;
}

static int se1000_share_loopback_hw_params(struct snd_soc_component *component,
			struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	share_loopback_pcm_t *pcm = runtime->private_data;

	if (!pcm || !params) {
		dev_err(component->dev, "%s: pointer is NULL\n", __func__);
		return -EINVAL;
	}

	/* get the playback parameters */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pcm->rate = params_rate(params);
		pcm->channels = params_channels(params);
		pcm->bits = params_physical_width(params);
	}
	dev_dbg(component->dev, "%s: pcm session rate %d, bit %d, channel %d\n", \
		__func__, pcm->rate, pcm->bits, pcm->channels);
	return 0;
}

static int se1000_share_loopback_prepare(struct snd_soc_component *component,
			struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	share_loopback_pcm_t *pcm = runtime->private_data;

	/* playback stream won't trigger start directly until reach to start threshold
	 * so set playback start when prepared state and wake up loop thread if not running
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pcm->playback_start = true;
		if (pcm->share_session > AUDIO_SHARE_LOOPBACK_UL_MAX)
			runtime->start_threshold = runtime->buffer_size;
	}

	dev_dbg(component->dev, "%s: session:%d, playback_start:%d,capture_start:%d\n", __func__, \
					pcm->share_session, pcm->playback_start, pcm->capture_start);

	if (pcm->playback_start && pcm->capture_start)
		se1000_share_loopback_start_pcm(pcm);
	return 0;
}

static const struct snd_soc_component_driver se1000_share_loopback_component = {
	.name          = "se1000-share-loopback",
	.probe_order   = SND_SOC_COMP_ORDER_LATE,
	.open          = se1000_share_loopback_open,
	.close         = se1000_share_loopback_close,
	.hw_params     = se1000_share_loopback_hw_params,
	.trigger       = se1000_share_loopback_trigger,
	.prepare       = se1000_share_loopback_prepare,
	.pcm_construct = se1000_share_loopback_new,
};

static int se1000_share_loopback_probe(struct platform_device *pdev)
{
	share_loopback_pcm_pdata_t *pdata;
	int ret = 0;
	int id = 0;

	dev_dbg(&pdev->dev, "%s: dev name %s\n", __func__, dev_name(&pdev->dev));
	pdata = kzalloc(sizeof(share_loopback_pcm_pdata_t), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "%s: No memory inited\n", __func__);
		return -ENOMEM;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "se-loopback-id", &id);
	if (ret) {
		dev_err(&pdev->dev, "%s: se_loopback_id missing in DT node\n",
					__func__);
		goto err_free_mem;
	}
	pdata->id = id;
	dev_set_drvdata(&pdev->dev, pdata);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev))
		goto err_pm;
	pm_runtime_get_sync(&pdev->dev);

	ret = devm_snd_soc_register_component(&pdev->dev, &se1000_share_loopback_component,
							 NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "err_component\n");
		goto err_component;
	}
	return 0;

err_component:
	pm_runtime_put_sync(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);
err_pm:
	pm_runtime_disable(&pdev->dev);
err_free_mem:
	kfree(pdata);

	return ret;
}

static int se1000_share_loopback_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id se1000_share_loopback_dts_match[] = {
	{ .compatible = "siengine,se1000-share-loopback", },
};
MODULE_DEVICE_TABLE(of, se1000_share_loopback_dts_match);

static struct platform_driver se1000_share_loopback_driver = {
	.driver = {
		.name = "se1000-share-loopback",
		.of_match_table = se1000_share_loopback_dts_match,
	},
	.probe   = se1000_share_loopback_probe,
	.remove  = se1000_share_loopback_remove,
};

module_platform_driver(se1000_share_loopback_driver);

MODULE_AUTHOR("ying.chen <ying.chen@siengine>");
MODULE_DESCRIPTION("ASoC Audio Share Hostless Loopback Platform Driver");
MODULE_LICENSE("GPL v2");
