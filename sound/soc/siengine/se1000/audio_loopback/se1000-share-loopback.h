/* SPDX-License-Identifier: GPL-2.0.
 * sound/soc/siengine/se1000/audio_loopback/
 * Copyright (C) 2023 SiEngine Technology Co., Ltd. All rights reserved
 */
#ifndef _AUDIO_SHARE_LOOPBACK_H_
#define _AUDIO_SHARE_LOOPBACK_H_

#define MAX_LOOP_THREAD_NAME 50

typedef struct share_loopback_pcm {
	struct snd_pcm_substream *playback_substream;
	struct snd_pcm_substream *capture_substream;
	int instance; /* for capture and playback ref */
	int share_session;
	struct mutex lock;

	unsigned int rate;
	unsigned int channels;
	unsigned int bits;

	bool playback_start;
	bool capture_start;
	bool thread_running;
	struct task_struct *loop_thread;
	char *data_buff;
} share_loopback_pcm_t;

enum share_loopback_session {
	AUDIO_SHARE_LOOPBACK_UL_DUMMY0,
	AUDIO_SHARE_LOOPBACK_UL_DUMMY1,
	AUDIO_SHARE_LOOPBACK_UL_DUMMY2,
	AUDIO_SHARE_LOOPBACK_UL_DUMMY3,
	AUDIO_SHARE_LOOPBACK_UL_DUMMY4,
	AUDIO_SHARE_LOOPBACK_UL_DUMMY5,
	AUDIO_SHARE_LOOPBACK_UL_DUMMY6,
	AUDIO_SHARE_LOOPBACK_UL_DUMMY7,
	AUDIO_SHARE_LOOPBACK_UL_DUMMY8,
	AUDIO_SHARE_LOOPBACK_UL_DUMMY9,
	AUDIO_SHARE_LOOPBACK_UL_DUMMY10,
	AUDIO_SHARE_LOOPBACK_UL_MAX = AUDIO_SHARE_LOOPBACK_UL_DUMMY10,
	AUDIO_SHARE_LOOPBACK_DL_DUMMY0,
	AUDIO_SHARE_LOOPBACK_DL_DUMMY1,
	AUDIO_SHARE_LOOPBACK_DL_DUMMY2,
	AUDIO_SHARE_LOOPBACK_DL_DUMMY3,
	AUDIO_SHARE_LOOPBACK_DL_DUMMY4,
	AUDIO_SHARE_LOOPBACK_DL_DUMMY5,
	AUDIO_SHARE_LOOPBACK_DL_DUMMY6,
	AUDIO_SHARE_LOOPBACK_DL_DUMMY7,
	AUDIO_SHARE_LOOPBACK_DL_DUMMY8,
	AUDIO_SHARE_LOOPBACK_DL_DUMMY9,
	AUDIO_SHARE_LOOPBACK_DL_DUMMY10,
	AUDIO_SHARE_LOOPBACK_MAX
};

typedef struct share_loopback_pcm_pdata {
	int share_pcm_session[AUDIO_SHARE_LOOPBACK_MAX][2];  /* store the playback & capture dai id*/
	int id;
	share_loopback_pcm_t *loop_session_map[AUDIO_SHARE_LOOPBACK_MAX];
} share_loopback_pcm_pdata_t;

#endif
