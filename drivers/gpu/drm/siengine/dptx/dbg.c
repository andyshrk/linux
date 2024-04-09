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

#include "dptx.h"

#ifdef DPTX_DEBUG_DPCD_CMDS

#define BYTES_STR_LEN 128

char *__bytes_str(u8 *bytes, unsigned int count)
{
	static char str[BYTES_STR_LEN];
	unsigned int i;
	int written = 0;
	int total = 0;

	memset(str, 0, sizeof(str));

	for (i = 0; i < count; i++) {
		written = snprintf(&str[total], BYTES_STR_LEN - total,
				   "0x%02x", bytes[i]);
		if (written >= (BYTES_STR_LEN - total))
			break;

		total += written;

		if (i < (count - 1)) {
			written = snprintf(&str[total], BYTES_STR_LEN - total,
					   ", ");
			if (written >= (BYTES_STR_LEN - total))
				break;

			total += written;
		}
	}

	if (written == (BYTES_STR_LEN - total)) {
		str[BYTES_STR_LEN - 2] = '.';
		str[BYTES_STR_LEN - 3] = '.';
		str[BYTES_STR_LEN - 4] = '.';
		str[BYTES_STR_LEN - 5] = ' ';
	}

	return str;
}

#endif
