// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Siengine
 */

#include <linux/printk.h>
#include <linux/string.h>
#include "komeda_kms_agent_debug.h"

unsigned int komeda_agent_debug = AGNET_ERR_LEVEL|AGENT_INFO_LEVEL;

void komeda_agent_printk(const char *level, const char *func, char *class,
		unsigned int category, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	if (!(komeda_agent_debug & category))
		return;

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;

	printk("%s" "[%s]%s %s %pV"  "\n",
		level, class, !strcmp(level, KERN_ERR) ? "*ERROR*" : "",
		func, &vaf);

	va_end(args);
}
