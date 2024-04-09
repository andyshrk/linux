
#ifndef __GPU_COOLING_H__
#define __GPU_COOLING_H__

#include <linux/thermal.h>

struct gpu_ops {
	unsigned long (*max_state)(void *);
	void (*set_cooling)(void *, uint64_t );
	void *args;
};

void gpu_cooling_register(struct device_node *gpu_node, struct gpu_ops *ops, char *node);

#endif /* __GPU_COOLING_H__ */
