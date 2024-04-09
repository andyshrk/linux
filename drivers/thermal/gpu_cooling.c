#include <linux/err.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/gpu_cooling.h>
#include "thermal_core.h"

struct gpu_cooling_device {
	struct gpu_ops ops;
	void *pKbase;
	unsigned long state;
};

static DEFINE_IDA(gpu_ida);

static int gpu_cooling_get_max_state(struct thermal_cooling_device *cdev,
					 unsigned long *state)
{
	struct gpu_cooling_device *gpu_cdev = cdev->devdata;

	*state = gpu_cdev->ops.max_state(gpu_cdev->pKbase);
	*state = 100;

	return 0;
}

static int gpu_cooling_get_cur_state(struct thermal_cooling_device *cdev,
					 unsigned long *state)
{
	struct gpu_cooling_device *gpu_cdev = cdev->devdata;

	*state = gpu_cdev->state;

	return 0;
}

static int gpu_cooling_set_cur_state(struct thermal_cooling_device *cdev,
					 unsigned long state)
{
	struct gpu_cooling_device *gpu_cdev = cdev->devdata;
	unsigned long current_state = gpu_cdev->state;

	if (current_state == state)
		return 0;

	gpu_cdev->state = state;
	gpu_cdev->ops.set_cooling(gpu_cdev->pKbase, state);

	return 0;
}

static struct thermal_cooling_device_ops gpu_cooling_ops = {
	.get_max_state = gpu_cooling_get_max_state,
	.get_cur_state = gpu_cooling_get_cur_state,
	.set_cur_state = gpu_cooling_set_cur_state,
};

static int __gpu_cooling_register(struct device_node *np,
				      struct gpu_ops *ops)
{
	struct gpu_cooling_device *gpu_cdev;
	struct thermal_cooling_device *cdev;
	char dev_name[THERMAL_NAME_LENGTH];
	int id, ret;

	gpu_cdev = kzalloc(sizeof(*gpu_cdev), GFP_KERNEL);
	if (!gpu_cdev) {
		ret = -ENOMEM;
		goto out;
	}

	id = ida_simple_get(&gpu_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		ret = id;
		goto out_kfree;
	}

	gpu_cdev->ops.max_state = ops->max_state;
	gpu_cdev->ops.set_cooling = ops->set_cooling;
	gpu_cdev->pKbase = ops->args;
	gpu_cdev->state = 0xffff;

	snprintf(dev_name, sizeof(dev_name), "gpu-cooling-%d", id);

	cdev = thermal_of_cooling_device_register(np, dev_name, gpu_cdev,
						  &gpu_cooling_ops);
	if (IS_ERR(cdev)) {
		ret = PTR_ERR(cdev);
		goto out_id;
	}

	pr_debug("%s: gpu cooling thermal control registe done\n",
		 dev_name);

	return 0;

out_id:
	ida_simple_remove(&gpu_ida, id);
out_kfree:
	kfree(gpu_cdev);
out:
	return ret;
}

void gpu_cooling_register(struct device_node *gpu_node, struct gpu_ops *ops, char * node)
{
	struct device_node *cooling_node;
	int ret;

	if (node == NULL) return ;

	cooling_node = of_get_child_by_name(gpu_node, node);

	of_node_put(gpu_node);

	if (!cooling_node) {
		pr_debug("%s node not found for gpu\n", node);
		return ;
	}

	ret = __gpu_cooling_register(cooling_node, ops);

	of_node_put(cooling_node);

	if (ret) {
		pr_err("Failed to register the gpu cooling device" \
		       "for gpu: %d\n", ret);
	}
}
