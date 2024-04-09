#ifndef __SIENGINE_RPMSG_H__
#define __SIENGINE_RPMSG_H__

#include <dt-bindings/rpmsg/siengine-rpmsg-id.h>

struct siengine_rpmsg_packet{
	u32 src;
	u32 dst;
	u32 reverse;
	u16 length;
	u16 flags;
	u8 data[0];//for msg payload
} __packed;

#define siengine_rpmsg_packet_size(pkt)		((pkt)->length + sizeof(*(pkt)))

struct device *rpmsg_find_device_by_of_node(struct device_node *rpmsg_node);
struct rpmsg_device *find_rpmsg_device_by_phandle(struct device_node *parent);
struct rpmsg_device *find_rpmsg_device_by_index(struct device_node *parent, int index);
struct rpmsg_device *find_rpmsg_device_by_name(struct device_node *parent, const char *name);

#endif /* __SIENGINE_RPMSG_H__ */
