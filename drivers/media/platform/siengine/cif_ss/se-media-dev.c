// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021-2022 Siengine Technology, Inc.
 *
 * Sienginie CIF Media Framework Driver
 *
 * Author: Siengine Technology, Inc.
 */

#include <linux/bug.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/media-device.h>

#include "se-media-dev.h"
#include "dw-mipi-csi.h"

/*create default links between registered entities  */
static int se_md_create_links(struct se_md *se_md)
{
	struct media_entity *source, *sink;
	struct se_cif_dev *se_cif;
	struct dw_csi *mipi_csi2;
	int i, j, ret = 0;
	u16  source_pad, sink_pad;
	u32 flags;
	u8 csi_idx, vc_idx;
	u32 mipi_vc = 0;
	//int j;
	struct se_sensor_info *sensor;
	//int num_sensors = se_md->subdev_notifier.num_subdevs;
	int num_sensors = se_md->num_sensors;

	/* Create links between each CIF's subdev and video node */
	flags = MEDIA_LNK_FL_ENABLED;
	for (i = 0; i < SE_MD_CIF_MAX_DEVS; i++) {
		se_cif = se_md->se_cif[i];
		if (!se_cif)
			continue;

		//TODO:
		/* Connect CIF source to video device */
		source = &se_cif->cif_cap.sd.entity;
		sink = &se_cif->cif_cap.vdev.entity;
		sink_pad = 0;
		source_pad = CIF_SD_PAD_SOURCE_MEM;

		ret = media_create_pad_link(source, source_pad,
					      sink, sink_pad, flags);
		if (ret) {
			v4l2_err(&se_md->v4l2_dev, "Failed created link [%s] %c> [%s]\n",
			  source->name, flags ? '=' : '-', sink->name);
			break;
		}
		v4l2_dbg(0, 0, &se_md->v4l2_dev, "created link [%s] %c> [%s]\n",
			  source->name, flags ? '=' : '-', sink->name);
		/* Notify capture subdev entity ,cif cap link setup */
		ret = media_entity_call(source, link_setup, &source->pads[source_pad],
						&sink->pads[sink_pad], flags);
		if (ret) {
			v4l2_err(&se_md->v4l2_dev, "failed call link_setup [%s] %c> [%s]\n",
			  source->name, flags ? '=' : '-', sink->name);
			break;
		}

		v4l2_dbg(0, 0, &se_md->v4l2_dev, "created link [%s] %c> [%s]\n",
			  source->name, flags ? '=' : '-', sink->name);

		/* Connect MIPI CSI source to CIF sink */
		//TODO: if CSI IPI channel & CIF channel bindings is fix
		//then source, source pad, sink pad are fix value.
		sink = &se_cif->cif_cap.sd.entity;
		csi_idx = se_cif->interface[IN_PORT];
		vc_idx = se_cif->interface[SUB_IN_PORT];
		if (csi_idx >= SE_MD_MIPI_CSI2_MAX_DEVS || vc_idx >= MAX_VC_NUM) {
			v4l2_err(&se_md->v4l2_dev, "Not support input interface: %x\n",
				se_cif->interface[IN_PORT]);
			return -EINVAL;
		}
		if (se_md->mipi_csi2[csi_idx] == NULL)
			continue;
		source = &se_md->mipi_csi2[csi_idx]->sd.entity;
		source_pad = vc_idx;
		sink_pad = MAX_VC_NUM * csi_idx + vc_idx;
		/* Create link MIPI to CIF */
		ret = media_create_pad_link(source, source_pad, sink, sink_pad, flags);
		if (ret) {
			v4l2_err(&se_md->v4l2_dev, "created link [%s] %c> [%s] fail\n",
			  source->name, flags ? '=' : '-', sink->name);
			break;
		}
		v4l2_dbg(0, 0, &se_md->v4l2_dev, "created link [%s] %c> [%s]\n",
			  source->name, flags ? '=' : '-', sink->name);
		/* Notify CIF subdev entity */
		ret = media_entity_call(sink, link_setup, &sink->pads[sink_pad],
					&source->pads[source_pad], 0);
		v4l2_dbg(0, 0, &se_md->v4l2_dev, "media_entity_call CIF, ret = %d.\n", ret);
		if (ret)
			break;

		/* Notify MIPI entity */
		ret = media_entity_call(source, link_setup, &source->pads[source_pad],
					&sink->pads[sink_pad], 0);
		v4l2_dbg(0, 0, &se_md->v4l2_dev, "media_entity_call CSI, ret = %d.\n", ret);
		if (ret)
			break;

		v4l2_dbg(0, 0, &se_md->v4l2_dev, "created link [%s] %c> [%s]\n",
			  source->name, flags ? '=' : '-', sink->name);
	}

	/* Connect MIPI Sensor to MIPI CSI2 */
	for (i = 0; i < num_sensors; i++) {
		sensor = &se_md->sensor[i];
		if (sensor == NULL || sensor->sd == NULL)
			continue;

		if (se_md->mipi_csi2) {
			mipi_csi2 = se_md->mipi_csi2[sensor->id]; //the sensor->id is port id
			if (mipi_csi2 ==  NULL)
				continue;

			source = &sensor->sd->entity;
			sink = &mipi_csi2->sd.entity;

			source_pad = 0;  /* sensor source pad: MIPI_CSI2_SENS_VC0_PAD_SOURCE */
			sink_pad = source_pad + 4;  /* mipi sink pad: MIPI_CSI2_VC0_PAD_SINK; */

			mipi_vc = 4;

			for (j = 0; j < mipi_vc; j++) {
				ret = media_create_pad_link(source, source_pad + j, sink,
					sink_pad + j,
					MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
				if (ret)
					return ret;

				/* Notify MIPI subdev entity */
				ret = media_entity_call(sink, link_setup, &sink->pads[sink_pad + j],
							&source->pads[source_pad + j], 0);
				if (ret)
					return ret;

				/* Notify MIPI sensor subdev entity */
				ret = media_entity_call(source, link_setup,
					&source->pads[source_pad + j], &sink->pads[sink_pad + j], 0);
				if (ret)
					return ret;

			}
			v4l2_info(&se_md->v4l2_dev, "created link [%s] => [%s]\n",
				  sensor->sd->entity.name, mipi_csi2->sd.entity.name);
		}
	}
	return 0;
}

static int subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *sd,
				struct v4l2_async_subdev *asd)
{
	struct se_md *se_md = notifier_to_se_md(notifier);
	struct se_sensor_info *sensor = NULL;
	int i;

	dev_info(&se_md->pdev->dev, "%s\n", __func__);

	/* Find platform data for this sensor subdev */
	for (i = 0; i < ARRAY_SIZE(se_md->sensor); i++) {
		if (se_md->sensor[i].asd.match.fwnode ==
				of_fwnode_handle(sd->dev->of_node))
			sensor = &se_md->sensor[i];
	}

	if (sensor == NULL)
		return -EINVAL;

	sd->grp_id = GRP_ID_SENSOR;

	sensor->sd = sd;

	se_md->num_sensors++;

	v4l2_info(&se_md->v4l2_dev, "Registered sensor subdevice: %s (%d)\n",
			sd->name, se_md->num_sensors);

	return 0;
}

static int subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct se_md *se_md = notifier_to_se_md(notifier);
	int ret;

	dev_info(&se_md->pdev->dev, "%s\n", __func__);

	mutex_lock(&se_md->media_dev.graph_mutex);

	ret = se_md_create_links(se_md);
	if (ret < 0)
	{
		dev_err(&se_md->pdev->dev, "se_md_create_links failed, ret = %d\n", ret);
		goto unlock;
	}

	se_md->link_status = 1;

	ret = v4l2_device_register_subdev_nodes(&se_md->v4l2_dev);

unlock:
	mutex_unlock(&se_md->media_dev.graph_mutex);
	if (ret < 0) {
		v4l2_err(&se_md->v4l2_dev, "%s error exit\n", __func__);
		return ret;
	}

	return media_device_register(&se_md->media_dev);
}


/**
 * se_sensor_notify - v4l2_device notification from a sensor subdev
 */
void se_sensor_notify(struct v4l2_subdev *sd, unsigned int notification,
			void *arg)
{
	return;
}

static void get_sensor_data(struct se_sensor_info *se_sensor)
{
	struct v4l2_async_subdev *asd;
	struct device_node *of_node;
	struct i2c_client *client;
	struct v4l2_subdev *sd;
	struct sensor_data *data;

	asd = &se_sensor->asd;
	of_node = to_of_node(asd->match.fwnode);
	if(of_node){
		client = of_find_i2c_device_by_node(of_node);
		of_node_put(of_node);
		if(client){
			sd = (struct v4l2_subdev *)i2c_get_clientdata(client);
			if(sd){
				data = container_of(sd, struct sensor_data, subdev);
				if(data){
					pr_debug("%s: sensor_is_there 0x%02x\n",
						__func__, data->sensor_is_there);
					se_sensor->data = data;
				}
			}
		}
	}
}

/* Register mipi sensor / Parallel CSI / HDMI Rx sub-devices */
static int register_sensor_entities(struct se_md *se_md)
{
	struct device_node *parent = se_md->pdev->dev.of_node;
	struct device_node *node, *ep, *rem;
	struct v4l2_fwnode_endpoint endpoint = { .bus_type = 0 };
	int index = 0;
	struct v4l2_async_subdev *asd;
	int ret;

	se_md->num_sensors = 0;

	/* Attach sensors linked to MIPI CSI2 / paralle csi / HDMI Rx */
	for_each_available_child_of_node(parent, node) {
		struct device_node *port;
		if (!of_node_cmp(node->name, "hdmi_rx")) {
			se_md->sensor[index].asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
			se_md->sensor[index].asd.match.fwnode = of_fwnode_handle(node);
			se_md->async_subdevs[index] = &se_md->sensor[index].asd;

			se_md->num_sensors++;
			index++;
			continue;
		}

		if (of_node_cmp(node->name, "csi") &&
			of_node_cmp(node->name, "pcsi"))
			continue;

		if (!of_device_is_available(node))
			continue;

		/* csi2 node have only port */
		port = of_get_next_child(node, NULL);
		if (!port)
			continue;

		/* port can have only endpoint */
		ep = of_get_next_child(port, NULL);
		if (!ep)
			return -EINVAL;

		ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &endpoint);
		if (ret) {
			v4l2_err(&se_md->v4l2_dev, "v4l2_fwnode_endpoint_parse failed, %d\n", ret);
		}
		if (WARN_ON(endpoint.base.port >= SE_MAX_SENSORS)) {
			v4l2_err(&se_md->v4l2_dev, "Failed to get sensor endpoint\n");
			return -EINVAL;
		}

		se_md->sensor[index].id = endpoint.base.port;

		if (!of_node_cmp(node->name, "csi"))
			se_md->sensor[index].mipi_mode = true;

		/* remote port---sensor node */
		rem = of_graph_get_remote_port_parent(ep);
		of_node_put(ep);
		if (rem == NULL) {
			v4l2_info(&se_md->v4l2_dev, "Remote device at %s not found\n",
								ep->full_name);
			continue;
		}

		asd = v4l2_async_notifier_add_fwnode_subdev(&se_md->subdev_notifier, of_fwnode_handle(rem),
			sizeof(struct v4l2_async_subdev));

		se_md->sensor[index].asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
		se_md->sensor[index].asd.match.fwnode = of_fwnode_handle(rem);
		se_md->async_subdevs[index] = &se_md->sensor[index].asd;
		get_sensor_data(&se_md->sensor[index]);
		se_md->num_sensors++;

		index++;
	}

	return 0;
}

static bool is_sensor_there(struct se_md *se_md, struct se_cif_dev *se_cif)
{
	struct se_sensor_info *se_sensor;
	u8 csi_idx, vc_idx;
	int i;
	unsigned char ret;

	ret = 0;
	csi_idx = se_cif->interface[IN_PORT];
	vc_idx = se_cif->interface[SUB_IN_PORT];
	for (i = 0; i < se_md->num_sensors; i++){
		se_sensor = &se_md->sensor[i];
		if (csi_idx == se_sensor->id){
			if(se_sensor->data)
				ret = se_sensor->data->sensor_is_there & (0x1 << vc_idx);
				v4l2_dbg(0, 0, &se_md->v4l2_dev,
					"camera sensor 0x%x connect CIF.%d\n", ret, se_cif->id);
			break;
		}
		else
		{
			v4l2_dbg(0, 0, &se_md->v4l2_dev, "No sensor connect CIF.%d\n", se_cif->id);
		}
	}
	return ret?true:false;
}

static int register_cif_entity(struct se_md *se_md, struct se_cif_dev *se_cif)
{
	struct v4l2_subdev *sd = &se_cif->cif_cap.sd;
	int ret;

	dev_dbg(&se_md->pdev->dev, "%s\n", __func__);
	if (WARN_ON(se_cif->id >= SE_CIF_MAX_DEVS))
		return -EBUSY;

	sd->grp_id = GRP_ID_CIF;

	ret = v4l2_device_register_subdev(&se_md->v4l2_dev, sd);
	if (!ret)
		se_md->se_cif[se_cif->id] = se_cif;
	else
		v4l2_err(&se_md->v4l2_dev, "Failed to register CIF.%d (%d)\n",
			se_cif->id, ret);
	return ret;
}

static int register_mipi_csi2_entity(struct se_md *se_md,
				struct dw_csi *mipi_csi2)
{
	struct v4l2_subdev *sd = &mipi_csi2->sd;
	int ret;

	if (WARN_ON(mipi_csi2->index >= SE_MD_MIPI_CSI2_MAX_DEVS))
		return -ENOENT;

	sd->grp_id = GRP_ID_CSI;
	ret = v4l2_device_register_subdev(&se_md->v4l2_dev, sd);
	if (!ret)
		se_md->mipi_csi2[mipi_csi2->index] = mipi_csi2;
	else
		v4l2_err(&se_md->v4l2_dev,
			 "Failed to register MIPI-CSIS.%d (%d)\n", mipi_csi2->index, ret);
	return ret;
}

static int se_md_register_platform_entity(struct se_md *se_md,
					struct platform_device *pdev,
					int plat_entity)
{
	struct device *dev = &pdev->dev;
	int ret = -EPROBE_DEFER;
	void *drvdata;

	/* Lock to ensure dev->driver won't change. */
	device_lock(dev);

	if (!dev->driver || !try_module_get(dev->driver->owner))
		goto dev_unlock;

	drvdata = dev_get_drvdata(dev);
	/* Some subdev didn't probe successfully id drvdata is NULL */
	if (drvdata) {
		switch (plat_entity) {
		case IDX_CIF:
			if (is_sensor_there(se_md, drvdata)){
				dev_info(&se_md->pdev->dev, "Register CIF entity.cif index: %d\n", ((struct se_cif_dev *)drvdata)->id);
				ret = register_cif_entity(se_md, drvdata);
			}
			else
			{
				dev_dbg(&se_md->pdev->dev, "No sensor connect CIF entity., cif index %d\n", ((struct se_cif_dev *)drvdata)->id);
				ret = 0;
			}
			break;
		case IDX_MIPI_CSI2:
			dev_info(&se_md->pdev->dev, "Register CSI entity, cif index %d\n", ((struct se_cif_dev *)drvdata)->id);
			ret = register_mipi_csi2_entity(se_md, drvdata);
			break;
		default:
			ret = -ENODEV;
		}
	}
	module_put(dev->driver->owner);

dev_unlock:
	device_unlock(dev);
	if (ret == -EPROBE_DEFER)
		dev_info(&se_md->pdev->dev, "deferring %s device registration\n",
			dev_name(dev));
	else if (ret < 0)
		dev_err(&se_md->pdev->dev, "%s device registration failed (%d)\n",
			dev_name(dev), ret);

	return ret;
}

/* Register CIF, MIPI CSI2 Media entities */
static int se_md_register_platform_entities(struct se_md *se_md,
					struct device_node *parent)
{
	struct device_node *node;
	int ret = 0;

	for_each_available_child_of_node(parent, node) {
		struct platform_device *pdev;
		int plat_entity = -1;

		pdev = of_find_device_by_node(node);
		if (!pdev)
			continue;

		/* If driver of any entity isn't ready try all again later. */
		if (!strcmp(node->name, MIPI_CSI2_OF_NODE_NAME))
			plat_entity = IDX_MIPI_CSI2;
		else if (!strcmp(node->name, CIF_OF_NODE_NAME))
			plat_entity = IDX_CIF;

		if (plat_entity >= 0)
			ret = se_md_register_platform_entity(se_md, pdev,
							plat_entity);

		put_device(&pdev->dev);
		if (ret < 0)
			break;
	}

	return ret;
}

static void se_md_unregister_entities(struct se_md *se_md)
{
	int i;

	for (i = 0; i < SE_MD_CIF_MAX_DEVS; i++) {
		struct se_cif_dev *dev = se_md->se_cif[i];
		if (dev == NULL)
			continue;
		v4l2_device_unregister_subdev(&dev->cif_cap.sd);
		se_md->se_cif[i] = NULL;
	}
	for (i = 0; i < SE_MD_MIPI_CSI2_MAX_DEVS; i++) {
		if (se_md->mipi_csi2[i] != NULL)
		{
			v4l2_device_unregister_subdev(&se_md->mipi_csi2[i]->sd);
			se_md->mipi_csi2[i] = NULL;
		}
	}


	v4l2_info(&se_md->v4l2_dev, "Unregistered all entities\n");
}

static int se_md_do_clean(struct se_md *se_md, struct media_pad *pad)
{
	struct device *dev = &se_md->pdev->dev;
	struct media_pad *remote_pad;
	struct v4l2_subdev	*subdev;
	struct se_cif_dev *se_cif;

	remote_pad = media_entity_remote_pad(pad);
	if (remote_pad == NULL) {
		dev_err(dev, "%s get remote pad fail\n", __func__);
		return -ENODEV;
	}

	subdev = media_entity_to_v4l2_subdev(remote_pad->entity);
	if (subdev == NULL) {
		dev_err(dev, "%s media entity to v4l2 subdev fail\n", __func__);
		return -ENODEV;
	}

	se_cif = v4l2_get_subdevdata(subdev);
	if (se_cif == NULL) {
		dev_err(dev, "%s Can't get subdev %s data\n", __func__, subdev->name);
		return -ENODEV;
	}

	v4l2_device_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);

	dev_info(dev, "clean cif channel[%d]\n", se_cif->id);

	return 0;
}

static int se_md_clean_channel(struct se_md *se_md, int index)
{
	//struct se_sensor_info *sensor = &se_md->sensor[index];
	struct dw_csi *mipi_csi2;
	struct media_pad *local_pad;
	struct media_entity *local_en;
	u32 i, mipi_vc = 0;
	int ret;

	if (se_md->mipi_csi2[index]) {
		mipi_csi2 = se_md->mipi_csi2[index];

		//if (mipi_csi2->vchannel == true)
		//	mipi_vc = 4;
		//else
		//	mipi_vc = 1;
		mipi_vc = 1;
		local_en = &mipi_csi2->sd.entity;
		if (local_en == NULL)
		{
			dev_err(&se_md->pdev->dev, "local_en is NULL.\n");
			return -ENODEV;
		}

		for (i = 0; i < mipi_vc; i++) {
			local_pad = &local_en->pads[CSI_PAD_VC0_SOURCE + i];
			ret = se_md_do_clean(se_md, local_pad);
			if (ret < 0)
			{
				dev_err(&se_md->pdev->dev, "se_md_do_clean fail.\n");
				return -ENODEV;
			}
		}
	}

	return 0;
}

static int __attribute__((__unused__)) se_md_clean_unlink_channels(struct se_md *se_md)
{
	struct se_sensor_info *sensor;
	//int num_subdevs = se_md->subdev_notifier.num_subdevs;
	int num_subdevs = se_md->num_sensors;
	int i, ret;

	for (i = 0; i < num_subdevs; i++) {
		sensor = &se_md->sensor[i];
		if (sensor->sd != NULL)
		{
			continue;
		}

		ret = se_md_clean_channel(se_md, i);
		if (ret < 0) {
			pr_err("%s: clean channel fail(%d)\n", __func__, i);
			return ret;
		}
	}

	return 0;
}

static void se_md_unregister_all(struct se_md *se_md)
{
	struct se_cif_dev *se_cif;
	int i;

	for (i = 0; i < SE_MD_CIF_MAX_DEVS; i++) {
		se_cif = se_md->se_cif[i];
		if (!se_cif)
			continue;

		v4l2_device_unregister_subdev(&se_cif->cif_cap.sd);
		media_entity_cleanup(&se_cif->cif_cap.sd.entity);

		dev_info(&se_cif->pdev->dev, "%s unregister cif channel[%d]\n",
					__func__, se_cif->id);
	}
}

static int se_md_link_notify(struct media_link *link, unsigned int flags,
				unsigned int notification)
{
	return 0;
}

static const struct media_device_ops se_md_ops = {
	.link_notify = se_md_link_notify,
};

static const struct v4l2_async_notifier_operations subdev_notify_ops = {
	.bound = subdev_notifier_bound,
	.complete = subdev_notifier_complete,
};

static int se_md_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_device *v4l2_dev;
	struct se_md *se_md;
	int ret;

	se_md = devm_kzalloc(dev, sizeof(*se_md), GFP_KERNEL);
	if (!se_md)
		return -ENOMEM;

	se_md->pdev = pdev;
	platform_set_drvdata(pdev, se_md);

	/* register media device  */
	strlcpy(se_md->media_dev.model, "Siengine Capture Media Deivce",
		sizeof(se_md->media_dev.model));
	se_md->media_dev.ops = &se_md_ops;
	se_md->media_dev.dev = dev;

	/* register v4l2 device */
	v4l2_dev = &se_md->v4l2_dev;
	v4l2_dev->mdev = &se_md->media_dev;
	v4l2_dev->notify = se_sensor_notify;
	strlcpy(v4l2_dev->name, "se-img-md", sizeof(v4l2_dev->name));

	media_device_init(&se_md->media_dev);

	ret = v4l2_device_register(dev, &se_md->v4l2_dev);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register v4l2_device: %d\n", ret);
		goto err_md;
	}

	v4l2_async_notifier_init(&se_md->subdev_notifier);

	ret = register_sensor_entities(se_md);
	if (ret < 0)
	{
		v4l2_err(v4l2_dev, "%s failed.\n", __func__);
		goto err_v4l2_dev;
	}

	ret = se_md_register_platform_entities(se_md, dev->of_node);
	if (ret < 0)
		goto err_v4l2_dev;

	if (list_empty(&se_md->subdev_notifier.asd_list)) {
		v4l2_err(v4l2_dev, "no subdev found in graph\n");
		goto err_v4l2_dev;
	}

	dev_info(&se_md->pdev->dev, " deserializer num = %d.\n", se_md->num_sensors);

	if (se_md->num_sensors > 0) {
		/*
		se_md->subdev_notifier.subdevs = se_md->async_subdevs;
		se_md->subdev_notifier.num_subdevs = se_md->num_sensors;
		se_md->subdev_notifier.bound = subdev_notifier_bound;
		se_md->subdev_notifier.complete = subdev_notifier_complete;
		*/
		se_md->subdev_notifier.ops = &subdev_notify_ops;
		se_md->num_sensors = 0;
		se_md->link_status = 0;

		ret = v4l2_async_notifier_register(&se_md->v4l2_dev,
						&se_md->subdev_notifier);
		if (ret < 0) {
			dev_warn(&se_md->pdev->dev, "Sensor register failed\n");
			goto err_m_ent;
		}

		if (!se_md->link_status) {
			if (se_md->num_sensors > 0) {
				ret = subdev_notifier_complete(&se_md->subdev_notifier);
				if (ret < 0)
				{
					dev_err(&se_md->pdev->dev,
						"subdev_notifier_complete failed, ret = %d", ret);
					goto err_m_ent;
				}
				dev_info(&se_md->pdev->dev,
					"subdev_notifier_complete success\n");
			} else {
				/* no sensors connected */
				se_md_unregister_all(se_md);
			}
		}
	}

	return 0;

err_m_ent:
	se_md_unregister_entities(se_md);
err_v4l2_dev:
	v4l2_device_unregister(&se_md->v4l2_dev);
err_md:
	media_device_cleanup(&se_md->media_dev);
	return ret;
}

static int se_md_remove(struct platform_device *pdev)
{
	struct se_md *se_md = platform_get_drvdata(pdev);

	if (!se_md)
		return 0;

	v4l2_async_notifier_unregister(&se_md->subdev_notifier);

	v4l2_device_unregister(&se_md->v4l2_dev);
	se_md_unregister_entities(se_md);
	media_device_unregister(&se_md->media_dev);
	media_device_cleanup(&se_md->media_dev);

	return 0;
}

static const struct of_device_id se_md_of_match[] = {
	{	.compatible = "siengine,se-md",},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, se_md_of_match);


static struct platform_driver se_md_driver = {
	.driver = {
		.name = SE_MD_DRIVER_NAME,
		.of_match_table	= se_md_of_match,
	},
	.probe = se_md_probe,
	.remove = se_md_remove,
};

module_platform_driver(se_md_driver);

MODULE_AUTHOR("Siengine Technology, Inc.");
MODULE_DESCRIPTION("Siengine Media Device driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" SE_MD_DRIVER_NAME);
